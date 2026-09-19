"""Shrinks the control page before it is gzipped into the firmware.

Comments and indentation are about half of web_page.html, and gzip keeps much of
what it is given, so they cost flash on every keyboard. This removes them and
nothing else: line breaks stay (JavaScript ends statements on them), names stay
(the page calls its own functions across windows by name), and the inside of
every string, template literal, regex literal, <textarea> and <pre> is copied
untouched.

It is a small tokenizer, not search-and-replace, so a // inside a URL string or a
/* inside a regex is never taken for a comment. Whatever it cannot account for
raises PageMinifyError, and the caller stores the page as written instead.
"""

import re


class PageMinifyError(Exception):
    pass


# After one of these a '/' starts a regex; after anything else that ends a value
# it divides.
_REGEX_AFTER_CHARS = set("(,=:[!&|?{};+-*%<>~^")
_REGEX_AFTER_WORDS = {
    "return", "typeof", "instanceof", "in", "of", "new", "delete", "void",
    "throw", "case", "do", "else", "yield", "await",
}


def _is_word(ch):
    return ch.isalnum() or ch in "_$" or ord(ch) > 127


def _minify_js(src):
    out = []
    i, n = 0, len(src)
    mode = "code"
    tpl = []      # brace depth at each open ${ … }
    depth = 0     # { } depth in code, template expressions included
    parens = 0
    prev = ""     # what the last code token was: a punctuator, "w" for a word, "v" for a literal
    word = ""
    gap_nl = gap_sp = False

    def put(text):
        nonlocal gap_nl, gap_sp
        # Whitespace between two tokens is kept as one character — a newline if
        # there was one, else a space — so no two tokens ever run together and
        # automatic semicolon insertion sees the same line breaks as before.
        if out:
            if gap_nl:
                out.append("\n")
            elif gap_sp:
                out.append(" ")
        gap_nl = gap_sp = False
        out.append(text)

    while i < n:
        c = src[i]
        if mode == "tpl":
            if c == "\\":
                out.append(src[i:i + 2])
                i += 2
            elif c == "`":
                out.append(c)
                i += 1
                mode, prev, word = "code", "v", ""
            elif src.startswith("${", i):
                out.append("${")
                i += 2
                tpl.append(depth)
                depth += 1
                mode, prev, word = "code", "{", ""
            else:
                out.append(c)
                i += 1
            continue

        if c in " \t\r":
            gap_sp = True
            i += 1
            continue
        if c == "\n":
            gap_nl = True
            i += 1
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            if j < 0:
                raise PageMinifyError("unterminated /* comment")
            if "\n" in src[i:j]:
                gap_nl = True
            else:
                gap_sp = True
            i = j + 2
            continue
        if c in "'\"":
            j = i + 1
            while True:
                if j >= n or src[j] == "\n":
                    raise PageMinifyError(f"string runs past the end of a line near {src[i:i + 40]!r}")
                if src[j] == "\\":
                    j += 2
                    continue
                if src[j] == c:
                    break
                j += 1
            put(src[i:j + 1])
            i = j + 1
            prev, word = "v", ""
            continue
        if c == "`":
            put("`")
            i += 1
            mode = "tpl"
            continue
        if c == "/":
            if prev == "" or prev in _REGEX_AFTER_CHARS or (prev == "w" and word in _REGEX_AFTER_WORDS):
                j = i + 1
                in_class = False
                while True:
                    if j >= n or src[j] == "\n":
                        raise PageMinifyError(f"regex runs past the end of a line near {src[i:i + 40]!r}")
                    ch = src[j]
                    if ch == "\\":
                        j += 2
                        continue
                    if in_class:
                        if ch == "]":
                            in_class = False
                    elif ch == "[":
                        in_class = True
                    elif ch == "/":
                        break
                    j += 1
                j += 1
                while j < n and src[j].isalpha():
                    j += 1
                put(src[i:j])
                i = j
                prev, word = "v", ""
                continue
            put("/")
            i += 1
            prev, word = "/", ""
            continue
        if _is_word(c):
            j = i + 1
            while j < n and _is_word(src[j]):
                j += 1
            put(src[i:j])
            word = src[i:j]
            prev = "w"
            i = j
            continue
        # Any other punctuator.
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if tpl and depth == tpl[-1]:
                tpl.pop()
                put("}")
                i += 1
                mode = "tpl"
                continue
        elif c == "(":
            parens += 1
        elif c == ")":
            parens -= 1
        put(c)
        i += 1
        prev, word = c, ""

    # A tokenizer that lost its place almost never ends back at the top level
    # with every bracket closed, so this is the check that the output is sound.
    if mode != "code" or tpl or depth != 0 or parens != 0:
        raise PageMinifyError(f"script ends unbalanced (mode {mode}, braces {depth}, parens {parens})")
    return "".join(out)


def _minify_css(src):
    out = []
    i, n = 0, len(src)
    gap_nl = gap_sp = False
    while i < n:
        c = src[i]
        if c in " \t\r":
            gap_sp = True
            i += 1
            continue
        if c == "\n":
            gap_nl = True
            i += 1
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            if j < 0:
                raise PageMinifyError("unterminated /* comment in the stylesheet")
            # Nothing in its place: in CSS a comment is not whitespace, and
            # `.a/**/.b` means `.a.b`.
            i = j + 2
            continue
        if c in "'\"":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            if j >= n:
                raise PageMinifyError("unterminated string in the stylesheet")
            token, i = src[i:j + 1], j + 1
        else:
            token, i = c, i + 1
        if out:
            if gap_nl:
                out.append("\n")
            elif gap_sp:
                out.append(" ")
        gap_nl = gap_sp = False
        out.append(token)
    return "".join(out)


_TAG_NAME = re.compile(r"<([A-Za-z][A-Za-z0-9]*)")
_TEXT_WS = re.compile(r"[ \t]*\n[ \t\n]*")


def minify_page(html):
    lower = html.lower()
    out = []
    i, n = 0, len(html)
    while i < n:
        if html.startswith("<!--", i):
            j = html.find("-->", i + 4)
            if j < 0:
                raise PageMinifyError("unterminated <!-- comment")
            i = j + 3
            continue
        if html[i] == "<" and i + 1 < n and (html[i + 1].isalpha() or html[i + 1] in "/!"):
            # A tag, copied as written; a '>' inside a quoted attribute does not end it.
            j, quote = i + 1, None
            while j < n:
                ch = html[j]
                if quote:
                    if ch == quote:
                        quote = None
                elif ch in "'\"":
                    quote = ch
                elif ch == ">":
                    break
                j += 1
            if j >= n:
                raise PageMinifyError("unterminated tag")
            tag = html[i:j + 1]
            out.append(tag)
            i = j + 1
            m = _TAG_NAME.match(tag)
            name = m.group(1).lower() if m else ""
            if name in ("script", "style", "textarea", "pre"):
                end = lower.find("</" + name, i)
                if end < 0:
                    raise PageMinifyError(f"<{name}> is never closed")
                body = html[i:end]
                if name == "script" and "src=" not in tag.lower():
                    body = _minify_js(body)
                elif name == "style":
                    body = _minify_css(body)
                out.append(body)
                i = end
            continue
        # Text between tags. A run of whitespace holding a line break renders as
        # one space either way, so the break is kept and the indentation dropped.
        j = html.find("<", i + 1) if html[i] == "<" else html.find("<", i)
        j = n if j < 0 else j
        text = _TEXT_WS.sub("\n", html[i:j])
        # Two runs either side of a removed comment would leave a blank line.
        if text.startswith("\n") and out and out[-1].endswith("\n"):
            text = text[1:]
        out.append(text)
        i = j
    return "".join(out)
