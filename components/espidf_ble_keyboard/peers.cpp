// Linked keyboards: the peer: verb, and the cache of each peer's /state that
// lets this keyboard's page show another keyboard's hosts. Compiled only when
// the config lists peers:, which is also what pulls esp_http_client into the
// build — a keyboard without peers carries none of this.
//
// Everything that talks to a peer runs on the action task. Presses go out in
// the order they were queued, so a macro's delay: still means what it says, and
// the cache is refreshed only on that task's quiet ticks and only while a page
// is asking for it. The web task never waits on the network: /peers copies the
// cache under peer_mutex_ and returns.
#include "espidf_ble_keyboard.h"

#ifdef USE_BLE_KB_PEERS

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esp_http_client.h"

namespace esphome {
namespace espidf_ble_keyboard {

static const char *const TAG = "espidf_ble_keyboard.peer";

// After a press fails, further presses to that peer are dropped for this long
// rather than each waiting out the timeout — five presses at a dead keyboard
// would otherwise hold the action task for over seven seconds.
static const uint32_t PEER_DOWN_MS = 5000;

namespace {

struct PeerReply {
  std::string *out;
  size_t cap;
  bool too_big;
};

// Collects the reply body. The 401 that opens a digest login has a body too, and
// it arrives through here before the real answer, so only a 200's body counts.
esp_err_t peer_http_event(esp_http_client_event_t *evt) {
  if (evt->event_id != HTTP_EVENT_ON_DATA || evt->user_data == nullptr)
    return ESP_OK;
  auto *r = static_cast<PeerReply *>(evt->user_data);
  if (r->out == nullptr || esp_http_client_get_status_code(evt->client) != 200)
    return ESP_OK;
  if (r->too_big || r->out->size() + (size_t) evt->data_len > r->cap) {
    r->too_big = true;
    return ESP_OK;
  }
  r->out->append(static_cast<const char *>(evt->data), (size_t) evt->data_len);
  return ESP_OK;
}

// application/x-www-form-urlencoded, the way the page's own apiPost sends it:
// the far keyboard reads its parameters from the body.
void form_encode_append(std::string &out, const std::string &s) {
  static const char HEX[] = "0123456789ABCDEF";
  for (unsigned char c : s) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' ||
        c == '.' || c == '*') {
      out += (char) c;
    } else if (c == ' ') {
      out += '+';
    } else {
      out += '%';
      out += HEX[c >> 4];
      out += HEX[c & 15];
    }
  }
}

}  // namespace

void EspidfBleKeyboard::add_peer(const std::string &name, const std::string &url, const std::string &user,
                                 const std::string &pass) {
  if (peer_mutex_ == nullptr)
    peer_mutex_ = xSemaphoreCreateMutex();
  Peer p;
  p.name = name;
  p.url = url;
  p.user = user;
  p.pass = pass;
  peers_.push_back(std::move(p));
}

// Never 0, which is what "nobody has asked yet" reads as.
void EspidfBleKeyboard::note_peer_interest() { peer_interest_ms_.store(millis() | 1); }

// One request, start to finish, with nothing left open afterwards: a client
// held between requests would keep a socket and its buffers for the whole time
// a page is open, on a device whose heap troughs near 23 KB.
bool EspidfBleKeyboard::peer_request_(Peer &p, bool post, const char *path, const std::string &body,
                                      std::string *out) {
  const std::string url = p.url + path;
  PeerReply reply{out, PEER_MAX_REPLY, false};
  if (out != nullptr)
    out->clear();

  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.method = post ? HTTP_METHOD_POST : HTTP_METHOD_GET;
  cfg.timeout_ms = PEER_TIMEOUT_MS;
  cfg.event_handler = peer_http_event;
  cfg.user_data = &reply;
  cfg.disable_auto_redirect = true;
  cfg.keep_alive_enable = false;
  cfg.user_agent = "espidf_ble_keyboard";
  // A digest Authorization header alone is ~300 bytes; the default 512 leaves
  // the request line and the other headers very little room.
  cfg.buffer_size_tx = 1024;
  if (!p.user.empty()) {
    // The client answers the far side's 401 with whichever scheme it asked for.
    cfg.username = p.user.c_str();
    cfg.password = p.pass.c_str();
    cfg.max_authorization_retries = 1;
  }

  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (client == nullptr) {
    ESP_LOGW(TAG, "Peer %s: no memory for a request", p.name.c_str());
    return false;
  }
  if (post) {
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
    esp_http_client_set_post_field(client, body.data(), (int) body.size());
  }
  const esp_err_t err = esp_http_client_perform(client);
  const int status = err == ESP_OK ? esp_http_client_get_status_code(client) : 0;
  esp_http_client_cleanup(client);

  if (err == ESP_OK && status == 200 && !reply.too_big)
    return true;
  if (status == 401) {
    ESP_LOGW(TAG, "Peer %s refused the login — check username and password under peers:", p.name.c_str());
  } else if (reply.too_big) {
    ESP_LOGW(TAG, "Peer %s: reply to %s over %u bytes, ignored", p.name.c_str(), path, (unsigned) PEER_MAX_REPLY);
  } else {
    ESP_LOGW(TAG, "Peer %s: %s %s failed (%s, HTTP %d)", p.name.c_str(), post ? "POST" : "GET", path,
             esp_err_to_name(err), status);
  }
  return false;
}

// peer:<name>:<action> — run <action> on that keyboard, exactly as its own page
// would: a press, hold:<a>, release, switch_host:N, a macro, anything.
void EspidfBleKeyboard::run_peer_action_(const std::string &action) {
  const size_t sep = action.find(':', 5);
  if (sep == std::string::npos || sep == 5 || sep + 1 >= action.size()) {
    ESP_LOGW(TAG, "peer: needs a name and an action, e.g. peer:bedroom:volume_up — got %s", action.c_str());
    return;
  }
  Peer *p = nullptr;
  for (auto &cand : peers_) {
    if (action.compare(5, sep - 5, cand.name) == 0 && cand.name.size() == sep - 5) {
      p = &cand;
      break;
    }
  }
  if (p == nullptr) {
    ESP_LOGW(TAG, "No peer called '%s' — peers: in this keyboard's YAML names them",
             action.substr(5, sep - 5).c_str());
    return;
  }
  // A YAML run_action runs on the loop, and the loop must never wait on the
  // network. Handed over, this step runs after whatever the loop does next
  // instead of in line with it — the price of not stalling everything else.
  if (action_task_ != nullptr && xTaskGetCurrentTaskHandle() != action_task_) {
    queue_action(action);
    return;
  }
  if (p->down_until_ms != 0 && (int32_t) (millis() - p->down_until_ms) < 0) {
    ESP_LOGW(TAG, "Peer %s is not answering; dropped %s", p->name.c_str(), action.c_str() + sep + 1);
    return;
  }
  std::string body = "action=";
  body.reserve(7 + (action.size() - sep) * 3);
  form_encode_append(body, action.substr(sep + 1));
  const bool ok = peer_request_(*p, true, "/api/ble_keyboard/press", body, nullptr);
  p->down_until_ms = ok ? 0 : (millis() + PEER_DOWN_MS) | 1;
  // A press can switch its host or re-skin its remote, so read it again soon
  // instead of on the usual timer — only happens while a page is looking.
  if (ok)
    p->next_due_ms = (millis() + 300) | 1;
  xSemaphoreTake(peer_mutex_, portMAX_DELAY);
  p->ok = ok;
  xSemaphoreGive(peer_mutex_);
}

// On the action task's quiet ticks. One peer per tick, so a press queued behind
// a refresh waits on one request at most.
void EspidfBleKeyboard::refresh_peers_() {
  const uint32_t now = millis();
  const uint32_t interest = peer_interest_ms_.load();
  if (interest == 0 || now - interest > PEER_INTEREST_MS) {
    // Nobody is looking: no traffic, and give the memory back.
    if (peer_cache_live_) {
      xSemaphoreTake(peer_mutex_, portMAX_DELAY);
      for (auto &p : peers_) {
        std::string().swap(p.state);
        p.fetched_ms = 0;
        p.next_due_ms = 0;
      }
      xSemaphoreGive(peer_mutex_);
      peer_cache_live_ = false;
    }
    return;
  }
  for (auto &p : peers_) {
    if (p.next_due_ms != 0 && (int32_t) (now - p.next_due_ms) < 0)
      continue;
    std::string body;
    bool ok = peer_request_(p, false, "/api/ble_keyboard/state", std::string(), &body);
    // Embedded as-is in /peers, so anything that is not one JSON object is
    // treated as no answer rather than passed on to break the page.
    ok = ok && body.size() >= 2 && body.front() == '{' && body.back() == '}';
    const uint32_t done = millis();
    xSemaphoreTake(peer_mutex_, portMAX_DELAY);
    if (ok) {
      p.state.swap(body);
      p.fetched_ms = done | 1;
    }
    p.ok = ok;
    xSemaphoreGive(peer_mutex_);
    p.next_due_ms = (done + (ok ? PEER_POLL_MS : PEER_RETRY_MS)) | 1;
    if (ok)
      p.down_until_ms = 0;
    peer_cache_live_ = true;
    return;
  }
}

size_t EspidfBleKeyboard::peers_json_size() {
  size_t n = 16;
  xSemaphoreTake(peer_mutex_, portMAX_DELAY);
  for (const auto &p : peers_)
    n += p.name.size() + p.state.size() + 64;
  xSemaphoreGive(peer_mutex_);
  return n;
}

void EspidfBleKeyboard::append_peers_json(std::string &out) {
  const uint32_t now = millis();
  out += "{\"peers\":[";
  xSemaphoreTake(peer_mutex_, portMAX_DELAY);
  for (size_t i = 0; i < peers_.size(); i++) {
    const Peer &p = peers_[i];
    if (i > 0)
      out += ',';
    out += "{\"name\":\"";
    out += p.name;  // [a-z0-9_], checked by the schema
    out += "\",\"ok\":";
    out += p.ok ? "true" : "false";
    // Seconds since its state was read; -1 before the first answer.
    out += ",\"age\":";
    out += p.fetched_ms != 0 ? std::to_string((now - p.fetched_ms) / 1000) : std::string("-1");
    out += ",\"state\":";
    if (p.state.empty())
      out += "null";
    else
      out += p.state;
    out += '}';
  }
  xSemaphoreGive(peer_mutex_);
  out += "]}";
}

}  // namespace espidf_ble_keyboard
}  // namespace esphome

#endif  // USE_BLE_KB_PEERS
