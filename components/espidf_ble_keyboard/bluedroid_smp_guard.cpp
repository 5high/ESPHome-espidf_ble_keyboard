#include "esphome/core/defines.h"
#ifdef USE_BLE_KB_SMP_KEY_GUARD

// ESP-IDF 5.5.6 wipes a paired host's keys from memory every time it reconnects.
//
// 5.5.6 moved "drop the old bond" from the Pairing Request (smp_proc_pair_cmd) to
// smp_check_auth_req(), which runs once the link is encrypted, so that a pairing that
// never completes no longer costs the old key. But ESP-IDF's own smp_link_encrypted()
// also sends a plain reconnect there: when a peripheral's host starts the encryption
// itself with the key it already holds (its branch "for example iPhones" — Android,
// Samsung's monitors and most other hosts do the same), it puts SMP into
// ENCRYPTION_PENDING and fires SMP_ENCRYPTED_EVT, which the slave state table hands
// to smp_check_auth_req. No pairing took place, yet the keys are cleared. The copy in
// flash survives, but on the disconnect that follows the stack frees the record it
// left key-less ("Deleting unpaired device"), and the next reconnect is answered with
// an LTK negative reply — the host cannot encrypt and gives up until the keyboard
// restarts. Measured on a phone and a Samsung monitor (2026-10-22); the same code is
// still on esp-idf master and release/v5.4 to v6.0.
//
// The wipe is let through here only when a Pairing Request was processed in this SMP
// procedure, which is what Bluedroid's own SMP_PAIR_FLAG_ENC_AFTER_PAIR records. Every
// other path is untouched: these wrappers only watch, and btm_sec_clear_ble_keys keeps
// its other callers (unbonding, eviction). All four run on Bluedroid's BTU task, one
// SMP procedure at a time, so plain statics are enough. Wrapped with -Wl,--wrap from
// __init__.py, for ESP-IDF 5.5.6 and newer only — the way ESPHome's esp32_ble_tracker
// patches l2cble_init_direct_conn. The prototypes take void* because the structs are
// private to Bluedroid and are only passed through.

#include "esphome/core/log.h"

// NOLINTBEGIN(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp,readability-identifier-naming)
extern "C" {

void __real_smp_proc_pair_cmd(void *p_cb, void *p_data);
void __real_smp_check_auth_req(void *p_cb, void *p_data);
void __real_smp_proc_pairing_cmpl(void *p_cb);
void __real_btm_sec_clear_ble_keys(void *p_dev_rec);

static const char *const SMP_GUARD_TAG = "espidf_ble_keyboard.smp";
static bool s_pairing_requested = false;  // a Pairing Request in the current SMP procedure
static bool s_in_check_auth = false;

void __wrap_smp_proc_pair_cmd(void *p_cb, void *p_data) {
  s_pairing_requested = true;
  __real_smp_proc_pair_cmd(p_cb, p_data);
}

void __wrap_smp_check_auth_req(void *p_cb, void *p_data) {
  s_in_check_auth = true;
  __real_smp_check_auth_req(p_cb, p_data);
  s_in_check_auth = false;
}

// Every SMP procedure ends here — paired, failed, timed out or the link lost.
void __wrap_smp_proc_pairing_cmpl(void *p_cb) {
  s_pairing_requested = false;
  __real_smp_proc_pairing_cmpl(p_cb);
}

void __wrap_btm_sec_clear_ble_keys(void *p_dev_rec) {
  if (s_in_check_auth) {
    if (!s_pairing_requested) {
      ESP_LOGD(SMP_GUARD_TAG, "Encrypted with the stored key - keeping it");
      return;
    }
    ESP_LOGD(SMP_GUARD_TAG, "New pairing - replacing the old keys");
  }
  __real_btm_sec_clear_ble_keys(p_dev_rec);
}

}  // extern "C"
// NOLINTEND(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp,readability-identifier-naming)

#endif  // USE_BLE_KB_SMP_KEY_GUARD
