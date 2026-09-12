/* Logging, at the stock board settings, goes nowhere. Two separate gates:
 *
 * esp32-hal-log.h undefines ESP_LOGE and remaps it onto log_e, which is gated
 * on CORE_DEBUG_LEVEL -- 0 unless the sketch raises Tools > Core Debug Level.
 * USE_ESP_IDF_LOG keeps ESP_LOGE meaning IDF's macro, which the rest of the
 * BitBang stack already uses, so this library's output matches its.
 *
 * That macro is then gated on LOG_LOCAL_LEVEL, which the core also derives
 * from CORE_DEBUG_LEVEL. Errors ship at any setting; above that, follow the
 * menu. Otherwise a failed begin() returns false with the reason nowhere.
 *
 * Both must precede every include: the header only defines them if unset. */
#define USE_ESP_IDF_LOG
#if !defined(CORE_DEBUG_LEVEL) || CORE_DEBUG_LEVEL < 1
#define LOG_LOCAL_LEVEL 1 /* ESP_LOG_ERROR, not yet in scope by name */
#endif

#include "BitBang.h"

extern "C" {
#include "esp_log.h"
#include "bitbang_identity.h"
#include "bitbang_signaling.h"
#include "bitbang_protocol.h"
#include "bitbang_httpd.h"
#include "app_webrtc.h"
#include "app_webrtc_if.h"
#include "kvs_peer_connection.h"
}

static const char *TAG = "BitBang";

/* Set once signaling registers, which is when the access code and pairing code
 * become meaningful. url() returns empty before that rather than a URL that
 * does not work yet. */
static volatile bool s_registered = false;

static void on_state(bitbang_protocol_state_t state, void *user)
{
  (void) user;
  switch (state) {
  case BITBANG_PROTO_CONNECTING:
    ESP_LOGI(TAG, "signaling: connecting");
    break;
  case BITBANG_PROTO_CONNECTED:
    s_registered = true;
    ESP_LOGI(TAG, "signaling: registered");
    break;
  case BITBANG_PROTO_DISCONNECTED:
    s_registered = false;
    ESP_LOGW(TAG, "signaling: disconnected");
    break;
  default:
    break;
  }
}

bool BitBangClass::begin(uint16_t port, uint32_t timeout_ms)
{
#if LOG_LOCAL_LEVEL > 1
  /* Third gate. initArduino() pins the runtime level to CONFIG_LOG_DEFAULT_LEVEL
   * -- ESP_LOG_ERROR in the shipped libraries -- and does not consult Tools >
   * Core Debug Level, so raising it is the only way anything compiled in above
   * error actually prints. Guarded, so a stock build does not have a library
   * quietly changing a global setting. */
  esp_log_level_set("*", (esp_log_level_t) LOG_LOCAL_LEVEL);
#endif

  if (!_started) {
    if (bitbang_identity_init() != ESP_OK) {
      ESP_LOGE(TAG, "identity init failed");
      return false;
    }

    /* Static: the signaling layer keeps the pointer for the life of the
     * connection, so a stack copy here would dangle the moment begin() returns. */
    static bitbang_signaling_config_t cfg = {};
    cfg.server             = CONFIG_BITBANG_SERVER;
    cfg.uid                = bitbang_identity_uid();
    cfg.public_key_b64     = bitbang_identity_public_b64();
    cfg.protocol_version   = CONFIG_BITBANG_PROTOCOL_VERSION;
    cfg.want_code          = true;
    cfg.connect_timeout_ms = 10000;

    static app_webrtc_config_t wcfg = {};
    wcfg.signaling_client_if = bitbang_signaling_client_if_get();
    wcfg.signaling_cfg       = &cfg;
    wcfg.peer_connection_if  = kvs_peer_connection_if_get();

    if (app_webrtc_init(&wcfg) != WEBRTC_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "webrtc init failed");
      return false;
    }

    /* After init, never before: app_webrtc_init runs bitbang_protocol_init, and
     * an observer set earlier is discarded by it. */
    bitbang_protocol_set_observer(on_state, nullptr, nullptr);

    /* LOOPBACK, not DIRECT. Arduino's WebServer is not esp_http_server -- it is
     * its own class over WiFiServer -- so there is no httpd_handle_t to hand
     * over. Dialling localhost works against whatever the sketch is running, and
     * gives it a real socket descriptor, so code calling send() or recv()
     * directly still behaves. */
    bitbang_httpd_config_t hcfg = {};
    hcfg.mode = BITBANG_HTTPD_LOOPBACK;
    hcfg.host = "127.0.0.1";
    hcfg.port = port;
    if (bitbang_httpd_bridge_start(&hcfg) != ESP_OK) {
      ESP_LOGE(TAG, "bridge failed to start; is the sketch's server listening on %u?", port);
      return false;
    }

    if (app_webrtc_run() != WEBRTC_STATUS_SUCCESS) {
      ESP_LOGE(TAG, "webrtc run failed");
      return false;
    }
    _started = true;
  }

  /* Wait for the signaling server to register us before returning.
   *
   * app_webrtc_run() only starts the transport; registration is a network
   * round trip that completes later, and url() is empty until it does. An
   * earlier version returned here, so a sketch printing url() on the next
   * line printed an empty string every time -- which is exactly what a sketch
   * will do, because setup() is synchronous by idiom.
   *
   * Timing out returns false rather than blocking setup() forever: a device
   * with no route to the server should say so and carry on serving locally. */
  const uint32_t deadline = millis() + timeout_ms;
  while (!s_registered && (int32_t)(deadline - millis()) > 0) {
    delay(50);
  }
  if (!s_registered) {
    ESP_LOGE(TAG, "signaling did not register within %u ms", (unsigned) timeout_ms);
    return false;
  }

  return true;
}

String BitBangClass::url()
{
  if (!s_registered) {
    return String();
  }
  const char *uid  = bitbang_identity_uid();
  const char *code = bitbang_identity_access_code();
  if (!uid || !code) {
    return String();
  }
  return String("https://") + CONFIG_BITBANG_SERVER + "/" + uid + "#" + code;
}

String BitBangClass::pairingCode()
{
  const char *code = bitbang_protocol_pairing_code();
  return (code && code[0]) ? String(code) : String();
}

String BitBangClass::uid()
{
  const char *u = bitbang_identity_uid();
  return u ? String(u) : String();
}

bool BitBangClass::connected()
{
  return _started && s_registered;
}

BitBangClass BitBang;
