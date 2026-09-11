#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

/* Bring up the camera and start an ordinary esp_http_server on 127.0.0.1:80.
 * Nothing here knows about BitBang. */
esp_err_t camera_server_start(void);

/* The running instance, so the bridge can inject requests straight into it
 * rather than going through a loopback socket. Valid after start. */
extern httpd_handle_t camera_server_handle;

/* Declare the data channels. Must run before app_webrtc_init: a channel can
 * only reach the SDP in the pre-offer window, and a browser may connect as
 * soon as signaling is up. */
esp_err_t camera_server_declare_channels(void);
