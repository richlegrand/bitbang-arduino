/*
 * CameraStream -- the stock CameraWebServer sketch, reachable from anywhere.
 *
 * The camera setup and the pin map are CameraWebServer's, unchanged. The web
 * server is not: upstream serves video as multipart/x-mixed-replace, which
 * WebKit does not render, so on any iPhone that page shows and the video does
 * not. camera_server.c here sends frames over a WebRTC data channel and draws
 * them on a canvas, which works everywhere, and keeps /capture and /stream as
 * the interface other software expects.
 *
 * Upload over USB and the Serial Monitor prints a URL and a pairing code.
 * Open the URL on a phone: no app, no account, no port forwarding.
 */
#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <BitBang.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
// Either edit the two lines below, or put them in arduino_secrets.h next to
// this sketch, which is not tracked and will not be committed:
//
//     #define SECRET_SSID "your network"
//     #define SECRET_PASS "your password"
#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#endif
#ifndef SECRET_SSID
#define SECRET_SSID "**********"
#define SECRET_PASS "**********"
#endif

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

extern "C" {
#include "camera_server.h"
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // No camera_config_t here: camera_server.c owns it, including the frame
  // buffer count and queue depth the transport work settled on. Two copies of
  // that config would mean editing the one that does not take effect.
  //
  // Select your board at the top of board_config.h, as with CameraWebServer.

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Declare the video channel before BitBang.begin(): channels reach the SDP
  // only in the pre-offer window, and a browser can connect the moment
  // signaling comes up.
  camera_server_declare_channels();
  camera_server_start();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // After the server is listening: the bridge connects to it, and one that has
  // not started yet refuses the connection rather than retrying.
  if (BitBang.begin()) {
    Serial.println("Or from anywhere: " + BitBang.url());
    String code = BitBang.pairingCode();
    if (code.length()) {
      Serial.println("Pairing code: " + code);
    }
  } else {
    // Local only. The page above still works on this network; what failed is
    // the part that makes it reachable off it.
    Serial.println("BitBang did not come up -- see the log above.");
  }
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}
