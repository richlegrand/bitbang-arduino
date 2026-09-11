/*
 * BitBang for Arduino: turn the web server your sketch already runs into a URL
 * anyone can open.
 *
 * The sketch keeps its own server. BitBang dials it on localhost and bridges
 * requests in over a peer-to-peer connection, so nothing about how the sketch
 * serves pages has to change:
 *
 *     #include <BitBang.h>
 *     WebServer server(80);
 *
 *     void setup() {
 *       Serial.begin(115200);
 *       WiFi.begin(ssid, password);
 *       while (WiFi.status() != WL_CONNECTED) delay(500);
 *
 *       server.on("/", handleRoot);
 *       server.begin();
 *
 *       BitBang.begin();                        // after the server is listening
 *       Serial.println(BitBang.url());
 *       Serial.println(BitBang.pairingCode());
 *     }
 *
 *     void loop() { server.handleClient(); }
 *
 * Everything behind this header is already compiled into the core's
 * precompiled libraries -- the WebRTC stack, SCTP, DTLS-SRTP. This is a
 * wrapper, not an implementation.
 */
#ifndef BITBANG_H
#define BITBANG_H

#include <Arduino.h>

class BitBangClass {
public:
  /* Start BitBang against a server already listening on localhost.
   *
   * Call after WiFi is up and after the sketch's server has begun: the bridge
   * connects to it, and a server that is not yet listening produces a
   * connection refused rather than a retry.
   *
   * port is where the sketch's server listens. Returns false if identity,
   * signaling or the transport failed to start; the reason goes to the log. */
  bool begin(uint16_t port = 80);

  /* The link to hand someone. Empty until begin() has reached the signaling
   * server, which needs a network round trip, so print it after begin()
   * returns true rather than before. */
  String url();

  /* The code that authorizes a first viewer, shown once at setup. Empty when
   * the device is already paired and no new code was requested. */
  String pairingCode();

  /* This device's identity, derived from its key. Stable across reboots and
   * reflashes; it is what the URL addresses. */
  String uid();

  /* Whether the transport is up and a viewer could connect right now. */
  bool connected();

private:
  bool _started = false;
};

extern BitBangClass BitBang;

#endif /* BITBANG_H */
