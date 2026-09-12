/*
 * A camera web server, written against the stock esp_http_server API and
 * nothing else. It does not know it is being reached over a data channel.
 *
 * Written rather than lifted: the closest ready-made server is esp-who's
 * app_httpd.cpp, which is C++ and pulls in their whole module tree
 * (detection pipelines, board peripherals). This is the same handful of
 * httpd calls that file uses -- register_uri_handler, resp_set_type,
 * resp_set_hdr, resp_send, resp_send_chunk -- without the dependency.
 *
 * /         the page: a canvas fed from the video data channel, plus the
 *            sensor controls the ESP32-CAM page has always had
 * /capture   one frame
 * /stream    multipart/x-mixed-replace, the handler that never returns
 * /control   ?var=<name>&val=<n> -- set one sensor field
 * /status    every sensor field, as JSON
 */

/* This file never includes Arduino.h, so nothing derives LOG_LOCAL_LEVEL from
 * the Tools > Core Debug Level menu for it, and it falls back to the level the
 * libraries were built at -- error. The menu only defines CORE_DEBUG_LEVEL;
 * turning that into something esp_log.h reads is left to each file. Without
 * this, raising the menu in the IDE silences this file anyway, which reads as
 * the setting not working. Must precede esp_log.h. */
#if defined(CORE_DEBUG_LEVEL) && !defined(LOG_LOCAL_LEVEL)
#define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL
#endif

#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_camera.h"
#include "bitbang_verify.h"
#include "app_webrtc.h"
#include "lwip/sockets.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "camera_server.h"
#include "board_config.h"

static const char *TAG = "camera_server";

/* Frames go out on their own data channel, not through httpd.
 *
 * That removes the whole HTTP pipeline from the video path -- chunked
 * encoding, multipart framing, the 8 KB coalescing buffer, the SWSP header and
 * two copies of every byte -- and leaves one frame per SCTP message, so the
 * message boundary is the frame boundary. It also renders on iOS, where
 * multipart/x-mixed-replace does not. */
/* <presentation>/<component>. The presentation is what a page binds to, so one
 * element bound to "cam" would receive cam/video and cam/audio together; the
 * component says which track it is. A video-only device simply has no
 * cam/audio. */
#define CAMERA_VIDEO_CHANNEL "cam/video"

/* What the channel carries, reaching the browser as RTCDataChannel.protocol.
 * The receiver has to choose a decoder before the first frame arrives, and
 * neither sniffing the payload nor reading the label can tell it -- the label
 * names the stream, not its type. */
#define CAMERA_VIDEO_PROTOCOL "bitbang-stream/mjpeg"

/* Frames go out in chunks rather than one message each.
 *
 * A whole frame is 15 to 20 KB, and the congestion window on this link settles
 * at 33 to 41 KB -- so on a LAN a frame fits and one message per frame works.
 * On cellular the window is a fraction of that, and a single message then needs
 * several round trips before any of it can be acknowledged. That is what
 * stalled the stream: every send hit the 5 s timeout and 125 of 126 frames were
 * displaced, while the connection itself stayed healthy the whole time. The
 * message was too big for the window, not the buffer too small.
 *
 * A chunk that fits inside even a small window clears in one round trip, so the
 * transport keeps making progress instead of blocking on a message it cannot
 * move. It also gives the lifetime policy something it can act on: under load
 * it abandons chunks, and a frame missing any chunk is discarded by the
 * receiver rather than held.
 *
 * 16 byte header, little-endian, matching the rest of the wire format:
 *
 *    0  u32  frame id      monotonic, never reused, so no wraparound compare
 *    4  u16  chunk index
 *    6  u16  chunk count
 *    8  u16  flags         bit 0 = keyframe, the rest reserved
 *   10  u16  reserved      must be zero
 *   12  u32  pts_ms        capture time, milliseconds since boot
 *
 * pts and the keyframe flag are here before anything needs them. Audio has to
 * share a timebase with video to be synchronized, and H.264 cannot be decoded
 * without knowing which frames are independent -- adding either later means a
 * version negotiation across firmware, Python and the browser. See
 * av-streaming-api.md. */
#define VIDEO_CHUNK_HEADER  16
#define VIDEO_FLAG_KEYFRAME 0x0001

/* The message size has to be a multiple of the SCTP fragment point or the
 * tail of every message costs a whole packet for almost nothing.
 *
 * usrsctp fragments at smallest_mtu minus the SCTP common header (12) and the
 * DATA chunk header (16). SCTP_MTU is 1200 in the fork -- it was 1360 when this
 * was written and was lowered for relay safety -- and I-DATA is not negotiated,
 * since usrsctp sets idata_supported = 0 at endpoint creation and nothing here
 * turns it on. So the payload per packet is 1172.
 *
 * At 4096 that is three full fragments and a fourth carrying 100 bytes, and a
 * 17 KB frame went out as 17 packets holding 1024 payload bytes each against a
 * possible 1332. Measured: 250 KB/s at 250 packets per second, where whole
 * frames on the same link ran 1171 bytes per packet at the same packet rate.
 * The packet rate is what stays fixed, so bytes per packet is throughput.
 *
 * At 3996 all three fragments are full and the same frame is 13 packets. The
 * size class is unchanged, so the cellular case keeps the property that made
 * chunking work there in the first place. */
/* Three, and eight was measured and rejected.
 *
 * Every message is one usrsctp_sendv, and usrsctp calls conn_output -- DTLS
 * encryption plus the socket write -- with the association lock held. Eight
 * fragments per message cuts the call count from 9.1 per frame to 3.4, which
 * does raise throughput: 254 to 289 KB/s.
 *
 * It buys that with latency, because the lock contention does not go away, it
 * changes sides. Holding the lock through eight encryptions is about 16 ms per
 * acquisition, so the sender stops blocking the receiver and starts being
 * blocked by it -- receive-side waits fell 456 to 88 ms/s while send-side waits
 * rose 57 to 642, a net 42% more contention. srtt went from 20-45 ms to
 * 114-120 ms, the worst single wait from 64 ms to 143 ms, and messages began
 * being abandoned. Inflated srtt grows cwnd, each SACK then acknowledges more
 * and costs more to process, and inbound per-packet cost nearly doubled.
 *
 * For live video that is the wrong trade. Three stays. See the contention
 * section of esp32-video-transport.md. */
#define SCTP_FRAG_POINT     1172
#define VIDEO_CHUNK_BYTES   (3 * SCTP_FRAG_POINT)   /* 3516 */

static inline void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t) v;
    p[1] = (uint8_t) (v >> 8);
}

static inline void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t) v;
    p[1] = (uint8_t) (v >> 8);
    p[2] = (uint8_t) (v >> 16);
    p[3] = (uint8_t) (v >> 24);
}
#define VIDEO_CHUNK_PAYLOAD (VIDEO_CHUNK_BYTES - VIDEO_CHUNK_HEADER)

/* Every viewer that has a video channel open, not just the most recent one.
 *
 * This was a single handle, and a second viewer silently replaced the first:
 * both browsers connected, both got the page, and only the one that arrived
 * last received frames. Nothing failed, so nothing said so.
 *
 * Sized to the session cap, since a viewer cannot have a channel without a
 * session. The send API looks a session up by peer id and rejects a NULL one,
 * so the handle alone is not enough to send on a channel -- the peer id has to
 * be kept alongside it. */
#define MAX_VIEWERS CONFIG_KVS_MAX_CONCURRENT_STREAMS

typedef struct {
    void *dc;
    char peer[64];
    int64_t last_ok;   /* when this viewer last took a whole frame */
} viewer_t;

/* How long a viewer may deliver nothing before its slot is freed.
 *
 * This was a count of consecutive failed frames, which is the wrong unit: at
 * a 250 ms send timeout, three of them is 750 ms, and a relayed connection
 * whose congestion window is still opening needs longer than that to place
 * its first frame. A phone was dropped 1.39 seconds after its channel opened,
 * having never received anything, and its browser sat waiting for frames that
 * would never come.
 *
 * The clock starts when the channel opens, so a viewer is judged on whether
 * it has ever delivered rather than on how it started. Long enough for a slow
 * start on a relay, short enough that a viewer which has genuinely stopped
 * reading cannot pace the others for long. */
#define VIEWER_STALL_US (5 * 1000 * 1000)

static viewer_t s_video_viewers[MAX_VIEWERS];
static SemaphoreHandle_t s_viewers_lock;

/* Registered by the signaling task while video_task is iterating, so the list
 * is held under a lock. It is short and taken once per frame, not per chunk. */
static void on_channel_open(void *dc, const char *label, const char *peer_id)
{
    if (label == NULL || strcmp(label, CAMERA_VIDEO_CHANNEL) != 0) {
        return;
    }

    xSemaphoreTake(s_viewers_lock, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MAX_VIEWERS; i++) {
        if (s_video_viewers[i].dc == dc) {
            slot = i;   /* re-opened on the same handle */
            break;
        }
        if (s_video_viewers[i].dc == NULL && slot < 0) {
            slot = i;
        }
    }
    if (slot >= 0) {
        s_video_viewers[slot].dc = dc;
        s_video_viewers[slot].last_ok = esp_timer_get_time();
        s_video_viewers[slot].peer[0] = '\0';
        if (peer_id != NULL) {
            strlcpy(s_video_viewers[slot].peer, peer_id, sizeof(s_video_viewers[slot].peer));
        }
    }
    xSemaphoreGive(s_viewers_lock);

    if (slot >= 0) {
        ESP_LOGI(TAG, "video channel open for %s (viewer %d)", peer_id != NULL ? peer_id : "", slot);
    } else {
        ESP_LOGW(TAG, "video channel open for %s but all %d viewer slots are taken",
                 peer_id != NULL ? peer_id : "", MAX_VIEWERS);
    }
}

static bool viewers_present(void)
{
    bool any = false;
    xSemaphoreTake(s_viewers_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_VIEWERS; i++) {
        if (s_video_viewers[i].dc != NULL) {
            any = true;
            break;
        }
    }
    xSemaphoreGive(s_viewers_lock);
    return any;
}

static void viewer_took_frame(void *dc)
{
    xSemaphoreTake(s_viewers_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_VIEWERS; i++) {
        if (s_video_viewers[i].dc == dc) {
            s_video_viewers[i].last_ok = esp_timer_get_time();
            break;
        }
    }
    xSemaphoreGive(s_viewers_lock);
}

static void viewer_missed_frame(void *dc)
{
    char gone[64] = "";
    int64_t stalled = 0;

    xSemaphoreTake(s_viewers_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_VIEWERS; i++) {
        if (s_video_viewers[i].dc == dc) {
            stalled = esp_timer_get_time() - s_video_viewers[i].last_ok;
            if (stalled > VIEWER_STALL_US) {
                strlcpy(gone, s_video_viewers[i].peer, sizeof(gone));
                s_video_viewers[i].dc = NULL;
                s_video_viewers[i].peer[0] = '\0';
            }
            break;
        }
    }
    xSemaphoreGive(s_viewers_lock);

    if (gone[0] != '\0') {
        ESP_LOGW(TAG, "viewer %s delivered nothing for %d ms; dropping it", gone, (int) (stalled / 1000));
        /* Free the session too, not just the viewer slot. Left alone it holds
         * one of the very few slots until ICE consent fails, which measured 30
         * to 60 seconds -- long enough that reconnecting was refused with
         * "Max streaming sessions reached". */
        app_webrtc_close_peer(gone);
    }
}

/* Drop one viewer, by handle so a slot that has already been reused is left
 * alone. Called when the send says the peer no longer resolves to a session. */
static void viewer_drop(void *dc)
{
    xSemaphoreTake(s_viewers_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_VIEWERS; i++) {
        if (s_video_viewers[i].dc == dc) {
            ESP_LOGI(TAG, "video channel went away (peer %s); viewer %d free", s_video_viewers[i].peer, i);
            s_video_viewers[i].dc = NULL;
            s_video_viewers[i].peer[0] = '\0';
            break;
        }
    }
    xSemaphoreGive(s_viewers_lock);
}

httpd_handle_t camera_server_handle;


/* Frames per reported window.
 *
 * 30 was too few: at ~10fps that is a three second sample, short enough to
 * catch Wi-Fi retries and camera jitter rather than average them out, and
 * the scatter was wide enough to read trends into that were not there. A
 * couple of hundred frames is ~20s, which smooths that without hiding a
 * real change. */
#define STATS_WINDOW 200

/* No rate cap.
 *
 * The loop clocks itself: httpd_resp_send_chunk blocks when the transport
 * cannot take more, so the frame rate is whatever the link carries and it
 * follows the link rather than a number chosen on one particular afternoon.
 *
 * That property is what the reference ESP32 camera server gets for free from a
 * 5760-byte socket buffer, and what 518 KB of queue and send buffer in this
 * path destroyed -- the producer could run most of a second ahead of the wire,
 * so it never felt backpressure until the backlog was already stale. With that
 * down to 97 KB the transport self-limits at about 284 KB/s on this link, the
 * producer blocks rather than piling up, and the peer's connectivity checks
 * stop going missing.
 *
 * A fixed cap was tried here and removed. It can only be wrong in one
 * direction or the other: too low wastes a good link, too high crowds a poor
 * one, and it cannot know which it is on. */

#define PART_BOUNDARY "bitbangframe"
static const char STREAM_CONTENT_TYPE[] =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char STREAM_BOUNDARY[] = "\r\n--" PART_BOUNDARY "\r\n";
static const char STREAM_PART[] =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/* A canvas fed from the video data channel, not an <img> on a multipart
 * stream.
 *
 * multipart/x-mixed-replace does not render in WebKit, so every browser on iOS
 * shows the page and no video -- the bytes arrive and are discarded. Frames now
 * come over their own data channel as whole JPEGs, one per message, and are
 * drawn here. Nothing in this path is multipart, so it works everywhere.
 *
 * The bytes reach this page from bitbang's bootstrap over a BroadcastChannel;
 * the page owns the canvas and the drawing. */
static const char INDEX_HTML[] =
    "<!doctype html><meta charset=utf-8><title>BitBang Camera</title>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<style>"
    "*{box-sizing:border-box}"
    "body{margin:0;background:#111;color:#ccc;font:13px system-ui;display:flex;"
    "flex-wrap:wrap;align-items:flex-start}"
    "#side{width:240px;padding:10px;background:#1a1a1a;height:100vh;overflow-y:auto}"
    "#main{flex:1;min-width:320px;padding:10px;text-align:center}"
    "canvas{max-width:100%;background:#000;border:1px solid #333}"
    "h2{font-size:12px;text-transform:uppercase;letter-spacing:.08em;color:#777;"
    "margin:14px 0 6px;border-bottom:1px solid #333;padding-bottom:3px}"
    "label{display:flex;align-items:center;justify-content:space-between;margin:5px 0}"
    "label span{flex:1}"
    "input[type=range]{width:110px}select{width:110px;background:#222;color:#ccc;"
    "border:1px solid #444}"
    "#s{margin-top:.6rem;color:#888}"
    "</style>"
    "<div id=side></div>"
    "<div id=main><canvas id=v width=640 height=480></canvas>"
    "<img id=m style=display:none>"
    "<div id=s>waiting for frames</div></div>"
    "<script>"
    /* var|kind|args -- kind r=range, c=checkbox, s=select. Grouped the way the
       ESP32-CAM page groups them, so the control someone is looking for is
       where they expect it. Register, PLL and windowing controls are left out:
       they are sensor-bringup tools, and a bad register write wedges the
       sensor. */
    "const G=[['Resolution',["
    /* No options here: /status sends them, built from the driver's own enum.
       See FRAMESIZES below for why a list of numbers in this page is wrong. */
    "['framesize','s',''],"
    /* Reversed on purpose. jpeg_quality is 4..63 with *lower* meaning better,
       so a plain slider labelled Quality gets worse as you drag it right --
       true of the stock ESP32-CAM page too. The wire value is unchanged, so
       /control?var=quality&val=N still means what it always did; only the
       slider's direction is flipped. */
    "['quality','R','4,63']]],"
    "['Image',["
    "['brightness','r','-2,2'],['contrast','r','-2,2'],['saturation','r','-2,2'],"
    "['special_effect','s','0:None,1:Negative,2:Grayscale,3:Red,4:Green,5:Blue,6:Sepia'],"
    "['hmirror','c',''],['vflip','c','']]],"
    "['Exposure',["
    "['aec','c',''],['aec2','c',''],['ae_level','r','-2,2'],['aec_value','r','0,1200']]],"
    "['Gain',["
    "['agc','c',''],['agc_gain','r','0,30'],"
    "['gainceiling','s','0:2x,1:4x,2:8x,3:16x,4:32x,5:64x,6:128x']]],"
    "['White balance',["
    "['awb','c',''],['awb_gain','c',''],"
    "['wb_mode','s','0:Auto,1:Sunny,2:Cloudy,3:Office,4:Home']]],"
    "['Correction',["
    "['bpc','c',''],['wpc','c',''],['raw_gma','c',''],['lenc','c',''],"
    "['dcw','c',''],['colorbar','c','']]]];"
    "const side=document.getElementById('side'),el={},lab={};"
    /* Manual controls only exist while their automatic counterpart is off.
       Leaving them visible means dragging a slider that the sensor ignores,
       which reads as a broken camera rather than a disabled control.
       [master, dependent, show-when-master-checked] */
    "const D=[['agc','gainceiling',1],['agc','agc_gain',0],"
    "['aec','aec_value',0],['awb_gain','wb_mode',1]];"
    "function sync(){for(const [m,d,w] of D){"
    "if(!el[m]||!lab[d])continue;"
    "lab[d].style.display=(el[m].checked===!!w)?'flex':'none'}}"
    "function set(v,x){fetch(`/control?var=${v}&val=${x}`).catch(()=>{})}"
    "for(const [title,rows] of G){"
    "const h=document.createElement('h2');h.textContent=title;side.appendChild(h);"
    "for(const [v,kind,arg] of rows){"
    "const l=document.createElement('label'),sp=document.createElement('span');"
    "sp.textContent=v.replace(/_/g,' ');l.appendChild(sp);let i;"
    "if(kind==='s'){i=document.createElement('select');"
    "for(const o of arg.split(',')){const [val,txt]=o.split(':');"
    "const op=document.createElement('option');op.value=val;op.textContent=txt;i.appendChild(op)}"
    "i.onchange=()=>set(v,i.value)}"
    "else if(kind==='c'){i=document.createElement('input');i.type='checkbox';"
    "i.onchange=()=>{set(v,i.checked?1:0);sync()}}"
    "else{const [a,b]=arg.split(',').map(Number);i=document.createElement('input');"
    "i.type='range';i.min=a;i.max=b;"
    "if(kind==='R'){i.dataset.rev=a+b;i.oninput=()=>set(v,a+b-i.value)}"
    "else i.oninput=()=>set(v,i.value)}"
    "el[v]=i;lab[v]=l;l.appendChild(i);side.appendChild(l)}}"
    /* Seed every control from the device rather than from defaults in this
       page: the sensor may already have been configured, and a slider showing
       a value the sensor does not hold is worse than no slider. */
    "fetch('/status').then(r=>r.json()).then(j=>{"
    "if(j.framesizes&&el.framesize){"
    "for(const o of j.framesizes.split(',')){const [val,txt]=o.split(':');"
    "const op=document.createElement('option');op.value=val;op.textContent=txt;"
    "el.framesize.appendChild(op)}}"
    "for(const k in el){if(!(k in j))continue;const i=el[k];"
    "if(i.type==='checkbox')i.checked=!!j[k];"
    "else if(i.dataset.rev)i.value=i.dataset.rev-j[k];"
    "else i.value=j[k]}"
    "sync();"
    "}).catch(()=>{});"
    /* Frames arrive over the video data channel, not an <img> on a multipart
       stream: multipart/x-mixed-replace does not render in WebKit, so every
       browser on iOS would show the page and no video. bootstrap.js forwards
       them here on a BroadcastChannel; this page owns the canvas. */
    "const c=document.getElementById('v'),x=c.getContext('2d');"
    "let s=document.getElementById('s');"
    /* Reached through bitbang the page is inside bootstrap's iframe; reached
       directly on the LAN it is top-level. That is synchronous and needs no
       timeout. __bitbang would be the obvious flag, but bootstrap assigns it
       on iframe load, after this script has already run.
       Without a data channel there is no canvas to feed, so fall back to
       /stream. Multipart does not render in WebKit, which is why the canvas
       exists at all -- but that only matters for the remote case, which is
       the one that has the data channel. */
    "if(window.self===window.top){"
    "const m=document.getElementById('m');"
    "c.style.display='none';m.style.display='';m.src='/stream';"
    "m.onload=()=>{if(s){s.remove();s=null}};"
    "}else{"
    "new BroadcastChannel('bitbang-video').onmessage=async e=>{"
    "if(e.data.type!=='frame')return;"
    "try{"
    "const b=await createImageBitmap(new Blob([e.data.data],{type:'image/jpeg'}));"
    "if(c.width!==b.width||c.height!==b.height){c.width=b.width;c.height=b.height;}"
    "x.drawImage(b,0,0);b.close();"
    /* The placeholder goes away with the first frame; there is no readout
       after that, just the picture. */
    "if(s){s.remove();s=null}"
    "}catch(err){/* a truncated frame is a dropped frame, not an error */}"
    "};"
    "}"
    "</script>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char *) fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

/* Loops until a send fails, which is how it learns the client went away.
 * Occupies its worker for as long as someone is watching -- the same
 * arrangement esp_http_server has over Wi-Fi. */
/* -- The hand-off between capture and network --
 *
 * Two threads, because the driver's frame queue cannot serve both jobs at
 * once. It is the buffer pool that keeps DMA flowing and it is the hand-off to
 * the sender, and those want opposite depths: esp_camera_fb_get is FIFO, so a
 * deep queue hands over a frame that is (depth - 1) frame periods old, while a
 * shallow one leaves the driver with nothing to DMA into while the network
 * holds a buffer.
 *
 * Splitting them settles it. The camera thread drains the driver queue as fast
 * as frames appear, so the queue stays near-empty and its depth stops
 * determining staleness. It publishes into a one-deep slot where a newer frame
 * displaces an older one, so the sender always gets the freshest frame. And it
 * returns buffers without waiting for the network, so the driver keeps
 * circulating.
 *
 * Frames captured while the network is busy are discarded here rather than
 * queued, which is the whole point: a frame that could not be sent when it was
 * current is worth less than the one behind it. */
static SemaphoreHandle_t s_slot_lock;
static camera_fb_t *s_slot;              /* newest frame not yet taken */
static TaskHandle_t s_video_task;        /* woken when a frame is published */
static volatile int s_viewers;
static SemaphoreHandle_t s_wake_camera;

/* Take the pending frame, if there is one. Caller owns it and must return it. */
static camera_fb_t *slot_take(void)
{
    camera_fb_t *fb = NULL;
    xSemaphoreTake(s_slot_lock, portMAX_DELAY);
    fb = s_slot;
    s_slot = NULL;
    xSemaphoreGive(s_slot_lock);
    return fb;
}

static void camera_task(void *arg)
{
    (void) arg;
    for (;;) {
        int64_t t_get = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "camera: fb_get returned nothing");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!viewers_present()) {
            /* Nobody to send to. Return it and wait rather than filling the
             * slot with frames that will only grow stale. */
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t *displaced = NULL;
        xSemaphoreTake(s_slot_lock, portMAX_DELAY);
        displaced = s_slot;
        s_slot = fb;
        xSemaphoreGive(s_slot_lock);

        {
            /* How long fb_get waited, and how often a publish displaced
             * nothing -- which is the case that returns no buffer to the
             * driver and is the one that can starve it. */
            static uint32_t n, dropped;
            static int64_t sum_wait, worst_wait, win;
            int64_t now = esp_timer_get_time();
            sum_wait += now - t_get;
            if (now - t_get > worst_wait) worst_wait = now - t_get;
            n++;
            if (displaced != NULL) dropped++;
            if (win == 0) win = now;
            else if (now - win > 5000000) {
                ESP_LOGW(TAG, "camera: %u captured, %u displaced, fb_get wait avg %.0f ms worst %.0f ms",
                         (unsigned) n, (unsigned) dropped,
                         sum_wait / 1000.0 / n, worst_wait / 1000.0);
                n = dropped = 0; sum_wait = worst_wait = 0; win = now;
            }
        }

        /* Straight back to the driver, so it never waits on us. */
        if (displaced != NULL) {
            esp_camera_fb_return(displaced);
        }
        if (s_video_task != NULL) {
            xTaskNotifyGive(s_video_task);
        }
    }
}

/* The other half of the pair: take the newest frame and send it.
 *
 * Blocks in the send when the transport cannot take more, which is what paces
 * the whole pipeline -- no frame rate is chosen anywhere. While it is blocked
 * the camera thread keeps capturing and displacing, so the frame it picks up
 * next is the freshest one rather than the one that was current when the send
 * began. */
static void video_task(void *arg)
{
    (void) arg;
    int64_t win_t0 = esp_timer_get_time(), win_bytes = 0;
    uint32_t frames = 0, win_frames = 0, failures = 0;
    uint32_t frame_id = 0, win_chunks = 0, win_chunks_dropped = 0;
    uint8_t stage[VIDEO_CHUNK_BYTES];   /* PSRAM: see VIDEO_TASK_STACK */

    for (;;) {
        camera_fb_t *fb = slot_take();
        if (fb == NULL) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            continue;
        }

        /* Snapshot the viewers, so the send loop is not holding the lock
         * across calls that can block for seconds on a slow link. */
        viewer_t sending[MAX_VIEWERS];
        int nsending = 0;
        xSemaphoreTake(s_viewers_lock, portMAX_DELAY);
        for (int i = 0; i < MAX_VIEWERS; i++) {
            if (s_video_viewers[i].dc != NULL) {
                sending[nsending++] = s_video_viewers[i];
            }
        }
        xSemaphoreGive(s_viewers_lock);

        if (nsending > 0) {
            /* Header and payload have to reach the send call as one buffer,
             * so the chunk is staged. It lives on this task's stack, which is
             * in PSRAM: internal RAM is the scarce pool here, and a static
             * buffer would spend 4 KB of it permanently. The frame buffer it
             * copies from is in PSRAM already. */
            uint8_t *chunk = stage;
            size_t count = (fb->len + VIDEO_CHUNK_PAYLOAD - 1) / VIDEO_CHUNK_PAYLOAD;
            /* The driver stamps this when the frame's first DMA buffer lands,
             * so it is capture time rather than send time, and it is monotonic
             * since boot -- which is the timebase audio would have to share. */
            uint32_t pts_ms = (uint32_t) ((uint64_t) fb->timestamp.tv_sec * 1000u +
                                          (uint64_t) fb->timestamp.tv_usec / 1000u);
            bool anyDelivered = false;

            if (count == 0) {
                count = 1;   /* an empty frame still gets one chunk */
            }
            for (size_t i = 0; i < count; i++) {
                size_t off = i * VIDEO_CHUNK_PAYLOAD;
                size_t n = fb->len - off;
                if (n > VIDEO_CHUNK_PAYLOAD) {
                    n = VIDEO_CHUNK_PAYLOAD;
                }
                put_le32(chunk + 0, frame_id);
                put_le16(chunk + 4, (uint16_t) i);
                put_le16(chunk + 6, (uint16_t) count);
                /* Every MJPEG frame is independently decodable. H.264 will set
                 * this only on an IDR. */
                put_le16(chunk + 8, VIDEO_FLAG_KEYFRAME);
                put_le16(chunk + 10, 0);
                put_le32(chunk + 12, pts_ms);
                memcpy(chunk + VIDEO_CHUNK_HEADER, fb->buf + off, n);

                /* One staged chunk, sent to each viewer. Building it once and
                 * sending it repeatedly is the only reason the copy is worth
                 * doing at all. */
                for (int v = 0; v < nsending; v++) {
                    if (sending[v].dc == NULL) {
                        continue;   /* dropped earlier in this frame */
                    }
                    WEBRTC_STATUS st = app_webrtc_send_data_channel_message(
                        sending[v].peer, sending[v].dc, true, chunk, VIDEO_CHUNK_HEADER + n);
                    if (st == WEBRTC_STATUS_SUCCESS) {
                        win_chunks++;
                        if (i + 1 == count) {
                            anyDelivered = true;
                            viewer_took_frame(sending[v].dc);
                        }
                        continue;
                    }

                    /* A failure belongs to one viewer, not to the frame. The
                     * others still get the rest of it. */
                    win_chunks_dropped += count - i;
                    if (st == WEBRTC_STATUS_INVALID_ARG || st == WEBRTC_STATUS_NULL_ARG) {
                        /* The peer no longer resolves to a session -- the
                         * browser navigated away, refreshed, or closed the
                         * tab. Every later frame would fail the same way, so
                         * free the slot and stop sending to it. */
                        viewer_drop(sending[v].dc);
                    } else {
                        if (failures++ == 0) {
                            /* Once, not twenty-five times a second. A send
                             * that fails on one frame fails on all of them,
                             * and the repeat drowns out whatever else the log
                             * was about to say. */
                            ESP_LOGE(TAG, "video send failed: 0x%08x (silencing until it recovers)",
                                     (unsigned) st);
                        }
                        /* Not "the peer is gone" -- the buffer stayed full.
                         * That is indistinguishable from a viewer that has
                         * stopped reading, and holding the slot open makes
                         * every other viewer wait behind it on each frame. */
                        viewer_missed_frame(sending[v].dc);
                    }
                    sending[v].dc = NULL;   /* skip for the rest of this frame */
                }
            }

            if (anyDelivered) {
                win_bytes += fb->len;
                frames++;
                win_frames++;
                failures = 0;
            }
            frame_id++;
        }
        esp_camera_fb_return(fb);

        if (win_frames >= STATS_WINDOW) {
            int64_t now = esp_timer_get_time();
            double secs = (now - win_t0) / 1e6;
            ESP_LOGI(TAG, "%u frames | %.1f fps over %.0fs, %.0f KB/s, %.1f KB/frame, "
                          "%u chunks sent, %u abandoned",
                     (unsigned) frames, win_frames / secs, secs,
                     win_bytes / 1024.0 / secs, win_bytes / 1024.0 / win_frames,
                     (unsigned) win_chunks, (unsigned) win_chunks_dropped);
            win_t0 = now;
            win_frames = 0;
            win_bytes = 0;
            win_chunks = 0;
            win_chunks_dropped = 0;
        }
    }
}

/* /control and /status, the sensor controls the ESP32-CAM page has always had.
 *
 * Ported from arduino-esp32's CameraWebServer rather than esp-who's: that one
 * carried a detection pipeline and a board-peripheral tree, and this one is
 * plain esp_http_server against sensor_t. Its register, PLL and windowing
 * endpoints are deliberately left out -- sensor-bringup tools, and a bad
 * register write wedges the sensor until a power cycle. */
static esp_err_t control_handler(httpd_req_t *req)
{
    char buf[128], variable[32], value[32];
    size_t len = httpd_req_get_url_query_len(req) + 1;
    if (len <= 1 || len > sizeof(buf) ||
        httpd_req_get_url_query_str(req, buf, len) != ESP_OK ||
        httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
        httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        return httpd_resp_send_500(req);
    }
    int res = 0;

    if (!strcmp(variable, "framesize")) {
        /* Only meaningful in JPEG mode, and the frame buffers were sized for
         * the mode the camera was started in. */
        if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t) val);
    }
    else if (!strcmp(variable, "quality"))        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))       res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))     res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))     res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))    res = s->set_gainceiling(s, (gainceiling_t) val);
    else if (!strcmp(variable, "colorbar"))       res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))            res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))            res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))            res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))          res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))       res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))       res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))      res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))           res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))            res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))            res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))            res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))           res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))       res = s->set_ae_level(s, val);
    else {
        ESP_LOGI(TAG, "unknown control: %s", variable);
        res = -1;
    }

    if (res < 0) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

/* The page cannot hardcode these numbers.
 *
 * framesize_t is an enum whose values shift when Espressif inserts a size:
 * esp32-camera 2.1.7 added 128X128 and 320X320 near the front, so everything
 * from QCIF up moved by two and VGA went from 8 to 10. A page carrying the old
 * numbering labels a VGA stream XGA, and sending what it calls XGA selects
 * something else again. Naming the constants keeps the two in step whichever
 * driver is linked -- which differs between the IDF example and the Arduino
 * package today. */
static const struct { framesize_t value; const char *name; } FRAMESIZES[] = {
    { FRAMESIZE_QVGA, "QVGA" }, { FRAMESIZE_CIF,  "CIF"  }, { FRAMESIZE_HVGA, "HVGA" },
    { FRAMESIZE_VGA,  "VGA"  }, { FRAMESIZE_SVGA, "SVGA" }, { FRAMESIZE_XGA,  "XGA"  },
    { FRAMESIZE_HD,   "HD"   }, { FRAMESIZE_SXGA, "SXGA" }, { FRAMESIZE_UXGA, "UXGA" },
};

static esp_err_t status_handler(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        return httpd_resp_send_500(req);
    }

    char sizes[160];
    int sn = 0;
    for (size_t i = 0; i < sizeof(FRAMESIZES) / sizeof(FRAMESIZES[0]); i++) {
        int w = snprintf(sizes + sn, sizeof(sizes) - sn, "%s%u:%s",
                         sn ? "," : "", (unsigned) FRAMESIZES[i].value, FRAMESIZES[i].name);
        if (w < 0 || (size_t) w >= sizeof(sizes) - sn) {
            break;
        }
        sn += w;
    }

    char json[640];
    int n = snprintf(json, sizeof(json),
        "{\"framesize\":%u,\"quality\":%u,\"brightness\":%d,\"contrast\":%d,"
        "\"saturation\":%d,\"special_effect\":%u,\"wb_mode\":%u,\"awb\":%u,"
        "\"awb_gain\":%u,\"aec\":%u,\"aec2\":%u,\"ae_level\":%d,\"aec_value\":%u,"
        "\"agc\":%u,\"agc_gain\":%u,\"gainceiling\":%u,\"bpc\":%u,\"wpc\":%u,"
        "\"raw_gma\":%u,\"lenc\":%u,\"hmirror\":%u,\"vflip\":%u,\"dcw\":%u,"
        "\"colorbar\":%u,\"pixformat\":%u,\"xclk\":%u,\"framesizes\":\"%s\"}",
        s->status.framesize, s->status.quality, s->status.brightness,
        s->status.contrast, s->status.saturation, s->status.special_effect,
        s->status.wb_mode, s->status.awb, s->status.awb_gain, s->status.aec,
        s->status.aec2, s->status.ae_level, s->status.aec_value, s->status.agc,
        s->status.agc_gain, s->status.gainceiling, s->status.bpc, s->status.wpc,
        s->status.raw_gma, s->status.lenc, s->status.hmirror, s->status.vflip,
        s->status.dcw, s->status.colorbar, s->pixformat,
        (unsigned) (s->xclk_freq_hz / 1000000), sizes);
    if (n < 0 || n >= (int) sizeof(json)) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, n);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    char part[64];
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    /* Rate over the last window, not since the stream began.
     *
     * A cumulative average decays asymptotically toward the steady rate and
     * never arrives, so an early fast burst -- the pipeline filling before
     * the link becomes the constraint -- makes the camera look like it is
     * slowing down for minutes. Every fps figure measured that way reads
     * high, by an amount that depends on how long the stream has run. */
    int64_t t0 = esp_timer_get_time();
    int64_t win_t0 = t0, win_bytes = 0, last_frame_us = t0;
    int64_t gap_min = INT64_MAX, gap_max = 0;
    uint32_t frames = 0, win_frames = 0;

    /* One viewer. The hand-off slot holds a single frame, so a second stream
     * would steal frames from the first rather than get its own; the fix if
     * that is ever wanted is a slot per viewer, not a deeper slot. */
    if (s_viewers > 0) {
        ESP_LOGW(TAG, "a stream is already running; refusing a second");
        return ESP_FAIL;
    }

    /* Discard anything left in the slot. The camera thread can publish one
     * more frame after the previous viewer left -- it may already have been
     * inside esp_camera_fb_get when the count dropped -- and that frame is now
     * as old as the gap between viewers. */
    {
        camera_fb_t *stale = slot_take();
        if (stale != NULL) {
            esp_camera_fb_return(stale);
        }
    }

    s_video_task = xTaskGetCurrentTaskHandle();
    s_viewers++;
    xSemaphoreGive(s_wake_camera);

    while (true) {
        /* Whatever the camera thread has most recently published. Waiting here
         * costs nothing: the network is the bottleneck, so by the time a send
         * finishes there is normally already a newer frame waiting. */
        camera_fb_t *fb = slot_take();
        while (fb == NULL) {
            /* Waiting is normal -- the camera has a frame period of its own,
             * and after a long send there may briefly be nothing new. Only
             * complain if it goes quiet for long enough to mean something is
             * wrong; ending the stream over it just turns a hiccup into a
             * disconnection. */
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0) {
                ESP_LOGW(TAG, "no frame for a second, still waiting");
            }
            fb = slot_take();
        }
        size_t fb_len_for_stats = fb->len;
        int hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned) fb->len);

        if ((res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY))) == ESP_OK &&
            (res = httpd_resp_send_chunk(req, part, hlen)) == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *) fb->buf, fb->len);
        }
        win_bytes += fb_len_for_stats;
        esp_camera_fb_return(fb);
        if (res != ESP_OK) {
            break;
        }

        frames++;
        win_frames++;
        {
            int64_t now = esp_timer_get_time();
            int64_t gap = now - last_frame_us;
            last_frame_us = now;
            if (gap < gap_min) gap_min = gap;
            if (gap > gap_max) gap_max = gap;
        }
        if (win_frames >= STATS_WINDOW) {
            int64_t now = esp_timer_get_time();
            double secs = (now - win_t0) / 1e6;
            ESP_LOGI(TAG,
                     "%u frames | %.1f fps over %.0fs, %.0f KB/s, %.1f KB/frame"
                     " | frame gap %.0f-%.0f ms",
                     (unsigned) frames, win_frames / secs, secs,
                     win_bytes / 1024.0 / secs,
                     win_bytes / 1024.0 / win_frames,
                     gap_min / 1000.0, gap_max / 1000.0);
            win_t0 = now;
            win_frames = 0;
            win_bytes = 0;
            gap_min = INT64_MAX;
            gap_max = 0;
        }
    }
    s_viewers--;
    s_video_task = NULL;
    {
        /* Do not leave a frame held after the viewer is gone. */
        camera_fb_t *stale = slot_take();
        if (stale != NULL) {
            esp_camera_fb_return(stale);
        }
    }
    ESP_LOGI(TAG, "stream ended after %u frames", (unsigned) frames);
    return res;
}

/* Nagle off on each accepted connection.
 *
 * The stream handler writes three chunks per frame -- boundary, part
 * header, then the JPEG -- and two small writes followed by a large one is
 * exactly what Nagle holds: the second waits for an ACK the peer is sitting
 * on for up to its delayed-ACK timer. Over Wi-Fi that cost this example
 * more than half its frame rate. esp_http_server only sets TCP_NODELAY on
 * its error path, so a handler that wants it has to ask. */
static esp_err_t sock_nodelay(httpd_handle_t hd, int sockfd)
{
    int one = 1;
    if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) < 0) {
        /* Fails on a non-socket descriptor, which is what direct injection
         * hands us. Nothing to turn off there. */
        ESP_LOGD(TAG, "TCP_NODELAY not applicable: %d", errno);
    }
    return ESP_OK;
}

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM, .pin_xclk = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM, .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM, .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        /* JPEG straight from the sensor: the alternative is converting every
         * frame on the CPU, which this does not have the time for. */
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        /* Four. Three things hold a buffer at any moment -- one in the
         * hand-off slot, one being sent, one being filled -- which leaves the
         * driver with exactly one and no room to start the next capture while
         * the current one is still queued. Three works only if nothing ever
         * overlaps; four gives the driver a spare so capture keeps running
         * while the network holds a frame. 61 KB each in PSRAM. */
        .fb_count = 4,
        .fb_location = CAMERA_FB_IN_PSRAM,
        /* Shortest queue the driver will give us for this fb_count. It
         * matters less than it would without the camera thread, which drains
         * the queue as fast as frames arrive -- but a shorter queue is still
         * strictly fresher, and nothing here wants the older end of it.
         *
         * Note this is not a different policy: the driver purges the oldest
         * frame when the queue is full in either mode (cam_hal.c). LATEST just
         * makes the queue one shorter. */
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init: %s", esp_err_to_name(err));
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        ESP_LOGI(TAG, "sensor pid 0x%02x", s->id.PID);
        /* The S3-EYE's module is mounted upside down relative to the
         * sensor's default orientation. */
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
    }
    return ESP_OK;
}

/* Declared before app_webrtc_init, not after.
 *
 * The pre-offer window is the only place a channel still reaches the SDP, and
 * a browser can connect at any point once signaling is up -- so declaring this
 * from camera_server_start, which runs later, would work until the day the
 * timing went the other way.
 *
 * SWSP keeps ordered and reliable because a lost frame there truncates an HTTP
 * response. Video is unordered so a retransmit of a stale frame cannot hold up
 * a fresh one behind it. No lifetime: measured, it abandons nothing, and
 * backpressure is what bounds staleness here. */
esp_err_t camera_server_declare_channels(void)
{
    app_webrtc_declare_data_channel(BITBANG_SWSP_CHANNEL, NULL);

    static app_webrtc_data_channel_init_t video_init;
    video_init.ordered = false;
    video_init.max_packet_lifetime_ms = 0;
    video_init.max_retransmits = 0;
    video_init.protocol = CAMERA_VIDEO_PROTOCOL;
    return (app_webrtc_declare_data_channel(CAMERA_VIDEO_CHANNEL, &video_init)
            == WEBRTC_STATUS_SUCCESS) ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_server_start(void)
{
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        return err;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 8;
    config.lru_purge_enable = true;
    config.open_fn = sock_nodelay;

    httpd_handle_t server = NULL;
    s_viewers_lock = xSemaphoreCreateMutex();
    s_slot_lock = xSemaphoreCreateMutex();
    s_wake_camera = xSemaphoreCreateBinary();
    if (s_slot_lock == NULL || s_wake_camera == NULL) {
        ESP_LOGE(TAG, "could not create the frame hand-off");
        return ESP_ERR_NO_MEM;
    }
    /* Above the httpd task that sends, so a finished capture is published
     * before the sender goes looking for it, and below the driver's own
     * cam_task. */
    xTaskCreate(camera_task, "camera", 4096, NULL, 6, NULL);

    /* PSRAM stack.
     *
     * This task descends into SCTP and DTLS, so it needs real room -- the
     * equivalent path in bitbang_verify measured 6.2 KB used. Taking that from
     * internal RAM broke the offer: KVS creates pthreads with 32 to 48 KB
     * stacks, internal heap was already down to about 97 KB and fragmented,
     * and a few more kilobytes was enough that one of them could no longer
     * find a contiguous block. The symptom was "pthread: Failed to create
     * task!" straight after both channels were created, and no session at all.
     *
     * CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY is already on. The constraint
     * that comes with it is that such a task must not run while the flash
     * cache is disabled; this one only captures and sends, and nothing here
     * writes flash after startup.
     *
     * Below the camera thread, so a finished capture is published before the
     * sender goes looking for it. */
    /* Roomy because the 4 KB chunk staging buffer is a local in video_task,
     * and this stack is in PSRAM where the space is not contended. */
    #define VIDEO_TASK_STACK (24 * 1024)
    static StaticTask_t video_tcb;
    StackType_t *video_stack = heap_caps_malloc(VIDEO_TASK_STACK,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (video_stack == NULL) {
        ESP_LOGE(TAG, "no PSRAM for the video task stack");
        return ESP_ERR_NO_MEM;
    }
    s_video_task = xTaskCreateStatic(video_task, "video", VIDEO_TASK_STACK, NULL, 5,
                                     video_stack, &video_tcb);

    /* The largest contiguous internal block, not just the total. KVS creates
     * pthreads with 32 to 48 KB stacks, so what matters is whether one of
     * those still fits -- a comfortable-looking total can be too fragmented
     * and the failure appears somewhere unrelated. */
    ESP_LOGI(TAG, "internal heap: %u free, largest block %u",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    bitbang_set_channel_open_handler(on_channel_open);

    err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return err;
    }

    /* /capture and /stream keep the ESP32-CAM names because that is what other
     * software already has in its config -- Home Assistant's generic and mjpeg
     * cameras, Frigate, a curl in a cron job. The page here uses neither: it
     * draws the data channel onto a canvas, which is the only path that works
     * on iOS. They are the machine-readable interface, not the human one. */
    httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler };
    httpd_uri_t stream_uri  = { .uri = "/stream",  .method = HTTP_GET, .handler = stream_handler };
    httpd_uri_t control_uri = { .uri = "/control", .method = HTTP_GET, .handler = control_handler };
    httpd_uri_t status_uri  = { .uri = "/status",  .method = HTTP_GET, .handler = status_handler };
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &control_uri);
    httpd_register_uri_handler(server, &status_uri);

    camera_server_handle = server;
    ESP_LOGI(TAG, "camera server listening on 127.0.0.1:%d", config.server_port);
    return ESP_OK;
}
