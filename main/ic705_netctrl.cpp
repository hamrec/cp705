// ---------------------------------------------------------------------------
// ic705_netctrl.cpp
//
// Icom WLAN remote-control network protocol (the proprietary protocol behind
// RS-BA1 / wfview / SDR-Control / FT8CN's network mode for the IC-705).
// This is NOT plain CI-V over a socket — it's a UDP session/auth layer on
// top of which CI-V frames are carried. Byte layouts below are ported from
// the open-source kappanhang project (github.com/nonoo/kappanhang), which
// reverse-engineered this protocol from real captures against an IC-705.
//
// Three UDP ports, each its own independent "stream" with its own session:
//   50001  control  - login (username/password) + auth + opens the others
//   50002  serial   - CI-V frames ride here once opened
//   50003  audio    - not implemented here; audio still goes through the
//                      existing stream_wifi_ic705.cpp path.
//
// Every stream starts with the same "are you there" SID handshake:
//   send pkt3 (type 0x10/0x03) x2 -> recv pkt4 (0x10/0x04), gives remote SID
//   send pkt6 (0x10/0x06) x2      -> recv pkt6 ack
// localSID = (low 16 bits of our local IP) << 16 | local UDP port (the
// shift is 32-bit and intentionally drops the IP's high 16 bits, matching
// the reference implementation's arithmetic exactly).
//
// Control stream then does: login (passcode-obfuscated user/pass) -> auth
// x2 -> wait for an unsolicited 0xa8 status packet + an auth-ok ack -> a
// "request serial+audio stream" packet, whose success reply hands us a
// fresh local/remote SID pair and auth ID to use for the rest of the
// session. The serial stream repeats the SID handshake on port 50002, then
// sends an "open" packet, after which raw CI-V bytes can be wrapped in a
// 21-byte stream header and sent.
//
// Keepalive: a small "pkt7" ping/pong every few seconds on each stream, plus
// periodic "pkt0" idle frames, plus a control-stream re-auth every 60s. The
// reference implementation also tracks sent packets for server-requested
// retransmission; we don't buffer history here (CAT traffic is low-rate and
// the link is a direct AP hop) — if the radio asks us to resend something
// we no longer have, we just answer with an idle frame at that sequence
// number, which is the documented fallback behavior.
// ---------------------------------------------------------------------------

#include "ic705_netctrl.h"

#include <cstring>
#include <cstdio>
#include <cstdarg>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "storage_service.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_heap_caps.h"

static const char* TAG = "IC705_NET";

// ---------------------------------------------------------------------------
// Passcode obfuscation (username/password encoding before they go on the wire)
// ---------------------------------------------------------------------------
static const uint8_t k_passcode_table[95] = {
    0x47, 0x5d, 0x4c, 0x42, 0x66, 0x20, 0x23, 0x46, 0x4e, 0x57, 0x45, 0x3d, 0x67, 0x76, 0x60, 0x41,
    0x62, 0x39, 0x59, 0x2d, 0x68, 0x7e, 0x7c, 0x65, 0x7d, 0x49, 0x29, 0x72, 0x73, 0x78, 0x21, 0x6e,
    0x5a, 0x5e, 0x4a, 0x3e, 0x71, 0x2c, 0x2a, 0x54, 0x3c, 0x3a, 0x63, 0x4f, 0x43, 0x75, 0x27, 0x79,
    0x5b, 0x35, 0x70, 0x48, 0x6b, 0x56, 0x6f, 0x34, 0x32, 0x6c, 0x30, 0x61, 0x6d, 0x7b, 0x2f, 0x4b,
    0x64, 0x38, 0x2b, 0x2e, 0x50, 0x40, 0x3f, 0x55, 0x33, 0x37, 0x25, 0x77, 0x24, 0x26, 0x74, 0x6a,
    0x28, 0x53, 0x4d, 0x69, 0x22, 0x5c, 0x44, 0x31, 0x36, 0x58, 0x3b, 0x7a, 0x51, 0x5f, 0x52,
};

static void passcode_encode(const char* s, uint8_t out[16]) {
    memset(out, 0, 16);
    size_t n = strlen(s);
    if (n > 16) n = 16;
    for (size_t i = 0; i < n; ++i) {
        int p = (uint8_t)s[i] + (int)i;
        if (p > 126) p = 32 + (p % 127);
        out[i] = k_passcode_table[p - 32];
    }
}

// ---------------------------------------------------------------------------
// Wire helpers
// ---------------------------------------------------------------------------
static inline void put_be32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}
static inline uint32_t get_be32(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static inline uint16_t get_be16(const uint8_t* p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}
static inline void put_be16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

// ---------------------------------------------------------------------------
// Per-stream UDP session (control or serial)
// ---------------------------------------------------------------------------
struct udp_sess_t {
    int      sock      = -1;
    uint32_t local_sid  = 0;
    uint32_t remote_sid = 0;
    uint16_t send_seq   = 1;   // pkt0-style outer sequence, shared by all tracked sends on this stream
    // Serializes the send_seq stamp across tasks. The audio stream is now sent
    // from BOTH the TX writer task (audio data, core 0) and the net task
    // (keepalives, core 1); without this a torn ++ could hand two packets the
    // same sequence, which the radio treats as a dup/loss -> audio gap.
    portMUX_TYPE seq_mux = portMUX_INITIALIZER_UNLOCKED;
};

static void dbg_log(const char* fmt, ...);  // forward decl: defined further below

static esp_err_t udp_sess_open_lr(udp_sess_t* s, uint32_t ip_be, uint16_t local_port, uint16_t remote_port) {
    s->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s->sock < 0) {
        dbg_log("udp_sess_open port=%u: socket() failed errno=%d", (unsigned)remote_port, errno);
        return ESP_ERR_NO_MEM;
    }

    // A retry can rebind the same local port within milliseconds of the
    // previous attempt's close() — without SO_REUSEADDR, lwIP can still
    // briefly consider that port in use and fail the bind.
    int reuse = 1;
    setsockopt(s->sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Reverted: binding to an ephemeral local port for SID1 (control, port
    // 50001) made it fail 100% of the time, where local==remote had been
    // reliable — so that channel still uses local==remote. SID2 (CIV) is
    // different: a real wfview session requests a random ephemeral local
    // civport in the Open request, not the fixed 50002 we'd been reusing
    // across dozens of reconnects tonight — worth testing whether stale
    // radio-side state tied to that fixed port is the rejection cause.
    struct sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port   = htons(local_port);
    local.sin_addr.s_addr = INADDR_ANY;
    if (bind(s->sock, (struct sockaddr*)&local, sizeof(local)) < 0) {
        dbg_log("udp_sess_open port=%u: bind() failed errno=%d", (unsigned)local_port, errno);
        close(s->sock); s->sock = -1;
        return ESP_FAIL;
    }

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_addr.s_addr = ip_be;
    dest.sin_port = htons(remote_port);
    if (connect(s->sock, (struct sockaddr*)&dest, sizeof(dest)) < 0) {
        dbg_log("udp_sess_open port=%u: connect() failed errno=%d", (unsigned)remote_port, errno);
        close(s->sock); s->sock = -1;
        return ESP_FAIL;
    }

    // getsockname() here was always returning 0.0.0.0 for the local address
    // (confirmed: every local_sid logged all night had its IP-derived upper
    // 16 bits at 0000, on two different networks) — lwIP doesn't seem to
    // backfill the real local address via getsockname() for a socket that
    // was explicitly bind()'d to INADDR_ANY before connect(). Query the
    // actual STA IP directly from the netif instead.
    esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info = {};
    if (!sta_netif || esp_netif_get_ip_info(sta_netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
        dbg_log("udp_sess_open port=%u: esp_netif_get_ip_info failed", (unsigned)remote_port);
        close(s->sock); s->sock = -1;
        return ESP_FAIL;
    }
    uint32_t local_ip_be32 = ntohl(ip_info.ip.addr);

    struct sockaddr_in got = {};
    socklen_t got_len = sizeof(got);
    if (getsockname(s->sock, (struct sockaddr*)&got, &got_len) < 0) {
        dbg_log("udp_sess_open port=%u: getsockname() failed errno=%d", (unsigned)remote_port, errno);
        close(s->sock); s->sock = -1;
        return ESP_FAIL;
    }
    uint16_t bound_port = ntohs(got.sin_port);
    s->local_sid = (local_ip_be32 << 16) | (uint32_t)bound_port;  // intentional 32-bit truncation, matches reference
    s->remote_sid = 0;
    s->send_seq = 1;
    return ESP_OK;
}

static esp_err_t udp_sess_open(udp_sess_t* s, uint32_t ip_be, uint16_t port) {
    return udp_sess_open_lr(s, ip_be, port, port);
}

static void udp_sess_close(udp_sess_t* s) {
    if (s->sock >= 0) { close(s->sock); s->sock = -1; }
}

// Polls with MSG_DONTWAIT instead of relying on select()/SO_RCVTIMEO — a
// non-blocking recv() cannot hang by definition, so this is bulletproof
// against any socket-option/select() quirk turning a bounded retry loop
// into an indefinite stall.
static int udp_recv_timeout(int sock, uint8_t* buf, size_t buflen, int ms) {
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
    for (;;) {
        int n = recv(sock, buf, buflen, MSG_DONTWAIT);
        if (n >= 0) return n;
        if (errno != EWOULDBLOCK && errno != EAGAIN) return -1;
        if (xTaskGetTickCount() >= deadline) return -1;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void dbg_log(const char* fmt, ...);  // forward decl: defined further below, used for send diagnostics here

static esp_err_t udp_send(udp_sess_t* s, const uint8_t* d, size_t len) {
    int sent = send(s->sock, d, len, 0);
    return (sent == (int)len) ? ESP_OK : ESP_FAIL;
}

// "Tracked" send: stamps the outer pkt0 sequence into bytes[6:8] and bumps it.
static esp_err_t udp_send_tracked(udp_sess_t* s, uint8_t* d, size_t len) {
    taskENTER_CRITICAL(&s->seq_mux);
    uint16_t seq = s->send_seq++;
    taskEXIT_CRITICAL(&s->seq_mux);
    d[6] = (uint8_t)(seq & 0xFF);
    d[7] = (uint8_t)((seq >> 8) & 0xFF);
    return udp_send(s, d, len);
}

// Same as above, but sent twice back-to-back for redundancy against packet
// loss — both copies carry the identical already-stamped sequence (the
// stamp/increment happens once), unlike a resend-after-timeout, so this
// can't make the radio see it as a second, separate request.
static esp_err_t udp_send_tracked_x2(udp_sess_t* s, uint8_t* d, size_t len) {
    d[6] = (uint8_t)(s->send_seq & 0xFF);
    d[7] = (uint8_t)((s->send_seq >> 8) & 0xFF);
    s->send_seq++;
    udp_send(s, d, len);
    return udp_send(s, d, len);
}

static char s_status[64] = "Idle";
static bool s_sd_log_ever_failed = false;
static bool s_sd_log_ever_succeeded = false;

// Durable debug trace, written straight to the SD card (/sdcard/IC705DBG.txt)
// so it can be read on a Mac with no copy step. Screen text truncates and
// clears on reboot/crash; this survives both. If the SD write itself is
// failing, that's surfaced directly in the on-screen status (see
// ic705_net_status_string()) so it doesn't require file access to diagnose.
// Per-packet SD logging is OFF by default. It was essential while reverse-
// engineering the protocol, but writing to the SD card for every packet
// during the time-critical SID/login handshake couples connection
// reliability to SD health — a flaky/slow card stalls these writes and the
// handshake times out (observed: a card that dropped into a bad state took
// the whole login down with it). Now that the protocol works, keep this
// dark. Flip s_verbose_sd_log to re-enable for future protocol debugging.
static bool s_verbose_sd_log = false;

static void dbg_log(const char* fmt, ...) {
    if (!s_verbose_sd_log) return;
    char buf[640];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 2) n = (int)sizeof(buf) - 2;
    buf[n] = '\n';
    buf[n + 1] = '\0';
    if (storage_sd_log_append("IC705DBG.txt", buf)) {
        s_sd_log_ever_succeeded = true;
    } else {
        s_sd_log_ever_failed = true;
    }
}

static void dbg_log_hex(const char* label, const uint8_t* d, int len, int max_show = 32) {
    if (!s_verbose_sd_log) return;
    char hex[3 * 168 + 1];
    int show = len < max_show ? len : max_show;
    if (show > 168) show = 168;
    int pos = 0;
    for (int i = 0; i < show && pos + 3 < (int)sizeof(hex); ++i) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02x ", d[i]);
    }
    dbg_log("%s len=%d: %s%s", label, len, hex, len > show ? "..." : "");
}

static void set_status(const char* fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    strncpy(s_status, buf, sizeof(s_status) - 1);
    s_status[sizeof(s_status) - 1] = '\0';
    dbg_log("%s", s_status);
}

static void service_incidental_pkt7(udp_sess_t* s, const uint8_t* r, int n);

// ---------------------------------------------------------------------------
// SID "are you there" handshake (pkt3/pkt4/pkt6), identical on every stream
// ---------------------------------------------------------------------------
static esp_err_t sess_handshake(udp_sess_t* s, const char* tag) {
    uint8_t pkt3[16] = { 0x10, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00 };
    put_be32(pkt3 + 8, s->local_sid);
    put_be32(pkt3 + 12, s->remote_sid);

    uint8_t rx[256];
    bool got_pkt4 = false;
    int pkt4_tries_used = 0;
    for (int tries = 0; tries < 8 && !got_pkt4; ++tries) {
        pkt4_tries_used = tries + 1;
        set_status("%s a%d/8", tag, tries + 1);
        udp_send(s, pkt3, sizeof(pkt3));
        udp_send(s, pkt3, sizeof(pkt3));
        int n = udp_recv_timeout(s->sock, rx, sizeof(rx), 1000);
        if (n > 0) service_incidental_pkt7(s, rx, n);
        if (n == 16 && memcmp(rx, "\x10\x00\x00\x00\x04\x00\x00\x00", 8) == 0) {
            s->remote_sid = get_be32(rx + 8);
            got_pkt4 = true;
        }
    }
    if (!got_pkt4) {
        ESP_LOGW("IC705_NET", "%s handshake FAILED at pkt3/pkt4 phase after %d/8 tries (no pkt4 reply)",
                 tag, pkt4_tries_used);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t pkt6[16] = { 0x10, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01, 0x00 };
    put_be32(pkt6 + 8, s->local_sid);
    put_be32(pkt6 + 12, s->remote_sid);

    bool got_ack = false;
    int ack_tries_used = 0;
    for (int tries = 0; tries < 8 && !got_ack; ++tries) {
        ack_tries_used = tries + 1;
        set_status("%s b%d/8", tag, tries + 1);
        udp_send(s, pkt6, sizeof(pkt6));
        udp_send(s, pkt6, sizeof(pkt6));
        int n = udp_recv_timeout(s->sock, rx, sizeof(rx), 1000);
        if (n > 0) service_incidental_pkt7(s, rx, n);
        if (n == 16 && memcmp(rx, "\x10\x00\x00\x00\x06\x00\x01\x00", 8) == 0) {
            got_ack = true;
        }
    }
    if (!got_ack) {
        ESP_LOGW("IC705_NET", "%s handshake FAILED at pkt6/ack phase after pkt4 ok (%d/8 tries), no ack reply",
                 tag, ack_tries_used);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI("IC705_NET", "%s handshake OK (pkt4 in %d/8, ack in %d/8)", tag, pkt4_tries_used, ack_tries_used);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static char     s_username[32] = "";
static char     s_password[32] = "";
static udp_sess_t s_ctrl;
static udp_sess_t s_serial;
static udp_sess_t s_audio;
static uint8_t  s_auth_id[6]   = {0};
static uint16_t s_auth_inner_seq = 0;
static volatile bool s_ready = false;
static volatile bool s_audio_ready = false;
// Audio (SID3) bring-up retry. The one-shot handshake inside do_connect fails
// intermittently because the radio's audio server (:50003) is silent on that
// single pass even though control/CIV already answer. Rather than give up audio
// for the whole session, we fall into a non-blocking retry driven from the main
// net loop (so keepalives keep flowing the entire time). NONE = not retrying
// (either already ready, or audio was never requested); WANT_PKT4 = resending
// areYouThere; WANT_ACK = got pkt4, resending areYouReady until acked.
enum audio_bringup_t { AUDIO_BU_NONE = 0, AUDIO_BU_WANT_PKT4, AUDIO_BU_WANT_ACK };
static audio_bringup_t s_audio_bu = AUDIO_BU_NONE;
static TickType_t s_audio_bu_last_send = 0;
static TaskHandle_t s_task = nullptr;
static QueueHandle_t s_civ_out_q = nullptr;
static QueueHandle_t s_audio_out_q = nullptr;   // PCM frames awaiting send (TX)
static QueueHandle_t s_audio_in_q  = nullptr;   // PCM frames received (RX)

#define CIV_FRAME_MAX 40
// RX: the IC-705 sends audio in alternating ~1364- and ~556-byte PCM
// payloads (per wfview's analysis), so the receive path must hold a full
// 1364-byte chunk or it truncates and corrupts the stream → no decode.
#define AUDIO_RX_PCM_MAX 1364
// TX: the writer sends uniform 480-sample (960-byte) frames. 1024 covers it with
// headroom. (Was 1364 for an abandoned 682/278 framing — reverted to save RAM.)
#define AUDIO_TX_PCM_MAX 1024

// Audio queue depths.
#define AUDIO_IN_Q_DEPTH  6
#define AUDIO_OUT_Q_DEPTH 4

// STATIC queue storage. The RX queue needs 6*(2+1364)=8196 contiguous bytes,
// but on this no-PSRAM board the largest free heap block sits around 7KB once
// WiFi + the FT8 pipeline are up — so a dynamic xQueueCreate() SILENTLY FAILS
// (returns NULL). When that happened, every audio packet the radio sent was
// dropped at the (null) queue, rx_pkts stayed 0, and the decoder starved
// despite audio flowing perfectly on the wire. Allocating the queue storage
// statically in .bss guarantees it exists, exactly like the static task
// stacks. Same treatment for the (small) TX queue for robustness.
static uint8_t      s_audio_in_q_storage[AUDIO_IN_Q_DEPTH * (2 + AUDIO_RX_PCM_MAX)];
static StaticQueue_t s_audio_in_q_buf;
static uint8_t      s_audio_out_q_storage[AUDIO_OUT_Q_DEPTH * (2 + AUDIO_TX_PCM_MAX)];
static StaticQueue_t s_audio_out_q_buf;

void ic705_net_set_credentials(const char* username, const char* password) {
    strncpy(s_username, username ? username : "", sizeof(s_username) - 1);
    strncpy(s_password, password ? password : "", sizeof(s_password) - 1);
}

const char* ic705_net_status_string(void) {
    static char display[80];
    if (s_sd_log_ever_failed) {
        snprintf(display, sizeof(display), "E%d dma=%u", g_storage_sd_log_last_code,
                 (unsigned)g_storage_sd_log_dma_largest);
        return display;
    }
    return s_status;
}
bool ic705_net_is_ready(void) { return s_ready; }

// TEMP DIAGNOSTIC: dump a packet as hex to the USB serial console (not SD).
static void serial_hexdump(const char* label, const uint8_t* d, int n, int max) {
    if (max > n) max = n;
    char line[3 * 96 + 1];
    int p = 0;
    for (int i = 0; i < max && p < (int)sizeof(line) - 3; ++i) {
        p += snprintf(line + p, sizeof(line) - p, "%02x", d[i]);
    }
    line[p] = 0;
    ESP_LOGW(TAG, "HEX %s n=%d: %s", label, n, line);
}

// ---------------------------------------------------------------------------
// Control-stream packet builders
// ---------------------------------------------------------------------------
static void build_login_pkt(uint8_t out[128]) {
    memset(out, 0, 128);
    out[0] = 0x80;
    put_be32(out + 8, s_ctrl.local_sid);
    put_be32(out + 12, s_ctrl.remote_sid);
    out[19] = 0x70; out[20] = 0x01;
    // innerseq (offset 0x16-0x17, BIG-ENDIAN), exactly as wfview's
    // qToBigEndian(authSeq) and as build_auth_pkt/build_request_serial_audio_pkt
    // already do. This was previously (mis)written at out[23]/[24] in
    // little-endian — benign by coincidence for small seq values, but a real
    // deviation from wfview's wire format; corrected for exact parity.
    out[22] = (uint8_t)((s_auth_inner_seq >> 8) & 0xFF);
    out[23] = (uint8_t)(s_auth_inner_seq & 0xFF);
    s_auth_inner_seq++;
    uint8_t auth_start[2];
    esp_fill_random(auth_start, sizeof(auth_start));
    out[26] = auth_start[0];
    out[27] = auth_start[1];
    uint8_t user_enc[16], pass_enc[16];
    passcode_encode(s_username, user_enc);
    passcode_encode(s_password, pass_enc);
    memcpy(out + 64, user_enc, 16);
    memcpy(out + 80, pass_enc, 16);
    // A real successful wfview session sends its own computer name + app
    // name here (e.g. "Deans-Ma-wfview"), not a generic placeholder — this
    // was the one reproducible difference found against a captured working
    // login, after verifying everything else (passcode encoding included)
    // byte-for-byte matches.
    // Device/app name shown on the radio's connected-client list (cosmetic).
    memcpy(out + 96, "CP705", 5);
}

static void build_auth_pkt(uint8_t out[64], uint8_t magic) {
    memset(out, 0, 64);
    out[0] = 0x40;
    put_be32(out + 8, s_ctrl.local_sid);
    put_be32(out + 12, s_ctrl.remote_sid);
    out[19] = 0x30; out[20] = 0x01; out[21] = magic;
    // innerseq (offset 0x16-0x17, BIG-ENDIAN) — same fix as the request_serial_audio
    // packet; was one byte too far right and little-endian here too.
    out[22] = (uint8_t)((s_auth_inner_seq >> 8) & 0xFF);
    out[23] = (uint8_t)(s_auth_inner_seq & 0xFF);
    s_auth_inner_seq++;
    memcpy(out + 26, s_auth_id, 6);
    // resetcap (offset 0x24, BIG-ENDIAN 0x0798) — confirmed via real wfview
    // capture; was never set at all (left zero). Likely THE cause of the
    // radio's persistent response=0xFFFFFFFF rejection on every single auth
    // attempt regardless of fresh token data, since a zeroed required field
    // would be rejected consistently rather than intermittently.
    out[36] = 0x07; out[37] = 0x98;
}

static void build_request_serial_audio_pkt(uint8_t out[144], uint16_t civ_local_port, uint16_t audio_local_port) {
    memset(out, 0, 144);
    out[0] = 0x90;
    put_be32(out + 8, s_ctrl.local_sid);
    put_be32(out + 12, s_ctrl.remote_sid);
    out[19] = 0x80; out[20] = 0x01; out[21] = 0x03;
    // innerseq (offset 0x16-0x17, BIG-ENDIAN) — verified against a real
    // captured wfview request packet byte-for-byte: was being written one
    // byte too far right (0x17-0x18) and in little-endian, which also
    // corrupted the following 2-byte zero-padding field.
    out[22] = (uint8_t)((s_auth_inner_seq >> 8) & 0xFF);
    out[23] = (uint8_t)(s_auth_inner_seq & 0xFF);
    s_auth_inner_seq++;
    // commoncap (offset 0x27, little-endian 0x8010) and macaddress (offset
    // 0x2a) — both verified against the real capture. macaddress must be
    // the RADIO's own MAC, not ours: since we're connected directly to the
    // radio's own AP, its BSSID *is* its MAC.
    out[39] = 0x10; out[40] = 0x80;
    wifi_ap_record_t ap_info = {};
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        memcpy(out + 42, ap_info.bssid, 6);
    }
    memcpy(out + 26, s_auth_id, 6);
    // offset 0x30 (32) is genuinely unused/zero, confirmed by real capture
    // name field (offset 0x40) is the TARGET RADIO's model string, not our
    // own device name — confirmed by real capture ("IC-705" appears here,
    // not the client's computer name). Reverting my earlier wrong fix.
    memcpy(out + 64, "IC-705", 6);
    uint8_t user_enc[16];
    passcode_encode(s_username, user_enc);
    memcpy(out + 96, user_enc, 16);
    // rx/txenable, codecs, sample rates: a real working request DOES
    // populate these (rxenable=1, txenable=1, codec=4/LPCM16, 48000/48000),
    // confirmed by real capture — my earlier "CIV-only" simplification was
    // a wrong guess, reverting it.
    out[112] = 0x01; out[113] = 0x01; out[114] = 0x04; out[115] = 0x04;
    const uint16_t sample_rate = 48000;
    out[118] = (uint8_t)(sample_rate >> 8); out[119] = (uint8_t)(sample_rate & 0xFF);
    out[122] = (uint8_t)(sample_rate >> 8); out[123] = (uint8_t)(sample_rate & 0xFF);
    out[126] = (uint8_t)(civ_local_port >> 8); out[127] = (uint8_t)(civ_local_port & 0xFF);
    out[130] = (uint8_t)(audio_local_port >> 8); out[131] = (uint8_t)(audio_local_port & 0xFF);
    out[134] = 0x00; out[135] = 0x96;  // buffer=150ms — wfview's proven value (rx+tx
                                       // both 150). kappanhang's 300ms may have added
                                       // RX latency and thrown off the decode-window
                                       // re-anchor; the clean-TX fix is the pkt0-idle
                                       // gating, not this, so 150 is the safe choice.
    out[136] = 0x01;
}

// pkt7 ping/pong (keepalive + latency probe), shared shape on every stream
static void send_pkt7_ping(udp_sess_t* s, uint16_t seq) {
    uint8_t rand_id[1];
    esp_fill_random(rand_id, sizeof(rand_id));
    uint8_t d[21] = { 0x15, 0x00, 0x00, 0x00, 0x07, 0x00,
                       (uint8_t)(seq & 0xFF), (uint8_t)((seq >> 8) & 0xFF) };
    put_be32(d + 8, s->local_sid);
    put_be32(d + 12, s->remote_sid);
    d[16] = 0x00;
    d[17] = rand_id[0];
    d[18] = 0x00; d[19] = 0x00; d[20] = 0x06;
    udp_send(s, d, sizeof(d));
}

static void send_pkt7_reply(udp_sess_t* s, const uint8_t* reply_id4, uint16_t seq) {
    uint8_t d[21] = { 0x15, 0x00, 0x00, 0x00, 0x07, 0x00,
                       (uint8_t)(seq & 0xFF), (uint8_t)((seq >> 8) & 0xFF) };
    put_be32(d + 8, s->local_sid);
    put_be32(d + 12, s->remote_sid);
    d[16] = 0x01;
    memcpy(d + 17, reply_id4, 4);
    udp_send(s, d, sizeof(d));
}

static bool is_pkt7(const uint8_t* r, int len) {
    return len == 21 && memcmp(r + 1, "\x00\x00\x00\x07\x00", 5) == 0;
}

// The radio starts sending pkt7 keepalive pings well before login/auth even
// finish — confirmed from a real capture: it pings within ~1s of the SID
// handshake, while we're still mid-login. If we don't reply, we look dead
// from the radio's side and it seems to abandon the session before ever
// sending the login/auth/open replies we're waiting for. Call this on every
// received packet during do_connect()'s wait loops, not just in the
// steady-state loop after the connection is already fully established.
static void service_incidental_pkt7(udp_sess_t* s, const uint8_t* r, int n) {
    if (!is_pkt7(r, n)) return;
    uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
    if (r[16] == 0x00) send_pkt7_reply(s, r + 17, seq);
}

static void send_pkt0_idle(udp_sess_t* s) {
    uint8_t d[16] = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    put_be32(d + 8, s->local_sid);
    put_be32(d + 12, s->remote_sid);
    udp_send_tracked(s, d, sizeof(d));
}

// Control packet type 0x05 = "disconnect": tells the radio to release this
// stream's session immediately instead of waiting for its inactivity timeout.
// Sent on every stream at teardown so the radio is free for the next connection
// (otherwise it holds the stale session and rejects a reconnect until it times
// out — which is why the radio currently needs a reboot between sessions).
// UDP, so send a few copies for reliability.
static void send_disconnect(udp_sess_t* s) {
    if (s->sock < 0) return;
    uint8_t d[16] = { 0x10, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00 };
    put_be32(d + 8, s->local_sid);
    put_be32(d + 12, s->remote_sid);
    udp_send(s, d, sizeof(d));
    udp_send(s, d, sizeof(d));
    udp_send(s, d, sizeof(d));
}

// Server asked us to retransmit a packet we no longer have on hand; the
// documented fallback is to answer with an idle frame stamped at that seq.
static void send_pkt0_idle_at_seq(udp_sess_t* s, uint16_t seq) {
    uint8_t d[16] = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
                       (uint8_t)(seq & 0xFF), (uint8_t)((seq >> 8) & 0xFF) };
    put_be32(d + 8, s->local_sid);
    put_be32(d + 12, s->remote_sid);
    udp_send(s, d, sizeof(d));
    udp_send(s, d, sizeof(d));
}

static bool is_pkt0(const uint8_t* r, int len) {
    return len >= 16 &&
           (memcmp(r, "\x10\x00\x00\x00\x00\x00", 6) == 0 ||
            memcmp(r, "\x10\x00\x00\x00\x01\x00", 6) == 0 ||
            memcmp(r, "\x18\x00\x00\x00\x01\x00", 6) == 0);
}

// ---------------------------------------------------------------------------
// Serial (CI-V) stream
// ---------------------------------------------------------------------------
static uint16_t s_serial_send_seq_inner = 0;

static esp_err_t serial_send_open_close(bool do_close) {
    uint8_t magic = do_close ? 0x00 : 0x05;
    uint8_t d[22] = { 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    put_be32(d + 8, s_serial.local_sid);
    put_be32(d + 12, s_serial.remote_sid);
    d[16] = 0xc0; d[17] = 0x01; d[18] = 0x00;
    d[19] = (uint8_t)((s_serial_send_seq_inner >> 8) & 0xFF);
    d[20] = (uint8_t)(s_serial_send_seq_inner & 0xFF);
    d[21] = magic;
    s_serial_send_seq_inner++;
    return udp_send_tracked(&s_serial, d, sizeof(d));
}

esp_err_t ic705_net_send_civ(const uint8_t* frame, size_t len) {
    if (!s_ready || !s_civ_out_q) return ESP_ERR_INVALID_STATE;
    if (len > CIV_FRAME_MAX) return ESP_ERR_INVALID_SIZE;
    uint8_t buf[1 + CIV_FRAME_MAX];
    buf[0] = (uint8_t)len;
    memcpy(buf + 1, frame, len);
    if (xQueueSend(s_civ_out_q, buf, pdMS_TO_TICKS(50)) != pdTRUE) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

static esp_err_t serial_send_civ_now(const uint8_t* frame, uint8_t len) {
    uint8_t d[21 + CIV_FRAME_MAX];
    d[0] = (uint8_t)(0x15 + len);
    memset(d + 1, 0, 7);
    put_be32(d + 8, s_serial.local_sid);
    put_be32(d + 12, s_serial.remote_sid);
    d[16] = 0xc1; d[17] = len; d[18] = 0x00;
    d[19] = (uint8_t)((s_serial_send_seq_inner >> 8) & 0xFF);
    d[20] = (uint8_t)(s_serial_send_seq_inner & 0xFF);
    s_serial_send_seq_inner++;
    memcpy(d + 21, frame, len);
    return udp_send_tracked(&s_serial, d, 21 + len);
}

// ---------------------------------------------------------------------------
// Audio stream — wire format per wfview's audio_packet struct (packettypes.h,
// AUDIO_SIZE=0x18/24-byte header), verified against the real protocol
// definition, NOT guessed:
//   0x00 len      (4, little-endian) total packet length
//   0x04 type     (2) — receive side: type!=0x01 && len>=0x20 means real
//                  audio data (anything else is incidental/idle)
//   0x06 seq      (2) outer tracked sequence, same convention as every other
//                  stream (stamped by udp_send_tracked())
//   0x08 sentid   (4, big-endian) — s_audio.local_sid
//   0x0c rcvdid   (4, big-endian) — s_audio.remote_sid
//   0x10 ident    (2, little-endian, direct assignment — NOT byte-swapped on
//                  the wire, same as commoncap elsewhere) — 0x0080 normally,
//                  0x9781 for an exactly-0xa0-byte payload (wfview's own
//                  special case, kept for exactness even though our LPCM16
//                  chunks won't usually land on that size)
//   0x12 sendseq  (2, BIG-endian) — separate per-audio-stream counter,
//                  distinct from the outer seq at 0x06
//   0x14 unused   (2)
//   0x16 datalen  (2, BIG-endian) — payload length that follows
//   0x18+ payload — raw PCM
// ---------------------------------------------------------------------------
static uint16_t s_audio_send_seq_inner = 0;

static esp_err_t audio_send_pcm_now(const uint8_t* pcm, uint16_t len) {
    if (len > AUDIO_TX_PCM_MAX) len = AUDIO_TX_PCM_MAX;
    uint16_t total = 24 + len;
    uint8_t d[24 + AUDIO_TX_PCM_MAX];
    memset(d, 0, 24);
    d[0] = (uint8_t)(total & 0xFF);
    d[1] = (uint8_t)((total >> 8) & 0xFF);
    put_be32(d + 8, s_audio.local_sid);
    put_be32(d + 12, s_audio.remote_sid);
    uint16_t ident = (len == 0xa0) ? 0x9781 : 0x0080;
    d[16] = (uint8_t)(ident & 0xFF);
    d[17] = (uint8_t)((ident >> 8) & 0xFF);
    put_be16(d + 18, s_audio_send_seq_inner);
    s_audio_send_seq_inner++;
    put_be16(d + 22, len);
    memcpy(d + 24, pcm, len);
    return udp_send_tracked(&s_audio, d, total);
}

// Diagnostic counters for the TX audio path — a failed send() (WiFi TX buffer
// exhaustion) drops a ~10ms chunk the radio can't conceal, which we suspect is
// the irregular tune/TX carrier pump. Surfaced once per TX via the getter below.
static volatile uint32_t s_audio_tx_ok = 0;
static volatile uint32_t s_audio_tx_fail = 0;
// Count of audio retransmit requests from the radio (it detected a gap in our
// audio — an air-level loss send() can't see — and we answer with an idle frame,
// inserting silence into its playout = a power dip). Suspected pump source.
static volatile uint32_t s_audio_rexmit = 0;

void ic705_net_get_audio_tx_stats(uint32_t* ok, uint32_t* fail, uint32_t* rexmit) {
    if (ok) *ok = s_audio_tx_ok;
    if (fail) *fail = s_audio_tx_fail;
    if (rexmit) *rexmit = s_audio_rexmit;
}
// Timestamp of the last TX audio packet — used to tell RX (no TX) from active
// TX so we keep the audio-stream pkt0 idle keepalive flowing during receive
// (the IC-705 needs it for smooth RX audio) but suppress it while transmitting
// (interleaving idle frames into the TX audio stream splatters the signal).
static volatile int64_t s_last_audio_tx_us = 0;

void ic705_net_reset_audio_tx_stats(void) {
    s_audio_tx_ok = 0;
    s_audio_tx_fail = 0;
    s_audio_rexmit = 0;
}

esp_err_t ic705_net_send_audio_pcm(const uint8_t* pcm, size_t len) {
    if (!s_audio_ready) return ESP_ERR_INVALID_STATE;
    if (len > AUDIO_TX_PCM_MAX) return ESP_ERR_INVALID_SIZE;
    // Send DIRECTLY from the caller (the TX writer task, which paces at an exact
    // 48kHz). Previously this queued the packet for the net task to send on its
    // own loop — but the two tasks drift in and out of phase, so a packet's
    // actual send time oscillated by up to a full 10ms tick, starving the
    // radio's audio buffer periodically and fluttering the carrier. Sending here
    // removes the hand-off entirely. The send_seq stamp is serialized against
    // the net task's keepalives by udp_sess_t::seq_mux. lwIP socket send() is
    // thread-safe, so concurrent send (here, core 0) + recv (net task, core 1)
    // on s_audio is fine.
    s_last_audio_tx_us = esp_timer_get_time();
    esp_err_t r = audio_send_pcm_now(pcm, (uint16_t)len);
    if (r == ESP_OK) s_audio_tx_ok++; else s_audio_tx_fail++;
    return r;
}

int ic705_net_recv_audio_pcm(uint8_t* out, size_t max_len, int timeout_ms) {
    if (!s_audio_in_q) return 0;
    uint8_t buf[2 + AUDIO_RX_PCM_MAX];
    if (xQueueReceive(s_audio_in_q, buf, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return 0;
    uint16_t len = (uint16_t)(buf[0] | (buf[1] << 8));
    if (len > max_len) len = (uint16_t)max_len;
    memcpy(out, buf + 2, len);
    return (int)len;
}

bool ic705_net_audio_is_ready(void) { return s_audio_ready; }

// ---------------------------------------------------------------------------
// Connect sequence (runs once at the start of the background task)
// ---------------------------------------------------------------------------
static esp_err_t do_connect(uint32_t ic705_ip) {
    set_status("SID1...");
    // PRIME-SUSPECT TEST: bind the control stream to a RANDOM EPHEMERAL local
    // port (remote stays 50001), matching wfview — which never uses
    // local==remote. The radio may validate the token against our source
    // port; a control stream sourced from 50001 (the radio's own port) is
    // the one structural difference from a working wfview session that
    // isn't in any packet's content. local_sid is still computed from the
    // actual bound port via getsockname() (verified to match wfview's myId
    // formula), so the SID/handshake stays self-consistent. A session-2
    // attempt at this "broke SID1 100%", but that likely also disturbed the
    // local_sid computation (since fixed) — retrying cleanly now.
    uint16_t ctrl_local_port = 45000 + (esp_random() % 15000);
    dbg_log("ctrl_local_port=%u (remote 50001)", ctrl_local_port);
    esp_err_t err = udp_sess_open_lr(&s_ctrl, ic705_ip, ctrl_local_port, 50001);
    if (err != ESP_OK) { set_status("Sock1 fail"); return err; }
    err = sess_handshake(&s_ctrl, "SID1");
    if (err != ESP_OK) { set_status("SID1 fail"); return err; }
    dbg_log("SID1 ok: local_sid=%08x remote_sid=%08x", s_ctrl.local_sid, s_ctrl.remote_sid);

    // No artificial delay before login — wfview sends login IMMEDIATELY on
    // receiving "I am ready" (icomUdpHandler::dataReceived, type 0x06 →
    // sendLogin()). A token-validity window on the radio could be sensitive
    // to delay; match wfview's back-to-back timing (its full login→token→open
    // completes in ~7ms in the real capture).

    // Resent on a timeout, same as the SID handshake: earlier tonight, a
    // build that resent login on timeout (with a fresh outer sequence each
    // try) was observed reaching the auth/open stages successfully more
    // than once; a later change to "send exactly once" — based on an
    // unconfirmed theory that resending could confuse the radio about an
    // already-logging-in session — has produced zero successful logins
    // since. Reverting to retry-with-resend.
    set_status("Login...");
    dbg_log("login creds: user='%s' pass='%s'", s_username, s_password);
    uint8_t login[128];
    build_login_pkt(login);
    dbg_log_hex("login tx", login, sizeof(login), 64);

    uint8_t rx[256];
    bool got_login_ans = false;
    {
        int last_unmatched_len = -1;
        uint8_t last_unmatched_b0 = 0;
        for (int tries = 0; tries < 8 && !got_login_ans; ++tries) {
            // Single send (was x2). wfview sends each control packet exactly
            // once; sending twice puts a duplicate outer-sequence packet on
            // the wire that the radio processes as a replay. See the Auth
            // step below for why this matters (token replay → rejection).
            udp_send_tracked(&s_ctrl, login, sizeof(login));
            TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
            while (!got_login_ans && xTaskGetTickCount() < deadline) {
                int n = udp_recv_timeout(s_ctrl.sock, rx, sizeof(rx), 200);
                if (n > 0) dbg_log_hex("login-wait rx", rx, n);
                if (n > 0) service_incidental_pkt7(&s_ctrl, rx, n);
                if (n == 96 && memcmp(rx, "\x60\x00\x00\x00\x00\x00", 6) == 0) {
                    got_login_ans = true;
                } else if (n > 0) {
                    last_unmatched_len = n;
                    last_unmatched_b0 = rx[0];
                }
            }
        }
        if (!got_login_ans) {
            // Surface whatever (if anything) showed up but didn't match,
            // so we can tell "radio answered with something unexpected"
            // apart from "radio never answered at all".
            if (last_unmatched_len >= 0) {
                set_status("LoginTO L%d:%02x", last_unmatched_len, last_unmatched_b0);
            } else {
                set_status("LoginTO none");
            }
            return ESP_ERR_TIMEOUT;
        }
    }
    serial_hexdump("login-resp", rx, 96, 64);
    if (memcmp(rx + 48, "\xff\xff\xff\xfe", 4) == 0) {
        set_status("BadAuth");
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(s_auth_id, rx + 26, 6);

    // Per the real wfview capture: there is NO direct 64-byte ack to the
    // initial sendToken(0x02) at all in the actual fast path (verified by
    // timestamp — wfview sends its Open/conninfo request ~7ms after Login,
    // having received only a 168-byte capabilities packet and a 144-byte
    // radio-initiated "conninfo" broadcast in between, never a TOKEN_SIZE
    // response). The REAL trigger to proceed to Open is that 144-byte
    // broadcast (requestreply=0x03), gated on its "busy" field (offset
    // 0x60/96) being 0. Our previous approach of gating on a direct
    // 64-byte response — which the radio was, in fact, persistently
    // rejecting (response=0xFFFFFFFF) every time we retried — was waiting
    // on a signal the real protocol doesn't even depend on.
    uint8_t auth02[64];
    build_auth_pkt(auth02, 0x02);
    dbg_log_hex("auth02 tx", auth02, sizeof(auth02), 64);

    // ROOT-CAUSE FIX (found by diffing the radio's responses against a real
    // wfview capture): the radio was REJECTING our token — the capabilities
    // (0xa8) and status (0x50) packets it sent back carried 0xFFFFFFFF in
    // their response field (offset 0x30), where a real wfview session gets
    // 0x00000000. Our token packet content is byte-perfect vs the capture,
    // so the rejection was contextual: we sent the token TWICE
    // (udp_send_tracked_x2, same outer sequence) AND resent it on every
    // retry iteration. wfview sends the token EXACTLY ONCE. The radio
    // processes the duplicate as a replay and invalidates the token, which
    // then makes every downstream step (Open) fail too. Fix: send the token
    // exactly once, then listen WITHOUT resending it.
    set_status("Auth...");
    udp_send_tracked(&s_ctrl, auth02, sizeof(auth02));
    bool ready_for_open = false;
    TickType_t auth_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    while (!ready_for_open && xTaskGetTickCount() < auth_deadline) {
        int n = udp_recv_timeout(s_ctrl.sock, rx, sizeof(rx), 200);
        if (n > 0) dbg_log_hex("auth-wait rx", rx, n, 168);
        if (n > 0) service_incidental_pkt7(&s_ctrl, rx, n);
        if (n == 168) serial_hexdump("caps", rx, 168, 120);
        if (n == 144) serial_hexdump("conninfo-bcast", rx, 144, 96);
        // The radio-initiated 144-byte conninfo broadcast (requestreply=3,
        // requesttype=0), busy==0, is the go-ahead to send Open.
        if (n == 144 && rx[0] == 0x90 && rx[20] == 0x03 && rx[21] == 0x00) {
            uint32_t busy = get_be32(rx + 96);
            if (busy == 0) ready_for_open = true;
        }
    }
    if (!ready_for_open) { set_status("AuthTO"); return ESP_ERR_TIMEOUT; }

    set_status("Open...");
    // Use a random ephemeral local CIV port (matching what a real wfview
    // session does) instead of the same fixed 50002 we'd reused across
    // dozens of reconnects tonight — testing whether stale radio-side
    // per-port session state is the rejection cause.
    uint16_t civ_local_port = 51000 + (esp_random() % 8000);
    uint16_t audio_local_port = 59000 + (esp_random() % 500);
    uint8_t req[144];
    build_request_serial_audio_pkt(req, civ_local_port, audio_local_port);
    dbg_log("civ_local_port=%u audio_local_port=%u", civ_local_port, audio_local_port);
    dbg_log_hex("open tx", req, sizeof(req), 144);

    // Send the Open/conninfo request EXACTLY ONCE (was: x8 retries). Same
    // root cause as the token fix above — a duplicate stream-open request
    // is a replay the radio can reject, and the Open allocates real stream
    // state server-side (wfview gates civ/audio creation on !streamOpened),
    // so it especially must not be sent twice. Listen ~3s for the 80-byte
    // status ack without resending.
    serial_hexdump("conn-req", req, sizeof(req), 96);
    udp_send_tracked(&s_ctrl, req, sizeof(req));
    bool opened = false;
    uint16_t granted_civ_port = 0, granted_audio_port = 0;
    TickType_t open_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    while (!opened && xTaskGetTickCount() < open_deadline) {
        int n = udp_recv_timeout(s_ctrl.sock, rx, sizeof(rx), 200);
        if (n > 0) dbg_log_hex("open-wait rx", rx, n, 80);
        if (n > 0) service_incidental_pkt7(&s_ctrl, rx, n);
        // The ack to a stream-open request is an 80-byte status_packet
        // (len field = 0x50). type != 1, error(offset0x30,u32)==0,
        // disc(offset0x40)!=1 means success; civport/audioport at 0x42/0x46
        // (big-endian) are the radio's GRANTED remote ports — per wfview
        // (civPort = qFromBigEndian(in->civport)), these are what we
        // actually connect to, not assumed to always equal what we asked for.
        if (n == 80 && rx[0] == 0x50) serial_hexdump("open-ack", rx, 80, 80);
        if (n == 168) serial_hexdump("caps", rx, 168, 96);
        if (n == 80 && rx[0] == 0x50 &&
            !(rx[4] == 0x01 && rx[5] == 0x00) &&
            get_be32(rx + 0x30) == 0 && rx[0x40] != 0x01) {
            s_ctrl.remote_sid = get_be32(rx + 8);
            s_ctrl.local_sid  = get_be32(rx + 12);
            memcpy(s_auth_id, rx + 26, 6);
            granted_civ_port   = get_be16(rx + 0x42);
            granted_audio_port = get_be16(rx + 0x46);
            opened = true;
        }
    }
    if (!opened) { set_status("OpenTO"); return ESP_ERR_TIMEOUT; }
    dbg_log("granted civport=%u audioport=%u", granted_civ_port, granted_audio_port);

    set_status("SID2...");
    err = udp_sess_open_lr(&s_serial, ic705_ip, civ_local_port, granted_civ_port);
    if (err != ESP_OK) { set_status("Sock2 fail"); return err; }
    err = sess_handshake(&s_serial, "SID2");
    if (err != ESP_OK) { set_status("SID2 fail"); return err; }

    serial_send_open_close(false);

    // Audio (SID3) — same SID-handshake pattern as control/serial, on the
    // local/remote ports negotiated in the Open request/ack above. Per
    // wfview's icomUdpAudio constructor: bind locally to the negotiated
    // port, connect to the radio's GRANTED remote audio port, then send
    // its own "are you there" (the exact same areYouThere/areYouReady
    // exchange used on every other stream). Non-fatal if it fails — CAT
    // control already works without it.
    set_status("SID3...");
    s_audio_send_seq_inner = 0;
    // Depth 4 (was 8): each slot is 2+AUDIO_PCM_MAX (~514B), so 8 cost ~8KB
    // across both queues — too much on a no-PSRAM board. Depth 4 ≈ 21ms of
    // 48kHz audio buffering, plenty since RX decode buffers 15s in the
    // waterfall and TX is paced by the writer task.
    // RX queue holds full ~1364-byte radio chunks; depth 6 ≈ 80ms of audio
    // buffering so the FT8 pipeline can't starve while the net task is busy.
    // TX queue is small (our 512-byte DDS chunks).
    // Static queues — guaranteed to exist regardless of heap fragmentation
    // (a dynamic xQueueCreate of the 8KB RX queue was failing on this board,
    // silently dropping ALL audio; see the storage declarations above).
    if (!s_audio_out_q)
        s_audio_out_q = xQueueCreateStatic(AUDIO_OUT_Q_DEPTH, 2 + AUDIO_TX_PCM_MAX,
                                           s_audio_out_q_storage, &s_audio_out_q_buf);
    if (!s_audio_in_q)
        s_audio_in_q = xQueueCreateStatic(AUDIO_IN_Q_DEPTH, 2 + AUDIO_RX_PCM_MAX,
                                          s_audio_in_q_storage, &s_audio_in_q_buf);
    err = udp_sess_open_lr(&s_audio, ic705_ip, audio_local_port, granted_audio_port);
    if (err == ESP_OK) {
        err = sess_handshake(&s_audio, "SID3");
        if (err == ESP_OK) {
            s_audio_ready = true;
            s_audio_bu = AUDIO_BU_NONE;
            ESP_LOGI(TAG, "SID3 (audio) ready");
        } else {
            // Don't give up: the socket is open and the radio granted the audio
            // port, it just didn't answer this pass. Keep retrying the handshake
            // from the main net loop (non-blocking) so audio comes up a beat late
            // instead of being dead for the whole session.
            s_audio_bu = AUDIO_BU_WANT_PKT4;
            s_audio_bu_last_send = 0;
            ESP_LOGI(TAG, "SID3 not ready yet — retrying audio from main loop");
        }
    } else {
        ESP_LOGW(TAG, "SID3 (audio) socket open failed (err=%s) — CAT control continues without audio", esp_err_to_name(err));
    }

    set_status("Ready");
    s_ready = true;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Background task: connect, then steady-state keepalive + CI-V tx/rx
// ---------------------------------------------------------------------------
static volatile bool s_stop_req = false;
static uint32_t s_pending_ip = 0;

static void handle_ctrl_rx(const uint8_t* r, int n) {
    if (is_pkt7(r, n)) {
        uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
        if (r[16] == 0x00) send_pkt7_reply(&s_ctrl, r + 17, seq);
        return;
    }
    if (n >= 16 && memcmp(r, "\x10\x00\x00\x00\x01\x00", 6) == 0) {
        uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
        send_pkt0_idle_at_seq(&s_ctrl, seq);
    }
}

static void handle_serial_rx(const uint8_t* r, int n) {
    if (is_pkt7(r, n)) {
        uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
        if (r[16] == 0x00) send_pkt7_reply(&s_serial, r + 17, seq);
        return;
    }
    if (n >= 16 && memcmp(r, "\x10\x00\x00\x00\x01\x00", 6) == 0) {
        uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
        send_pkt0_idle_at_seq(&s_serial, seq);
        return;
    }
    // CI-V data frames and idle pkt0s both flow through here; we only need
    // to keep the receive socket drained, the FT8 flow here is transmit-only.
}

static volatile uint32_t s_audio_rx_pkts  = 0;   // real audio DATA packets received
static volatile uint32_t s_audio_rx_bytes = 0;   // PCM bytes received
static volatile uint32_t s_audio_rx_other = 0;   // non-data (ping/idle/other) on audio sock
static volatile uint32_t s_audio_rx_drops = 0;   // RX audio packets dropped (in_q full)

// Radio sample-clock measurement. The radio sends RX audio at ITS 48kHz crystal;
// by counting samples received vs our esp_timer over CONTINUOUS RX flow (gaps >
// 60ms, e.g. during TX, are excluded) we recover the radio's true rate in OUR
// time base. TX then paces to this rate so the radio's playout buffer doesn't
// drift against us — the suspected root cause of the FT8 splatter.
static int64_t  s_rxr_last_us    = 0;
static int64_t  s_rxr_acc_us     = 0;
static uint64_t s_rxr_acc_samps  = 0;
static portMUX_TYPE s_rxr_mux    = portMUX_INITIALIZER_UNLOCKED;

double ic705_net_get_measured_rx_rate(void) {
    int64_t us; uint64_t s;
    taskENTER_CRITICAL(&s_rxr_mux);
    us = s_rxr_acc_us; s = s_rxr_acc_samps;
    taskEXIT_CRITICAL(&s_rxr_mux);
    if (us > 15000000 && s > 0)              // need >=15s of continuous RX
        return (double)s * 1000000.0 / (double)us;
    return 48000.0;                          // not enough data yet → nominal
}

static void handle_audio_rx(const uint8_t* r, int n) {
    // Audio bring-up retry: catch the radio's delayed areYouThere reply (pkt4)
    // and the areYouReady ack (pkt6) so a late-arriving audio server recovers
    // without a reconnect. These 16-byte control packets don't match the data
    // path below, so handle them here before anything else consumes them.
    if (!s_audio_ready && s_audio_bu != AUDIO_BU_NONE && n == 16) {
        if (s_audio_bu == AUDIO_BU_WANT_PKT4 &&
            memcmp(r, "\x10\x00\x00\x00\x04\x00\x00\x00", 8) == 0) {
            s_audio.remote_sid = get_be32(r + 8);
            s_audio_bu = AUDIO_BU_WANT_ACK;
            s_audio_bu_last_send = 0;  // fire pkt6 on the next loop tick
            ESP_LOGW(TAG, "SID3 retry: got pkt4 remote_sid=%08x", (unsigned)s_audio.remote_sid);
            return;
        }
        if (s_audio_bu == AUDIO_BU_WANT_ACK &&
            memcmp(r, "\x10\x00\x00\x00\x06\x00\x01\x00", 8) == 0) {
            s_audio_ready = true;
            s_audio_bu = AUDIO_BU_NONE;
            ESP_LOGW(TAG, "SID3 retry: audio READY (recovered via main-loop retry)");
            return;
        }
    }
    if (is_pkt7(r, n)) {
        uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
        if (r[16] == 0x00) send_pkt7_reply(&s_audio, r + 17, seq);
        s_audio_rx_other++;
        return;
    }
    if (n >= 16 && memcmp(r, "\x10\x00\x00\x00\x01\x00", 6) == 0) {
        uint16_t seq = (uint16_t)r[6] | ((uint16_t)r[7] << 8);
        send_pkt0_idle_at_seq(&s_audio, seq);
        s_audio_rx_other++;
        s_audio_rexmit++;
        return;
    }
    // Real audio data: per wfview, type(offset 0x04, u16 LE) != 1 and
    // len >= 0x20 (32) distinguishes it from incidental/idle traffic.
    // Payload is everything after the 24-byte header, sized by the
    // packet's ACTUAL received length — matches wfview's r.mid(0x18),
    // not a header-declared size.
    if (n >= 0x20) {
        uint16_t type = (uint16_t)r[4] | ((uint16_t)r[5] << 8);
        if (type != 0x01 && s_audio_in_q) {
            int pcm_len = n - 24;
            if (pcm_len > AUDIO_RX_PCM_MAX) pcm_len = AUDIO_RX_PCM_MAX;
            s_audio_rx_pkts++;
            s_audio_rx_bytes += (uint32_t)(pcm_len > 0 ? pcm_len : 0);
            if (pcm_len > 0) {  // accumulate the radio sample-clock measurement
                int64_t now = esp_timer_get_time();
                int samps = pcm_len / 2;     // 16-bit mono
                taskENTER_CRITICAL(&s_rxr_mux);
                if (s_rxr_last_us != 0) {
                    int64_t dt = now - s_rxr_last_us;
                    if (dt > 0 && dt < 60000) {       // <60ms => continuous RX flow
                        s_rxr_acc_us    += dt;
                        s_rxr_acc_samps += samps;
                        if (s_rxr_acc_us > 180000000) {  // keep window ~recent
                            s_rxr_acc_us    >>= 1;
                            s_rxr_acc_samps >>= 1;
                        }
                    }
                }
                s_rxr_last_us = now;
                taskEXIT_CRITICAL(&s_rxr_mux);
            }
            if (pcm_len > 0) {
                uint8_t buf[2 + AUDIO_RX_PCM_MAX];
                buf[0] = (uint8_t)(pcm_len & 0xFF);
                buf[1] = (uint8_t)((pcm_len >> 8) & 0xFF);
                memcpy(buf + 2, r + 24, pcm_len);
                if (xQueueSend(s_audio_in_q, buf, 0) != pdTRUE) s_audio_rx_drops++;  // queue full
            }
        }
    }
}

static void ic705_net_task(void*) {
    esp_err_t err = do_connect(s_pending_ip);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "connect failed: %s", esp_err_to_name(err));
        udp_sess_close(&s_ctrl);
        udp_sess_close(&s_serial);
        udp_sess_close(&s_audio);
        s_audio_ready = false;
        s_task = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    TickType_t last_idle = xTaskGetTickCount();
    TickType_t last_ping = xTaskGetTickCount();
    TickType_t last_reauth = xTaskGetTickCount();

    while (!s_stop_req) {
        // Drain the CI-V TX queue (non-blocking — must NOT block the loop;
        // audio RX below runs at ~100 packets/sec and can't wait 20ms).
        uint8_t civ_msg[1 + CIV_FRAME_MAX];
        while (xQueueReceive(s_civ_out_q, civ_msg, 0) == pdTRUE) {
            serial_send_civ_now(civ_msg + 1, civ_msg[0]);
        }

        if (s_audio_ready && s_audio_out_q) {
            uint8_t audio_msg[2 + AUDIO_TX_PCM_MAX];
            while (xQueueReceive(s_audio_out_q, audio_msg, 0) == pdTRUE) {
                uint16_t len = (uint16_t)(audio_msg[0] | (audio_msg[1] << 8));
                // One packet per pump. With STATIC WiFi TX buffers (1600B each)
                // the full ~984B frame fits a single buffer with no runtime DMA
                // alloc — so chunking (which doubled buffer demand) is reverted.
                audio_send_pcm_now(audio_msg + 2, len);
            }
        }

        // Drain ALL pending packets on each socket (non-blocking), not just
        // one — at 48kHz the radio sends ~100 audio packets/sec; reading one
        // per ~20ms loop dropped half of them and corrupted the stream.
        uint8_t rx[256];
        int n;
        while ((n = udp_recv_timeout(s_ctrl.sock, rx, sizeof(rx), 0)) > 0) handle_ctrl_rx(rx, n);
        while ((n = udp_recv_timeout(s_serial.sock, rx, sizeof(rx), 0)) > 0) handle_serial_rx(rx, n);

        if (s_audio_ready || s_audio_bu != AUDIO_BU_NONE) {
            uint8_t arx[24 + AUDIO_RX_PCM_MAX + 16];
            while ((n = udp_recv_timeout(s_audio.sock, arx, sizeof(arx), 0)) > 0) handle_audio_rx(arx, n);
        }

        // Drive the non-blocking SID3 bring-up retry: resend areYouThere (pkt4
        // phase) / areYouReady (ack phase) once per second until handle_audio_rx
        // above catches the reply and flips s_audio_ready. Keepalives on the
        // already-established ctrl/serial streams keep flowing below, so the
        // radio stays active the whole time we wait for its audio server.
        if (!s_audio_ready && s_audio_bu != AUDIO_BU_NONE) {
            TickType_t nowt = xTaskGetTickCount();
            if (s_audio_bu_last_send == 0 || nowt - s_audio_bu_last_send >= pdMS_TO_TICKS(1000)) {
                s_audio_bu_last_send = nowt;
                uint8_t pkt[16] = { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                pkt[4] = (s_audio_bu == AUDIO_BU_WANT_ACK) ? 0x06 : 0x03;
                if (s_audio_bu == AUDIO_BU_WANT_ACK) pkt[6] = 0x01;
                put_be32(pkt + 8, s_audio.local_sid);
                put_be32(pkt + 12, s_audio.remote_sid);
                udp_send(&s_audio, pkt, sizeof(pkt));
                udp_send(&s_audio, pkt, sizeof(pkt));
            }
        }

        // pkt0 idle is the radio's primary liveness signal — the reference
        // implementation sends these every 100ms while active, far faster
        // than the pkt7 ping below, and without it the radio drops the
        // session after a couple of seconds of perceived silence.
        TickType_t now = xTaskGetTickCount();
        if (now - last_idle >= pdMS_TO_TICKS(100)) {
            send_pkt0_idle(&s_ctrl);
            send_pkt0_idle(&s_serial);
            // Audio-stream pkt0 idle keepalive: the IC-705 needs it to keep RX
            // audio flowing SMOOTHLY (without it we were losing ~2% of RX audio →
            // gappy decode window → candidates found but 0 decoded). BUT during
            // TX, interleaving idle frames into the audio stream splatters the
            // signal — so send it ONLY when we're NOT actively transmitting
            // (no audio TX packet in the last 150ms). Best of both: smooth RX,
            // clean TX. (kappanhang skips it entirely; the IC-705 differs.)
            if (s_audio_ready &&
                (esp_timer_get_time() - s_last_audio_tx_us) > 150000) {
                send_pkt0_idle(&s_audio);
            }
            last_idle = now;
        }
        if (now - last_ping >= pdMS_TO_TICKS(3000)) {
            send_pkt7_ping(&s_ctrl, 2);
            send_pkt7_ping(&s_serial, 1);
            if (s_audio_ready) send_pkt7_ping(&s_audio, 3);
            last_ping = now;
        }
        if (now - last_reauth >= pdMS_TO_TICKS(60000)) {
            uint8_t auth05[64];
            build_auth_pkt(auth05, 0x05);
            udp_send_tracked_x2(&s_ctrl, auth05, sizeof(auth05));
            last_reauth = now;
        }
        // DIAGNOSTIC: RX audio health every 5s — measured received rate (Hz; a
        // clean stream reads ~48000), queue drops, and packet count. Tells us if
        // we're losing RX audio (and where: <48000 rate or nonzero drops).
        static TickType_t last_rxh = 0;
        if (now - last_rxh >= pdMS_TO_TICKS(5000)) {
            last_rxh = now;
            ESP_LOGW(TAG, "RXHEALTH rate=%.1fHz pkts=%u drops=%u",
                     ic705_net_get_measured_rx_rate(),
                     (unsigned)s_audio_rx_pkts, (unsigned)s_audio_rx_drops);
        }

        // Pace the loop and yield CPU. The sockets above are non-blocking, so
        // without a real delay this task busy-spins. CRITICAL: the FreeRTOS
        // tick is 100Hz, so pdMS_TO_TICKS(2) rounds to 0 ticks — a vTaskDelay(0)
        // only yields to equal/higher-priority tasks and NEVER lets the
        // priority-0 IDLE task run, which (a) tripped the task watchdog on
        // IDLE1 every ~5s and (b) starved the WiFi/lwIP RX path so the
        // high-rate audio stream was dropped while low-rate keepalives still
        // got through. Use 1 full tick (10ms): IDLE and lwIP get CPU, and the
        // drain-all loops above still clear every queued packet per wake.
        vTaskDelay(1);
    }

    // Graceful release: close the CI-V data stream, then send the protocol
    // disconnect (type 0x05) on every stream so the radio frees the session
    // right away. Brief delay to let the UDP frames actually leave before we
    // tear the sockets down.
    ESP_LOGI(TAG, "Disconnecting: token-remove + per-stream 0x05, then close");
    // The IC-705 connection is token-based — the control 0x05 packet alone does
    // NOT free it (confirmed: radio's WLAN icon stayed lit). We must REMOVE the
    // token: build_auth_pkt magic 0x01 (vs 0x02=request at login, 0x05=renew).
    // Send it on the control stream first, give the radio a moment, then the
    // per-stream 0x05 disconnects, then close.
    {
        uint8_t tok_remove[64];
        build_auth_pkt(tok_remove, 0x01);
        udp_send_tracked(&s_ctrl, tok_remove, sizeof(tok_remove));
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    serial_send_open_close(true);
    if (s_audio_ready) send_disconnect(&s_audio);
    send_disconnect(&s_serial);
    send_disconnect(&s_ctrl);
    vTaskDelay(pdMS_TO_TICKS(60));

    if (s_audio_ready) udp_sess_close(&s_audio);
    udp_sess_close(&s_serial);
    udp_sess_close(&s_ctrl);
    s_ready = false;
    s_audio_ready = false;
    set_status("Idle");
    s_task = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t ic705_net_connect(uint32_t ic705_ip) {
    if (s_task) return ESP_ERR_INVALID_STATE;  // a connect is already in progress
    ESP_LOGI(TAG, "Connecting: starting CI-V network login");
    if (!s_civ_out_q) s_civ_out_q = xQueueCreate(8, 1 + CIV_FRAME_MAX);
    s_pending_ip = ic705_ip;
    s_stop_req = false;
    s_ready = false;
    s_audio_ready = false;
    s_audio_bu = AUDIO_BU_NONE;
    s_audio_bu_last_send = 0;
    s_auth_inner_seq = 0;
    s_serial_send_seq_inner = 0;
    // Generous stack: do_connect() nests calls (sess_handshake etc.) that
    // each carry their own 1500-byte recv buffer live on the stack at the
    // same time, on top of lwIP's own per-call socket overhead.
    //
    // Priority 6, pinned to core 1: app_task_core0 (priority 5) and the
    // audio TX task both live on core 0, so an unpinned/lower-priority task
    // here could be starved of any CPU time at all by either of them —
    // indistinguishable from a hang. The audio RX task also runs on core 1
    // at priority 4, so this still outranks it there too.
    xTaskCreatePinnedToCore(ic705_net_task, "ic705_net", 16384, nullptr, 6, &s_task, 1);
    return ESP_OK;
}

void ic705_net_disconnect(void) {
    s_stop_req = true;
}
