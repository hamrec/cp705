# CP705 — A Self-Contained IC-705 WiFi FT8 Client

> Built on **[Mini-FT8 by Wei, AG6AQ](https://github.com/wcheng95/Mini-FT8)** — with
> deep gratitude. Please visit and star the original project.

CP705 is a port by **Dean (KD3AN)** of Wei (AG6AQ)'s
**[Mini-FT8](https://github.com/wcheng95/Mini-FT8)**, retargeting it to drive an
Icom IC-705 entirely over WiFi. It stands on the shoulders of that work and the
lineage behind it — Karlis Goba's [ft8_lib](https://github.com/kgoba/ft8_lib), the
audio/DSP and autoseq foundation from Zhenxing (N6HAN), and the inspiration of the
DX-FT8 team. **All credit for the original application belongs to them**; this fork
only adapts it to a different radio and link.

Where Mini-FT8 drives QMX/QDX/KH1 radios over a serial/USB-audio path, **CP705
has a completely different aim**: it turns the Cardputer ADV into a *standalone
wireless FT8 station for the Icom IC-705* — CAT control, receive-audio decode,
and transmit, all over WiFi, with **no PC, no soundcard, and no audio cables**.
The IC-705's own WLAN server is the only link. In doing so it pushes the
ESP32-S3 Cardputer ADV (which has **no PSRAM**) well past what the platform was
expected to do.

## Install

**One-click (recommended):** open the web flasher in **Chrome or Edge** on a
desktop, plug in the Cardputer ADV via USB, and click Install — no toolchain
required:

> ### → [Install CP705 in your browser](https://hamrec.github.io/cp705/)

**Manual:** download `CP705_Merged_Auto.bin` from the
[latest release](https://github.com/hamrec/cp705/releases/latest) and flash it at
offset `0x0`:

```sh
python -m esptool --chip esp32s3 write_flash 0x0 CP705_Merged_Auto.bin
```

After flashing, set up your station on the device: press `M` to open the
settings menu, then `1` for **Station** (call/grid) and `3` for **IC-705/Network**
(WiFi + login). Settings persist in NVS. See **IC-705 Setup** below for the
full radio-side configuration.

## Challenges overcome to make FT8 work with the IC-705 over WiFi

- **Implementing the Icom WLAN remote protocol on a microcontroller** — the same
  control / CI-V-serial / audio UDP streams used by RS-BA1, wfview, and
  kappanhang: SID handshakes, login + authentication, tracked sequence numbers,
  and pkt7 keepalives, all on a tiny embedded stack.
- **The login token window** — a few hundred milliseconds of delay before the
  login packet silently poisoned the radio's token and broke the handshake; the
  connect sequence had to be made delay-free.
- **No PSRAM (~512 KB SRAM total)** — fitting the entire FT8 decode pipeline
  *plus* the WiFi stack *plus* live audio streaming into internal RAM, including
  trimming decode oversampling and converting buffers to static allocation.
- **Smooth receive audio over WiFi** — a dynamically-allocated audio queue was
  silently failing to allocate and dropping every sample; moving to a static
  queue, plus a watchdog-safe yield, restored continuous RX.
- **Keeping the 15-second FT8 window aligned without a PC** — locking timing to
  GPS UTC and re-anchoring the decode window after each synchronous decode so
  decodes stopped drifting out of the slot.
- **Clean, constant-envelope transmit over WiFi** — pacing TX with a hardware
  timer matched to the radio's *measured* sample clock to eliminate buffer
  drift, and gating the protocol's idle keepalive (kept flowing during RX for
  smooth audio, suppressed during TX) to stop the carrier from pumping and
  splattering.
- **Transmit under memory pressure** — WiFi `send()` buffer exhaustion on the
  no-PSRAM board was dropping TX audio; static TX buffers and disabling A-MPDU
  TX aggregation got delivery clean.
- **CI-V quirks** — fixing an unintended filter clobber and adding handshake
  retries so CAT control comes up reliably.
- **Reliable config + log storage on a board with no internal FATFS partition** —
  station settings and the ADIF QSO log are stored in **NVS** (non-volatile
  flash key/value store), which survives power-off and never depends on the SD
  card at all. The log can also be exported to the SD card on demand, byte-verified
  after writing.
- **SD card writes silently corrupted by the LoRa-1262 cap** — the cap's SPI bus
  (MOSI/MISO/SCK) is identical to the SD card's, and its chip-select was left
  floating, letting the SX1262 radio interfere with SD write commands even
  though reads worked fine — reproduced identically across multiple different,
  known-good cards, so it looked exactly like a bad card or a driver bug until
  the shared bus was the actual culprit. Fixed by parking the cap's CS (`G5`)
  high before every SD access. See **Logging and Download** below.

## New features and menu changes (vs. Mini-FT8)

- **IC-705 over WiFi** as a first-class radio target — CAT, RX decode, and TX
  all run over the radio's WLAN connection instead of a serial/USB-audio path.
- **No external audio hardware** — the soundcard/USB-C-audio-adapter and audio
  cabling that other radios require are gone; the WiFi audio stream replaces them.
- **NVS-backed persistence** — callsign, grid, WiFi/network credentials, CI-V
  address, bands, and the ADIF QSO log all persist in NVS across reboots (no
  dependence on the SD card or an internal FATFS partition).
- **Categorized settings menu** — `M` opens a category picker (Station,
  Operating, IC-705/Network, Logging, System) instead of a flat run of
  numbered pages. Band frequency editing, GPS telemetry, the performance
  monitor, and display brightness all live inside this menu too (Station,
  System, and Logging) — there's no separate top-level hotkey for any of them.
- **Display brightness** (System category → `5`) — 10 steps from dim to full,
  persisted across reboots.
- **On-device network login editor** — WiFi SSID/password and IC-705 network
  user/password are editable under **IC-705/Network**.
- **Editable CQ prompt with activity markers (POTA/SOTA/QRP/…)** — pressing `C`
  opens the CQ message pre-filled with `CQ <call> <grid>`, cursor placed right
  after `CQ ` so an activity marker can be typed immediately. Type `POTA ` and
  you send `CQ POTA KD3AN EM66`; the same works for `SOTA`, `QRP`, `DX`, `TEST`,
  or a 3-digit contest number. Confirming sends it and automatically re-calls the
  same CQ every idle cycle (no separate Beacon toggle — calling CQ *is* the
  beacon) until you answer someone, press ESC, or start a fresh `C` for a new
  default.

  - **How it's sent:** the message is packed by the standard FT8 encoder
    (`ftx_message_encode`), so `CQ POTA <call> <grid>` goes out as a proper
    *structured* FT8 CQ — the same format WSJT-X uses — and decodes cleanly as a
    POTA CQ on the other end, not as free text.
  - **Marker length limit (an FT8 protocol rule, not a CP705 one):** the marker
    slot after `CQ` holds **1–4 characters**. POTA, SOTA, QRP, DX, TEST and 3-digit
    contest numbers all fit. A longer term can't pack as a structured CQ and would
    fall back to FT8's 13-character free-text limit, which `CQ <term> <call> <grid>`
    won't fit into. CP705 does not transmit garbage in that case: the encoder
    returns an error, TX is cleanly aborted, and an `Encode failed for TX` line is
    logged. So a valid 1–4 char marker sends correctly; an over-long one simply
    doesn't transmit.
- **Persistent, recency-sorted decode list** — decoded lines no longer blank
  out every 15s cycle. Anything addressed to you stays pinned at the top no
  matter how old it is; everything else is sorted newest-heard-first, and a
  station repeating (or advancing to its next message) refreshes its existing
  line in place instead of spawning a duplicate.
- **Auto-contrast waterfall** — each frame is auto-scaled and gamma-corrected
  against its own noise floor, then rendered on a black→dark-blue→light-blue
  ramp so quiet band segments stay genuinely black and only real signals light
  up, closer to the IC-705's own spectrum scope.
- **Icom-styled hero card** — the moment a CQ or QSO is active, the decode list
  is replaced by a full-screen card: callsign/grid, a 6-stage exchange
  tracker, the current TX line, and frequency/SNR. A completed QSO turns the
  card fully green ("QSO COMPLETE — Logged") and an exchange that runs out of
  retries with no reply shows an amber "NO REPLY" instead — both auto-return
  to CQ or the decode list after a few seconds, and `` ` `` bails out of the
  QSO at will at any time.
- **End+Export Log SD** (Logging category → `1`) — writes the accumulated NVS
  ADIF log to the card as a single verified, date-named `YYYYMMDD.adi` file for
  import into logging software (see **Logging and Download** below). This is the
  only file CP705 puts on the card — QSOs are not written to SD as they happen.
- **Clear QSO log** (Logging category → `2`) — a two-press-confirm action to
  wipe the NVS log and start fresh, e.g. between POTA activations.
- **POTA / SOTA activation logging** (Logging category → `4` toggle, `5` ref) —
  pick the program (POTA or SOTA) and enter your reference (`US-1234` for a park,
  `W7A/MN-001` for a summit); every QSO logged from then on gets the program's
  ADIF fields — `MY_SIG:POTA` + `MY_SIG_INFO:<ref>` for POTA, or `MY_SOTA_REF:<ref>`
  for SOTA — so the export imports straight into the POTA/SOTA site. The
  Activation row also shows a live progress counter toward the validity
  threshold (`POTA 7/10`, `SOTA 3/4`, then `… OK`). The ref is **session-scoped**:
  RAM only, never saved to `Station.txt`/NVS, **clears on reboot** — one
  activation per outing, so it can't silently tag a later casual session. Blank
  the ref to turn it off. Records also carry a `BAND` field (e.g. `20m`) and a
  QSO-**start** `TIME_ON`, both of which the POTA/SOTA specs expect. See
  **POTA / SOTA Activations** below for the full walkthrough.
- **Streamlined to the IC-705 target** — the entire KH1 radio backend was
  removed to keep the build focused on the wireless IC-705 use case, and
  several settings inherited from Mini-FT8 (ignore list, QSO comment template,
  RX/TX text log, PORTA GPS wiring) were dropped as dead weight on this board.
- **Fewer top-level keys** — the manual TX-queue view is gone (autoseq manages
  the queue on its own; the hero card and ESC cover everything you actually
  need to see or drop), and the Mass Storage toggle is gone too (it needed a
  FATFS partition this board doesn't have, so it was always inert here).

> CP705 is an experimental, boundary-pushing build. Huge thanks again to the
> Mini-FT8 authors — this project exists only because of the foundation they
> shared with the community.

# IC-705 Setup

CP705 talks to the radio entirely over the IC-705's built-in WLAN remote-control
server — the same protocol RS-BA1, wfview, and kappanhang use. There is **no
cable, no soundcard, and no PC**. The Cardputer joins the radio's WiFi, logs in
with a registered network user, and then carries CAT, receive audio, and
transmit audio over three UDP ports.

## How it connects (at a glance)

```text
┌─────────────────────┐   WiFi (IC-705 is the Access Point)   ┌──────────────────┐
│ IC-705              │ SSID broadcast, radio = 192.168.59.1  │ Cardputer ADV    │
│  WLAN AP + remote   │<─────────────────────────────────────>│  CP705 (client)  │
│  server             │   UDP 50001 control  (login/CI-V auth)│                  │
│                     │   UDP 50002 serial   (CI-V frames)    │                  │
│                     │   UDP 50003 audio    (RX + TX PCM)    │                  │
└─────────────────────┘                                       └──────────────────┘
```

- The **radio runs as the WiFi Access Point** (Connection Type = Access Point).
  In that mode the IC-705 is reachable at the fixed address **`192.168.59.1`**,
  which CP705 targets directly (no mDNS lookup needed).
- The Cardputer joins as a normal WiFi client and is handed an address by the
  radio's DHCP server.

## Step 1 — Configure the IC-705

> Exact menu wording varies slightly by firmware version. The required end state
> is: WLAN access point up, network control on, a registered **administrator**
> user, the three default UDP ports, and the **modulation input set to WLAN**.

**MENU → SET → WLAN Set**
- `WLAN`: **ON**
- `Connection Type`: **Access Point**
- `SSID`: choose a name (this is what you'll enter in CP705)
- `Password`: choose a WPA2 password (8–63 chars)
- `DHCP Server`: **ON** (radio becomes `192.168.59.1` and assigns the Cardputer an IP)

**MENU → SET → Network**
- `Network Control`: **ON**
- `Network User1`: set an **ID** and **Password**, and `Administrator`: **YES**
  (CP705 logs in as this user — the names must match what you put in CP705)
- `Control Port (UDP)`: **50001**
- `Serial Port (UDP)`: **50002**
- `Audio Port (UDP)`: **50003**
- (Leave these at the IC-705 defaults — CP705 expects exactly these ports.)

**MENU → SET → Connectors**
- `MOD Input` → `DATA MOD`: **WLAN** (so network audio drives transmit in data modes)
- Optional: adjust `MOD Input` → WLAN level if your TX drive needs trimming.

**Operating mode**
- Use a **data mode** for FT8/FT4 — i.e. **USB-D** (the radio's "DATA" sub-mode).
  CP705 sets frequency/mode over CAT, but the radio must be in a data mode for
  the WLAN modulation path to key the transmitter cleanly.

## Step 2 — Configure CP705

All connection settings live in the **IC-705/Network** settings category:
press **`M`** for the category picker, then **`3`**.

| Key | Setting | Notes |
|---|---|---|
| `1` | WiFi SSID | Must match the radio's Access-Point SSID from Step 1. |
| `2` | WiFi Password | The radio's Access-Point password. |
| `3` | Net User | Must match the radio's `Network User1` **ID**. |
| `4` | Net Password | Must match the radio's `Network User1` **password**. |
| `5` | CI-V Address | The IC-705 default is `0xA4`. |
| `6` | Re-resolve / Connect status | Re-points CP705 at `192.168.59.1`; the row shows live WiFi status. |

Each field is an in-place edit: type the value and press **Enter** to save
(`` ` `` cancels). Passwords are masked with `*` when not being edited.

> **Network login must match the radio:** `Net User` / `Net Password` here have to
> equal the **Network User1 ID/Password** you set on the radio in Step 1, and that
> user must be an **Administrator**.

All settings are written to `Station.txt` on the internal flash the moment you
save them, so they persist across reboots — enter your callsign, grid, WiFi, and
network login once. You can also pre-load `Station.txt` from the SD card.

## Step 3 — Connect and operate

1. Power up the radio with WLAN on; confirm it is broadcasting its AP SSID.
2. On CP705, press **`S`** (STATUS) then **`2`** to connect. The Cardputer joins
   the WiFi, logs in, opens the CI-V and audio streams, and starts decoding.
   Watch the WiFi status line in the **IC-705/Network** category for progress.
3. Pick a band from **`S` → `3`** (steps through your active bands) and let the
   waterfall fill; decodes appear in **`R`** (RX) as a numbered list of CQ
   callers. Tap a number to answer, or press **`C`** to call CQ yourself — the
   screen switches to a full-screen QSO card while the exchange is in progress.
4. (Recommended) Plug in a GPS or DS3231 so the 15-second FT8 window is locked to
   UTC — see **GPS Connections** / **DS3231 RTC Connections** below. Accurate
   time is required for reliable decodes and properly-timed transmit.
5. Logging is automatic: each completed QSO is appended to the ADIF log in **NVS**
   (survives power-off). To get an importable `.adi`, export to the card from the
   **Logging** category, item `1` — this writes one date-named `YYYYMMDD.adi`
   (see **Logging and Download**). The Logging category's Clear-Log row also
   shows a running QSO count (`Clear QSO Log: N`).

## Quick reference

| Item | Value |
|---|---|
| Radio WiFi mode | Access Point |
| Radio IP | `192.168.59.1` |
| Control / Serial / Audio ports | `50001` / `50002` / `50003` (UDP) |
| Audio format | 48 kHz, 16-bit, mono (LPCM) |
| IC-705 CI-V address | `0xA4` (default) |
| FT8 operating mode | USB-D (data) |
| QSO log (primary) | NVS (ADIF); export to SD via Logging category → `1` |

## Troubleshooting

- **WiFi won't connect:** confirm the SSID/password in CP705 match the radio's
  Access-Point SSID/password exactly, and that `WLAN` + `DHCP Server` are ON.
- **WiFi connects but no decodes / no CAT:** check `Network Control` is ON, the
  `Network User1` is an **Administrator**, and the three UDP ports are at their
  defaults (50001/50002/50003). Re-resolve from the **IC-705/Network** category,
  item `6`, or reconnect with `S → 1`.
- **Login rejected:** the CP705 network user/password must match `Network User1`
  on the radio. Only one client can use the radio's remote server at a time, so
  make sure wfview/RS-BA1/SDR-Control isn't already connected.
- **Transmits but no RF / no modulation:** set `MOD Input → DATA MOD = WLAN` and
  operate in a **data mode (USB-D)**.
- **No decodes despite strong signals:** time isn't UTC-locked — connect a GPS or
  DS3231 and confirm the time source shows `G` (GPS) or `R` (RTC) on `S → 5`.
- **Brief TX audio dropouts (radio's power output dips to zero, then resumes):**
  RF getting into the ESP32's WiFi front-end from a nearby transmitting
  antenna, confirmed by per-transmission diagnostics (`TXDONE` lines in
  `IC705DBG.TXT` on the SD card) and cleared by physically repositioning the
  radio/antenna relative to the Cardputer. Give them more separation; a USB
  tether cable can also act as an antenna and make it worse.

## Decode sensitivity — a hardware limit, not a bug

CP705 runs the FT8 decoder at **time oversampling ratio 1 (`time_osr=1`)**, and
this is the ceiling on the Cardputer ADV. You will see fewer decodes than a
PC-class client (WSJT-X, SDR-Control) hears on the same band at the same time.
That is expected, and here is the honest reason:

- **No PSRAM.** The Cardputer ADV has no external RAM. The FT8 decoder's
  waterfall magnitude buffer is the single largest working allocation, and at
  `time_osr=2` (the sensitivity level a desktop uses) it needs ~40 KB more than
  at `time_osr=1` — roughly the board's entire steady-state free-heap margin.
  Measured: with `time_osr=1` the firmware runs with ~41 KB free heap under full
  RX/TX load; `time_osr=2` drops that to ~1 KB and the board panics on the first
  transient allocation the moment decode spins up. There is no framebuffer or
  other large buffer left to reclaim to make room (the UI draws directly to the
  panel — it holds no canvas in RAM), and the WiFi stack is already trimmed to
  the bone. So `time_osr=1` is genuinely the best this hardware can do.
- **Lighter decoder than WSJT-X.** The DSP core is
  [ft8_lib](https://github.com/kgoba/ft8_lib), a clean C implementation. It does
  a single decode pass; WSJT-X's Fortran decoder does multi-pass decoding with
  signal subtraction and a-priori decoding, which pulls out more (and weaker)
  signals — but assumes desktop CPU and RAM.
- **The radio's waterfall is not a decoder.** The IC-705's on-screen waterfall
  shows *RF energy* — including signals too weak or off-timing to decode, plus
  non-FT8 activity — so it will always look busier than any decode list, on any
  hardware. A crowded radio waterfall is not by itself evidence of missed
  decodes.

Full-sensitivity FT8 (`time_osr=2`, multi-pass) is a **PSRAM-hardware feature**
— see the T-Deck Plus (TD705) or an M5Stack CoreS3 port. CP705's strengths are
its self-contained WiFi operation, connection resilience, and TX stability, not
raw weak-signal decode count.

## Known Issues

Open issues, all real and all chased hard without a confirmed root cause yet:

- **Cold-boot Tune sometimes fires no real RF.** If the radio and CP705 are
  both freshly rebooted on the same band and you press `3` to Tune, CP705's
  display shows the tune running but the radio's SWR reads infinity — no
  actual carrier goes out. Switching bands and back (or tuning on a different
  band first) reliably clears it. A theory involving the WLAN audio session
  not being fully ready yet was tested twice (fail fast, then wait up to
  1.5s) and disproved — waiting longer doesn't help, so that fix was reverted
  rather than shipping dead code. Root cause unknown.
- **A QSO's TX can occasionally stall out with no further activity.** Once,
  after a retry counter exhausted, the expected give-up transmission never
  fired and the context just sat there — no crash, no further log lines, no
  RF. A separate QSO with the same retry-exhaustion path resolved cleanly
  minutes later, so the underlying retry/give-up logic is not universally
  broken — this looks like a genuine, intermittent edge case. Diagnostic
  logging is now in place (an empty-TX-buffer warning and a throttled dump of
  every TX-trigger guard condition) to capture the exact state next time it
  happens, but no fix exists yet.

---

# CP705 Operation Manual

> This manual is inherited from Mini-FT8 and describes the shared on-device UI.

## Printable Pocket Card

A two-panel cheat sheet to keep with the device. **Print this section** (a
monospace/fixed-width font prints the borders cleanly), cut out both panels, and
fold or tape them back-to-back — each panel is sized to tuck into a badge holder
or sit next to the Cardputer. Full details for everything on it are in the
sections below.

```text
+------------------------------------------------+
|        CP705  -  POCKET OPERATING CARD         |
+------------------------------------------------+
|                                                |
| GET ON THE AIR                                 |
|   Radio: AP mode, WLAN + DHCP Server = ON      |
|   CP705 auto-joins the radio's WiFi            |
|   Press  S 1   to connect + sync               |
|   Ready when the radio shows its WLAN icon     |
|                                                |
| KEYS                                           |
|   R   decode list / live QSO card              |
|   C   call CQ (opens editable text)            |
|   S   STATUS: connect, tune, gain, time        |
|   M   MENU: settings categories                |
|   `   ESC - bail QSO / cancel edit             |
|   1-6 pick the visible row                     |
|   Up/Dn page    Lt/Rt move in date-time        |
|                                                |
| WORK A STATION                                 |
|   Answer:  in R, press its line #  1-6         |
|   Call CQ: C, type a marker, Enter             |
|     e.g.  CQ POTA <call> <grid>                |
|     marker = 1-4 ch: POTA SOTA QRP DX          |
|   Drop a QSO:  `  (backtick)                   |
|                                                |
| STATUS (S) KEYS                                |
|   1 connect/sync     2 next band               |
|   3 tune ~5s         4 date    5 time          |
|   6 disconnect      +/- TX drive               |
+------------------------------------------------+

+------------------------------------------------+
|         CP705  -  RADIO SETUP & FIXES          |
+------------------------------------------------+
|                                                |
| IC-705 SETUP  (once)                           |
|   WiFi mode: Access Point                      |
|   WLAN + DHCP Server = ON                      |
|   Network Control = ON                         |
|   Network User1 = Administrator                |
|   Operating mode: USB-D (data)                 |
|   MOD Input:  DATA MOD = WLAN                  |
|   IP 192.168.59.1     CI-V 0xA4                |
|   UDP ports 50001 / 50002 / 50003              |
|                                                |
| TIME  (required for decodes)                   |
|   Connect a GPS or DS3231 RTC                  |
|   S 5 shows source:  G=GPS   R=RTC             |
|                                                |
| LOGGING / POTA / SOTA                          |
|   Auto-saved to NVS (survives power-off)       |
|   Export ADIF:  M > Logging > 1                |
|   Program POTA/SOTA:  M > Logging > 4          |
|   Ref (US-1234 / W7A/MN-001): >5               |
|   Counter shows N/10 POTA, N/4 SOTA            |
|   (activation clears on reboot)                |
|                                                |
| QUICK FIXES                                    |
|   TX but no RF -> DATA MOD=WLAN, USB-D         |
|   No decodes   -> fix UTC time (G/R)           |
|   No CAT/audio -> Network Control ON;          |
|                   re-resolve  M > Net > 6      |
|   TX drops out -> separate antenna and         |
|                   USB cable from device        |
|   Cold tune no RF -> switch band & back        |
+------------------------------------------------+
```

## Quick Mode Map

| Key | Mode | Purpose |
|---|---|---|
| `R` | RX | Idle: numbered list of decoded/CQ traffic. Active: full-screen QSO hero card. |
| `C` | Call CQ | Opens an editable CQ prompt (from RX); confirming sends it. |
| `` ` `` | Bail | From the hero card, drops the in-progress QSO and returns to the decode list. |
| `S` | STATUS | Access connect/sync, band step, tune, gain, and date/time functions. |
| `M` | MENU | Opens the settings category picker: Station, Operating, IC-705/Network, Logging, System. |

BAND editing, GPS telemetry, and the performance monitor don't have standalone
hotkeys — they're actions inside the `M` menu (see the table below) and
`` ` `` returns you to the category you launched them from.

## Global Keys and Navigation

- `R` / `S`: switch to the selected mode. Press the same mode key again to return to `RX`.
- `M`: open the settings category picker; press again from the picker to return to `RX`, or from inside a category to go back up to the picker.
- `C`: from `RX`, opens the editable CQ prompt.
- `` ` ``: in `RX`, bails out of the active QSO/CQ (hero card); in a settings category, goes back to the picker; in BAND/GPS/PERF (all reached via `M`), returns to the category that launched it; cancels TX globally in `RX`/`STATUS` when not editing; cancels an in-progress text edit elsewhere.
- `▲` / `▼`: page up / page down in `RX` and `BAND`. Settings categories each fit on one screen, so there's no in-category paging.
- `◀` / `▶`: move left / right in `STATUS` date/time.
- `1`..`6`: always select the currently visible row in the active mode.

## Per-Mode Controls

- ` acts as ESC where applicable (no physical Esc key on this keyboard).
- Text Edit: Backspace deletes, ` cancels, Enter saves.

| Mode | Item | Notes |
|---|---|---|
| `R` (RX, idle) | `1..6` | Select a decoded line to reply to. The list persists across cycles instead of resetting every 15s: messages addressed to you are pinned at the top regardless of age, everything else is newest-heard-first, and a station repeating (or advancing to its next message) refreshes its existing line in place rather than adding a duplicate. |
|  | `C` | Open the CQ prompt, pre-filled `CQ <call> <grid>`, cursor right after `CQ ` so you can type a 1–4 char activity marker (`POTA`, `SOTA`, `QRP`, `DX`, `TEST`, 3-digit contest #) to send e.g. `CQ POTA <call> <grid>`. Enter sends it, `` ` `` cancels with no TX. See the "Editable CQ prompt with activity markers" feature note for the marker-length rule. |
|  | `▲` `▼` | Page up/down is available when line 1 or line 6 is cyan. |
| `R` (RX, hero card) |  | Shows the worked station's callsign/grid, a 6-stage exchange tracker (CQ/GRID/RPT/R/RR73/73), the current TX line, frequency/SNR, and a running QSO count bottom-right. Appears automatically the moment a CQ or QSO is active. On a genuine sign-off, it goes fully green ("QSO COMPLETE — Logged") for a few seconds before auto-continuing; if an exchange stalls out and retries run out with no reply ever heard, it shows an amber "NO REPLY" instead (tracker frozen at whatever stage it actually reached) on the same auto-continue timing — either way you're never stuck waiting on ESC. |
|  | `` ` `` | Bail out of the QSO and return to the decode list immediately, any time. |
| `S` (STATUS) | `1` | Run connect/sync now; starts audio and follows the CAT sync path. Press again after `6` to reconnect. |
|  | `2` | Step to the next active band, high-to-low. Applies after key 1 is pressed or when leaving STATUS. |
|  | `3` | Brief automatic tune burst (~5s), not a toggle — keys PTT and streams a test tone, then auto-stops. Press again mid-burst to stop early. |
|  | `4` | Edit Date (in place). On the Time line, `G` means GPS time and `R` means DS3231 RTC time. |
|  | `5` | Edit Time (in place). |
|  | `6` | Gracefully disconnect the IC-705 (releases the radio's session immediately instead of waiting out a timeout — lets you flash/power-cycle the Cardputer without rebooting the radio). Press `1` to reconnect. |
|  | `+` / `-` | Adjust live TX drive level (gain), saved per-band. Also works from the hero card in RX mode, not just here. |
| `M` → **Station** | `1` | Edit Call (in place). |
|  | `2` | Edit Grid (in place). Supports 4/6/8-character grid. If GPS is available, the GPS grid is shown and used, but not saved. |
|  | `3` | Edit active bands (Long Edit) — the ordered list cycled by STATUS `2`. |
|  | `4` | Edit Band Freqs — a paged list of all 12 band slots; `1..6` picks a slot to edit in place, `▲`/`▼` pages, `` ` `` returns to this category. |
|  | (display) | Current UTC time + source (`G`=GPS, `R`=DS3231 RTC, blank=other). |
| `M` → **Operating** | `1` | Select offset source: Random / RX / Fixed. Random values are within 1500-2000 Hz. |
|  | `2` | Edit fixed cursor offset (in place). Enter directly or use `▲` `▼` `◀` `▶`. |
|  | `3` | Turn SkipTX1 on/off. Skips `dxcall mycall mygrid` and replies with the SNR report directly — useful for contest/pileup speed; leave off for normal grid-exchange QSOs. |
|  | `4` | Edit max retry (in place). Accepts any natural number or `0`. |
|  | `5` | Select FT8 / FT4 protocol. Reboot to apply the change. |
| `M` → **IC-705/Network** | `1` | Edit WiFi SSID (in place). |
|  | `2` | Edit WiFi Password (in place, masked). |
|  | `3` | Edit Net User (in place). |
|  | `4` | Edit Net Password (in place, masked). |
|  | `5` | Edit CI-V Address (in place). Accepts decimal or `0xNN` hex. |
| `M` → **Logging** | `1` | Export Log to SD. Writes the NVS ADIF log to a unique file, then reads it back and byte-verifies it. Feedback: `Verified N QSOs` / `SD write failed` / `Verify FAILED` / `No log yet`. |
|  | `2` | Clear QSO Log: press once to arm (`Press 2 again: confirm`), press `2` again within 3 s to wipe the NVS log and reset the QSO count. Any other key or letting it lapse cancels with no change. Export first if you want a copy — this never touches the SD card. The row shows the running count (`Clear QSO Log: N`) when not armed. |
|  | `3` | Performance Monitor — CPU/heap diagnostics. `` ` `` returns to this category. |
|  | `4` | Activation program — toggles `POTA ⇄ SOTA` (which ADIF fields a logged QSO gets). When a ref is set, the row also shows progress toward the validity threshold: `Activation: POTA 7/10` (POTA needs 10, SOTA needs 4), then `… OK` once reached. |
|  | `5` | Activation Ref (in place). Type the reference you're activating — `US-1234` (POTA park) or `W7A/MN-001` (SOTA summit) — Enter to arm. Every QSO logged after that carries the program's ADIF fields (`MY_SIG`/`MY_SIG_INFO` for POTA, `MY_SOTA_REF` for SOTA). Blank it and Enter to turn off. Uppercased on input. Row shows `Ref: <ref>` when armed or `Ref: (off)`. Session-scoped — not saved, clears on reboot. Changing to a new ref resets the progress counter. |
| `M` → **System** | (display) | Sleep/Batt %. |
|  | `2` | Enter deep sleep now. |
|  | `3` | GPS Status — live telemetry: 3D fix, satellites, UTC time, grid square, and last synchronization age (LoRa-1262 cap GNSS is the only source). `` ` `` returns to this category. |
|  | `4` | Re-resolve / reconnect. Re-points CP705 at the radio's IP even if WiFi is already up (STATUS `2` skips this when already connected) — the row shows live WiFi status. |
|  | `5` | Display brightness: cycles 1-10 (10%-100%), wraps back to 1 after 10. Persists across reboots. |

## POTA / SOTA Activations

CP705 can tag your logged QSOs for a **Parks on the Air (POTA)** or **Summits on
the Air (SOTA)** activation, so the exported `.adi` imports straight into the
POTA or SOTA database with no hand-editing.

### Configure it (Logging category)

Press `M` → `4` (**Logging**), then:

1. **`4` — Activation program.** Toggles between `POTA` and `SOTA`. This chooses
   *which* ADIF fields your QSOs get (they differ per program — see below).
2. **`5` — Activation Ref.** Type the reference you are activating and press Enter
   to arm it:
   - **POTA** park reference, e.g. `US-1234` (older logs use `K-1234`).
   - **SOTA** summit reference, e.g. `W7A/MN-001`.
   - Input is uppercased automatically. **Blank the field and Enter to turn it
     off.**

The Activation row shows the live state at a glance: `Activation: POTA 7/10` (or
`Activation: SOTA 3/4`), with the count climbing as you log QSOs and an `OK`
appended once you reach the validity threshold — **10** QSOs for POTA, **4** for
SOTA. The Ref row shows `Ref: US-1234` when armed or `Ref: (off)`.

### What CP705 writes to the ADIF

Each QSO logged while an activation is armed carries the correct program fields:

| Program | ADIF fields added to every QSO |
|---|---|
| **POTA** | `<MY_SIG>POTA` and `<MY_SIG_INFO>` = your park ref |
| **SOTA** | `<MY_SOTA_REF>` = your summit ref (SOTA uses its **own** field, *not* `MY_SIG`) |

Every record — activation or not — also includes `BAND` (e.g. `20m`) and a
`TIME_ON` stamped at the QSO **start** (per ADIF semantics and how POTA tallies),
alongside the usual `CALL` / `STATION_CALLSIGN` / `QSO_DATE` / `MODE` / grids /
reports. That's the complete required field set for a solo activator upload.

### Field workflow

1. Arm the program + ref before you start calling.
2. Operate — watch the counter tick toward `10`/`4` so you know when the
   activation is valid.
3. When done, export the log to SD (**Logging → `1`**, see **Logging and
   Download** below) and upload the `.adi` to the POTA/SOTA site from a computer.
4. For a **second park/summit in the same outing**: export first, then **Clear
   QSO Log** (`2`), change the **Ref** (`5`) to the new reference — that resets
   the progress counter for the new activation — and carry on.

### Notes and limits

- **Session-scoped, reboot-clears.** The program stays where you left it, but the
  ref (and the counter) live in RAM only — never saved to `Station.txt`/NVS — and
  **clear on power-cycle**. This is deliberate: one activation per outing, so a
  stale reference can never silently tag a later casual session's QSOs.
- **Calling `CQ POTA` is separate.** Pressing `C` lets you send a `CQ POTA …` /
  `CQ SOTA …` message (see the CQ prompt feature note); that shapes your
  *transmission*. The activation Ref shapes your *log*. Set both when activating.
- **Park-to-Park / Summit-to-Summit is not supported.** Logging the *other*
  station's reference (`SIG`/`SIG_INFO`, `SOTA_REF`) would require typing it per
  QSO, because an FT8 message doesn't carry the other station's park/summit — the
  reference simply isn't in the mode. This is an FT8 limitation, not a CP705 one.

## Logging and Download

- **Where the log lives:** every completed QSO is appended to an ADIF log held in
  **NVS** (non-volatile flash), so it survives power-off, reflashing, and even a
  full firmware reinstall. NVS is the single source of truth — QSOs are **not**
  written to the SD card as they happen. Station settings persist in NVS too.
- **Why NVS:** on this board there is no internal FATFS partition and SD writes
  are flaky, so NVS is the primary, always-available store. The SD card only ever
  gets **one** file, written once when you Export (below).
- **Getting the log off the device (one file, named by date):**
  1. Insert a FAT/FAT32-formatted SD card.
  2. Press `M` for the settings menu, then `4` for **Logging**, then `1`
     (**End+Export Log SD**). This is a **one-way, end-of-session action**: it
     stops RX/TX, releases the IC-705 connection, and drops WiFi before
     writing the card (press `S → 1` to reconnect afterward, or reboot). The full
     NVS log is written to a single **`YYYYMMDD.adi`** file (named by date — one
     per day; exporting again the same day overwrites it), then read back and
     byte-verified.
  3. On `Verified N QSOs`, pull the card and import that one `.adi` into your
     logging software. Your log stays safe in NVS regardless of the SD result, so
     a failed export never loses a QSO — just retry the export.
- **Starting a fresh log (e.g. between POTA activations):** export first (above)
  to save a copy, then in the same **Logging** category press `2`
  (**Clear QSO Log**). The row re-labels itself `Press 2 again: confirm`;
  press `2` again within 3 seconds to erase the NVS log and reset the
  on-screen QSO count to 0. Any other key, or letting the 3 seconds lapse,
  cancels it with no change. This does not touch the SD card; only the
  durable NVS copy is cleared.

### If SD export fails with the LoRa-1262 cap installed

The M5Stack **LoRa-1262 cap** (used here for GNSS, see below) shares its SPI bus
with the SD card slot — both use MOSI `G14` / MISO `G39` / SCK `G40`. If the
cap's own chip-select (`G5`, NSS) isn't held deasserted while the SD card is
being written, the SX1262 radio can respond on the shared bus and corrupt the
SD write, even though **reads still work fine** and the card itself is not at
fault (this reproduces identically across different, known-good cards). CP705
parks the cap's NSS pin high before every SD access specifically to prevent
this, so exporting should work with the cap installed. If you ever build a
custom variant without that fix (or add other SPI peripherals to the same
header), and SD writes fail while reads succeed, check this first before
suspecting the card.

## GPS Connections

CP705 gets GPS exclusively from the M5Stack **LoRa-1262 cap's** GNSS module, on
UART2 (`RX=G15`, `TX=G13`) at a fixed 115200 baud. The LoRa/SX1262 radio side of
the cap is not used for anything — only its GNSS chip. There is no external
PORTA GPS wiring option; the cap is required for GPS/UTC lock.

The physical G4/G5 debug UART path is disabled and the pins are left as
floating inputs to avoid conflicts with the cap. USB Serial/JTAG host commands
still work.

The GPS view (`M` → System → `3`) shows the active source, 3D fix, satellites,
UTC time, grid square, and last synchronization age.

## DS3231 RTC Connections

CP705 can use an optional DS3231 module as an external UTC clock. Connect it
to the Cardputer Adv shared I2C bus: `SDA=G8`, `SCL=G9`, plus module power and
ground. On boot, a valid DS3231 time is used before the ESP RTC or saved
`Station.txt` time. Status `S -> 6` appends `R` when the active time came from
the DS3231, and appends `G` after a full GPS time sync. GPS and manual time
updates write the DS3231 when it is present; FT8 decode fine corrections do not.
