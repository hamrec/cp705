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
  Operating, IC-705/Network, Logging) instead of a flat run of numbered pages;
  `N`/`O` still jump straight to Operating/IC-705 Network as shortcuts.
- **On-device network login editor** — WiFi SSID/password and IC-705 network
  user/password are editable under **IC-705/Network**.
- **Editable CQ prompt** — pressing `C` opens the CQ message pre-filled with
  `CQ <call> <grid>`, cursor placed right after `CQ ` so a prefix (e.g. `POTA`)
  can be typed immediately; confirming sends it, and repeats (manual or
  beacon) keep reusing that same text until you press `C` again for a fresh
  default.
- **Icom-styled hero card** — the moment a CQ or QSO is active, the decode list
  is replaced by a full-screen card: callsign/grid, a 6-stage exchange
  tracker, the current TX line, and frequency/SNR. `` ` `` bails out of the
  QSO at will and returns to the decode list.
- **End+Export Log SD** (Logging category → `1`) — writes the accumulated NVS
  ADIF log to the card as a verified, uniquely-named `.adi` file for import
  into logging software (see **Logging and Download** below).
- **Clear QSO log** (Logging category → `2`) — a two-press-confirm action to
  wipe the NVS log and start fresh, e.g. between POTA activations.
- **Streamlined to the IC-705 target** — the KH1-specific CAT/diagnostic keys
  were removed to keep the build focused on the wireless IC-705 use case, and
  several settings inherited from Mini-FT8 (ignore list, QSO comment template,
  RX/TX text log, PORTA GPS wiring) were dropped as dead weight on this board.

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
press **`O`** to jump there directly (or `M` for the category picker, then `3`).

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
   (survives power-off) and best-effort to the SD card. To get an importable
   `.adi`, export to the card from the **Logging** category, item `1`
   (see **Logging and Download**). The QSO view (**`Q`**) shows the session
   log status.

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
  item `6`, or reconnect with `S → 2`.
- **Login rejected:** the CP705 network user/password must match `Network User1`
  on the radio. Only one client can use the radio's remote server at a time, so
  make sure wfview/RS-BA1/SDR-Control isn't already connected.
- **Transmits but no RF / no modulation:** set `MOD Input → DATA MOD = WLAN` and
  operate in a **data mode (USB-D)**.
- **No decodes despite strong signals:** time isn't UTC-locked — connect a GPS or
  DS3231 and confirm the time source shows `G` (GPS) or `R` (RTC) on `S → 6`.

---

# CP705 Operation Manual

> This manual is inherited from Mini-FT8 and describes the shared on-device UI.

## Quick Mode Map

| Key | Mode | Purpose |
|---|---|---|
| `R` | RX | Idle: numbered list of decoded/CQ traffic. Active: full-screen QSO hero card. |
| `C` | Call CQ | Opens an editable CQ prompt (from RX); confirming sends it. |
| `` ` `` | Bail | From the hero card, drops the in-progress QSO and returns to the decode list. |
| `T` | TX Queue | View and manage the transmit queue. |
| `S` | STATUS | Access beacon, connect/sync, band step, tune, and date/time functions. |
| `G` | GPS | View GPS telemetry and synchronization status. |
| `M` | MENU | Opens the settings category picker: Station, Operating, IC-705/Network, Logging. |
| `N` | MENU (Operating) | Shortcut straight into the Operating category. |
| `O` | MENU (IC-705/Network) | Shortcut straight into the IC-705/Network category. |
| `Q` | QSO | View the session QSO count / logging status (full log is in NVS). |
| `D` | Mass Storage | Toggle internal FATFS ownership between CP705 and the PC (needs a FATFS partition; inert on this board). |
| `B` | BAND | Edit per-band frequencies. |
| `P` | Performance | View A Simple Performance Monitor. (added in V2.0.4)|

## Global Keys and Navigation

- `R` / `T` / `B` / `S` / `G` / `Q` / `D`: switch to the selected mode. Press the same mode key again to return to `RX`.
- `M`: open the settings category picker; press again from the picker to return to `RX`, or from inside a category to go back up to the picker.
- `N` / `O`: jump straight into the Operating / IC-705/Network category; press again to return to `RX`.
- `C`: from `RX`, opens the editable CQ prompt.
- `` ` ``: in `RX`, bails out of the active QSO/CQ (hero card); in a settings category, goes back to the picker; cancels TX globally in `TX` and `STATUS` when not editing; cancels an in-progress text edit elsewhere.
- `▲` / `▼`: page up / page down in `RX`, `TX`, `BAND`, and `QSO`. Settings categories each fit on one screen, so there's no in-category paging.
- `◀` / `▶`: move left / right in `QSO-SNR` and `STATUS` date/time.
- `1`..`6`: always select the currently visible row in the active mode.

## Per-Mode Controls

- ` acts as ESC where applicable (no physical Esc key on this keyboard).
- Text Edit: Backspace deletes, ` cancels, Enter saves.

| Mode | Item | Notes |
|---|---|---|
| `R` (RX, idle) | `1..6` | Select a decoded line to reply to. CQ messages are sorted from strongest to weakest. |
|  | `C` | Open the CQ prompt, pre-filled `CQ <call> <grid>`, cursor right after `CQ `. Enter sends it, `` ` `` cancels with no TX. |
|  | `▲` `▼` | Page up/down is available when line 1 or line 6 is cyan. |
| `R` (RX, hero card) |  | Shows the worked station's callsign/grid, a 6-stage exchange tracker (CQ/GRID/RPT/R/RR73/73), the current TX line, and frequency/SNR. Appears automatically the moment a CQ or QSO is active. |
|  | `` ` `` | Bail out of the QSO and return to the decode list. |
| `T` (TX Queue) | `1` | Rotate the queue to the next same-parity entry. |
|  | `2..6` | Drop the queue item on the current page. |
|  | `` ` `` | Cancel TX immediately. |
| `G` (GPS) |  | View live GPS telemetry: 3D fix, satellites, UTC time, grid square, and last synchronization age (LoRa-1262 cap GNSS is the only source). |
| `S` (STATUS) | `1` | Cycle Beacon mode. Applies when leaving STATUS mode. |
|  | `2` | Run connect/sync now; starts audio and follows the CAT sync path. |
|  | `3` | Step to the next active band. Applies after key 2 is pressed or when leaving STATUS. |
|  | `4` | Toggle Tune. |
|  | `5` | Edit Date (in place). On the Time line, `G` means GPS time and `R` means DS3231 RTC time. |
|  | `6` | Edit Time (in place). |
| `M` → **Station** | `1` | Edit Call (in place). |
|  | `2` | Edit Grid (in place). Supports 4/6/8-character grid. If GPS is available, the GPS grid is shown and used, but not saved. |
|  | (display) | Sleep/Batt %, and current UTC time + source (`G`=GPS, `R`=DS3231 RTC, blank=other). |
|  | `5` | Enter deep sleep now. |
| `M` → **Operating** | `1` | Select offset source: Random / RX / Fixed. Random values are within 500-2500 Hz. |
|  | `2` | Edit fixed cursor offset (in place). Enter directly or use `▲` `▼` `◀` `▶`. |
|  | `3` | Turn SkipTX1 on/off. Skips `dxcall mycall mygrid` and replies with the SNR report directly — useful for contest/pileup speed; leave off for normal grid-exchange QSOs. |
|  | `4` | Edit max retry (in place). Accepts any natural number or `0`. |
|  | `5` | Edit active bands (Long Edit). Used by STATUS -> Band. |
|  | `6` | Select FT8 / FT4 protocol. Reboot to apply the change. |
| `M` → **IC-705/Network** | `1` | Edit WiFi SSID (in place). |
|  | `2` | Edit WiFi Password (in place, masked). |
|  | `3` | Edit Net User (in place). |
|  | `4` | Edit Net Password (in place, masked). |
|  | `5` | Edit CI-V Address (in place). Accepts decimal or `0xNN` hex. |
|  | `6` | Re-resolve / reconnect. Re-points CP705 at the radio's IP even if WiFi is already up (STATUS `2` skips this when already connected) — the row shows live WiFi status. |
| `M` → **Logging** | `1` | Export Log to SD. Writes the NVS ADIF log to a unique file, then reads it back and byte-verifies it. Feedback: `Verified N QSOs` / `SD write failed` / `Verify FAILED` / `No log yet`. |
|  | `2` | Clear QSO Log: press once to arm (`Press 2 again: confirm`), press `2` again within 3 s to wipe the NVS log and reset the QSO count. Any other key or letting it lapse cancels with no change. Export first if you want a copy — this never touches the SD card. |
| `Q` (QSO) |  | Shows session QSO count and logging status. The full ADIF log lives in NVS / on the SD card (export via MENU → Logging → `1`); the internal-flash file browser is unavailable on this board. |
| `B` (BAND) | `1..6` | Choose a band slot to edit. |
| `D` (Mass Storage) |  | Stop radio audio and expose FATFS to the PC, then eject and press `D` to return to RX. Requires a FATFS partition — inert on this board (config/logs live in NVS instead). |
| `P` (PERFORMANCE) | | A Simple Performance Monitor. (added in V2.0.4) |

## Logging and Download

- **Where the log lives:** every completed QSO is appended to an ADIF log held in
  **NVS** (non-volatile flash), so it survives power-off, reflashing, and even a
  full firmware reinstall. It is also written best-effort to the SD card as
  `YYYYMMDD.adi`. Station settings persist in NVS the same way.
- **Why NVS:** on this board there is no internal FATFS partition, so NVS is the
  primary, always-available store; the SD card is a convenience copy for pulling
  your log off the device.
- **Getting the log off the device:**
  1. Insert a FAT/FAT32-formatted SD card.
  2. Press `M` for the settings menu, then `4` for **Logging**, then `1`
     (**End+Export Log SD**). This is a **one-way, end-of-session action**: it
     stops RX/TX, releases the IC-705 connection, and drops WiFi before
     writing the card (press `2` to reconnect afterward, or reboot). The full
     NVS log is written to a unique `MMDDHHMM.adi` file, then read back and
     byte-verified.
  3. On `Verified N QSOs`, pull the card and import the `.adi` into your logging
     software. Your log stays safe in NVS regardless of the SD result, so a
     failed export never loses a QSO — just retry the export.
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

The GPS view (`G`) shows the active source, 3D fix, satellites, UTC time, grid
square, and last synchronization age.

## DS3231 RTC Connections

CP705 can use an optional DS3231 module as an external UTC clock. Connect it
to the Cardputer Adv shared I2C bus: `SDA=G8`, `SCL=G9`, plus module power and
ground. On boot, a valid DS3231 time is used before the ESP RTC or saved
`Station.txt` time. Status `S -> 6` appends `R` when the active time came from
the DS3231, and appends `G` after a full GPS time sync. GPS and manual time
updates write the DS3231 when it is present; FT8 decode fine corrections do not.
