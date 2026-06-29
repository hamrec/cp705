# CP705 — A Self-Contained IC-705 WiFi FT8 Client

> Built on **[Mini-FT8 by Wei, AG6AQ](https://github.com/wcheng95/Mini-FT8)** — with
> deep gratitude. Please visit and star the original project.

CP705 is a fork of Wei (AG6AQ)'s **[Mini-FT8](https://github.com/wcheng95/Mini-FT8)**,
and it stands entirely on the shoulders of that work — Karlis Goba's [ft8_lib](https://github.com/kgoba/ft8_lib),
the audio/DSP and autoseq foundation from Zhenxing (N6HAN), and the inspiration
of the DX-FT8 team. **All credit for the original application belongs to them.**

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

After flashing, set up your station on the device — call/grid (`M`→`4`/`5`) and the
IC-705 WiFi/network settings (`O`, then page down). Settings persist in NVS. See
**IC-705 Setup** below for the full radio-side configuration.

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
- **Reliable config + log storage on a board with no usable filesystem** — this
  unit has no internal FATFS partition and its SD card accepts reads but fails
  writes at the driver level, so station settings and the ADIF QSO log are stored
  in **NVS** (non-volatile flash key/value store), which survives power-off. The
  log is also written best-effort to the SD card, and can be exported to the card
  on demand in one shot.

## New features and menu changes (vs. Mini-FT8)

- **IC-705 over WiFi** as a first-class radio target — CAT, RX decode, and TX
  all run over the radio's WLAN connection instead of a serial/USB-audio path.
- **No external audio hardware** — the soundcard/USB-C-audio-adapter and audio
  cabling that other radios require are gone; the WiFi audio stream replaces them.
- **NVS-backed persistence** — callsign, grid, WiFi/network credentials, CI-V
  address, bands, and the ADIF QSO log all persist in NVS across reboots (no
  dependence on the SD card or an internal FATFS partition).
- **On-device network login editor** — WiFi SSID/password and IC-705 network
  user/password are editable on a fourth MENU page (`O`, then page down).
- **Export Log to SD** (MENU P3 → `5`) — writes the accumulated NVS ADIF log to
  the card as `YYYYMMDD.adi` in one shot for import into logging software.
- **Streamlined to the IC-705 target** — the KH1-specific CAT/diagnostic keys
  were removed to keep the build focused on the wireless IC-705 use case.

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
│ IC-705              │  SSID broadcast, radio = 192.168.59.1 │ Cardputer ADV    │
│  WLAN AP + remote   │<─────────────────────────────────────>│  CP705 (client)  │
│  server             │   UDP 50001 control  (login/CI-V auth) │                  │
│                     │   UDP 50002 serial   (CI-V frames)     │                  │
│                     │   UDP 50003 audio    (RX + TX PCM)     │                  │
└─────────────────────┘                                        └──────────────────┘
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

All connection settings live on a **fourth MENU page**: press **`O`** (MENU P3),
then **`▼`** to page down to the IC-705 WiFi/network page.

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
   Watch the WiFi status line on the IC-705 WiFi MENU page for progress.
3. Pick a band from **`S` → `3`** (steps through your active bands) and let the
   waterfall fill; decodes appear in **`R`** (RX).
4. (Recommended) Plug in a GPS or DS3231 so the 15-second FT8 window is locked to
   UTC — see **GPS Connections** / **DS3231 RTC Connections** below. Accurate
   time is required for reliable decodes and properly-timed transmit.
5. Logging is automatic: each completed QSO is appended to the ADIF log in **NVS**
   (survives power-off) and best-effort to the SD card. To get an importable
   `.adi`, export to the card with **MENU P3 → `5`** (see **Logging and Download**).
   The QSO view (**`Q`**) shows the session log status.

## Quick reference

| Item | Value |
|---|---|
| Radio WiFi mode | Access Point |
| Radio IP | `192.168.59.1` |
| Control / Serial / Audio ports | `50001` / `50002` / `50003` (UDP) |
| Audio format | 48 kHz, 16-bit, mono (LPCM) |
| IC-705 CI-V address | `0xA4` (default) |
| FT8 operating mode | USB-D (data) |
| QSO log (primary) | NVS (ADIF); export to SD via `O`→`5` |

## Troubleshooting

- **WiFi won't connect:** confirm the SSID/password in CP705 match the radio's
  Access-Point SSID/password exactly, and that `WLAN` + `DHCP Server` are ON.
- **WiFi connects but no decodes / no CAT:** check `Network Control` is ON, the
  `Network User1` is an **Administrator**, and the three UDP ports are at their
  defaults (50001/50002/50003). Re-resolve with MENU page 4 → `4`, or reconnect
  with `S → 2`.
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
| `R` | RX | View decoded messages and tap one to start a QSO. |
| `T` | TX Queue | View and manage the transmit queue. |
| `S` | STATUS | Access beacon, connect/sync, band step, tune, and date/time functions. |
| `G` | GPS | View GPS telemetry and synchronization status. |
| `M` | MENU P1 | Configure core station and operator settings. |
| `N` | MENU P2 | Configure radio, input, and comment settings. |
| `O` | MENU P3 | Configure logging, active bands, GNSS LoRa GPS, export-log-to-SD, and retry settings. |
| `Q` | QSO | View the session QSO count / logging status (full log is in NVS — export with `O`→`5`). |
| `D` | Delete Files | Browse/delete internal-FATFS files (inert on boards without a FATFS partition). |
| `B` | BAND | Edit per-band frequencies. |
| `C` | USB Drive | Toggle internal FATFS ownership between CP705 and the PC (needs a FATFS partition; inert on this board). |
| `P` | Performance | View A Simple Performance Monitor. (added in V2.0.4)|

## Global Keys and Navigation

- `R` / `T` / `B` / `S` / `G` / `Q` / `D` / `C`: switch to the selected mode. Press the same mode key again to return to `RX`.
- `M` / `N` / `O`: jump to MENU page 1 / 2 / 3. Press the current page key again to return to `RX`.
- `` ` ``: cancel TX globally in `RX`, `TX`, and `STATUS` when not editing.
- `▲` / `▼`: page up / page down in `RX`, `TX`, `BAND`, `MENU`, `QSO`, and `Delete`.
- `◀` / `▶`: move left / right in `QSO-SNR`, `STATUS` date/time, `MENU P2` (N->2).
- `1`..`6`: always select the currently visible row in the active mode.

## Per-Mode Controls

- ` acts as ESC where applicable.
- Text Edit: Backspace deletes, ` cancels, Enter saves.
  
| Mode | Item | Notes |
|---|---|---|
| `R` (RX) | `1..6` | Select a decoded line to reply to. CQ messages are sorted from strongest to weakest. If selected within 4 s, TX starts immediately. |
|  | `▲` `▼` | Page up/down is available when line 1 or line 6 is cyan. |
| `T` (TX Queue) | `1` | Rotate the queue to the next same-parity entry. |
|  | `2..6` | Drop the queue item on the current page. |
|  | `` ` `` | Cancel TX immediately. |
| `G` (GPS) |  | View live GPS telemetry including active source, 3D fix, satellites, UTC time, grid square, and last synchronization age. |
| `S` (STATUS) | `1` | Cycle Beacon mode. Applies when leaving STATUS mode. |
|  | `2` | Run connect/sync now; starts audio and follows the CAT sync path. |
|  | `3` | Step to the next active band. Applies after key 2 is pressed or when leaving STATUS. |
|  | `4` | Toggle Tune. |
|  | `5` | Edit Date (in place). On the Time line, `G` means GPS time and `R` means DS3231 RTC time. |
|  | `6` | Edit Time (in place). |
| `M` (MENU P1) | `1` | Cycle CQ Type. For CQ FD, enter operating class and ARRL/RAC section in FreeText, for example `1B SCV`. |
|  | `2` | Send FreeText once. |
|  | `3` | Edit FreeText (Long Edit). Used for SOTAMAT, park/summit reference, ARRL Field Day exchange, CQ modifiers (`CQ EU`, `CQ ASIA`), and similar text. |
|  | `4` | Edit Call (in place). |
|  | `5` | Edit Grid (in place). Supports 4/6/8-character grid. If GPS is available, the GPS grid is shown and used, but not saved. |
|  | `6` | Enter Sleep. Shows battery info. |
| `N` (MENU P2) | `1` | Select offset source: Random / RX / Fixed. Random values are within 500-2500 Hz. |
|  | `2` | Edit fixed cursor offset (in place). Enter directly or use `▲` `▼` `◀` `▶`. |
|  | `3` | Select radio. CP705 drives the IC-705 over WiFi only; the legacy `KH1-MIC` toggle is inert. |
|  | `4` | Edit ignore list (Long Edit). Prefixes are separated by spaces; maximum 64 characters. |
|  | `5` | Edit comment (Long Edit). Used for ADIF logging. Supports `/Radio` and `/Grid` macro expansion. |
|  | `6` | Select FT8 / FT4 protocol. Reboot to apply the change. |
| `O` (MENU P3) | `1` | Turn RxTx log on/off. Note: RxTxLog has been renamed to `RT[YYMMDD].txt`. |
|  | `2` | Turn SkipTX1 on/off. Skips `dxcall mycall mygrid` and replies with the SNR report. |
|  | `3` | Edit active bands (Long Edit). Used by STATUS -> Band. |
|  | `4` | Toggle `GNSS_LoRa`. `OFF` uses PORTA GPS; `ON` uses the LoRa-1262 cap GNSS. |
|  | `5` | Export Log to SD. Writes the NVS ADIF log to the card as `YYYYMMDD.adi`. Feedback: `Exported to SD` / `SD write failed` / `No log yet`. |
|  | `6` | Edit max retry (in place). Accepts any natural number or `0`. |
| `Q` (QSO) |  | Shows session QSO count and logging status. The full ADIF log lives in NVS / on the SD card (export via MENU P3 → `5`); the internal-flash file browser is unavailable on this board. |
| `D` (Delete Files) | `1..6` | Delete the selected internal-flash file. Inert when there's no FATFS partition. |
| `B` (BAND) | `1..6` | Choose a band slot to edit. |
| `C` (USB Drive) |  | Stop radio audio and expose FATFS to the PC, then eject and press `C` to return to RX. Requires a FATFS partition — inert on this board (config/logs live in NVS instead). |
| `P` (PERFORMANCE) | | A Simple Performance Monitor. (added in V2.0.4) |

## Logging and Download

- **Where the log lives:** every completed QSO is appended to an ADIF log held in
  **NVS** (non-volatile flash), so it survives power-off. It is also written
  best-effort to the SD card as `YYYYMMDD.adi`. Station settings persist in NVS
  the same way.
- **Why NVS:** on this board there is no internal FATFS partition, and the SD
  card reads fine but its writes fail intermittently at the driver level under
  load — so NVS is the reliable store and the SD card is a convenience copy.
- **Getting the log off the device:**
  1. Insert a FAT/FAT32-formatted SD card.
  2. When idle (not decoding), open MENU P3 (`O`) and press `5` (**Export Log to
     SD**). The full NVS log is written to the card as `YYYYMMDD.adi`.
  3. On `Exported to SD`, pull the card and import the `.adi` into your logging
     software. If it shows `SD write failed`, retry while the radio isn't actively
     decoding.

## GPS Connections

CP705 supports two GPS sources selected from MENU P3 (`O -> 4`):

- `GNSS_LoRa:OFF` uses the PORTA GPS wiring below. Both 9600 and 115200 baud GPS modules are supported and auto-detected. **Make sure the micro switch is on the left.** Once CP705 gets its time/grid, the GPS can be removed.
- `GNSS_LoRa:ON` uses the M5Stack LoRa-1262 cap GNSS on UART2 (`RX=G15`, `TX=G13`) at 115200 baud. The LoRa/SX1262 radio side is not used.

When `GNSS_LoRa` is `ON`, the physical G4/G5 debug UART path is disabled and the pins are left as floating inputs to avoid conflicts. USB Serial/JTAG host commands still work.

The GPS view shows the active source on its first line.
```text
┌──────────────────┐                 ┌─────────────────────────────┐
│ GPS              │                 │ Cardputer ADV               │
│                  │                 │ PORTA                       │
│ GND ─────────────┼─────────────────┤ GND                         │
│ VDD ─────────────┼─────────────────┤ 5V                          │
│ RX  ─────────────┼<──(Not Used)────┤ TX (G2)                     │
│ TX  ─────────────┼────────────────>┤ RX (G1)                     │
└──────────────────┘                 │                             │
                                     │ SW: 5VOUT (Left)            │
                                     └─────────────────────────────┘
```

## DS3231 RTC Connections

CP705 can use an optional DS3231 module as an external UTC clock. Connect it
to the Cardputer Adv shared I2C bus: `SDA=G8`, `SCL=G9`, plus module power and
ground. On boot, a valid DS3231 time is used before the ESP RTC or saved
`Station.txt` time. Status `S -> 6` appends `R` when the active time came from
the DS3231, and appends `G` after a full GPS time sync. GPS and manual time
updates write the DS3231 when it is present; FT8 decode fine corrections do not.
