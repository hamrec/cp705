#pragma once

// ---------------------------------------------------------------------------
// qso_log — durable ADIF QSO logging for CP705.
//
// Owns everything about turning a completed contact into a stored ADIF record:
// timezone-independent UTC formatting, the ADIF line itself, the durable NVS
// log blob (bounded so it can't starve the config), the best-effort SD/flash
// copies, and the verified SD export. `main` stays the "hands": it assembles a
// QsoLogRecord from live app state and calls qso_log_write(); the module does
// the rest. Extracted from main.cpp (ported in spirit from TD705's qso_log).
//
// Storage reality on this board: SD writes are flaky and there is no FATFS
// partition, so NVS is the primary, always-available store. The SD/flash copies
// are best-effort; the verified export is how the operator pulls the log.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>

// One completed QSO, assembled by main and handed to the logger.
struct QsoLogRecord {
  std::string dxcall;    // worked station's callsign
  std::string dxgrid;    // worked station's grid (may be empty)
  int         rst_sent;  // dB report we sent (-99 = omit the field)
  int         rst_rcvd;  // dB report we received (-99 = omit the field)
  double      freq_mhz;  // dial frequency in MHz
  std::string mode;      // e.g. "FT8"
  std::string mycall;    // our callsign
  std::string mygrid;    // our grid (4-char)
  std::string comment;   // already macro-expanded
  int64_t     utc_ms;    // QSO time, Unix ms UTC
};

// Append one QSO. Formats the ADIF record, appends it to the durable NVS log,
// and makes best-effort SD + internal-flash copies. Returns true once the NVS
// write is done (SD/flash failures are non-fatal and never force a retry).
bool qso_log_write(const QsoLogRecord& r);

// Load the accumulated NVS ADIF log blob. Returns false if empty/absent.
bool qso_log_load_nvs(std::string& out);

// Count the QSO records currently stored in NVS (number of <eor> markers). Used
// at boot to seed the on-screen counter so it reflects the durable log instead
// of resetting to 0 every power-on. Returns 0 if the log is empty/absent.
int qso_log_count_nvs();

// Export the NVS log to a UNIQUE YYYYMMDD_HHMMSS.adi on the SD card, read it
// back and byte-verify before trusting it, then return a short human message
// ("Verified N QSOs" or an error). NVS is left intact (it is the durable copy).
// `utc_ms` names the file; run this while idle, not while decoding.
std::string qso_log_export_to_sd(int64_t utc_ms);
