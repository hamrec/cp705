// qso_log — durable ADIF QSO logging for CP705. See qso_log.h.

#include "qso_log.h"

#include <cstdio>
#include <cstring>

#include "nvs.h"
#include "nvs_flash.h"

#include "storage_service.h"

// NVS namespace shared with the rest of CP705 (config lives here too).
static const char* kNvsNamespace = "cp705";

// ADIF file header, written once at the top of a fresh log.
static const char* const kAdifHeader =
    "CP705 ADIF export\n<adif_ver:5>3.1.4\n<programid:5>CP705\n<eoh>\n";

// Unix ms (UTC) -> civil UTC date/time, timezone-independent (Howard Hinnant's
// civil_from_days). Avoids relying on the ESP TZ defaulting to UTC the way
// localtime_r() does, so ADIF QSO_DATE/TIME_ON are always correct.
static void civil_from_ms(int64_t ms, int* Y, int* M, int* D, int* h, int* mi, int* s) {
  int64_t days = ms / 86400000LL;
  int64_t rem  = ms - days * 86400000LL;
  if (rem < 0) { rem += 86400000LL; days -= 1; }
  int64_t secs = rem / 1000;
  *h = (int)(secs / 3600); *mi = (int)((secs / 60) % 60); *s = (int)(secs % 60);
  int64_t z = days + 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  int64_t doe = z - era * 146097;
  int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t y = yoe + era * 400;
  int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  int64_t mp = (5 * doy + 2) / 153;
  int64_t d = doy - (153 * mp + 2) / 5 + 1;
  int64_t m = mp < 10 ? mp + 3 : mp - 9;
  *Y = (int)(y + (m <= 2)); *M = (int)m; *D = (int)d;
}

// Appends one ADIF record to the day's log blob in NVS (key "adiflog"). The blob
// is bounded (kAdifNvsCap) by dropping whole oldest records from the front, so a
// long session can't starve the config in NVS; the SD card keeps the full log.
// May be called from the core1 decode task — NVS has its own internal locking.
static void nvs_append_adif(const std::string& record, const char* header) {
  static const size_t kAdifNvsCap = 8192;
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) return;
  std::string log;
  size_t len = 0;
  if (nvs_get_blob(h, "adiflog", nullptr, &len) == ESP_OK && len > 0) {
    log.resize(len);
    if (nvs_get_blob(h, "adiflog", &log[0], &len) != ESP_OK) log.clear();
  }
  const std::string hdr = (header && log.empty()) ? header : "";
  if (!hdr.empty()) log = hdr;
  log += record;
  if (log.size() > kAdifNvsCap) {
    const size_t hdr_len = header ? strlen(header) : 0;
    std::string body = log.substr(hdr_len);
    while (body.size() > kAdifNvsCap - hdr_len) {
      size_t eor = body.find("<eor>\n");
      if (eor == std::string::npos) { body.clear(); break; }
      body.erase(0, eor + 6);
    }
    log = (header ? std::string(header) : std::string()) + body;
  }
  if (nvs_set_blob(h, "adiflog", log.data(), log.size()) == ESP_OK) {
    nvs_commit(h);
  }
  nvs_close(h);
}

bool qso_log_write(const QsoLogRecord& r) {
  int year, month, day, hour, min, sec;
  civil_from_ms(r.utc_ms, &year, &month, &day, &hour, &min, &sec);
  char date[16];
  snprintf(date, sizeof(date), "%04d%02d%02d", year % 10000, month % 100, day % 100);

  char time_on[16];
  snprintf(time_on, sizeof(time_on), "%02d%02d%02d", hour % 100, min % 100, sec % 100);
  char freq_str[16];
  snprintf(freq_str, sizeof(freq_str), "%.3f", r.freq_mhz);

  // Build rst_sent/rst_rcvd fragments — omit when -99 (no data),
  // matching DXFT8 reference behavior (ADIF.c omits when value is 0).
  char rst_sent_buf[32] = "";
  char rst_rcvd_buf[32] = "";
  if (r.rst_sent != -99) {
    snprintf(rst_sent_buf, sizeof(rst_sent_buf), "<rst_sent:%d>%d ",
             (int)snprintf(nullptr, 0, "%d", r.rst_sent), r.rst_sent);
  }
  if (r.rst_rcvd != -99) {
    snprintf(rst_rcvd_buf, sizeof(rst_rcvd_buf), "<rst_rcvd:%d>%d ",
             (int)snprintf(nullptr, 0, "%d", r.rst_rcvd), r.rst_rcvd);
  }
  // BAND — POTA requires it (with unit, e.g. "20M"); we emit the ADIF-canonical
  // lowercase form ("20m") which importers accept case-insensitively. Omitted
  // if unknown so the record stays valid.
  char band_buf[24] = "";
  if (!r.band.empty()) {
    snprintf(band_buf, sizeof(band_buf), "<band:%zu>%s ", r.band.size(), r.band.c_str());
  }
  // Activation fields — only when a program is selected AND a ref is set. Each
  // program uses DIFFERENT ADIF fields (POTA has MY_SIG/MY_SIG_INFO; SOTA has
  // its own MY_SOTA_REF, NOT MY_SIG). None/empty emits nothing, so a plain
  // (non-activation) log is byte-identical to before.
  char act_buf[80] = "";
  if (!r.sig_ref.empty()) {
    if (r.sig_program == SigProgram::POTA) {
      snprintf(act_buf, sizeof(act_buf), "<my_sig:4>POTA <my_sig_info:%zu>%s ",
               r.sig_ref.size(), r.sig_ref.c_str());
    } else if (r.sig_program == SigProgram::SOTA) {
      snprintf(act_buf, sizeof(act_buf), "<my_sota_ref:%zu>%s ",
               r.sig_ref.size(), r.sig_ref.c_str());
    }
  }
  char line[512];
  snprintf(line, sizeof(line),
           "<call:%zu>%s <gridsquare:%zu>%s <mode:%zu>%s<qso_date:8>%s <time_on:6>%s <freq:%zu>%s %s<station_callsign:%zu>%s <my_gridsquare:%zu>%s %s%s%s<comment:%zu>%s <eor>\n",
           r.dxcall.size(), r.dxcall.c_str(),
           r.dxgrid.size(), r.dxgrid.c_str(),
           r.mode.size(), r.mode.c_str(),
           date, time_on,
           strlen(freq_str), freq_str,
           band_buf,
           r.mycall.size(), r.mycall.c_str(),
           r.mygrid.size(), r.mygrid.c_str(),
           rst_sent_buf, rst_rcvd_buf, act_buf,
           r.comment.size(), r.comment.c_str());

  // Durable store: NVS only. Every QSO lives in the bounded NVS blob, which is
  // the reliable copy on this board (SD writes are flaky and there's no FATFS
  // partition). We deliberately do NOT write the SD card per-QSO anymore -- that
  // produced a second, cryptically-named, unverified file that confused more
  // than it helped. The SD copy is written once, verified, by qso_log_export_to_sd()
  // as a single YYYYMMDD.adi when the operator presses Export.
  nvs_append_adif(line, kAdifHeader);
  return true;       // NVS write makes the record durable; never force a retry
}

bool qso_log_load_nvs(std::string& out) {
  out.clear();
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return false;
  size_t len = 0;
  if (nvs_get_blob(h, "adiflog", nullptr, &len) != ESP_OK || len == 0) { nvs_close(h); return false; }
  out.resize(len);
  esp_err_t e = nvs_get_blob(h, "adiflog", &out[0], &len);
  nvs_close(h);
  return e == ESP_OK;
}

int qso_log_count_nvs() {
  std::string log;
  if (!qso_log_load_nvs(log) || log.empty()) return 0;
  int n = 0;
  for (size_t p = log.find("<eor>"); p != std::string::npos; p = log.find("<eor>", p + 5)) ++n;
  return n;
}

std::string qso_log_export_to_sd(int64_t utc_ms) {
  std::string log;
  if (!qso_log_load_nvs(log) || log.empty()) return "No log yet";

  int Y, M, D, h, mi, s;
  civil_from_ms(utc_ms, &Y, &M, &D, &h, &mi, &s);
  // ONE file per day, named by date: YYYYMMDD.adi. FATFS long filenames are
  // disabled (CONFIG_FATFS_LFN_NONE) so the name must fit classic 8.3 -- 8 chars
  // is exactly the date, leaving no room for a time stamp, so a second export
  // the same day overwrites (re-writes the full verified log, which is what you
  // want). This is the only SD file CP705 produces.
  char path[16];
  snprintf(path, sizeof(path), "%04d%02d%02d.adi", Y, M, D);

  if (!storage_sd_write_file(path, log)) return "SD write failed";

  // Verify: read the SD file back and byte-compare before trusting the export.
  std::string back;
  if (!storage_sd_read_file(path, back) || back != log) return "Verify FAILED";

  int n = 0;  // count records exported (each QSO ends in <eor>)
  for (size_t p = log.find("<eor>"); p != std::string::npos; p = log.find("<eor>", p + 5)) ++n;
  char msg[40];
  snprintf(msg, sizeof(msg), "Verified %d QSOs", n);
  return msg;
}

void qso_log_clear_nvs() {
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) return;
  nvs_erase_key(h, "adiflog");  // ESP_ERR_NVS_NOT_FOUND if already empty — fine
  nvs_commit(h);
  nvs_close(h);
}
