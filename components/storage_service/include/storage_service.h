#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "esp_err.h"

enum class StorageOwner : uint8_t {
    UNAVAILABLE,
    FIRMWARE,
    USB_HOST,
    TRANSITION,
};

enum class StorageOpenMode : uint8_t {
    READ,
    WRITE_TRUNCATE,
    APPEND,
};

struct StorageStream;

struct StorageCopyResult {
    esp_err_t err = ESP_OK;
    int copied_count = 0;
    int missed_count = 0;
};

esp_err_t storage_service_init();
StorageOwner storage_service_owner();
bool storage_service_firmware_available();
bool storage_service_usb_drive_enabled();
size_t storage_service_open_stream_count();
esp_err_t storage_service_set_usb_drive_enabled(bool enabled);

bool storage_file_exists(const std::string& name);
bool storage_file_remove(const std::string& name);
bool storage_file_list(std::vector<std::string>& files);
bool storage_file_read_text(const std::string& name, std::string& content);
bool storage_file_write_atomic(const std::string& name, const std::string& content);
bool storage_file_append(const std::string& name,
                         const std::string& content,
                         const std::string& header_if_new = "",
                         bool sync_to_flash = false);
bool storage_file_append_cabrillo(const std::string& mycall,
                                  const std::string& location,
                                  const std::string& qso_line);

StorageStream* storage_stream_open(const std::string& name, StorageOpenMode mode);
size_t storage_stream_read(StorageStream* stream, void* data, size_t size);
bool storage_stream_read_line(StorageStream* stream, char* line, size_t line_size);
size_t storage_stream_write(StorageStream* stream, const void* data, size_t size);
bool storage_stream_seek(StorageStream* stream, long offset, int whence);
long storage_stream_tell(StorageStream* stream);
long storage_stream_size(StorageStream* stream);
bool storage_stream_sync(StorageStream* stream);
void storage_stream_close(StorageStream* stream);

bool storage_sync_station_from_sd();
StorageCopyResult storage_copy_all_to_sd(const std::string& priority_file);

// Appends directly to a file on the SD card (mount point /sdcard), mounting
// it on demand if needed. Bypasses the internal-storage "copy to SD" path
// entirely — useful for debug logs that need to survive a crash/reboot and
// be readable straight off the SD card without going through that copy step.
bool storage_sd_log_append(const std::string& name, const std::string& content);

// Like storage_sd_log_append(), but writes header_if_new to the file FIRST when
// (and only when) the file is being created (currently empty). Used for ADIF
// QSO logs on the SD card: the .adi file gets its ADIF header exactly once, on
// the first contact of the day, and every later QSO just appends a record.
bool storage_sd_append_with_header(const std::string& name,
                                   const std::string& content,
                                   const std::string& header_if_new);

// Reads an entire file from the SD card (mount point /sdcard) into out, mounting
// the card on demand. Independent of the internal-flash FATFS owner state, so it
// works even when that FATFS is unavailable. Returns false if absent/unreadable.
bool storage_sd_read_file(const std::string& name, std::string& out);

// Overwrites a file on the SD card with content (creating it if needed) and
// fsyncs it to the card. Independent of the internal-flash FATFS owner state.
bool storage_sd_write_file(const std::string& name, const std::string& content);

// Reports total/free bytes on the SD card (mounts on demand). Returns false if
// the card isn't available. Either pointer may be null.
bool storage_sd_space(uint64_t* total_bytes, uint64_t* free_bytes);

// Mounts the SD card now, while heap is most plentiful (call early at boot).
// storage_sd_log_append()'s later calls become a fast already-mounted no-op
// instead of paying the mount's allocation cost once WiFi/audio/decode are
// all competing for heap.
esp_err_t storage_sd_log_premount();

// Diagnostic code from the most recent storage_sd_log_append() call:
// 0 = ok, 1 = couldn't take the storage mutex, 2+esp_err = SD mount failed
// (subtract 2 for the esp_err_t value), -1-errno = fopen failed, -2 = short
// write. Exposed because this debug path has no other way to report errors.
extern int g_storage_sd_log_last_code;

// Largest free DMA-capable block (bytes), captured at the moment of the last
// mount failure — tells us whether DMA memory specifically was exhausted.
extern size_t g_storage_sd_log_dma_largest;
