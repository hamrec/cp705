#include "storage_service.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <new>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "wear_levelling.h"

namespace {

constexpr char kPartitionLabel[] = "fatfs";
constexpr char kBasePath[] = "/storage";
constexpr char kStationFile[] = "Station.txt";
constexpr char kStationTemp[] = "Station.tmp";
constexpr char kSdBasePath[] = "/sdcard";
constexpr gpio_num_t kSdMiso = GPIO_NUM_39;
constexpr gpio_num_t kSdMosi = GPIO_NUM_14;
constexpr gpio_num_t kSdClock = GPIO_NUM_40;
constexpr gpio_num_t kSdChipSelect = GPIO_NUM_12;

const char* TAG = "storage_service";

StaticSemaphore_t s_mutex_buffer;
SemaphoreHandle_t s_mutex;
StorageOwner s_owner = StorageOwner::UNAVAILABLE;
wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
tinyusb_msc_storage_handle_t s_msc_storage;
bool s_tinyusb_installed;
size_t s_open_streams;
bool s_station_sync_attempted;
sdmmc_card_t* s_sd_card;
bool s_sd_mounted;
// Once the SD log mount is established at boot (while DMA memory is plentiful),
// keep it up for the rest of the session: a later remount reliably fails with
// ESP_ERR_NO_MEM once WiFi/audio/decode have eaten DMA-capable heap. With this
// set, unmount_sd_locked() becomes a no-op so a mid-session "copy to SD" can't
// tear down the persistent logging mount.
bool s_sd_keep_mounted;

enum class MountTransitionResult : uint8_t {
    NONE,
    STARTED,
    COMPLETE,
    FAILED,
};

volatile MountTransitionResult s_mount_transition_result = MountTransitionResult::NONE;
volatile tinyusb_msc_mount_point_t s_mount_transition_point =
    TINYUSB_MSC_STORAGE_MOUNT_APP;

class StorageGuard {
public:
    StorageGuard() : held_(s_mutex && xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) == pdTRUE) {}
    ~StorageGuard() {
        if (held_) {
            xSemaphoreGiveRecursive(s_mutex);
        }
    }
    bool held() const { return held_; }

private:
    bool held_;
};

bool firmware_owns_storage_locked() {
    return s_owner == StorageOwner::FIRMWARE && s_msc_storage != nullptr;
}

bool normalize_name(const std::string& input, std::string& name) {
    name = input;
    const std::string prefix = std::string(kBasePath) + "/";
    if (name.compare(0, prefix.size(), prefix) == 0) {
        name.erase(0, prefix.size());
    }
    if (name.empty() || name.front() == '/' || name.find('\\') != std::string::npos ||
        name.find("..") != std::string::npos) {
        return false;
    }
    return true;
}

bool build_path(const std::string& input, std::string& path) {
    std::string name;
    if (!normalize_name(input, name)) {
        return false;
    }
    path = std::string(kBasePath) + "/" + name;
    return true;
}

bool sync_file(FILE* file) {
    return file && fflush(file) == 0 && fsync(fileno(file)) == 0;
}

bool write_atomic_locked(const std::string& input, const std::string& content) {
    if (!firmware_owns_storage_locked()) {
        return false;
    }

    std::string name;
    std::string final_path;
    if (!normalize_name(input, name) || !build_path(name, final_path)) {
        return false;
    }
    const std::string temp_name = (name == kStationFile) ? kStationTemp : name + ".tmp";
    std::string temp_path;
    if (!build_path(temp_name, temp_path)) {
        return false;
    }

    FILE* file = fopen(temp_path.c_str(), "wb");
    if (!file) {
        return false;
    }

    bool ok = content.empty() || fwrite(content.data(), 1, content.size(), file) == content.size();
    ok = ok && sync_file(file);
    if (fclose(file) != 0) {
        ok = false;
    }
    if (!ok) {
        unlink(temp_path.c_str());
        return false;
    }

    if (unlink(final_path.c_str()) != 0 && errno != ENOENT) {
        unlink(temp_path.c_str());
        return false;
    }
    if (rename(temp_path.c_str(), final_path.c_str()) != 0) {
        ESP_LOGE(TAG, "rename failed: %s -> %s", temp_path.c_str(), final_path.c_str());
        unlink(temp_path.c_str());
        return false;
    }
    return true;
}

bool list_files_locked(std::vector<std::string>& files) {
    files.clear();
    if (!firmware_owns_storage_locked()) {
        return false;
    }

    DIR* dir = opendir(kBasePath);
    if (!dir) {
        return false;
    }

    while (dirent* entry = readdir(dir)) {
        const char* name = entry->d_name;
        if (!name || name[0] == '.') {
            continue;
        }
        std::string path;
        if (!build_path(name, path)) {
            continue;
        }
        struct stat info {};
        if (stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode)) {
            files.emplace_back(name);
        }
    }
    closedir(dir);
    return true;
}

esp_err_t mount_sd_locked() {
    if (s_sd_mounted && s_sd_card) {
        return ESP_OK;
    }

    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = kSdMosi;
    bus_config.miso_io_num = kSdMiso;
    bus_config.sclk_io_num = kSdClock;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 1024;

    // The SD-over-SPI host driver requires DMA (SPI_DMA_DISABLED returns
    // ESP_ERR_INVALID_ARG from esp_vfs_fat_sdspi_mount) — DMA-capable memory
    // is a separate, more constrained pool than general heap, so under load
    // (WiFi + audio streaming + FT8 decode all running) the mount can fail
    // with a transient ESP_ERR_NO_MEM even when regular heap looks fine.
    // Retry a few times with a short backoff rather than failing outright.
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 4; ++attempt) {
        err = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) break;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = kSdChipSelect;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 3;  // copy_file_locked() opens source+dest simultaneously; keep headroom
    mount_config.allocation_unit_size = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;
    for (int attempt = 0; attempt < 4; ++attempt) {
        err = esp_vfs_fat_sdspi_mount(kSdBasePath, &host, &slot_config, &mount_config, &s_sd_card);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    if (err != ESP_OK) {
        s_sd_card = nullptr;
        s_sd_mounted = false;
        spi_bus_free(SPI2_HOST);
        return err;
    }

    s_sd_mounted = true;
    return ESP_OK;
}

void unmount_sd_locked() {
    // Preserve the persistent logging mount established by storage_sd_log_premount().
    // Remounting later (DMA exhausted) would fail and silently kill all SD logging.
    if (s_sd_keep_mounted) {
        return;
    }
    if (s_sd_mounted && s_sd_card) {
        esp_vfs_fat_sdcard_unmount(kSdBasePath, s_sd_card);
        s_sd_card = nullptr;
        s_sd_mounted = false;
    }
    spi_bus_free(SPI2_HOST);
}

} // namespace

int g_storage_sd_log_last_code = 0;  // 0=ok, 1=no mutex, 2+esp_err=mount fail, -1=fopen fail, -2=short write
size_t g_storage_sd_log_dma_largest = 0;  // largest free DMA-capable block (bytes) at last mount failure

esp_err_t storage_sd_log_premount() {
    StorageGuard guard;
    if (!guard.held()) return ESP_FAIL;
    esp_err_t err = mount_sd_locked();
    if (err == ESP_OK) {
        s_sd_keep_mounted = true;  // pin the mount for the rest of the session
    }
    return err;
}

bool storage_sd_log_append(const std::string& name, const std::string& content) {
    StorageGuard guard;
    if (!guard.held()) { g_storage_sd_log_last_code = 1; return false; }
    // mount_sd_locked() is a fast no-op once already mounted, so repeated
    // calls (one per log line) only pay the real mount cost once.
    esp_err_t mount_err = mount_sd_locked();
    if (mount_err != ESP_OK) {
        g_storage_sd_log_last_code = 2 + (int)mount_err;
        g_storage_sd_log_dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        return false;
    }
    const std::string path = std::string(kSdBasePath) + "/" + name;
    FILE* f = fopen(path.c_str(), "a");
    if (!f) { g_storage_sd_log_last_code = -1 - errno; return false; }
    const size_t written = fwrite(content.data(), 1, content.size(), f);
    // Force the data AND the FAT/directory update to the physical card now, so
    // the record survives the card being pulled without a clean unmount.
    sync_file(f);
    fclose(f);
    if (written != content.size()) { g_storage_sd_log_last_code = -2; return false; }
    g_storage_sd_log_last_code = 0;
    return true;
}

bool storage_sd_append_with_header(const std::string& name,
                                   const std::string& content,
                                   const std::string& header_if_new) {
    StorageGuard guard;
    if (!guard.held()) { g_storage_sd_log_last_code = 1; return false; }
    esp_err_t mount_err = mount_sd_locked();
    if (mount_err != ESP_OK) {
        g_storage_sd_log_last_code = 2 + (int)mount_err;
        g_storage_sd_log_dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        return false;
    }
    const std::string path = std::string(kSdBasePath) + "/" + name;
    FILE* f = fopen(path.c_str(), "a");
    if (!f) { g_storage_sd_log_last_code = -1 - errno; return false; }
    // In append mode the stream is positioned at end-of-file, so ftell() == 0
    // means the file is brand new/empty: write the header before the record.
    bool ok = true;
    if (ftell(f) == 0 && !header_if_new.empty()) {
        ok = (fwrite(header_if_new.data(), 1, header_if_new.size(), f) == header_if_new.size());
    }
    if (ok) {
        ok = (fwrite(content.data(), 1, content.size(), f) == content.size());
    }
    // Flush data + FAT/directory entry to the card so a pulled card keeps the QSO.
    sync_file(f);
    fclose(f);
    g_storage_sd_log_last_code = ok ? 0 : -2;
    return ok;
}

bool storage_sd_read_file(const std::string& name, std::string& out) {
    out.clear();
    StorageGuard guard;
    if (!guard.held()) return false;
    if (mount_sd_locked() != ESP_OK) return false;
    const std::string path = std::string(kSdBasePath) + "/" + name;
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    const bool ok = ferror(f) == 0;
    fclose(f);
    return ok;
}

bool storage_sd_write_file(const std::string& name, const std::string& content) {
    StorageGuard guard;
    if (!guard.held()) { g_storage_sd_log_last_code = 1; return false; }
    const esp_err_t mount_err = mount_sd_locked();
    if (mount_err != ESP_OK) {
        g_storage_sd_log_last_code = 2 + (int)mount_err;
        g_storage_sd_log_dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        return false;
    }
    const std::string path = std::string(kSdBasePath) + "/" + name;
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) { g_storage_sd_log_last_code = -1 - errno; return false; }
    const bool ok = content.empty() ||
                    fwrite(content.data(), 1, content.size(), f) == content.size();
    sync_file(f);  // flush data + FAT/dir entry so it survives a card pull
    fclose(f);
    g_storage_sd_log_last_code = ok ? 0 : -2;
    return ok;
}

namespace {

esp_err_t copy_file_locked(const std::string& source, const std::string& destination) {
    FILE* input = fopen(source.c_str(), "rb");
    if (!input) {
        return ESP_FAIL;
    }
    FILE* output = fopen(destination.c_str(), "wb");
    if (!output) {
        fclose(input);
        return ESP_FAIL;
    }

    uint8_t buffer[4096];
    bool ok = true;
    while (true) {
        const size_t count = fread(buffer, 1, sizeof(buffer), input);
        if (count > 0 && fwrite(buffer, 1, count, output) != count) {
            ok = false;
            break;
        }
        if (count < sizeof(buffer)) {
            if (ferror(input)) {
                ok = false;
            }
            break;
        }
    }
    ok = ok && sync_file(output);
    if (fclose(output) != 0) {
        ok = false;
    }
    fclose(input);
    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t copy_file_retry_locked(const std::string& source,
                                 const std::string& destination,
                                 int attempts = 5) {
    esp_err_t result = ESP_FAIL;
    for (int attempt = 0; attempt < attempts; ++attempt) {
        result = copy_file_locked(source, destination);
        if (result == ESP_OK) {
            break;
        }
        if (attempt + 1 < attempts) {
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }
    return result;
}

bool cabrillo_ensure_header_locked(const std::string& path,
                                   const std::string& mycall,
                                   const std::string& location) {
    struct stat info {};
    if (stat(path.c_str(), &info) == 0 && info.st_size > 0) {
        return true;
    }

    FILE* file = fopen(path.c_str(), "wb");
    if (!file) {
        return false;
    }
    bool ok = fprintf(file, "START-OF-LOG: 3.0\n") >= 0;
    ok = ok && fprintf(file, "CREATED-BY: QC705\n") >= 0;
    ok = ok && fprintf(file, "CONTEST: ARRL-FIELD-DAY\n") >= 0;
    ok = ok && fprintf(file, "CALLSIGN: %s\n", mycall.c_str()) >= 0;
    ok = ok && fprintf(file, "CATEGORY-OPERATOR: SINGLE-OP\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-TRANSMITTER: ONE\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-ASSISTED: NON-ASSISTED\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-BAND: ALL\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-MODE: MIXED\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-POWER: LOW\n") >= 0;
    ok = ok && fprintf(file, "CATEGORY-STATION: PORTABLE\n") >= 0;
    ok = ok && fprintf(file, "LOCATION: %s\n", location.c_str()) >= 0;
    ok = ok && fprintf(file, "OPERATORS: %s\n", mycall.c_str()) >= 0;
    ok = ok && fprintf(file, "END-OF-LOG:\n") >= 0;
    ok = ok && sync_file(file);
    if (fclose(file) != 0) {
        ok = false;
    }
    return ok;
}

bool cabrillo_truncate_end_marker(FILE* file) {
    if (!file || fseek(file, 0, SEEK_END) != 0) {
        return false;
    }
    const long file_end = ftell(file);
    if (file_end <= 0) {
        return false;
    }

    constexpr long kMaxTail = 256;
    const long tail_start = file_end > kMaxTail ? file_end - kMaxTail : 0;
    if (fseek(file, tail_start, SEEK_SET) != 0) {
        return false;
    }

    std::string tail(static_cast<size_t>(file_end - tail_start), '\0');
    tail.resize(fread(tail.data(), 1, tail.size(), file));
    size_t line_end = tail.size();
    while (line_end > 0 && (tail[line_end - 1] == '\n' || tail[line_end - 1] == '\r')) {
        --line_end;
    }
    if (line_end == 0) {
        return false;
    }
    size_t line_start = tail.rfind('\n', line_end - 1);
    line_start = line_start == std::string::npos ? 0 : line_start + 1;
    if (tail.substr(line_start, line_end - line_start) != "END-OF-LOG:") {
        return false;
    }

    const int descriptor = fileno(file);
    const long truncate_at = tail_start + static_cast<long>(line_start);
    return descriptor >= 0 && ftruncate(descriptor, truncate_at) == 0 &&
           fseek(file, 0, SEEK_END) == 0;
}

void storage_event_callback(tinyusb_msc_storage_handle_t,
                            tinyusb_msc_event_t* event,
                            void*) {
    if (!event) {
        return;
    }

    s_mount_transition_point = event->mount_point;
    switch (event->id) {
        case TINYUSB_MSC_EVENT_MOUNT_START:
            s_mount_transition_result = MountTransitionResult::STARTED;
            break;
        case TINYUSB_MSC_EVENT_MOUNT_COMPLETE:
            s_mount_transition_result = MountTransitionResult::COMPLETE;
            break;
        case TINYUSB_MSC_EVENT_MOUNT_FAILED:
        case TINYUSB_MSC_EVENT_FORMAT_REQUIRED:
        case TINYUSB_MSC_EVENT_FORMAT_FAILED:
            s_mount_transition_result = MountTransitionResult::FAILED;
            break;
    }
    ESP_LOGI(TAG, "MSC storage event=%d mount=%d", event->id, event->mount_point);
}

esp_err_t set_storage_mount_point_locked(tinyusb_msc_mount_point_t mount_point) {
    s_mount_transition_result = MountTransitionResult::NONE;
    s_mount_transition_point = mount_point;

    const esp_err_t err = tinyusb_msc_set_storage_mount_point(s_msc_storage, mount_point);
    if (err != ESP_OK) {
        return err;
    }
    if (s_mount_transition_result != MountTransitionResult::COMPLETE ||
        s_mount_transition_point != mount_point) {
        ESP_LOGE(TAG, "FATFS ownership transition to mount=%d was not confirmed",
                 mount_point);
        return ESP_FAIL;
    }
    return ESP_OK;
}

}  // namespace

struct StorageStream {
    FILE* file = nullptr;
};

esp_err_t storage_service_init() {
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_buffer);
        if (!s_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    StorageGuard guard;
    if (!guard.held()) {
        return ESP_FAIL;
    }
    if (s_owner != StorageOwner::UNAVAILABLE) {
        return ESP_OK;
    }

    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, kPartitionLabel);
    if (!partition) {
        ESP_LOGE(TAG, "FATFS partition '%s' not found", kPartitionLabel);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = wl_mount(partition, &s_wl_handle);
    if (err != ESP_OK) {
        s_wl_handle = WL_INVALID_HANDLE;
        ESP_LOGE(TAG, "wear levelling mount failed: %s", esp_err_to_name(err));
        return err;
    }

    tinyusb_msc_driver_config_t driver_config = {};
    driver_config.user_flags.auto_mount_off = 1;
    driver_config.callback = storage_event_callback;
    err = tinyusb_msc_install_driver(&driver_config);
    if (err != ESP_OK) {
        wl_unmount(s_wl_handle);
        s_wl_handle = WL_INVALID_HANDLE;
        ESP_LOGE(TAG, "MSC driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    tinyusb_msc_storage_config_t storage_config = {};
    storage_config.medium.wl_handle = s_wl_handle;
    storage_config.fat_fs.base_path = const_cast<char*>(kBasePath);
    storage_config.fat_fs.config.format_if_mount_failed = true;
    storage_config.fat_fs.config.max_files = 8;
    storage_config.fat_fs.config.allocation_unit_size = 4096;
    storage_config.mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP;
    err = tinyusb_msc_new_storage_spiflash(&storage_config, &s_msc_storage);
    if (err != ESP_OK) {
        tinyusb_msc_uninstall_driver();
        wl_unmount(s_wl_handle);
        s_wl_handle = WL_INVALID_HANDLE;
        s_msc_storage = nullptr;
        ESP_LOGE(TAG, "FATFS storage creation failed: %s", esp_err_to_name(err));
        return err;
    }

    s_owner = StorageOwner::FIRMWARE;
    ESP_LOGI(TAG, "initialized owner: firmware owns %s on partition '%s'",
             kBasePath, kPartitionLabel);
    return ESP_OK;
}

StorageOwner storage_service_owner() {
    StorageGuard guard;
    return guard.held() ? s_owner : StorageOwner::UNAVAILABLE;
}

bool storage_service_firmware_available() {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked();
}

bool storage_service_usb_drive_enabled() {
    StorageGuard guard;
    return guard.held() && s_owner == StorageOwner::USB_HOST;
}

size_t storage_service_open_stream_count() {
    StorageGuard guard;
    return guard.held() ? s_open_streams : 0;
}

esp_err_t storage_service_set_usb_drive_enabled(bool enabled) {
    StorageGuard guard;
    if (!guard.held() || !s_msc_storage) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enabled) {
        if (s_owner == StorageOwner::USB_HOST) {
            return ESP_OK;
        }
        if (s_owner != StorageOwner::FIRMWARE) {
            return ESP_ERR_INVALID_STATE;
        }
        if (s_open_streams != 0) {
            ESP_LOGW(TAG, "USB Drive blocked by %u open storage stream(s)",
                     static_cast<unsigned>(s_open_streams));
            return ESP_ERR_INVALID_STATE;
        }

        s_owner = StorageOwner::TRANSITION;
        esp_err_t err = set_storage_mount_point_locked(TINYUSB_MSC_STORAGE_MOUNT_USB);
        if (err != ESP_OK) {
            s_owner = s_mount_transition_result == MountTransitionResult::NONE
                          ? StorageOwner::FIRMWARE
                          : StorageOwner::UNAVAILABLE;
            return err;
        }

        const tinyusb_config_t tinyusb_config = TINYUSB_DEFAULT_CONFIG();
        err = tinyusb_driver_install(&tinyusb_config);
        if (err != ESP_OK) {
            const esp_err_t rollback =
                set_storage_mount_point_locked(TINYUSB_MSC_STORAGE_MOUNT_APP);
            s_owner = rollback == ESP_OK ? StorageOwner::FIRMWARE : StorageOwner::UNAVAILABLE;
            return err;
        }

        s_tinyusb_installed = true;
        s_owner = StorageOwner::USB_HOST;
        ESP_LOGI(TAG, "USB Drive ON: PC owns FATFS");
        return ESP_OK;
    }

    if (s_owner == StorageOwner::FIRMWARE) {
        return ESP_OK;
    }
    if (s_owner != StorageOwner::USB_HOST || !s_tinyusb_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    s_owner = StorageOwner::TRANSITION;
    esp_err_t err = tinyusb_driver_uninstall();
    if (err != ESP_OK) {
        s_owner = StorageOwner::USB_HOST;
        return err;
    }
    s_tinyusb_installed = false;

    err = set_storage_mount_point_locked(TINYUSB_MSC_STORAGE_MOUNT_APP);
    if (err != ESP_OK) {
        s_owner = StorageOwner::UNAVAILABLE;
        return err;
    }

    s_owner = StorageOwner::FIRMWARE;
    ESP_LOGI(TAG, "USB Drive OFF: firmware owns FATFS");
    return ESP_OK;
}

bool storage_file_exists(const std::string& name) {
    StorageGuard guard;
    std::string path;
    struct stat info {};
    return guard.held() && firmware_owns_storage_locked() && build_path(name, path) &&
           stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

bool storage_file_remove(const std::string& name) {
    StorageGuard guard;
    std::string path;
    return guard.held() && firmware_owns_storage_locked() && build_path(name, path) &&
           unlink(path.c_str()) == 0;
}

bool storage_file_list(std::vector<std::string>& files) {
    StorageGuard guard;
    return guard.held() && list_files_locked(files);
}

bool storage_file_read_text(const std::string& name, std::string& content) {
    StorageGuard guard;
    content.clear();
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() || !build_path(name, path)) {
        return false;
    }

    FILE* file = fopen(path.c_str(), "rb");
    if (!file) {
        return false;
    }
    char buffer[512];
    while (true) {
        const size_t count = fread(buffer, 1, sizeof(buffer), file);
        if (count > 0) {
            content.append(buffer, count);
        }
        if (count < sizeof(buffer)) {
            break;
        }
    }
    const bool ok = ferror(file) == 0;
    fclose(file);
    return ok;
}

bool storage_file_write_atomic(const std::string& name, const std::string& content) {
    StorageGuard guard;
    return guard.held() && write_atomic_locked(name, content);
}

bool storage_file_append(const std::string& name,
                         const std::string& content,
                         const std::string& header_if_new,
                         bool sync_to_flash) {
    StorageGuard guard;
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() || !build_path(name, path)) {
        return false;
    }

    bool need_header = false;
    if (!header_if_new.empty()) {
        struct stat info {};
        need_header = stat(path.c_str(), &info) != 0 || info.st_size == 0;
    }

    FILE* file = fopen(path.c_str(), "ab");
    if (!file) {
        return false;
    }
    bool ok = !need_header ||
              fwrite(header_if_new.data(), 1, header_if_new.size(), file) ==
                  header_if_new.size();
    ok = ok && (content.empty() ||
                fwrite(content.data(), 1, content.size(), file) == content.size());
    if (ok && sync_to_flash) {
        ok = sync_file(file);
    }
    if (fclose(file) != 0) {
        ok = false;
    }
    return ok;
}

bool storage_file_append_cabrillo(const std::string& mycall,
                                  const std::string& location,
                                  const std::string& qso_line) {
    StorageGuard guard;
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() ||
        !build_path("fieldday.txt", path) ||
        !cabrillo_ensure_header_locked(path, mycall, location)) {
        return false;
    }

    FILE* file = fopen(path.c_str(), "r+b");
    if (!file) {
        file = fopen(path.c_str(), "a+b");
    }
    if (!file) {
        return false;
    }

    cabrillo_truncate_end_marker(file);
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return false;
    }
    const long end = ftell(file);
    if (end > 0 && fseek(file, -1, SEEK_END) == 0) {
        const int last = fgetc(file);
        fseek(file, 0, SEEK_END);
        if (last != '\n') {
            fputc('\n', file);
        }
    }
    bool ok = fprintf(file, "%s\nEND-OF-LOG:\n", qso_line.c_str()) >= 0;
    ok = ok && sync_file(file);
    if (fclose(file) != 0) {
        ok = false;
    }
    return ok;
}

StorageStream* storage_stream_open(const std::string& name, StorageOpenMode mode) {
    StorageGuard guard;
    std::string path;
    if (!guard.held() || !firmware_owns_storage_locked() || !build_path(name, path)) {
        return nullptr;
    }

    const char* open_mode = "rb";
    if (mode == StorageOpenMode::WRITE_TRUNCATE) {
        open_mode = "wb";
    } else if (mode == StorageOpenMode::APPEND) {
        open_mode = "ab";
    }

    StorageStream* stream = new (std::nothrow) StorageStream;
    if (!stream) {
        return nullptr;
    }
    stream->file = fopen(path.c_str(), open_mode);
    if (!stream->file) {
        delete stream;
        return nullptr;
    }
    ++s_open_streams;
    return stream;
}

size_t storage_stream_read(StorageStream* stream, void* data, size_t size) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return 0;
    }
    return fread(data, 1, size, stream->file);
}

bool storage_stream_read_line(StorageStream* stream, char* line, size_t line_size) {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked() && stream && stream->file &&
           line && line_size > 0 && fgets(line, static_cast<int>(line_size), stream->file);
}

size_t storage_stream_write(StorageStream* stream, const void* data, size_t size) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return 0;
    }
    return fwrite(data, 1, size, stream->file);
}

bool storage_stream_seek(StorageStream* stream, long offset, int whence) {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked() && stream && stream->file &&
           fseek(stream->file, offset, whence) == 0;
}

long storage_stream_tell(StorageStream* stream) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return -1;
    }
    return ftell(stream->file);
}

long storage_stream_size(StorageStream* stream) {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked() || !stream || !stream->file) {
        return -1;
    }
    const long current = ftell(stream->file);
    if (current < 0 || fseek(stream->file, 0, SEEK_END) != 0) {
        return -1;
    }
    const long size = ftell(stream->file);
    fseek(stream->file, current, SEEK_SET);
    return size;
}

bool storage_stream_sync(StorageStream* stream) {
    StorageGuard guard;
    return guard.held() && firmware_owns_storage_locked() && stream && stream->file &&
           sync_file(stream->file);
}

void storage_stream_close(StorageStream* stream) {
    if (!stream) {
        return;
    }
    StorageGuard guard;
    if (guard.held()) {
        if (stream->file) {
            fclose(stream->file);
            stream->file = nullptr;
        }
        if (s_open_streams > 0) {
            --s_open_streams;
        }
    }
    delete stream;
}

bool storage_sync_station_from_sd() {
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked()) {
        return false;
    }
    if (s_station_sync_attempted) {
        return true;
    }
    s_station_sync_attempted = true;

    if (mount_sd_locked() != ESP_OK) {
        ESP_LOGI(TAG, "SD not mounted; using internal Station.txt");
        return false;
    }

    const std::string source = std::string(kSdBasePath) + "/" + kStationFile;
    FILE* file = fopen(source.c_str(), "rb");
    if (!file) {
        ESP_LOGI(TAG, "Station.txt not found on SD");
        unmount_sd_locked();
        return false;
    }

    std::string content;
    char buffer[512];
    while (true) {
        const size_t count = fread(buffer, 1, sizeof(buffer), file);
        if (count > 0) {
            content.append(buffer, count);
        }
        if (count < sizeof(buffer)) {
            break;
        }
    }
    const bool read_ok = ferror(file) == 0;
    fclose(file);
    const bool write_ok = read_ok && write_atomic_locked(kStationFile, content);
    unmount_sd_locked();

    ESP_LOGI(TAG, "%s Station.txt from SD", write_ok ? "Imported" : "Failed to import");
    return write_ok;
}

StorageCopyResult storage_copy_all_to_sd(const std::string& priority_file) {
    StorageCopyResult result {};
    StorageGuard guard;
    if (!guard.held() || !firmware_owns_storage_locked()) {
        result.err = ESP_ERR_INVALID_STATE;
        result.missed_count = 1;
        return result;
    }

    std::vector<std::string> files;
    if (!list_files_locked(files)) {
        result.err = ESP_FAIL;
        result.missed_count = 1;
        return result;
    }
    if (mount_sd_locked() != ESP_OK) {
        result.err = ESP_FAIL;
        result.missed_count = std::max(1, static_cast<int>(files.size()));
        return result;
    }

    std::sort(files.begin(), files.end());
    for (const std::string& name : files) {
        std::string source;
        if (!build_path(name, source)) {
            ++result.missed_count;
            continue;
        }
        const std::string destination = std::string(kSdBasePath) + "/" + name;
        const int attempts = name == priority_file ? 6 : 5;
        if (copy_file_retry_locked(source, destination, attempts) == ESP_OK) {
            ++result.copied_count;
        } else {
            ++result.missed_count;
        }
    }

    unmount_sd_locked();
    result.err = result.missed_count == 0 ? ESP_OK : ESP_FAIL;
    return result;
}
