idf_component_register(
    SRCS "storage_service.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        driver
        esp_partition
        espressif__esp_tinyusb
        fatfs
        freertos
        sdmmc
        wear_levelling
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
