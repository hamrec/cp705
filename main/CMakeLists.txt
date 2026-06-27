idf_component_register(
    SRCS "main.cpp"
         "autoseq.cpp"
         "audio_source.cpp"
         "core_api.cpp"
         "gps.cpp"
         "radio_control.cpp"
         "radio_control_ic705.cpp"
         "wifi_manager.cpp"
         "ic705_netctrl.cpp"
         "ft8_audio_pipeline.cpp"
         "stream_wifi_ic705.cpp"
         "dds_q15.cpp"
         "resample.cpp"
    INCLUDE_DIRS "."
    REQUIRES ui M5Unified M5GFX M5Cardputer board_cardputer_adv ft8_lib
             esp_wifi esp_netif mdns lwip nvs_flash
             storage_service external_rtc efuse
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
