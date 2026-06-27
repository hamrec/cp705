idf_component_register(
    SRCS "external_rtc.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board_cardputer_adv esp_driver_i2c
)
