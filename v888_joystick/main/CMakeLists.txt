idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_i2c
        esp_driver_gpio
    REQUIRES 
        esp_wifi 
        esp_event 
        nvs_flash 
        esp_netif 
        lwip driver 
        esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>
)

