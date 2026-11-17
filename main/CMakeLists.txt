idf_component_register(SRCS "main.c" "motor_logic.cpp" "wifi_server.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver esp_driver_ledc esp_driver_gpio esp_netif esp_wifi esp_event nvs_flash esp_http_server json esp_timer ESP32_VL53L0X)