idf_component_register(
    SRCS "main.c" "ota_web.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_event mqtt driver esp_http_server app_update
)
