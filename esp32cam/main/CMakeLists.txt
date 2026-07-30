idf_component_register(
    SRCS "main.c" "ota_web.c" "camera.c" "dht.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_event mqtt driver esp_http_server app_update esp_rom
)
# esp32-camera NO va en REQUIRES: el component manager lo agrega solo como
# dependencia de 'main' a partir de idf_component.yml. Listarlo a mano (y con el
# nombre sin namespace) hace fallar la resolución.
