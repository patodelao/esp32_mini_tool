# Este nodo hospeda el broker MQTT del home-lab (mqtt_broker.c sobre el
# componente 'mongoose'). Ya no usa el cliente esp-mqtt (se quitó 'mqtt' de
# REQUIRES); el resto de la flota se conecta a él por TCP.
idf_component_register(
    SRCS "main.c" "ota_web.c" "mqtt_broker.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_event driver esp_http_server app_update mongoose
)
