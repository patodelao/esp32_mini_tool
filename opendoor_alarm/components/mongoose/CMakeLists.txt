# Mongoose vendorizado (cesanta/mongoose 7.18), usado solo por su broker MQTT.
#
# REQUIRES sale de lo que mongoose.c/.h referencian en el arch ESP32:
#   - lwip           : sockets / netdb.h (Mongoose usa la pila del OS, MG_ENABLE_TCPIP=0)
#   - esp_timer      : esp_timer_get_time (base de tiempo de Mongoose)
#   - app_update     : esp_ota_* (mongoose.h los incluye siempre en el bloque ESP32)
#   - esp_hw_support : esp_random (fuente de aleatoriedad de mg_random)
idf_component_register(
    SRCS "mongoose.c"
    INCLUDE_DIRS "."
    REQUIRES lwip esp_timer app_update esp_hw_support
)
