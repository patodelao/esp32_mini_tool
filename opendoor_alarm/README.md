# opendoor_alarm

Proyecto ESP-IDF para alarma de puerta de refrigerador con bajo consumo.

## Hardware

- Sensor magnético: GPIO 10 (entrada con pull-up, activo en nivel alto).
- Buzzer pasivo: GPIO 5 (LEDC, tono 2.5 kHz).
- LED de alarma:
  - Intenta usar WS2812 integrado en GPIO48 (ESP32-S3) vía RMT.
  - Fallback por pines RGB: R=GPIO2, G=GPIO3, B=GPIO4.

## Flujo

1. Inicializa GPIO, buzzer y LED.
2. Conecta a Wi-Fi y a MQTT (`mqtt://broker.hivemq.com`).
3. Si la puerta está abierta al arrancar, publica `ABIERTO` en `proyectos/casa/refri/puerta` (QoS 1).
4. Mientras la puerta siga abierta:
   - desde los 20 s: LED alternando rojo/verde.
   - desde los 30 s: buzzer intermitente sincronizado con el parpadeo.
   - cada 5 s: publica heartbeat `ABIERTO` por MQTT.
5. Al detectar cierre, apaga salidas y confirma cierre estable por 3 s.
6. Si se confirma, publica `CERRADO` (QoS 1). Si se reabre antes, vuelve a alarma.
7. Configura wakeup por GPIO 10 y entra en deep sleep.

## Configuración rápida

Edita en `main/main.c`:

- `WIFI_SSID`
- `WIFI_PASSWORD`
- Pines si tu placa usa otro mapeo.

## Compilar y cargar

```bash
idf.py set-target esp32s3
idf.py fullclean
idf.py build
idf.py -p <PUERTO> flash monitor
```
