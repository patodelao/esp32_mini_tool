// Nodo sensor ESP8266 (NodeMCU v3) - aire (DHT22) + suelo (HW-103) + WiFi + MQTT.
//
// Publica al broker del home-lab siguiendo las convenciones del minitool,
// para aparecer solo en sus tools Sensores y Nodos:
//
//   labo/sensor/<NODE_ID>/temp   -> temperatura aire, C, texto     [retain]
//   labo/sensor/<NODE_ID>/hum    -> humedad aire, %, texto          [retain]
//   labo/sensor/<NODE_ID>/suelo  -> humedad suelo, %, texto          [retain]
//   labo/nodo/<NODE_ID>/status   -> "online" / "offline"            [retain]
//
// El estado usa Last-Will (LWT): si el nodo cae, el broker publica "offline".
//
// Sensores (ambos opcionales via ENABLE_* mas abajo):
//   - Aire:  AM2302 (DHT22) en modulo 140C80 -> digital en D2 (GPIO4).
//   - Suelo: sonda resistiva de 2 patas + modulo HW-103 -> analogico en A0.
//            Usamos la salida A0 del modulo (D0 es solo umbral on/off).
//
// Conexionado (NodeMCU v3):
//   DHT22  VCC->3V3   GND->GND   DATA->D2
//   HW-103 VCC->3V3   GND->GND   A0 (del modulo)->A0 (del NodeMCU)
//   (NO usar VU/VIN=5V para nada: el GPIO/ADC del ESP8266 es de 3.3V)
//
// OJO sensor de suelo resistivo: se corroe si esta siempre energizado. Mejora
// facil (no implementada por defecto): alimentar su VCC desde un GPIO y
// encenderlo solo un instante antes de medir.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- Configuracion: EDITAR ANTES DE FLASHEAR -------------------------------
#define WIFI_SSID     "DonPatoysusecuaces"
#define WIFI_PASSWORD "Armando1910"

// Identificador del nodo (sin espacios ni '/'). Aparece como labo/nodo/<NODE_ID>
// y labo/sensor/<NODE_ID>/... en las tools del minitool.
#define NODE_ID       "sala"

// Sensores activos (1 = si, 0 = no). Por defecto ambos; pon uno en 0 para un
// nodo solo-aire o solo-suelo sin tocar mas codigo.
#define ENABLE_DHT    1     // aire: DHT22 en D2 -> temp + hum
#define ENABLE_SUELO  1     // suelo: modulo HW-103 en A0

// --- Hardware --------------------------------------------------------------
#define DHT_PIN   D2        // GPIO4
#define DHT_TYPE  DHT22     // AM2302 = DHT22
#define SUELO_PIN A0        // unica entrada analogica del ESP8266

// Calibracion del suelo (valor crudo del ADC, 0..1023). En el HW-103 resistivo:
//   seco / al aire -> lectura ALTA     en agua -> lectura BAJA
// Ajusta estos dos numeros mirando el "raw" que imprime el monitor serie:
// deja la sonda al aire y anota SECO_RAW; sumergela en agua y anota AGUA_RAW.
#define SUELO_SECO_RAW  820
#define SUELO_AGUA_RAW  380

// --- MQTT ------------------------------------------------------------------
#define MQTT_BROKER   "broker.hivemq.com"
#define MQTT_PORT     1883

#define TOPIC_STATUS  "labo/nodo/"   NODE_ID "/status"
#define TOPIC_TEMP    "labo/sensor/" NODE_ID "/temp"
#define TOPIC_HUM     "labo/sensor/" NODE_ID "/hum"
#define TOPIC_SUELO   "labo/sensor/" NODE_ID "/suelo"

// Cada cuanto leer y publicar. El DHT22 admite como maximo 1 lectura / 2 s.
static const unsigned long PUBLICAR_MS = 10000;

// --- Objetos globales ------------------------------------------------------
#if ENABLE_DHT
DHT dht(DHT_PIN, DHT_TYPE);
#endif
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

static unsigned long ultimaPublicacion = 0;
static char clientId[32];   // se arma con el chip id para ser unico en el broker

// Parpadeo corto del LED integrado (GPIO2, activo en bajo) como senal de vida.
static void parpadeo() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(20);
  digitalWrite(LED_BUILTIN, HIGH);
}

#if ENABLE_SUELO
// Promedio de varias lecturas del ADC para reducir ruido.
static int leer_suelo_raw() {
  const int N = 10;
  long acc = 0;
  for (int i = 0; i < N; i++) { acc += analogRead(SUELO_PIN); delay(5); }
  return (int)(acc / N);
}

// Convierte el crudo a % de humedad (0 = seco, 100 = saturado), con recorte.
static float suelo_pct(int raw) {
  float pct = 100.0f * (float)(SUELO_SECO_RAW - raw) /
                       (float)(SUELO_SECO_RAW - SUELO_AGUA_RAW);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}
#endif

static void conectarWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("WiFi: conectando a \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
  }
  Serial.printf("\nWiFi: conectado. IP %s  RSSI %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

static void conectarMqtt() {
  while (!mqtt.connected()) {
    Serial.printf("MQTT: conectando a %s:%d como %s ...\n",
                  MQTT_BROKER, MQTT_PORT, clientId);

    // connect(id, willTopic, willQoS, willRetain, willMessage): el LWT deja
    // "offline" retenido si el nodo se desconecta sin avisar.
    bool ok = mqtt.connect(clientId,
                           TOPIC_STATUS, 1, true, "offline");
    if (ok) {
      Serial.println("MQTT: conectado");
      mqtt.publish(TOPIC_STATUS, "online", true);
    } else {
      Serial.printf("MQTT: fallo (rc=%d), reintento en 3 s\n", mqtt.state());
      delay(3000);
    }
  }
}

static void publicarLectura() {
  char buf[16];

#if ENABLE_DHT
  float humedad = dht.readHumidity();
  float tempC   = dht.readTemperature();
  if (isnan(humedad) || isnan(tempC)) {
    Serial.println("[ERROR] lectura DHT22 fallida (revisa cableado/pin D2)");
  } else {
    snprintf(buf, sizeof(buf), "%.1f", tempC);
    mqtt.publish(TOPIC_TEMP, buf, true);
    snprintf(buf, sizeof(buf), "%.1f", humedad);
    mqtt.publish(TOPIC_HUM, buf, true);
    Serial.printf("Aire  -> T: %.1f C   HR: %.1f %%\n", tempC, humedad);
  }
#endif

#if ENABLE_SUELO
  int raw = leer_suelo_raw();
  float pct = suelo_pct(raw);
  snprintf(buf, sizeof(buf), "%.0f", pct);
  mqtt.publish(TOPIC_SUELO, buf, true);
  Serial.printf("Suelo -> %.0f %% (raw %d)\n", pct, raw);
#endif

  parpadeo();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== ESP8266 sensor: aire (DHT22) + suelo (HW-103) + MQTT ===");
  Serial.printf("Nodo: %s\n", NODE_ID);
#if ENABLE_DHT
  Serial.printf("  aire  -> %s | %s (D2)\n", TOPIC_TEMP, TOPIC_HUM);
#endif
#if ENABLE_SUELO
  Serial.printf("  suelo -> %s (A0)\n", TOPIC_SUELO);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // apagado

#if ENABLE_DHT
  dht.begin();
#endif

  // Client id unico en el broker publico (compartido con todo internet).
  snprintf(clientId, sizeof(clientId), "esp8266-%s-%06X", NODE_ID, ESP.getChipId());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  conectarWifi();
  conectarMqtt();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) conectarWifi();
  if (!mqtt.connected())            conectarMqtt();
  mqtt.loop();

  unsigned long ahora = millis();
  if (ahora - ultimaPublicacion >= PUBLICAR_MS) {
    ultimaPublicacion = ahora;
    publicarLectura();
  }
}
