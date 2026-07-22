// Nodo sensor ESP8266 (NodeMCU v3) - DHT22 / AM2302 + WiFi + MQTT.
//
// Publica temperatura y humedad al broker del home-lab siguiendo las
// convenciones del minitool, para aparecer solo en sus tools Sensores y Nodos
// sin cambios del lado del minitool:
//
//   labo/sensor/<NODE_ID>/temp   -> temperatura en C, texto (ej. "23.4")  [retain]
//   labo/sensor/<NODE_ID>/hum    -> humedad relativa %, texto (ej. "51.2") [retain]
//   labo/nodo/<NODE_ID>/status   -> "online" / "offline"                   [retain]
//
// El estado usa Last-Will (LWT): si el nodo cae (corte de energia o WiFi),
// el broker publica "offline" por el automaticamente.
//
// Sensor: AM2302 (DHT22) en modulo 140C80 (ya trae pull-up y capacitor).
// Conexionado (NodeMCU v3):
//   VCC  -> 3V3   (NO usar VU/VIN: son 5V y danarian el GPIO)
//   GND  -> GND
//   DATA -> D2 (GPIO4)

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- Configuracion: EDITAR ANTES DE FLASHEAR -------------------------------
#define WIFI_SSID     "DonPatoysusecuaces"
#define WIFI_PASSWORD "Armando1910"

// Identificador del nodo. Aparece como labo/nodo/<NODE_ID> en la tool Nodos
// y como labo/sensor/<NODE_ID>/... en la tool Sensores. Sin espacios ni '/'.
#define NODE_ID       "sala"

// --- Hardware --------------------------------------------------------------
#define DHT_PIN   D2        // GPIO4
#define DHT_TYPE  DHT22     // AM2302 = DHT22

// --- MQTT ------------------------------------------------------------------
#define MQTT_BROKER   "broker.hivemq.com"
#define MQTT_PORT     1883

// Topics derivados de NODE_ID (concatenacion de literales en preprocesador).
#define TOPIC_STATUS  "labo/nodo/"   NODE_ID "/status"
#define TOPIC_TEMP    "labo/sensor/" NODE_ID "/temp"
#define TOPIC_HUM     "labo/sensor/" NODE_ID "/hum"

// Cada cuanto leer y publicar. El DHT22 admite como maximo 1 lectura / 2 s.
static const unsigned long PUBLICAR_MS = 10000;

// --- Objetos globales ------------------------------------------------------
DHT dht(DHT_PIN, DHT_TYPE);
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
      // Anunciar presencia: "online" retenido en el topic de estado.
      mqtt.publish(TOPIC_STATUS, "online", true);
    } else {
      Serial.printf("MQTT: fallo (rc=%d), reintento en 3 s\n", mqtt.state());
      delay(3000);
    }
  }
}

static void publicarLectura() {
  float humedad = dht.readHumidity();
  float tempC   = dht.readTemperature();

  if (isnan(humedad) || isnan(tempC)) {
    Serial.println("[ERROR] lectura fallida del DHT22 (revisa cableado/pin)");
    return;
  }

  char buf[16];

  snprintf(buf, sizeof(buf), "%.1f", tempC);
  mqtt.publish(TOPIC_TEMP, buf, true);       // retain: la tool ve el ultimo valor

  snprintf(buf, sizeof(buf), "%.1f", humedad);
  mqtt.publish(TOPIC_HUM, buf, true);

  Serial.printf("Publicado -> T: %.1f C   HR: %.1f %%\n", tempC, humedad);
  parpadeo();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== ESP8266 sensor: DHT22 + MQTT ===");
  Serial.printf("Nodo: %s   pin: D2 (GPIO%d)\n", NODE_ID, DHT_PIN);
  Serial.printf("Topics -> %s | %s | %s\n", TOPIC_TEMP, TOPIC_HUM, TOPIC_STATUS);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // apagado

  dht.begin();

  // Client id unico en el broker publico (compartido con todo internet).
  snprintf(clientId, sizeof(clientId), "esp8266-%s-%06X", NODE_ID, ESP.getChipId());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  conectarWifi();
  conectarMqtt();
}

void loop() {
  // Mantener WiFi y MQTT vivos.
  if (WiFi.status() != WL_CONNECTED) conectarWifi();
  if (!mqtt.connected())            conectarMqtt();
  mqtt.loop();

  unsigned long ahora = millis();
  if (ahora - ultimaPublicacion >= PUBLICAR_MS) {
    ultimaPublicacion = ahora;
    publicarLectura();
  }
}
