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
// Riego (umbral configurado desde la UI del minitool):
//   labo/config/<NODE_ID>/suelo/umbral      <- umbral % (retenido); 0 = no vigilar
//   labo/config/<NODE_ID>/suelo/histeresis  <- margen % de recuperacion (retenido)
//   labo/alerta/<NODE_ID>                   -> alerta JSON si el suelo esta seco
//
// El nodo alerta por su cuenta (aunque el minitool este apagado): confirma con
// varias lecturas seguidas, usa histeresis para no oscilar, y repite el aviso
// cada 30 min mientras siga seco. Tambien avisa si la sonda parece desconectada.
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
//   HW-103 VCC->D5    GND->GND   A0 (del modulo)->A0 (del NodeMCU)
//          (VCC en D5, no en 3V3: el nodo enciende la sonda solo al medir)
//   (NO usar VU/VIN=5V para nada: el GPIO/ADC del ESP8266 es de 3.3V)
//
// OJO sensor de suelo resistivo: se corroe si esta siempre energizado. Por eso
// SUELO_VCC_PIN permite alimentarlo desde un GPIO y encenderlo solo un instante
// antes de medir (ver mas abajo; necesita mover un cable).
//
// Nodo pensado para quedar instalado y NO volver a tocarse fisicamente:
//   - OTA: se reflashea por WiFi (pio run -t upload --upload-port <ip>).
//   - Telemetria: publica rssi / uptime / heap como sensores mas del minitool.
//   - Comandos: escucha labo/nodo/<NODE_ID>/cmd (leer, reset, cal on|off).
//   - Reconexion no bloqueante y reinicio de rescate si se queda sin red.

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- Configuracion: EDITAR ANTES DE FLASHEAR -------------------------------
//
// Las credenciales NO van aca: viven en secrets.ini (ignorado por git) y
// PlatformIO las inyecta como -D al compilar. Copia secrets.ini.example a
// secrets.ini y completa el tuyo. Los valores de abajo son solo el respaldo
// para que el archivo compile si alguien lo abre suelto, y no sirven para
// conectarse a nada.
#ifndef WIFI_SSID
#define WIFI_SSID     "cambiame"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "cambiame"
#endif

// Identificador del nodo (sin espacios ni '/'). Aparece como labo/nodo/<NODE_ID>
// y labo/sensor/<NODE_ID>/... en las tools del minitool.
#define NODE_ID       "pieza"

// Sensores activos (1 = si, 0 = no). Por defecto ambos; pon uno en 0 para un
// nodo solo-aire o solo-suelo sin tocar mas codigo.
#define ENABLE_DHT    1     // aire: DHT22 en D2 -> temp + hum
#define ENABLE_SUELO  1     // suelo: modulo HW-103 en A0

// Clave para actualizar por WiFi (OTA). Tambien viene de secrets.ini.
#ifndef OTA_PASSWORD
#define OTA_PASSWORD  "cambiame"
#endif

// --- Hardware --------------------------------------------------------------
#define DHT_PIN   D2        // GPIO4
#define DHT_TYPE  DHT22     // AM2302 = DHT22
#define SUELO_PIN A0        // unica entrada analogica del ESP8266

// Alimentacion de la sonda de suelo. La sonda resistiva se CORROE si esta
// siempre energizada (dura semanas en vez de meses). Si mueves el VCC del
// modulo HW-103 de 3V3 al pin D5, pon aqui 14 y el nodo lo enciende solo el
// instante de medir.
//   -1 = sonda siempre alimentada (cableado original)
//   14 = alimentada solo al medir, cable VCC en D5 (recomendado)
// Va el numero de GPIO, no "D5": esto lo evalua el preprocesador.
// Ojo: si el modulo trae LED de encendido puede pasar de los ~12 mA que da un
// GPIO; en ese caso desuelda el LED o usa un transistor pequeno.
//
// ESTA EN 14: el VCC del modulo HW-103 TIENE que ir a D5. Si lo dejas en 3V3,
// la sonda queda sin alimentar al medir y el nodo avisa "sonda sin
// alimentacion" (ver SUELO_RAW_MINIMO) en vez de inventar un 100 % de humedad.
#define SUELO_VCC_PIN   14
#define SUELO_ESTABILIZAR_MS  300   // espera tras energizar, antes de leer

// Calibracion del suelo (valor crudo del ADC, 0..1023). En el HW-103 resistivo:
//   seco -> lectura ALTA     en agua -> lectura BAJA
//
// SECO_RAW conviene medirlo con la sonda en TIERRA SECA, no al aire. Al aire la
// lectura se va al tope del ADC (1023) y eso trae dos problemas: se desperdicia
// escala (la tierra seca real nunca llega ahi, asi que los % quedan comprimidos)
// y encima no queda margen para distinguir "tierra seca" de "sonda al aire", que
// es justo lo que necesita SUELO_RAW_DESCONECTADO.
//
// Medidos por el usuario: en agua 520, al aire 1023.
#define SUELO_SECO_RAW  1023
#define SUELO_AGUA_RAW  520

// Deteccion de sonda en falla. Los dos umbrales se DERIVAN de la calibracion:
// si se fijan a mano es facil que se solapen con las lecturas normales y el
// nodo confunda tierra seca con sonda rota (deja de publicar y no te enteras).
//
// Arriba de seco + margen: sonda al aire o cable de senal suelto.
// Ojo: si SUELO_SECO_RAW esta pegado al tope del ADC (o sea, sin calibrar),
// este chequeo queda inalcanzable a proposito: sin calibrar no hay forma de
// distinguir "tierra muy seca" de "sonda al aire", y equivocarse hacia el lado
// de callarse es peor que publicar.
#define SUELO_RAW_DESCONECTADO  (SUELO_SECO_RAW + 8 > 1023 ? 1024 : SUELO_SECO_RAW + 8)

// Por DEBAJO de este crudo tampoco hay medida valida: ni la tierra empapada
// baja tanto (por eso se calcula desde el crudo en agua). Casi siempre
// significa que la sonda no esta recibiendo tension (VCC no quedo en D5, ver
// SUELO_VCC_PIN) o que hay un corto. Sin este chequeo se reportaria un 100 %
// de humedad falso, justo lo contrario de la realidad.
#define SUELO_RAW_MINIMO  (SUELO_AGUA_RAW / 4)

// Modo calibracion/diagnostico: imprime el crudo del ADC cada segundo (ademas
// del ciclo normal de 10 s), para ajustar los dos valores de arriba y para
// revisar cableado moviendo cables en vivo. Es el valor INICIAL: ya instalado
// se enciende y apaga a distancia con el comando "cal on" / "cal off".
#define CALIBRAR_SUELO  0

// Limpieza puntual tras renombrar el nodo: publica un payload VACIO retenido en
// los topics del id anterior para borrar sus mensajes retenidos del broker, y
// que no queden sensores fantasma en el minitool. Ya se corrio con id "sala",
// asi que queda en 0.
#define LIMPIAR_TOPICS_VIEJOS  0
#define ID_VIEJO               "sala"

// --- MQTT ------------------------------------------------------------------
// Broker propio en la Raspberry Pi (Mosquitto), IP fija por reserva DHCP.
// Antes: broker.hivemq.com (publico).
#define MQTT_BROKER   "192.168.1.100"
#define MQTT_PORT     1883

#define TOPIC_STATUS  "labo/nodo/"   NODE_ID "/status"
#define TOPIC_IP      "labo/nodo/"   NODE_ID "/ip"    /* para poder actualizarlo por OTA */
#define TOPIC_TEMP    "labo/sensor/" NODE_ID "/temp"
#define TOPIC_HUM     "labo/sensor/" NODE_ID "/hum"
#define TOPIC_SUELO   "labo/sensor/" NODE_ID "/suelo"
#define TOPIC_RSSI    "labo/sensor/" NODE_ID "/rssi"    /* salud: senal WiFi   */
#define TOPIC_UPTIME  "labo/sensor/" NODE_ID "/uptime"  /* salud: minutos on   */
#define TOPIC_HEAP    "labo/sensor/" NODE_ID "/heap"    /* salud: RAM libre kB */
#define TOPIC_SUELO_RAW "labo/sensor/" NODE_ID "/suelo_raw"       /* crudo del ADC, solo en calibracion */
#define TOPIC_UMBRAL  "labo/config/" NODE_ID "/suelo/umbral"      /* config del minitool (retenido) */
#define TOPIC_HISTER  "labo/config/" NODE_ID "/suelo/histeresis"  /* idem */
#define TOPIC_INTERV  "labo/config/" NODE_ID "/suelo/intervalo"   /* idem, en segundos */
#define TOPIC_ALERTA  "labo/alerta/" NODE_ID                      /* bus de alertas del home-lab */
#define TOPIC_CMD     "labo/nodo/"   NODE_ID "/cmd"               /* ordenes desde la tool Control */

// Cada cuanto leer y publicar el AIRE. El DHT22 admite 1 lectura / 2 s.
static const unsigned long PUBLICAR_MS = 10000;

// El SUELO tiene su propio ritmo, que se ajusta desde el minitool (TOPIC_INTERV)
// porque la humedad de la tierra cambia en horas, no en segundos: medir cada
// varios minutos alcanza, energiza menos la sonda y ahorra trafico. Este es el
// valor por defecto hasta que llegue la config retenida.
#define SUELO_INTERVALO_DEF_S   10
#define SUELO_INTERVALO_MIN_S    5
#define SUELO_INTERVALO_MAX_S 3600

// El modo calibracion publica el crudo del ADC (para poder recalibrar sin
// desmontar el nodo) y se apaga solo, para no dejarlo publicando de mas.
static const unsigned long CALIBRAR_MAX_MS = 30UL * 60UL * 1000UL;

// La telemetria de salud cambia lento: cada minuto sobra.
static const unsigned long TELEMETRIA_MS = 60000;

// Si pasa este rato sin WiFi, reiniciar: barato y saca al nodo de estados
// raros de la pila de red sin que tengas que ir a desenchufarlo.
static const unsigned long RESCATE_MS = 15UL * 60UL * 1000UL;

// Riego: valores por defecto hasta que llegue la config retenida del minitool.
#define SUELO_UMBRAL_DEF   20.0f   /* % por debajo del cual el suelo esta seco  */
#define SUELO_HISTER_DEF    5.0f   /* margen de recuperacion, para no oscilar   */

// Lecturas seguidas necesarias para dar por buena la condicion. A 10 s por
// ciclo son 30 s: filtra picos del ADC sin retrasar de mas el aviso.
#define SUELO_CONFIRMAR     3

// Cada cuanto repetir el aviso mientras el suelo siga seco (recordatorio).
static const unsigned long REAVISO_MS = 30UL * 60UL * 1000UL;

// --- Objetos globales ------------------------------------------------------
#if ENABLE_DHT
DHT dht(DHT_PIN, DHT_TYPE);
#endif
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

static unsigned long ultimaPublicacion = 0;   // aire
static unsigned long ultimaTelemetria  = 0;
static unsigned long ultimoSuelo       = 0;
static char clientId[32];   // se arma con el chip id para ser unico en el broker

static bool          s_calibrar   = (CALIBRAR_SUELO != 0);  // se cambia por MQTT
static unsigned long s_calibrarMs = 0;      // cuando se encendio la calibracion
static unsigned long s_sinWifiMs  = 0;      // millis del ultimo momento con WiFi
static unsigned long s_proxMqtt   = 0;      // cuando reintentar conectar al broker
static unsigned long s_esperaMqtt = 3000;   // backoff del reintento

#if ENABLE_SUELO
static float s_umbral_suelo = SUELO_UMBRAL_DEF;  // % de riego (lo pisa el minitool)
static float s_hister_suelo = SUELO_HISTER_DEF;  // margen de recuperacion
static unsigned long s_suelo_ms = SUELO_INTERVALO_DEF_S * 1000UL;  // ritmo de medicion
static bool  s_suelo_seco   = false;             // estado con histeresis
static int   s_seco_n       = 0;                 // lecturas seguidas confirmando
static int   s_humedo_n     = 0;
static unsigned long s_ultimo_aviso = 0;         // para el recordatorio
static bool  s_sonda_ko     = false;             // sonda al aire / desconectada
static int   s_sonda_ko_n   = 0;
static unsigned long s_sonda_ko_ms = 0;          // para repetir el aviso
#endif

// Parpadeo corto del LED integrado (GPIO2, activo en bajo) como senal de vida.
static void parpadeo() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(20);
  digitalWrite(LED_BUILTIN, HIGH);
}

// Publica una alerta JSON en el bus del home-lab (el minitool la muestra).
static void publicar_alerta(const char *nivel, const char *msg) {
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"origen\":\"Pieza\",\"nivel\":\"%s\",\"msg\":\"%s\"}", nivel, msg);
  mqtt.publish(TOPIC_ALERTA, payload);
}

#if ENABLE_SUELO
// Mediana de varias lecturas del ADC. Mejor que el promedio para este sensor:
// un pico aislado (ruido de la fuente, cable largo) no arrastra el resultado.
static int leer_suelo_raw() {
  const int N = 9;
  int v[N];

  // Con SUELO_VCC_PIN la sonda solo se energiza para esta medicion (menos
  // corrosion). Necesita un momento para estabilizarse antes de leer.
#if SUELO_VCC_PIN >= 0
  digitalWrite(SUELO_VCC_PIN, HIGH);
  delay(SUELO_ESTABILIZAR_MS);
#endif

  for (int i = 0; i < N; i++) { v[i] = analogRead(SUELO_PIN); delay(5); }

#if SUELO_VCC_PIN >= 0
  digitalWrite(SUELO_VCC_PIN, LOW);
#endif

  for (int i = 1; i < N; i++) {            // insercion: N es chico
    int x = v[i], j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[N / 2];
}

// Convierte el crudo a % de humedad (0 = seco, 100 = saturado), con recorte.
static float suelo_pct(int raw) {
  float pct = 100.0f * (float)(SUELO_SECO_RAW - raw) /
                       (float)(SUELO_SECO_RAW - SUELO_AGUA_RAW);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// Avisa si el crudo cae fuera de lo fisicamente posible: pegado al tope (sonda
// al aire o cable suelto) o al piso (sin alimentacion / corto). En ambos casos
// el % que saldria seria una falsa alarma, en un sentido o en el otro.
static bool revisar_sonda(int raw) {
  if (raw >= SUELO_RAW_DESCONECTADO || raw <= SUELO_RAW_MINIMO) {
    bool sin_tension = (raw <= SUELO_RAW_MINIMO);
    const char *que = sin_tension ? "Sonda de suelo sin alimentacion"
                                  : "Sonda de suelo desconectada o al aire";
    unsigned long ahora = millis();
    if (++s_sonda_ko_n >= SUELO_CONFIRMAR && !s_sonda_ko) {
      s_sonda_ko = true;
      s_sonda_ko_ms = ahora;
      publicar_alerta("alarma", que);
      Serial.printf("ALERTA: sonda de suelo %s (raw %d)\n",
                    sin_tension ? "sin alimentacion" : "desconectada", raw);
    } else if (s_sonda_ko && ahora - s_sonda_ko_ms >= REAVISO_MS) {
      // Mientras la sonda este en falla no se publica humedad, asi que el aviso
      // se repite: si no, un solo toast perdido y el riego queda ciego sin que
      // nada lo recuerde.
      s_sonda_ko_ms = ahora;
      publicar_alerta("alarma", que);
      Serial.printf("RECORDATORIO: sonda de suelo %s (raw %d)\n",
                    sin_tension ? "sin alimentacion" : "desconectada", raw);
    }
  } else {
    s_sonda_ko_n = 0;
    if (s_sonda_ko) {
      s_sonda_ko = false;
      publicar_alerta("ok", "Sonda de suelo conectada");
      Serial.println("Sonda de suelo de vuelta");
    }
  }
  return s_sonda_ko;
}

// Chequea el umbral y publica alerta al bus. Para no spamear ni oscilar:
//   - exige SUELO_CONFIRMAR lecturas seguidas antes de cambiar de estado,
//   - la vuelta a normal pide superar el umbral + histeresis,
//   - mientras siga seco, repite el aviso cada REAVISO_MS.
static void revisar_umbral_suelo(float pct) {
  if (s_umbral_suelo <= 0.0f) return;   // vigilancia desactivada desde la UI

  char msg[80];
  unsigned long ahora = millis();

  if (!s_suelo_seco) {
    if (pct < s_umbral_suelo) s_seco_n++;
    else                      s_seco_n = 0;

    if (s_seco_n >= SUELO_CONFIRMAR) {
      s_suelo_seco = true;
      s_seco_n = 0;
      s_ultimo_aviso = ahora;
      snprintf(msg, sizeof(msg), "Suelo seco %.0f%% (umbral %.0f%%)", pct, s_umbral_suelo);
      publicar_alerta("aviso", msg);
      Serial.printf("ALERTA: %s\n", msg);
    }
    return;
  }

  // Ya estaba seco: ver si se recupero, y si no, recordarlo cada tanto.
  if (pct > s_umbral_suelo + s_hister_suelo) s_humedo_n++;
  else                                       s_humedo_n = 0;

  if (s_humedo_n >= SUELO_CONFIRMAR) {
    s_suelo_seco = false;
    s_humedo_n = 0;
    snprintf(msg, sizeof(msg), "Suelo recuperado %.0f%%", pct);
    publicar_alerta("ok", msg);
    Serial.printf("%s\n", msg);
  } else if (ahora - s_ultimo_aviso >= REAVISO_MS) {
    s_ultimo_aviso = ahora;
    snprintf(msg, sizeof(msg), "Sigue seco %.0f%% (umbral %.0f%%)", pct, s_umbral_suelo);
    publicar_alerta("aviso", msg);
    Serial.printf("RECORDATORIO: %s\n", msg);
  }
}
#endif

// --- Callback MQTT: config del minitool + comandos remotos ------------------
//
// Comandos en labo/nodo/<NODE_ID>/cmd (los manda la tool Control):
//   "leer"     -> publica una lectura ahora, sin esperar el ciclo
//   "reset"    -> reinicia el nodo
//   "cal on"   -> empieza a imprimir el crudo del ADC por serie
//   "cal off"  -> deja de imprimirlo
static void on_mqtt(char *topic, byte *payload, unsigned int len) {
  char b[24];
  unsigned int n = len < sizeof(b) - 1 ? len : sizeof(b) - 1;
  memcpy(b, payload, n);
  b[n] = '\0';

  if (strcmp(topic, TOPIC_CMD) == 0) {
    Serial.printf("CMD: %s\n", b);
    if (strcmp(b, "leer") == 0) {
      ultimaPublicacion = 0;              // el loop publica en el proximo giro
      ultimoSuelo       = 0;
    } else if (strcmp(b, "reset") == 0 || strcmp(b, "reiniciar") == 0) {
      publicar_alerta("ok", "Reiniciando el nodo");
      mqtt.loop();                        // dale tiempo a salir antes del reset
      delay(200);
      ESP.restart();
    } else if (strcmp(b, "cal on") == 0) {
      s_calibrar   = true;
      s_calibrarMs = millis();
      Serial.println("Calibracion ON: publicando el crudo del ADC");
    } else if (strcmp(b, "cal off") == 0) {
      s_calibrar = false;
      Serial.println("Calibracion OFF");
    }
    return;
  }

#if ENABLE_SUELO
  float v = strtof(b, NULL);
  if (strcmp(topic, TOPIC_UMBRAL) == 0) {
    // 0 = el minitool apago el limite bajo: dejamos de vigilar aqui.
    if (v >= 0.0f && v <= 100.0f) {
      s_umbral_suelo = v;
      s_seco_n = s_humedo_n = 0;
      Serial.printf("Umbral de suelo actualizado: %.0f %%%s\n",
                    s_umbral_suelo, s_umbral_suelo <= 0.0f ? " (sin vigilancia)" : "");
    }
  } else if (strcmp(topic, TOPIC_HISTER) == 0) {
    if (v >= 0.0f && v <= 50.0f) {
      s_hister_suelo = v;
      Serial.printf("Histeresis de suelo actualizada: %.0f %%\n", s_hister_suelo);
    }
  } else if (strcmp(topic, TOPIC_INTERV) == 0) {
    if (v >= SUELO_INTERVALO_MIN_S && v <= SUELO_INTERVALO_MAX_S) {
      s_suelo_ms = (unsigned long)v * 1000UL;
      Serial.printf("Intervalo de suelo actualizado: %lu s\n", s_suelo_ms / 1000UL);
    }
  }
#endif
}

// --- Conectividad (no bloqueante) ------------------------------------------
//
// Nada de while(!conectado): el loop tiene que seguir girando para atender OTA
// aunque el broker este caido. Si se cae la red, el nodo publica igual apenas
// vuelve, y si no vuelve en RESCATE_MS se reinicia solo.

static void wifiIniciar() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);        // no desgastar la flash guardando credenciales
  WiFi.setAutoReconnect(true);
  WiFi.hostname(NODE_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi: conectando a \"%s\"...\n", WIFI_SSID);
}

static void wifiMantener() {
  if (WiFi.status() == WL_CONNECTED) {
    if (s_sinWifiMs) {           // acabamos de recuperarla
      Serial.printf("WiFi: conectado. IP %s  RSSI %d dBm\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      s_sinWifiMs = 0;
    }
    return;
  }

  unsigned long ahora = millis();
  if (!s_sinWifiMs) {
    s_sinWifiMs = ahora ? ahora : 1;   // 0 esta reservado para "hay WiFi"
    Serial.println("WiFi: caida, reintentando en segundo plano");
  } else if (ahora - s_sinWifiMs >= RESCATE_MS) {
    Serial.println("WiFi: sin red hace rato, reiniciando el nodo");
    delay(100);
    ESP.restart();
  }
}

// Se suscribe y saluda; se llama en cada (re)conexion al broker.
static void mqttAlConectar() {
  Serial.println("MQTT: conectado");
  mqtt.publish(TOPIC_STATUS, "online", true);
  // La IP queda retenida y visible en la tool Nodos: es la que se le pasa al
  // OTA cuando el nombre mDNS no resuelve (pasa seguido en Windows).
  mqtt.publish(TOPIC_IP, WiFi.localIP().toString().c_str(), true);
#if LIMPIAR_TOPICS_VIEJOS
  /* Payload vacio + retain = borra el mensaje retenido en el broker. */
  mqtt.publish("labo/sensor/" ID_VIEJO "/temp",   "", true);
  mqtt.publish("labo/sensor/" ID_VIEJO "/hum",    "", true);
  mqtt.publish("labo/sensor/" ID_VIEJO "/suelo",  "", true);
  mqtt.publish("labo/nodo/"   ID_VIEJO "/status", "", true);
  Serial.println("Limpieza: borrados los retenidos de '" ID_VIEJO "'");
#endif
  mqtt.subscribe(TOPIC_CMD);      // ordenes desde la tool Control
#if ENABLE_SUELO
  mqtt.subscribe(TOPIC_UMBRAL);   // config de riego del minitool (retenida)
  mqtt.subscribe(TOPIC_HISTER);
  mqtt.subscribe(TOPIC_INTERV);
#endif
}

static void mqttMantener() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) { s_esperaMqtt = 3000; return; }

  unsigned long ahora = millis();
  if (s_proxMqtt && ahora - s_proxMqtt < s_esperaMqtt) return;
  s_proxMqtt = ahora ? ahora : 1;

  Serial.printf("MQTT: conectando a %s:%d como %s ...\n",
                MQTT_BROKER, MQTT_PORT, clientId);
  // connect(id, willTopic, willQoS, willRetain, willMessage): el LWT deja
  // "offline" retenido si el nodo se desconecta sin avisar.
  if (mqtt.connect(clientId, TOPIC_STATUS, 1, true, "offline")) {
    mqttAlConectar();
    s_esperaMqtt = 3000;
  } else {
    // Backoff: si el broker esta caido, no lo martillamos cada 3 s.
    s_esperaMqtt = s_esperaMqtt < 60000 ? s_esperaMqtt * 2 : 60000;
    Serial.printf("MQTT: fallo (rc=%d), reintento en %lu s\n",
                  mqtt.state(), s_esperaMqtt / 1000);
  }
}

// --- OTA: actualizar por WiFi, sin desmontar el nodo ------------------------
static void otaIniciar() {
  ArduinoOTA.setHostname(NODE_ID);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: recibiendo firmware nuevo");
    if (mqtt.connected()) {
      publicar_alerta("aviso", "Actualizando firmware por OTA");
      mqtt.loop();
    }
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA: listo, reiniciando"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA: error %u\n", e); });
  ArduinoOTA.begin();
  Serial.println("OTA: activo (pio run -e " NODE_ID "_ota -t upload)");
}

// --- Telemetria de salud ----------------------------------------------------
// Se publica como sensores normales, asi el minitool ya los grafica, guarda su
// record del dia y les aplica umbrales (p.ej. avisar si el WiFi baja de -85 dBm).
static void publicarTelemetria() {
  char buf[16];

  snprintf(buf, sizeof(buf), "%d", WiFi.RSSI());
  mqtt.publish(TOPIC_RSSI, buf, true);

  snprintf(buf, sizeof(buf), "%lu", millis() / 60000UL);
  mqtt.publish(TOPIC_UPTIME, buf, true);

  snprintf(buf, sizeof(buf), "%.1f", ESP.getFreeHeap() / 1024.0f);
  mqtt.publish(TOPIC_HEAP, buf, true);

  Serial.printf("Salud -> RSSI %d dBm   uptime %lu min   heap %.1f kB\n",
                WiFi.RSSI(), millis() / 60000UL, ESP.getFreeHeap() / 1024.0f);
}

static void publicarAire() {
#if ENABLE_DHT
  char buf[16];
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
  parpadeo();
}

static void publicarSuelo() {
#if ENABLE_SUELO
  char buf[16];
  int raw = leer_suelo_raw();
  float pct = suelo_pct(raw);
  bool sonda_ko = revisar_sonda(raw);

  // Con la sonda en falla no publicamos el % (seria un valor enganoso que
  // dispararia una alerta de riego al reves de la realidad).
  if (!sonda_ko) {
    snprintf(buf, sizeof(buf), "%.0f", pct);
    mqtt.publish(TOPIC_SUELO, buf, true);
    revisar_umbral_suelo(pct);
  }

  // En calibracion el crudo tambien va por MQTT: asi se recalibra mirando el
  // minitool, sin cable USB ni monitor serie.
  if (s_calibrar) {
    snprintf(buf, sizeof(buf), "%d", raw);
    mqtt.publish(TOPIC_SUELO_RAW, buf, false);
  }

  Serial.printf("Suelo -> %.0f %% (raw %d)%s\n", pct, raw, sonda_ko ? "  [sonda KO]" : "");
#endif
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

#if ENABLE_SUELO && SUELO_VCC_PIN >= 0
  pinMode(SUELO_VCC_PIN, OUTPUT);
  digitalWrite(SUELO_VCC_PIN, LOW); // sonda apagada salvo al medir
  Serial.println("  sonda de suelo alimentada por GPIO (anti-corrosion)");
#endif

#if ENABLE_DHT
  dht.begin();
#endif

  // Client id unico en el broker publico (compartido con todo internet).
  snprintf(clientId, sizeof(clientId), "esp8266-%s-%06X", NODE_ID, ESP.getChipId());

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(on_mqtt);

  wifiIniciar();

  // Espera acotada solo para que el primer arranque se vea completo por serie;
  // si la red no esta, seguimos igual y el loop se encarga.
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  otaIniciar();
  wifiMantener();
  mqttMantener();
}

void loop() {
  wifiMantener();
  mqttMantener();
  ArduinoOTA.handle();   // atiende una actualizacion por WiFi si llega
  mqtt.loop();

  unsigned long ahora = millis();

#if ENABLE_SUELO
  // La calibracion se apaga sola: es un modo de diagnostico, no un estado
  // normal (mide y publica seguido, y energiza la sonda cada segundo).
  if (s_calibrar && ahora - s_calibrarMs >= CALIBRAR_MAX_MS) {
    s_calibrar = false;
    Serial.println("Calibracion OFF (por tiempo)");
  }
#endif

  if (mqtt.connected()) {
    if (ahora - ultimaPublicacion >= PUBLICAR_MS) {
      ultimaPublicacion = ahora;
      publicarAire();
    }
#if ENABLE_SUELO
    // En calibracion se mide cada segundo para poder ajustar mirando el valor
    // en vivo; el resto del tiempo, al ritmo configurado desde el minitool.
    unsigned long cada = s_calibrar ? 1000UL : s_suelo_ms;
    if (ahora - ultimoSuelo >= cada) {
      ultimoSuelo = ahora;
      publicarSuelo();
    }
#endif
    if (ahora - ultimaTelemetria >= TELEMETRIA_MS) {
      ultimaTelemetria = ahora;
      publicarTelemetria();
    }
  }
}
