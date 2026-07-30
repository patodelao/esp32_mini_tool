/*
 * mqtt_broker.c — Broker MQTT embebido (sobre Mongoose) hospedado en el refri.
 *
 * Ver mqtt_broker.h para el porqué. Mongoose aporta el event-loop TCP y el
 * parseo de paquetes MQTT; la semántica de broker (sesiones, suscripciones,
 * retenidos, fanout) la implementamos acá. Estado en tablas de tamaño fijo, sin
 * malloc: es un nodo que corre meses sin reiniciar, no queremos fragmentación.
 *
 * Concurrencia: TODO el estado del broker (subs/retenidos) y TODA llamada a
 * Mongoose ocurren en la task del broker. Las publicaciones locales del propio
 * refri llegan desde otras tasks por una cola FreeRTOS; la task del broker las
 * drena y las reparte. Así nunca se toca Mongoose desde dos tasks a la vez.
 */
#include "mqtt_broker.h"
#include "mongoose.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"   /* esp_restart */

#include <string.h>

static const char *TAG = "mqtt_broker";

#define LISTEN_URL   "mqtt://0.0.0.0:1883"

#define TOPIC_MAX    96
#define FILTER_MAX   96
#define PAYLOAD_MAX  192
#define MAX_SUBS     48   /* suscripciones vivas (varias por cliente)         */
#define MAX_RETAINED 80   /* topics retenidos (sensores + status/ip + config) */
#define MAX_LOCAL     4   /* suscriptores in-process (el cmd del refri, etc.) */

/* Reaping de conexiones mudas. Un cliente que muere sin FIN (corte de energía,
 * reboot por OTA) deja un socket TCP zombie que lwIP no detecta; con solo
 * ~10 sockets, unos pocos zombies agotan el pool y el broker queda "escuchando
 * pero mudo" (no acepta a nadie). Cerramos toda conexión sin tráfico entrante
 * por más de esto. Debe ser > el keepalive de los clientes (esp-mqtt 120 s,
 * PubSubClient 15 s): un cliente sano manda PINGREQ y nunca cae en el corte. */
#define CONN_IDLE_MS 180000

/* Una suscripción de un cliente de red: su conexión + el filtro (con comodines). */
typedef struct {
    struct mg_connection *c;
    char filter[FILTER_MAX];
    bool used;
} sub_t;

/* Un mensaje retenido: el broker guarda el ÚLTIMO valor de cada topic con retain
 * y se lo entrega a quien se suscriba después. Es lo que hace que el minitool
 * repueble su vista de Sensores/Nodos apenas conecta. */
typedef struct {
    char    topic[TOPIC_MAX];
    uint8_t payload[PAYLOAD_MAX];
    int     len;
    bool    used;
} retained_t;

/* Un suscriptor local (in-process): el refri escucha su propio topic de comandos
 * sin abrir un cliente TCP a sí mismo. */
typedef struct {
    char filter[FILTER_MAX];
    mqtt_broker_local_cb_t cb;
    bool used;
} local_sub_t;

/* Ítem de la cola de publicaciones locales (refri -> su propio broker). */
typedef struct {
    char    topic[TOPIC_MAX];
    uint8_t payload[PAYLOAD_MAX];
    int     len;
    bool    retain;
} txmsg_t;

static sub_t       s_subs[MAX_SUBS];
static retained_t  s_ret[MAX_RETAINED];
static local_sub_t s_local[MAX_LOCAL];

static struct mg_mgr s_mgr;
static QueueHandle_t  s_txq = NULL;
static volatile bool  s_running = false;
static volatile uint64_t s_alive_us = 0;   /* latido de la task del broker (watchdog) */

/* -------------------------- Coincidencia de topics ------------------------ */

/* Traduce un filtro MQTT a un patrón de mg_match y compara contra un topic
 * concreto. En MQTT '+' es un nivel (no cruza '/') y '#' es multinivel; en
 * mg_match eso es justo '*' (no cruza '/') y '#' (sí lo cruza). O sea: cambiar
 * '+' por '*' y dejar '#' como está. Ambos strings terminados en '\0'. */
static bool topic_match(const char *filter, const char *topic)
{
    char pat[FILTER_MAX];
    int n = 0;
    for (; filter[n] && n < FILTER_MAX - 1; n++) {
        pat[n] = (filter[n] == '+') ? '*' : filter[n];
    }
    pat[n] = '\0';
    return mg_match(mg_str(topic), mg_str_n(pat, (size_t)n), NULL);
}

/* --------------------------- Mensajes retenidos --------------------------- */

/* Guarda (o borra) el retenido de un topic. payload vacío = borrar, que es la
 * convención MQTT (payload vacío + retain) para olvidar un retenido; la usa, por
 * ejemplo, sensor_forget del minitool. */
static void retained_set(const char *topic, const uint8_t *payload, int len)
{
    if (len <= 0) {                        /* borrar el retenido de este topic */
        for (int i = 0; i < MAX_RETAINED; i++) {
            if (s_ret[i].used && strcmp(s_ret[i].topic, topic) == 0) {
                s_ret[i].used = false;
                return;
            }
        }
        return;
    }
    if (len > PAYLOAD_MAX) len = PAYLOAD_MAX;

    int slot = -1;
    for (int i = 0; i < MAX_RETAINED; i++) {
        if (s_ret[i].used && strcmp(s_ret[i].topic, topic) == 0) { slot = i; break; }
        if (slot < 0 && !s_ret[i].used) slot = i;
    }
    if (slot < 0) { ESP_LOGW(TAG, "Tabla de retenidos llena (%s)", topic); return; }

    strlcpy(s_ret[slot].topic, topic, sizeof(s_ret[slot].topic));
    memcpy(s_ret[slot].payload, payload, (size_t)len);
    s_ret[slot].len  = len;
    s_ret[slot].used = true;
}

/* Al suscribirse un cliente, entregarle los retenidos que hagan match. */
static void retained_flush_to(struct mg_connection *c, const char *filter)
{
    for (int i = 0; i < MAX_RETAINED; i++) {
        if (!s_ret[i].used) continue;
        if (!topic_match(filter, s_ret[i].topic)) continue;
        struct mg_mqtt_opts o;
        memset(&o, 0, sizeof(o));
        o.topic   = mg_str(s_ret[i].topic);
        o.message = mg_str_n((char *) s_ret[i].payload, (size_t) s_ret[i].len);
        o.qos     = 0;
        o.retain  = true;   /* marcar como retenido, según la spec */
        mg_mqtt_pub(c, &o);
    }
}

/* ------------------------------- Suscripciones ---------------------------- */

static void sub_add(struct mg_connection *c, const char *filter)
{
    for (int i = 0; i < MAX_SUBS; i++) {          /* ya existe: no duplicar */
        if (s_subs[i].used && s_subs[i].c == c &&
            strcmp(s_subs[i].filter, filter) == 0) return;
    }
    for (int i = 0; i < MAX_SUBS; i++) {
        if (!s_subs[i].used) {
            s_subs[i].c = c;
            strlcpy(s_subs[i].filter, filter, sizeof(s_subs[i].filter));
            s_subs[i].used = true;
            return;
        }
    }
    ESP_LOGW(TAG, "Tabla de suscripciones llena (%s)", filter);
}

static void subs_drop_conn(struct mg_connection *c)
{
    for (int i = 0; i < MAX_SUBS; i++)
        if (s_subs[i].used && s_subs[i].c == c) s_subs[i].used = false;
}

/* ------------------------------- Reparto ---------------------------------- */

/* Reparte un mensaje a todos los suscriptores (de red y locales) que coincidan,
 * y actualiza el retenido si corresponde. Corre siempre en la task del broker. */
static void deliver(const char *topic, const uint8_t *payload, int len, bool retain)
{
    if (retain) retained_set(topic, payload, len);

    for (int i = 0; i < MAX_SUBS; i++) {
        if (!s_subs[i].used) continue;
        if (!topic_match(s_subs[i].filter, topic)) continue;
        struct mg_mqtt_opts o;
        memset(&o, 0, sizeof(o));
        o.topic   = mg_str(topic);
        o.message = mg_str_n((char *) payload, (size_t) (len < 0 ? 0 : len));
        o.qos     = 0;       /* entrega degradada a QoS0: sin PUBACK pendiente */
        o.retain  = false;   /* es entrega en vivo, no un retenido */
        mg_mqtt_pub(s_subs[i].c, &o);
    }

    for (int i = 0; i < MAX_LOCAL; i++) {
        if (!s_local[i].used) continue;
        if (!topic_match(s_local[i].filter, topic)) continue;
        char pz[PAYLOAD_MAX];                 /* NUL-terminado para el callback */
        int n = len < 0 ? 0 : (len < PAYLOAD_MAX ? len : PAYLOAD_MAX - 1);
        memcpy(pz, payload, (size_t) n);
        pz[n] = '\0';
        s_local[i].cb(topic, pz, n);
    }
}

/* -------------------------- Handler de Mongoose --------------------------- */

/* Envía un paquete de control sin payload variable (CONNACK, SUBACK, PUBACK). */
static void send_ack(struct mg_connection *c, uint8_t cmd, const uint8_t *body, uint32_t n)
{
    mg_mqtt_send_header(c, cmd, 0, n);
    if (n) mg_send(c, body, n);
}

static void broker_fn(struct mg_connection *c, int ev, void *ev_data)
{
    /* Sello de última actividad para el reaping: nace al aceptar y se refresca
     * con cualquier dato entrante (PUBLISH, SUBSCRIBE, y sobre todo PINGREQ). */
    if (ev == MG_EV_ACCEPT || ev == MG_EV_READ) {
        *(uint64_t *) c->data = mg_millis();
    }

    if (ev == MG_EV_MQTT_CMD) {
        struct mg_mqtt_message *mm = (struct mg_mqtt_message *) ev_data;
        switch (mm->cmd) {
            case MQTT_CMD_CONNECT: {
                /* Aceptar anónimo (como el allow_anonymous de Mosquitto). CONNACK
                 * con return code 0 = conexión aceptada. */
                uint8_t connack[2] = { 0, 0 };
                send_ack(c, MQTT_CMD_CONNACK, connack, sizeof(connack));
                break;
            }
            case MQTT_CMD_PUBLISH: {
                bool retain = (mm->dgram.len > 0) && (mm->dgram.buf[0] & 0x01);
                char topic[TOPIC_MAX];
                int tn = (int) (mm->topic.len < TOPIC_MAX - 1 ? mm->topic.len : TOPIC_MAX - 1);
                memcpy(topic, mm->topic.buf, (size_t) tn);
                topic[tn] = '\0';
                deliver(topic, (const uint8_t *) mm->data.buf, (int) mm->data.len, retain);
                /* QoS1 entrante: hay que confirmar con PUBACK o el cliente
                 * (p.ej. esp-mqtt) reintenta y se cuelga. */
                if (mm->qos == 1) {
                    uint8_t id[2] = { (uint8_t)(mm->id >> 8), (uint8_t)(mm->id & 0xff) };
                    send_ack(c, MQTT_CMD_PUBACK, id, sizeof(id));
                }
                break;
            }
            case MQTT_CMD_SUBSCRIBE: {
                /* Payload del SUBSCRIBE: tras el header y el packet-id (2 B), una
                 * lista de (len topic 2 B, topic, qos 1 B). Mongoose ya nos dio
                 * mm->id; ubicamos el payload saltando el header de longitud. */
                const uint8_t *buf = (const uint8_t *) mm->dgram.buf;
                size_t dlen = mm->dgram.len;
                size_t p = 1;                          /* saltar byte de comando */
                for (int k = 0; k < 4 && p < dlen; k++) /* saltar remaining-length */
                    if (!(buf[p++] & 0x80)) break;
                p += 2;                                 /* saltar packet-id */

                uint8_t granted[64];
                int ntopics = 0;
                while (p + 2 <= dlen && ntopics < (int) sizeof(granted)) {
                    uint16_t tlen = (uint16_t)((buf[p] << 8) | buf[p + 1]);
                    p += 2;
                    if (p + tlen >= dlen) break;        /* +1 para el byte de qos */
                    char filter[FILTER_MAX];
                    int fn = tlen < FILTER_MAX - 1 ? tlen : FILTER_MAX - 1;
                    memcpy(filter, buf + p, (size_t) fn);
                    filter[fn] = '\0';
                    p += tlen;
                    uint8_t qos = buf[p++];
                    sub_add(c, filter);
                    retained_flush_to(c, filter);       /* entregar retenidos ya */
                    granted[ntopics++] = qos <= 1 ? qos : 1;  /* concedemos ≤ QoS1 */
                }
                /* SUBACK: packet-id (2 B) + un código de retorno por topic. */
                mg_mqtt_send_header(c, MQTT_CMD_SUBACK, 0, (uint32_t)(2 + ntopics));
                uint8_t id[2] = { (uint8_t)(mm->id >> 8), (uint8_t)(mm->id & 0xff) };
                mg_send(c, id, 2);
                mg_send(c, granted, (size_t) ntopics);
                break;
            }
            case MQTT_CMD_PINGREQ:
                mg_mqtt_pong(c);   /* keepalive: responder PINGRESP */
                break;
            case MQTT_CMD_DISCONNECT:
                c->is_draining = 1;   /* cierre limpio: vaciar y cerrar */
                break;
            default:
                break;
        }
    } else if (ev == MG_EV_CLOSE) {
        subs_drop_conn(c);   /* el cliente se fue: soltar sus suscripciones */
    }
}

/* ------------------------------- Task / API ------------------------------- */

static void drain_txq(void)
{
    txmsg_t m;
    while (s_txq && xQueueReceive(s_txq, &m, 0) == pdTRUE)
        deliver(m.topic, m.payload, m.len, m.retain);
}

/* Cierra las conexiones mudas hace demasiado (sockets zombie). Marcar
 * is_closing basta: Mongoose las cierra en el próximo poll, dispara MG_EV_CLOSE
 * (que suelta sus suscripciones) y libera el socket. Solo marcamos, no tocamos
 * la lista, así que iterar es seguro. */
static void reap_idle(void)
{
    uint64_t now = mg_millis();
    for (struct mg_connection *c = s_mgr.conns; c != NULL; c = c->next) {
        if (c->is_listening) continue;
        uint64_t last = *(uint64_t *) c->data;
        if (last != 0 && now - last > CONN_IDLE_MS) {
            ESP_LOGW(TAG, "Conexión muda hace %llus: cerrando (socket zombie)",
                     (unsigned long long)((now - last) / 1000));
            c->is_closing = 1;
        }
    }
}

/* Watchdog del broker. Corre en la task de esp_timer (independiente de la del
 * broker), así que sigue vivo aunque el broker se cuelgue. Si el latido no
 * avanza por un buen rato, mg_mgr_poll dejó de correr (se vio: puerto abierto
 * pero sin atender MQTT) y la única salida es reiniciar el nodo — todos los
 * nodos republican, así que el bus se recupera solo en segundos. */
#define BROKER_STALL_S 20
static void broker_wd_cb(void *arg)
{
    (void) arg;
    if (!s_running || s_alive_us == 0) return;
    uint64_t age = (esp_timer_get_time() - s_alive_us) / 1000000ULL;
    if (age >= BROKER_STALL_S) {
        ESP_LOGE(TAG, "Broker sin avanzar hace %llus: reiniciando el nodo", age);
        esp_restart();
    }
}

static void broker_task(void *arg)
{
    (void) arg;
    mg_mgr_init(&s_mgr);
    if (mg_mqtt_listen(&s_mgr, LISTEN_URL, broker_fn, NULL) == NULL) {
        ESP_LOGE(TAG, "No se pudo escuchar en %s", LISTEN_URL);
        mg_mgr_free(&s_mgr);
        vTaskDelete(NULL);
        return;
    }
    s_running = true;
    s_alive_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Broker MQTT escuchando en %s", LISTEN_URL);

    /* Arranca el watchdog (chequea cada 5 s). */
    const esp_timer_create_args_t wd = { .callback = broker_wd_cb, .name = "broker_wd" };
    esp_timer_handle_t wdh;
    if (esp_timer_create(&wd, &wdh) == ESP_OK)
        esp_timer_start_periodic(wdh, 5ULL * 1000000ULL);

    uint64_t last_reap = 0;
    for (;;) {
        mg_mgr_poll(&s_mgr, 50);   /* atiende la red; ≤50 ms de latencia local */
        drain_txq();               /* aplica las publicaciones del propio refri */

        uint64_t now = mg_millis();
        if (now - last_reap >= 1000) {   /* barrer zombies ~1 Hz */
            last_reap = now;
            reap_idle();
        }

        s_alive_us = esp_timer_get_time();   /* latido: el poll sigue avanzando */
    }
}

void mqtt_broker_on_local(const char *filter, mqtt_broker_local_cb_t cb)
{
    if (!filter || !cb) return;
    for (int i = 0; i < MAX_LOCAL; i++) {
        if (!s_local[i].used) {
            strlcpy(s_local[i].filter, filter, sizeof(s_local[i].filter));
            s_local[i].cb = cb;
            s_local[i].used = true;
            return;
        }
    }
    ESP_LOGW(TAG, "Sin espacio para más suscriptores locales (%s)", filter);
}

void mqtt_broker_start(void)
{
    if (s_txq) return;   /* idempotente */
    s_txq = xQueueCreate(24, sizeof(txmsg_t));
    if (!s_txq) { ESP_LOGE(TAG, "Sin memoria para la cola de publicación"); return; }
    /* Prioridad 4: por debajo de alarm_task (5), para no robarle tiempo real al
     * lazo de la puerta. Stack holgado (16 KB): Mongoose + el fanout + logging
     * usan bastante, y un overflow silencioso era el sospechoso #1 del cuelgue. */
    xTaskCreate(broker_task, "mqtt_broker", 16384, NULL, 4, NULL);
}

bool mqtt_broker_running(void) { return s_running; }

void mqtt_broker_local_publish(const char *topic, const char *payload, bool retain)
{
    if (!s_txq || !topic) return;
    txmsg_t m;
    memset(&m, 0, sizeof(m));
    strlcpy(m.topic, topic, sizeof(m.topic));
    int n = 0;
    if (payload) {
        n = (int) strlen(payload);
        if (n > PAYLOAD_MAX) n = PAYLOAD_MAX;
        memcpy(m.payload, payload, (size_t) n);
    }
    m.len    = n;
    m.retain = retain;
    if (xQueueSend(s_txq, &m, 0) != pdTRUE)
        ESP_LOGW(TAG, "Cola de publicación llena, se descarta %s", topic);
}
