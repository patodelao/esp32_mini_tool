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

/* Auth del broker (Fase 2). Si secrets.h define MQTT_USER y MQTT_PASS, el broker
 * EXIGE esas credenciales en el CONNECT y rechaza al resto (incluido anónimo).
 * Si no las define, acepta anónimo como hasta ahora: queda DORMIDO hasta que las
 * pongas en el secrets.h del refri y reflashees. Deben coincidir con las que
 * mandan los clientes (mismos MQTT_USER/MQTT_PASS en sus secrets). */
#if defined(__has_include)
#  if __has_include("secrets.h")
#    include "secrets.h"
#  endif
#endif
#if defined(MQTT_USER) && defined(MQTT_PASS)
#  define BROKER_REQUIRE_AUTH 1
#else
#  define BROKER_REQUIRE_AUTH 0
#endif

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

/* Backpressure: tope del buffer de envío por cliente. Si un suscriptor deja de
 * leer (medio-muerto, lento), su cola de salida crece sin límite con cada
 * PUBLISH y presiona el heap. Arriba de esto lo cerramos. Se chequea en el
 * barrido de ~1 Hz, no en el fanout: una ráfaga legítima (flush de retenidos al
 * suscribirse) drena en un poll, así que 1 s después solo queda el atascado.
 * Holgado (16 KB) para no cortar esas ráfagas: los payloads son ≤192 B. */
#define SEND_CAP     16384

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

/* Client-id por conexión, para el takeover: si llega un CONNECT con un id que
 * ya tiene una conexión viva (típico: un nodo que reinició por OTA y reconecta
 * antes de que su socket viejo se note muerto), cerramos la vieja en el acto.
 * Así no hay que esperar al reaping, que queda para los que desaparecen sin
 * volver. MQTT lo exige, además. */
#define CID_MAX      40
typedef struct {
    struct mg_connection *c;
    char cid[CID_MAX];
    bool used;
} cidmap_t;

static sub_t       s_subs[MAX_SUBS];
static retained_t  s_ret[MAX_RETAINED];
static local_sub_t s_local[MAX_LOCAL];
static cidmap_t    s_cids[MAX_SUBS];   /* una entrada por cliente conectado */

static struct mg_mgr s_mgr;
static QueueHandle_t  s_txq = NULL;
static volatile bool  s_running = false;
static volatile uint64_t s_alive_us = 0;   /* latido de la task del broker (watchdog) */
static volatile int      s_clients = 0;    /* conexiones TCP vivas (telemetría)    */
static volatile uint32_t s_reaped  = 0;    /* zombies cerrados por el reaper (tel.) */

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

/* ------------------------------- Client-ids ------------------------------- */

static void cid_forget(struct mg_connection *c)
{
    for (int i = 0; i < MAX_SUBS; i++)
        if (s_cids[i].used && s_cids[i].c == c) s_cids[i].used = false;
}

/* Registra el client-id de esta conexión y cierra cualquier OTRA con el mismo
 * id (session takeover). Un id vacío (clean session anónima) no dispara
 * takeover: no hay sesión que reemplazar. */
static void cid_register(struct mg_connection *c, const char *cid)
{
    if (cid && cid[0]) {
        for (int i = 0; i < MAX_SUBS; i++) {
            if (s_cids[i].used && s_cids[i].c != c &&
                strcmp(s_cids[i].cid, cid) == 0) {
                ESP_LOGW(TAG, "Takeover de '%s': cerrando la conexión anterior", cid);
                s_cids[i].c->is_closing = 1;   /* Mongoose la cierra en el próximo poll */
                s_cids[i].used = false;
            }
        }
    }
    for (int i = 0; i < MAX_SUBS; i++) {       /* ya tiene entrada: actualizar */
        if (s_cids[i].used && s_cids[i].c == c) {
            strlcpy(s_cids[i].cid, cid ? cid : "", sizeof(s_cids[i].cid));
            return;
        }
    }
    for (int i = 0; i < MAX_SUBS; i++) {       /* nueva entrada */
        if (!s_cids[i].used) {
            s_cids[i].c = c;
            strlcpy(s_cids[i].cid, cid ? cid : "", sizeof(s_cids[i].cid));
            s_cids[i].used = true;
            return;
        }
    }
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

/* Lee un string MQTT (2 B big-endian de largo + bytes) desde buf[*p]; lo copia a
 * out (NUL-terminado, truncado a outsz) si out != NULL, y avanza *p. false si el
 * paquete no da para tanto. */
static bool mqtt_rd_str(const uint8_t *buf, size_t dlen, size_t *p, char *out, int outsz)
{
    if (*p + 2 > dlen) return false;
    uint16_t l = (uint16_t)((buf[*p] << 8) | buf[*p + 1]);
    *p += 2;
    if (*p + l > dlen) return false;
    if (out) {
        int n = l < outsz - 1 ? l : outsz - 1;
        memcpy(out, buf + *p, (size_t) n);
        out[n] = '\0';
    }
    *p += l;
    return true;
}

/* Parsea un paquete CONNECT: client-id y (si vienen, según los connect flags)
 * usuario y contraseña. Devuelve false si está malformado. */
static bool parse_connect(const uint8_t *buf, size_t dlen,
                          char *cid, int cidsz,
                          char *user, int usersz, bool *has_user,
                          char *pass, int passsz, bool *has_pass)
{
    if (cid)  cid[0]  = '\0';
    if (user) user[0] = '\0';
    if (pass) pass[0] = '\0';
    *has_user = *has_pass = false;

    size_t p = 1;                                /* saltar byte de comando */
    for (int k = 0; k < 4 && p < dlen; k++)      /* saltar remaining-length */
        if (!(buf[p++] & 0x80)) break;

    if (!mqtt_rd_str(buf, dlen, &p, NULL, 0)) return false;  /* nombre de protocolo */
    if (p + 4 > dlen) return false;              /* nivel(1)+flags(1)+keepalive(2) */
    p += 1;                                      /* nivel de protocolo */
    uint8_t flags = buf[p++];                    /* connect flags */
    p += 2;                                      /* keepalive */

    bool has_will = (flags & 0x04) != 0;
    *has_user     = (flags & 0x80) != 0;
    *has_pass     = (flags & 0x40) != 0;

    if (!mqtt_rd_str(buf, dlen, &p, cid, cidsz)) return false;   /* client-id */
    if (has_will) {                                              /* will topic + msg */
        if (!mqtt_rd_str(buf, dlen, &p, NULL, 0)) return false;
        if (!mqtt_rd_str(buf, dlen, &p, NULL, 0)) return false;
    }
    if (*has_user && !mqtt_rd_str(buf, dlen, &p, user, usersz)) return false;
    if (*has_pass && !mqtt_rd_str(buf, dlen, &p, pass, passsz)) return false;
    return true;
}

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
    if (ev == MG_EV_ACCEPT) {
        *(uint64_t *) c->data = mg_millis();
        s_clients++;
    } else if (ev == MG_EV_READ) {
        *(uint64_t *) c->data = mg_millis();
    }

    if (ev == MG_EV_MQTT_CMD) {
        struct mg_mqtt_message *mm = (struct mg_mqtt_message *) ev_data;
        switch (mm->cmd) {
            case MQTT_CMD_CONNECT: {
                char cid[CID_MAX] = {0}, user[48] = {0}, pass[48] = {0};
                bool has_user = false, has_pass = false;
                parse_connect((const uint8_t *) mm->dgram.buf, mm->dgram.len,
                              cid, sizeof(cid), user, sizeof(user), &has_user,
                              pass, sizeof(pass), &has_pass);
#if BROKER_REQUIRE_AUTH
                bool ok = has_user && has_pass &&
                          strcmp(user, MQTT_USER) == 0 && strcmp(pass, MQTT_PASS) == 0;
                if (!ok) {
                    /* CONNACK return code 0x05 = no autorizado; luego cerrar. */
                    uint8_t connack[2] = { 0, 5 };
                    send_ack(c, MQTT_CMD_CONNACK, connack, sizeof(connack));
                    c->is_draining = 1;   /* mandar el CONNACK y cerrar */
                    ESP_LOGW(TAG, "CONNECT rechazado (auth) client-id='%s'", cid);
                    break;
                }
#else
                (void) user; (void) pass; (void) has_user; (void) has_pass;
#endif
                cid_register(c, cid);   /* takeover por client-id */

                /* CONNACK return code 0 = conexión aceptada. */
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
        cid_forget(c);
        if (!c->is_listening && s_clients > 0) s_clients--;
    }
}

/* ------------------------------- Task / API ------------------------------- */

static void drain_txq(void)
{
    txmsg_t m;
    while (s_txq && xQueueReceive(s_txq, &m, 0) == pdTRUE)
        deliver(m.topic, m.payload, m.len, m.retain);
}

/* Cierra las conexiones enfermas: mudas hace demasiado (socket zombie) o con el
 * buffer de envío desbordado (cliente que no drena). Marcar is_closing basta:
 * Mongoose las cierra en el próximo poll, dispara MG_EV_CLOSE (que suelta sus
 * suscripciones) y libera el socket. Solo marcamos, no tocamos la lista, así que
 * iterar es seguro. */
static void reap_unhealthy(void)
{
    uint64_t now = mg_millis();
    for (struct mg_connection *c = s_mgr.conns; c != NULL; c = c->next) {
        if (c->is_listening || c->is_closing) continue;

        uint64_t last = *(uint64_t *) c->data;
        if (last != 0 && now - last > CONN_IDLE_MS) {
            ESP_LOGW(TAG, "Conexión muda hace %llus: cerrando (socket zombie)",
                     (unsigned long long)((now - last) / 1000));
            c->is_closing = 1;
            s_reaped++;
            continue;
        }
        if (c->send.len > SEND_CAP) {
            ESP_LOGW(TAG, "Cliente lento (%u B sin drenar): cerrando (backpressure)",
                     (unsigned) c->send.len);
            c->is_closing = 1;
            s_reaped++;
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
        if (now - last_reap >= 1000) {   /* barrer conexiones enfermas ~1 Hz */
            last_reap = now;
            reap_unhealthy();
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

int      mqtt_broker_client_count(void) { return s_clients; }
uint32_t mqtt_broker_reaped(void)       { return s_reaped; }

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
