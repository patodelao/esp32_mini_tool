/*
 * sensor_alert.h — Motor de umbrales y alertas por sensor.
 *
 * Cada sensor visto por sensor_service tiene una regla con banda de normalidad
 * (mínimo y máximo, cada uno activable por separado), histéresis para la vuelta
 * a normal y vigilancia de "sin datos". El motor:
 *
 *   - evalúa cada lectura que llega por MQTT,
 *   - exige varias muestras seguidas fuera de rango antes de avisar (anti-ruido
 *     del ADC del suelo, que da picos aislados),
 *   - notifica el cambio de estado con ui_notify (aviso al salirse, ok al
 *     volver) y repite el aviso cada 30 min mientras siga fuera,
 *   - persiste las reglas en NVS,
 *   - deja el estado disponible para que la tool Sensores coloree el valor.
 *
 * Reparto con los nodos: si la regla tiene 'remote_lo' (caso del suelo), el
 * límite bajo lo vigila el propio nodo — el minitool le publica el umbral por
 * MQTT y NO emite su propio toast, para no duplicar el que manda el nodo. Así
 * el riego sigue avisando aunque el minitool esté apagado.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSOR_ALERT_OK = 0,
    SENSOR_ALERT_LOW,     /* por debajo del mínimo */
    SENSOR_ALERT_HIGH,    /* por encima del máximo */
    SENSOR_ALERT_STALE,   /* dejó de publicar */
} sensor_alert_state_t;

typedef struct {
    bool     lo_on;      /* vigilar el mínimo */
    float    lo;
    bool     hi_on;      /* vigilar el máximo */
    float    hi;
    float    hyst;       /* margen de recuperación (evita el parpadeo del aviso) */
    bool     stale_on;   /* avisar si el sensor deja de publicar */
    bool     remote_lo;  /* el mínimo lo vigila el nodo (config MQTT retenida) */
    uint16_t sample_s;   /* cada cuánto mide el nodo, s (0 = lo decide el nodo).
                          * No es un umbral: es config que se le publica al nodo,
                          * pero se guarda junto a la regla porque se edita en la
                          * misma pantalla y define cuándo un sensor está "mudo". */
} sensor_rule_t;

void sensor_alert_init(void);

/* Alimenta el motor con una lectura nueva. La llama sensor_service. */
void sensor_alert_on_value(const char *id, float v);

/* Regla vigente del sensor (la guardada, o la por defecto de su magnitud). */
void sensor_alert_get_rule(const char *id, sensor_rule_t *out);

/* Fija y persiste la regla; reevalúa el estado con el último valor. */
void sensor_alert_set_rule(const char *id, const sensor_rule_t *r);

/* Estado actual del sensor (para colorear la UI). */
sensor_alert_state_t sensor_alert_state(const char *id);

/* Paso y rango recomendados para editar los umbrales de esa magnitud. */
void sensor_alert_edit_hints(const char *id, float *step, float *min, float *max);

/* Segundos sin publicar tras los cuales el sensor se considera mudo. Sale del
 * intervalo de muestreo configurado (3 ciclos), así un sensor que mide cada
 * 10 min no se marca como caído a los 2 minutos. */
uint32_t sensor_alert_stale_limit(const char *id);

#ifdef __cplusplus
}
#endif
