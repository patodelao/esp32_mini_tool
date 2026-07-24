/*
 * datalog.c — Implementación del registro histórico en SPIFFS.
 *
 * La escritura va en su propia tarea, no en un esp_timer: abrir archivos y
 * escribir en flash necesita más pila de la que tiene la tarea de los timers, y
 * un SPIFFS lento no debe atrasar al resto de los timers del sistema.
 *
 * La foto se toma alineada a la media hora del reloj (no cada 30 min desde el
 * arranque), así las filas caen siempre en :00 y :30 y las series de días
 * distintos se comparan sin interpolar.
 */
#include "datalog.h"
#include "sensor_service.h"
#include "sensor_alert.h"

#include "esp_spiffs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static const char *TAG = "datalog";

#define MOUNT_POINT  "/data"
#define FILE_CUR     MOUNT_POINT "/log.csv"
#define FILE_PREV    MOUNT_POINT "/log1.csv"

/* Cada cuánto se guarda una foto de todos los sensores. */
#define PERIOD_S     (30 * 60)

/* Límite por archivo. Con ~13 sensores cada foto ocupa unos 430 bytes, o sea
 * ~20 kB por día: 400 kB son unas tres semanas por archivo, y como se guardan
 * dos, el registro cubre entre 3 y 6 semanas. Los dos juntos nunca pasan de
 * 800 kB de la partición de 1 MB. */
#define FILE_LIMIT   (400 * 1024)

#define CSV_HEADER   "fecha,sensor,valor\n"

/* El reloj arranca en 1970 hasta que SNTP o la NVS lo ponen en hora. Sin esto
 * el registro se llenaría de filas fechadas en 1970. */
#define TIME_VALID(t) ((t) > 1600000000)

static bool s_ready = false;
static long s_last_slot = -1;   /* media hora ya registrada */

/* --------------------------------- Escritura ------------------------------ */

static size_t file_size(const char *path)
{
    struct stat st;
    return (stat(path, &st) == 0) ? (size_t)st.st_size : 0;
}

/* Cuando el archivo actual se pasa del límite, el anterior se descarta y el
 * actual toma su lugar. Es todo lo que hace falta: no se copia ni se recorta
 * nada, que en una partición de 1 MB no entraría. */
static void rotate_if_needed(void)
{
    if (file_size(FILE_CUR) < FILE_LIMIT) return;

    unlink(FILE_PREV);
    if (rename(FILE_CUR, FILE_PREV) == 0) {
        ESP_LOGI(TAG, "Registro rotado; se descartó el tramo más viejo");
    } else {
        /* Si el rename falla no queda otra que empezar de nuevo: seguir
         * escribiendo llenaría la partición. */
        ESP_LOGW(TAG, "No se pudo rotar, se reinicia el registro");
        unlink(FILE_CUR);
    }
}

static void write_snapshot(time_t now)
{
    struct tm tm;
    localtime_r(&now, &tm);
    char fecha[20];
    strftime(fecha, sizeof(fecha), "%Y-%m-%d %H:%M", &tm);

    FILE *f = fopen(FILE_CUR, "a");
    if (!f) {
        ESP_LOGW(TAG, "No se pudo abrir %s", FILE_CUR);
        return;
    }

    int escritas = 0;
    int n = sensor_count();
    for (int i = 0; i < n; i++) {
        char id[SENSOR_ID_MAX], val[16];
        uint32_t age = 0;
        if (!sensor_get(i, id, sizeof(id), val, sizeof(val), &age)) continue;

        /* Un sensor mudo repetiría su último valor para siempre y ensuciaría la
         * serie con una línea recta que nunca existió. Mejor un hueco. */
        if (age > sensor_alert_stale_limit(id)) continue;

        fprintf(f, "%s,%s,%s\n", fecha, id, val);
        escritas++;
    }
    fclose(f);

    ESP_LOGI(TAG, "%s: %d sensores (%u kB en total)",
             fecha, escritas, (unsigned)(datalog_size() / 1024));

    rotate_if_needed();
}

/* El montaje va acá y no en datalog_init() porque la primera vez SPIFFS
 * formatea la partición, y eso son varios segundos: hacerlo en app_main
 * retrasaría el panel web y el clima. Así el arranque no espera a nadie. */
static bool mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = MOUNT_POINT,
        .partition_label = "storage",
        .max_files = 4,
        .format_if_mount_failed = true,
    };

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS no montó (%s): sin registro histórico", esp_err_to_name(err));
        return false;
    }

    size_t total = 0, usado = 0;
    if (esp_spiffs_info("storage", &total, &usado) == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS montado: %u/%u kB usados",
                 (unsigned)(usado / 1024), (unsigned)(total / 1024));
    }
    return true;
}

static void datalog_task(void *arg)
{
    (void)arg;

    if (!mount()) vTaskDelete(NULL);
    s_ready = true;
    ESP_LOGI(TAG, "Registro cada %d min, %u kB guardados", PERIOD_S / 60,
             (unsigned)(datalog_size() / 1024));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20000));

        time_t now;
        time(&now);
        if (!TIME_VALID(now)) continue;

        long slot = (long)(now / PERIOD_S);
        if (slot == s_last_slot) continue;
        s_last_slot = slot;

        write_snapshot(now);
    }
}

/* ------------------------------- API pública ------------------------------ */

void datalog_init(void)
{
    static bool started = false;
    if (started) return;
    started = true;
    xTaskCreate(datalog_task, "datalog", 4096, NULL, 3, NULL);
}

bool datalog_ready(void) { return s_ready; }

size_t datalog_size(void)
{
    if (!s_ready) return 0;
    return file_size(FILE_PREV) + file_size(FILE_CUR);
}

bool datalog_dump(datalog_sink_t sink, void *ctx)
{
    if (!s_ready) return false;
    if (datalog_size() == 0) return false;

    if (!sink(ctx, CSV_HEADER, sizeof(CSV_HEADER) - 1)) return true;

    /* Primero el tramo viejo y después el actual: el archivo sale ordenado. */
    const char *paths[2] = { FILE_PREV, FILE_CUR };
    static char buf[512];

    for (int p = 0; p < 2; p++) {
        FILE *f = fopen(paths[p], "r");
        if (!f) continue;
        size_t got;
        while ((got = fread(buf, 1, sizeof(buf), f)) > 0) {
            if (!sink(ctx, buf, (int)got)) { fclose(f); return true; }
        }
        fclose(f);
    }
    return true;
}

void datalog_clear(void)
{
    if (!s_ready) return;
    unlink(FILE_PREV);
    unlink(FILE_CUR);
    s_last_slot = -1;
    ESP_LOGI(TAG, "Registro borrado");
}
