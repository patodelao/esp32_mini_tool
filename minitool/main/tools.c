/*
 * tools.c — Registro central de herramientas del menú.
 * Para añadir una herramienta nueva: implementa su `const tool_t` en un .c
 * dentro de tools/, decláralo aquí como extern y añádelo al array g_tools[].
 */
#include "tool.h"

extern const tool_t tool_casa;      /* Resumen del hogar */
extern const tool_t tool_clock;     /* Hora + Crono + Timer */
extern const tool_t tool_level;
extern const tool_t tool_gyro;
extern const tool_t tool_dice;
extern const tool_t tool_flashlight;
extern const tool_t tool_wifi;
extern const tool_t tool_wifiscan;
extern const tool_t tool_netinfo;
extern const tool_t tool_bt;
extern const tool_t tool_gym;
extern const tool_t tool_tracker;
extern const tool_t tool_dashboard;
extern const tool_t tool_weather;
extern const tool_t tool_pedometer;
extern const tool_t tool_fleet;
extern const tool_t tool_sensors;
extern const tool_t tool_control;
extern const tool_t tool_alerts;
extern const tool_t tool_wifiqr;
extern const tool_t tool_wfdata;
extern const tool_t tool_cam;
extern const tool_t tool_settings;
extern const tool_t tool_phone;
extern const tool_t tool_alarm;

const tool_t *const g_tools[] = {
    &tool_casa,
    &tool_clock,
    &tool_alarm,
    &tool_level,
    &tool_pedometer,
    &tool_gyro,
    &tool_dice,
    &tool_flashlight,
    &tool_weather,
    &tool_gym,
    //&tool_tracker,
    &tool_dashboard,
    &tool_fleet,
    &tool_sensors,
    &tool_alerts,
    &tool_phone,
    &tool_cam,
    &tool_control,
    &tool_settings,

    /* Las de abajo están marcadas 'hidden': no salen en el menú principal,
     * se abren desde la tool Config, que las agrupa. */
    &tool_wifi,
    &tool_wifiscan,
    &tool_netinfo,
    &tool_wifiqr,
    &tool_wfdata,
    &tool_bt,
};

const int g_tools_count = (int)(sizeof(g_tools) / sizeof(g_tools[0]));
