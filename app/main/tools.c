/*
 * tools.c — Registro central de herramientas del menú.
 * Para añadir una herramienta nueva: implementa su `const tool_t` en un .c
 * dentro de tools/, decláralo aquí como extern y añádelo al array g_tools[].
 */
#include "tool.h"

extern const tool_t tool_clock;     /* Hora + Crono + Timer */
extern const tool_t tool_level;
extern const tool_t tool_gyro;
extern const tool_t tool_dice;
extern const tool_t tool_flashlight;
extern const tool_t tool_wifi;
extern const tool_t tool_wifiscan;
extern const tool_t tool_netinfo;
extern const tool_t tool_bt;

const tool_t *const g_tools[] = {
    &tool_clock,
    &tool_level,
    &tool_gyro,
    &tool_dice,
    &tool_flashlight,
    &tool_wifi,
    &tool_wifiscan,
    &tool_netinfo,
    &tool_bt,
};

const int g_tools_count = (int)(sizeof(g_tools) / sizeof(g_tools[0]));
