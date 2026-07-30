@echo off
title Home-lab - logger de sensores
set "LOG_DIR=%USERPROFILE%\Documents\homelab-logs"
echo ================================================================
echo   LOGGER DE SENSORES DEL HOME-LAB
echo.
echo   Guardando los CSV en:
echo   %LOG_DIR%
echo.
echo   Deja esta ventana ABIERTA para seguir registrando.
echo   Para PARAR: cerra la ventana (o Ctrl+C).
echo ================================================================
echo.
py "C:\Users\alons\Documents\Proyectos_ESP32\esp32_mini_tool\raspberry\logger\mqtt_logger.py"
echo.
echo (El logger se detuvo.)
pause
