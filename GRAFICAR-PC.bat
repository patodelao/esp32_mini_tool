@echo off
title Home-lab - graficar sensores
echo ================================================================
echo   GRAFICAR SENSORES DEL HOME-LAB
echo   Lee los CSV de:  %USERPROFILE%\Documents\homelab-logs
echo   y crea un PNG por sensor en la subcarpeta 'graficos'.
echo ================================================================
echo.
py "C:\Users\alons\Documents\Proyectos_ESP32\esp32_mini_tool\raspberry\logger\graficar_sensores.py"
echo.
echo Abriendo la carpeta de graficos...
start "" "%USERPROFILE%\Documents\homelab-logs\graficos"
pause
