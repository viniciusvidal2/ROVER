#!/bin/bash

SSID="rede"
PASSWORD="asenhamudou"
LOGFILE="/var/log/wifi_reconnect.log"

# Redirect output to log file
exec &>> "$LOGFILE"

echo "[INFO] Starting WiFi connection script at $(date)"

while true; do
    # Check if we are connected to WiFi
    if iwgetid -r | grep -q "$SSID"; then
        echo "[INFO] Already connected to $SSID. Checking again later..."
    else
        echo "[WARNING] Not connected. Attempting to connect to $SSID..."
        nmcli dev wifi connect "$SSID" password "$PASSWORD"
    fi

    # Wait before checking again
    sleep 5
done

