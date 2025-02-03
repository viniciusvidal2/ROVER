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
        echo "[INFO] Already connected to $SSID. Exiting..."
        exit 0
    fi

    echo "[WARNING] Not connected. Attempting to connect to $SSID..."
    
    # Attempt to connect to WiFi
    nmcli dev wifi connect "$SSID" password "$PASSWORD"
    
    # Wait a few seconds before checking again
    sleep 5
done
