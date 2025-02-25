#!/bin/bash

SSID="rover"
PASSWORD="rover"

echo "[INFO] Starting WiFi connection script"

while true; do
    # Check if we are connected to the desired WiFi
    if iwgetid -r | grep -q "$SSID"; then
        echo "[INFO] Already connected to $SSID. Checking again later..."
    else
        echo "[WARNING] Not connected. Searching for $SSID..."
        
        # Get a list of available networks
        AVAILABLE_NETWORKS=$(nmcli -t -f SSID dev wifi list | grep -v '^$')

        if echo "$AVAILABLE_NETWORKS" | grep -q "$SSID"; then
            echo "[INFO] $SSID found. Attempting to connect..."
            nmcli dev wifi connect "$SSID" password "$PASSWORD"
        else
            echo "[INFO] $SSID not found. Scanning for networks..."
            nmcli dev wifi rescan
            sleep 10  # Give mandatory time for rescan to complete

            # Try connecting again after rescan
            if nmcli -t -f SSID dev wifi list | grep -q "$SSID"; then
                echo "[INFO] $SSID found after rescan. Attempting to connect..."
                nmcli dev wifi connect "$SSID" password "$PASSWORD"
            else
                echo "[ERROR] $SSID not found even after rescan. Retrying in 10 seconds..."
            fi
        fi
    fi

    # Wait before checking again
    sleep 5
done
