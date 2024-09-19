#!/bin/bash
while true
do
echo "keep plugged"
sudo picotool load -F build/Telemetrix4RpiPico.uf2
sudo picotool reboot
echo "unplug"
sleep 2
done