#!/bin/bash

cd build/
sudo picotool load Telemetrix4RpiPico.uf2 -F 
sudo picotool reboot