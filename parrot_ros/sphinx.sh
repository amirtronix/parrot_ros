#!/bin/bash

sudo systemctl start firmwared.service

gnome-terminal -- sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::firmware="https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip"::wifi_iface=""

gnome-terminal -- parrot-ue4-empty -ams-path="DefaultPath,Pickup:*" -ams-path="RunPath,Jasper"

