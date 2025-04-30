#!/usr/bin/env bash
# Complete bringup: hardware + software
set -e

echo "[bringup] Starting full robot bringup"
ros2 launch kaya_bringup bringup.launch.py