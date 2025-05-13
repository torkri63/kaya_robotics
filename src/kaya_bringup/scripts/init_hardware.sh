#!/usr/bin/env bash
# Initialize hardware drivers and interfaces
set -e

echo "[bringup] Loading kernel modules..."
sudo modprobe spi_bcm2835
sudo modprobe i2c_dev

echo "[bringup] Setting up device permissions..."
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "[bringup] Launching hardware driver nodes"
ros2 launch kaya_bringup hardware.launch.py