#!/bin/bash
# CAN interface setup for Ranger Mini 2.0 on Jetson
# Usage: sudo bash can_bringup.sh [interface] [bitrate]
# Default: can1, 500000

IFACE="${1:-can1}"
BITRATE="${2:-500000}"

set -e

echo "[ranger_nav] Setting up CAN interface: ${IFACE} at ${BITRATE} bps"

# Load kernel module
if ! lsmod | grep -q gs_usb; then
    echo "[ranger_nav] Loading gs_usb kernel module..."
    sudo modprobe gs_usb
fi

# Bring down the interface first
sudo ip link set ${IFACE} down 2>/dev/null || true

# Configure and bring up
sudo ip link set ${IFACE} type can bitrate ${BITRATE}
sudo ip link set ${IFACE} up

# Verify
if ip link show ${IFACE} | grep -q "UP"; then
    echo "[ranger_nav] ${IFACE} is UP at ${BITRATE} bps"
else
    echo "[ranger_nav] ERROR: Failed to bring up ${IFACE}"
    exit 1
fi
