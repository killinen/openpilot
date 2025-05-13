#!/usr/bin/env bash

# From here on, you're in real bash
set -e
echo "✅ Running in bash $BASH_VERSION"

TELETYPED_DIR="/data/openpilot/tools/teletyped"
RESOLV_SRC="$TELETYPED_DIR/resolv.conf"
RESOLV_DEST="/etc/resolv.conf"

# Create resolv.conf content if needed
if [ ! -f "$RESOLV_SRC" ] || ! grep -q "^nameserver 8.8.8.8$" "$RESOLV_SRC"; then
  echo "[+] Creating resolv.conf with 8.8.8.8"
  echo "nameserver 8.8.8.8" > "$RESOLV_SRC"
else
  echo "[=] resolv.conf already has correct content"
fi

# Remount /system writable temporarily
mount -o rw,remount /system

# Ensure resolv.conf exists
echo "[+] Ensuring /etc/resolv.conf exists"
touch "$RESOLV_DEST"

mount -o ro,remount /system

# Only bind-mount if it's not already bound to the source
CURRENT_MOUNT=$(mount | grep "on $RESOLV_DEST type" | awk '{print $1}')
if [ "$CURRENT_MOUNT" != "$RESOLV_SRC" ]; then
  echo "[+] Bind-mounting DNS fix"
  mount --bind "$RESOLV_SRC" "$RESOLV_DEST"
  echo "[✅] DNS configured via bind-mounted resolv.conf"
else
  echo "[=] resolv.conf already bind-mounted correctly"
fi

