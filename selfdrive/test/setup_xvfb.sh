#!/usr/bin/env bash

# Sets up a virtual display for running map renderer and simulator without an X11 display

DISP_ID=99
export DISPLAY=:$DISP_ID

XVFB_BIN=$(command -v Xvfb || command -v /usr/bin/Xvfb)
if [ -n "$XVFB_BIN" ]; then
  echo "Starting Xvfb as $(whoami) using $XVFB_BIN"
  "$XVFB_BIN" $DISPLAY -screen 0 2160x1080x24 -ac -nolisten tcp 2>/dev/null &
else
  echo "Xvfb binary not found, falling back to sudo"
  sudo Xvfb $DISPLAY -screen 0 2160x1080x24 -ac -nolisten tcp 2>/dev/null &
fi

# check for x11 socket for the specified display ID
while [ ! -S /tmp/.X11-unix/X$DISP_ID ]
do
  echo "Waiting for Xvfb..."
  sleep 1
done

: "${XAUTHORITY:=$HOME/.Xauthority}"
touch "$XAUTHORITY"
export XDG_SESSION_TYPE="x11"
xset -q
