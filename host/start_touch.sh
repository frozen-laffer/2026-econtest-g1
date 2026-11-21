#!/bin/bash
cd /home/frozen-laffer/lsa || exit 1
export DISPLAY=:0
export XAUTHORITY=/home/frozen-laffer/.Xauthority
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run/user/1000
exec python3 lsa_touch.py "$@"