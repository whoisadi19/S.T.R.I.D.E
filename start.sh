#!/bin/bash
set -e

# Setup display
export DISPLAY=:0
export RESOLUTION=${RESOLUTION:-1280x720}
export LIBGL_ALWAYS_SOFTWARE=1

# Start Xvfb (Virtual Framebuffer)
rm -rf /tmp/.X*
Xvfb $DISPLAY -screen 0 ${RESOLUTION}x24 &
sleep 1

# Start window manager (fluxbox)
fluxbox &
sleep 1

# Start x11vnc without password
x11vnc -display $DISPLAY -nopw -listen localhost -xkb -ncache 10 -ncache_cr -forever &
sleep 1

# Start NoVNC using websockify
websockify --web=/usr/share/novnc/ 8080 localhost:5900 &

echo "=================================================="
echo " NoVNC is running at: http://localhost:8080"
echo " ROS 2 Environment is ready!"
echo "=================================================="

# Setup ROS 2 environment
source /opt/ros/humble/setup.bash

# Keep container running with a bash shell
exec /bin/bash
