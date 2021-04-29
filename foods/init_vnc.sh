export DISPLAY=:1
export VNC_RESOLUTION=1600x1200
export VNC_COL_DEPTH=24
export VNC_PORT=5900
export NOVNC_PORT=6080

sudo /usr/bin/Xvfb $DISPLAY -screen 0 ${VNC_RESOLUTION}x${VNC_COL_DEPTH} &
sleep 1
sudo rm -rf /tmp/.X1-lock
dbus-launch /usr/bin/startxfce4 2>/dev/null &
sudo x11vnc -display $DISPLAY -xkb -noxrecord -noxfixes -noxdamage -nopw -wait 1 -shared -forever -bg
/noVNC/utils/launch.sh --vnc localhost:$VNC_PORT --listen $NOVNC_PORT
