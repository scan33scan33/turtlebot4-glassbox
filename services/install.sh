#!/usr/bin/env bash
# Install + enable the auto-start services so the robot brings up the OAK camera
# and the nav control server (web UI :5000) on boot, and chimes when ready.
#   bash services/install.sh
set -e
# Portable: derive the services dir from this script's location; honors TB4_ROOT if set
D=${TB4_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}/services
# Fallback to the stock Pi path if the derived path doesn't exist (e.g. when run via sudo)
[ -d "$D" ] || D=/home/ubuntu/Workspace/turtlebot4-glassbox/services
chmod +x "$D/tb4-oakd-run.sh"
sudo cp "$D/tb4-oakd.service" "$D/tb4-nav.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tb4-oakd.service tb4-nav.service
echo "enabled on boot. start them now with:"
echo "  sudo systemctl start tb4-oakd tb4-nav"
echo "watch:   journalctl -u tb4-nav -f     stop: sudo systemctl stop tb4-nav tb4-oakd"
