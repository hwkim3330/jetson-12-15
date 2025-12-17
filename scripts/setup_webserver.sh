#!/bin/bash
# KETI Robot Web Server Setup Script
# Run with sudo: sudo ./setup_webserver.sh

set -e

WEB_SRC="/home/nvidia/ros2_ws/src/keti/robot_web/www"
WEB_DST="/var/www/robot"
NGINX_CONF="/etc/nginx/sites-available/robot"

echo "=== KETI Robot Web Server Setup ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo $0"
    exit 1
fi

# Install nginx if needed
if ! command -v nginx &> /dev/null; then
    echo "Installing nginx..."
    apt-get update
    apt-get install -y nginx
fi

# Create web directory
echo "Setting up web directory..."
mkdir -p "$WEB_DST"
cp -rL "$WEB_SRC"/* "$WEB_DST"/
chown -R www-data:www-data "$WEB_DST"
chmod -R 755 "$WEB_DST"

# Create nginx configuration
echo "Configuring nginx..."
cat > "$NGINX_CONF" << 'EOF'
server {
    listen 80 default_server;
    listen [::]:80 default_server;

    root /var/www/robot;
    index index.html;

    server_name _;

    # Main web interface
    location / {
        try_files $uri $uri/ =404;
    }

    # Rosbridge WebSocket proxy
    location /rosbridge {
        proxy_pass http://127.0.0.1:9090;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_read_timeout 86400;
    }

    # web_video_server proxy
    location /stream {
        proxy_pass http://127.0.0.1:8080;
        proxy_http_version 1.1;
        proxy_buffering off;
    }

    # CORS headers
    add_header Access-Control-Allow-Origin *;
    add_header Access-Control-Allow-Methods "GET, POST, OPTIONS";
    add_header Access-Control-Allow-Headers "Content-Type";
}
EOF

# Enable site
ln -sf "$NGINX_CONF" /etc/nginx/sites-enabled/robot
rm -f /etc/nginx/sites-enabled/default

# Test and restart nginx
echo "Testing nginx configuration..."
nginx -t

echo "Starting nginx..."
systemctl enable nginx
systemctl restart nginx

echo ""
echo "=== Web Server Setup Complete ==="
echo ""
echo "Access the web interface at:"
echo "  http://10.42.0.1/ (AP mode)"
echo "  http://$(hostname -I | awk '{print $1}')/ (current IP)"
echo ""
echo "Make sure to also run:"
echo "  - rosbridge: ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo "  - robot: ros2 launch robot_bringup robot.launch.py"
