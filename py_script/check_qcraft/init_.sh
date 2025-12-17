#!/bin/bash
# install-web-socket-service.sh

SERVICE_FILE="/etc/systemd/system/web-socket.service"
SCRIPT_DIR="/home/qcraft/ms/check_qcraft"

if [ ! -f "$SCRIPT_DIR/web_socket.py" ]; then
    echo "错误: web_socket.py 不存在于 $SCRIPT_DIR"
    exit 1
fi

sudo tee $SERVICE_FILE > /dev/null <<EOF
[Unit]
Description=Web Socket Monitoring Service
After=network.target
Wants=network.target

[Service]
Type=simple
User=root
Group=root
WorkingDirectory=$SCRIPT_DIR
ExecStart=/usr/bin/python3 $SCRIPT_DIR/web_socket.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload

sudo systemctl enable web-socket.service

echo "服务安装完成!"
echo "使用以下命令管理服务:"
echo "启动: sudo systemctl start web-socket.service"
echo "停止: sudo systemctl stop web-socket.service"
echo "状态: sudo systemctl status web-socket.service"
echo "日志: sudo journalctl -u web-socket.service -f"