#!/bin/bash

# 停止脚本
echo "正在停止服务..."

# 停止 HTTP 服务器
if [ -f /tmp/http_server.pid ]; then
    HTTP_PID=$(cat /tmp/http_server.pid)
    if kill $HTTP_PID 2>/dev/null; then
        echo "已停止 HTTP 服务器 (PID: $HTTP_PID)"
        rm /tmp/http_server.pid
    else
        echo "HTTP 服务器进程不存在或已停止"
        rm /tmp/http_server.pid
    fi
else
    echo "HTTP 服务器 PID 文件不存在，尝试查找并终止进程..."
    # 尝试通过端口查找并终止进程
    PID=$(lsof -ti:5173 2>/dev/null)
    if [ ! -z "$PID" ]; then
        echo "找到端口 5173 的进程: $PID，正在终止..."
        kill $PID 2>/dev/null
    fi
fi

# 停止 ROS2 后端
if [ -f /tmp/ros2_backend.pid ]; then
    ROS2_PID=$(cat /tmp/ros2_backend.pid)
    if kill $ROS2_PID 2>/dev/null; then
        echo "已停止 ROS2 后端 (PID: $ROS2_PID)"
        rm /tmp/ros2_backend.pid
    else
        echo "ROS2 后端进程不存在或已停止"
        rm /tmp/ros2_backend.pid
    fi
else
    echo "ROS2 后端 PID 文件不存在，尝试通过名称查找进程..."
    # 尝试通过进程名查找并终止
    PIDS=$(pgrep -f "python3 ros2-backend.py")
    if [ ! -z "$PIDS" ]; then
        echo "找到 ROS2 后端进程: $PIDS，正在终止..."
        kill $PIDS 2>/dev/null
    fi
fi

# 等待进程完全终止
sleep 1

# 再次检查是否还有相关进程运行
HTTP_RUNNING=$(pgrep -f "python3 -m http.server 5173")
ROS2_RUNNING=$(pgrep -f "python3 ros2-backend.py")

if [ -z "$HTTP_RUNNING" ] && [ -z "$ROS2_RUNNING" ]; then
    echo "所有服务已成功停止"
else
    echo "警告：仍有进程在运行，尝试强制终止..."
    # 强制终止
    pkill -f "python3 -m http.server 5173"
    pkill -f "python3 ros2-backend.py"
    sleep 1
    echo "强制终止完成"
fi
