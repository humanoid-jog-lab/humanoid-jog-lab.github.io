#!/bin/bash

export ROS_DOMAIN_ID=55
source /opt/ros/humble/setup.bash
source /home/user/joint_controller/install/setup.bash

# 启动第一个服务（HTTP服务器）
echo "启动 HTTP 服务器..."
cd /home/user/dq_vizlab/dist || { echo "目录不存在: /home/user/dq_vizlab/dist"; exit 1; }
HTTP_RUNNING=$(pgrep -f "python3 -m http.server 5173")
if [ -n "$HTTP_RUNNING" ]; then
  echo "HTTP服务存在，强制终止"
  kill -9 $HTTP_RUNNING
fi

HTTP_PORT=5173
nohup python3 -m http.server $HTTP_PORT > http_server.log 2>&1 &
HTTP_PID=$!
echo $HTTP_PID > /tmp/http_server.pid
echo "HTTP 服务器启动完成, 端口号: $HTTP_PORT，PID: $HTTP_PID，日志: http_server.log"


# 启动第二个服务（ROS2后端）
echo "启动 ROS2 后端..."
cd /home/user/dq_vizlab/server/ || { echo "目录不存在: /home/user/dq_vizlab/server/"; exit 1; }
ROS2_BACKEND_RUNNING=$(pgrep -f "python3 ros2-backend.py")
if [ -n "$ROS2_BACKEND_RUNNING" ]; then
  echo "ROS2后端服务存在，强制终止"
  kill -9 $ROS2_BACKEND_RUNNING
fi

BACKEND_PORT=8082
nohup python3 ros2-backend.py $BACKEND_PORT > ros2_backend.log 2>&1 &
ROS2_PID=$!
echo $ROS2_PID > /tmp/ros2_backend.pid
echo "ROS2 后端启动完成, 端口: $BACKEND_PORT，PID: $ROS2_PID，日志: ros2_backend.log"

echo "所有服务已启动"
echo "HTTP 服务器 PID: $HTTP_PID (保存于 /tmp/http_server.pid)"
echo "ROS2 后端 PID: $ROS2_PID (保存于 /tmp/ros2_backend.pid)"
