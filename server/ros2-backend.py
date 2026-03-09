#!/usr/bin/env python3
"""
ROS 2 后端服务 - Python 版本
运行在 Python 中，处理与 ROS 2 的通讯
通过 WebSocket 向前端浏览器提供 ROS 2 功能

依赖：
    pip install websockets rclpy asyncio

运行：
    python server/ros2-backend.py
"""

import asyncio
import json
import logging
from typing import Dict, Set, Optional
import os

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# 使用logger
logger = logging.getLogger(__name__)

logger.info(f"ROS_DOMAIN_ID: {os.environ['ROS_DOMAIN_ID']}")

DEFAULT_REAL_JOINT_STATE_TOPIC = '/whole/joint_states'

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("rclpy 未安装，ROS2 功能不可用")

try:
    import websockets
    from websockets.server import WebSocketServerProtocol
    from websockets.server import serve
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    logging.error("websockets 未安装，请运行: pip install websockets")

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 导入自定义消息类型（需要先编译你的消息包）
try:
    from robot_control_msg.msg import Robotarmservomsg
    ROBOT_ARM_MSG_AVAILABLE = True
except ImportError:
    Robotarmservomsg = None
    ROBOT_ARM_MSG_AVAILABLE = False
    logger.warning("Robotarmservomsg 未找到，将禁用机械臂自定义消息发布")

try:
    from robot_control_msg.msg import Robotservomsg
    ROBOT_SERVO_MSG_AVAILABLE = True
except ImportError:
    Robotservomsg = None
    ROBOT_SERVO_MSG_AVAILABLE = False
    logger.warning("Robotservomsg 未找到，将禁用腰部自定义消息发布")


class ROS2Node(Node):
    """ROS2 节点，处理订阅和发布"""
    
    def __init__(self):
        super().__init__('dq_vizlab_node')
        # 使用下划线前缀避免与基类属性冲突
        self._topic_subscriptions: Dict[str, any] = {}
        self._topic_publishers: Dict[str, any] = {}
        self._message_callbacks: Dict[str, callable] = {}
        
        # QoS 配置
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        logger.info("ROS2 节点已创建")
    
    def subscribe_topic(self, topic_name: str, message_type: str, callback: callable):
        """订阅话题"""
        if topic_name in self._topic_subscriptions:
            logger.info(f"话题 {topic_name} 已订阅，复用现有订阅")
            return
        
        try:
            # 根据消息类型创建订阅者
            if message_type == 'sensor_msgs/msg/JointState':
                from sensor_msgs.msg import JointState
                sub = self.create_subscription(
                    JointState,
                    topic_name,
                    callback,
                    self.qos_profile
                )
            elif message_type == 'sensor_msgs/msg/BatteryState':
                from sensor_msgs.msg import BatteryState
                sub = self.create_subscription(
                    BatteryState,
                    topic_name,
                    callback,
                    self.qos_profile
                )
            elif message_type == 'std_msgs/msg/String':
                sub = self.create_subscription(
                    String,
                    topic_name,
                    callback,
                    self.qos_profile
                )
            else:
                logger.error(f"不支持的消息类型: {message_type}")
                return None
            
            self._topic_subscriptions[topic_name] = sub
            self._message_callbacks[topic_name] = callback
            logger.info(f"已订阅话题: {topic_name} ({message_type})")
            return sub
        except Exception as e:
            logger.error(f"订阅话题失败 {topic_name}: {e}")
            return None
    
    def unsubscribe_topic(self, topic_name: str):
        """取消订阅"""
        if topic_name in self._topic_subscriptions:
            self.destroy_subscription(self._topic_subscriptions[topic_name])
            del self._topic_subscriptions[topic_name]
            if topic_name in self._message_callbacks:
                del self._message_callbacks[topic_name]
            logger.info(f"已取消订阅: {topic_name}")
    
    def create_publisher_for_topic(self, topic_name: str, message_type: str):
        """为话题创建发布者"""
        if topic_name in self._topic_publishers:
            return self._topic_publishers[topic_name]
        
        try:
            if message_type == 'robot_control_msg/msg/Robotarmservomsg':
                if ROBOT_ARM_MSG_AVAILABLE:
                    pub = self.create_publisher(Robotarmservomsg, topic_name, self.qos_profile)
                else:
                    logger.error("Robotarmservomsg 不可用")
                    return None
            elif message_type == 'robot_control_msg/msg/Robotservomsg':
                if ROBOT_SERVO_MSG_AVAILABLE:
                    pub = self.create_publisher(Robotservomsg, topic_name, self.qos_profile)
                else:
                    logger.error("Robotservomsg 不可用")
                    return None
            
            else:
                logger.error(f"不支持的消息类型: {message_type}")
                return None
            
            self._topic_publishers[topic_name] = pub
            logger.info(f"已创建发布者: {topic_name} ({message_type})")
            return pub
        except Exception as e:
            logger.error(f"创建发布者失败 {topic_name}: {e}")
            return None
    
    def get_current_time(self):
        """获取当前 ROS2 时间"""
        return self.get_clock().now()


class ROS2BackendServer:
    """ROS2 后端服务器，处理 WebSocket 连接"""
    
    def __init__(self, port: int = 8082):
        self.port = port
        self.ros2_node: Optional[ROS2Node] = None
        self.ros2_initialized = False
        self.clients: Set[WebSocketServerProtocol] = set()
        self.client_subscriptions: Dict[WebSocketServerProtocol, Set[str]] = {}
        self.topic_subscribers: Dict[str, int] = {}
        self.event_loop: Optional[asyncio.AbstractEventLoop] = None
        self.robot_control_mode = "manual"
    
    async def broadcast_to_all(self, message: dict):
        """向所有客户端广播消息"""
        message_str = json.dumps(message)
        disconnected = []
        for client in self.clients:
            try:
                await client.send(message_str)
            except Exception as e:
                logger.error(f"发送消息失败: {e}")
                disconnected.append(client)
        
        # 清理断开的客户端
        for client in disconnected:
            await self.cleanup_client(client)
    
    async def cleanup_client(self, ws: WebSocketServerProtocol):
        """清理客户端资源"""
        if ws in self.client_subscriptions:
            topics = self.client_subscriptions[ws]
            for topic in topics:
                count = self.topic_subscribers.get(topic, 0) - 1
                if count <= 0:
                    if self.ros2_node and topic in self.ros2_node._topic_subscriptions:
                        self.ros2_node.unsubscribe_topic(topic)
                    self.topic_subscribers.pop(topic, None)
                else:
                    self.topic_subscribers[topic] = count
            del self.client_subscriptions[ws]
        
        self.clients.discard(ws)
        logger.info("客户端已清理")
    
    async def handle_init(self, ws: WebSocketServerProtocol):
        """初始化 ROS2"""
        if not ROS2_AVAILABLE:
            await ws.send(json.dumps({
                'type': 'error',
                'message': 'ROS2 (rclpy) 未安装'
            }))
            return
        
        if self.ros2_initialized:
            await ws.send(json.dumps({
                'type': 'status',
                'status': 'connected',
                'message': 'ROS 2 已连接'
            }))
            return
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros2_node = ROS2Node()
            
            # 在后台运行 ROS2 节点
            def spin_ros2():
                rclpy.spin(self.ros2_node)
            
            # 在单独的线程中运行
            import threading
            ros2_thread = threading.Thread(target=spin_ros2, daemon=True)
            ros2_thread.start()
            
            self.ros2_initialized = True
            
            await ws.send(json.dumps({
                'type': 'status',
                'status': 'connected',
                'message': 'ROS 2 已初始化并连接'
            }))
            
            logger.info("✓ ROS 2 已初始化")
        except Exception as e:
            logger.error(f"ROS 2 初始化失败: {e}")
            await ws.send(json.dumps({
                'type': 'error',
                'message': str(e)
            }))
    
    async def handle_shutdown(self, ws: WebSocketServerProtocol):
        """关闭 ROS2"""
        if self.ros2_node:
            self.ros2_node.destroy_node()
            self.ros2_node = None
        
        if rclpy.ok():
            rclpy.shutdown()
        
        self.ros2_initialized = False
        self.topic_subscribers.clear()
        
        await ws.send(json.dumps({
            'type': 'status',
            'status': 'disconnected',
            'message': 'ROS 2 已断开连接'
        }))
    
    async def handle_subscribe(self, ws: WebSocketServerProtocol, data: dict):
        """订阅话题"""
        if not self.ros2_node or not self.ros2_initialized:
            raise Exception('ROS 2 未初始化')
        
        topic_name = data.get('topicName')
        message_type = data.get('messageType')
        
        if not topic_name or not message_type:
            raise Exception('话题名称和消息类型不能为空')

        if topic_name == DEFAULT_REAL_JOINT_STATE_TOPIC and message_type == 'sensor_msgs/msg/JointState':
            logger.info(f'订阅整机 JointState 话题: {topic_name}')
        
        client_topics = self.client_subscriptions.setdefault(ws, set())
        
        if topic_name in client_topics:
            await ws.send(json.dumps({
                'type': 'subscribed',
                'topic': topic_name,
                'message': f'已订阅: {topic_name}（重复请求）'
            }))
            return
        
        current_count = self.topic_subscribers.get(topic_name, 0)
        
        if current_count == 0:
            def message_callback(msg):
                try:
                    # 将 ROS2 消息转换为字典
                    msg_dict = self.ros2_msg_to_dict(msg, message_type)
                    
                    # 调试信息
                    if isinstance(msg_dict, dict):
                        keys = list(msg_dict.keys())
                        logger.debug(f"收到消息 {topic_name}: 转换成功, 键={keys[:5]}...")  # 只显示前5个键
                    else:
                        logger.warning(f"收到消息 {topic_name}: 转换失败, 返回类型={type(msg_dict)}, 值={str(msg_dict)[:100]}")
                        # 如果转换失败，尝试直接序列化
                        try:
                            # 尝试使用 getattr 直接访问消息字段
                            if hasattr(msg, 'name') and hasattr(msg, 'position'):
                                msg_dict = {
                                    'header': self.ros2_msg_to_dict(msg.header, 'std_msgs/msg/Header') if hasattr(msg, 'header') else {},
                                    'name': list(msg.name) if hasattr(msg.name, '__iter__') else [],
                                    'position': list(msg.position) if hasattr(msg.position, '__iter__') else [],
                                    'velocity': list(msg.velocity) if hasattr(msg, 'velocity') and hasattr(msg.velocity, '__iter__') else [],
                                    'effort': list(msg.effort) if hasattr(msg, 'effort') and hasattr(msg.effort, '__iter__') else []
                                }
                                logger.info(f"使用直接访问方式转换消息 {topic_name} 成功")
                        except Exception as e2:
                            logger.error(f"直接访问消息字段也失败: {e2}", exc_info=True)
                            return  # 放弃这条消息
                    
                    message_to_send = {
                        'type': 'message',
                        'topic': topic_name,
                        'data': msg_dict
                    }
                    
                    # 使用线程安全的方式将消息传递到主事件循环
                    if self.event_loop and self.event_loop.is_running():
                        try:
                            asyncio.run_coroutine_threadsafe(
                                self.broadcast_to_all(message_to_send),
                                self.event_loop
                            )
                        except Exception as e:
                            logger.error(f"广播消息失败 {topic_name}: {e}", exc_info=True)
                    else:
                        logger.warning(f"事件循环不可用，无法广播消息 {topic_name}")
                except Exception as e:
                    logger.error(f"处理消息回调失败 {topic_name}: {e}", exc_info=True)
            
            self.ros2_node.subscribe_topic(topic_name, message_type, message_callback)
        
        client_topics.add(topic_name)
        self.topic_subscribers[topic_name] = current_count + 1
        
        await ws.send(json.dumps({
            'type': 'subscribed',
            'topic': topic_name,
            'message': f'已订阅: {topic_name}'
        }))
    
    async def handle_publish_robot_arm_servo_msg(self, ws: WebSocketServerProtocol, data: dict):
        """发布机械臂控制消息"""
        if not self.ros2_node or not self.ros2_initialized:
            raise Exception('ROS 2 未初始化')
        
        topic_name = data.get('topicName')
        message_data = data.get('messageData', {})
        
        if not topic_name:
            raise Exception('话题名称不能为空')
        
        message_type = 'robot_control_msg/msg/Robotarmservomsg'
        publisher = self.ros2_node.create_publisher_for_topic(topic_name, message_type)
        
        if not publisher:
            raise Exception('创建发布者失败')
        
        if ROBOT_ARM_MSG_AVAILABLE:
            msg = Robotarmservomsg()
            
            # 填充左臂关节位置
            msg.ljoint1_position = float(message_data.get('ljoint1_position', 0))
            msg.ljoint2_position = float(message_data.get('ljoint2_position', 0))
            msg.ljoint3_position = float(message_data.get('ljoint3_position', 0))
            msg.ljoint4_position = float(message_data.get('ljoint4_position', 0))
            msg.ljoint5_position = float(message_data.get('ljoint5_position', 0))
            msg.ljoint6_position = float(message_data.get('ljoint6_position', 0))
            msg.ljoint7_position = float(message_data.get('ljoint7_position', 0))
            
            # 填充右臂关节位置
            msg.rjoint1_position = float(message_data.get('rjoint1_position', 0))
            msg.rjoint2_position = float(message_data.get('rjoint2_position', 0))
            msg.rjoint3_position = float(message_data.get('rjoint3_position', 0))
            msg.rjoint4_position = float(message_data.get('rjoint4_position', 0))
            msg.rjoint5_position = float(message_data.get('rjoint5_position', 0))
            msg.rjoint6_position = float(message_data.get('rjoint6_position', 0))
            msg.rjoint7_position = float(message_data.get('rjoint7_position', 0))
            
            # 其他字段
            msg.run_mode = int(message_data.get('run_mode', 0))
            msg.robot_power = bool(message_data.get('robot_power', False))
            msg.run_time = float(message_data.get('run_time', 0))
            
            # 设置时间戳
            msg.stamp = self.ros2_node.get_current_time().to_msg()
            
            publisher.publish(msg)
            
            await ws.send(json.dumps({
                'type': 'published',
                'topic': topic_name,
                'message': f'机械臂控制消息已发送到: {topic_name}'
            }))
        else:
            raise Exception('Robotarmservomsg 消息类型不可用')

    async def handle_publish_robot_servo_msg(self, ws: WebSocketServerProtocol, data: dict):
        """发布腰部/下肢控制消息"""
        if not self.ros2_node or not self.ros2_initialized:
            raise Exception('ROS 2 未初始化')

        topic_name = data.get('topicName')
        message_data = data.get('messageData', {})

        if not topic_name:
            raise Exception('话题名称不能为空')

        message_type = 'robot_control_msg/msg/Robotservomsg'
        publisher = self.ros2_node.create_publisher_for_topic(topic_name, message_type)

        if not publisher:
            raise Exception('创建发布者失败')

        if ROBOT_SERVO_MSG_AVAILABLE:
            msg = Robotservomsg()
            msg.run_mode = int(message_data.get('run_mode', 0))
            msg.ankle_position = float(message_data.get('ankle_position', 0))
            msg.knee_position = float(message_data.get('knee_position', 0))
            msg.hip_pitch_position = float(message_data.get('hip_pitch_position', 0))
            msg.hip_yaw_position = float(message_data.get('hip_yaw_position', 0))
            msg.x = float(message_data.get('x', 0))
            msg.y = float(message_data.get('y', 0))
            msg.phi = float(message_data.get('phi', 0))
            msg.mode_leg_select = int(message_data.get('mode_leg_select', 0))
            msg.vel = float(message_data.get('vel', 0.1))

            publisher.publish(msg)

            await ws.send(json.dumps({
                'type': 'published',
                'topic': topic_name,
                'message': f'腰部控制消息已发送到: {topic_name}'
            }))
        else:
            raise Exception('Robotservomsg 消息类型不可用')
    
    async def handle_unsubscribe(self, ws: WebSocketServerProtocol, data: dict):
        """取消订阅话题"""
        if not self.ros2_node or not self.ros2_initialized:
            raise Exception('ROS 2 未初始化')
        
        topic_name = data.get('topicName')
        if not topic_name:
            raise Exception('话题名称不能为空')
        
        client_topics = self.client_subscriptions.get(ws)
        if not client_topics or topic_name not in client_topics:
            await ws.send(json.dumps({
                'type': 'warn',
                'message': f'未订阅话题: {topic_name}'
            }))
            return
        
        client_topics.remove(topic_name)
        count = self.topic_subscribers.get(topic_name, 0) - 1
        
        if count <= 0:
            self.ros2_node.unsubscribe_topic(topic_name)
            self.topic_subscribers.pop(topic_name, None)
        else:
            self.topic_subscribers[topic_name] = count
        
        await ws.send(json.dumps({
            'type': 'unsubscribed',
            'topic': topic_name,
            'message': f'已取消订阅: {topic_name}'
        }))
    
    def ros2_msg_to_dict(self, msg, msg_type: str) -> dict:
        """将 ROS2 消息转换为字典（类似 Node.js 的 normalizeMessageData）"""
        if msg is None:
            return None
        
        # 处理基本类型
        if isinstance(msg, (str, int, float, bool)):
            return msg
        
        # 处理列表/数组
        if isinstance(msg, (list, tuple)):
            return [self._normalize_value(item) for item in msg]
        
        # 处理字典
        if isinstance(msg, dict):
            return {k: self._normalize_value(v) for k, v in msg.items()}
        
        # 专门处理 JointState 消息
        if msg_type == 'sensor_msgs/msg/JointState' or type(msg).__name__ == 'JointState':
            return self._convert_joint_state_to_dict(msg)
        
        # 处理 ROS2 消息对象
        result = {}
        
        # 方法1: 尝试使用 get_fields_and_field_types（ROS2 Python 消息的标准方法）
        try:
            if hasattr(msg, 'get_fields_and_field_types'):
                field_types = msg.get_fields_and_field_types()
                for field_name in field_types.keys():
                    value = getattr(msg, field_name, None)
                    result[field_name] = self._normalize_value(value)
                if result:
                    return result
        except Exception as e:
            logger.debug(f"get_fields_and_field_types 方法失败: {e}")
        
        # 方法2: 尝试使用 __slots__（ROS2 消息通常使用这个）
        try:
            if hasattr(msg, '__slots__'):
                for slot in msg.__slots__:
                    if not slot.startswith('_'):
                        value = getattr(msg, slot, None)
                        result[slot] = self._normalize_value(value)
                if result:
                    return result
        except Exception as e:
            logger.debug(f"__slots__ 方法失败: {e}")
        
        # 方法3: 尝试使用 __dict__
        try:
            if hasattr(msg, '__dict__'):
                for key, value in msg.__dict__.items():
                    if not key.startswith('_'):
                        result[key] = self._normalize_value(value)
                if result:
                    return result
        except Exception as e:
            logger.debug(f"__dict__ 方法失败: {e}")
        
        # 如果仍然没有结果，返回空字典
        if not result:
            logger.warning(f"无法转换消息类型 {type(msg).__name__}，返回空字典")
            return {}
        
        return result
    
    def _convert_joint_state_to_dict(self, msg) -> dict:
        """专门处理 JointState 消息的转换"""
        result = {}
        try:
            joint_names = []

            # Header
            if hasattr(msg, 'header'):
                header = msg.header
                result['header'] = {
                    'stamp': self._normalize_value(header.stamp) if hasattr(header, 'stamp') else {},
                    'frame_id': getattr(header, 'frame_id', '')
                }
            
            # Name (关节名称列表)
            if hasattr(msg, 'name'):
                joint_names = [str(name) for name in msg.name] if hasattr(msg.name, '__iter__') else []
                result['name'] = joint_names

            joint_count = len(joint_names)

            # Position (关节位置列表)
            if hasattr(msg, 'position'):
                positions = msg.position
                if hasattr(positions, '__iter__') and not isinstance(positions, str):
                    normalized = [float(p) for p in positions]
                    result['position'] = normalized[:joint_count] if joint_count > 0 else normalized
                else:
                    result['position'] = []
            
            # Velocity (关节速度列表)
            if hasattr(msg, 'velocity'):
                velocities = msg.velocity
                if hasattr(velocities, '__iter__') and not isinstance(velocities, str):
                    normalized = [float(v) for v in velocities]
                    result['velocity'] = normalized[:joint_count] if joint_count > 0 else normalized
                else:
                    result['velocity'] = []
            
            # Effort (关节力矩列表)
            if hasattr(msg, 'effort'):
                efforts = msg.effort
                if hasattr(efforts, '__iter__') and not isinstance(efforts, str):
                    normalized = [float(e) for e in efforts]
                    result['effort'] = normalized[:joint_count] if joint_count > 0 else normalized
                else:
                    result['effort'] = []

            result['joint_count'] = joint_count
            
        except Exception as e:
            logger.error(f"转换 JointState 消息失败: {e}", exc_info=True)
            return {}
        
        return result
    
    def _normalize_value(self, value):
        """递归规范化值"""
        if value is None:
            return None
        
        if isinstance(value, (str, int, float, bool)):
            return value
        
        if isinstance(value, (list, tuple)):
            return [self._normalize_value(item) for item in value]
        
        if isinstance(value, dict):
            return {k: self._normalize_value(v) for k, v in value.items()}
        
        # 处理 ROS2 时间对象
        if hasattr(value, 'sec') and hasattr(value, 'nanosec'):
            return {
                'sec': int(value.sec),
                'nanosec': int(value.nanosec)
            }
        
        # 处理 ROS2 消息对象（嵌套）
        if hasattr(value, '__dict__') or hasattr(value, '_fields'):
            return self.ros2_msg_to_dict(value, '')
        
        # 对于 numpy 数组或其他特殊类型，尝试转换为列表
        try:
            if hasattr(value, 'tolist'):
                return value.tolist()
            # 处理可迭代对象（但排除字符串）
            if hasattr(value, '__iter__') and not isinstance(value, (str, bytes)):
                # 检查是否是已知的序列类型
                if isinstance(value, (list, tuple, set)):
                    return [self._normalize_value(item) for item in value]
                # 尝试转换为列表
                try:
                    return [self._normalize_value(item) for item in iter(value)]
                except:
                    pass
        except Exception as e:
            logger.debug(f"转换可迭代对象失败: {e}")
            pass
        
        # 最后回退：尝试转换为字符串或返回原值
        try:
            # 如果值看起来像是一个对象，尝试获取其字符串表示
            if hasattr(value, '__class__'):
                return str(value)
            return value
        except:
            return value
    
    async def handle_client_message(self, ws: WebSocketServerProtocol, message: dict):
        """处理客户端消息"""
        msg_type = message.get('type')
        data = message.get('data', {})
        
        try:
            if msg_type == 'init':
                await self.handle_init(ws)
            elif msg_type == 'shutdown':
                await self.handle_shutdown(ws)
            elif msg_type == 'subscribe':
                await self.handle_subscribe(ws, data)
            elif msg_type == 'unsubscribe':
                await self.handle_unsubscribe(ws, data)
            elif msg_type == 'publishRobotArmServoMsg':
                await self.handle_publish_robot_arm_servo_msg(ws, data)
            elif msg_type == 'publishRobotServoMsg':
                await self.handle_publish_robot_servo_msg(ws, data)
            elif msg_type == 'getStatus':
                await ws.send(json.dumps({
                    'type': 'status',
                    'status': 'connected' if self.ros2_initialized else 'not-initialized'
                }))
            elif msg_type == "switchMode":
                logger.info(f"switch mode ===> {data}")
                self.robot_control_mode = data
                result =  await self.handle_cotrol_mode_switch(data)
                await ws.send(json.dumps(result))
            elif msg_type == "getMode":
                logger.info(f"get current mode: {self.robot_control_mode}")
                await ws.send(json.dumps({
                    'type': 'mode',
                    "mode": self.robot_control_mode
                }))
            else:
                logger.warning(f'未知的消息类型: {msg_type}')
        except Exception as e:
            logger.error(f'处理 {msg_type} 消息失败: {e}')
            await ws.send(json.dumps({
                'type': 'error',
                'message': str(e),
                'originalType': msg_type
            }))
    

    async def handle_cotrol_mode_switch(self, mode):
        try:
            if mode == "manual":
                logger.info("切换manual模式...")
                os.system("/home/user/joint_controller/stop_tele_op.sh") 
                logger.info("已切换manual模式...")
            elif mode == "teleop":
                logger.info("切换teleop模式...")
                os.system("/home/user/joint_controller/start_tele_op.sh")
                logger.info("已切换teleop模式...")
            else:
                raise ValueError(f"未知的模式: {mode}") 
            return {"type": "success", "message": f"切换到模式 {mode} 成功"}
        except Exception as ex:
            logger.error(f"模式切换失败: {ex}")
            return {"type": "error", "message": f"模式切换失败: {ex}"}


    async def handle_client(self, ws: WebSocketServerProtocol, path: str):
        """处理客户端连接"""
        logger.info("✓ 新客户端连接")
        self.clients.add(ws)
        self.client_subscriptions[ws] = set()
        
        try:
            async for message in ws:
                try:
                    data = json.loads(message)
                    await self.handle_client_message(ws, data)
                except json.JSONDecodeError:
                    logger.error("无效的 JSON 消息")
                    await ws.send(json.dumps({
                        'type': 'error',
                        'message': '无效的 JSON 格式'
                    }))
        except websockets.exceptions.ConnectionClosed:
            logger.info("客户端断开连接")
        finally:
            await self.cleanup_client(ws)
    
    async def start(self):
        """启动服务器"""
        if not WEBSOCKETS_AVAILABLE:
            logger.error("websockets 库未安装，请运行: pip install websockets")
            return
        
        # 保存事件循环引用，供 ROS2 回调使用
        self.event_loop = asyncio.get_running_loop()
        
        logger.info(f"启动 WebSocket 服务器，端口: {self.port}")
        async with serve(self.handle_client, "0.0.0.0", self.port):
            await asyncio.Future()  # 永久运行


def main():
    """主函数"""
    server = ROS2BackendServer(port=8081)
    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        logger.info("服务器已停止")


if __name__ == '__main__':
    main()
