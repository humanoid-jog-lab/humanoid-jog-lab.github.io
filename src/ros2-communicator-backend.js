/**
 * ROS 2 通讯核心模块 - 后端版本（Node.js）
 * 在 Node.js 服务器中运行，直接使用 rclnodejs
 */

export class ROS2Communicator {
    constructor() {
        this.node = null;
        this.rclnodejs = null;
        this.isRclInitialized = false;
        this.isInitialized = false;
        this.publishers = new Map();
        this.subscriptions = new Map();
        this.topics = [];
        this.services = [];
        this.ros2Status = 'disconnected'; // disconnected, connecting, connected, error
        this.statusCallbacks = [];
        this.messageCallbacks = new Map();
        this.messageReceivedCallbacks = [];
    }

    /**
     * 初始化 ROS 2 通讯
     */
    async init() {
        try {
            this.setStatus('connecting');
            
            // 导入 rclnodejs
            const rclnodejs = await import('rclnodejs');
            this.rclnodejs = rclnodejs.default || rclnodejs;
            
            // 初始化 rclnodejs
            if (!this.rclnodejs.isInitialized || !this.isRclInitialized) {
                await this.rclnodejs.init();
                this.isRclInitialized = true;
            }

            // 创建节点并开始 spin
            this.node = new this.rclnodejs.Node('dq_vizlab_node');
            if (!this.node.spinning && typeof this.node.spin === 'function') {
                try {
                    this.node.spin();
                } catch (spinError) {
                    console.error('启动 ROS 2 节点 spin 失败:', spinError);
                    throw spinError;
                }
            }
            this.isInitialized = true;
            this.setStatus('connected');
            
            console.log('✓ ROS 2 通讯已初始化');
            return true;
        } catch (error) {
            console.error('ROS 2 初始化失败:', error);
            this.setStatus('error');
            return false;
        }
    }

    /**
     * 关闭 ROS 2 通讯
     */
    async shutdown() {
        try {
            // 清理所有订阅
            for (const [topicName, subscription] of this.subscriptions) {
                if (this.node && this.node.destroySubscription) {
                    this.node.destroySubscription(subscription);
                }
            }
            this.subscriptions.clear();

            // 清理所有发布器
            for (const [topicName, publisher] of this.publishers) {
                if (this.node && this.node.destroyPublisher) {
                    this.node.destroyPublisher(publisher);
                }
            }
            this.publishers.clear();

            // 销毁节点
            if (this.node) {
                if (this.node.spinning && typeof this.node.stop === 'function') {
                    try {
                        this.node.stop();
                    } catch (stopError) {
                        console.warn('停止 ROS 2 节点 spin 失败:', stopError);
                    }
                }
                this.node.destroy();
                this.node = null;
            }

            if (this.rclnodejs?.shutdown && this.isRclInitialized) {
                try {
                    await this.rclnodejs.shutdown();
                } catch (shutdownError) {
                    console.warn('关闭 rclnodejs 上下文失败:', shutdownError);
                } finally {
                    this.isRclInitialized = false;
                }
            }

            this.isInitialized = false;
            this.setStatus('disconnected');
            console.log('✓ ROS 2 通讯已关闭');
        } catch (error) {
            console.error('关闭 ROS 2 通讯失败:', error);
        }
    }

    /**
     * 发布消息
     */
    async publish(topicName, messageType, message) {
        if (!this.isInitialized || !this.node) {
            console.warn('ROS 2 未初始化');
            return false;
        }

        try {
            let publisher = this.publishers.get(topicName);
            
            // 如果发布器不存在，创建新的
            if (!publisher) {
                publisher = this.node.createPublisher(messageType, topicName);
                this.publishers.set(topicName, publisher);
            }

            publisher.publish(message);
            return true;
        } catch (error) {
            console.error(`发布消息到 ${topicName} 失败:`, error);
            return false;
        }
    }

    /**
     * 订阅话题
     */
    async subscribe(topicName, messageType, callback) {
        if (!this.isInitialized || !this.node) {
            console.warn('ROS 2 未初始化');
            return false;
        }

        try {
            // 检查是否已订阅
            if (this.subscriptions.has(topicName)) {
                console.warn(`已订阅 ${topicName}，复用现有订阅`);
                return true;
            }

            // 创建订阅
            const subscription = this.node.createSubscription(
                messageType,
                topicName,
                (message) => {
                    const processedMessage = this.normalizeMessageData(message);

                    // 触发用户提供的回调
                    if (typeof callback === 'function') {
                        callback(processedMessage);
                    }
                    
                    // 触发通用消息接收回调（用于广播到前端）
                    this.messageReceivedCallbacks.forEach(cb => cb(topicName, processedMessage));
                    
                    // 触发特定话题的回调
                    if (this.messageCallbacks.has(topicName)) {
                        const callbacks = this.messageCallbacks.get(topicName);
                        callbacks.forEach(cb => cb(processedMessage));
                    }
                }
            );

            this.subscriptions.set(topicName, subscription);
            console.log(`✓ 已订阅 ${topicName}`);
            return true;
        } catch (error) {
            console.error(`订阅 ${topicName} 失败:`, error);
            throw error;
        }
    }

    /**
     * 取消订阅
     */
    unsubscribe(topicName) {
        try {
            const subscription = this.subscriptions.get(topicName);
            if (subscription && this.node) {
                this.node.destroySubscription(subscription);
                this.subscriptions.delete(topicName);
                console.log(`✓ 取消订阅 ${topicName}`);
                return true;
            }
            return false;
        } catch (error) {
            console.error(`取消订阅 ${topicName} 失败:`, error);
            return false;
        }
    }

    /**
     * 获取所有话题列表
     */
    async getTopicNames() {
        if (!this.isInitialized || !this.node) {
            return [];
        }

        try {
            // 尝试获取话题列表
            const graph = this.node.getGraph?.();
            if (graph && graph.getTopicNamesAndTypes) {
                const topics = graph.getTopicNamesAndTypes();
                return topics;
            }
            return [];
        } catch (error) {
            console.error('获取话题列表失败:', error);
            return [];
        }
    }

    /**
     * 设置状态并通知所有监听器
     */
    setStatus(status) {
        this.ros2Status = status;
        this.statusCallbacks.forEach(callback => callback(status));
    }

    /**
     * 注册状态变化回调
     */
    onStatusChange(callback) {
        this.statusCallbacks.push(callback);
    }

    /**
     * 注册消息接收回调（用于后端广播到前端）
     */
    onMessageReceived(callback) {
        this.messageReceivedCallbacks.push(callback);
    }

    /**
     * 注册话题消息回调
     */
    onMessage(topicName, callback) {
        if (!this.messageCallbacks.has(topicName)) {
            this.messageCallbacks.set(topicName, []);
        }
        this.messageCallbacks.get(topicName).push(callback);
    }

    /**
     * 移除消息回调
     */
    offMessage(topicName, callback) {
        if (this.messageCallbacks.has(topicName)) {
            const callbacks = this.messageCallbacks.get(topicName);
            const index = callbacks.indexOf(callback);
            if (index > -1) {
                callbacks.splice(index, 1);
            }
        }
    }

    /**
     * 创建和发送 std_msgs/Float64MultiArray 消息
     */
    async publishJointCommands(topicName, jointAngles) {
        if (!this.isInitialized) return false;

        try {
            const message = this.rclnodejs.createMessageObject('std_msgs/msg/Float64MultiArray');
            message.data = jointAngles;
            return await this.publish(topicName, 'std_msgs/msg/Float64MultiArray', message);
        } catch (error) {
            console.error('发送关节指令失败:', error);
            return false;
        }
    }

    /**
     * 创建和发送 geometry_msgs/Twist 消息（速度控制）
     */
    async publishTwist(topicName, linear, angular) {
        if (!this.isInitialized) return false;

        try {
            const message = this.rclnodejs.createMessageObject('geometry_msgs/msg/Twist');
            message.linear = { x: linear?.x || 0, y: linear?.y || 0, z: linear?.z || 0 };
            message.angular = { x: angular?.x || 0, y: angular?.y || 0, z: angular?.z || 0 };
            return await this.publish(topicName, 'geometry_msgs/msg/Twist', message);
        } catch (error) {
            console.error('发送速度指令失败:', error);
            return false;
        }
    }

    /**
     * 获取当前 ROS2 时间戳
     * @returns {Object} 包含 sec 和 nanosec 的时间对象
     */
    getCurrentTimestamp() {
        try {
            // 尝试使用节点时钟
            if (this.node && typeof this.node.get_clock === 'function') {
                const clock = this.node.get_clock();
                if (clock && typeof clock.now === 'function') {
                    return clock.now();
                }
            }
        } catch (error) {
            console.warn('无法使用节点时钟获取时间戳，使用系统时间:', error);
        }
        
        // 回退方案：使用系统时间创建时间对象
        const now = Date.now();
        return {
            sec: Math.floor(now / 1000),
            nanosec: (now % 1000) * 1000000
        };
    }

    /**
     * 创建和发送 geometry_msgs/PoseStamped 消息
     */
    async publishPose(topicName, position, orientation) {
        if (!this.isInitialized) return false;

        try {
            const message = this.rclnodejs.createMessageObject('geometry_msgs/msg/PoseStamped');
            message.header.stamp = this.getCurrentTimestamp();
            message.header.frame_id = 'world';
            message.pose.position = position;
            message.pose.orientation = orientation;
            return await this.publish(topicName, 'geometry_msgs/msg/PoseStamped', message);
        } catch (error) {
            console.error('发送位姿消息失败:', error);
            return false;
        }
    }

    /**
     * 创建和发送 robot_control_msg/Robotarmservomsg 消息（自定义机械臂控制消息）
     */
    async publishRobotArmServoMsg(topicName, messageData) {
        if (!this.isInitialized) return false;

        try {
            const message = this.rclnodejs.createMessageObject('robot_control_msg/msg/Robotarmservomsg');
            
            // 设置左臂关节位置（7个关节）
            message.ljoint1_position = messageData.ljoint1_position ?? 0;
            message.ljoint2_position = messageData.ljoint2_position ?? 0;
            message.ljoint3_position = messageData.ljoint3_position ?? 0;
            message.ljoint4_position = messageData.ljoint4_position ?? 0;
            message.ljoint5_position = messageData.ljoint5_position ?? 0;
            message.ljoint6_position = messageData.ljoint6_position ?? 0;
            message.ljoint7_position = messageData.ljoint7_position ?? 0;
            
            // 设置右臂关节位置（7个关节）
            message.rjoint1_position = messageData.rjoint1_position ?? 0;
            message.rjoint2_position = messageData.rjoint2_position ?? 0;
            message.rjoint3_position = messageData.rjoint3_position ?? 0;
            message.rjoint4_position = messageData.rjoint4_position ?? 0;
            message.rjoint5_position = messageData.rjoint5_position ?? 0;
            message.rjoint6_position = messageData.rjoint6_position ?? 0;
            message.rjoint7_position = messageData.rjoint7_position ?? 0;
            
            // 设置运行模式、电源状态和运行时间
            message.run_mode = messageData.run_mode ?? 0;
            message.robot_power = messageData.robot_power ?? false;
            message.run_time = messageData.run_time ?? 0;
            
            // 设置时间戳
            message.stamp = this.getCurrentTimestamp();
            
            return await this.publish(topicName, 'robot_control_msg/msg/Robotarmservomsg', message);
        } catch (error) {
            console.error('发送机械臂控制消息失败:', error);
            return false;
        }
    }

    /**
     * 创建和发送 robot_control_msg/Robotservomsg 消息（腰部/下肢控制消息）
     */
    async publishRobotServoMsg(topicName, messageData) {
        if (!this.isInitialized) return false;

        try {
            const message = this.rclnodejs.createMessageObject('robot_control_msg/msg/Robotservomsg');

            message.run_mode = messageData.run_mode ?? 0;
            message.ankle_position = messageData.ankle_position ?? 0;
            message.knee_position = messageData.knee_position ?? 0;
            message.hip_pitch_position = messageData.hip_pitch_position ?? 0;
            message.hip_yaw_position = messageData.hip_yaw_position ?? 0;
            message.x = messageData.x ?? 0;
            message.y = messageData.y ?? 0;
            message.phi = messageData.phi ?? 0;
            message.mode_leg_select = messageData.mode_leg_select ?? 0;
            message.vel = messageData.vel ?? 0.3;

            return await this.publish(topicName, 'robot_control_msg/msg/Robotservomsg', message);
        } catch (error) {
            console.error('发送腰部控制消息失败:', error);
            return false;
        }
    }

    /**
     * 将 ROS 2 消息中的 TypedArray 等结构转换为普通 JS 数据，便于序列化
     */
    normalizeMessageData(data) {
        if (data === null || data === undefined) {
            return data;
        }

        if (Array.isArray(data)) {
            return data.map(item => this.normalizeMessageData(item));
        }

        if (ArrayBuffer.isView(data)) {
            if (typeof data[Symbol.iterator] === 'function') {
                return Array.from(data).map(item => this.normalizeMessageData(item));
            }
            return data;
        }

        if (typeof data === 'object') {
            const normalized = {};
            Object.entries(data).forEach(([key, value]) => {
                normalized[key] = this.normalizeMessageData(value);
            });
            return normalized;
        }

        return data;
    }

    /**
     * 获取 ROS 2 状态
     */
    getStatus() {
        return this.ros2Status;
    }

    /**
     * 获取活跃订阅列表
     */
    getActiveSubscriptions() {
        return Array.from(this.subscriptions.keys());
    }

    /**
     * 获取活跃发布器列表
     */
    getActivePublishers() {
        return Array.from(this.publishers.keys());
    }
}

export default ROS2Communicator;
