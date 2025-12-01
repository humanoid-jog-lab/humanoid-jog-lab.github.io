/**
 * ROS 2 通讯模块 - 浏览器客户端版本
 * 通过 WebSocket 与后端 Node.js 服务通讯
 */

export class ROS2Communicator {
    constructor(wsUrl = null) {
        // 优先使用传入的 URL，其次使用本地存储的 URL，最后使用默认 URL
        if (wsUrl) {
            this.wsUrl = wsUrl;
        } else {
            // 尝试从本地存储获取用户配置的 WebSocket 地址
            const savedUrl = localStorage.getItem('ros2-websocket-url');
            this.wsUrl = savedUrl || 'ws://localhost:8081';
        }
        
        this.ws = null;
        this.isConnected = false;
        this.isInitialized = false;
        this.ros2Status = 'disconnected'; // disconnected, connecting, connected, error
        this.statusCallbacks = [];
        this.messageCallbacks = new Map();
        this.subscriptions = new Set();
        this.publishers = new Set();
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 3000;
    }

    /**
     * 设置 WebSocket URL
     */
    setWebSocketUrl(url) {
        this.wsUrl = url;
        // 保存到本地存储
        localStorage.setItem('ros2-websocket-url', url);
        console.log('✓ WebSocket URL 已保存:', url);
    }

    /**
     * 获取当前 WebSocket URL
     */
    getWebSocketUrl() {
        return this.wsUrl;
    }

    /**
     * 连接到后端 WebSocket 服务
     */
    async connect() {
        return new Promise((resolve, reject) => {
            try {
                this.setStatus('connecting');
                
                this.ws = new WebSocket(this.wsUrl);
                
                this.ws.onopen = () => {
                    console.log('✓ WebSocket 已连接');
                    this.isConnected = true;
                    this.reconnectAttempts = 0;
                    resolve(true);
                };
                
                this.ws.onmessage = (event) => {
                    this.handleMessage(JSON.parse(event.data));
                };
                
                this.ws.onerror = (error) => {
                    console.error('WebSocket 错误:', error);
                    this.setStatus('error');
                    reject(new Error('WebSocket 连接错误'));
                };
                
                this.ws.onclose = () => {
                    console.log('WebSocket 已断开连接');
                    this.isConnected = false;
                    this.isInitialized = false;
                    this.setStatus('disconnected');
                    this.attemptReconnect();
                };
                
                // 连接超时
                setTimeout(() => {
                    if (!this.isConnected) {
                        this.setStatus('error');
                        if (this.ws) this.ws.close();
                        reject(new Error('WebSocket 连接超时'));
                    }
                }, 5000);
                
            } catch (error) {
                this.setStatus('error');
                reject(error);
            }
        });
    }

    /**
     * 尝试自动重连
     */
    attemptReconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            console.log(`准备重连... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
            
            setTimeout(() => {
                this.connect().catch(error => {
                    console.error('重连失败:', error);
                });
            }, this.reconnectDelay);
        }
    }

    /**
     * 初始化 ROS 2（通过后端）
     */
    async init() {
        try {
            if (!this.isConnected) {
                await this.connect();
            }

            // 发送初始化命令
            this.send({
                type: 'init'
            });

            this.isInitialized = true;
            this.setStatus('connected');
            return true;
        } catch (error) {
            console.error('ROS 2 初始化失败:', error);
            this.setStatus('error');
            return false;
        }
    }

    /**
     * 关闭 ROS 2 连接
     */
    async shutdown() {
        try {
            this.send({ type: 'shutdown' });
            this.isInitialized = false;
            this.subscriptions.clear();
            this.publishers.clear();
        } catch (error) {
            console.error('关闭 ROS 2 失败:', error);
        }
    }

    /**
     * 发送消息到后端
     */
    send(message) {
        if (!this.isConnected || !this.ws) {
            throw new Error('WebSocket 未连接');
        }
        this.ws.send(JSON.stringify(message));
    }

    /**
     * 处理来自后端的消息
     */
    handleMessage(message) {
        const { type, topic, data, status, error, message: msg, topics } = message;

        switch (type) {
            case 'status':
                this.setStatus(status);
                console.log(msg);
                break;

            case 'message':
                // 广播订阅消息
                if (this.messageCallbacks.has(topic)) {
                    const callbacks = this.messageCallbacks.get(topic);
                    callbacks.forEach(cb => cb(data));
                }
                break;

            case 'subscribed':
                this.subscriptions.add(topic);
                console.log(msg);
                break;

            case 'unsubscribed':
                this.subscriptions.delete(topic);
                console.log(msg);
                break;

            case 'published':
            case 'warn':
            case 'info':
                console.log(msg);
                break;

            case 'error':
                console.error('后端错误:', error || msg || '未知错误');
                break;

            case 'topics':
                // 处理话题列表响应
                console.log('话题列表:', data ?? topics);
                break;

            default:
                console.warn('未知的消息类型:', type);
        }
    }

    /**
     * 发布消息
     */
    async publish(topicName, messageType, message) {
        if (!this.isInitialized) return false;

        try {
            this.send({
                type: 'publish',
                data: { topicName, messageType, message }
            });
            return true;
        } catch (error) {
            console.error(`发布消息到 ${topicName} 失败:`, error);
            return false;
        }
    }

    /**
     * 发布 Twist 消息
     */
    async publishTwist(topicName, linear, angular) {
        if (!this.isInitialized) return false;

        try {
            this.send({
                type: 'publishTwist',
                data: {
                    topicName,
                    linear: linear || { x: 0, y: 0, z: 0 },
                    angular: angular || { x: 0, y: 0, z: 0 }
                }
            });
            return true;
        } catch (error) {
            console.error('发送速度指令失败:', error);
            return false;
        }
    }

    /**
     * 订阅话题
     */
    async subscribe(topicName, messageType, callback) {
        if (!this.isInitialized) {
            console.warn('ROS 2 未初始化');
            return false;
        }

        try {
            // 注册消息回调
            if (!this.messageCallbacks.has(topicName)) {
                this.messageCallbacks.set(topicName, []);
            }
            this.messageCallbacks.get(topicName).push(callback);

            // 发送订阅请求
            this.send({
                type: 'subscribe',
                data: { topicName, messageType }
            });

            return true;
        } catch (error) {
            console.error(`订阅 ${topicName} 失败:`, error);
            return false;
        }
    }

    /**
     * 取消订阅
     */
    async unsubscribe(topicName) {
        try {
            this.send({
                type: 'unsubscribe',
                data: { topicName }
            });
            this.messageCallbacks.delete(topicName);
            return true;
        } catch (error) {
            console.error(`取消订阅 ${topicName} 失败:`, error);
            return false;
        }
    }

    /**
     * 发布关节指令
     */
    async publishJointCommands(topicName, jointAngles) {
        if (!this.isInitialized) return false;

        try {
            this.send({
                type: 'publishJointCommands',
                data: { topicName, jointAngles }
            });
            return true;
        } catch (error) {
            console.error('发送关节指令失败:', error);
            return false;
        }
    }

    /**
     * 发布位姿消息
     */
    async publishPose(topicName, position, orientation) {
        if (!this.isInitialized) return false;

        try {
            this.send({
                type: 'publishPose',
                data: { topicName, position, orientation }
            });
            return true;
        } catch (error) {
            console.error('发送位姿消息失败:', error);
            return false;
        }
    }

    /**
     * 发布机械臂控制消息（自定义消息格式）
     */
    async publishRobotArmServoMsg(topicName, messageData) {
        if (!this.isInitialized) return false;

        try {
            this.send({
                type: 'publishRobotArmServoMsg',
                data: { topicName, messageData }
            });
            return true;
        } catch (error) {
            console.error('发送机械臂控制消息失败:', error);
            return false;
        }
    }

    /**
     * 获取话题列表
     */
    async getTopicNames() {
        if (!this.isInitialized) {
            return [];
        }

        try {
            this.send({
                type: 'getTopics'
            });
            return [];
        } catch (error) {
            console.error('获取话题列表失败:', error);
            return [];
        }
    }

    /**
     * 设置状态
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
     * 注册消息回调
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
     * 获取活跃订阅
     */
    getActiveSubscriptions() {
        return Array.from(this.subscriptions);
    }

    /**
     * 获取活跃发布器
     */
    getActivePublishers() {
        return Array.from(this.publishers);
    }

    /**
     * 获取状态
     */
    getStatus() {
        return this.ros2Status;
    }
}

export default ROS2Communicator;
