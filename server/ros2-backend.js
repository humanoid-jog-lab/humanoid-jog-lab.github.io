/**
 * ROS 2 后端服务
 * 运行在 Node.js 中，处理与 ROS 2 的通讯
 * 通过 WebSocket 向前端浏览器提供 ROS 2 功能
 * 
 * 支持两种运行方式：
 * 1. 独立运行：node server/ros2-backend.js
 * 2. Electron 中导入：import { createROS2Server } from './ros2-backend.js'
 */

import express from 'express';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import ROS2Communicator from '../src/ros2-communicator-backend.js';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);

/**
 * 创建 ROS 2 后端服务器
 * 可以被 Electron 或独立脚本导入使用
 */
export function createROS2Server(port = 8081) {
    const app = express();
    const httpServer = createServer(app);
    const wss = new WebSocketServer({ server: httpServer });
    
    let ros2Communicator = null;
    const clients = new Set();
    const clientSubscriptions = new Map(); // ws -> Set<topicName>
    const topicSubscribers = new Map(); // topicName -> count

    // 中间件
    app.use(express.json());
    app.use((req, res, next) => {
        res.header('Access-Control-Allow-Origin', '*');
        res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
        next();
    });

    // 健康检查端点
    app.get('/health', (req, res) => {
        res.json({
            status: 'ok',
            ros2Status: ros2Communicator?.getStatus() || 'not-initialized'
        });
    });

    // 获取状态端点
    app.get('/api/status', (req, res) => {
        if (!ros2Communicator) {
            return res.json({ status: 'not-initialized' });
        }
        
        res.json({
            status: ros2Communicator.getStatus(),
            subscriptions: ros2Communicator.getActiveSubscriptions(),
            publishers: ros2Communicator.getActivePublishers()
        });
    });

    /**
     * WebSocket 连接处理
     */
    wss.on('connection', (ws) => {
        console.log('✓ 新客户端连接');
        clients.add(ws);
        clientSubscriptions.set(ws, new Set());

        ws.on('message', async (data) => {
            try {
                const message = JSON.parse(data);
                await handleClientMessage(ws, message);
            } catch (error) {
                console.error('处理消息失败:', error);
                ws.send(JSON.stringify({
                    type: 'error',
                    error: error.message
                }));
            }
        });

        ws.on('close', () => {
            console.log('✓ 客户端断开连接');
            cleanupClientSubscriptions(ws);
            clients.delete(ws);
        });

        ws.on('error', (error) => {
            console.error('WebSocket 错误:', error);
            cleanupClientSubscriptions(ws);
            clients.delete(ws);
        });
    });

    /**
     * 处理来自客户端的消息
     */
    async function handleClientMessage(ws, message) {
        const { type, data, id } = message;

        try {
            switch (type) {
                case 'init':
                    await handleInit(ws);
                    break;

                case 'shutdown':
                    await handleShutdown(ws);
                    break;

                case 'subscribe':
                    await handleSubscribe(ws, data);
                    break;

                case 'unsubscribe':
                    await handleUnsubscribe(ws, data);
                    break;

                case 'publish':
                    await handlePublish(ws, data);
                    break;

                case 'publishTwist':
                    await handlePublishTwist(ws, data);
                    break;

                case 'publishJointCommands':
                    await handlePublishJointCommands(ws, data);
                    break;

                case 'publishPose':
                    await handlePublishPose(ws, data);
                    break;

                case 'publishRobotArmServoMsg':
                    await handlePublishRobotArmServoMsg(ws, data);
                    break;

                case 'publishRobotServoMsg':
                    await handlePublishRobotServoMsg(ws, data);
                    break;

                case 'getStatus':
                    ws.send(JSON.stringify({
                        type: 'status',
                        status: ros2Communicator?.getStatus() || 'not-initialized'
                    }));
                    break;

                case 'getTopics':
                    await handleGetTopics(ws);
                    break;

                default:
                    console.warn('未知的消息类型:', type);
            }
        } catch (error) {
            console.error(`处理 ${type} 消息失败:`, error);
            ws.send(JSON.stringify({
                type: 'error',
                message: error.message,
                originalType: type
            }));
        }
    }

    /**
     * 初始化 ROS 2
     */
    async function handleInit(ws) {
        if (ros2Communicator?.isInitialized) {
            ws.send(JSON.stringify({
                type: 'status',
                status: 'connected',
                message: 'ROS 2 已连接'
            }));
            return;
        }

        try {
            ros2Communicator = new ROS2Communicator();

            // 注册状态变化回调
            ros2Communicator.onStatusChange((status) => {
                broadcastToAll({
                    type: 'status',
                    status: status,
                    message: `ROS 2 状态: ${status}`
                });
            });

            // 注册消息回调
            ros2Communicator.onMessageReceived((topicName, message) => {
                broadcastToAll({
                    type: 'message',
                    topic: topicName,
                    data: message
                });
            });

            // 初始化
            await ros2Communicator.init();

            ws.send(JSON.stringify({
                type: 'status',
                status: 'connected',
                message: 'ROS 2 已初始化并连接'
            }));

            console.log('✓ ROS 2 已初始化');
        } catch (error) {
            console.error('ROS 2 初始化失败:', error);
            ws.send(JSON.stringify({
                type: 'error',
                message: error.message
            }));
        }
    }

    /**
     * 关闭 ROS 2
     */
    async function handleShutdown(ws) {
        if (ros2Communicator) {
            await ros2Communicator.shutdown();
            ros2Communicator = null;
            topicSubscribers.clear();
            clientSubscriptions.forEach(set => set.clear());
        }

        ws.send(JSON.stringify({
            type: 'status',
            status: 'disconnected',
            message: 'ROS 2 已断开连接'
        }));
    }

    /**
     * 订阅话题
     */
    async function handleSubscribe(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, messageType } = data;

        if (!topicName || !messageType) {
            throw new Error('话题名称和消息类型不能为空');
        }

        let clientTopics = clientSubscriptions.get(ws);
        if (!clientTopics) {
            clientTopics = new Set();
            clientSubscriptions.set(ws, clientTopics);
        }

        if (clientTopics.has(topicName)) {
            ws.send(JSON.stringify({
                type: 'subscribed',
                topic: topicName,
                message: `已订阅: ${topicName}（重复请求）`
            }));
            return;
        }

        const currentCount = topicSubscribers.get(topicName) || 0;

        if (currentCount === 0) {
            await ros2Communicator.subscribe(topicName, messageType);
        } else {
            console.log(`复用现有订阅 ${topicName}，当前订阅客户端数: ${currentCount}`);
        }

        clientTopics.add(topicName);
        topicSubscribers.set(topicName, currentCount + 1);

        ws.send(JSON.stringify({
            type: 'subscribed',
            topic: topicName,
            message: currentCount === 0
                ? `已订阅: ${topicName}`
                : `已订阅: ${topicName}（复用现有订阅）`
        }));
    }

    /**
     * 取消订阅
     */
    async function handleUnsubscribe(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName } = data;

        if (!topicName) {
            throw new Error('话题名称不能为空');
        }

        const clientTopics = clientSubscriptions.get(ws);
        if (!clientTopics || !clientTopics.has(topicName)) {
            ws.send(JSON.stringify({
                type: 'warn',
                topic: topicName,
                message: `未找到订阅: ${topicName}`
            }));
            return;
        }

        clientTopics.delete(topicName);
        const remaining = decrementTopicSubscriber(topicName);

        ws.send(JSON.stringify({
            type: 'unsubscribed',
            topic: topicName,
            message: remaining > 0
                ? `已取消订阅: ${topicName}（剩余 ${remaining} 个客户端）`
                : `已取消订阅: ${topicName}`
        }));
    }

    /**
     * 发布消息
     */
    async function handlePublish(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, messageType, message } = data;

        if (!topicName || !messageType) {
            throw new Error('话题名称和消息类型不能为空');
        }

        const result = await ros2Communicator.publish(topicName, messageType, message);

        if (result) {
            ws.send(JSON.stringify({
                type: 'published',
                topic: topicName,
                message: `消息已发布到: ${topicName}`
            }));
        } else {
            throw new Error(`发布失败: ${topicName}`);
        }
    }

    /**
     * 发布 Twist 消息
     */
    async function handlePublishTwist(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, linear, angular } = data;

        if (!topicName) {
            throw new Error('话题名称不能为空');
        }

        const result = await ros2Communicator.publishTwist(
            topicName,
            linear || { x: 0, y: 0, z: 0 },
            angular || { x: 0, y: 0, z: 0 }
        );

        if (result) {
            ws.send(JSON.stringify({
                type: 'published',
                topic: topicName,
                message: `Twist 消息已发送到: ${topicName}`
            }));
        } else {
            throw new Error('发布 Twist 消息失败');
        }
    }

    /**
     * 发布关节指令
     */
    async function handlePublishJointCommands(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, jointAngles } = data;

        if (!topicName || !Array.isArray(jointAngles)) {
            throw new Error('话题名称和关节角度不能为空');
        }

        const result = await ros2Communicator.publishJointCommands(topicName, jointAngles);

        if (result) {
            ws.send(JSON.stringify({
                type: 'published',
                topic: topicName,
                message: `关节指令已发送到: ${topicName}`
            }));
        } else {
            throw new Error('发布关节指令失败');
        }
    }

    /**
     * 发布位姿消息
     */
    async function handlePublishPose(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, position, orientation } = data;

        if (!topicName || !position || !orientation) {
            throw new Error('话题名称、位置和方向不能为空');
        }

        const result = await ros2Communicator.publishPose(topicName, position, orientation);

        if (result) {
            ws.send(JSON.stringify({
                type: 'published',
                topic: topicName,
                message: `位姿消息已发送到: ${topicName}`
            }));
        } else {
            throw new Error('发布位姿消息失败');
        }
    }

    /**
     * 发布机械臂控制消息（自定义消息格式）
     */
    async function handlePublishRobotArmServoMsg(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, messageData } = data;

        if (!topicName || !messageData) {
            throw new Error('话题名称和消息数据不能为空');
        }

        const result = await ros2Communicator.publishRobotArmServoMsg(topicName, messageData);

        if (result) {
            ws.send(JSON.stringify({
                type: 'published',
                topic: topicName,
                message: `机械臂控制消息已发送到: ${topicName}`
            }));
        } else {
            throw new Error('发布机械臂控制消息失败');
        }
    }

    async function handlePublishRobotServoMsg(ws, data) {
        if (!ros2Communicator?.isInitialized) {
            throw new Error('ROS 2 未初始化');
        }

        const { topicName, messageData } = data;

        if (!topicName || !messageData) {
            throw new Error('话题名称和消息数据不能为空');
        }

        const result = await ros2Communicator.publishRobotServoMsg(topicName, messageData);

        if (result) {
            ws.send(JSON.stringify({
                type: 'published',
                topic: topicName,
                message: `腰部控制消息已发送到: ${topicName}`
            }));
        } else {
            throw new Error('发布腰部控制消息失败');
        }
    }

    /**
     * 获取话题列表
     */
    async function handleGetTopics(ws) {
        if (!ros2Communicator?.isInitialized) {
            ws.send(JSON.stringify({
                type: 'topics',
                topics: []
            }));
            return;
        }

        try {
            const topics = await ros2Communicator.getTopicNames();
            ws.send(JSON.stringify({
                type: 'topics',
                topics: topics || []
            }));
        } catch (error) {
            ws.send(JSON.stringify({
                type: 'topics',
                topics: [],
                error: error.message
            }));
        }
    }

    /**
     * 广播消息到所有连接的客户端
     */
    function broadcastToAll(message) {
        clients.forEach(client => {
            if (client.readyState === 1) { // WebSocket.OPEN
                client.send(JSON.stringify(message));
            }
        });
    }

    function decrementTopicSubscriber(topicName) {
        const current = topicSubscribers.get(topicName);
        if (!current) {
            return 0;
        }

        if (current <= 1) {
            topicSubscribers.delete(topicName);
            if (ros2Communicator?.isInitialized) {
                ros2Communicator.unsubscribe(topicName);
            }
            return 0;
        }

        const next = current - 1;
        topicSubscribers.set(topicName, next);
        return next;
    }

    function cleanupClientSubscriptions(ws) {
        const topics = clientSubscriptions.get(ws);
        if (!topics || topics.size === 0) {
            clientSubscriptions.delete(ws);
            return;
        }

        topics.forEach(topic => {
            decrementTopicSubscriber(topic);
        });
        clientSubscriptions.delete(ws);
    }

    // 返回服务器和应用，供 Electron 使用
    return { httpServer, app, wss, clients };
}

/**
 * 独立运行此脚本
 */
if (import.meta.url === `file://${process.argv[1]}`) {
    const PORT = process.env.PORT || 8081;
    const { httpServer } = createROS2Server(PORT);
    
    httpServer.listen(PORT, () => {
        console.log(`\n🚀 ROS 2 WebSocket 后端服务已启动`);
        console.log(`📡 WebSocket 服务地址: ws://localhost:${PORT}`);
        console.log(`✓ HTTP 健康检查: http://localhost:${PORT}/health\n`);
    });

    /**
     * 优雅关闭
     */
    process.on('SIGINT', async () => {
        console.log('\n\n正在关闭服务...');
        
        httpServer.close(() => {
            console.log('✓ 服务已关闭');
            process.exit(0);
        });

        // 5 秒后强制退出
        setTimeout(() => {
            console.error('强制退出');
            process.exit(1);
        }, 5000);
    });
}

export default { createROS2Server };
