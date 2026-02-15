/**
 * ROS2 Manager 模块
 * 
 * 连接管理、日志、状态、事件监听器设置、话题管理。
 */

import { JOINT_POSITION_PUBLISH_INTERVAL, ROS2_MAX_LOG_ENTRIES, ROS2_BATTERY_STATE_TOPIC } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import ROS2Communicator from '../ros2-communicator.js';
import { startJointPositionPublishing, stopJointPositionPublishing, setupRealJointStateControls, updateJointStateSubscribeButton, setRos2JointStatus } from './ros2-joint-state.js';
import { subscribeBatteryState } from './ros2-battery.js';

// ═══════════════════════════════════════════
//  ROS2 Event Listeners Setup
// ═══════════════════════════════════════════

function setupROS2EventListeners() {
    // 按钮状态跟踪变量
    let switchModeBtnProcessing = false;

    // ROS 2 WebSocket 地址设置
    const websocketUrlInput = document.getElementById('ros2-websocket-url');
    const saveUrlBtn = document.getElementById('ros2-save-url-btn');
    
    if (websocketUrlInput && saveUrlBtn) {
        // 页面加载时，显示保存的地址
        const savedUrl = localStorage.getItem('ros2-websocket-url');
        if (savedUrl) {
            websocketUrlInput.value = savedUrl;
        }
        
        // 保存 URL 按钮
        saveUrlBtn.addEventListener('click', () => {
            const url = websocketUrlInput.value.trim();
            
            if (!url) {
                addROS2LogEntry('请输入有效的地址', 'warn');
                return;
            }
            
            if (!url.startsWith('ws://') && !url.startsWith('wss://')) {
                addROS2LogEntry('地址必须以 ws:// 或 wss:// 开头', 'warn');
                return;
            }
            
            // 如果已连接，先断开
            if (appState.ros2Communicator && appState.ros2Communicator.isConnected) {
                appState.ros2Communicator.shutdown();
            }
            
            // 保存新地址
            if (appState.ros2Communicator) {
                appState.ros2Communicator.setWebSocketUrl(url);
            }
            
            localStorage.setItem('ros2-websocket-url', url);
            addROS2LogEntry(`✓ 地址已保存: ${url}，请重新连接`, 'info');
        });
        
        // 回车保存
        websocketUrlInput.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                saveUrlBtn.click();
            }
        });
    }

    // ROS 2 连接按钮
    const connectBtn = document.getElementById('ros2-connect-btn');
    if (connectBtn) {
        connectBtn.addEventListener('click', async () => {
            if (!appState.ros2Communicator) {
                // 先读取 URL 输入框的值
                const urlInput = document.getElementById('ros2-websocket-url');
                const customUrl = urlInput ? urlInput.value.trim() : null;
                
                await initROS2Communication(customUrl);
            } else if (appState.ros2Communicator.getStatus() === 'connected') {
                await appState.ros2Communicator.shutdown();
                updateROS2StatusDisplay('disconnected');
            } else {
                await appState.ros2Communicator.init();
            }
        });
    }

    if (callbacks.updateIKRobotSelection) {
        callbacks.updateIKRobotSelection();
    }
    
    // ROS 2 切换控制模式按钮
    const switchModeBtn = document.getElementById('ros2-switch-mode-btn');
    if (switchModeBtn) {
        switchModeBtn.addEventListener('click', async () => {
            // 防止快速多次点击
            if (switchModeBtnProcessing) {
                return;
            }

            if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
                addROS2LogEntry('ROS 2 未连接', 'error');
                return;
            }

            const controlModeSelect = document.getElementById('ros2-control-mode');
            const selectedMode = controlModeSelect ? controlModeSelect.value : 'manual';

            // 设置处理中状态
            switchModeBtnProcessing = true;
            const originalText = switchModeBtn.textContent;
            switchModeBtn.textContent = '切换中...';
            switchModeBtn.disabled = true;
            switchModeBtn.style.opacity = '0.6';

            try {
                await appState.ros2Communicator.switchMode(selectedMode);
                addROS2LogEntry(`切换控制模式: ${selectedMode}`, 'info');

                // 更新状态显示
                const modeStatusEl = document.getElementById('ros2-mode-status');
                if (modeStatusEl) {
                    const modeText = selectedMode === 'manual' ? '手动模式 - 可直接控制机器人关节' :
                                   selectedMode === 'teleop' ? '遥操作模式 - 通过输入设备控制' : '未知模式';
                    modeStatusEl.textContent = `当前模式：${modeText}`;
                }
            } catch (error) {
                addROS2LogEntry(`切换模式失败: ${error.message}`, 'error');
            } finally {
                // 重置按钮状态
                switchModeBtnProcessing = false;
                switchModeBtn.textContent = originalText;
                switchModeBtn.disabled = false;
                switchModeBtn.style.opacity = '1';
            }
        });
    }

    // ROS 2 刷新话题按钮
    const refreshTopicsBtn = document.getElementById('ros2-refresh-topics-btn');
    if (refreshTopicsBtn) {
        refreshTopicsBtn.addEventListener('click', async () => {
            if (appState.ros2Communicator && appState.ros2Communicator.isInitialized) {
                updateROS2ActiveTopics();
                addROS2LogEntry('刷新话题列表', 'info');
            }
        });
    }
    
    // ROS 2 订阅按钮
    const subscribeBtn = document.getElementById('ros2-subscribe-btn');
    if (subscribeBtn) {
        subscribeBtn.addEventListener('click', async () => {
            if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
                addROS2LogEntry('ROS 2 未连接', 'error');
                return;
            }
            
            const topicName = document.getElementById('ros2-topic-name').value.trim();
            const messageType = document.getElementById('ros2-message-type').value;
            
            if (!topicName) {
                addROS2LogEntry('请输入话题名称', 'warn');
                return;
            }
            
            try {
                const result = await appState.ros2Communicator.subscribe(topicName, messageType, (message) => {
                    console.log(`收到消息从 ${topicName}:`, message);
                    addROS2LogEntry(`收到: ${topicName}`, 'info');
                });
                
                if (result) {
                    updateROS2ActiveTopics();
                    addROS2LogEntry(`成功订阅: ${topicName}`, 'info');
                } else {
                    addROS2LogEntry(`订阅失败: ${topicName}`, 'error');
                }
            } catch (error) {
                addROS2LogEntry(`订阅异常: ${error.message}`, 'error');
            }
        });
    }
    
    // ROS 2 发布按钮
    const publishBtn = document.getElementById('ros2-publish-btn');
    if (publishBtn) {
        publishBtn.addEventListener('click', async () => {
            if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
                addROS2LogEntry('ROS 2 未连接', 'error');
                return;
            }
            
            const topicName = document.getElementById('ros2-topic-name').value.trim();
            const messageType = document.getElementById('ros2-message-type').value;
            
            if (!topicName) {
                addROS2LogEntry('请输入话题名称', 'warn');
                return;
            }
            
            try {
                // 这里创建一个空消息作为示例
                const message = appState.ros2Communicator.rclnodejs.createMessageObject(messageType);
                const result = await appState.ros2Communicator.publish(topicName, messageType, message);
                
                if (result) {
                    updateROS2ActiveTopics();
                    addROS2LogEntry(`成功发布: ${topicName}`, 'info');
                } else {
                    addROS2LogEntry(`发布失败: ${topicName}`, 'error');
                }
            } catch (error) {
                addROS2LogEntry(`发布异常: ${error.message}`, 'error');
            }
        });
    }
    
    // ROS 2 发送 Twist 消息按钮
    const sendTwistBtn = document.getElementById('ros2-send-twist-btn');
    if (sendTwistBtn) {
        sendTwistBtn.addEventListener('click', async () => {
            if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
                addROS2LogEntry('ROS 2 未连接', 'error');
                return;
            }
            
            const topicName = document.getElementById('ros2-topic-name').value.trim() || '/cmd_vel';
            
            try {
                const linearX = parseFloat(document.getElementById('ros2-twist-linear-x').value) || 0;
                const linearY = parseFloat(document.getElementById('ros2-twist-linear-y').value) || 0;
                const linearZ = parseFloat(document.getElementById('ros2-twist-linear-z').value) || 0;
                const angularZ = parseFloat(document.getElementById('ros2-twist-angular-z').value) || 0;
                
                const result = await appState.ros2Communicator.publishTwist(
                    topicName,
                    { x: linearX, y: linearY, z: linearZ },
                    { x: 0, y: 0, z: angularZ }
                );
                
                if (result) {
                    addROS2LogEntry(`Twist 消息已发送: ${topicName} (vx=${linearX}, vy=${linearY}, wz=${angularZ})`, 'info');
                } else {
                    addROS2LogEntry(`Twist 消息发送失败`, 'error');
                }
            } catch (error) {
                addROS2LogEntry(`发送 Twist 异常: ${error.message}`, 'error');
            }
        });
    }

    setupRealJointStateControls();

    // 关节位置发布控制（开关模式）
    const jointPositionToggle = document.getElementById('ros2-joint-position-toggle');
    if (jointPositionToggle) {
        jointPositionToggle.addEventListener('change', (e) => {
            const isEnabled = e.target.checked;
            
            if (isEnabled) {
                // 开启持续发布
                startJointPositionPublishing();
            } else {
                // 停止发布
                stopJointPositionPublishing();
            }
        });
    }
}

// ═══════════════════════════════════════════
//  ROS2 Log & Topics
// ═══════════════════════════════════════════

function addROS2LogEntry(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const logEntry = `[${timestamp}] ${message}`;
    appState.ros2MessageLog.push({ message: logEntry, type });
    
    if (appState.ros2MessageLog.length > ROS2_MAX_LOG_ENTRIES) {
        appState.ros2MessageLog.shift();
    }
    
    updateROS2LogDisplay();
}

/**
 * 更新 ROS 2 消息日志显示
 */
function updateROS2LogDisplay() {
    const logContainer = document.getElementById('ros2-message-log');
    if (!logContainer) return;
    
    if (appState.ros2MessageLog.length === 0) {
        logContainer.innerHTML = '<div style="color: #94a3b8; font-size: 0.85rem;">暂无消息</div>';
        return;
    }
    
    logContainer.innerHTML = appState.ros2MessageLog.map(entry => 
        `<div class="ros2-message-log-entry ${entry.type}">${entry.message}</div>`
    ).join('');
    
    // 滚动到底部
    logContainer.scrollTop = logContainer.scrollHeight;
}

/**
 * 更新 ROS 2 活跃话题列表
 */
function updateROS2ActiveTopics() {
    const topicsContainer = document.getElementById('ros2-active-topics');
    if (!topicsContainer || !appState.ros2Communicator) return;
    
    const subscriptions = appState.ros2Communicator.getActiveSubscriptions();
    const publishers = appState.ros2Communicator.getActivePublishers();
    const allTopics = [...new Set([...subscriptions, ...publishers])];
    
    if (allTopics.length === 0) {
        topicsContainer.innerHTML = '<div style="color: #94a3b8; font-size: 0.85rem;">暂无活跃话题</div>';
        return;
    }
    
    topicsContainer.innerHTML = allTopics.map(topic => {
        const isSub = subscriptions.includes(topic);
        const isPub = publishers.includes(topic);
        const type = isSub && isPub ? '收/发' : isSub ? '订阅' : '发布';
        
        return `
            <div class="ros2-topic-item active">
                <div>
                    <span class="ros2-topic-item-name">${topic}</span>
                    <span class="ros2-topic-item-type">${type}</span>
                </div>
                <button class="ros2-topic-remove-btn" onclick="removeROS2Topic('${topic}')">✕</button>
            </div>
        `;
    }).join('');
}

/**
 * 移除 ROS 2 话题
 */
function removeROS2Topic(topicName) {
    if (appState.ros2Communicator) {
        appState.ros2Communicator.unsubscribe(topicName);
        updateROS2ActiveTopics();
        addROS2LogEntry(`取消订阅: ${topicName}`, 'info');
    }
}

// ═══════════════════════════════════════════
//  ROS2 Communication Init & Status
// ═══════════════════════════════════════════

/**
 * 初始化 ROS 2 通讯
 */
async function initROS2Communication(customUrl = null) {
    try {
        // 如果提供了自定义 URL，使用它；否则让构造函数从本地存储读取
        appState.ros2Communicator = new ROS2Communicator(customUrl);
        
        // 注册状态变化回调
        appState.ros2Communicator.onStatusChange((status) => {
            updateROS2StatusDisplay(status);
            const statusText = {
                'disconnected': '未连接',
                'connecting': '连接中...',
                'connected': '已连接',
                'error': '错误'
            }[status] || status;
            addROS2LogEntry(`状态: ${statusText}`, 'info');
            if (status === 'disconnected' && appState.ros2JointStateSubscribed) {
                appState.ros2JointStateSubscribed = false;
                updateJointStateSubscribeButton();
                setRos2JointStatus('连接断开，已停止订阅', 'error');
            }
            // 如果断开连接，停止电池状态订阅
            if (status === 'disconnected' && appState.ros2BatteryStateSubscribed) {
                appState.ros2BatteryStateSubscribed = false;
            }
            // 如果断开连接，停止关节位置发布
            if (status === 'disconnected' || status === 'error') {
                if (appState.jointPositionPublishTimer !== null) {
                    stopJointPositionPublishing();
                    const toggle = document.getElementById('ros2-joint-position-toggle');
                    if (toggle) toggle.checked = false;
                }
            }
        });

        // 注册模式变化回调
        appState.ros2Communicator.onModeChange((mode) => {
            callbacks.updateROS2ControlModeDisplay?.(mode);
            addROS2LogEntry(`当前控制模式: ${mode}`, 'info');
        });
        
        // 显示连接的 URL
        addROS2LogEntry(`连接地址: ${appState.ros2Communicator.getWebSocketUrl()}`, 'info');
        
        // 尝试初始化
        const result = await appState.ros2Communicator.init();
        if (result) {
            addROS2LogEntry('ROS 2 初始化成功', 'info');
            // 查询当前控制模式
            try {
                await appState.ros2Communicator.getMode();
            } catch (error) {
                console.warn('查询当前模式失败:', error);
            }
            // 自动订阅电池状态话题
            try {
                await subscribeBatteryState();
            } catch (error) {
                console.warn('订阅电池状态失败:', error);
            }
        } else {
            addROS2LogEntry('ROS 2 初始化失败', 'error');
        }
    } catch (error) {
        console.error('ROS 2 初始化异常:', error);
        addROS2LogEntry(`初始化异常: ${error.message}`, 'error');
    }
}

/**
 * 更新 ROS 2 状态显示
 */
function updateROS2StatusDisplay(status) {
    const indicator = document.getElementById('ros2-status-indicator');
    const text = document.getElementById('ros2-status-text');
    
    if (!indicator || !text) return;
    
    // 移除旧状态类
    ['disconnected', 'connecting', 'connected', 'error'].forEach(s => {
        indicator.classList.remove(s);
    });
    
    indicator.classList.add(status);
    
    const statusMap = {
        'disconnected': '未连接',
        'connecting': '连接中...',
        'connected': '已连接',
        'error': '错误'
    };
    text.textContent = statusMap[status] || status;
}

// ═══════════════════════════════════════════
//  Callback Registration
// ═══════════════════════════════════════════

/**
 * 注册 ROS2 Manager 回调到全局 callbacks 对象
 */
function registerROS2ManagerCallbacks() {
    callbacks.addROS2LogEntry = addROS2LogEntry;
    callbacks.initROS2Communication = initROS2Communication;
    callbacks.updateROS2StatusDisplay = updateROS2StatusDisplay;
    callbacks.setupROS2EventListeners = setupROS2EventListeners;
}

// ═══════════════════════════════════════════
//  Exports
// ═══════════════════════════════════════════

export {
    setupROS2EventListeners,
    addROS2LogEntry,
    updateROS2LogDisplay,
    updateROS2ActiveTopics,
    removeROS2Topic,
    initROS2Communication,
    updateROS2StatusDisplay,
    registerROS2ManagerCallbacks,
};
