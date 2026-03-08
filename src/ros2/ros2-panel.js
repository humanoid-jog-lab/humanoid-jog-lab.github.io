/**
 * ROS2 面板模块
 * 
 * 包含所有 ROS2 相关功能：连接管理、话题订阅/发布、关节状态同步、
 * 关节位置发布、电池状态监控、日志显示等。
 */

import { JOINT_POSITION_PUBLISH_INTERVAL, ROS2_MAX_LOG_ENTRIES, ROS2_BATTERY_STATE_TOPIC } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import ROS2Communicator from '../ros2-communicator.js';

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
//  Joint Position Publishing
// ═══════════════════════════════════════════

/**
 * 更新关节位置发布状态显示
 */
function updateJointPositionStatus(message, type = 'info') {
    const statusEl = document.getElementById('ros2-joint-position-status');
    if (!statusEl) return;

    statusEl.textContent = message;
    statusEl.style.color = {
        'success': '#10b981',
        'error': '#ef4444',
        'warn': '#f59e0b',
        'info': '#94a3b8'
    }[type] || '#94a3b8';
}

/**
 * 发布关节位置消息（内部函数）
 */
async function publishJointPositionMessage() {
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateJointPositionStatus('ROS 2 未连接', 'error');
        return false;
    }

    const topicName = document.getElementById('ros2-joint-position-topic')?.value.trim();
    if (!topicName) {
        updateJointPositionStatus('话题名称未设置', 'error');
        return false;
    }

    try {
        // 固定参数：运行模式=1，机器人电源=true，运行时间=0
        const messageData = {
            run_mode: 1,
            robot_power: true,
            run_time: 0
        };

        // 根据单臂/双臂模式填充关节角度
        if (appState.dualArmMode) {
            // 双臂模式：根据URDF定义，robot1(panda_1)是右臂，robot2(panda_2)是左臂
            const angles1 = appState.jointAngles1 || [];  // robot1 (panda_1) = 右臂
            const angles2 = appState.jointAngles2 || [];  // robot2 (panda_2) = 左臂

            // 填充左臂关节（robot2, panda_2）
            messageData.ljoint1_position = angles2[0] ?? 0;
            messageData.ljoint2_position = angles2[1] ?? 0;
            messageData.ljoint3_position = angles2[2] ?? 0;
            messageData.ljoint4_position = angles2[3] ?? 0;
            messageData.ljoint5_position = angles2[4] ?? 0;
            messageData.ljoint6_position = angles2[5] ?? 0;
            messageData.ljoint7_position = angles2[6] ?? 0;

            // 填充右臂关节（robot1, panda_1）
            messageData.rjoint1_position = angles1[0] ?? 0;
            messageData.rjoint2_position = angles1[1] ?? 0;
            messageData.rjoint3_position = angles1[2] ?? 0;
            messageData.rjoint4_position = angles1[3] ?? 0;
            messageData.rjoint5_position = angles1[4] ?? 0;
            messageData.rjoint6_position = angles1[5] ?? 0;
            messageData.rjoint7_position = angles1[6] ?? 0;
        } else {
            // 单臂模式：作为右臂，左臂填充0
            const angles = appState.jointAngles || [];

            // 左臂全部为0
            messageData.ljoint1_position = 0;
            messageData.ljoint2_position = 0;
            messageData.ljoint3_position = 0;
            messageData.ljoint4_position = 0;
            messageData.ljoint5_position = 0;
            messageData.ljoint6_position = 0;
            messageData.ljoint7_position = 0;

            // 填充右臂关节（7个）
            messageData.rjoint1_position = angles[0] ?? 0;
            messageData.rjoint2_position = angles[1] ?? 0;
            messageData.rjoint3_position = angles[2] ?? 0;
            messageData.rjoint4_position = angles[3] ?? 0;
            messageData.rjoint5_position = angles[4] ?? 0;
            messageData.rjoint6_position = angles[5] ?? 0;
            messageData.rjoint7_position = angles[6] ?? 0;
        }

        // 发布消息
        const result = await appState.ros2Communicator.publishRobotArmServoMsg(topicName, messageData);
        return result;
    } catch (error) {
        console.error('发布关节位置消息失败:', error);
        return false;
    }
}


/**
 * 开始持续发布关节位置
 */
function startJointPositionPublishing() {
    // 如果已经在发布，先停止
    if (appState.jointPositionPublishTimer !== null) {
        clearInterval(appState.jointPositionPublishTimer);
    }

    // 检查ROS2连接
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateJointPositionStatus('ROS 2 未连接，无法发布', 'error');
        const toggle = document.getElementById('ros2-joint-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    const topicName = document.getElementById('ros2-joint-position-topic')?.value.trim();
    if (!topicName) {
        updateJointPositionStatus('请设置话题名称', 'error');
        const toggle = document.getElementById('ros2-joint-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    // 立即发布一次
    publishJointPositionMessage();

    // 设置定时器持续发布
    appState.jointPositionPublishTimer = setInterval(() => {
        publishJointPositionMessage();
    }, JOINT_POSITION_PUBLISH_INTERVAL);

    updateJointPositionStatus(`正在持续发布到: ${topicName} (${1000/JOINT_POSITION_PUBLISH_INTERVAL}Hz)`, 'success');
    addROS2LogEntry(`开始持续发布关节位置到: ${topicName}`, 'info');
}

/**
 * 停止持续发布关节位置
 */
function stopJointPositionPublishing() {
    if (appState.jointPositionPublishTimer !== null) {
        clearInterval(appState.jointPositionPublishTimer);
        appState.jointPositionPublishTimer = null;
    }
    updateJointPositionStatus('已停止发布', 'info');
    addROS2LogEntry('已停止关节位置发布', 'info');
}

// ═══════════════════════════════════════════
//  Real Joint State Controls
// ═══════════════════════════════════════════

function setupRealJointStateControls() {
    const subscribeBtn = document.getElementById('ros2-joint-subscribe-btn');
    const topicInput = document.getElementById('ros2-joint-topic');
    appState.ros2JointStatusElement = document.getElementById('ros2-joint-status');
    appState.ros2JointStateButton = subscribeBtn;

    if (!subscribeBtn) {
        return;
    }

    updateJointStateSubscribeButton();
    setRos2JointStatus(appState.ros2JointStateSubscribed ? `已订阅 ${appState.ros2JointStateTopic}` : '未订阅真机关节状态', 'info');

    subscribeBtn.addEventListener('click', async () => {
        if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
            setRos2JointStatus('请先连接 ROS 2', 'error');
            return;
        }

        const topic = topicInput?.value.trim() || '/arm/joint_states';

        if (!appState.ros2JointStateSubscribed) {
            const success = await appState.ros2Communicator.subscribe(topic, 'sensor_msgs/msg/JointState', handleRealJointStateMessage);
            if (success) {
                appState.ros2JointStateSubscribed = true;
                appState.ros2JointStateTopic = topic;
                updateJointStateSubscribeButton();
                setRos2JointStatus(`已订阅 ${topic}`, 'success');
                addROS2LogEntry(`已订阅真机关节: ${topic}`, 'info');
            } else {
                setRos2JointStatus('订阅失败，请检查话题', 'error');
            }
        } else {
            appState.ros2Communicator.unsubscribe(appState.ros2JointStateTopic);
            appState.ros2JointStateSubscribed = false;
            updateJointStateSubscribeButton();
            setRos2JointStatus('已取消订阅', 'warn');
            addROS2LogEntry('已取消真机关节订阅', 'info');
        }
    });
}

function handleRealJointStateMessage(message) {
    const names = Array.isArray(message?.name) ? message.name : null;
    const positions = Array.isArray(message?.position) ? message.position : null;
    if (!names || !positions || names.length === 0 || positions.length === 0) {
        return;
    }

    // 人形模式：使用 humanoidQ / humanoidJointNames 驱动整机 URDF
    if (appState.humanoidMode) {
        const updatedHumanoid = applyHumanoidJointStatesFromMessage(names, positions);
        if (updatedHumanoid) {
            if (callbacks.updateHumanoidJointSlidersFromAngles) {
                callbacks.updateHumanoidJointSlidersFromAngles();
            }
            if (callbacks.updateGlobalVariables) {
                callbacks.updateGlobalVariables();
            }
            if (callbacks.updateRobotPose) {
                callbacks.updateRobotPose();
            }
            setRos2JointStatus(`已接收 ${new Date().toLocaleTimeString()}`, 'success');
        }
        return;
    }

    // 单臂 / 双臂模式：使用关节角数组驱动 URDF
    const updated = applyJointStatesFromMessage(names, positions);
    if (updated) {
        if (callbacks.updateJointSlidersFromAngles) {
            callbacks.updateJointSlidersFromAngles();
        }
        if (callbacks.updateDualArmJointSlidersFromAngles) {
            callbacks.updateDualArmJointSlidersFromAngles();
        }
        if (callbacks.updateGlobalVariables) {
            callbacks.updateGlobalVariables();
        }
        if (callbacks.updateRobotPose) {
            callbacks.updateRobotPose();
        }
        setRos2JointStatus(`已接收 ${new Date().toLocaleTimeString()}`, 'success');
    }
}

function applyJointStatesFromMessage(names, positions) {
    if (appState.dualArmMode) {
        const updatedRight = updateRobotAnglesFromJointState(names, positions, 'robot1');
        const updatedLeft = updateRobotAnglesFromJointState(names, positions, 'robot2');
        return updatedRight || updatedLeft;
    }
    return updateRobotAnglesFromJointState(names, positions, 'single');
}

/**
 * 根据 JointState 消息更新人形机器人关节角（humanoidQ）
 *  - 使用 appState.humanoidJointNames 与 message.name 做一一对应
 *  - 仅在 appState.humanoidMode 为 true 时有效
 */
function applyHumanoidJointStatesFromMessage(names, positions) {
    if (!appState.humanoidMode) {
        return false;
    }
    const jointNames = Array.isArray(appState.humanoidJointNames) ? appState.humanoidJointNames : null;
    const q = Array.isArray(appState.humanoidQ) ? appState.humanoidQ : null;

    if (!jointNames || !q || jointNames.length === 0 || jointNames.length !== q.length) {
        return false;
    }

    const newQ = [...q];
    let updated = false;

    jointNames.forEach((jointName, idx) => {
        const nameIndex = names.indexOf(jointName);
        if (nameIndex !== -1 && Number.isFinite(positions[nameIndex])) {
            newQ[idx] = positions[nameIndex];
            updated = true;
        }
    });

    if (!updated) {
        return false;
    }

    // 覆盖原有 humanoidQ（保持引用不变，方便其他模块使用同一数组）
    newQ.forEach((value, idx) => {
        q[idx] = value;
    });

    return true;
}

function updateRobotAnglesFromJointState(names, positions, robotKey) {
    let jointAngles;
    let jointOrder;
    let urdf = null;
    let config = null;

    if (robotKey === 'single') {
        jointAngles = appState.jointAngles;
        jointOrder = appState.customRobotConfig?.joint_chain?.joints || null;
        urdf = appState.visualizer?.urdfRobot || null;
        config = appState.customRobotConfig || null;
    } else if (robotKey === 'robot1') {
        jointAngles = appState.jointAngles1;
        jointOrder = appState.customRobotConfig?.robot1?.joint_chain?.joints || null;
        urdf = appState.visualizer?.robot1URDF || null;
        config = appState.customRobotConfig?.robot1 || null;
    } else {
        jointAngles = appState.jointAngles2;
        jointOrder = appState.customRobotConfig?.robot2?.joint_chain?.joints || null;
        urdf = appState.visualizer?.robot2URDF || null;
        config = appState.customRobotConfig?.robot2 || null;
    }

    if (!jointAngles || jointAngles.length === 0) {
        return false;
    }

    const newAngles = [...jointAngles];
    let updated = false;

    if (jointOrder && jointOrder.length === jointAngles.length) {
        jointOrder.forEach((jointName, idx) => {
            const nameIndex = names.indexOf(jointName);
            if (nameIndex !== -1 && Number.isFinite(positions[nameIndex])) {
                newAngles[idx] = positions[nameIndex];
                updated = true;
            }
        });
    } else {
        let prefix = null;
        if (appState.dualArmMode) {
            prefix = robotKey === 'robot2' ? 'l' : 'r';
        }
        let assignIndex = 0;
        for (let i = 0; i < names.length && assignIndex < newAngles.length; i++) {
            const value = positions[i];
            if (!Number.isFinite(value)) {
                continue;
            }
            if (!prefix || names[i]?.toLowerCase().startsWith(prefix)) {
                newAngles[assignIndex] = value;
                assignIndex += 1;
                updated = true;
            }
        }
    }

    if (!updated) {
        return false;
    }

    if (jointOrder && callbacks.clampJointAngles) {
        const clamped = callbacks.clampJointAngles(newAngles, jointOrder, urdf, config);
        clamped.forEach((value, idx) => {
            jointAngles[idx] = value;
        });
    } else {
        newAngles.forEach((value, idx) => {
            jointAngles[idx] = value;
        });
    }

    return true;
}

function updateJointStateSubscribeButton() {
    if (appState.ros2JointStateButton) {
        appState.ros2JointStateButton.textContent = appState.ros2JointStateSubscribed ? '取消订阅' : '订阅';
    }
}

function setRos2JointStatus(message, type = 'info') {
    if (!appState.ros2JointStatusElement) {
        return;
    }
    appState.ros2JointStatusElement.textContent = message;
    appState.ros2JointStatusElement.style.color = type === 'success' ? '#10b981' : type === 'error' ? '#f87171' : '#94a3b8';
}

// ═══════════════════════════════════════════
//  Battery State
// ═══════════════════════════════════════════

/**
 * 订阅电池状态话题
 */
async function subscribeBatteryState() {
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        console.warn('ROS 2 未初始化，无法订阅电池状态');
        return false;
    }

    if (appState.ros2BatteryStateSubscribed) {
        console.log('电池状态已订阅');
        return true;
    }

    try {
        const success = await appState.ros2Communicator.subscribe(
            ROS2_BATTERY_STATE_TOPIC,
            'sensor_msgs/msg/BatteryState',
            handleBatteryStateMessage
        );
        
        if (success) {
            appState.ros2BatteryStateSubscribed = true;
            console.log(`✓ 已订阅电池状态话题: ${ROS2_BATTERY_STATE_TOPIC}`);
            return true;
        } else {
            console.warn('订阅电池状态失败');
            return false;
        }
    } catch (error) {
        console.error('订阅电池状态异常:', error);
        return false;
    }
}

/**
 * 处理电池状态消息
 */
function handleBatteryStateMessage(message) {
    if (!message) {
        return;
    }

    // 更新显示
    updateBatteryStateDisplay(message);
}

/**
 * 更新电池状态显示
 */
function updateBatteryStateDisplay(batteryState) {
    console.log(batteryState);
    // 电量百分比 (0-1 转换为 0-100%)
    const percentage = batteryState.percentage;
    const percentageElement = document.getElementById('battery-percentage');
    const batteryBar = document.getElementById('battery-bar');
    
    if (percentageElement && batteryBar) {
        if (typeof percentage === 'number' && !isNaN(percentage)) {
            const percentValue = Math.max(0, Math.min(100, percentage * 100));
            percentageElement.textContent = `${percentValue.toFixed(1)}%`;
            batteryBar.style.width = `${percentValue}%`;
            
            // 根据电量设置颜色
            if (percentValue < 20) {
                batteryBar.style.background = '#ef4444'; // 红色
            } else if (percentValue < 50) {
                batteryBar.style.background = '#f59e0b'; // 橙色
            } else {
                batteryBar.style.background = '#10b981'; // 绿色
            }
        } else {
            percentageElement.textContent = '--';
            batteryBar.style.width = '0%';
        }
    }

    // 电压
    const voltage = batteryState.voltage;
    const voltageElement = document.getElementById('battery-voltage');
    if (voltageElement) {
        if (typeof voltage === 'number' && !isNaN(voltage)) {
            voltageElement.textContent = voltage.toFixed(2);
        } else {
            voltageElement.textContent = '--';
        }
    }

    // 电流
    const current = batteryState.current;
    const currentElement = document.getElementById('battery-current');
    if (currentElement) {
        if (typeof current === 'number' && !isNaN(current)) {
            currentElement.textContent = current.toFixed(2);
        } else {
            currentElement.textContent = '--';
        }
    }

    // 充电状态
    const statusElement = document.getElementById('battery-status');
    if (statusElement) {
        const powerSupplyStatus = batteryState.power_supply_status;
        if (typeof powerSupplyStatus === 'number') {
            // power_supply_status 枚举值:
            // 0 = UNKNOWN, 1 = CHARGING, 2 = DISCHARGING, 3 = NOT_CHARGING, 4 = FULL
            const statusMap = {
                0: '未知',
                1: '充电中',
                2: '放电中',
                3: '未充电',
                4: '已充满'
            };
            statusElement.textContent = statusMap[powerSupplyStatus] || '未知';
            
            // 根据状态设置颜色
            if (powerSupplyStatus === 1) {
                statusElement.style.color = '#10b981'; // 绿色 - 充电中
            } else if (powerSupplyStatus === 2) {
                statusElement.style.color = '#f59e0b'; // 橙色 - 放电中
            } else if (powerSupplyStatus === 4) {
                statusElement.style.color = '#3b82f6'; // 蓝色 - 已充满
            } else {
                statusElement.style.color = '#94a3b8'; // 灰色 - 其他状态
            }
        } else {
            statusElement.textContent = '--';
            statusElement.style.color = '#94a3b8';
        }
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

/**
 * 更新 ROS 2 控制模式显示
 */
// ═══════════════════════════════════════════
//  Callback Registration
// ═══════════════════════════════════════════

/**
 * 注册 ROS2 面板回调到全局 callbacks 对象
 */
export function registerROS2PanelCallbacks() {
    callbacks.addROS2LogEntry = addROS2LogEntry;
    callbacks.initROS2Communication = initROS2Communication;
    callbacks.updateROS2StatusDisplay = updateROS2StatusDisplay;
    callbacks.startJointPositionPublishing = startJointPositionPublishing;
    callbacks.stopJointPositionPublishing = stopJointPositionPublishing;
    callbacks.setupROS2EventListeners = setupROS2EventListeners;
}

// ═══════════════════════════════════════════
//  Exports
// ═══════════════════════════════════════════

export {
    setupROS2EventListeners,
    updateJointPositionStatus,
    publishJointPositionMessage,
    startJointPositionPublishing,
    stopJointPositionPublishing,
    setupRealJointStateControls,
    handleRealJointStateMessage,
    applyJointStatesFromMessage,
    updateRobotAnglesFromJointState,
    updateJointStateSubscribeButton,
    setRos2JointStatus,
    subscribeBatteryState,
    handleBatteryStateMessage,
    updateBatteryStateDisplay,
    addROS2LogEntry,
    updateROS2LogDisplay,
    updateROS2ActiveTopics,
    removeROS2Topic,
    initROS2Communication,
    updateROS2StatusDisplay,
};
