/**
 * ROS2 面板模块
 * 
 * 包含所有 ROS2 相关功能：连接管理、话题订阅/发布、关节状态同步、
 * 关节位置发布、电池状态监控、日志显示等。
 */

import { JOINT_POSITION_PUBLISH_INTERVAL, ROS2_MAX_LOG_ENTRIES, ROS2_BATTERY_STATE_TOPIC } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import ROS2Communicator from '../ros2-communicator.js';
import {
    DEFAULT_REAL_JOINT_STATE_TOPIC,
    DEFAULT_HUMANOID_IGNORED_JOINTS,
    mapHumanoidJointPositions
} from './joint-state-mapping.js';
import {
    buildRobotArmServoMessageData,
    buildRobotServoMessageData
} from './servo-message-payloads.js';

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
                // 断开连接前，先取消真机关节状态订阅，避免残留状态影响下次自动订阅
                try {
                    if (appState.ros2JointStateSubscribed && appState.ros2JointStateTopic) {
                        appState.ros2Communicator.unsubscribe(appState.ros2JointStateTopic);
                        appState.ros2JointStateSubscribed = false;
                        updateJointStateSubscribeButton();
                        setRos2JointStatus('已取消订阅（准备断开连接）', 'warn');
                        addROS2LogEntry(`断开前已取消真机关节订阅: ${appState.ros2JointStateTopic}`, 'info');
                    }
                } catch (error) {
                    console.warn('断开前取消真机关节订阅失败:', error);
                }

                await appState.ros2Communicator.shutdown();
                updateROS2StatusDisplay('disconnected');
            } else {
                const ok = await appState.ros2Communicator.init();
                if (ok) {
                    // 重新连接成功后自动订阅真机关节状态
                    await autoSubscribeRealJointState();
                }
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

    const waistPositionToggle = document.getElementById('ros2-waist-position-toggle');
    if (waistPositionToggle) {
        waistPositionToggle.addEventListener('change', (e) => {
            if (e.target.checked) {
                startWaistPositionPublishing();
            } else {
                stopWaistPositionPublishing();
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
    updateServoPublishStatus('ros2-joint-position-status', message, type);
}

function updateWaistPositionStatus(message, type = 'info') {
    updateServoPublishStatus('ros2-waist-position-status', message, type);
}

function updateServoPublishStatus(elementId, message, type = 'info') {
    const statusEl = document.getElementById(elementId);
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

    if (appState.humanoidMode && (!Array.isArray(appState.humanoidQ) || appState.humanoidQ.length < 18)) {
        updateJointPositionStatus('humanoidQ 未就绪，无法发布人形手臂关节', 'warn');
        return false;
    }

    try {
        const { messageData, source } = buildRobotArmServoMessageData({
            humanoidMode: !!appState.humanoidMode,
            humanoidQ: appState.humanoidQ,
            dualArmMode: !!appState.dualArmMode,
            jointAngles: appState.jointAngles,
            jointAngles1: appState.jointAngles1,
            jointAngles2: appState.jointAngles2,
            runMode: 1,
            robotPower: true,
            runTime: 0
        });

        // 发布消息
        const result = await appState.ros2Communicator.publishRobotArmServoMsg(topicName, messageData);
        if (result) {
            updateJointPositionStatus(`已发送到 ${topicName}（来源: ${source}）`, 'success');
        } else {
            updateJointPositionStatus(`发送失败: ${topicName}`, 'error');
        }
        return result;
    } catch (error) {
        console.error('发布关节位置消息失败:', error);
        updateJointPositionStatus(`发布失败: ${error.message}`, 'error');
        return false;
    }
}

async function publishWaistPositionMessage() {
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateWaistPositionStatus('ROS 2 未连接', 'error');
        return false;
    }

    const topicName = document.getElementById('ros2-waist-position-topic')?.value.trim();
    if (!topicName) {
        updateWaistPositionStatus('话题名称未设置', 'error');
        return false;
    }

    const { valid, messageData } = buildRobotServoMessageData({
        humanoidMode: !!appState.humanoidMode,
        humanoidQ: appState.humanoidQ,
        runMode: 0
    });

    if (!valid) {
        updateWaistPositionStatus('仅人形模式可发布腰部关节', 'warn');
        return false;
    }

    try {
        const result = await appState.ros2Communicator.publishRobotServoMsg(topicName, messageData);
        if (result) {
            updateWaistPositionStatus(`已发送到 ${topicName}`, 'success');
        }
        return result;
    } catch (error) {
        console.error('发布腰部位置消息失败:', error);
        updateWaistPositionStatus(`发布失败: ${error.message}`, 'error');
        return false;
    }
}


/**
 * 开始持续发布关节位置
 */
function startJointPositionPublishing() {
    // 与真机关节订阅互锁：若当前已订阅真机关节状态，则不允许开启持续发布
    if (appState.ros2JointStateSubscribed) {
        updateJointPositionStatus('当前已订阅真机关节状态，如需持续发布关节位置，请先取消真机关节订阅。', 'warn');
        const toggle = document.getElementById('ros2-joint-position-toggle');
        if (toggle) toggle.checked = false;
        addROS2LogEntry('开启持续发布关节位置被阻止：已订阅真机关节状态，请先取消订阅。', 'warn');
        return;
    }

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

    if (appState.humanoidMode && (!Array.isArray(appState.humanoidQ) || appState.humanoidQ.length < 18)) {
        updateJointPositionStatus('humanoidQ 未就绪，无法发布人形手臂关节', 'warn');
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

function startWaistPositionPublishing() {
    // 与真机关节订阅互锁：若当前已订阅真机关节状态，则不允许开启持续发布
    if (appState.ros2JointStateSubscribed) {
        updateWaistPositionStatus('当前已订阅真机关节状态，如需持续发布腰部关节，请先取消真机关节订阅。', 'warn');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        addROS2LogEntry('开启持续发布腰部关节被阻止：已订阅真机关节状态，请先取消订阅。', 'warn');
        return;
    }

    if (appState.waistPositionPublishTimer !== null) {
        clearInterval(appState.waistPositionPublishTimer);
    }

    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateWaistPositionStatus('ROS 2 未连接，无法发布', 'error');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    const topicName = document.getElementById('ros2-waist-position-topic')?.value.trim();
    if (!topicName) {
        updateWaistPositionStatus('请设置话题名称', 'error');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    if (!appState.humanoidMode || !Array.isArray(appState.humanoidQ) || appState.humanoidQ.length < 4) {
        updateWaistPositionStatus('仅人形模式可发布腰部关节', 'warn');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    publishWaistPositionMessage();

    appState.waistPositionPublishTimer = setInterval(() => {
        publishWaistPositionMessage();
    }, JOINT_POSITION_PUBLISH_INTERVAL);

    updateWaistPositionStatus(`正在持续发布到: ${topicName} (${1000 / JOINT_POSITION_PUBLISH_INTERVAL}Hz)`, 'success');
    addROS2LogEntry(`开始持续发布腰部关节到: ${topicName}`, 'info');
}

function stopWaistPositionPublishing() {
    if (appState.waistPositionPublishTimer !== null) {
        clearInterval(appState.waistPositionPublishTimer);
        appState.waistPositionPublishTimer = null;
    }
    updateWaistPositionStatus('已停止发布', 'info');
    addROS2LogEntry('已停止腰部关节发布', 'info');
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

        const topic = topicInput?.value.trim() || DEFAULT_REAL_JOINT_STATE_TOPIC;

        if (!appState.ros2JointStateSubscribed) {
            // 与持续发布互锁：若当前已开启任一持续发布，则不允许订阅真机关节状态
            if (appState.jointPositionPublishTimer !== null || appState.waistPositionPublishTimer !== null) {
                setRos2JointStatus('当前已开启持续发布关节/腰部关节，请先关闭相应发布，再订阅真机关节状态。', 'warn');
                addROS2LogEntry('订阅真机关节状态被阻止：存在持续发布关节/腰部关节，请先关闭相应发布。', 'warn');
                return;
            }

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

/**
 * 连接成功后自动订阅真机关节状态（使用当前输入框或默认话题）
 * - 仅在尚未订阅时执行，避免重复订阅
 * - 可在首次连接和后续重新连接时复用
 */
async function autoSubscribeRealJointState() {
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        return;
    }
    if (appState.ros2JointStateSubscribed) {
        return;
    }

    // 与持续发布互锁：若当前已开启任一持续发布，则跳过自动订阅，仅给出提示
    if (appState.jointPositionPublishTimer !== null || appState.waistPositionPublishTimer !== null) {
        addROS2LogEntry('检测到已开启持续发布关节/腰部关节，已跳过自动订阅真机关节状态。如需订阅，请先关闭持续发布。', 'warn');
        return;
    }

    try {
        const topicInput = document.getElementById('ros2-joint-topic');
        const topic = topicInput?.value?.trim() || DEFAULT_REAL_JOINT_STATE_TOPIC;
        const success = await appState.ros2Communicator.subscribe(
            topic,
            'sensor_msgs/msg/JointState',
            handleRealJointStateMessage
        );
        if (success) {
            appState.ros2JointStateSubscribed = true;
            appState.ros2JointStateTopic = topic;
            updateJointStateSubscribeButton();
            setRos2JointStatus(`已订阅 ${topic}`, 'success');
            addROS2LogEntry(`已订阅真机关节: ${topic}`, 'info');
        }
    } catch (error) {
        console.warn('自动订阅真机关节状态失败:', error);
    }
}

function handleRealJointStateMessage(message) {
    const names = Array.isArray(message?.name) ? message.name : (Array.isArray(message?.names) ? message.names : null);
    const positions = Array.isArray(message?.position) ? message.position : (Array.isArray(message?.positions) ? message.positions : null);
    if (!names || !positions || names.length === 0 || positions.length === 0) {
        return;
    }

    // 人形模式：使用 humanoidQ / humanoidJointNames 驱动整机 URDF
    if (appState.humanoidMode) {
        const humanoidResult = applyHumanoidJointStatesFromMessage(names, positions);
        if (humanoidResult.updated) {
            if (typeof window !== 'undefined' && Array.isArray(window.humanoidQ) && window.humanoidQ.length === appState.humanoidQ.length) {
                appState.humanoidQ.forEach((value, idx) => {
                    window.humanoidQ[idx] = value;
                });
            }
            if (appState.visualizer?.setHumanoidJoints) {
                appState.visualizer.setHumanoidJoints(appState.humanoidQ);
            }
            if (callbacks.updateHumanoidJointSlidersFromAngles) {
                callbacks.updateHumanoidJointSlidersFromAngles();
            }
            if (callbacks.updateGlobalVariables) {
                callbacks.updateGlobalVariables();
            }
            if (callbacks.updateRobotPose) {
                callbacks.updateRobotPose();
            }
            setRos2JointStatus(`已接收 ${new Date().toLocaleTimeString()}，匹配 ${humanoidResult.updatedCount} 个关节`, 'success');
        } else {
            setRos2JointStatus(`收到 ${new Date().toLocaleTimeString()}，但未匹配到可更新的人形关节`, 'warn');
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
        return { updated: false, updatedCount: 0 };
    }
    const jointNames = Array.isArray(appState.humanoidJointNames) ? appState.humanoidJointNames : null;
    const q = Array.isArray(appState.humanoidQ) ? appState.humanoidQ : null;

    if (!jointNames || !q || jointNames.length === 0 || jointNames.length !== q.length) {
        return { updated: false, updatedCount: 0 };
    }

    const result = mapHumanoidJointPositions({
        messageNames: names,
        positions,
        jointNames,
        currentQ: q,
        ignoredJointNames: DEFAULT_HUMANOID_IGNORED_JOINTS
    });

    if (!result.updated) {
        return { updated: false, updatedCount: 0 };
    }

    // 覆盖原有 humanoidQ（保持引用不变，方便其他模块使用同一数组）
    result.nextQ.forEach((value, idx) => {
        q[idx] = value;
    });

    return {
        updated: true,
        updatedCount: result.updatedCount
    };
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
    const colorMap = {
        success: '#10b981',
        error: '#f87171',
        warn: '#f59e0b',
        info: '#94a3b8'
    };
    const color = colorMap[type] || colorMap.info;
    appState.ros2JointStatusElement.style.color = color;
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
                if (appState.waistPositionPublishTimer !== null) {
                    stopWaistPositionPublishing();
                    const toggle = document.getElementById('ros2-waist-position-toggle');
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
            // 连接成功后自动订阅真机关节状态（使用设置的话题）
            await autoSubscribeRealJointState();
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
    const connectBtn = document.getElementById('ros2-connect-btn');
    
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
    
    // 未连接时按钮显示「连接」，已连接时显示「断开」
    if (connectBtn) {
        connectBtn.textContent = status === 'connected' ? '断开' : '连接';
    }
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
    updateWaistPositionStatus,
    publishJointPositionMessage,
    publishWaistPositionMessage,
    startJointPositionPublishing,
    stopJointPositionPublishing,
    startWaistPositionPublishing,
    stopWaistPositionPublishing,
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
    initROS2Communication,
    updateROS2StatusDisplay,
};
