/**
 * ROS2 面板模块
 * 
 * 包含所有 ROS2 相关功能：连接管理、话题订阅/发布、关节状态同步、
 * 关节位置发布、电池状态监控、日志显示等。
 */

import { JOINT_POSITION_PUBLISH_INTERVAL, ROS2_MAX_LOG_ENTRIES, ROS2_BATTERY_STATE_TOPIC } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { onLanguageChange, t } from '../i18n.js';
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

let ros2PanelLanguageBound = false;

function getRos2ConnectionStatusText(status) {
    return {
        disconnected: t('Disconnected'),
        connecting: t('Connecting...'),
        connected: t('Connected'),
        error: t('Error')
    }[status] || status;
}

function getRos2BatteryStatusText(powerSupplyStatus) {
    return {
        0: t('Unknown'),
        1: t('Charging'),
        2: t('Discharging'),
        3: t('Not Charging'),
        4: t('Full')
    }[powerSupplyStatus] || t('Unknown');
}

function refreshROS2LanguageState() {
    updateROS2StatusDisplay(appState.ros2Communicator?.getStatus?.() || 'disconnected');
    callbacks.updateROS2ControlModeDisplay?.(document.getElementById('ros2-control-mode')?.value || 'manual');

    if (appState.ros2JointStateSubscribed && appState.ros2JointStateTopic) {
        setRos2JointStatus(t('Subscribed to {{topic}}', { topic: appState.ros2JointStateTopic }), 'success');
    } else {
        setRos2JointStatus(t('Not subscribed to real joint state'), 'info');
    }

    const jointTopic = document.getElementById('ros2-joint-position-topic')?.value.trim();
    if (appState.jointPositionPublishTimer !== null && jointTopic) {
        updateJointPositionStatus(
            t('Continuously publishing to: {{topic}} ({{hz}}Hz)', {
                topic: jointTopic,
                hz: 1000 / JOINT_POSITION_PUBLISH_INTERVAL
            }),
            'success'
        );
    } else {
        updateJointPositionStatus(t('Switch is off; no messages published'), 'info');
    }

    const waistTopic = document.getElementById('ros2-waist-position-topic')?.value.trim();
    if (appState.waistPositionPublishTimer !== null && waistTopic) {
        updateWaistPositionStatus(
            t('Continuously publishing to: {{topic}} ({{hz}}Hz)', {
                topic: waistTopic,
                hz: 1000 / JOINT_POSITION_PUBLISH_INTERVAL
            }),
            'success'
        );
    } else {
        updateWaistPositionStatus(t('Switch is off; no messages published'), 'info');
    }

    updateROS2LogDisplay();
}

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
                addROS2LogEntry(t('Please enter a valid URL'), 'warn');
                return;
            }
            
            if (!url.startsWith('ws://') && !url.startsWith('wss://')) {
                addROS2LogEntry(t('The URL must start with ws:// or wss://'), 'warn');
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
            addROS2LogEntry(t('Saved URL: {{url}}. Please reconnect.', { url }), 'info');
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
                        setRos2JointStatus(t('Unsubscribed (preparing to disconnect)'), 'warn');
                        addROS2LogEntry(
                            t('Unsubscribed from real joint state before disconnecting: {{topic}}', {
                                topic: appState.ros2JointStateTopic
                            }),
                            'info'
                        );
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
                addROS2LogEntry(t('ROS 2 is not connected'), 'error');
                return;
            }

            const controlModeSelect = document.getElementById('ros2-control-mode');
            const selectedMode = controlModeSelect ? controlModeSelect.value : 'manual';

            // 设置处理中状态
            switchModeBtnProcessing = true;
            const originalText = switchModeBtn.textContent;
            switchModeBtn.textContent = t('Switching...');
            switchModeBtn.disabled = true;
            switchModeBtn.style.opacity = '0.6';

            try {
                await appState.ros2Communicator.switchMode(selectedMode);
                addROS2LogEntry(t('Switched control mode: {{mode}}', { mode: selectedMode }), 'info');
                callbacks.updateROS2ControlModeDisplay?.(selectedMode);
            } catch (error) {
                addROS2LogEntry(t('Failed to switch mode: {{message}}', { message: error.message }), 'error');
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

    if (!ros2PanelLanguageBound) {
        onLanguageChange(() => {
            refreshROS2LanguageState();
        });
        ros2PanelLanguageBound = true;
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

function getHumanoidRos2PublishDefaults() {
    const defaults = appState.humanoidRos2PublishDefaults || {};
    return {
        armVel: Number.isFinite(defaults.armVel) ? defaults.armVel : 0.5,
        armAcc: Number.isFinite(defaults.armAcc) ? defaults.armAcc : 1.0,
        waistVel: Number.isFinite(defaults.waistVel) ? defaults.waistVel : 0.5,
        waistAcc: Number.isFinite(defaults.waistAcc) ? defaults.waistAcc : 1.0
    };
}

/**
 * 发布关节位置消息（内部函数）
 */
async function publishJointPositionMessage() {
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateJointPositionStatus(t('ROS 2 is not connected'), 'error');
        return false;
    }

    const topicName = document.getElementById('ros2-joint-position-topic')?.value.trim();
    if (!topicName) {
        updateJointPositionStatus(t('Topic name is not set'), 'error');
        return false;
    }

    if (appState.humanoidMode && (!Array.isArray(appState.humanoidQ) || appState.humanoidQ.length < 18)) {
        updateJointPositionStatus(t('humanoidQ is not ready, so humanoid arm joints cannot be published'), 'warn');
        return false;
    }

    try {
        const publishDefaults = getHumanoidRos2PublishDefaults();
        const { messageData, source } = buildRobotArmServoMessageData({
            humanoidMode: !!appState.humanoidMode,
            humanoidQ: appState.humanoidQ,
            dualArmMode: !!appState.dualArmMode,
            jointAngles: appState.jointAngles,
            jointAngles1: appState.jointAngles1,
            jointAngles2: appState.jointAngles2,
            runMode: 1,
            robotPower: true,
            runTime: 0,
            vel: publishDefaults.armVel,
            acc: publishDefaults.armAcc
        });

        // 发布消息
        const result = await appState.ros2Communicator.publishRobotArmServoMsg(topicName, messageData);
        if (result) {
            updateJointPositionStatus(
                t('Sent to {{topic}} (source: {{source}})', { topic: topicName, source }),
                'success'
            );
        } else {
            updateJointPositionStatus(t('Send failed: {{topic}}', { topic: topicName }), 'error');
        }
        return result;
    } catch (error) {
        console.error('发布关节位置消息失败:', error);
        updateJointPositionStatus(t('Publish failed: {{message}}', { message: error.message }), 'error');
        return false;
    }
}

async function publishWaistPositionMessage() {
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateWaistPositionStatus(t('ROS 2 is not connected'), 'error');
        return false;
    }

    const topicName = document.getElementById('ros2-waist-position-topic')?.value.trim();
    if (!topicName) {
        updateWaistPositionStatus(t('Topic name is not set'), 'error');
        return false;
    }

    const publishDefaults = getHumanoidRos2PublishDefaults();
    const { valid, messageData } = buildRobotServoMessageData({
        humanoidMode: !!appState.humanoidMode,
        humanoidQ: appState.humanoidQ,
        runMode: 0,
        vel: publishDefaults.waistVel,
        acc: publishDefaults.waistAcc
    });

    if (!valid) {
        updateWaistPositionStatus(t('Waist joints can only be published in humanoid mode'), 'warn');
        return false;
    }

    try {
        const result = await appState.ros2Communicator.publishRobotServoMsg(topicName, messageData);
        if (result) {
            updateWaistPositionStatus(t('Sent to {{topic}}', { topic: topicName }), 'success');
        }
        return result;
    } catch (error) {
        console.error('发布腰部位置消息失败:', error);
        updateWaistPositionStatus(t('Publish failed: {{message}}', { message: error.message }), 'error');
        return false;
    }
}


/**
 * 开始持续发布关节位置
 */
function startJointPositionPublishing() {
    // 与真机关节订阅互锁：若当前已订阅真机关节状态，则不允许开启持续发布
    if (appState.ros2JointStateSubscribed) {
        updateJointPositionStatus(
            t('Real joint state is currently subscribed. Please unsubscribe before starting continuous joint publishing.'),
            'warn'
        );
        const toggle = document.getElementById('ros2-joint-position-toggle');
        if (toggle) toggle.checked = false;
        addROS2LogEntry(
            t('Continuous joint publishing was blocked because real joint state is subscribed. Please unsubscribe first.'),
            'warn'
        );
        return;
    }

    // 如果已经在发布，先停止
    if (appState.jointPositionPublishTimer !== null) {
        clearInterval(appState.jointPositionPublishTimer);
    }

    // 检查ROS2连接
    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateJointPositionStatus(t('ROS 2 is not connected, so publishing is unavailable'), 'error');
        const toggle = document.getElementById('ros2-joint-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    const topicName = document.getElementById('ros2-joint-position-topic')?.value.trim();
    if (!topicName) {
        updateJointPositionStatus(t('Please set a topic name'), 'error');
        const toggle = document.getElementById('ros2-joint-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    if (appState.humanoidMode && (!Array.isArray(appState.humanoidQ) || appState.humanoidQ.length < 18)) {
        updateJointPositionStatus(t('humanoidQ is not ready, so humanoid arm joints cannot be published'), 'warn');
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

    updateJointPositionStatus(
        t('Continuously publishing to: {{topic}} ({{hz}}Hz)', {
            topic: topicName,
            hz: 1000 / JOINT_POSITION_PUBLISH_INTERVAL
        }),
        'success'
    );
    addROS2LogEntry(t('Started continuous joint publishing to: {{topic}}', { topic: topicName }), 'info');
}

/**
 * 停止持续发布关节位置
 */
function stopJointPositionPublishing() {
    if (appState.jointPositionPublishTimer !== null) {
        clearInterval(appState.jointPositionPublishTimer);
        appState.jointPositionPublishTimer = null;
    }
    updateJointPositionStatus(t('Publishing stopped'), 'info');
    addROS2LogEntry(t('Continuous joint publishing stopped'), 'info');
}

function startWaistPositionPublishing() {
    // 与真机关节订阅互锁：若当前已订阅真机关节状态，则不允许开启持续发布
    if (appState.ros2JointStateSubscribed) {
        updateWaistPositionStatus(
            t('Real joint state is currently subscribed. Please unsubscribe before starting continuous waist publishing.'),
            'warn'
        );
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        addROS2LogEntry(
            t('Continuous waist publishing was blocked because real joint state is subscribed. Please unsubscribe first.'),
            'warn'
        );
        return;
    }

    if (appState.waistPositionPublishTimer !== null) {
        clearInterval(appState.waistPositionPublishTimer);
    }

    if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
        updateWaistPositionStatus(t('ROS 2 is not connected, so publishing is unavailable'), 'error');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    const topicName = document.getElementById('ros2-waist-position-topic')?.value.trim();
    if (!topicName) {
        updateWaistPositionStatus(t('Please set a topic name'), 'error');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    if (!appState.humanoidMode || !Array.isArray(appState.humanoidQ) || appState.humanoidQ.length < 4) {
        updateWaistPositionStatus(t('Waist joints can only be published in humanoid mode'), 'warn');
        const toggle = document.getElementById('ros2-waist-position-toggle');
        if (toggle) toggle.checked = false;
        return;
    }

    publishWaistPositionMessage();

    appState.waistPositionPublishTimer = setInterval(() => {
        publishWaistPositionMessage();
    }, JOINT_POSITION_PUBLISH_INTERVAL);

    updateWaistPositionStatus(
        t('Continuously publishing to: {{topic}} ({{hz}}Hz)', {
            topic: topicName,
            hz: 1000 / JOINT_POSITION_PUBLISH_INTERVAL
        }),
        'success'
    );
    addROS2LogEntry(t('Started continuous waist publishing to: {{topic}}', { topic: topicName }), 'info');
}

function stopWaistPositionPublishing() {
    if (appState.waistPositionPublishTimer !== null) {
        clearInterval(appState.waistPositionPublishTimer);
        appState.waistPositionPublishTimer = null;
    }
    updateWaistPositionStatus(t('Publishing stopped'), 'info');
    addROS2LogEntry(t('Continuous waist publishing stopped'), 'info');
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
    setRos2JointStatus(
        appState.ros2JointStateSubscribed
            ? t('Subscribed to {{topic}}', { topic: appState.ros2JointStateTopic })
            : t('Not subscribed to real joint state'),
        'info'
    );

    subscribeBtn.addEventListener('click', async () => {
        if (!appState.ros2Communicator || !appState.ros2Communicator.isInitialized) {
            setRos2JointStatus(t('Please connect to ROS 2 first'), 'error');
            return;
        }

        const topic = topicInput?.value.trim() || DEFAULT_REAL_JOINT_STATE_TOPIC;

        if (!appState.ros2JointStateSubscribed) {
            // 与持续发布互锁：若当前已开启任一持续发布，则不允许订阅真机关节状态
            if (appState.jointPositionPublishTimer !== null || appState.waistPositionPublishTimer !== null) {
                setRos2JointStatus(
                    t('Continuous joint or waist publishing is active. Please stop publishing before subscribing to real joint state.'),
                    'warn'
                );
                addROS2LogEntry(
                    t('Subscribing to real joint state was blocked because continuous publishing is active. Please stop publishing first.'),
                    'warn'
                );
                return;
            }

            const success = await appState.ros2Communicator.subscribe(topic, 'sensor_msgs/msg/JointState', handleRealJointStateMessage);
            if (success) {
                appState.ros2JointStateSubscribed = true;
                appState.ros2JointStateTopic = topic;
                updateJointStateSubscribeButton();
                setRos2JointStatus(t('Subscribed to {{topic}}', { topic }), 'success');
                addROS2LogEntry(t('Subscribed to real joint state: {{topic}}', { topic }), 'info');
            } else {
                setRos2JointStatus(t('Subscription failed. Please check the topic.'), 'error');
            }
        } else {
            appState.ros2Communicator.unsubscribe(appState.ros2JointStateTopic);
            appState.ros2JointStateSubscribed = false;
            updateJointStateSubscribeButton();
            setRos2JointStatus(t('Unsubscribed'), 'warn');
            addROS2LogEntry(t('Unsubscribed from real joint state'), 'info');
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
        addROS2LogEntry(
            t('Continuous joint or waist publishing is active, so automatic real joint state subscription was skipped. Stop publishing first if you want to subscribe.'),
            'warn'
        );
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
            setRos2JointStatus(t('Subscribed to {{topic}}', { topic }), 'success');
            addROS2LogEntry(t('Subscribed to real joint state: {{topic}}', { topic }), 'info');
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
            setRos2JointStatus(
                t('Received at {{time}}; matched {{count}} joints', {
                    time: new Date().toLocaleTimeString(),
                    count: humanoidResult.updatedCount
                }),
                'success'
            );
        } else {
            setRos2JointStatus(
                t('Received at {{time}}, but no updatable humanoid joints were matched', {
                    time: new Date().toLocaleTimeString()
                }),
                'warn'
            );
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
        setRos2JointStatus(t('Received at {{time}}', { time: new Date().toLocaleTimeString() }), 'success');
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
        appState.ros2JointStateButton.textContent = appState.ros2JointStateSubscribed
            ? t('Unsubscribe')
            : t('Subscribe');
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
            statusElement.textContent = getRos2BatteryStatusText(powerSupplyStatus);
            
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
        logContainer.innerHTML = `<div style="color: #94a3b8; font-size: 0.85rem;">${t('No messages yet')}</div>`;
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
            addROS2LogEntry(t('Status: {{status}}', { status: getRos2ConnectionStatusText(status) }), 'info');
            if (status === 'disconnected' && appState.ros2JointStateSubscribed) {
                appState.ros2JointStateSubscribed = false;
                updateJointStateSubscribeButton();
                setRos2JointStatus(t('Connection lost; subscription stopped'), 'error');
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
            addROS2LogEntry(t('Current control mode: {{mode}}', { mode }), 'info');
        });
        
        // 显示连接的 URL
        addROS2LogEntry(
            t('Connection URL: {{url}}', { url: appState.ros2Communicator.getWebSocketUrl() }),
            'info'
        );
        
        // 尝试初始化
        const result = await appState.ros2Communicator.init();
        if (result) {
            addROS2LogEntry(t('ROS 2 initialized successfully'), 'info');
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
            addROS2LogEntry(t('ROS 2 initialization failed'), 'error');
        }
    } catch (error) {
        console.error('ROS 2 初始化异常:', error);
        addROS2LogEntry(t('Initialization exception: {{message}}', { message: error.message }), 'error');
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
    text.textContent = getRos2ConnectionStatusText(status);
    
    // 未连接时按钮显示「连接」，已连接时显示「断开」
    if (connectBtn) {
        connectBtn.textContent = status === 'connected' ? t('Disconnect') : t('Connect');
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
