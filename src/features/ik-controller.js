/**
 * IK 控制器模块
 * - formatDualQuaternion: 格式化 DQ 数组为字符串
 * - updateIKRobotSelection: 更新 IK 机器人选择器
 * - getCurrentRobotDQ: 获取当前机器人 DQ
 * - setupInverseKinematicsControls: 设置逆运动学控制面板
 */

import { appState, callbacks } from '../core/state.js';
import { DQ, getDQModule } from '../dqrobotics.js';

// ──────────────────────────────────────────────────────────────
// formatDualQuaternion
// ──────────────────────────────────────────────────────────────

export function formatDualQuaternion(dqArray) {
    if (!Array.isArray(dqArray)) {
        return '';
    }
    return dqArray
        .map((value) => {
            const rounded = Number(value.toFixed(6));
            return Number.isFinite(rounded) ? rounded : 0;
        })
        .join(',');
}

// ──────────────────────────────────────────────────────────────
// updateIKRobotSelection
// ──────────────────────────────────────────────────────────────

/**
 * 更新 IK 机器人选择依据单/双臂
 */
export function updateIKRobotSelection() {
    const ikSelect = document.getElementById('ik-robot-select');
    const ikStatus = document.getElementById('ik-status');
    if (!ikSelect) return;

    const prevValue = ikSelect.value;
    ikSelect.innerHTML = '';
    const arm1Label = appState.humanoidMode ? '左臂' : (appState.dualArmMode ? '机器人1' : '机器人');
    const opt1 = document.createElement('option');
    opt1.value = 'robot1';
    opt1.textContent = arm1Label;
    ikSelect.appendChild(opt1);

    const showDualOptions = appState.dualArmMode || appState.humanoidMode;
    if (showDualOptions) {
        const opt2 = document.createElement('option');
        opt2.value = 'robot2';
        opt2.textContent = appState.humanoidMode ? '右臂' : '机器人2';
        ikSelect.appendChild(opt2);

        const optAbs = document.createElement('option');
        optAbs.value = 'absolute';
        optAbs.textContent = '绝对位姿';
        ikSelect.appendChild(optAbs);

        const optRel = document.createElement('option');
        optRel.value = 'relative';
        optRel.textContent = '相对位姿';
        ikSelect.appendChild(optRel);

        ikSelect.disabled = false;
        if (ikStatus) {
            ikStatus.textContent = appState.humanoidMode
                ? '人形模式：左臂/右臂/绝对位姿/相对位姿'
                : '双臂模式：可选择机器人1/机器人2/绝对位姿/相对位姿';
        }
    } else {
        ikSelect.disabled = true;
        if (ikStatus) {
            ikStatus.textContent = '单臂模式：只控制机器人1';
        }
    }

    // 尽量保留之前的选择
    if (prevValue) {
        const hasPrev = Array.from(ikSelect.options).some(opt => opt.value === prevValue);
        if (hasPrev) {
            ikSelect.value = prevValue;
        }
    }
}

// ──────────────────────────────────────────────────────────────
// getCurrentRobotDQ
// ──────────────────────────────────────────────────────────────

/**
 * 获取当前选中机器人的双四元数
 * @param {string} robotChoice
 * @returns {number[]|null}
 */
export function getCurrentRobotDQ(robotChoice) {
    if (!window.DQModule || !window.DQModule.DQWrapper) {
        console.warn('DQ 模块未加载');
        return null;
    }

    try {
        if (appState.humanoidMode) {
            if (!appState.humanoidModel || !appState.humanoidQ?.length) return null;
            const baseDQ = DQ.pose(
                0, 0, 1,
                appState.humanoidBasePose?.yaw || 0,
                appState.humanoidBasePose?.x || 0,
                appState.humanoidBasePose?.y || 0,
                0
            ).toArray();
            const baseObj = window.DQModule.DQWrapper.createFromArray(baseDQ);

            if (robotChoice === 'robot1') {
                const local = appState.humanoidModel.fkmEffector(appState.humanoidQ, 1);
                return window.DQModule.DQWrapper.toArray(
                    window.DQModule.DQWrapper.multiply(baseObj, window.DQModule.DQWrapper.createFromArray(local))
                );
            }
            if (robotChoice === 'robot2') {
                const local = appState.humanoidModel.fkmEffector(appState.humanoidQ, 2);
                return window.DQModule.DQWrapper.toArray(
                    window.DQModule.DQWrapper.multiply(baseObj, window.DQModule.DQWrapper.createFromArray(local))
                );
            }
            if (robotChoice === 'absolute') {
                if (!appState.dualArmSystem || typeof appState.dualArmSystem.getAbsolutePose !== 'function') {
                    return null;
                }
                const local = appState.dualArmSystem.getAbsolutePose(appState.humanoidQ);
                return window.DQModule.DQWrapper.toArray(
                    window.DQModule.DQWrapper.multiply(baseObj, window.DQModule.DQWrapper.createFromArray(local))
                );
            }
            if (robotChoice === 'relative') {
                if (!appState.dualArmSystem || typeof appState.dualArmSystem.getRelativePose !== 'function') {
                    return null;
                }
                return appState.dualArmSystem.getRelativePose(appState.humanoidQ);
            }
            return null;
        }

        if (appState.dualArmMode) {
            // 双臂：支持机器人1/2 + 绝对/相对位姿
            const da = appState.dualArmSystem;
            if (robotChoice === 'absolute') {
                if (!da || !appState.jointAngles1?.length || !appState.jointAngles2?.length) {
                    return null;
                }
                const allJointAngles = [...appState.jointAngles1, ...appState.jointAngles2];
                return da.getAbsolutePose(allJointAngles);
            }
            if (robotChoice === 'relative') {
                if (!da || !appState.jointAngles1?.length || !appState.jointAngles2?.length) {
                    return null;
                }
                const allJointAngles = [...appState.jointAngles1, ...appState.jointAngles2];
                return da.getRelativePose(allJointAngles);
            }
            if (robotChoice === 'robot2') {
                if (!appState.robot2 || !appState.jointAngles2?.length) {
                    return null;
                }
                return appState.robot2.fkm(appState.jointAngles2);
            }
            if (!appState.robot1 || !appState.jointAngles1?.length) {
                return null;
            }
            return appState.robot1.fkm(appState.jointAngles1);
        }

        if (!appState.currentRobot || !appState.jointAngles?.length) {
            return null;
        }
        return appState.currentRobot.fkm(appState.jointAngles);
    } catch (error) {
        console.error('获取当前双四元数失败:', error);
        return null;
    }
}

// ──────────────────────────────────────────────────────────────
// setupInverseKinematicsControls
// ──────────────────────────────────────────────────────────────

/**
 * 控制面板 - 逆运动学按钮
 */
export function setupInverseKinematicsControls() {
    const ikSolveBtn = document.getElementById('ik-solve-btn');
    const copyDqBtn = document.getElementById('copy-dq-btn');
    const ikStatus = document.getElementById('ik-status');
    if (!ikSolveBtn && !copyDqBtn) {
        return;
    }

    const ikSelect = document.getElementById('ik-robot-select');
    const updateIkSolveAvailability = () => {
        // 双臂或人形模式下允许求解 absolute/relative；人形下左臂=robot1、右臂=robot2
        if (!ikStatus) return;
        if (!appState.dualArmMode && !appState.humanoidMode) {
            ikStatus.textContent = '单臂模式：使用雅可比伪逆更新当前机器人角度';
            ikStatus.style.color = '#94a3b8';
            return;
        }
        const choice = ikSelect?.value || 'robot1';
        if (choice === 'absolute') {
            ikStatus.textContent = appState.humanoidMode ? '人形绝对位姿：求解绝对位姿，保持相对位姿不变' : '双臂绝对位姿：求解绝对位姿，保持相对位姿不变';
            ikStatus.style.color = '#94a3b8';
            return;
        }
        if (choice === 'relative') {
            ikStatus.textContent = appState.humanoidMode ? '人形相对位姿：求解相对位姿，保持绝对位姿不变' : '双臂相对位姿：求解相对位姿，保持绝对位姿不变';
            ikStatus.style.color = '#94a3b8';
            return;
        }
        ikStatus.textContent = appState.humanoidMode ? '人形单臂求解：仅更新左臂或右臂角度' : '双臂单臂求解：仅更新机器人1或机器人2角度';
        ikStatus.style.color = '#94a3b8';
    };

    ikSolveBtn.addEventListener('click', () => {
        const dqText = document.getElementById('ik-dq-input')?.value || '';
        const dqArray = parseDualQuaternionInput(dqText);
        if (!dqArray) {
            callbacks.addROS2LogEntry?.('请输入 8 个双四元数元素（逗号或空格分隔）', 'warn');
            if (ikStatus) {
                ikStatus.textContent = '请输入 8 个双四元数元素（逗号或空格分隔）';
                ikStatus.style.color = '#f87171';
            }
            return;
        }

        const robotChoice = document.getElementById('ik-robot-select')?.value || 'robot1';
        const vis = appState.visualizer;
        if (!vis) {
            callbacks.addROS2LogEntry?.('视觉器未初始化，无法求解逆运动学', 'error');
            if (ikStatus) {
                ikStatus.textContent = '视觉器未初始化，无法求解逆运动学';
                ikStatus.style.color = '#f87171';
            }
            return;
        }

        try {
            let success = false;
            if (appState.humanoidMode && (robotChoice === 'absolute' || robotChoice === 'relative')) {
                const allQ = appState.humanoidQ;
                if (!allQ || allQ.length === 0) {
                    if (ikStatus) { ikStatus.textContent = '人形关节未初始化'; ikStatus.style.color = '#f87171'; }
                    return;
                }
                const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
                if (robotChoice === 'absolute') {
                    success = vis.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst?.(
                        dqArray,
                        allQ,
                        enableLimitAvoidance,
                        true
                    ) === true;
                } else {
                    success = vis.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst?.(
                        dqArray,
                        allQ,
                        enableLimitAvoidance,
                        true
                    ) === true;
                }
                if (success) callbacks.refreshJointControlDisplay?.();
            } else if (!appState.dualArmMode) {
                success = vis.updateSingleArmJointsWithLBFGS?.(dqArray) === true;
            } else if (robotChoice === 'robot1') {
                success = vis.updateRobot1JointsWithLBFGS?.(dqArray) === true;
            } else if (robotChoice === 'robot2') {
                success = vis.updateRobot2JointsWithLBFGS?.(dqArray) === true;
            } else if (robotChoice === 'absolute') {
                const allJointAngles = [...(appState.jointAngles1 || []), ...(appState.jointAngles2 || [])];
                success = vis.updateDualArmJointsFromAbsolutePoseTarget?.(dqArray, allJointAngles) === true;
            } else if (robotChoice === 'relative') {
                const allJointAngles = [...(appState.jointAngles1 || []), ...(appState.jointAngles2 || [])];
                success = vis.updateDualArmJointsFromRelativePoseTarget?.(dqArray, allJointAngles) === true;
            } else {
                // 兜底：防止误选 absolute/relative 时错误调用 robot2
                callbacks.addROS2LogEntry?.('当前逆运动学求解仅支持机器人1/机器人2', 'warn');
                if (ikStatus) {
                    ikStatus.textContent = '当前逆运动学求解仅支持机器人1/机器人2';
                    ikStatus.style.color = '#f59e0b';
                }
                return;
            }
            if (!success) {
                callbacks.addROS2LogEntry?.('逆运动学求解失败：L-BFGS 未收敛', 'warn');
                if (ikStatus) {
                    ikStatus.textContent = '逆运动学求解失败：L-BFGS 未收敛';
                    ikStatus.style.color = '#f87171';
                }
                return;
            }
            callbacks.updateJointSlidersFromAngles?.();
            callbacks.updateDualArmJointSlidersFromAngles?.();
            const label = robotChoice === 'robot1'
                ? (appState.humanoidMode ? '左臂' : '机器人1')
                : robotChoice === 'robot2'
                    ? (appState.humanoidMode ? '右臂' : '机器人2')
                    : robotChoice === 'absolute'
                        ? '绝对位姿（保持相对不变）'
                        : robotChoice === 'relative'
                            ? '相对位姿（保持绝对不变）'
                            : robotChoice;
            callbacks.addROS2LogEntry?.(`逆运动学求解成功，已更新 ${label}`, 'info');
            if (ikStatus) {
                ikStatus.textContent = `逆运动学求解成功，已更新 ${label}`;
                ikStatus.style.color = '#10b981';
            }
        } catch (error) {
            console.error('逆运动学求解失败:', error);
            callbacks.addROS2LogEntry?.(`逆运动学失败：${error.message}`, 'error');
            if (ikStatus) {
                ikStatus.textContent = `逆运动学失败：${error.message}`;
                ikStatus.style.color = '#f87171';
            }
        }
    });

    updateIKRobotSelection();
    updateIkSolveAvailability();
    if (ikSelect) {
        ikSelect.addEventListener('change', () => {
            updateIkSolveAvailability();
        });
    }

    if (copyDqBtn) {
        copyDqBtn.addEventListener('click', async () => {
            const robotChoice = document.getElementById('ik-robot-select')?.value || 'robot1';
            const dqArray = getCurrentRobotDQ(robotChoice);
            if (!dqArray) {
                callbacks.addROS2LogEntry?.('无法获取当前姿态的双四元数', 'warn');
                if (ikStatus) {
                    ikStatus.textContent = '无法获取当前姿态的双四元数';
                    ikStatus.style.color = '#f87171';
                }
                return;
            }

            const formatted = dqArray.map((value) => {
                const rounded = Number(value.toFixed(6));
                return Number.isFinite(rounded) ? rounded : 0;
            }).join(',');

            try {
                if (navigator?.clipboard?.writeText) {
                    await navigator.clipboard.writeText(formatted);
                } else {
                    const tempInput = document.createElement('textarea');
                    tempInput.value = formatted;
                    document.body.appendChild(tempInput);
                    tempInput.select();
                    document.execCommand('copy');
                    document.body.removeChild(tempInput);
                }
                callbacks.addROS2LogEntry?.(`已复制双四元数：${formatted}`, 'info');
                if (ikStatus) {
                    ikStatus.textContent = '已复制当前姿态的双四元数';
                    ikStatus.style.color = '#10b981';
                }
            } catch (error) {
                console.error('复制双四元数失败:', error);
                callbacks.addROS2LogEntry?.(`复制双四元数失败：${error.message}`, 'error');
                if (ikStatus) {
                    ikStatus.textContent = `复制失败：${error.message}`;
                    ikStatus.style.color = '#f87171';
                }
            }
        });
    }
}

// ──────────────────────────────────────────────────────────────
// parseDualQuaternionInput (local helper used by setupInverseKinematicsControls)
// ──────────────────────────────────────────────────────────────

/**
 * 解析双四元数输入（8个数）
 * @param {string} text
 * @returns {number[]|null}
 */
function parseDualQuaternionInput(text) {
    if (!text) {
        return null;
    }

    const parts = text
        .trim()
        .split(/[\s,]+/)
        .map(value => Number(value))
        .filter((value) => !Number.isNaN(value));

    if (parts.length !== 8) {
        return null;
    }
    return parts;
}

// ──────────────────────────────────────────────────────────────
// 回调注册
// ──────────────────────────────────────────────────────────────

export function registerIKControllerCallbacks() {
    callbacks.updateIKRobotSelection = updateIKRobotSelection;
    callbacks.setupInverseKinematicsControls = setupInverseKinematicsControls;
}
