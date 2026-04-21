/**
 * IK 控制器模块
 * - formatDualQuaternion: 格式化 DQ 数组为字符串
 * - updateIKRobotSelection: 更新 IK 机器人选择器
 * - getCurrentRobotDQ: 获取当前机器人 DQ
 * - setupInverseKinematicsControls: 设置逆运动学控制面板
 */

import { appState, callbacks } from '../core/state.js';
import { onLanguageChange, t } from '../i18n.js';

let ikLanguageBound = false;
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
    const arm1Label = appState.humanoidMode ? t('Left Arm') : (appState.dualArmMode ? t('Robot 1') : t('Robot'));
    const opt1 = document.createElement('option');
    opt1.value = 'robot1';
    opt1.textContent = arm1Label;
    ikSelect.appendChild(opt1);

    const showDualOptions = appState.dualArmMode || appState.humanoidMode;
    if (showDualOptions) {
        const opt2 = document.createElement('option');
        opt2.value = 'robot2';
        opt2.textContent = appState.humanoidMode ? t('Right Arm') : t('Robot 2');
        ikSelect.appendChild(opt2);

        const optAbs = document.createElement('option');
        optAbs.value = 'absolute';
        optAbs.textContent = t('Absolute Pose');
        ikSelect.appendChild(optAbs);

        const optRel = document.createElement('option');
        optRel.value = 'relative';
        optRel.textContent = t('Relative Pose');
        ikSelect.appendChild(optRel);

        ikSelect.disabled = false;
        if (ikStatus) {
            ikStatus.textContent = appState.humanoidMode
                ? t('Humanoid mode: left arm / right arm / absolute pose / relative pose')
                : t('Dual-arm mode: choose robot 1 / robot 2 / absolute pose / relative pose');
        }
    } else {
        ikSelect.disabled = true;
        if (ikStatus) {
            ikStatus.textContent = t('Single-arm mode: only control robot 1');
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
            const dqWrapper = window.DQModule.DQWrapper;
            const baseTrans = dqWrapper.translation(
                appState.humanoidBasePose?.x || 0,
                appState.humanoidBasePose?.y || 0,
                0
            );
            const baseRot = dqWrapper.rotation(
                appState.humanoidBasePose?.yaw || 0,
                0,
                0,
                1
            );
            const baseObj = dqWrapper.multiply(baseTrans, baseRot);

            if (robotChoice === 'robot1') {
                const local = appState.humanoidModel.fkmEffector(appState.humanoidQ, 1);
                return dqWrapper.toArray(
                    dqWrapper.multiply(baseObj, dqWrapper.createFromArray(local))
                );
            }
            if (robotChoice === 'robot2') {
                const local = appState.humanoidModel.fkmEffector(appState.humanoidQ, 2);
                return dqWrapper.toArray(
                    dqWrapper.multiply(baseObj, dqWrapper.createFromArray(local))
                );
            }
            if (robotChoice === 'absolute') {
                if (!appState.dualArmSystem || typeof appState.dualArmSystem.getAbsolutePose !== 'function') {
                    return null;
                }
                const local = appState.dualArmSystem.getAbsolutePose(appState.humanoidQ);
                return dqWrapper.toArray(
                    dqWrapper.multiply(baseObj, dqWrapper.createFromArray(local))
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
            ikStatus.textContent = t('Single-arm mode: update current robot joints with Jacobian pseudoinverse');
            ikStatus.style.color = '#94a3b8';
            return;
        }
        const choice = ikSelect?.value || 'robot1';
        if (choice === 'absolute') {
            ikStatus.textContent = appState.humanoidMode
                ? t('Humanoid absolute pose: solve the absolute pose while keeping the relative pose unchanged')
                : t('Dual-arm absolute pose: solve the absolute pose while keeping the relative pose unchanged');
            ikStatus.style.color = '#94a3b8';
            return;
        }
        if (choice === 'relative') {
            ikStatus.textContent = appState.humanoidMode
                ? t('Humanoid relative pose: solve the relative pose while keeping the absolute pose unchanged')
                : t('Dual-arm relative pose: solve the relative pose while keeping the absolute pose unchanged');
            ikStatus.style.color = '#94a3b8';
            return;
        }
        ikStatus.textContent = appState.humanoidMode
            ? t('Humanoid single-arm solve: update only the left or right arm joints')
            : t('Dual-arm single-arm solve: update only robot 1 or robot 2 joints');
        ikStatus.style.color = '#94a3b8';
    };

    ikSolveBtn.addEventListener('click', () => {
        const dqText = document.getElementById('ik-dq-input')?.value || '';
        const dqArray = parseDualQuaternionInput(dqText);
        if (!dqArray) {
            callbacks.addROS2LogEntry?.(t('Please enter 8 dual quaternion elements (comma or space separated)'), 'warn');
            if (ikStatus) {
                ikStatus.textContent = t('Please enter 8 dual quaternion elements (comma or space separated)');
                ikStatus.style.color = '#f87171';
            }
            return;
        }

        const robotChoice = document.getElementById('ik-robot-select')?.value || 'robot1';
        const vis = appState.visualizer;
        if (!vis) {
            callbacks.addROS2LogEntry?.(t('Visualizer is not initialized; inverse kinematics unavailable'), 'error');
            if (ikStatus) {
                ikStatus.textContent = t('Visualizer is not initialized; inverse kinematics unavailable');
                ikStatus.style.color = '#f87171';
            }
            return;
        }

        try {
            vis.lastIKSolveStatus = null;
            let success = false;
            if (appState.humanoidMode && (robotChoice === 'absolute' || robotChoice === 'relative')) {
                const allQ = appState.humanoidQ;
                if (!allQ || allQ.length === 0) {
                    if (ikStatus) { ikStatus.textContent = t('Humanoid joints are not initialized'); ikStatus.style.color = '#f87171'; }
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
            } else if (appState.humanoidMode && robotChoice === 'robot1') {
                success = vis.updateHumanoidSingleArmPoseTargetLBFGS?.(
                    dqArray,
                    appState.humanoidQ,
                    1,
                    false
                ) === true;
            } else if (appState.humanoidMode && robotChoice === 'robot2') {
                success = vis.updateHumanoidSingleArmPoseTargetLBFGS?.(
                    dqArray,
                    appState.humanoidQ,
                    2,
                    false
                ) === true;
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
                callbacks.addROS2LogEntry?.(t('The current inverse-kinematics solve supports only robot 1 / robot 2'), 'warn');
                if (ikStatus) {
                    ikStatus.textContent = t('The current inverse-kinematics solve supports only robot 1 / robot 2');
                    ikStatus.style.color = '#f59e0b';
                }
                return;
            }
            if (!success) {
                const solveInfo = vis.lastIKSolveStatus;
                let detail = t('Inverse kinematics failed: target accuracy not reached');
                if (solveInfo?.solver === 'lbfgs') {
                    detail = `Inverse kinematics failed: target accuracy not reached (reason=${solveInfo.reason || 'unknown'}, residual=${Number.isFinite(solveInfo.finalResidual) ? solveInfo.finalResidual.toExponential(2) : 'n/a'})`;
                } else if (solveInfo?.solver === 'vfi') {
                    detail = `Inverse kinematics failed: ${solveInfo.message || 'VFI solve failed'} (reason=${solveInfo.reason || 'unknown'}, residual=${Number.isFinite(solveInfo.finalResidual) ? solveInfo.finalResidual.toExponential(2) : 'n/a'}${typeof solveInfo.qpConverged === 'boolean' ? `, qp=${solveInfo.qpConverged ? 'yes' : 'no'}` : ''}${solveInfo.qpMessage ? `, qp_message=${solveInfo.qpMessage}` : ''})`;
                } else if (typeof solveInfo?.message === 'string' && solveInfo.message.trim()) {
                    detail = `Inverse kinematics failed: ${solveInfo.message.trim()}`;
                } else {
                    const backend = appState.humanoidMode
                        ? ((window.humanoidJogSolverMode || vis.humanoidJogSolverMode || 'unknown').toString())
                        : (appState.dualArmMode ? 'dual-arm' : 'single-arm');
                    detail = `Inverse kinematics failed: target accuracy not reached (mode=${robotChoice}, humanoid=${appState.humanoidMode ? 'yes' : 'no'}, backend=${backend}, diagnostic=missing)`;
                }
                callbacks.addROS2LogEntry?.(detail, 'warn');
                if (ikStatus) {
                    ikStatus.textContent = detail;
                    ikStatus.style.color = '#f87171';
                }
                return;
            }
            callbacks.updateJointSlidersFromAngles?.();
            callbacks.updateDualArmJointSlidersFromAngles?.();
            const label = robotChoice === 'robot1'
                ? (appState.humanoidMode ? t('Left Arm') : t('Robot 1'))
                : robotChoice === 'robot2'
                    ? (appState.humanoidMode ? t('Right Arm') : t('Robot 2'))
                    : robotChoice === 'absolute'
                        ? t('Absolute Pose (keep relative pose unchanged)')
                        : robotChoice === 'relative'
                            ? t('Relative Pose (keep absolute pose unchanged)')
                            : robotChoice;
            callbacks.addROS2LogEntry?.(t('Inverse kinematics succeeded; updated {{label}}', { label }), 'info');
            if (ikStatus) {
                ikStatus.textContent = t('Inverse kinematics succeeded; updated {{label}}', { label });
                ikStatus.style.color = '#10b981';
            }
        } catch (error) {
            console.error('逆运动学求解失败:', error);
            callbacks.addROS2LogEntry?.(t('Inverse kinematics failed: {{message}}', { message: error.message }), 'error');
            if (ikStatus) {
                ikStatus.textContent = t('Inverse kinematics failed: {{message}}', { message: error.message });
                ikStatus.style.color = '#f87171';
            }
        }
    });

    updateIKRobotSelection();
    updateIkSolveAvailability();
    if (!ikLanguageBound) {
        ikLanguageBound = true;
        onLanguageChange(() => {
            updateIKRobotSelection();
            updateIkSolveAvailability();
        });
    }
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
                callbacks.addROS2LogEntry?.(t('Unable to get the current pose dual quaternion'), 'warn');
                if (ikStatus) {
                    ikStatus.textContent = t('Unable to get the current pose dual quaternion');
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
                callbacks.addROS2LogEntry?.(t('Copied dual quaternion: {{value}}', { value: formatted }), 'info');
                if (ikStatus) {
                    ikStatus.textContent = t('Copied the current pose dual quaternion');
                    ikStatus.style.color = '#10b981';
                }
            } catch (error) {
                console.error('复制双四元数失败:', error);
                callbacks.addROS2LogEntry?.(t('Failed to copy dual quaternion: {{message}}', { message: error.message }), 'error');
                if (ikStatus) {
                    ikStatus.textContent = t('Copy failed: {{message}}', { message: error.message });
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
