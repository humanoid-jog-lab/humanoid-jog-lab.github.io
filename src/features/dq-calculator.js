/**
 * DQ 计算器面板模块
 * - setupDQCalculatorPanel: 设置计算器面板 UI 交互
 * - parseDualQuaternionInput: 解析 DQ 输入字符串
 * - updateDQCalculatorRobotSelection: 更新计算器机器人选择器
 */

import { appState, callbacks } from '../core/state.js';
import { getDQModule } from '../dqrobotics.js';
import { getCurrentRobotDQ, formatDualQuaternion } from './ik-controller.js';
import { onLanguageChange, t } from '../i18n.js';

let dqCalculatorLanguageBound = false;

// ──────────────────────────────────────────────────────────────
// parseDualQuaternionInput
// ──────────────────────────────────────────────────────────────

/**
 * 解析双四元数输入（8个数）
 * @param {string} text
 * @returns {number[]|null}
 */
export function parseDualQuaternionInput(text) {
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
// updateDQCalculatorRobotSelection
// ──────────────────────────────────────────────────────────────

export function updateDQCalculatorRobotSelection() {
    const select = document.getElementById('dq-calculator-robot');
    if (!select) {
        return;
    }

    const prevValue = select.value;
    select.innerHTML = '';
    const arm1LabelCalc = appState.humanoidMode ? t('Left Arm') : (appState.dualArmMode ? t('Robot 1') : t('Robot'));
    const opt1 = document.createElement('option');
    opt1.value = 'robot1';
    opt1.textContent = arm1LabelCalc;
    select.appendChild(opt1);

    const showDualOpts = appState.dualArmMode || appState.humanoidMode;
    if (showDualOpts) {
        const opt2 = document.createElement('option');
        opt2.value = 'robot2';
        opt2.textContent = appState.humanoidMode ? t('Right Arm') : t('Robot 2');
        select.appendChild(opt2);

        const optAbs = document.createElement('option');
        optAbs.value = 'absolute';
        optAbs.textContent = t('Absolute Pose');
        select.appendChild(optAbs);

        const optRel = document.createElement('option');
        optRel.value = 'relative';
        optRel.textContent = t('Relative Pose');
        select.appendChild(optRel);

        select.disabled = false;
    } else {
        select.disabled = true;
    }

    if (prevValue) {
        const hasPrev = Array.from(select.options).some(opt => opt.value === prevValue);
        if (hasPrev) {
            select.value = prevValue;
        }
    }
}

// ──────────────────────────────────────────────────────────────
// setupDQCalculatorPanel
// ──────────────────────────────────────────────────────────────

export function setupDQCalculatorPanel() {
    const loadBtn = document.getElementById('dq-calculator-load-current');
    const loadCurrentQuaternionBtn = document.getElementById('dq-calculator-load-current-quaternion');
    const loadCurrentPositionBtn = document.getElementById('dq-calculator-load-current-position');
    const applyBtn = document.getElementById('dq-calculator-apply');
    const copyInputBtn = document.getElementById('dq-calculator-copy-input');
    const copyOutputBtn = document.getElementById('dq-calculator-copy-output');
    const statusEl = document.getElementById('dq-calculator-status');
    const inputEl = document.getElementById('dq-calculator-input');
    const outputEl = document.getElementById('dq-calculator-output');

    if (!loadBtn && !applyBtn && !copyInputBtn && !copyOutputBtn) {
        return;
    }

    updateDQCalculatorRobotSelection();

    if (loadBtn) {
        loadBtn.addEventListener('click', () => {
            const robotChoice = document.getElementById('dq-calculator-robot')?.value || 'robot1';
            const dqArray = getCurrentRobotDQ(robotChoice);
            if (!dqArray) {
                setDQCalculatorStatus(t('Unable to get the current pose dual quaternion'), 'error');
                return;
            }
            if (inputEl) {
                inputEl.value = formatDualQuaternion(dqArray);
            }
            setDQCalculatorStatus(t('Loaded current pose dual quaternion'), 'success');
        });
    }

    if (loadCurrentQuaternionBtn) {
        loadCurrentQuaternionBtn.addEventListener('click', () => {
            const currentPose = getCurrentSelectedRobotPose();
            if (!currentPose) {
                return;
            }

            const quatW = document.getElementById('dq-calculator-quat-w');
            const quatX = document.getElementById('dq-calculator-quat-x');
            const quatY = document.getElementById('dq-calculator-quat-y');
            const quatZ = document.getElementById('dq-calculator-quat-z');

            if (!quatW || !quatX || !quatY || !quatZ) {
                setDQCalculatorQuatStatus(t('Quaternion input fields are unavailable'), 'error');
                return;
            }

            quatW.value = formatScalarInput(currentPose.rotation[0]);
            quatX.value = formatScalarInput(currentPose.rotation[1]);
            quatY.value = formatScalarInput(currentPose.rotation[2]);
            quatZ.value = formatScalarInput(currentPose.rotation[3]);
            setDQCalculatorQuatStatus(t('Loaded the current quaternion into the quaternion inputs'), 'success');
        });
    }

    if (loadCurrentPositionBtn) {
        loadCurrentPositionBtn.addEventListener('click', () => {
            const currentPose = getCurrentSelectedRobotPose();
            if (!currentPose) {
                return;
            }

            const posX = document.getElementById('dq-calculator-pos-x');
            const posY = document.getElementById('dq-calculator-pos-y');
            const posZ = document.getElementById('dq-calculator-pos-z');

            if (!posX || !posY || !posZ) {
                setDQCalculatorQuatStatus(t('Translation input fields are unavailable'), 'error');
                return;
            }

            posX.value = formatScalarInput(currentPose.translation[1]);
            posY.value = formatScalarInput(currentPose.translation[2]);
            posZ.value = formatScalarInput(currentPose.translation[3]);
            setDQCalculatorQuatStatus(t('Loaded the current position into the translation inputs'), 'success');
        });
    }

    if (copyInputBtn && inputEl) {
        copyInputBtn.addEventListener('click', async () => {
            await copyTextToClipboard(inputEl.value || '', t('Input dual quaternion'));
        });
    }

    if (copyOutputBtn && outputEl) {
        copyOutputBtn.addEventListener('click', async () => {
            await copyTextToClipboard(outputEl.value || '', t('Output dual quaternion'));
        });
    }

    if (applyBtn) {
        applyBtn.addEventListener('click', () => {
            if (!inputEl || !outputEl) {
                return;
            }

            const dqArray = parseDualQuaternionInput(inputEl.value || '');
            if (!dqArray) {
                setDQCalculatorStatus(t('Please enter 8 dual quaternion elements'), 'error');
                return;
            }

            const tx = parseFloat(document.getElementById('dq-calculator-translate-x')?.value || '0') || 0;
            const ty = parseFloat(document.getElementById('dq-calculator-translate-y')?.value || '0') || 0;
            const tz = parseFloat(document.getElementById('dq-calculator-translate-z')?.value || '0') || 0;
            const mode = document.querySelector('input[name="dq-calculator-mode"]:checked')?.value || 'left';

            if (!window.DQModule || !window.DQModule.DQWrapper) {
                setDQCalculatorStatus(t('DQ module is not loaded; cannot calculate'), 'error');
                return;
            }

            try {
                const dqWrapper = window.DQModule.DQWrapper;
                const dqInput = dqWrapper.createFromArray(dqArray);
                const dqTranslation = dqWrapper.translation(tx, ty, tz);
                const { multiply } = dqWrapper;

                const dqResult = mode === 'left'
                    ? multiply(dqTranslation, dqInput)
                    : multiply(dqInput, dqTranslation);

                const dqResultArray = dqWrapper.toArray(dqResult);
                const formatted = formatDualQuaternion(dqResultArray);
                outputEl.value = formatted;
                setDQCalculatorStatus(t('Calculation complete, result ready to copy'), 'success');
            } catch (error) {
                console.error('DQ 计算失败:', error);
                setDQCalculatorStatus(t('Calculation failed: {{message}}', { message: error.message }), 'error');
            }
        });
    }

    // 从四元数和平移创建DQ的按钮
    const createFromQuatBtn = document.getElementById('dq-calculator-create-from-quat');
    const quatStatusEl = document.getElementById('dq-calculator-quat-status');

    if (createFromQuatBtn) {
        createFromQuatBtn.addEventListener('click', () => {
            if (!outputEl) {
                return;
            }

            const qw = parseFloat(document.getElementById('dq-calculator-quat-w')?.value || '1') || 1;
            const qx = parseFloat(document.getElementById('dq-calculator-quat-x')?.value || '0') || 0;
            const qy = parseFloat(document.getElementById('dq-calculator-quat-y')?.value || '0') || 0;
            const qz = parseFloat(document.getElementById('dq-calculator-quat-z')?.value || '0') || 0;
            const tx = parseFloat(document.getElementById('dq-calculator-pos-x')?.value || '0') || 0;
            const ty = parseFloat(document.getElementById('dq-calculator-pos-y')?.value || '0') || 0;
            const tz = parseFloat(document.getElementById('dq-calculator-pos-z')?.value || '0') || 0;

            if (!window.DQModule || !window.DQModule.DQWrapper) {
                setDQCalculatorQuatStatus(t('DQ module is not loaded; cannot calculate'), 'error');
                return;
            }

            try {
                const dqWrapper = window.DQModule.DQWrapper;

                // 尝试使用新函数，如果不存在则使用组合方式
                let dqResult;
                if (dqWrapper.fromQuaternionAndTranslation) {
                    // 使用新的专用函数
                    dqResult = dqWrapper.fromQuaternionAndTranslation(qw, qx, qy, qz, tx, ty, tz);
                } else {
                    // 后备方案：使用现有的函数组合
                    // 从四元数创建旋转DQ（四元数需要归一化）
                    const quatNorm = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
                    if (quatNorm < 1e-10) {
                        throw new Error(t('Quaternion cannot be zero'));
                    }
                    const normalizedQw = qw / quatNorm;
                    const normalizedQx = qx / quatNorm;
                    const normalizedQy = qy / quatNorm;
                    const normalizedQz = qz / quatNorm;

                    // 创建旋转DQ（实部是归一化的四元数，对偶部为0）
                    const dqRotation = dqWrapper.createFromArray([normalizedQw, normalizedQx, normalizedQy, normalizedQz, 0, 0, 0, 0]);
                    // 创建平移DQ
                    const dqTranslation = dqWrapper.translation(tx, ty, tz);
                    // DQ = translation * rotation（平移在左，旋转在右）
                    dqResult = dqWrapper.multiply(dqTranslation, dqRotation);
                }

                const dqResultArray = dqWrapper.toArray(dqResult);
                const formatted = formatDualQuaternion(dqResultArray);
                outputEl.value = formatted;

                // 验证：从结果中提取平移，检查是否与输入一致
                try {
                    const dqResultObj = dqWrapper.createFromArray(dqResultArray);
                    const extractedTranslation = dqWrapper.getTranslation(dqResultObj);
                    const extractedX = extractedTranslation[1];
                    const extractedY = extractedTranslation[2];
                    const extractedZ = extractedTranslation[3];

                    const diffX = Math.abs(extractedX - tx);
                    const diffY = Math.abs(extractedY - ty);
                    const diffZ = Math.abs(extractedZ - tz);
                    const maxDiff = Math.max(diffX, diffY, diffZ);

                    if (maxDiff < 1e-6) {
                        setDQCalculatorQuatStatus(t('Calculation complete, verification passed (translation matches)'), 'success');
                    } else {
                        setDQCalculatorQuatStatus(t('Calculation complete, but translation verification failed (difference: {{difference}})', { difference: maxDiff.toFixed(6) }), 'error');
                        console.log('输入平移:', tx, ty, tz);
                        console.log('提取平移:', extractedX, extractedY, extractedZ);
                    }
                } catch (verifyError) {
                    setDQCalculatorQuatStatus(t('Calculation complete, but verification failed'), 'error');
                    console.error('验证失败:', verifyError);
                }
            } catch (error) {
                console.error('从四元数创建DQ失败:', error);
                setDQCalculatorQuatStatus(t('Calculation failed: {{message}}', { message: error.message }), 'error');
            }
        });
    }

    if (!dqCalculatorLanguageBound) {
        dqCalculatorLanguageBound = true;
        onLanguageChange(() => {
            updateDQCalculatorRobotSelection();
        });
    }

    function setDQCalculatorStatus(message, type) {
        if (!statusEl) return;
        statusEl.textContent = message;
        statusEl.style.color = type === 'success' ? '#10b981' : '#f87171';
    }

    function setDQCalculatorQuatStatus(message, type) {
        if (!quatStatusEl) return;
        quatStatusEl.textContent = message;
        quatStatusEl.style.color = type === 'success' ? '#10b981' : '#f87171';
    }

    function getCurrentSelectedRobotPose() {
        const robotChoice = document.getElementById('dq-calculator-robot')?.value || 'robot1';
        const dqArray = getCurrentRobotDQ(robotChoice);
        if (!dqArray) {
            setDQCalculatorQuatStatus(t('Unable to get the current pose dual quaternion'), 'error');
            return null;
        }

        const dqModule = getDQModule() || window.DQModule;
        if (!dqModule?.DQWrapper) {
            setDQCalculatorQuatStatus(t('DQ module is not loaded; cannot calculate'), 'error');
            return null;
        }

        try {
            const dqWrapper = dqModule.DQWrapper;
            const dqObject = dqWrapper.createFromArray(dqArray);
            return {
                rotation: dqWrapper.getRotation(dqObject),
                translation: dqWrapper.getTranslation(dqObject)
            };
        } catch (error) {
            console.error('提取当前姿态失败:', error);
            setDQCalculatorQuatStatus(t('Failed to extract the current pose: {{message}}', { message: error.message }), 'error');
            return null;
        }
    }

    function formatScalarInput(value) {
        const rounded = Number(Number(value || 0).toFixed(6));
        return Number.isFinite(rounded) ? String(rounded) : '0';
    }

    async function copyTextToClipboard(text, label) {
        if (!text) {
            setDQCalculatorStatus(t('Nothing to copy for {{label}}', { label }), 'error');
            return;
        }
        try {
            if (navigator?.clipboard?.writeText) {
                await navigator.clipboard.writeText(text);
            } else {
                const tempInput = document.createElement('textarea');
                tempInput.value = text;
                document.body.appendChild(tempInput);
                tempInput.select();
                document.execCommand('copy');
                document.body.removeChild(tempInput);
            }
            setDQCalculatorStatus(t('Copied {{label}}', { label }), 'success');
        } catch (error) {
            console.error('复制失败:', error);
            setDQCalculatorStatus(t('Copy failed: {{message}}', { message: error.message }), 'error');
        }
    }
}

// ──────────────────────────────────────────────────────────────
// 回调注册
// ──────────────────────────────────────────────────────────────

export function registerDQCalculatorCallbacks() {
    callbacks.updateDQCalculatorRobotSelection = updateDQCalculatorRobotSelection;
    callbacks.setupDQCalculatorPanel = setupDQCalculatorPanel;
}
