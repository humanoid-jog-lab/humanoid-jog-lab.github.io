/**
 * DQ 计算器面板模块
 * - setupDQCalculatorPanel: 设置计算器面板 UI 交互
 * - parseDualQuaternionInput: 解析 DQ 输入字符串
 * - updateDQCalculatorRobotSelection: 更新计算器机器人选择器
 */

import { appState, callbacks } from '../core/state.js';
import { getDQModule } from '../dqrobotics.js';
import { getCurrentRobotDQ, formatDualQuaternion } from './ik-controller.js';

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
    const arm1LabelCalc = appState.humanoidMode ? '左臂' : (appState.dualArmMode ? '机器人1' : '机器人');
    const opt1 = document.createElement('option');
    opt1.value = 'robot1';
    opt1.textContent = arm1LabelCalc;
    select.appendChild(opt1);

    const showDualOpts = appState.dualArmMode || appState.humanoidMode;
    if (showDualOpts) {
        const opt2 = document.createElement('option');
        opt2.value = 'robot2';
        opt2.textContent = appState.humanoidMode ? '右臂' : '机器人2';
        select.appendChild(opt2);

        const optAbs = document.createElement('option');
        optAbs.value = 'absolute';
        optAbs.textContent = '绝对位姿';
        select.appendChild(optAbs);

        const optRel = document.createElement('option');
        optRel.value = 'relative';
        optRel.textContent = '相对位姿';
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
                setDQCalculatorStatus('无法获取当前姿态的双四元数', 'error');
                return;
            }
            if (inputEl) {
                inputEl.value = formatDualQuaternion(dqArray);
            }
            setDQCalculatorStatus('已加载当前姿态的双四元数', 'success');
        });
    }

    if (copyInputBtn && inputEl) {
        copyInputBtn.addEventListener('click', async () => {
            await copyTextToClipboard(inputEl.value || '', '输入双四元数');
        });
    }

    if (copyOutputBtn && outputEl) {
        copyOutputBtn.addEventListener('click', async () => {
            await copyTextToClipboard(outputEl.value || '', '输出双四元数');
        });
    }

    if (applyBtn) {
        applyBtn.addEventListener('click', () => {
            if (!inputEl || !outputEl) {
                return;
            }

            const dqArray = parseDualQuaternionInput(inputEl.value || '');
            if (!dqArray) {
                setDQCalculatorStatus('请输入 8 个双四元数元素', 'error');
                return;
            }

            const tx = parseFloat(document.getElementById('dq-calculator-translate-x')?.value || '0') || 0;
            const ty = parseFloat(document.getElementById('dq-calculator-translate-y')?.value || '0') || 0;
            const tz = parseFloat(document.getElementById('dq-calculator-translate-z')?.value || '0') || 0;
            const mode = document.querySelector('input[name="dq-calculator-mode"]:checked')?.value || 'left';

            if (!window.DQModule || !window.DQModule.DQWrapper) {
                setDQCalculatorStatus('DQ 模块未加载，无法计算', 'error');
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
                setDQCalculatorStatus('计算完成，可复制结果', 'success');
            } catch (error) {
                console.error('DQ 计算失败:', error);
                setDQCalculatorStatus(`计算失败：${error.message}`, 'error');
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
                setDQCalculatorQuatStatus('DQ 模块未加载，无法计算', 'error');
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
                        throw new Error('四元数不能为零');
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
                        setDQCalculatorQuatStatus('计算完成，验证通过（平移一致）', 'success');
                    } else {
                        setDQCalculatorQuatStatus(`计算完成，但平移验证失败（差异: ${maxDiff.toFixed(6)}）`, 'error');
                        console.log('输入平移:', tx, ty, tz);
                        console.log('提取平移:', extractedX, extractedY, extractedZ);
                    }
                } catch (verifyError) {
                    setDQCalculatorQuatStatus('计算完成，但验证失败', 'error');
                    console.error('验证失败:', verifyError);
                }
            } catch (error) {
                console.error('从四元数创建DQ失败:', error);
                setDQCalculatorQuatStatus(`计算失败：${error.message}`, 'error');
            }
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

    async function copyTextToClipboard(text, label) {
        if (!text) {
            setDQCalculatorStatus(`没有可复制的${label}`, 'error');
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
            setDQCalculatorStatus(`已复制${label}`, 'success');
        } catch (error) {
            console.error('复制失败:', error);
            setDQCalculatorStatus(`复制失败：${error.message}`, 'error');
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
