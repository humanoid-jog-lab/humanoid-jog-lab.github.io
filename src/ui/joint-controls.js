/**
 * 关节控制滑块/输入框 UI 模块
 * 负责创建和更新单臂、双臂、人形机器人的关节控制界面
 */

import { appState, callbacks } from '../core/state.js';
import { onLanguageChange, t } from '../i18n.js';
import { angleToDisplay, displayToAngle, getSliderPrecision, getInputPrecision, displayStep, getAnglePlaceholder, formatDisplayValue, formatDisplayAngle, getActiveJointConfig } from '../utils/math-utils.js';

let jointControlsLanguageBound = false;

function getJointFallbackName(index) {
    return t('Joint {{number}}', { number: index + 1 });
}

function getRobotFallbackName(number) {
    return t('Robot {{number}}', { number });
}

/**
 * 获取关节限位
 */
export function getJointLimits(jointName, urdfRobot, config) {
    let lower = -Math.PI;  // 默认下限 -180度
    let upper = Math.PI;   // 默认上限 180度
    
    // 优先从配置文件读取
    if (config?.joint_limits && config.joint_limits[jointName]) {
        const limits = config.joint_limits[jointName];
        if (Array.isArray(limits) && limits.length >= 2) {
            lower = limits[0];
            upper = limits[1];
            return { lower, upper };
        }
    }
    
    // 从URDF读取（不在此处打 log，避免每次 clamp 都刷屏；首次限位可在加载时统一打印）
    if (urdfRobot && urdfRobot.joints && urdfRobot.joints[jointName]) {
        const joint = urdfRobot.joints[jointName];
        
        // 尝试多种可能的属性名
        let limit = joint.limit || joint.limits || null;
        
        if (limit) {
            // 检查不同的属性结构
            if (limit.lower !== undefined && limit.upper !== undefined) {
                lower = parseFloat(limit.lower);
                upper = parseFloat(limit.upper);
            } else if (limit.min !== undefined && limit.max !== undefined) {
                lower = parseFloat(limit.min);
                upper = parseFloat(limit.max);
            }
        } else {
            // 尝试直接访问属性（urdf-loader可能将limit属性直接挂在joint上）
            if (joint.lower !== undefined && joint.upper !== undefined) {
                lower = parseFloat(joint.lower);
                upper = parseFloat(joint.upper);
            }
        }
    }
    
    return { lower, upper };
}

/**
 * 限制角度在关节限位范围内
 */
export function clampJointAngles(angles, jointNames, urdfRobot, config) {
    if (!angles || !jointNames || jointNames.length !== angles.length) {
        return angles;
    }
    
    const clamped = angles.map((angle, i) => {
        const jointName = jointNames[i];
        if (!jointName) return angle;
        
        const limits = getJointLimits(jointName, urdfRobot, config);
        let clampedAngle = angle;
        
        if (clampedAngle < limits.lower) {
            console.warn(`  ⚠ 关节 ${jointName} 角度 ${angle.toFixed(4)} 低于下限 ${limits.lower.toFixed(4)}，已限制为下限`);
            clampedAngle = limits.lower;
        } else if (clampedAngle > limits.upper) {
            console.warn(`  ⚠ 关节 ${jointName} 角度 ${angle.toFixed(4)} 超过上限 ${limits.upper.toFixed(4)}，已限制为上限`);
            clampedAngle = limits.upper;
        }
        
        return clampedAngle;
    });
    
    return clamped;
}

/**
 * 设置关节输入框的事件处理（辅助函数）
 * @param {HTMLInputElement} input - 输入框元素
 * @param {string} sliderId - 对应的滑块ID
 * @param {number} lowerRad - 下限（弧度）
 * @param {number} upperRad - 上限（弧度）
 * @param {Function} onUpdate - 更新回调函数，接收角度（弧度）
 */
export function setupJointInput(input, sliderId, lowerRad, upperRad, onUpdate) {
    let inputTimeout = null;
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const displayLower = angleToDisplay(lowerRad);
    const displayUpper = angleToDisplay(upperRad);
    
    const clampDisplayValue = (value) => {
        if (!Number.isFinite(value)) {
            return displayLower;
        }
        return Math.max(displayLower, Math.min(displayUpper, value));
    };
    
    const commitValue = (displayValue) => {
        const clampedDisplay = clampDisplayValue(displayValue);
        const angleRad = displayToAngle(clampedDisplay);
        
        const slider = document.getElementById(sliderId);
        if (slider) {
            slider.value = formatDisplayValue(clampedDisplay, sliderPrecision);
        }
        
        input.value = formatDisplayValue(clampedDisplay, inputPrecision);
        onUpdate(angleRad);
    };
    
    input.addEventListener('input', (e) => {
        if (inputTimeout) {
            clearTimeout(inputTimeout);
        }

        inputTimeout = setTimeout(() => {
            const rawValue = e.target.value;

            // 允许用户输入中间状态，例如 "-" 或 "1."，这时不进行更新
            if (rawValue === '' || rawValue === '-' || rawValue === '.' || rawValue === '-.' || rawValue === '+') {
                return;
            }

            const displayValue = parseFloat(rawValue);
            if (Number.isNaN(displayValue)) {
                return;
            }

            commitValue(displayValue);
        }, 200);
    });
    
    input.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
            if (inputTimeout) {
                clearTimeout(inputTimeout);
            }
            const angleDeg = parseFloat(e.target.value);
            if (!isNaN(angleDeg)) {
                commitValue(angleDeg);
            }
            e.target.blur();
        }
    });
    
    input.addEventListener('blur', (e) => {
        const displayValue = parseFloat(e.target.value);
        if (Number.isNaN(displayValue)) {
            // 恢复当前值（从滑块获取）
            const slider = document.getElementById(sliderId);
            if (slider) {
                const currentDisplay = parseFloat(slider.value);
                e.target.value = formatDisplayValue(currentDisplay, inputPrecision);
            }
        } else {
            commitValue(displayValue);
        }
    });
}

/**
 * 创建关节控制滑块（单臂或双臂模式）
 */
export function createJointControls(numJoints, config = null) {
    const container = document.getElementById('joint-controls');
    container.innerHTML = '';
    
    // 如果是双臂模式，创建双臂关节控制
    if (appState.dualArmMode && config) {
        createDualArmJointControls(config);
        return;
    }
    
    // 获取URDF机器人对象（用于读取关节限位）
    const urdfRobot = appState.visualizer?.urdfRobot || null;
    
    // 单臂模式：创建单臂关节控制
    for (let i = 0; i < numJoints; i++) {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        
        const jointName = config?.joint_chain?.joints?.[i] || getJointFallbackName(i);
        
        // 获取关节限位
        const limits = getJointLimits(jointName, urdfRobot, config);
        const lowerRad = limits.lower;
        const upperRad = limits.upper;
        const sliderPrecision = getSliderPrecision();
        const inputPrecision = getInputPrecision();
        const step = displayStep().toString();
        
        // 创建标签行（关节名称 + 输入框）
        const label = document.createElement('div');
        label.className = 'joint-label';
        const nameSpan = document.createElement('span');
        nameSpan.textContent = jointName;
        label.appendChild(nameSpan);
        
        // 创建角度输入框
        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'joint-input';
        input.id = `joint-input-${i}`;
        input.step = step;
        input.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        input.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        input.placeholder = getAnglePlaceholder();
        input.value = formatDisplayAngle(appState.jointAngles[i] ?? 0);
        
        // 输入框事件处理
        setupJointInput(input, `joint-${i}`, lowerRad, upperRad, (angleRad) => {
            appState.jointAngles[i] = angleRad;
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            callbacks.updateRobotPose?.();
        });
        
        label.appendChild(input);
        
        // 创建滑块
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.id = `joint-${i}`;
        slider.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        slider.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        slider.step = step;
        slider.value = formatDisplayValue(angleToDisplay(appState.jointAngles[i] ?? 0), sliderPrecision);
        
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) {
                return;
            }
            const angleRad = displayToAngle(displayValue);
            appState.jointAngles[i] = angleRad;
            
            // 更新输入框
            input.value = formatDisplayValue(displayValue, inputPrecision);
            
            // 同步全局变量
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            callbacks.updateRobotPose?.();
        });
        
        jointDiv.appendChild(label);
        jointDiv.appendChild(slider);
        container.appendChild(jointDiv);
    }
    
    // 创建滑块后，用实际的关节角度更新滑块值和显示
    updateJointSlidersFromAngles();
}

/**
 * 从当前关节角度更新所有滑块的值和显示
 */
export function updateJointSlidersFromAngles() {
    if (!appState.jointAngles || appState.jointAngles.length === 0) return;
    
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const step = displayStep().toString();
    const placeholder = getAnglePlaceholder();
    
    appState.jointAngles.forEach((angle, i) => {
        const displayValue = angleToDisplay(angle);
        const slider = document.getElementById(`joint-${i}`);
        const input = document.getElementById(`joint-input-${i}`);
        
        if (slider) {
            slider.value = formatDisplayValue(displayValue, sliderPrecision);
            slider.step = step;
        }
        
        if (input) {
            input.value = formatDisplayValue(displayValue, inputPrecision);
            input.step = step;
            input.placeholder = placeholder;
        }
    });
}

/**
 * 创建双臂关节控制滑块
 */
export function createDualArmJointControls(config) {
    const container = document.getElementById('joint-controls');
    // 清除旧的关节控制
    container.innerHTML = '';
    
    // 获取URDF机器人对象（用于读取关节限位）
    const robot1URDF = appState.visualizer?.robot1URDF || null;
    const robot2URDF = appState.visualizer?.robot2URDF || null;
    
    // 机器人1的关节控制
    const robot1Section = document.createElement('div');
    robot1Section.className = 'joint-control-section';
    robot1Section.innerHTML = `<h3 style="color: #3b82f6; margin-bottom: 0.5rem; font-size: 1rem;">${config.robot1?.name || getRobotFallbackName(1)}</h3>`;
    
    const robot1Controls = document.createElement('div');
    const numJoints1 = config.robot1?.joint_chain?.joints?.length || appState.jointAngles1.length;
    for (let i = 0; i < numJoints1; i++) {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        
        const jointName = config.robot1?.joint_chain?.joints?.[i] || getJointFallbackName(i);
        
        // 获取关节限位（优先从配置文件，其次从URDF）
        const limits = getJointLimits(jointName, robot1URDF, config.robot1);
        const lowerRad = limits.lower;
        const upperRad = limits.upper;
        const sliderPrecision = getSliderPrecision();
        const inputPrecision = getInputPrecision();
        const step = displayStep().toString();
        
        // 创建标签行（关节名称 + 输入框）
        const label = document.createElement('div');
        label.className = 'joint-label';
        const nameSpan = document.createElement('span');
        nameSpan.textContent = jointName;
        label.appendChild(nameSpan);
        
        // 创建角度输入框
        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'joint-input';
        input.id = `joint1-input-${i}`;
        input.step = step;
        input.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        input.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        input.placeholder = getAnglePlaceholder();
        input.value = formatDisplayAngle(appState.jointAngles1[i] ?? 0);
        
        // 输入框事件处理（与单臂模式相同的逻辑）
        setupJointInput(input, `joint1-${i}`, lowerRad, upperRad, (angleRad) => {
            appState.jointAngles1[i] = angleRad;
            callbacks.updateRobotPose?.();
        });
        
        label.appendChild(input);
        
        // 创建滑块
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.id = `joint1-${i}`;
        slider.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        slider.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        slider.step = step;
        slider.value = formatDisplayValue(angleToDisplay(appState.jointAngles1[i] ?? 0), sliderPrecision);
        
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) {
                return;
            }
            const angleRad = displayToAngle(displayValue);
            appState.jointAngles1[i] = angleRad;
            input.value = formatDisplayValue(displayValue, inputPrecision);
            callbacks.updateRobotPose?.();
        });
        
        jointDiv.appendChild(label);
        jointDiv.appendChild(slider);
        robot1Controls.appendChild(jointDiv);
    }
    robot1Section.appendChild(robot1Controls);
    container.appendChild(robot1Section);
    
    // 机器人2的关节控制
    const robot2Section = document.createElement('div');
    robot2Section.className = 'joint-control-section';
    robot2Section.style.marginTop = '1.5rem';
    robot2Section.innerHTML = `<h3 style="color: #10b981; margin-bottom: 0.5rem; font-size: 1rem;">${config.robot2?.name || getRobotFallbackName(2)}</h3>`;
    
    const robot2Controls = document.createElement('div');
    const numJoints2 = config.robot2?.joint_chain?.joints?.length || appState.jointAngles2.length;
    for (let i = 0; i < numJoints2; i++) {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        
        const jointName = config.robot2?.joint_chain?.joints?.[i] || getJointFallbackName(i);
        
        // 获取关节限位（优先从配置文件，其次从URDF）
        const limits = getJointLimits(jointName, robot2URDF, config.robot2);
        const lowerRad = limits.lower;
        const upperRad = limits.upper;
        const sliderPrecision = getSliderPrecision();
        const inputPrecision = getInputPrecision();
        const step = displayStep().toString();
        
        // 创建标签行（关节名称 + 输入框）
        const label = document.createElement('div');
        label.className = 'joint-label';
        const nameSpan = document.createElement('span');
        nameSpan.textContent = jointName;
        label.appendChild(nameSpan);
        
        // 创建角度输入框
        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'joint-input';
        input.id = `joint2-input-${i}`;
        input.step = step;
        input.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        input.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        input.placeholder = getAnglePlaceholder();
        input.value = formatDisplayAngle(appState.jointAngles2[i] ?? 0);
        
        // 输入框事件处理
        setupJointInput(input, `joint2-${i}`, lowerRad, upperRad, (angleRad) => {
            appState.jointAngles2[i] = angleRad;
            callbacks.updateRobotPose?.();
        });
        
        label.appendChild(input);
        
        // 创建滑块
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.id = `joint2-${i}`;
        slider.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        slider.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        slider.step = step;
        slider.value = formatDisplayValue(angleToDisplay(appState.jointAngles2[i] ?? 0), sliderPrecision);
        
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) {
                return;
            }
            const angleRad = displayToAngle(displayValue);
            appState.jointAngles2[i] = angleRad;
            input.value = formatDisplayValue(displayValue, inputPrecision);
            callbacks.updateRobotPose?.();
        });
        
        jointDiv.appendChild(label);
        jointDiv.appendChild(slider);
        robot2Controls.appendChild(jointDiv);
    }
    robot2Section.appendChild(robot2Controls);
    container.appendChild(robot2Section);
    
    // 更新双臂滑块值和显示（会在限制角度后调用）
    updateDualArmJointSlidersFromAngles();
}

/**
 * 创建人形关节控制（腰+左臂+右臂+头，姿态控制以左臂、右臂为主）
 * 按部位分组显示：腰部、左臂、右臂、头部
 */
export function createHumanoidJointControls(config) {
    const container = document.getElementById('joint-controls');
    if (!container) return;
    container.innerHTML = '';
    if (appState.visualizer) {
        appState.visualizer.setAddWaypointMode?.(false);
        appState.visualizer.onGroundClick = null;
    }
    const urdfRobot = appState.visualizer?.humanoidURDF || null;
    const n = appState.humanoidJointNames.length;
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const step = displayStep().toString();
    
    // 获取各部分长度
    const waistLen = appState.visualizer?.humanoidWaistLength ?? 0;
    const leftLen = appState.visualizer?.humanoidLeftArmLength ?? 0;
    const rightLen = appState.visualizer?.humanoidRightArmLength ?? 0;
    const headLen = appState.visualizer?.humanoidHeadLength ?? 0;
    
    // 计算各部分的起始索引
    const waistStart = 0;
    const leftStart = waistLen;
    const rightStart = waistLen + leftLen;
    const headStart = waistLen + leftLen + rightLen;

    // 创建分组标题的辅助函数
    const createGroupHeader = (title, jointCount) => {
        const header = document.createElement('div');
        header.className = 'joint-group-header';
        header.innerHTML = `<span class="group-title">${title}</span><span class="group-count">${t('{{count}} Joints', { count: jointCount })}</span>`;
        header.style.cssText = `
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 0.5rem 0.75rem;
            margin: 0.5rem 0 0.25rem 0;
            background: linear-gradient(90deg, rgba(99, 102, 241, 0.3), transparent);
            border-left: 3px solid #6366f1;
            border-radius: 0 4px 4px 0;
            font-weight: 600;
            font-size: 0.85rem;
            color: #e0e0ff;
        `;
        const countSpan = header.querySelector('.group-count');
        if (countSpan) {
            countSpan.style.cssText = `
                font-size: 0.75rem;
                color: #a0a0c0;
                font-weight: normal;
            `;
        }
        return header;
    };
    
    for (let i = 0; i < n; i++) {
        // 在各部分开始处添加分组标题
        if (i === waistStart && waistLen > 0) {
            container.appendChild(createGroupHeader(t('🦴 Waist'), waistLen));
        } else if (i === leftStart && leftLen > 0) {
            container.appendChild(createGroupHeader(t('🦾 Left Arm'), leftLen));
        } else if (i === rightStart && rightLen > 0) {
            container.appendChild(createGroupHeader(t('🦿 Right Arm'), rightLen));
        } else if (i === headStart && headLen > 0) {
            container.appendChild(createGroupHeader(t('🗣️ Head'), headLen));
        }
        
        const jointName = appState.humanoidJointNames[i] || `q${i}`;
        const limits = getJointLimits(jointName, urdfRobot, config);
        const lowerRad = limits.lower;
        const upperRad = limits.upper;
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        const label = document.createElement('div');
        label.className = 'joint-label';
        const nameSpan = document.createElement('span');
        nameSpan.textContent = jointName;
        label.appendChild(nameSpan);
        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'joint-input';
        input.id = `humanoid-input-${i}`;
        input.step = step;
        input.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        input.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        input.placeholder = getAnglePlaceholder();
        input.value = formatDisplayAngle(appState.humanoidQ[i] ?? 0);
        setupJointInput(input, `humanoid-${i}`, lowerRad, upperRad, (angleRad) => {
            appState.humanoidQ[i] = angleRad;
            callbacks.updateRobotPose?.();
        });
        label.appendChild(input);
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.id = `humanoid-${i}`;
        slider.min = formatDisplayValue(angleToDisplay(lowerRad), sliderPrecision);
        slider.max = formatDisplayValue(angleToDisplay(upperRad), sliderPrecision);
        slider.step = step;
        slider.value = formatDisplayValue(angleToDisplay(appState.humanoidQ[i] ?? 0), sliderPrecision);
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) return;
            appState.humanoidQ[i] = displayToAngle(displayValue);
            input.value = formatDisplayValue(displayValue, inputPrecision);
            callbacks.updateRobotPose?.();
        });
        jointDiv.appendChild(label);
        jointDiv.appendChild(slider);
        container.appendChild(jointDiv);
    }
}

/**
 * 从当前双臂关节角度更新所有滑块的值和显示
 */
export function updateDualArmJointSlidersFromAngles() {
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const step = displayStep().toString();
    const placeholder = getAnglePlaceholder();
    
    // 更新机器人1的滑块和输入框
    if (appState.jointAngles1 && appState.jointAngles1.length > 0) {
        appState.jointAngles1.forEach((angle, i) => {
            const displayValue = angleToDisplay(angle);
            const slider = document.getElementById(`joint1-${i}`);
            const input = document.getElementById(`joint1-input-${i}`);
            
            if (slider) {
                slider.value = formatDisplayValue(displayValue, sliderPrecision);
                slider.step = step;
            }
            
            if (input) {
                input.value = formatDisplayValue(displayValue, inputPrecision);
                input.step = step;
                input.placeholder = placeholder;
            }
        });
    }
    
    // 更新机器人2的滑块和输入框
    if (appState.jointAngles2 && appState.jointAngles2.length > 0) {
        appState.jointAngles2.forEach((angle, i) => {
            const displayValue = angleToDisplay(angle);
            const slider = document.getElementById(`joint2-${i}`);
            const input = document.getElementById(`joint2-input-${i}`);
            
            if (slider) {
                slider.value = formatDisplayValue(displayValue, sliderPrecision);
                slider.step = step;
            }
            
            if (input) {
                input.value = formatDisplayValue(displayValue, inputPrecision);
                input.step = step;
                input.placeholder = placeholder;
            }
        });
    }
}

/**
 * 从当前人形关节角度更新所有滑块的值和显示
 */
export function updateHumanoidJointSlidersFromAngles() {
    if (!appState.humanoidQ || appState.humanoidQ.length === 0) return;
    
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const step = displayStep().toString();
    const placeholder = getAnglePlaceholder();
    
    appState.humanoidQ.forEach((angle, i) => {
        const displayValue = angleToDisplay(angle);
        const slider = document.getElementById(`humanoid-${i}`);
        const input = document.getElementById(`humanoid-input-${i}`);
        
        if (slider) {
            slider.value = formatDisplayValue(displayValue, sliderPrecision);
            slider.step = step;
        }
        
        if (input) {
            input.value = formatDisplayValue(displayValue, inputPrecision);
            input.step = step;
            input.placeholder = placeholder;
        }
    });

}

/**
 * 刷新关节控制显示（根据当前模式重新创建）
 */
export function refreshJointControlDisplay() {
    const container = document.getElementById('joint-controls');
    if (!container) return;
    const config = getActiveJointConfig();
    if (appState.humanoidMode && appState.humanoidJointNames.length > 0) {
        createHumanoidJointControls(config || {});
        return;
    }
    if (appState.dualArmMode && config) {
        const totalJoints =
            (config.robot1?.joint_chain?.joints?.length || appState.jointAngles1.length) +
            (config.robot2?.joint_chain?.joints?.length || appState.jointAngles2.length);
        createJointControls(totalJoints, config);
    } else {
        const numJoints =
            appState.jointAngles.length ||
            config?.joint_chain?.joints?.length ||
            (appState.currentRobot?.getDimConfigurationSpace?.() ?? 0);
        
        if (numJoints > 0) {
            createJointControls(numJoints, config);
        } else {
            container.innerHTML = '';
        }
    }
}

/**
 * 注册本模块的回调函数
 */
export function registerJointControlsCallbacks() {
    callbacks.createJointControls = createJointControls;
    callbacks.updateJointSlidersFromAngles = updateJointSlidersFromAngles;
    callbacks.updateDualArmJointSlidersFromAngles = updateDualArmJointSlidersFromAngles;
    callbacks.updateHumanoidJointSlidersFromAngles = updateHumanoidJointSlidersFromAngles;
    callbacks.refreshJointControlDisplay = refreshJointControlDisplay;
    callbacks.clampJointAngles = clampJointAngles;
    callbacks.getJointLimits = getJointLimits;
    callbacks.createDualArmJointControls = createDualArmJointControls;
    callbacks.createHumanoidJointControls = createHumanoidJointControls;

    if (!jointControlsLanguageBound) {
        onLanguageChange(() => {
            refreshJointControlDisplay();
        });
        jointControlsLanguageBound = true;
    }
}
