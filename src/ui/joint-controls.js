/**
 * 关节控制滑块/输入框 UI 模块
 * 负责创建和更新单臂、双臂、人形机器人的关节控制界面
 */

import { DEG_TO_RAD, RAD_TO_DEG, CM_TO_M } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { angleToDisplay, displayToAngle, getSliderPrecision, getInputPrecision, displayStep, getAnglePlaceholder, formatDisplayValue, formatDisplayAngle, getActiveJointConfig, wrapAngle } from '../utils/math-utils.js';
import { resetRPPState } from '../humanoid/base-control.js';

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
        
        const jointName = config?.joint_chain?.joints?.[i] || `关节 ${i + 1}`;
        
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
    robot1Section.innerHTML = `<h3 style="color: #3b82f6; margin-bottom: 0.5rem; font-size: 1rem;">${config.robot1?.name || '机器人1'}</h3>`;
    
    const robot1Controls = document.createElement('div');
    const numJoints1 = config.robot1?.joint_chain?.joints?.length || appState.jointAngles1.length;
    for (let i = 0; i < numJoints1; i++) {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        
        const jointName = config.robot1?.joint_chain?.joints?.[i] || `关节 ${i + 1}`;
        
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
    robot2Section.innerHTML = `<h3 style="color: #10b981; margin-bottom: 0.5rem; font-size: 1rem;">${config.robot2?.name || '机器人2'}</h3>`;
    
    const robot2Controls = document.createElement('div');
    const numJoints2 = config.robot2?.joint_chain?.joints?.length || appState.jointAngles2.length;
    for (let i = 0; i < numJoints2; i++) {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        
        const jointName = config.robot2?.joint_chain?.joints?.[i] || `关节 ${i + 1}`;
        
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

    // ── 差速底盘控制区域 ──
    const baseHeader = document.createElement('div');
    baseHeader.className = 'joint-group-header';
    baseHeader.innerHTML = '<span class="group-title">🚗 差速底盘</span><span class="group-count" id="humanoid-base-mode-label">点稳定</span>';
    baseHeader.style.cssText = `
        display: flex;
        justify-content: space-between;
        align-items: center;
        padding: 0.5rem 0.75rem;
        margin: 0.5rem 0 0.25rem 0;
        background: linear-gradient(90deg, rgba(16, 185, 129, 0.3), transparent);
        border-left: 3px solid #10b981;
        border-radius: 0 4px 4px 0;
        font-weight: 600;
        font-size: 0.85rem;
        color: #d1fae5;
    `;
    container.appendChild(baseHeader);

    // ── 控制模式切换按钮 ──
    const modeToggleRow = document.createElement('div');
    modeToggleRow.style.cssText = 'display:flex; gap:6px; padding:4px 8px; margin-bottom:4px;';
    const createModeBtn = (label, mode) => {
        const btn = document.createElement('button');
        btn.textContent = label;
        btn.dataset.mode = mode;
        btn.className = 'rpp-mode-btn';
        btn.style.cssText = `
            flex:1; padding:4px 8px; border:1px solid #4b5563; border-radius:4px;
            background:${appState.humanoidBaseControlMode === mode ? '#10b981' : '#374151'};
            color:#e5e7eb; font-size:0.78rem; cursor:pointer; transition:background 0.15s;
        `;
        btn.addEventListener('click', () => {
            appState.humanoidBaseControlMode = mode;
            // 切换视觉样式
            modeToggleRow.querySelectorAll('.rpp-mode-btn').forEach(b => {
                b.style.background = b.dataset.mode === mode ? '#10b981' : '#374151';
            });
            // 显示/隐藏对应面板
            pointControlsDiv.style.display = mode === 'point' ? '' : 'none';
            pathControlsDiv.style.display = mode === 'path' ? '' : 'none';
            // 更新模式标签
            const modeLabel = document.getElementById('humanoid-base-mode-label');
            if (modeLabel) modeLabel.textContent = mode === 'point' ? '点稳定' : '路径跟踪(RPP)';
            // 切换到路径模式时启用航点添加
            if (appState.visualizer) {
                appState.visualizer.setAddWaypointMode(mode === 'path');
            }
            // 重置路径跟踪状态
            if (mode === 'path') {
                resetRPPState();
                appState.humanoidBaseConvergedCount = 0;
            } else {
                // 切回点模式，清除路径可视化
                if (appState.visualizer) {
                    appState.visualizer.clearPathVisualization();
                    appState.visualizer.setAddWaypointMode(false);
                }
                appState.humanoidBasePath = [];
            }
        });
        return btn;
    };
    modeToggleRow.appendChild(createModeBtn('点稳定', 'point'));
    modeToggleRow.appendChild(createModeBtn('路径跟踪 (RPP)', 'path'));
    container.appendChild(modeToggleRow);

    // ── 点稳定控件容器 ──
    const pointControlsDiv = document.createElement('div');
    pointControlsDiv.id = 'humanoid-base-point-controls';
    pointControlsDiv.style.display = appState.humanoidBaseControlMode === 'point' ? '' : 'none';
    container.appendChild(pointControlsDiv);

    const createBaseControl = ({ key, labelText, min, max, stepValue, toState, toDisplay }) => {
        const jointDiv = document.createElement('div');
        jointDiv.className = 'joint-control';
        const label = document.createElement('div');
        label.className = 'joint-label';
        const nameSpan = document.createElement('span');
        nameSpan.textContent = labelText;
        label.appendChild(nameSpan);

        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'joint-input';
        input.id = `humanoid-base-input-${key}`;
        input.step = String(stepValue);
        input.min = String(min);
        input.max = String(max);
        input.value = toDisplay(appState.humanoidBaseTargetPose[key]).toFixed(key === 'yaw' ? inputPrecision : 3);
        label.appendChild(input);

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.id = `humanoid-base-${key}`;
        slider.min = String(min);
        slider.max = String(max);
        slider.step = String(stepValue);
        slider.value = toDisplay(appState.humanoidBaseTargetPose[key]).toFixed(key === 'yaw' ? sliderPrecision : 3);

        const applyValue = (displayValue) => {
            if (!Number.isFinite(displayValue)) return;
            const clamped = Math.max(min, Math.min(max, displayValue));
            appState.humanoidBaseTargetPose[key] = toState(clamped);
            appState.humanoidBaseConvergedCount = 0;
            appState.humanoidBaseReverseDrive = false;
            input.value = clamped.toFixed(key === 'yaw' ? inputPrecision : 3);
            slider.value = clamped.toFixed(key === 'yaw' ? sliderPrecision : 3);
        };

        input.addEventListener('change', (e) => {
            applyValue(parseFloat(e.target.value));
        });
        slider.addEventListener('input', (e) => {
            applyValue(parseFloat(e.target.value));
        });

        jointDiv.appendChild(label);
        jointDiv.appendChild(slider);
        pointControlsDiv.appendChild(jointDiv);
    };

    createBaseControl({
        key: 'x',
        labelText: 'target_x (m)',
        min: -5,
        max: 5,
        stepValue: 0.005,
        toState: (v) => v,
        toDisplay: (v) => Number(v) || 0
    });
    createBaseControl({
        key: 'y',
        labelText: 'target_y (m)',
        min: -5,
        max: 5,
        stepValue: 0.005,
        toState: (v) => v,
        toDisplay: (v) => Number(v) || 0
    });
    createBaseControl({
        key: 'yaw',
        labelText: appState.angleDisplayMode === 'deg' ? 'target_yaw (deg)' : 'target_yaw (rad)',
        min: appState.angleDisplayMode === 'deg' ? -180 : -Math.PI,
        max: appState.angleDisplayMode === 'deg' ? 180 : Math.PI,
        stepValue: displayStep(),
        toState: (v) => displayToAngle(v),
        toDisplay: (v) => angleToDisplay(v)
    });

    // ── 路径跟踪控件容器 ──
    const pathControlsDiv = document.createElement('div');
    pathControlsDiv.id = 'humanoid-base-path-controls';
    pathControlsDiv.style.display = appState.humanoidBaseControlMode === 'path' ? '' : 'none';
    container.appendChild(pathControlsDiv);

    // 路径操作按钮行
    const pathBtnRow = document.createElement('div');
    pathBtnRow.style.cssText = 'display:flex; gap:6px; padding:4px 8px; flex-wrap:wrap;';

    const pathBtnStyle = `
        padding:4px 10px; border:1px solid #4b5563; border-radius:4px;
        background:#374151; color:#e5e7eb; font-size:0.76rem; cursor:pointer;
        transition:background 0.15s;
    `;

    // ▶ 运行 / ■ 停止 按钮
    const runBtn = document.createElement('button');
    runBtn.id = 'rpp-run-btn';
    const updateRunBtnStyle = () => {
        const running = appState.humanoidBaseRPPState.isRunning;
        runBtn.textContent = running ? '■ 停止' : '▶ 运行';
        runBtn.style.cssText = `
            padding:4px 14px; border:1px solid ${running ? '#ef4444' : '#10b981'}; border-radius:4px;
            background:${running ? '#991b1b' : '#065f46'}; color:#e5e7eb; font-size:0.76rem;
            cursor:pointer; transition:background 0.15s; font-weight:600;
        `;
    };
    updateRunBtnStyle();
    runBtn.addEventListener('click', () => {
        if (appState.humanoidBaseRPPState.isRunning) {
            // 停止
            appState.humanoidBaseRPPState.isRunning = false;
            appState.humanoidBaseWheelVelocity.left = 0;
            appState.humanoidBaseWheelVelocity.right = 0;
            appState.humanoidBaseRPPState.lastCmdLinear = 0;
            appState.humanoidBaseRPPState.lastCmdAngular = 0;
            // 重新启用航点添加
            if (appState.visualizer) appState.visualizer.setAddWaypointMode(true);
        } else {
            // 启动运行
            if (appState.humanoidBasePath.length < 2) {
                alert('请先添加至少 2 个航点');
                return;
            }
            // 重置跟踪状态但保留路径
            appState.humanoidBaseRPPState.isRotatingToHeading = false;
            appState.humanoidBaseRPPState.hasReachedGoalXY = false;
            appState.humanoidBaseRPPState.lastCmdLinear = 0;
            appState.humanoidBaseRPPState.lastCmdAngular = 0;
            appState.humanoidBaseRPPState.isFinished = false;
            appState.humanoidBaseRPPState.carrotWorld = null;
            appState.humanoidBaseRPPState.progressSegIdx = 0;
            appState.humanoidBaseRPPState.isRunning = true;
            appState.humanoidBaseConvergedCount = 0;
            // 运行中禁用航点添加，防止误点
            if (appState.visualizer) appState.visualizer.setAddWaypointMode(false);
        }
        updateRunBtnStyle();
        updatePathStatusDisplay();
    });
    pathBtnRow.appendChild(runBtn);
    // 暴露按钮样式更新函数
    window._updateRPPRunBtn = updateRunBtnStyle;

    // 撤销最后一个航点
    const undoWpBtn = document.createElement('button');
    undoWpBtn.textContent = '↩ 撤销航点';
    undoWpBtn.style.cssText = pathBtnStyle;
    undoWpBtn.addEventListener('click', () => {
        if (appState.humanoidBasePath.length > 0) {
            appState.humanoidBasePath.pop();
            resetRPPState();
            appState.humanoidBaseConvergedCount = 0;
            if (appState.visualizer) {
                appState.visualizer.setPathWaypoints(appState.humanoidBasePath);
                appState.visualizer.setAddWaypointMode(true);
            }
            updateRunBtnStyle();
            updatePathStatusDisplay();
        }
    });
    pathBtnRow.appendChild(undoWpBtn);

    // 清除路径
    const clearPathBtn = document.createElement('button');
    clearPathBtn.textContent = '🗑 清除路径';
    clearPathBtn.style.cssText = pathBtnStyle;
    clearPathBtn.addEventListener('click', () => {
        appState.humanoidBasePath = [];
        resetRPPState();
        appState.humanoidBaseConvergedCount = 0;
        if (appState.visualizer) {
            appState.visualizer.clearPathVisualization();
            appState.visualizer.setAddWaypointMode(true);
        }
        updateRunBtnStyle();
        updatePathStatusDisplay();
    });
    pathBtnRow.appendChild(clearPathBtn);

    // 设置终点朝向
    const setGoalYawBtn = document.createElement('button');
    setGoalYawBtn.textContent = '🧭 设终点朝向';
    setGoalYawBtn.style.cssText = pathBtnStyle;
    setGoalYawBtn.addEventListener('click', () => {
        if (appState.humanoidBasePath.length === 0) return;
        const yawStr = prompt('请输入终点朝向角度 (deg):', '0');
        if (yawStr === null) return;
        const yawDeg = parseFloat(yawStr);
        if (Number.isFinite(yawDeg)) {
            appState.humanoidBasePath[appState.humanoidBasePath.length - 1].yaw = yawDeg * DEG_TO_RAD;
            resetRPPState();
            appState.humanoidBaseConvergedCount = 0;
            if (appState.visualizer) appState.visualizer.setPathWaypoints(appState.humanoidBasePath);
            updatePathStatusDisplay();
        }
    });
    pathBtnRow.appendChild(setGoalYawBtn);

    pathControlsDiv.appendChild(pathBtnRow);

    // 路径状态信息
    const pathStatusDiv = document.createElement('div');
    pathStatusDiv.id = 'humanoid-base-path-status';
    pathStatusDiv.className = 'panel-subtitle';
    pathStatusDiv.style.cssText = 'padding:2px 8px; font-size:0.75rem; color:#9ca3af;';
    pathStatusDiv.textContent = '点击3D场景地面添加航点（至少2个）';
    pathControlsDiv.appendChild(pathStatusDiv);

    // RPP 参数折叠面板
    const rppParamsToggle = document.createElement('div');
    rppParamsToggle.style.cssText = `
        padding:3px 8px; font-size:0.75rem; color:#6ee7b7; cursor:pointer;
        user-select:none; margin-top:2px;
    `;
    rppParamsToggle.textContent = '▶ RPP 参数';
    let rppParamsExpanded = false;
    const rppParamsBody = document.createElement('div');
    rppParamsBody.style.display = 'none';

    rppParamsToggle.addEventListener('click', () => {
        rppParamsExpanded = !rppParamsExpanded;
        rppParamsBody.style.display = rppParamsExpanded ? '' : 'none';
        rppParamsToggle.textContent = (rppParamsExpanded ? '▼' : '▶') + ' RPP 参数';
    });
    pathControlsDiv.appendChild(rppParamsToggle);
    pathControlsDiv.appendChild(rppParamsBody);

    // RPP 参数控件
    const rppParamDefs = [
        { key: 'lookahead_dist', label: '前视距离 (m)', min: 0.1, max: 2.0, step: 0.05 },
        { key: 'min_lookahead_dist', label: '最小前视距离 (m)', min: 0.05, max: 1.0, step: 0.05 },
        { key: 'max_lookahead_dist', label: '最大前视距离 (m)', min: 0.3, max: 3.0, step: 0.1 },
        { key: 'max_linear_vel', label: '最大线速度 (m/s)', min: 0.05, max: 2.0, step: 0.05 },
        { key: 'max_angular_vel', label: '最大角速度 (rad/s)', min: 0.5, max: 5.0, step: 0.1 },
        { key: 'regulated_linear_scaling_min_radius', label: '最小转弯半径 (m)', min: 0.1, max: 3.0, step: 0.1 },
        { key: 'goal_dist_tol', label: '到达容差 (m)', min: 0.01, max: 0.5, step: 0.01 },
        { key: 'rotate_to_heading_min_angle', label: '原地转向角度 (rad)', min: 0.1, max: 1.57, step: 0.05 },
    ];
    for (const def of rppParamDefs) {
        const row = document.createElement('div');
        row.className = 'joint-control';
        const lbl = document.createElement('div');
        lbl.className = 'joint-label';
        const nameSpan = document.createElement('span');
        nameSpan.textContent = def.label;
        nameSpan.style.fontSize = '0.73rem';
        lbl.appendChild(nameSpan);

        const inp = document.createElement('input');
        inp.type = 'number';
        inp.className = 'joint-input';
        inp.step = String(def.step);
        inp.min = String(def.min);
        inp.max = String(def.max);
        inp.value = (appState.humanoidBaseRPPParams[def.key] ?? 0).toFixed(2);
        inp.style.fontSize = '0.73rem';
        inp.addEventListener('change', () => {
            const val = parseFloat(inp.value);
            if (Number.isFinite(val)) {
                appState.humanoidBaseRPPParams[def.key] = Math.max(def.min, Math.min(def.max, val));
                slider.value = appState.humanoidBaseRPPParams[def.key].toFixed(2);
            }
        });
        lbl.appendChild(inp);

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'joint-slider';
        slider.min = String(def.min);
        slider.max = String(def.max);
        slider.step = String(def.step);
        slider.value = (appState.humanoidBaseRPPParams[def.key] ?? 0).toFixed(2);
        slider.addEventListener('input', () => {
            const val = parseFloat(slider.value);
            appState.humanoidBaseRPPParams[def.key] = val;
            inp.value = val.toFixed(2);
        });

        row.appendChild(lbl);
        row.appendChild(slider);
        rppParamsBody.appendChild(row);
    }

    // 更新路径状态显示的辅助函数
    function updatePathStatusDisplay() {
        const el = document.getElementById('humanoid-base-path-status');
        if (!el) return;
        const pathLen = appState.humanoidBasePath.length;
        const st = appState.humanoidBaseRPPState;
        if (pathLen === 0) {
            el.textContent = '点击3D场景地面添加航点（至少2个）';
        } else if (pathLen === 1) {
            el.textContent = `已添加 1 个航点，再添加至少 1 个`;
        } else {
            const lastWP = appState.humanoidBasePath[pathLen - 1];
            const goalYaw = lastWP.yaw !== undefined ? ` | 终点朝向: ${(lastWP.yaw * RAD_TO_DEG).toFixed(1)}°` : '';
            let statusText;
            if (st.isFinished) statusText = '✓ 已到达';
            else if (st.isRunning) statusText = st.isRotatingToHeading ? '转向中...' : '跟踪中...';
            else statusText = '就绪（点击 ▶ 运行）';
            el.textContent = `${pathLen} 个航点${goalYaw} | ${statusText}`;
        }
    }
    // 暴露更新函数供控制循环使用
    window._updatePathStatusDisplay = updatePathStatusDisplay;

    // ── 连接 visualizer 的地面点击回调 ──
    if (appState.visualizer) {
        appState.visualizer.onGroundClick = (x, y) => {
            if (appState.humanoidBaseControlMode !== 'path') return;
            appState.humanoidBasePath.push({ x, y });
            resetRPPState();
            appState.humanoidBaseConvergedCount = 0;
            appState.visualizer.setPathWaypoints(appState.humanoidBasePath);
            updatePathStatusDisplay();
        };
    }

    // 通用底盘状态栏
    const baseStatus = document.createElement('div');
    baseStatus.className = 'panel-subtitle';
    baseStatus.id = 'humanoid-base-wheel-status';
    baseStatus.textContent = '轮速: left=0.00 rad/s, right=0.00 rad/s | 当前: x=0.000, y=0.000, yaw=0.00';
    container.appendChild(baseStatus);
    
    // 创建分组标题的辅助函数
    const createGroupHeader = (title, jointCount) => {
        const header = document.createElement('div');
        header.className = 'joint-group-header';
        header.innerHTML = `<span class="group-title">${title}</span><span class="group-count">${jointCount} 关节</span>`;
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
            container.appendChild(createGroupHeader('🦴 腰部', waistLen));
        } else if (i === leftStart && leftLen > 0) {
            container.appendChild(createGroupHeader('🦾 左臂', leftLen));
        } else if (i === rightStart && rightLen > 0) {
            container.appendChild(createGroupHeader('🦿 右臂', rightLen));
        } else if (i === headStart && headLen > 0) {
            container.appendChild(createGroupHeader('🗣️ 头部', headLen));
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

    const baseXSlider = document.getElementById('humanoid-base-x');
    const baseXInput = document.getElementById('humanoid-base-input-x');
    const baseYSlider = document.getElementById('humanoid-base-y');
    const baseYInput = document.getElementById('humanoid-base-input-y');
    const baseYawSlider = document.getElementById('humanoid-base-yaw');
    const baseYawInput = document.getElementById('humanoid-base-input-yaw');
    if (baseXSlider) baseXSlider.value = (Number(appState.humanoidBaseTargetPose.x) || 0).toFixed(3);
    if (baseXInput) baseXInput.value = (Number(appState.humanoidBaseTargetPose.x) || 0).toFixed(3);
    if (baseYSlider) baseYSlider.value = (Number(appState.humanoidBaseTargetPose.y) || 0).toFixed(3);
    if (baseYInput) baseYInput.value = (Number(appState.humanoidBaseTargetPose.y) || 0).toFixed(3);
    const yawDisplay = angleToDisplay(Number(appState.humanoidBaseTargetPose.yaw) || 0);
    if (baseYawSlider) baseYawSlider.value = formatDisplayValue(yawDisplay, sliderPrecision);
    if (baseYawInput) baseYawInput.value = formatDisplayValue(yawDisplay, inputPrecision);

    const wheelStatus = document.getElementById('humanoid-base-wheel-status');
    if (wheelStatus) {
        const modeText = appState.humanoidBaseControlMode === 'path' ? 'RPP' : 'point';
        const currentYawDisplay = angleToDisplay(Number(appState.humanoidBasePose.yaw) || 0);

        let posErr, yawErrText;
        if (appState.humanoidBaseControlMode === 'path' && appState.humanoidBasePath.length >= 2) {
            const goal = appState.humanoidBasePath[appState.humanoidBasePath.length - 1];
            posErr = Math.hypot((goal.x || 0) - (appState.humanoidBasePose.x || 0), (goal.y || 0) - (appState.humanoidBasePose.y || 0));
            const rotating = appState.humanoidBaseRPPState.isRotatingToHeading ? ' [转向]' : '';
            yawErrText = `到终点: ${posErr.toFixed(3)}m${rotating}`;
        } else {
            const dx = (appState.humanoidBaseTargetPose.x || 0) - (appState.humanoidBasePose.x || 0);
            const dy = (appState.humanoidBaseTargetPose.y || 0) - (appState.humanoidBasePose.y || 0);
            posErr = Math.hypot(dx, dy);
            const yawErrDisplay = angleToDisplay(
                wrapAngle((appState.humanoidBaseTargetPose.yaw || 0) - (appState.humanoidBasePose.yaw || 0))
            );
            yawErrText = `误差: pos=${posErr.toFixed(3)}m, yaw=${yawErrDisplay.toFixed(2)} ${appState.angleDisplayMode === 'deg' ? 'deg' : 'rad'}`;
        }

        wheelStatus.textContent =
            `${modeText} | ` +
            `轮速: L=${(appState.humanoidBaseWheelVelocity.left || 0).toFixed(2)}, ` +
            `R=${(appState.humanoidBaseWheelVelocity.right || 0).toFixed(2)} rad/s | ` +
            `位姿: (${(appState.humanoidBasePose.x || 0).toFixed(3)}, ` +
            `${(appState.humanoidBasePose.y || 0).toFixed(3)}, ` +
            `${currentYawDisplay.toFixed(2)}${appState.angleDisplayMode === 'deg' ? '°' : 'rad'}) | ` +
            yawErrText;
    }

    // 更新路径状态显示
    if (appState.humanoidBaseControlMode === 'path' && typeof window._updatePathStatusDisplay === 'function') {
        window._updatePathStatusDisplay();
    }
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
}
