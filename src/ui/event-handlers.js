/**
 * 事件处理模块
 * 
 * 包含主事件监听器设置、拖拽加载、预设位姿、标签切换、
 * DQ 演示、MDH 参数提取/对比/应用等功能。
 */

import { DEG_TO_RAD } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { DQ, SerialManipulator, FrankaPanda, getDQModule } from '../dqrobotics.js';
import { extractMDHFromURDF, createMDHTestSuite, formatMDHForDQ } from '../urdf-to-mdh.js';
import { URDFPackageManager, loadURDFFromPackage } from '../urdf-loader.js';
import { getActiveJointConfig, displayStep, angleToDisplay } from '../utils/math.js';
import { setupGlobalDragAndDrop } from './drag-drop.js';
import { setupScenePresetListeners } from './environment-panel.js';
import { setupPlannedMotionListeners } from '../features/planned-motion.js';

// ═══════════════════════════════════════════
//  Main Event Listeners Setup
// ═══════════════════════════════════════════

function setupEventListeners() {
    // 预设位姿按钮
    document.querySelectorAll('.preset-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const preset = btn.dataset.preset;
            applyPreset(preset);
        });
    });
    
    // 标签切换
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const tabName = btn.dataset.tab;
            switchTab(tabName);
        });
    });
    
    // DQ 演示按钮
    document.getElementById('dq-rotation-demo').addEventListener('click', demoRotation);
    document.getElementById('dq-translation-demo').addEventListener('click', demoTranslation);
    document.getElementById('dq-pose-demo').addEventListener('click', demoPose);
    
    // URDF → MDH 按钮
    document.getElementById('extract-mdh-btn').addEventListener('click', extractMDHFromCurrentURDF);
    document.getElementById('compare-mdh-btn').addEventListener('click', compareMDHParameters);
    document.getElementById('use-urdf-mdh-btn').addEventListener('click', useURDFMDH);
    
    // MDH坐标系显示开关
    const mdhFramesToggle = document.getElementById('show-mdh-frames-toggle');
    if (mdhFramesToggle) {
        mdhFramesToggle.addEventListener('change', (e) => {
            if (appState.visualizer) {
                appState.visualizer.toggleMDHFrames(e.target.checked);
            }
        });
    }
    
    // URDF坐标系显示开关
    const urdfFramesToggle = document.getElementById('show-urdf-frames-toggle');
    if (urdfFramesToggle) {
        urdfFramesToggle.addEventListener('change', (e) => {
            if (appState.visualizer) {
                appState.visualizer.toggleURDFFrames(e.target.checked);
            }
        });
    }
    
    const angleUnitToggle = document.getElementById('angle-unit-toggle');
    if (angleUnitToggle) {
        const updateAngleUnitToggleLabel = () => {
            angleUnitToggle.textContent = appState.angleDisplayMode === 'deg' ? '切换为弧度' : '切换为角度';
        };
        updateAngleUnitToggleLabel();
        angleUnitToggle.addEventListener('click', () => {
            appState.angleDisplayMode = appState.angleDisplayMode === 'deg' ? 'rad' : 'deg';
            if (callbacks.refreshJointControlDisplay) {
                callbacks.refreshJointControlDisplay();
            }
            updateAngleUnitToggleLabel();
        });
    }

    const importAnglesBtn = document.getElementById('import-joint-angles-btn');
    if (importAnglesBtn) {
        const defaultText = importAnglesBtn.textContent.trim();
        importAnglesBtn.addEventListener('click', async () => {
            let inputText = '';
            importAnglesBtn.disabled = true;
            try {
                if (navigator?.clipboard?.readText) {
                    inputText = await navigator.clipboard.readText();
                    if (!inputText || !inputText.trim()) {
                        throw new Error('剪贴板为空，无法导入关节角度');
                    }
                } else {
                    const promptText = window.prompt('粘贴关节角度数组（支持直接粘贴复制的输出）：');
                    if (promptText === null) {
                        importAnglesBtn.disabled = false;
                        return;
                    }
                    inputText = promptText;
                }

                if (callbacks.importJointAnglesFromText) {
                    callbacks.importJointAnglesFromText(inputText);
                }
                importAnglesBtn.textContent = '导入成功';
            } catch (error) {
                console.error('导入关节角度失败:', error);
                importAnglesBtn.textContent = '导入失败';
                window.alert(`导入关节角度失败：${error.message}`);
            } finally {
                setTimeout(() => {
                    importAnglesBtn.textContent = defaultText;
                    importAnglesBtn.disabled = false;
                }, 1500);
            }
        });
    }

    const addEnvironmentBoxBtn = document.getElementById('add-environment-box-btn');
    if (addEnvironmentBoxBtn) {
        const defaultLabel = addEnvironmentBoxBtn.textContent.trim();
        addEnvironmentBoxBtn.addEventListener('click', () => {
            addEnvironmentBoxBtn.disabled = true;
            try {
                if (callbacks.addEnvironmentBoxToScene) {
                    callbacks.addEnvironmentBoxToScene();
                }
                addEnvironmentBoxBtn.textContent = '已添加箱子';
            } catch (error) {
                console.error('添加环境箱体失败:', error);
                addEnvironmentBoxBtn.textContent = '添加失败';
                window.alert(`添加环境箱体失败：${error.message}`);
            } finally {
                setTimeout(() => {
                    addEnvironmentBoxBtn.textContent = defaultLabel;
                    addEnvironmentBoxBtn.disabled = false;
                }, 1500);
            }
        });
    }

    // 场景预设下拉菜单
    setupScenePresetListeners();

    // 规划运动事件监听
    setupPlannedMotionListeners();

    const copyAnglesBtn = document.getElementById('copy-joint-angles-btn');
    if (copyAnglesBtn) {
        const defaultCopyText = copyAnglesBtn.textContent.trim();
        copyAnglesBtn.addEventListener('click', async () => {
            copyAnglesBtn.disabled = true;
            try {
                if (callbacks.copyJointAnglesToClipboard) {
                    await callbacks.copyJointAnglesToClipboard();
                }
                copyAnglesBtn.textContent = '已复制！';
            } catch (error) {
                console.error('复制关节角度失败:', error);
                copyAnglesBtn.textContent = '复制失败';
            } finally {
                setTimeout(() => {
                    copyAnglesBtn.textContent = defaultCopyText;
                    copyAnglesBtn.disabled = false;
                }, 1500);
            }
        });
    }

    const copyLimitsBtn = document.getElementById('copy-joint-limits-btn');
    if (copyLimitsBtn) {
        const defaultText = copyLimitsBtn.textContent.trim();
        copyLimitsBtn.addEventListener('click', async () => {
            copyLimitsBtn.disabled = true;
            try {
                if (callbacks.copyJointLimitsToClipboard) {
                    await callbacks.copyJointLimitsToClipboard();
                }
                copyLimitsBtn.textContent = '已复制！';
            } catch (error) {
                console.error('复制关节限位失败:', error);
                copyLimitsBtn.textContent = '复制失败';
            } finally {
                setTimeout(() => {
                    copyLimitsBtn.textContent = defaultText;
                    copyLimitsBtn.disabled = false;
                }, 1500);
            }
        });
    }
    
    // 点动乘法顺序切换
    const teachOrderToggle = document.getElementById('teach-multiply-order-toggle');
    const teachOrderStatus = document.getElementById('teach-multiply-order-status');
    const updateTeachOrderStatus = (isDeltaFirst) => {
        if (!teachOrderStatus) return;
        teachOrderStatus.textContent = isDeltaFirst 
            ? '当前模式：局部坐标（当前 × Δ）'
            : '当前模式：世界坐标（Δ × 当前）';
    };
    const applyTeachOrder = (isDeltaFirst) => {
        if (appState.visualizer?.setTeachMultiplicationOrder) {
            const order = isDeltaFirst ? 'delta-first' : 'current-first';
            appState.visualizer.setTeachMultiplicationOrder(order);
        }
    };
    if (teachOrderToggle) {
        updateTeachOrderStatus(teachOrderToggle.checked);
        applyTeachOrder(teachOrderToggle.checked);
        teachOrderToggle.addEventListener('change', (e) => {
            const isDeltaFirst = e.target.checked;
            updateTeachOrderStatus(isDeltaFirst);
            applyTeachOrder(isDeltaFirst);
        });
    }

    // 示教器速度百分比（影响每步增量）
    const teachSpeedSlider = document.getElementById('teach-speed-percent');
    const teachSpeedValue = document.getElementById('teach-speed-value');
    const TEACH_SPEED_STORAGE_KEY = 'teach-speed-percent';
    const clampInt = (v, min, max) => Math.max(min, Math.min(max, Number.parseInt(v, 10) || min));
    const applyTeachSpeedPercent = (percent) => {
        const p = clampInt(percent, 1, 100);
        if (teachSpeedSlider) teachSpeedSlider.value = String(p);
        if (teachSpeedValue) teachSpeedValue.textContent = String(p);
        try {
            localStorage.setItem(TEACH_SPEED_STORAGE_KEY, String(p));
        } catch (_) {}
        if (appState.visualizer?.setTeachSpeedPercent) {
            appState.visualizer.setTeachSpeedPercent(p);
        } else if (appState.visualizer) {
            appState.visualizer.teachSpeedPercent = p;
        }
    };
    if (teachSpeedSlider) {
        const saved = (() => {
            try { return localStorage.getItem(TEACH_SPEED_STORAGE_KEY); } catch (_) { return null; }
        })();
        applyTeachSpeedPercent(saved ?? teachSpeedSlider.value ?? 100);
        teachSpeedSlider.addEventListener('input', (e) => {
            applyTeachSpeedPercent(e.target?.value);
        });
    }
    
    // 示教器控制按钮（移除拖动末端执行器模式）
    const teachButtons = document.querySelectorAll('.teach-btn');
    teachButtons.forEach(button => {
        let intervalId = null;
        
        button.addEventListener('mousedown', (e) => {
            // 相对距离按钮单独处理（避免走 6 轴逻辑）
            if (button.classList.contains('teach-distance-btn')) {
                const direction = button.dataset.direction;
                if (!appState.visualizer || !direction) return;
                
                // 双臂或人形模式 + distance/distance-fullbody 模式下有效
                if (!appState.visualizer.dualArmMode && !appState.visualizer.humanoidMode) return;
                const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
                const controlMode = controlModeSelect ? controlModeSelect.value : 'absolute';
                if (controlMode !== 'distance' && controlMode !== 'distance-fullbody') return;
                if (typeof appState.visualizer.moveDualArmRelativeDistance !== 'function') return;
                
                const includeWaist = (controlMode === 'distance-fullbody');
                appState.visualizer.moveDualArmRelativeDistance(direction, includeWaist);
                intervalId = setInterval(() => {
                    appState.visualizer.moveDualArmRelativeDistance(direction, includeWaist);
                }, 20);
                return;
            }
            const axis = button.dataset.axis;
            const direction = button.dataset.direction;
            const type = button.dataset.type || 'translation'; // 'translation' 或 'rotation'
            
            if (!appState.visualizer || !axis || !direction) return;
            
            // 检查是否为双臂或人形模式（人形下左臂=robot1，右臂=robot2，支持绝对/相对/距离控制）
            const isDualArm = appState.visualizer.dualArmMode || appState.visualizer.humanoidMode;
            
            if (isDualArm) {
                // 获取控制模式选择器
                const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
                const controlMode = controlModeSelect ? controlModeSelect.value : 'absolute';
                // 人形模式下左臂/右臂单臂示教
                if (appState.visualizer.humanoidMode && controlMode === 'robot1') {
                    // 人形机器人左臂控制
                    if (type === 'rotation') {
                        appState.visualizer.rotateHumanoidLeftArmAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateHumanoidLeftArmAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveHumanoidLeftArmAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidLeftArmAlongAxis(axis, direction);
                        }, 20);
                    }
                    return;
                }
                if (appState.visualizer.humanoidMode && controlMode === 'robot2') {
                    // 人形机器人右臂控制
                    if (type === 'rotation') {
                        appState.visualizer.rotateHumanoidRightArmAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateHumanoidRightArmAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveHumanoidRightArmAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidRightArmAlongAxis(axis, direction);
                        }, 20);
                    }
                    return;
                }
                if (controlMode === 'robot1') {
                    // 控制机器人1
                    if (type === 'rotation') {
                        appState.visualizer.rotateRobot1EndEffectorAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateRobot1EndEffectorAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveRobot1EndEffectorAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveRobot1EndEffectorAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'robot2') {
                    // 控制机器人2
                    if (type === 'rotation') {
                        appState.visualizer.rotateRobot2EndEffectorAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateRobot2EndEffectorAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveRobot2EndEffectorAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveRobot2EndEffectorAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'relative') {
                    // 控制相对位姿（保持绝对位姿不变）
                    if (type === 'rotation') {
                        appState.visualizer.rotateDualArmRelativePoseAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateDualArmRelativePoseAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveDualArmRelativePoseAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveDualArmRelativePoseAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'decoupled') {
                    // 全身绝对位姿：腰部+双臂联动，控制绝对位姿
                    if (appState.visualizer.humanoidMode) {
                        if (type === 'rotation') {
                            appState.visualizer.rotateHumanoidAbsolutePoseAroundAxisDecoupled(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.rotateHumanoidAbsolutePoseAroundAxisDecoupled(axis, direction);
                            }, 20);
                        } else {
                            appState.visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupled(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupled(axis, direction);
                            }, 20);
                        }
                    }
                } else if (controlMode === 'relative-fullbody') {
                    // 全身相对位姿：腰部+双臂联动，控制相对位姿
                    if (appState.visualizer.humanoidMode) {
                        if (type === 'rotation') {
                            appState.visualizer.rotateHumanoidRelativePoseAroundAxisFullbody(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.rotateHumanoidRelativePoseAroundAxisFullbody(axis, direction);
                            }, 20);
                        } else {
                            appState.visualizer.moveHumanoidRelativePoseAlongAxisFullbody(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.moveHumanoidRelativePoseAlongAxisFullbody(axis, direction);
                            }, 20);
                        }
                    }
                } else if (controlMode === 'distance') {
                    // 距离模式：按键已单独处理，这里兜底不做事
                    return;
                } else {
                    // 默认：控制绝对位姿（保持相对位姿不变）
                    if (type === 'rotation') {
                        appState.visualizer.rotateDualArmAbsolutePoseAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateDualArmAbsolutePoseAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveDualArmAbsolutePoseAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveDualArmAbsolutePoseAlongAxis(axis, direction);
                        }, 20);
                    }
                }
            } else {
                // 单臂模式：控制末端执行器
                if (type === 'rotation') {
                    // 旋转控制
                    appState.visualizer.rotateEndEffectorAroundAxis(axis, direction);
                    intervalId = setInterval(() => {
                        appState.visualizer.rotateEndEffectorAroundAxis(axis, direction);
                    }, 20);
                } else {
                    // 平移控制
                    appState.visualizer.moveEndEffectorAlongAxis(axis, direction);
                    intervalId = setInterval(() => {
                        appState.visualizer.moveEndEffectorAlongAxis(axis, direction);
                    }, 20);
                }
            }
        });
        
        button.addEventListener('mouseup', () => {
            if (intervalId) {
                clearInterval(intervalId);
                intervalId = null;
            }
        });
        
        button.addEventListener('mouseleave', () => {
            if (intervalId) {
                clearInterval(intervalId);
                intervalId = null;
            }
        });
    });
    
    // 全局拖拽加载
    if (callbacks.renderEnvironmentObjectList) {
        callbacks.renderEnvironmentObjectList();
    }
    setupGlobalDragAndDrop();
    
    // 浮动面板与底部功能栏
    if (callbacks.setupFloatingPanels) {
        callbacks.setupFloatingPanels();
    }
}

// ═══════════════════════════════════════════
//  Presets & Tab Switching
// ═══════════════════════════════════════════

async function applyPreset(preset) {
    const numJoints = appState.jointAngles.length;
    
    // 获取当前配置和关节名称（用于限位检查）
    let config = null;
    let jointNames = [];
    let urdfRobot = null;
    
    // 尝试从当前加载的配置获取信息
    if (appState.customRobotConfig) {
        config = appState.customRobotConfig;
        jointNames = config.joint_chain?.joints || [];
        urdfRobot = appState.visualizer?.urdfRobot || null;
    } else if (appState.currentRobotType === 'franka') {
        // 对于内置Franka，尝试加载配置文件获取限位
        try {
            const baseUrl = import.meta.env.BASE_URL || '/';
            const response = await fetch(`${baseUrl}robots/franka/robot_config.yaml`);
            if (response.ok) {
                const yaml = await import('js-yaml');
                const text = await response.text();
                config = yaml.load(text);
                jointNames = config.joint_chain?.joints || [];
            }
        } catch (error) {
            // 使用硬编码的关节名称
            jointNames = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
                         'panda_joint5', 'panda_joint6', 'panda_joint7'];
        }
    }
    
    switch (preset) {
        case 'home':
            appState.jointAngles = new Array(numJoints).fill(0);
            break;
            
        case 'extended':
            if (appState.currentRobotType === 'franka') {
                appState.jointAngles = [0, -30, 0, -120, 0, 90, 45].map(deg => deg * Math.PI / 180);
            } else {
                appState.jointAngles = [0, -45, 90, 0, 45, 0, 0].map(deg => deg * Math.PI / 180);
            }
            break;
            
        case 'folded':
            if (appState.currentRobotType === 'franka') {
                appState.jointAngles = [0, 45, 0, -90, 0, 135, 0].map(deg => deg * Math.PI / 180);
            } else {
                appState.jointAngles = [0, 45, -90, 0, -45, 0, 0].map(deg => deg * Math.PI / 180);
            }
            break;
            
        case 'random':
            appState.jointAngles = appState.jointAngles.map(() => (Math.random() - 0.5) * Math.PI);
            break;
    }
    
    // 限制角度在限位范围内
    if (jointNames.length === appState.jointAngles.length && callbacks.clampJointAngles) {
        appState.jointAngles = callbacks.clampJointAngles(appState.jointAngles, jointNames, urdfRobot, config);
    }
    
    if (appState.dualArmMode) {
        if (callbacks.updateDualArmJointSlidersFromAngles) {
            callbacks.updateDualArmJointSlidersFromAngles();
        }
    } else {
        if (callbacks.updateJointSlidersFromAngles) {
            callbacks.updateJointSlidersFromAngles();
        }
    }
    
    if (callbacks.updateRobotPose) {
        callbacks.updateRobotPose();
    }
}

/**
 * 切换标签
 */
function switchTab(tabName) {
    // 更新按钮状态
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.tab === tabName);
    });
    
    // 更新标签内容
    document.querySelectorAll('.tab-pane').forEach(pane => {
        // 对于pose标签，根据模式显示不同的内容
        if (tabName === 'pose') {
            if (appState.dualArmMode) {
                // 双臂模式：显示双臂位姿
                if (pane.id === 'dual-arm-pose-tab') {
                    pane.classList.add('active');
                    pane.style.display = 'block';
                } else if (pane.id === 'pose-tab') {
                    pane.classList.remove('active');
                    pane.style.display = 'none';
                } else {
                    pane.classList.toggle('active', pane.id === `${tabName}-tab`);
                }
            } else {
                // 单臂模式：显示单臂位姿
                if (pane.id === 'pose-tab') {
                    pane.classList.add('active');
                    pane.style.display = 'block';
                } else if (pane.id === 'dual-arm-pose-tab') {
                    pane.classList.remove('active');
                    pane.style.display = 'none';
                } else {
                    pane.classList.toggle('active', pane.id === `${tabName}-tab`);
                }
            }
        } else {
            pane.classList.toggle('active', pane.id === `${tabName}-tab`);
        }
    });
}

// ═══════════════════════════════════════════
//  DQ Demos
// ═══════════════════════════════════════════

/**
 * DQ 旋转演示
 */
function demoRotation() {
    console.log('=== DQ 旋转演示 ===');
    
    // 创建绕 Z 轴旋转 90 度的 DQ
    const rotation = DQ.rotation(Math.PI / 2, 0, 0, 1);
    console.log('旋转 DQ (90° 绕 Z 轴):', rotation.toArray());
    
    alert('查看控制台以查看旋转 DQ 输出');
}

/**
 * DQ 平移演示
 */
function demoTranslation() {
    console.log('=== DQ 平移演示 ===');
    
    // 创建平移 (1, 2, 3) 的 DQ
    const translation = DQ.translation(1, 2, 3);
    console.log('平移 DQ (1, 2, 3):', translation.toArray());
    
    alert('查看控制台以查看平移 DQ 输出');
}

/**
 * DQ 位姿演示
 */
function demoPose() {
    console.log('=== DQ 位姿演示 ===');
    
    // 创建组合位姿: 绕 Z 轴旋转 45° + 平移 (0.5, 0.5, 0.5)
    const pose = DQ.pose(0, 0, 1, Math.PI / 4, 0.5, 0.5, 0.5);
    console.log('位姿 DQ:', pose.toArray());
    
    const translation = pose.getTranslation();
    const rotation = pose.getRotation();
    console.log('平移部分:', translation);
    console.log('旋转部分:', rotation);
    
    alert('查看控制台以查看位姿 DQ 输出');
}

// ═══════════════════════════════════════════
//  MDH Parameter Extraction & Comparison
// ═══════════════════════════════════════════

/**
 * 从当前 URDF 提取 MDH 参数
 */
function extractMDHFromCurrentURDF() {
    if (!appState.visualizer || !appState.visualizer.urdfRobot) {
        alert('⚠ URDF 模型未加载\n\n请确保 URDF 模型已成功加载。');
        return;
    }
    
    console.log('═══════════════════════════════════════════');
    console.log('       从 URDF 提取 MDH 参数');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        const jointNames = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6',
            'panda_joint7'
        ];
        
        const result = extractMDHFromURDF(appState.visualizer.urdfRobot, jointNames);
        // 处理新的返回格式
        if (result && typeof result === 'object' && result.mdh) {
            appState.extractedMDH = result.mdh;
            
            // 设置MDH坐标系可视化
            if (result.allMDHFrames && appState.visualizer) {
                appState.visualizer.setMDHFrames(result.allMDHFrames);
                console.log('  ✓ 已设置MDH坐标系可视化');
            }
        } else {
            appState.extractedMDH = result; // 向后兼容
        }
        
        // 显示状态
        const statusDiv = document.getElementById('mdh-status');
        statusDiv.innerHTML = `
            <div style="color: #10b981; font-size: 0.875rem; margin-top: 0.5rem;">
                ✓ 已提取 ${appState.extractedMDH.length} 个关节的 MDH 参数<br>
                查看控制台了解详情
            </div>
        `;
        
        alert('✓ MDH 参数提取成功！\n\n请打开控制台 (F12) 查看详细参数。');
        
    } catch (error) {
        console.error('提取 MDH 失败:', error);
        alert('❌ 提取失败\n\n' + error.message);
    }
}

/**
 * 对比 MDH 参数
 */
function compareMDHParameters() {
    if (!appState.visualizer || !appState.visualizer.urdfRobot) {
        alert('⚠ URDF 模型未加载');
        return;
    }
    
    console.log('═══════════════════════════════════════════');
    console.log('       MDH 参数对比测试');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        const result = createMDHTestSuite(appState.visualizer.urdfRobot);
        
        const statusDiv = document.getElementById('mdh-status');
        
        if (result.comparison.match) {
            statusDiv.innerHTML = `
                <div style="color: #10b981; font-size: 0.875rem; margin-top: 0.5rem;">
                    ✓ MDH 参数完全一致！<br>
                    URDF 和预定义参数匹配
                </div>
            `;
            alert('✓ 参数一致！\n\nURDF 提取的 MDH 参数与官方参考参数完全一致。\n\n查看控制台了解详情。');
        } else {
            const diffCount = result.comparison.differences.length;
            statusDiv.innerHTML = `
                <div style="color: #f59e0b; font-size: 0.875rem; margin-top: 0.5rem;">
                    ⚠ 发现 ${diffCount} 处差异<br>
                    查看控制台了解详情
                </div>
            `;
            alert(`⚠ 发现差异\n\n在 ${diffCount} 个参数上存在差异。\n\n这可能是由于：\n- 坐标系约定不同\n- 零位定义不同\n- 参数来源不同\n\n查看控制台了解详情。`);
        }
        
    } catch (error) {
        console.error('对比失败:', error);
        alert('❌ 对比失败\n\n' + error.message);
    }
}

/**
 * 使用 URDF 提取的 MDH 创建机器人
 */
function useURDFMDH() {
    if (!appState.extractedMDH) {
        alert('⚠ 请先提取 MDH 参数\n\n点击 "从URDF提取MDH" 按钮。');
        return;
    }
    
    try {
        console.log('使用 URDF 提取的 MDH 参数创建机器人...');
        
        // 格式化为 DQ Robotics 格式
        const formatted = formatMDHForDQ(appState.extractedMDH);
        
        // 转置为列格式（DQ 需要的格式）
        const mdhMatrix = [];
        for (let row = 0; row < 5; row++) {
            mdhMatrix.push(formatted[row]);
        }
        
        // 创建新的串联机械臂
        appState.urdfBasedRobot = new SerialManipulator(mdhMatrix);
        
        // 切换到使用新机器人
        appState.currentRobot = appState.urdfBasedRobot;
        if (callbacks.updateGlobalVariables) {
            callbacks.updateGlobalVariables();
        }
        
        const statusDiv = document.getElementById('mdh-status');
        statusDiv.innerHTML = `
            <div style="color: #3b82f6; font-size: 0.875rem; margin-top: 0.5rem;">
                ✓ 正在使用 URDF-MDH<br>
                运动学现在基于URDF参数
            </div>
        `;
        
        // 更新显示
        if (callbacks.updateRobotPose) {
            callbacks.updateRobotPose();
        }
        
        alert('✓ 切换成功！\n\n现在使用从 URDF 提取的 MDH 参数进行运动学计算。\n\n黄色球应该与 URDF 末端完全对齐！');
        
    } catch (error) {
        console.error('切换失败:', error);
        alert('❌ 切换失败\n\n' + error.message);
    }
}

// ═══════════════════════════════════════════
//  Callback Registration
// ═══════════════════════════════════════════

/**
 * 注册事件处理回调到全局 callbacks 对象
 */
export function registerEventHandlersCallbacks() {
    callbacks.switchTab = switchTab;
    callbacks.setupEventListeners = setupEventListeners;
}

// ═══════════════════════════════════════════
//  Exports
// ═══════════════════════════════════════════

export {
    setupEventListeners,
    applyPreset,
    switchTab,
    demoRotation,
    demoTranslation,
    demoPose,
    extractMDHFromCurrentURDF,
    compareMDHParameters,
    useURDFMDH,
};
