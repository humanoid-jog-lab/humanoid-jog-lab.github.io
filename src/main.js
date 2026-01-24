/**
 * DQ Robotics 可视化实验室主程序
 */

import { initDQRobotics, FrankaPanda, KukaLw4, DQ, SerialManipulator, CooperativeDualArm, quaternionToRotationMatrix, rotationMatrixToEuler, getDQModule } from './dqrobotics.js';
import * as THREE from 'three';
import { RobotVisualizer } from './visualizer.js';
import { extractMDHFromURDF, createMDHTestSuite, FRANKA_PANDA_MDH_REFERENCE, formatMDHForDQ } from './urdf-to-mdh.js';
import { URDFPackageManager, loadURDFFromPackage } from './urdf-loader.js';
import ROS2Communicator from './ros2-communicator.js';

const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;
const CM_TO_M = 0.01;

let angleDisplayMode = 'rad';
let frankaConfigCache = null;

// 全局状态
let currentRobot = null;
let currentRobotType = 'ia_robot';
let visualizer = null;
let jointAngles = [];
let extractedMDH = null; // 从 URDF 提取的 MDH 参数
let urdfBasedRobot = null; // 基于 URDF MDH 的机器人
let packageManager = null; // URDF 包管理器
let customRobotConfig = null; // 自定义机器人配置
let currentRobotIsYUp = false; // 若为基于URDF提取（URDF坐标系），则为 true
let dropOverlayElement = null; // 全局拖拽提示元素
let dropOverlayHideTimer = null; // 隐藏提示的定时器
let dockElement = null; // 底部功能栏元素
let panelStackIndex = 30; // 面板叠放顺序

// 暴露到全局作用域，供 visualizer 使用
window.currentRobot = currentRobot;
window.jointAngles = jointAngles;
window.customRobotConfig = customRobotConfig;
// 注意：clampJointAngles 会在函数定义后通过 updateGlobalVariables 暴露

// 双臂机器人状态
let dualArmMode = false;
let robot1 = null;  // 机器人1（DQ）
let robot2 = null;  // 机器人2（DQ）
let robot1URDF = null;  // 机器人1（URDF）
let robot2URDF = null;  // 机器人2（URDF）
let jointAngles1 = [];  // 机器人1关节角度
let jointAngles2 = [];  // 机器人2关节角度
let dualArmSystem = null;  // CooperativeDualArm对象（用于计算绝对位姿）

// 环境物体
let environmentObjects = [];
let environmentObjectCounter = 0;

// 关节位置发布定时器
let jointPositionPublishTimer = null;
const JOINT_POSITION_PUBLISH_INTERVAL = 10; // 10ms，即100Hz发布频率

const DEFAULT_ENVIRONMENT_BOX_CM = {
    length: 41.5,
    width: 30.5,
    height: 14
};
const DEFAULT_ENVIRONMENT_BOX_COLOR = 0xf97316;
const ENVIRONMENT_BOX_BASE_OFFSET_X = 0.45; // 米

// ROS 2 通讯
let ros2Communicator = null;
let ros2MessageLog = [];
const ROS2_MAX_LOG_ENTRIES = 50;
let ros2JointStateSubscribed = false;
let ros2JointStateTopic = '/arm/joint_states';
let ros2JointStateButton = null;
let ros2JointStatusElement = null;
let ros2BatteryStateSubscribed = false;
const ROS2_BATTERY_STATE_TOPIC = '/battery_state';

if (typeof window !== 'undefined') {
    window.currentDualArmControlMode = window.currentDualArmControlMode || 'absolute';
}

function angleToDisplay(angleRad) {
    if (!Number.isFinite(angleRad)) {
        return 0;
    }
    return angleDisplayMode === 'deg' ? angleRad * RAD_TO_DEG : angleRad;
}

function displayToAngle(displayValue) {
    if (!Number.isFinite(displayValue)) {
        return 0;
    }
    return angleDisplayMode === 'deg' ? displayValue * DEG_TO_RAD : displayValue;
}

function getSliderPrecision() {
    return angleDisplayMode === 'deg' ? 1 : 3;
}

function getInputPrecision() {
    return angleDisplayMode === 'deg' ? 2 : 4;
}

function displayStep() {
    return angleDisplayMode === 'deg' ? 0.1 : 0.01;
}

function getAnglePlaceholder() {
    return angleDisplayMode === 'deg' ? '角度(°)' : '角度(rad)';
}

function formatDisplayValue(value, digits) {
    const safeValue = Number.isFinite(value) ? value : 0;
    return safeValue.toFixed(digits);
}

function formatDisplayAngle(angleRad, digits = getInputPrecision()) {
    return formatDisplayValue(angleToDisplay(angleRad), digits);
}

function getActiveJointConfig() {
    return customRobotConfig || frankaConfigCache;
}

function buildJointAnglesClipboardText() {
    const digits = angleDisplayMode === 'deg' ? 3 : 5;
    const formatList = (angles = []) => {
        const list = Array.isArray(angles) ? angles : Array.from(angles || []);
        const formatted = list.map(a => parseFloat(angleToDisplay(a ?? 0).toFixed(digits)));
        return `[${formatted.join(', ')}]`;
    };

    if (dualArmMode) {
        const robot1 = formatList(jointAngles1);
        const robot2 = formatList(jointAngles2);
        return `robot1=${robot1}\nrobot2=${robot2}`;
    }

    return formatList(jointAngles);
}

function parseJointAngleArray(arrayStr) {
    const trimmed = arrayStr.trim();
    if (!trimmed.startsWith('[') || !trimmed.endsWith(']')) {
        throw new Error('关节角度需使用 [ ] 表示数组');
    }
    try {
        const arr = JSON.parse(trimmed);
        if (!Array.isArray(arr)) {
            throw new Error('解析结果不是数组');
        }
        return arr.map((value) => {
            const num = Number(value);
            if (!Number.isFinite(num)) {
                throw new Error('数组中包含非数值项');
            }
            return num;
        });
    } catch (error) {
        throw new Error(`无法解析关节角度数组: ${error.message}`);
    }
}

function parseJointAnglesInput(text) {
    const trimmed = (text || '').trim();
    if (!trimmed) {
        throw new Error('未提供任何角度数据');
    }

    if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
        return { single: parseJointAngleArray(trimmed) };
    }

    const result = {};
    const normalized = trimmed.replace(/\r\n/g, '\n');
    const regex = /(robot1|robot2)\s*[:=]\s*(\[[^\]]*\])/gi;
    let match;
    while ((match = regex.exec(normalized)) !== null) {
        const key = match[1].toLowerCase();
        result[key] = parseJointAngleArray(match[2]);
    }

    if (Object.keys(result).length === 0) {
        throw new Error('未检测到 robot1 或 robot2 的角度数组');
    }

    return result;
}

function importJointAnglesFromText(text) {
    const parsed = parseJointAnglesInput(text);
    const toRad = (value) => (angleDisplayMode === 'deg' ? value * DEG_TO_RAD : value);

    if (dualArmMode) {
        const arr1 = parsed.robot1 || parsed.single;
        const arr2 = parsed.robot2;

        if (!arr1 || !arr2) {
            throw new Error('双臂模式需要同时提供 robot1 和 robot2 的角度数组');
        }

        if (arr1.length !== jointAngles1.length || arr2.length !== jointAngles2.length) {
            throw new Error('导入数组长度与当前机器人关节数不匹配');
        }

        jointAngles1 = arr1.map(toRad);
        jointAngles2 = arr2.map(toRad);

        if (window.customRobotConfig?.robot1) {
            const jointNames1 = window.customRobotConfig.robot1.joint_chain?.joints || [];
            jointAngles1 = clampJointAngles(jointAngles1, jointNames1, visualizer?.robot1URDF || null, window.customRobotConfig.robot1);
        }
        if (window.customRobotConfig?.robot2) {
            const jointNames2 = window.customRobotConfig.robot2.joint_chain?.joints || [];
            jointAngles2 = clampJointAngles(jointAngles2, jointNames2, visualizer?.robot2URDF || null, window.customRobotConfig.robot2);
        }

        updateGlobalVariables();
        updateRobotPose();
        updateDualArmJointSlidersFromAngles();
    } else {
        const arr = parsed.single || parsed.robot1 || parsed.robot2;
        if (!arr) {
            throw new Error('单臂模式需要提供一个角度数组');
        }
        if (arr.length !== jointAngles.length) {
            throw new Error('导入数组长度与当前机器人关节数不匹配');
        }

        jointAngles = arr.map(toRad);

        const config = getActiveJointConfig();
        const jointNames = config?.joint_chain?.joints || [];
        const urdfRobot = visualizer?.urdfRobot || null;
        if (jointNames.length === jointAngles.length) {
            jointAngles = clampJointAngles(jointAngles, jointNames, urdfRobot, config);
        }

        updateGlobalVariables();
        updateRobotPose();
        updateJointSlidersFromAngles();
    }
}

function formatJointLimitsForClipboard(lowerAngles = [], upperAngles = [], label = 'joint') {
    const listLower = Array.isArray(lowerAngles) ? lowerAngles : Array.from(lowerAngles || []);
    const listUpper = Array.isArray(upperAngles) ? upperAngles : Array.from(upperAngles || []);
    const digits = angleDisplayMode === 'deg' ? 3 : 5;
    const lowerDisplay = listLower.map(a => parseFloat(angleToDisplay(a ?? 0).toFixed(digits)));
    const upperDisplay = listUpper.map(a => parseFloat(angleToDisplay(a ?? 0).toFixed(digits)));
    const prefix = angleDisplayMode === 'deg' ? '' : '';
    return `${label}_q_min: [${lowerDisplay.join(', ')}]\n${label}_q_max: [${upperDisplay.join(', ')}]`;
}

function buildJointLimitsClipboardText() {
    const config = getActiveJointConfig();
    const formatLimits = (jointNames = [], urdfRobotRef = null, configRef = null, label = 'joint') => {
        const lower = [];
        const upper = [];
        jointNames.forEach(name => {
            const limits = getJointLimits(name, urdfRobotRef, configRef);
            lower.push(limits.lower);
            upper.push(limits.upper);
        });
        return formatJointLimitsForClipboard(lower, upper, label);
    };

    if (dualArmMode && config) {
        const jointNames1 = config.robot1?.joint_chain?.joints || [];
        const jointNames2 = config.robot2?.joint_chain?.joints || [];
        const text1 = formatLimits(jointNames1, visualizer?.robot1URDF || null, config.robot1, 'robot1');
        const text2 = formatLimits(jointNames2, visualizer?.robot2URDF || null, config.robot2, 'robot2');
        return `${text1}\n${text2}`;
    }

    const jointNames = config?.joint_chain?.joints || [];
    return formatLimits(jointNames, visualizer?.urdfRobot || null, config, 'joint');
}

async function copyJointLimitsToClipboard() {
    const text = buildJointLimitsClipboardText();
    if (!text) {
        throw new Error('无法获取关节限位');
    }

    if (navigator?.clipboard?.writeText) {
        await navigator.clipboard.writeText(text);
        return true;
    }

    const textarea = document.createElement('textarea');
    textarea.value = text;
    textarea.style.position = 'fixed';
    textarea.style.opacity = '0';
    document.body.appendChild(textarea);
    textarea.select();
    const succeeded = document.execCommand('copy');
    document.body.removeChild(textarea);
    if (!succeeded) {
        throw new Error('复制失败');
    }
    return true;
}

async function copyJointAnglesToClipboard() {
    const text = buildJointAnglesClipboardText();
    if (!text) {
        throw new Error('无法获取关节角度');
    }
    
    if (navigator?.clipboard?.writeText) {
        await navigator.clipboard.writeText(text);
        return true;
    }
    
    const textarea = document.createElement('textarea');
    textarea.value = text;
    textarea.style.position = 'fixed';
    textarea.style.opacity = '0';
    document.body.appendChild(textarea);
    textarea.select();
    const succeeded = document.execCommand('copy');
    document.body.removeChild(textarea);
    if (!succeeded) {
        throw new Error('复制失败');
    }
    return true;
}
/**
 * 计算末端固定偏移：查找紧邻 end_link 的固定关节的 origin
 * @param {Object} robot - URDF 机器人对象
 * @param {string} endLinkName - 末端连杆名
 * @returns {{x:number,y:number,z:number}|null}
 */
function computeEffectorOffsetFromURDF(robot, endLinkName) {
    try {
        if (!robot || !endLinkName) return null;
        const joints = robot.joints || {};
        // 查找 child==endLink 的 fixed 关节
        for (const name of Object.keys(joints)) {
            const j = joints[name];
            const isFixed = j.jointType === 'fixed' || j.type === 'fixed';
            const childName = j.child || j.childLink || (j.child && j.child.name);
            if (isFixed && childName === endLinkName) {
                const p = j.position || new THREE.Vector3();
                return { x: p.x || 0, y: p.y || 0, z: p.z || 0 };
            }
        }
        return null;
    } catch (e) {
        console.warn('computeEffectorOffsetFromURDF 失败:', e.message);
        return null;
    }
}

/**
 * 计算末端完整姿态（相对末端前一关节坐标系），Z-up
 * - 若末端还有一个或多个 fixed 关节（如手爪安装座），将它们的位姿合并
 */
function computeEffectorPoseFromURDF(robot, endLinkName) {
    if (!robot || !endLinkName) return null;
    // 找到 child==endLink 的最后一个 fixed 关节
    const joints = robot.joints || {};
    let target = null;
    let targetName = null;
    for (const name of Object.keys(joints)) {
        const j = joints[name];
        const isFixed = j.jointType === 'fixed' || j.type === 'fixed';
        const child = j.child || j.childLink || (j.child && j.child.name);
        if (isFixed && child === endLinkName) {
            target = j; // 记录最后一个匹配（更近的）
            targetName = name;
        }
    }
    if (!target) {
        console.log(`    未找到指向 ${endLinkName} 的fixed关节`);
        return null;
    }

    console.log(`    找到fixed关节: ${targetName}`);
    
    // 取其相对父的位姿（URDF原始坐标，Y-up）
    const posY = target.position ? target.position.clone() : new THREE.Vector3();
    const quatY = target.quaternion ? target.quaternion.clone() : new THREE.Quaternion();
    console.log(`    URDF Y-up 位移: (${posY.x.toFixed(4)}, ${posY.y.toFixed(4)}, ${posY.z.toFixed(4)})`);
    
    // 转 Z-up（Rx(-90°)）
    const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
    const posZ = posY.clone().applyQuaternion(qx);
    const quatZ = qx.clone().multiply(quatY);
    console.log(`    转换Z-up 位移: (${posZ.x.toFixed(4)}, ${posZ.y.toFixed(4)}, ${posZ.z.toFixed(4)})`);

    return {
        qw: quatZ.w, qx: quatZ.x, qy: quatZ.y, qz: quatZ.z,
        tx: posZ.x, ty: posZ.y, tz: posZ.z
    };
}

/**
 * 初始化应用
 */
async function init() {
    try {
        // 初始化 WASM 模块
        await initDQRobotics();
        
        // 隐藏加载界面
        document.getElementById('loading').classList.add('hidden');
        
        // 初始化可视化器
        await initVisualizer();
        
        // 设置事件监听器
        setupEventListeners();
        
        // 默认加载 ia_robot（如果没有自定义机器人）
        // 延迟一点执行，确保UI已准备好
        setTimeout(() => {
            initDefaultIARobot();
        }, 100);
        
        console.log('✓ 应用初始化完成');
    } catch (error) {
        console.error('初始化失败:', error);
        document.getElementById('loading').innerHTML = `
            <div style="color: #ef4444;">
                <h2>❌ 加载失败</h2>
                <p>${error.message}</p>
                <p style="font-size: 0.875rem; margin-top: 1rem;">
                    请确保已正确构建 WASM 模块
                </p>
            </div>
        `;
    }
}

/**
 * 默认初始化 ia_robot（如果未加载自定义机器人）
 */
async function initDefaultIARobot() {
    // 如果用户已经通过拖拽加载/或已经有机器人，则不覆盖
    if (customRobotConfig || dualArmMode || currentRobot) {
        return;
    }

    try {
        await loadBuiltInRobotFromPublic('ia_robot');
        updateRobotPose();
    } catch (error) {
        console.warn('⚠ 默认加载 ia_robot 失败，将回退到内置 Franka:', error);
        await initRobot('franka');
        updateRobotPose();
    }
}

/**
 * 从 public/robots/<robotDir>/ 自动加载内置 URDF 机器人（复用拖拽加载逻辑）
 * 目前用于默认加载 ia_robot。
 */
async function loadBuiltInRobotFromPublic(robotDir) {
    if (!visualizer) {
        throw new Error('可视化器未初始化');
    }

    // 在加载新模型前，先清理所有旧模型
    if (visualizer && typeof visualizer.cleanupOldRobots === 'function') {
        visualizer.cleanupOldRobots();
    }

    const baseUrl = import.meta.env.BASE_URL || '/';

    // ia_robot 是双臂包：这里显式列出其 public 资源（URDF/YAML/mesh）
    // 注意：path 必须以包名（ia_robot）作为第一级目录，以便 package://ia_robot/... 能正确匹配。
    if (robotDir !== 'ia_robot') {
        throw new Error(`暂不支持的内置机器人: ${robotDir}`);
    }

    const filesInPackage = [
        'robot_config.yaml',
        'right_arm/right_arm.urdf',
        'left_arm/left_arm.urdf',
        // meshes
        'meshes/arm_right.STL',
        'meshes/base_link_arm.STL',
        'meshes/base_link_body.STL',
        'meshes/base_link.STL',
        'meshes/baselink.obj',
        'meshes/body_link.STL',
        'meshes/car_link.STL',
        'meshes/leftlink1.obj',
        'meshes/leftlink2.obj',
        'meshes/leftlink3.obj',
        'meshes/leftlink4.obj',
        'meshes/leftlink5.obj',
        'meshes/leftlink6.obj',
        'meshes/leftlink7.obj',
        'meshes/link1.STL',
        'meshes/link2.STL',
        'meshes/link3.STL',
        'meshes/llink1.STL',
        'meshes/llink2.STL',
        'meshes/llink3.STL',
        'meshes/llink4.STL',
        'meshes/llink5.STL',
        'meshes/llink6.STL',
        'meshes/llink7.STL',
        'meshes/low_leg.STL',
        'meshes/ree_link.STL',
        'meshes/rightlink1.obj',
        'meshes/rightlink2.obj',
        'meshes/rightlink3.obj',
        'meshes/rightlink4.obj',
        'meshes/rightlink5.obj',
        'meshes/rightlink6.obj',
        'meshes/rightlink7.obj',
        'meshes/rlink1.STL',
        'meshes/rlink2.STL',
        'meshes/rlink3.STL',
        'meshes/rlink4.STL',
        'meshes/rlink5.STL',
        'meshes/rlink6.STL',
        'meshes/rlink7.STL',
        'meshes/up_leg.STL'
    ];

    packageManager = new URDFPackageManager();
    const remoteEntries = filesInPackage.map((rel) => {
        const path = `${robotDir}/${rel}`;
        const url = `${baseUrl}robots/${robotDir}/${rel}`;
        return { path, url };
    });
    await packageManager.addRemoteFiles(remoteEntries);

    // 读取配置
    customRobotConfig = packageManager.getConfig();
    if (!customRobotConfig) {
        throw new Error('未找到 robot_config.yaml 配置文件');
    }

    currentRobotType = robotDir;

    // 为了复用现有流程，仍然 load 一次（dualArmSystem 内部会按 config 再次加载指定 URDF）
    const robot = await loadURDFFromPackage(packageManager);

    const isDualArm = customRobotConfig?.dual_arm === true;
    dualArmMode = isDualArm;

    if (isDualArm) {
        const jointControlsContainer = document.getElementById('joint-controls');
        if (jointControlsContainer) {
            jointControlsContainer.innerHTML = '';
        }

        await loadDualArmSystem(robot, customRobotConfig);
        switchToDualArmUI();
        createDualArmJointControls(customRobotConfig);
    } else {
        const loaded = await visualizer.loadCustomURDF(robot, null, customRobotConfig);
        if (!loaded) {
            throw new Error('可视化器加载失败');
        }
        switchToSingleArmUI();

        if (customRobotConfig.mdh_extraction?.auto_extract) {
            await extractMDHFromCustomRobot(robot, customRobotConfig);
        }
    }

    refreshDockState();
}

/**
 * 初始化机器人
 */
async function initRobot(robotType) {
    currentRobotType = robotType;
    let frankaConfig = null;
    
    if (robotType === 'franka') {
        currentRobot = new FrankaPanda();
        currentRobotIsYUp = false; // 内置 Franka 为 Z-up
        updateGlobalVariables();
        
        // 尝试加载Franka配置文件以获取关节限位
        try {
            const baseUrl = import.meta.env.BASE_URL || '/';
            const response = await fetch(`${baseUrl}robots/franka/robot_config.yaml`);
            if (response.ok) {
                const yaml = await import('js-yaml');
                const text = await response.text();
                frankaConfig = yaml.load(text);
                console.log('✓ 已加载Franka配置文件，包含关节限位信息');
            }
        } catch (error) {
            console.warn('⚠ 无法加载Franka配置文件，将使用默认限位:', error);
        }
    } else if (robotType === 'kuka') {
        currentRobot = new KukaLw4();
        currentRobotIsYUp = false;
    }
    frankaConfigCache = robotType === 'franka' ? frankaConfig : null;
    
    const numJoints = currentRobot.getDimConfigurationSpace();
    
    // 初始化关节角度为零位或从配置文件读取
    if (frankaConfig && frankaConfig.initial_joint_angles) {
        jointAngles = [...frankaConfig.initial_joint_angles];
        // 限制初始角度在限位范围内
        const jointNames = frankaConfig.joint_chain?.joints || [];
        if (jointNames.length === jointAngles.length) {
            jointAngles = clampJointAngles(jointAngles, jointNames, null, frankaConfig);
        }
    } else {
        jointAngles = new Array(numJoints).fill(0);
    }
    updateGlobalVariables();
    
    // 创建关节控制滑块（传入配置以读取限位）
    createJointControls(numJoints, frankaConfig);
    
    // 更新滑块显示（使用限制后的角度）
    updateJointSlidersFromAngles();
}

/**
 * 获取关节限位（从URDF或配置文件）
 */
function getJointLimits(jointName, urdfRobot, config) {
    let lower = -Math.PI;  // 默认下限 -180度
    let upper = Math.PI;   // 默认上限 180度
    
    // 优先从配置文件读取
    if (config?.joint_limits && config.joint_limits[jointName]) {
        const limits = config.joint_limits[jointName];
        if (Array.isArray(limits) && limits.length >= 2) {
            lower = limits[0];
            upper = limits[1];
            console.log(`  ✓ 从配置文件读取 ${jointName} 限位: [${lower.toFixed(4)}, ${upper.toFixed(4)}]`);
            return { lower, upper };
        }
    }
    
    // 从URDF读取
    if (urdfRobot && urdfRobot.joints && urdfRobot.joints[jointName]) {
        const joint = urdfRobot.joints[jointName];
        
        // 尝试多种可能的属性名
        let limit = joint.limit || joint.limits || null;
        
        if (limit) {
            // 检查不同的属性结构
            if (limit.lower !== undefined && limit.upper !== undefined) {
                lower = parseFloat(limit.lower);
                upper = parseFloat(limit.upper);
                console.log(`  ✓ 从URDF读取 ${jointName} 限位: [${lower.toFixed(4)}, ${upper.toFixed(4)}]`);
            } else if (limit.min !== undefined && limit.max !== undefined) {
                lower = parseFloat(limit.min);
                upper = parseFloat(limit.max);
                console.log(`  ✓ 从URDF读取 ${jointName} 限位: [${lower.toFixed(4)}, ${upper.toFixed(4)}]`);
            }
        } else {
            // 尝试直接访问属性（urdf-loader可能将limit属性直接挂在joint上）
            if (joint.lower !== undefined && joint.upper !== undefined) {
                lower = parseFloat(joint.lower);
                upper = parseFloat(joint.upper);
                console.log(`  ✓ 从URDF读取 ${jointName} 限位: [${lower.toFixed(4)}, ${upper.toFixed(4)}]`);
            }
        }
    }
    
    return { lower, upper };
}

/**
 * 限制角度在关节限位范围内
 */
function clampJointAngles(angles, jointNames, urdfRobot, config) {
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
 * @param {number} lowerDeg - 下限（度）
 * @param {number} upperDeg - 上限（度）
 * @param {Function} onUpdate - 更新回调函数，接收角度（弧度）
 */
function setupJointInput(input, sliderId, lowerRad, upperRad, onUpdate) {
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
function createJointControls(numJoints, config = null) {
    const container = document.getElementById('joint-controls');
    container.innerHTML = '';
    
    // 如果是双臂模式，创建双臂关节控制
    if (dualArmMode && config) {
        createDualArmJointControls(config);
        return;
    }
    
    // 获取URDF机器人对象（用于读取关节限位）
    const urdfRobot = visualizer?.urdfRobot || null;
    
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
        input.value = formatDisplayAngle(jointAngles[i] ?? 0);
        
        // 输入框事件处理
        setupJointInput(input, `joint-${i}`, lowerRad, upperRad, (angleRad) => {
            jointAngles[i] = angleRad;
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            updateRobotPose();
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
        slider.value = formatDisplayValue(angleToDisplay(jointAngles[i] ?? 0), sliderPrecision);
        
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) {
                return;
            }
            const angleRad = displayToAngle(displayValue);
            jointAngles[i] = angleRad;
            
            // 更新输入框
            input.value = formatDisplayValue(displayValue, inputPrecision);
            
            // 同步全局变量
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            updateRobotPose();
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
function updateJointSlidersFromAngles() {
    if (!jointAngles || jointAngles.length === 0) return;
    
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const step = displayStep().toString();
    const placeholder = getAnglePlaceholder();
    
    jointAngles.forEach((angle, i) => {
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
function createDualArmJointControls(config) {
    const container = document.getElementById('joint-controls');
    // 清除旧的关节控制
    container.innerHTML = '';
    
    // 获取URDF机器人对象（用于读取关节限位）
    const robot1URDF = visualizer?.robot1URDF || null;
    const robot2URDF = visualizer?.robot2URDF || null;
    
    // 机器人1的关节控制
    const robot1Section = document.createElement('div');
    robot1Section.className = 'joint-control-section';
    robot1Section.innerHTML = `<h3 style="color: #3b82f6; margin-bottom: 0.5rem; font-size: 1rem;">${config.robot1?.name || '机器人1'}</h3>`;
    
    const robot1Controls = document.createElement('div');
    const numJoints1 = config.robot1?.joint_chain?.joints?.length || jointAngles1.length;
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
        input.value = formatDisplayAngle(jointAngles1[i] ?? 0);
        
        // 输入框事件处理（与单臂模式相同的逻辑）
        setupJointInput(input, `joint1-${i}`, lowerRad, upperRad, (angleRad) => {
            jointAngles1[i] = angleRad;
            updateRobotPose();
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
        slider.value = formatDisplayValue(angleToDisplay(jointAngles1[i] ?? 0), sliderPrecision);
        
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) {
                return;
            }
            const angleRad = displayToAngle(displayValue);
            jointAngles1[i] = angleRad;
            input.value = formatDisplayValue(displayValue, inputPrecision);
            updateRobotPose();
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
    const numJoints2 = config.robot2?.joint_chain?.joints?.length || jointAngles2.length;
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
        input.value = formatDisplayAngle(jointAngles2[i] ?? 0);
        
        // 输入框事件处理
        setupJointInput(input, `joint2-${i}`, lowerRad, upperRad, (angleRad) => {
            jointAngles2[i] = angleRad;
            updateRobotPose();
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
        slider.value = formatDisplayValue(angleToDisplay(jointAngles2[i] ?? 0), sliderPrecision);
        
        slider.addEventListener('input', (e) => {
            const displayValue = parseFloat(e.target.value);
            if (Number.isNaN(displayValue)) {
                return;
            }
            const angleRad = displayToAngle(displayValue);
            jointAngles2[i] = angleRad;
            input.value = formatDisplayValue(displayValue, inputPrecision);
            updateRobotPose();
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
 * 从当前双臂关节角度更新所有滑块的值和显示
 */
function updateDualArmJointSlidersFromAngles() {
    const sliderPrecision = getSliderPrecision();
    const inputPrecision = getInputPrecision();
    const step = displayStep().toString();
    const placeholder = getAnglePlaceholder();
    
    // 更新机器人1的滑块和输入框
    if (jointAngles1 && jointAngles1.length > 0) {
        jointAngles1.forEach((angle, i) => {
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
    if (jointAngles2 && jointAngles2.length > 0) {
        jointAngles2.forEach((angle, i) => {
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

function refreshJointControlDisplay() {
    const container = document.getElementById('joint-controls');
    if (!container) {
        return;
    }
    
    const config = getActiveJointConfig();
    
    if (dualArmMode && config) {
        const totalJoints =
            (config.robot1?.joint_chain?.joints?.length || jointAngles1.length) +
            (config.robot2?.joint_chain?.joints?.length || jointAngles2.length);
        createJointControls(totalJoints, config);
    } else {
        const numJoints =
            jointAngles.length ||
            config?.joint_chain?.joints?.length ||
            (currentRobot?.getDimConfigurationSpace?.() ?? 0);
        
        if (numJoints > 0) {
            createJointControls(numJoints, config);
        } else {
            container.innerHTML = '';
        }
    }
}

/**
 * 初始化可视化器
 */
async function initVisualizer() {
    const canvas = document.getElementById('robot-canvas');
    visualizer = new RobotVisualizer(canvas);
    window.visualizer = visualizer;
    await visualizer.init();
    
    // 根据UI初始化点动乘法顺序
    const teachOrderToggle = document.getElementById('teach-multiply-order-toggle');
    if (teachOrderToggle && visualizer?.setTeachMultiplicationOrder) {
        const initialOrder = teachOrderToggle.checked ? 'delta-first' : 'current-first';
        visualizer.setTeachMultiplicationOrder(initialOrder);
    }
}

/**
 * 更新机器人位姿（支持单臂和双臂模式）
 */
function updateRobotPose() {
    if (dualArmMode) {
        updateDualArmPose();
    } else {
        updateSingleArmPose();
    }
}

// 暴露到全局作用域
window.updateRobotPose = updateRobotPose;

// 更新全局变量（当这些变量改变时）
function updateGlobalVariables() {
    window.currentRobot = currentRobot;
    window.jointAngles = jointAngles;  // 保持引用一致
    window.customRobotConfig = customRobotConfig;
    // 暴露 DQModule 供 visualizer 使用
    window.DQModule = getDQModule();
    // 暴露 updateGlobalVariables 供 visualizer 调用
    window.updateGlobalVariables = updateGlobalVariables;
    // 暴露 clampJointAngles 供 visualizer 使用
    window.clampJointAngles = clampJointAngles;
    // 暴露双臂相关变量
    window.dualArmSystem = dualArmSystem;
    window.jointAngles1 = jointAngles1;
    window.jointAngles2 = jointAngles2;
    window.dualArmMode = dualArmMode;
    window.robot1 = robot1;
    window.robot2 = robot2;
    window.robot1URDF = robot1URDF;
    window.robot2URDF = robot2URDF;
}

/**
 * 更新单臂机器人位姿
 */
function updateSingleArmPose() {
    try {
        // 确保使用最新的关节角度（如果 window.jointAngles 被修改，同步到本地）
        if (window.jointAngles && window.jointAngles.length === jointAngles.length) {
            // 检查是否有差异，如果有则同步
            for (let i = 0; i < jointAngles.length; i++) {
                if (Math.abs(jointAngles[i] - window.jointAngles[i]) > 1e-8) {
                    jointAngles = window.jointAngles;  // 使用 window.jointAngles 的引用
                    break;
                }
            }
        }
        
        // 计算正向运动学
        let pose = currentRobot.getEndEffectorPose(jointAngles);
        
        // 创建用于显示的位姿副本（可能需要坐标转换）
        let displayPose = pose;
        if (currentRobotIsYUp && pose && pose.translation) {
            const x = pose.translation[1];
            const y = pose.translation[2];
            const z = pose.translation[3];
            displayPose = {
                translation: [pose.translation[0], x, z, y],
                rotation: pose.rotation
            };
        }
        
        // 更新位姿信息显示
        updatePoseDisplay(displayPose);
        
        // 更新 DQ 显示
        updateDQDisplay();
        
        // 更新雅可比矩阵显示
        updateJacobianDisplay();
        
        // 更新 3D 可视化
        if (visualizer) {
            const orderedNames = customRobotConfig?.joint_chain?.joints || [];
            // 调试：打印 DQ 返回的原始坐标
            if (pose && pose.translation) {
                const [w, x, y, z] = pose.translation;
                console.log(`[DQ fkm] translation: [${w.toFixed(3)}, ${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}]`);
            }
            visualizer.updateRobotPose(jointAngles, pose, orderedNames);
        }

        // 同步关节滑块和输入框显示
        updateJointSlidersFromAngles();
    } catch (error) {
        console.error('更新单臂位姿失败:', error);
    }
}

/**
 * 更新双臂机器人位姿
 */
function updateDualArmPose() {
    try {
        if (!robot1 || !robot2) {
            console.warn('双臂机器人未初始化');
            return;
        }
        
        // 计算两个机器人的位姿
        const pose1 = robot1.getEndEffectorPose(jointAngles1);
        const pose2 = robot2.getEndEffectorPose(jointAngles2);
        
        // 更新 3D 可视化
        if (visualizer) {
            const jointNames1 = customRobotConfig?.robot1?.joint_chain?.joints || [];
            const jointNames2 = customRobotConfig?.robot2?.joint_chain?.joints || [];
            
            visualizer.updateRobotPose(jointAngles1, pose1, jointNames1, 0);
            visualizer.updateRobotPose(jointAngles2, pose2, jointNames2, 1);
            
            // 使用 absolute_pose 函数计算绝对位姿
            // 合并两个机器人的关节角度：[robot1_joints..., robot2_joints...]
            const allJointAngles = [...jointAngles1, ...jointAngles2];
            
            let absolutePose = null;
            
            if (dualArmSystem && dualArmSystem.getAbsoluteWorldPose) {
                // 使用absolute_pose函数计算
                const absPose = dualArmSystem.getAbsoluteWorldPose(allJointAngles);
                
                // 解析DQ结果：前4个是四元数，translation包含位置信息
                absolutePose = {
                    rotation: absPose.rotation,  // 前4个元素：四元数 [qw, qx, qy, qz]
                    translation: absPose.translation  // [w, x, y, z]，其中x,y,z是位置
                };
                
                console.log('  ✓ 使用absolute_pose函数计算绝对位姿');
            } else {
                // 回退到原来的简单计算方式
                const x1 = pose1.translation[1] || 0;
                const y1 = pose1.translation[2] || 0;
                const z1 = pose1.translation[3] || 0;
                const x2 = pose2.translation[1] || 0;
                const y2 = pose2.translation[2] || 0;
                const z2 = pose2.translation[3] || 0;
                
                absolutePose = {
                    translation: [
                        1, // DQ实数部分
                        (x1 + x2) / 2,
                        (y1 + y2) / 2,
                        (z1 + z2) / 2
                    ],
                    rotation: [
                        (pose1.rotation[0] + pose2.rotation[0]) / 2,
                        (pose1.rotation[1] + pose2.rotation[1]) / 2,
                        (pose1.rotation[2] + pose2.rotation[2]) / 2,
                        (pose1.rotation[3] + pose2.rotation[3]) / 2
                    ]
                };
            }
            
            // 相对位姿（从robot1到robot2的变换）
            visualizer.updateDualArmRelativeVisualizations(pose1, pose2, absolutePose, null);
        }
        
        // 更新 UI 显示
        updateDualArmPoseDisplay(pose1, pose2);

        // 同步双臂关节滑块和输入框显示
        updateDualArmJointSlidersFromAngles();
        
    } catch (error) {
        console.error('更新双臂位姿失败:', error);
    }
}

/**
 * 加载双臂机器人系统
 */
/**
 * 加载指定的URDF文件
 */
async function loadSpecificURDF(urdfFile, urdfPath, config) {
    // 查找URDF文件
    let urdfFilePath = null;
    const allUrdfFiles = Array.from(packageManager.files.keys()).filter(name => 
        name.endsWith('.urdf') && (
            name.includes(urdfFile) || 
            (urdfPath && name.includes(urdfPath))
        )
    );
    
    if (allUrdfFiles.length > 0) {
        // 优先选择路径中包含 urdf_path 的文件
        urdfFilePath = allUrdfFiles.find(name => 
            urdfPath && name.includes(urdfPath)
        ) || allUrdfFiles[0];
    }
    
    if (!urdfFilePath) {
        console.warn(`  未找到URDF文件: ${urdfFile}`);
        console.log('  可用URDF文件:', Array.from(packageManager.files.keys()).filter(n => n.endsWith('.urdf')));
        throw new Error(`未找到URDF文件: ${urdfFile}`);
    }
    
    console.log(`  ✓ 找到 URDF: ${urdfFilePath}`);
    
    // 创建包管理器（复用文件）
    const robotPackageManager = new URDFPackageManager();
    robotPackageManager.files = packageManager.files;
    robotPackageManager.packages = packageManager.packages;
    robotPackageManager.config = config;
    
    // 临时覆盖 findURDFFile 方法
    const originalFindURDFFile = robotPackageManager.findURDFFile;
    robotPackageManager.findURDFFile = () => urdfFilePath;
    
    const robotURDFObj = await loadURDFFromPackage(robotPackageManager);
    
    // 恢复原始方法
    robotPackageManager.findURDFFile = originalFindURDFFile;
    
    return robotURDFObj;
}

async function loadDualArmSystem(firstRobot, config) {
    try {
        const dualArmType = config.dual_arm_type || 'two_urdf'; // 默认为two_urdf格式
        console.log(`  → 检测到双臂格式: ${dualArmType}`);
        
        let robot1URDFObj, robot2URDFObj;
        const robot1Config = config.robot1;
        const robot2Config = config.robot2;
        
        if (dualArmType === 'single_urdf') {
            // 单URDF格式：只加载一次URDF文件（包含两条链）
            console.log('  → 加载共享URDF文件（单文件双链格式）...');
            const sharedURDF = config.shared_urdf;
            if (!sharedURDF || !sharedURDF.urdf_file) {
                throw new Error('单URDF格式需要配置shared_urdf.urdf_file');
            }
            
            robot1URDFObj = await loadSpecificURDF(
                sharedURDF.urdf_file,
                sharedURDF.urdf_path || 'ia_robot',
                config
            );
            robot2URDFObj = robot1URDFObj; // 使用同一个URDF对象
            
            console.log('  ✓ 共享URDF文件加载成功');
        } else {
            // 双URDF格式：加载两个独立的URDF文件
            console.log('  → 加载机器人1 URDF...');
            robot1URDFObj = await loadSpecificURDF(
                robot1Config.urdf_file, 
                robot1Config.urdf_path,
                robot1Config
            );
            
            console.log('  → 加载机器人2 URDF...');
            robot2URDFObj = await loadSpecificURDF(
                robot2Config.urdf_file,
                robot2Config.urdf_path,
                robot2Config
            );
        }
        
        // 提取两个机器人的 MDH 参数（对于single_urdf格式，从同一个URDF提取两条链）
        console.log('  → 提取机器人1的MDH...');
        await extractMDHFromDualArmRobot(robot1URDFObj, robot1Config, 1);
        console.log('  → 提取机器人2的MDH...');
        await extractMDHFromDualArmRobot(robot2URDFObj, robot2Config, 2);
        
        // 创建CooperativeDualArm对象用于计算绝对位姿
        // 需要从robot1和robot2获取MDH矩阵
        // 注意：这里需要提取MDH矩阵，CooperativeDualArm构造函数需要MDH矩阵数组
        try {
            // 重新提取MDH用于创建CooperativeDualArm
            const robot1JointNames = robot1Config.joint_chain?.joints || [];
            const robot2JointNames = robot2Config.joint_chain?.joints || [];
            const getMDHFromResult = (result) => {
                // 处理新的返回格式（对象）或旧格式（数组）
                if (result && typeof result === 'object' && result.mdh) {
                    return result.mdh;
                }
                return result; // 向后兼容
            };
            
            const mdhResult1 = robot1Config.mdh_extraction?.auto_extract !== false ? 
                extractMDHFromURDF(robot1URDFObj, robot1JointNames, false, robot1Config.joint_chain?.end_link, 
                    robot1Config.mdh_extraction?.base_frame ? {
                        origin: robot1Config.mdh_extraction.base_frame.origin || [0, 0, 0],
                        z_axis: robot1Config.mdh_extraction.base_frame.z_axis || [0, 0, 1],
                        x_axis: robot1Config.mdh_extraction.base_frame.x_axis || [1, 0, 0]
                    } : null,
                    robot1Config.joint_chain?.base_link || null) : 
                { mdh: robot1Config.mdh_extraction?.manual_mdh, effectorPose: null };
            const mdhResult2 = robot2Config.mdh_extraction?.auto_extract !== false ? 
                extractMDHFromURDF(robot2URDFObj, robot2JointNames, false, robot2Config.joint_chain?.end_link,
                    robot2Config.mdh_extraction?.base_frame ? {
                        origin: robot2Config.mdh_extraction.base_frame.origin || [0, 0, 0],
                        z_axis: robot2Config.mdh_extraction.base_frame.z_axis || [0, 0, 1],
                        x_axis: robot2Config.mdh_extraction.base_frame.x_axis || [1, 0, 0]
                    } : null,
                    robot2Config.joint_chain?.base_link || null) : 
                { mdh: robot2Config.mdh_extraction?.manual_mdh, effectorPose: null };
            
            const mdh1 = getMDHFromResult(mdhResult1);
            const mdh2 = getMDHFromResult(mdhResult2);

            if (mdhResult1 && typeof mdhResult1 === 'object' && mdhResult1.autoBasePose && !Array.isArray(robot1Config._autoBasePose)) {
                robot1Config._autoBasePose = mdhResult1.autoBasePose;
            }
            if (mdhResult2 && typeof mdhResult2 === 'object' && mdhResult2.autoBasePose && !Array.isArray(robot2Config._autoBasePose)) {
                robot2Config._autoBasePose = mdhResult2.autoBasePose;
            }
            
            if (mdh1 && mdh2) {
                // 格式化MDH为CooperativeDualArm需要的格式
                const formatMDH = (mdh) => {
                    const numJoints = mdh.length;
                    const theta_row = mdh.map(p => p[0]);
                    const d_row = mdh.map(p => p[1]);
                    const a_row = mdh.map(p => p[2]);
                    const alpha_row = mdh.map(p => p[3]);
                    const type_row = new Array(numJoints).fill(0);
                    return [theta_row, d_row, a_row, alpha_row, type_row];
                };
                
                const mdhMatrix1 = formatMDH(mdh1);
                const mdhMatrix2 = formatMDH(mdh2);
                
                dualArmSystem = new CooperativeDualArm(mdhMatrix1, mdhMatrix2);
                
                // 设置基座位姿
                let robot1BasePose = robot1Config.base_pose;
                if ((!robot1BasePose || !Array.isArray(robot1BasePose) || robot1BasePose.length < 3) && Array.isArray(robot1Config._autoBasePose)) {
                    robot1BasePose = robot1Config._autoBasePose;
                    console.log('  机器人1未配置基座位姿，使用自动估计值');
                }
                if (robot1BasePose && robot1BasePose.length >= 3) {
                    if (robot1BasePose.length === 8) {
                        console.log(`  设置机器人1基座 (双四元数格式):`);
                        console.log(`    DQ: [${robot1BasePose.map(v => v.toFixed(6)).join(', ')}]`);
                        dualArmSystem.setRobot1BaseDQ(robot1BasePose);
                        console.log('  ✓ 机器人1基座双四元数已应用');
                    } else if (robot1BasePose.length === 7) {
                        const [qw, qx, qy, qz, tx, ty, tz] = robot1BasePose;
                        console.log(`  设置机器人1基座位姿 (Z-up):`);
                        console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                        console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                        dualArmSystem.setRobot1BasePose(qw, qx, qy, qz, tx, ty, tz);
                        console.log('  ✓ 机器人1基座完整位姿已应用');
                    } else if (robot1BasePose.length === 4) {
                        const [, tx, ty, tz] = robot1BasePose;
                        console.log(`  设置机器人1基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        dualArmSystem.setRobot1Base(tx, ty, tz);
                        console.log('  ✓ 机器人1基座偏移已应用');
                    } else {
                        const [tx, ty, tz] = robot1BasePose;
                        console.log(`  设置机器人1基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        dualArmSystem.setRobot1Base(tx, ty, tz);
                        console.log('  ✓ 机器人1基座偏移已应用');
                    }
                }
                
                let robot2BasePose = robot2Config.base_pose;
                if ((!robot2BasePose || !Array.isArray(robot2BasePose) || robot2BasePose.length < 3) && Array.isArray(robot2Config._autoBasePose)) {
                    robot2BasePose = robot2Config._autoBasePose;
                    console.log('  机器人2未配置基座位姿，使用自动估计值');
                }
                if (robot2BasePose && robot2BasePose.length >= 3) {
                    if (robot2BasePose.length === 8) {
                        console.log(`  设置机器人2基座 (双四元数格式):`);
                        console.log(`    DQ: [${robot2BasePose.map(v => v.toFixed(6)).join(', ')}]`);
                        dualArmSystem.setRobot2BaseDQ(robot2BasePose);
                        console.log('  ✓ 机器人2基座双四元数已应用');
                    } else if (robot2BasePose.length === 7) {
                        const [qw, qx, qy, qz, tx, ty, tz] = robot2BasePose;
                        console.log(`  设置机器人2基座位姿 (Z-up):`);
                        console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                        console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                        dualArmSystem.setRobot2BasePose(qw, qx, qy, qz, tx, ty, tz);
                        console.log('  ✓ 机器人2基座完整位姿已应用');
                    } else if (robot2BasePose.length === 4) {
                        const [, tx, ty, tz] = robot2BasePose;
                        console.log(`  设置机器人2基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        dualArmSystem.setRobot2Base(tx, ty, tz);
                        console.log('  ✓ 机器人2基座偏移已应用');
                    } else {
                        const [tx, ty, tz] = robot2BasePose;
                        console.log(`  设置机器人2基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        dualArmSystem.setRobot2Base(tx, ty, tz);
                        console.log('  ✓ 机器人2基座偏移已应用');
                    }
                }
                
                // 设置末端执行器偏移
                // 优先使用从MDH提取中获得的effectorPose
                let robot1EffPose = null;
                if (mdhResult1 && typeof mdhResult1 === 'object' && mdhResult1.effectorPose) {
                    robot1EffPose = mdhResult1.effectorPose;
                    console.log('  ✓ 机器人1: 使用从MDH提取中获得的末端执行器偏移');
                } else if (robot1Config.mdh_extraction?.effector_offset) {
                    const offset = robot1Config.mdh_extraction.effector_offset;
                    if (offset === null) {
                        console.log('  ⊘ 机器人1: 末端偏移已禁用（配置文件设置为null）');
                    } else if (offset.length === 3) {
                        robot1EffPose = { qw: 1, qx: 0, qy: 0, qz: 0, tx: offset[0], ty: offset[1], tz: offset[2] };
                    } else if (offset.length === 7) {
                        robot1EffPose = { qw: offset[0], qx: offset[1], qy: offset[2], qz: offset[3], tx: offset[4], ty: offset[5], tz: offset[6] };
                    }
                } else if (robot1Config.effector_offset) {
                    // 向后兼容：直接在robot1Config下
                    const offset = robot1Config.effector_offset;
                    if (offset.length === 3) {
                        robot1EffPose = { qw: 1, qx: 0, qy: 0, qz: 0, tx: offset[0], ty: offset[1], tz: offset[2] };
                    } else if (offset.length === 7) {
                        robot1EffPose = { qw: offset[0], qx: offset[1], qy: offset[2], qz: offset[3], tx: offset[4], ty: offset[5], tz: offset[6] };
                    }
                }
                
                if (robot1EffPose) {
                    console.log('  机器人1末端偏移 (Z-up):');
                    console.log(`    平移: (${robot1EffPose.tx.toFixed(4)}, ${robot1EffPose.ty.toFixed(4)}, ${robot1EffPose.tz.toFixed(4)})`);
                    console.log(`    四元数: (${robot1EffPose.qw.toFixed(4)}, ${robot1EffPose.qx.toFixed(4)}, ${robot1EffPose.qy.toFixed(4)}, ${robot1EffPose.qz.toFixed(4)})`);
                    dualArmSystem.setRobot1EffectorPose(
                        robot1EffPose.qw, robot1EffPose.qx, robot1EffPose.qy, robot1EffPose.qz,
                        robot1EffPose.tx, robot1EffPose.ty, robot1EffPose.tz
                    );
                    console.log('  ✓ 机器人1末端偏移已应用');
                }
                
                let robot2EffPose = null;
                if (mdhResult2 && typeof mdhResult2 === 'object' && mdhResult2.effectorPose) {
                    robot2EffPose = mdhResult2.effectorPose;
                    console.log('  ✓ 机器人2: 使用从MDH提取中获得的末端执行器偏移');
                } else if (robot2Config.mdh_extraction?.effector_offset) {
                    const offset = robot2Config.mdh_extraction.effector_offset;
                    if (offset === null) {
                        console.log('  ⊘ 机器人2: 末端偏移已禁用（配置文件设置为null）');
                    } else if (offset.length === 3) {
                        robot2EffPose = { qw: 1, qx: 0, qy: 0, qz: 0, tx: offset[0], ty: offset[1], tz: offset[2] };
                    } else if (offset.length === 7) {
                        robot2EffPose = { qw: offset[0], qx: offset[1], qy: offset[2], qz: offset[3], tx: offset[4], ty: offset[5], tz: offset[6] };
                    }
                } else if (robot2Config.effector_offset) {
                    // 向后兼容：直接在robot2Config下
                    const offset = robot2Config.effector_offset;
                    if (offset.length === 3) {
                        robot2EffPose = { qw: 1, qx: 0, qy: 0, qz: 0, tx: offset[0], ty: offset[1], tz: offset[2] };
                    } else if (offset.length === 7) {
                        robot2EffPose = { qw: offset[0], qx: offset[1], qy: offset[2], qz: offset[3], tx: offset[4], ty: offset[5], tz: offset[6] };
                    }
                }
                
                if (robot2EffPose) {
                    console.log('  机器人2末端偏移 (Z-up):');
                    console.log(`    平移: (${robot2EffPose.tx.toFixed(4)}, ${robot2EffPose.ty.toFixed(4)}, ${robot2EffPose.tz.toFixed(4)})`);
                    console.log(`    四元数: (${robot2EffPose.qw.toFixed(4)}, ${robot2EffPose.qx.toFixed(4)}, ${robot2EffPose.qy.toFixed(4)}, ${robot2EffPose.qz.toFixed(4)})`);
                    dualArmSystem.setRobot2EffectorPose(
                        robot2EffPose.qw, robot2EffPose.qx, robot2EffPose.qy, robot2EffPose.qz,
                        robot2EffPose.tx, robot2EffPose.ty, robot2EffPose.tz
                    );
                    console.log('  ✓ 机器人2末端偏移已应用');
                }
                
                console.log('  ✓ CooperativeDualArm对象已创建');
                
                // 更新全局变量，确保 dualArmSystem 被暴露
                updateGlobalVariables();
            }
        } catch (error) {
            console.warn('  ⚠ 创建CooperativeDualArm失败，将使用简单计算方式:', error);
            dualArmSystem = null;
            // 即使创建失败，也要更新全局变量
            updateGlobalVariables();
        }
        
        // 加载到可视化器
        console.log('  → 加载到 3D 场景...');
        if (dualArmType === 'single_urdf') {
            // 单URDF格式：只传入一个URDF对象，但传递dualArmType标识
            await visualizer.loadCustomURDF(robot1URDFObj, null, config, 'single_urdf');
        } else {
            // 双URDF格式：传入两个URDF对象
            await visualizer.loadCustomURDF(robot1URDFObj, robot2URDFObj, config, 'two_urdf');
        }
        
        // 初始化关节角度
        jointAngles1 = config.robot1?.initial_joint_angles || [];
        jointAngles2 = robot2Config?.initial_joint_angles || [];
        
        // 限制初始角度在限位范围内
        const robot1JointNames = config.robot1?.joint_chain?.joints || [];
        const robot2JointNames = robot2Config?.joint_chain?.joints || [];
        
        if (robot1JointNames.length === jointAngles1.length) {
            jointAngles1 = clampJointAngles(jointAngles1, robot1JointNames, robot1URDFObj, config.robot1);
        }
        if (robot2JointNames.length === jointAngles2.length) {
            jointAngles2 = clampJointAngles(jointAngles2, robot2JointNames, robot2URDFObj, robot2Config);
        }
        
        console.log(`✓ 双臂系统加载完成`);
        console.log(`  机器人1: ${jointAngles1.length} 个关节`);
        console.log(`  机器人2: ${jointAngles2.length} 个关节`);
        
        // 保存URDF对象引用（用于更新关节角度）
        robot1URDF = robot1URDFObj;
        robot2URDF = robot2URDFObj;
        
        // 设置visualizer的URDF引用
        if (visualizer) {
            visualizer.robot1URDF = robot1URDFObj;
            visualizer.robot2URDF = robot2URDFObj;
            
            // 初始化URDF坐标系显示（如果开关已打开）- 双臂模式同时初始化两个机器人
            if (visualizer.showURDFFrames) {
                // 双臂模式：同时初始化两个机器人的坐标系
                const robot1Links = robot1URDFObj && robot1URDFObj.links ? Object.keys(robot1URDFObj.links) : [];
                const robot2Links = robot2URDFObj && robot2URDFObj.links ? Object.keys(robot2URDFObj.links) : [];
                visualizer.initURDFFrames(null, null, robot1URDFObj, robot2URDFObj);
            }
        }
        
        // 设置双臂模式标志
        dualArmMode = true;
        
        // 更新全局变量，确保所有双臂相关变量都被暴露
        updateGlobalVariables();
        
        // 更新双臂滑块显示（使用限制后的角度）
        // 延迟执行，确保关节控制器已创建
        setTimeout(() => {
            updateDualArmJointSlidersFromAngles();
        }, 150);
        
        // 初始更新位姿显示
        setTimeout(() => {
            try {
                updateRobotPose();
            } catch (error) {
                console.warn('初始位姿更新失败（可忽略）:', error.message);
            }
        }, 100);
        
    } catch (error) {
        console.error('加载双臂系统失败:', error);
        console.error('错误堆栈:', error.stack);
        throw error;
    }
}

/**
 * 从双臂机器人提取 MDH（辅助函数）
 */
async function extractMDHFromDualArmRobot(urdfRobot, robotConfig, robotIndex) {
    try {
        const jointNames = robotConfig.joint_chain?.joints || [];
        const mdhConfig = robotConfig.mdh_extraction || {};
        const baseLinkName = robotConfig.joint_chain?.base_link || null;
        
        let mdh;
        let effectorPose = null;
        
        if (mdhConfig.auto_extract !== false) {
            const endLinkName = robotConfig.joint_chain?.end_link;
            // 读取基座配置（如果配置了）
            let baseConfig = null;
            if (robotConfig.mdh_extraction?.base_frame) {
                const baseFrame = robotConfig.mdh_extraction.base_frame;
                baseConfig = {
                    origin: baseFrame.origin || [0, 0, 0],
                    z_axis: baseFrame.z_axis || [0, 0, 1],
                    x_axis: baseFrame.x_axis || [1, 0, 0]
                };
                console.log(`  ✓ 机器人${robotIndex}使用配置的基座坐标系`);
            }
            
            const result = extractMDHFromURDF(urdfRobot, jointNames, false, endLinkName, baseConfig, baseLinkName);
            
            // 处理新的返回格式（对象）或旧格式（数组，向后兼容）
            if (result && typeof result === 'object' && result.mdh) {
                mdh = result.mdh;
                effectorPose = result.effectorPose;
                if (result.autoBasePose) {
                    robotConfig._autoBasePose = result.autoBasePose;
                    console.log(`  ✓ 机器人${robotIndex}自动估计基座位姿已记录`);
                }
            } else {
                // 向后兼容：如果返回的是数组
                mdh = result;
            }
        } else if (mdhConfig.manual_mdh) {
            mdh = mdhConfig.manual_mdh;
        } else {
            throw new Error(`机器人${robotIndex}未提供MDH参数`);
        }
        
        // 创建 DQ 机器人
        const numJoints = mdh.length;
        const theta_row = mdh.map(p => p[0]);
        const d_row = mdh.map(p => p[1]);
        const a_row = mdh.map(p => p[2]);
        const alpha_row = mdh.map(p => p[3]);
        const type_row = new Array(numJoints).fill(0);
        const mdhMatrix = [theta_row, d_row, a_row, alpha_row, type_row];
        
        const dqRobot = new SerialManipulator(mdhMatrix);
        
        // 设置基座位姿
        // 优先从 mdh_extraction.base_pose 读取，如果不存在则从 robotConfig.base_pose 读取
        let basePose = robotConfig.mdh_extraction?.base_pose || robotConfig.base_pose;
        
        // 如果 basePose 是字符串，尝试解析为数组
        if (typeof basePose === 'string') {
            try {
                basePose = JSON.parse(basePose);
            } catch (e) {
                console.warn('  无法解析 base_pose 字符串:', basePose);
            }
        }
        if ((!basePose || !Array.isArray(basePose) || basePose.length < 3) && Array.isArray(robotConfig._autoBasePose)) {
            basePose = robotConfig._autoBasePose;
            console.log(`  机器人${robotIndex}未配置基座位姿，使用自动估计值`);
        }
        
        if (basePose && Array.isArray(basePose) && basePose.length >= 3) {
            if (basePose.length === 8) {
                // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                console.log(`  设置基座 (双四元数格式):`);
                console.log(`    DQ: [${basePose.map(v => v.toFixed(6)).join(', ')}]`);
                dqRobot.setBaseFrameDQ(basePose);
                console.log('  ✓ 基座双四元数已应用');
            } else if (basePose.length === 7) {
                // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                const [qw, qx, qy, qz, tx, ty, tz] = basePose;
                console.log(`  设置基座位姿 (Z-up):`);
                console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                dqRobot.setBaseFramePose(qw, qx, qy, qz, tx, ty, tz);
                console.log('  ✓ 基座完整位姿已应用');
            } else if (basePose.length === 4) {
                // 可能是 [qw, tx, ty, tz] 格式，取后3个
                const [, tx, ty, tz] = basePose;
                console.log(`  设置基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                dqRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                console.log('  ✓ 基座偏移已应用');
            } else {
                // 仅平移 [tx, ty, tz]
                const [tx, ty, tz] = basePose;
                console.log(`  设置基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                dqRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                console.log('  ✓ 基座偏移已应用');
            }
        } else {
            console.log('  ⊘ 基座偏移未配置');
        }
        
        // 设置末端执行器偏移
        let effPose = effectorPose; // 优先使用从MDH提取中获得的effectorPose
        
        // 1. 优先使用配置文件中的手动偏移（覆盖自动提取的）
        // 如果设置为null，则跳过所有末端偏移
        if (robotConfig.effector_offset === null) {
            console.log(`  ⊘ 机器人${robotIndex}末端偏移已禁用（配置文件设置为null）`);
            effPose = null;
        } else if (robotConfig.effector_offset) {
            const offset = robotConfig.effector_offset;
            if (offset.length === 3) {
                effPose = {
                    qw: 1, qx: 0, qy: 0, qz: 0,
                    tx: offset[0], ty: offset[1], tz: offset[2]
                };
                console.log(`  机器人${robotIndex}: 使用配置文件中的末端偏移`);
            } else if (offset.length === 7) {
                effPose = {
                    qw: offset[0], qx: offset[1], qy: offset[2], qz: offset[3],
                    tx: offset[4], ty: offset[5], tz: offset[6]
                };
                console.log(`  机器人${robotIndex}: 使用配置文件中的末端偏移（包含旋转）`);
            }
        }
        // 2. 如果自动提取失败，尝试使用旧的computeEffectorPoseFromURDF方法（向后兼容）
        else if (!effPose) {
            const endLinkName = robotConfig.joint_chain?.end_link;
            if (endLinkName && urdfRobot) {
                console.log(`  机器人${robotIndex}: 尝试从URDF自动提取末端偏移（end_link: ${endLinkName}）...`);
                effPose = computeEffectorPoseFromURDF(urdfRobot, endLinkName);
            }
        }
        
        // 应用偏移
        if (effPose) {
            console.log(`  机器人${robotIndex}末端偏移 (Z-up):`);
            console.log(`    平移: (${effPose.tx.toFixed(4)}, ${effPose.ty.toFixed(4)}, ${effPose.tz.toFixed(4)})`);
            console.log(`    四元数: (${effPose.qw.toFixed(4)}, ${effPose.qx.toFixed(4)}, ${effPose.qy.toFixed(4)}, ${effPose.qz.toFixed(4)})`);
            dqRobot.setEffectorPose(
                effPose.qw, effPose.qx, effPose.qy, effPose.qz,
                effPose.tx, effPose.ty, effPose.tz
            );
            console.log(`  ✓ 机器人${robotIndex}末端偏移已应用`);
        } else {
            console.log(`  ⊘ 机器人${robotIndex}末端偏移为 0（未配置且未找到fixed关节）`);
        }
        
        if (robotIndex === 1) {
            robot1 = dqRobot;
        } else {
            robot2 = dqRobot;
        }
        
        console.log(`✓ 机器人${robotIndex} MDH 提取完成`);
        
    } catch (error) {
        console.error(`提取机器人${robotIndex} MDH失败:`, error);
        throw error;
    }
}

/**
 * 更新位姿显示
 */
function updatePoseDisplay(pose) {
    const { translation, rotation } = pose;
    
    // 更新位置
    document.getElementById('pos-x').textContent = translation[1].toFixed(3);
    document.getElementById('pos-y').textContent = translation[2].toFixed(3);
    document.getElementById('pos-z').textContent = translation[3].toFixed(3);
    
    // 更新四元数
    document.getElementById('quat-w').textContent = rotation[0].toFixed(3);
    document.getElementById('quat-x').textContent = rotation[1].toFixed(3);
    document.getElementById('quat-y').textContent = rotation[2].toFixed(3);
    document.getElementById('quat-z').textContent = rotation[3].toFixed(3);
    
    // 计算并更新欧拉角
    const rotMatrix = quaternionToRotationMatrix(rotation);
    const euler = rotationMatrixToEuler(rotMatrix);
    
    document.getElementById('euler-roll').textContent = (euler.roll * 180 / Math.PI).toFixed(1);
    document.getElementById('euler-pitch').textContent = (euler.pitch * 180 / Math.PI).toFixed(1);
    document.getElementById('euler-yaw').textContent = (euler.yaw * 180 / Math.PI).toFixed(1);
}

/**
 * 更新双臂位姿显示
 */
function updateDualArmPoseDisplay(pose1, pose2) {
    // 机器人1的位置和四元数
    const x1 = pose1.translation[1] || 0;
    const y1 = pose1.translation[2] || 0;
    const z1 = pose1.translation[3] || 0;
    const qw1 = pose1.rotation[0] || 1;
    const qx1 = pose1.rotation[1] || 0;
    const qy1 = pose1.rotation[2] || 0;
    const qz1 = pose1.rotation[3] || 0;
    
    // 机器人2的位置和四元数
    const x2 = pose2.translation[1] || 0;
    const y2 = pose2.translation[2] || 0;
    const z2 = pose2.translation[3] || 0;
    const qw2 = pose2.rotation[0] || 1;
    const qx2 = pose2.rotation[1] || 0;
    const qy2 = pose2.rotation[2] || 0;
    const qz2 = pose2.rotation[3] || 0;
    
    // 获取绝对位姿（使用absolute_pose函数）
    let absX, absY, absZ, absQW, absQX, absQY, absQZ, absDQ = null;
    
    if (dualArmSystem && jointAngles1.length > 0 && jointAngles2.length > 0) {
        const allJointAngles = [...jointAngles1, ...jointAngles2];
        try {
            const absPose = dualArmSystem.getAbsoluteWorldPose(allJointAngles);
            // translation格式：[w, x, y, z]，其中x,y,z是位置
            absX = absPose.translation[1] || 0;
            absY = absPose.translation[2] || 0;
            absZ = absPose.translation[3] || 0;
            // rotation格式：[qw, qx, qy, qz]
            absQW = absPose.rotation[0] || 1;
            absQX = absPose.rotation[1] || 0;
            absQY = absPose.rotation[2] || 0;
            absQZ = absPose.rotation[3] || 0;
            
            // 获取绝对位姿的DQ表示
            const absDQArray = dualArmSystem.getAbsolutePose(allJointAngles);
            absDQ = absDQArray;
        } catch (error) {
            console.warn('使用absolute_pose计算绝对位姿失败，回退到简单计算:', error);
            // 回退到简单计算
            absX = (x1 + x2) / 2;
            absY = (y1 + y2) / 2;
            absZ = (z1 + z2) / 2;
            absQW = (qw1 + qw2) / 2;
            absQX = (qx1 + qx2) / 2;
            absQY = (qy1 + qy2) / 2;
            absQZ = (qz1 + qz2) / 2;
        }
    } else {
        // 回退到简单计算（两个末端的平均）
        absX = (x1 + x2) / 2;
        absY = (y1 + y2) / 2;
        absZ = (z1 + z2) / 2;
        absQW = (qw1 + qw2) / 2;
        absQX = (qx1 + qx2) / 2;
        absQY = (qy1 + qy2) / 2;
        absQZ = (qz1 + qz2) / 2;
    }
    
    // 计算相对位姿（使用relative_pose函数）
    let relX, relY, relZ, relQW, relQX, relQY, relQZ, relDQ = null;
    
    if (dualArmSystem && jointAngles1.length > 0 && jointAngles2.length > 0) {
        const allJointAngles = [...jointAngles1, ...jointAngles2];
        try {
            // 获取相对位姿的DQ表示
            relDQ = dualArmSystem.getRelativePose(allJointAngles);
            
            // 解析相对位姿DQ
            if (relDQ && relDQ.length >= 8) {
                // DQ格式：[q0, q1, q2, q3, q4, q5, q6, q7]
                // 前4个是实部（旋转四元数），后4个是对偶部
                // 从DQ提取位置需要计算 translation = 2 * dual_part * conj(rotation)
                const dqModule = getDQModule();
                if (dqModule && dqModule.DQWrapper) {
                    const dqObj = dqModule.DQWrapper.createFromArray(relDQ);
                    const translation = dqModule.DQWrapper.getTranslation(dqObj);
                    const rotation = dqModule.DQWrapper.getRotation(dqObj);
                
                    relX = translation[1] || 0;
                    relY = translation[2] || 0;
                    relZ = translation[3] || 0;
                    relQW = rotation[0] || 1;
                    relQX = rotation[1] || 0;
                    relQY = rotation[2] || 0;
                    relQZ = rotation[3] || 0;
                } else {
                    throw new Error('DQModule未初始化');
                }
            } else {
                throw new Error('相对位姿DQ格式错误');
            }
        } catch (error) {
            console.warn('使用relative_pose计算相对位姿失败，回退到简单计算:', error);
            // 回退到简单计算（仅位置）
            relX = x2 - x1;
            relY = y2 - y1;
            relZ = z2 - z1;
            // 相对旋转：rel_q = q2 * conj(q1)
            // 简化计算
            relQW = qw2 * qw1 + qx2 * qx1 + qy2 * qy1 + qz2 * qz1;
            relQX = qw2 * qx1 - qx2 * qw1 + qy2 * qz1 - qz2 * qy1;
            relQY = qw2 * qy1 - qy2 * qw1 + qz2 * qx1 - qx2 * qz1;
            relQZ = qw2 * qz1 - qz2 * qw1 + qx2 * qy1 - qy2 * qx1;
            // 归一化
            const relLen = Math.sqrt(relQW * relQW + relQX * relQX + relQY * relQY + relQZ * relQZ);
            if (relLen > 0.001) {
                relQW /= relLen;
                relQX /= relLen;
                relQY /= relLen;
                relQZ /= relLen;
            }
        }
    } else {
        // 回退到简单计算
        relX = x2 - x1;
        relY = y2 - y1;
        relZ = z2 - z1;
        // 相对旋转：rel_q = q2 * conj(q1)
        relQW = qw2 * qw1 + qx2 * qx1 + qy2 * qy1 + qz2 * qz1;
        relQX = qw2 * qx1 - qx2 * qw1 + qy2 * qz1 - qz2 * qy1;
        relQY = qw2 * qy1 - qy2 * qw1 + qz2 * qx1 - qx2 * qz1;
        relQZ = qw2 * qz1 - qz2 * qw1 + qx2 * qy1 - qy2 * qx1;
        // 归一化
        const relLen = Math.sqrt(relQW * relQW + relQX * relQX + relQY * relQY + relQZ * relQZ);
        if (relLen > 0.001) {
            relQW /= relLen;
            relQX /= relLen;
            relQY /= relLen;
            relQZ /= relLen;
        }
    }
    
    const relDistance = Math.sqrt(relX * relX + relY * relY + relZ * relZ);
    
    // 获取机器人1和机器人2的DQ表示
    let dq1 = null, dq2 = null;
    try {
        if (robot1) {
            const dq1Array = robot1.fkm(jointAngles1);
            dq1 = dq1Array;
        }
        if (robot2) {
            const dq2Array = robot2.fkm(jointAngles2);
            dq2 = dq2Array;
        }
    } catch (error) {
        console.warn('获取机器人DQ表示失败:', error);
    }
    
    // 更新机器人1位姿
    const updateElement = (id, value) => {
        const elem = document.getElementById(id);
        if (elem) elem.textContent = typeof value === 'number' ? value.toFixed(3) : value;
    };
    
    // 机器人1
    updateElement('robot1-pos-x', x1);
    updateElement('robot1-pos-y', y1);
    updateElement('robot1-pos-z', z1);
    updateElement('robot1-quat-w', qw1);
    updateElement('robot1-quat-x', qx1);
    updateElement('robot1-quat-y', qy1);
    updateElement('robot1-quat-z', qz1);
    if (dq1 && dq1.length >= 8) {
        updateElement('robot1-dq-primary', `${dq1[0].toFixed(4)} ${dq1[1].toFixed(4)} ${dq1[2].toFixed(4)} ${dq1[3].toFixed(4)}`);
        updateElement('robot1-dq-dual', `${dq1[4].toFixed(4)} ${dq1[5].toFixed(4)} ${dq1[6].toFixed(4)} ${dq1[7].toFixed(4)}`);
    }
    
    // 机器人2
    updateElement('robot2-pos-x', x2);
    updateElement('robot2-pos-y', y2);
    updateElement('robot2-pos-z', z2);
    updateElement('robot2-quat-w', qw2);
    updateElement('robot2-quat-x', qx2);
    updateElement('robot2-quat-y', qy2);
    updateElement('robot2-quat-z', qz2);
    if (dq2 && dq2.length >= 8) {
        updateElement('robot2-dq-primary', `${dq2[0].toFixed(4)} ${dq2[1].toFixed(4)} ${dq2[2].toFixed(4)} ${dq2[3].toFixed(4)}`);
        updateElement('robot2-dq-dual', `${dq2[4].toFixed(4)} ${dq2[5].toFixed(4)} ${dq2[6].toFixed(4)} ${dq2[7].toFixed(4)}`);
    }
    
    // 绝对位姿
    updateElement('absolute-pos-x', absX);
    updateElement('absolute-pos-y', absY);
    updateElement('absolute-pos-z', absZ);
    updateElement('absolute-quat-w', absQW);
    updateElement('absolute-quat-x', absQX);
    updateElement('absolute-quat-y', absQY);
    updateElement('absolute-quat-z', absQZ);
    if (absDQ && absDQ.length >= 8) {
        updateElement('absolute-dq-primary', `${absDQ[0].toFixed(4)} ${absDQ[1].toFixed(4)} ${absDQ[2].toFixed(4)} ${absDQ[3].toFixed(4)}`);
        updateElement('absolute-dq-dual', `${absDQ[4].toFixed(4)} ${absDQ[5].toFixed(4)} ${absDQ[6].toFixed(4)} ${absDQ[7].toFixed(4)}`);
    }
    
    // 相对位姿
    updateElement('relative-pos-x', relX);
    updateElement('relative-pos-y', relY);
    updateElement('relative-pos-z', relZ);
    updateElement('relative-distance', relDistance);
    updateElement('relative-quat-w', relQW);
    updateElement('relative-quat-x', relQX);
    updateElement('relative-quat-y', relQY);
    updateElement('relative-quat-z', relQZ);
    if (relDQ && relDQ.length >= 8) {
        updateElement('relative-dq-primary', `${relDQ[0].toFixed(4)} ${relDQ[1].toFixed(4)} ${relDQ[2].toFixed(4)} ${relDQ[3].toFixed(4)}`);
        updateElement('relative-dq-dual', `${relDQ[4].toFixed(4)} ${relDQ[5].toFixed(4)} ${relDQ[6].toFixed(4)} ${relDQ[7].toFixed(4)}`);
    }
}

/**
 * 更新 DQ 显示
 */
function updateDQDisplay() {
    try {
        const dqArray = currentRobot.fkm(jointAngles);
        
        // 实部 (primary) - 前4个元素
        const primaryDiv = document.getElementById('dq-primary');
        primaryDiv.innerHTML = '';
        for (let i = 0; i < 4; i++) {
            const code = document.createElement('code');
            code.textContent = dqArray[i].toFixed(4);
            primaryDiv.appendChild(code);
        }
        
        // 对偶部 (dual) - 后4个元素
        const dualDiv = document.getElementById('dq-dual');
        dualDiv.innerHTML = '';
        for (let i = 4; i < 8; i++) {
            const code = document.createElement('code');
            code.textContent = dqArray[i].toFixed(4);
            dualDiv.appendChild(code);
        }
    } catch (error) {
        console.error('更新 DQ 显示失败:', error);
    }
}

/**
 * 更新雅可比矩阵显示
 */
function updateJacobianDisplay() {
    try {
        const jacobian = currentRobot.poseJacobian(jointAngles);
        
        const container = document.getElementById('jacobian-matrix');
        
        // 创建表格
        let html = '<table>';
        for (let i = 0; i < Math.min(8, jacobian.length); i++) {
            html += '<tr>';
            for (let j = 0; j < jacobian[i].length; j++) {
                const value = jacobian[i][j].toFixed(4);
                html += `<td>${value}</td>`;
            }
            html += '</tr>';
        }
        html += '</table>';
        
        container.innerHTML = html;
    } catch (error) {
        console.error('更新雅可比矩阵失败:', error);
    }
}

/**
 * 渲染环境物体列表
 */
function renderEnvironmentObjectList() {
    const list = document.getElementById('environment-object-list');
    if (!list) {
        return;
    }

    list.innerHTML = '';

    if (!environmentObjects.length) {
        const empty = document.createElement('div');
        empty.className = 'env-empty-message';
        empty.textContent = '当前场景没有环境物体';
        list.appendChild(empty);
        return;
    }

    environmentObjects.forEach((object) => {
        // 同步可视化器中的最新附着及位置
        if (visualizer?.getEnvironmentObjectAttachment) {
            const currentAttachment = visualizer.getEnvironmentObjectAttachment(object.id);
            object.attachedTo = currentAttachment || null;
        }
        if (visualizer?.getEnvironmentObjectWorldPosition) {
            const worldPos = visualizer.getEnvironmentObjectWorldPosition(object.id);
            if (worldPos) {
                object.position = { x: worldPos.x, y: worldPos.y, z: worldPos.z };
            }
        }
        if (visualizer?.getEnvironmentObjectOffset) {
            const offset = visualizer.getEnvironmentObjectOffset(object.id);
            if (offset) {
                object.offset = offset;
            }
        }

        const item = document.createElement('div');
        item.className = 'env-object-item';

        const info = document.createElement('div');
        info.className = 'env-object-info';

        const title = document.createElement('div');
        title.className = 'env-object-title';
        title.textContent = object.displayName || '环境物体';

        const dimsCm = object.dimensionsCm || {};
        const lengthCm = Number(dimsCm.length ?? 0).toFixed(1);
        const widthCm = Number(dimsCm.width ?? 0).toFixed(1);
        const heightCm = Number(dimsCm.height ?? 0).toFixed(1);

        const dimensions = document.createElement('div');
        dimensions.className = 'env-object-meta';
        dimensions.textContent = `尺寸：${lengthCm} × ${widthCm} × ${heightCm} cm`;

        const posInfo = document.createElement('div');
        posInfo.className = 'env-object-meta';
        const pos = object.position || { x: 0, y: 0, z: 0 };
        posInfo.textContent = `位置 (m)：(${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`;

        const attachStatus = document.createElement('div');
        attachStatus.className = 'env-object-meta';
        attachStatus.textContent = `附着：${getEnvironmentAttachmentLabel(object.attachedTo)}`;

        info.appendChild(title);
        info.appendChild(dimensions);
        info.appendChild(posInfo);
        info.appendChild(attachStatus);
        const offset = object.offset || {
            position: { x: 0, y: 0, z: 0 },
            rotation: { x: 0, y: 0, z: 0 }
        };
        const offsetPos = offset.position || { x: 0, y: 0, z: 0 };
        const offsetRot = offset.rotation || { x: 0, y: 0, z: 0 };
        const offsetInfo = document.createElement('div');
        offsetInfo.className = 'env-object-meta';
        offsetInfo.textContent = `偏移(cm)：(${(offsetPos.x / CM_TO_M).toFixed(1)}, ${(offsetPos.y / CM_TO_M).toFixed(1)}, ${(offsetPos.z / CM_TO_M).toFixed(1)}) ｜ 旋转(°)：(${offsetRot.x.toFixed(1)}, ${offsetRot.y.toFixed(1)}, ${offsetRot.z.toFixed(1)})`;
        info.appendChild(offsetInfo);

        const controls = document.createElement('div');
        controls.className = 'env-object-controls';

        const attachmentOptions = getEnvironmentAttachmentOptions();
        const attachSelect = document.createElement('select');
        attachSelect.className = 'env-attach-select';
        attachmentOptions.forEach((option) => {
            const opt = document.createElement('option');
            opt.value = option.value;
            opt.textContent = option.label;
            opt.disabled = option.disabled;
            attachSelect.appendChild(opt);
        });
        const currentValue = object.attachedTo || 'none';
        if (!attachmentOptions.some((opt) => opt.value === currentValue && !opt.disabled)) {
            attachSelect.value = 'none';
        } else {
            attachSelect.value = currentValue;
        }
        attachSelect.addEventListener('change', (e) => {
            updateEnvironmentAttachment(object.id, e.target.value);
        });
        controls.appendChild(attachSelect);

        const adjustBtn = document.createElement('button');
        adjustBtn.className = 'env-adjust-btn';
        adjustBtn.textContent = '调整';
        if (!object.attachedTo) {
            adjustBtn.disabled = true;
            adjustBtn.title = '附着后可调整偏移';
        }
        adjustBtn.addEventListener('click', () => {
            promptEnvironmentObjectAdjustment(object.id);
        });
        controls.appendChild(adjustBtn);

        const removeBtn = document.createElement('button');
        removeBtn.className = 'env-remove-btn';
        removeBtn.textContent = '移除';
        removeBtn.addEventListener('click', () => {
            removeEnvironmentObjectEntry(object.id);
        });
        controls.appendChild(removeBtn);

        item.appendChild(info);
        item.appendChild(controls);
        list.appendChild(item);
    });
}

function getEnvironmentAttachmentLabel(value) {
    switch (value) {
        case 'end-effector':
            return '末端小球';
        case 'dual-absolute':
            return '双臂绝对姿态小球';
        default:
            return '未附着';
    }
}

function getEnvironmentAttachmentOptions() {
    const options = [
        { value: 'none', label: '未附着', disabled: false }
    ];
    const hasVisualizer = !!visualizer;
    const hasEndEffector = hasVisualizer && typeof visualizer.hasEndEffectorAnchor === 'function'
        ? visualizer.hasEndEffectorAnchor()
        : false;
    options.push({
        value: 'end-effector',
        label: '末端小球',
        disabled: !hasEndEffector
    });
    const hasAbsolute = hasVisualizer && typeof visualizer.hasAbsolutePoseMarker === 'function'
        ? visualizer.hasAbsolutePoseMarker()
        : false;
    if (hasAbsolute) {
        options.push({
            value: 'dual-absolute',
            label: '双臂绝对姿态小球',
            disabled: false
        });
    }
    return options;
}

function promptEnvironmentObjectAdjustment(id) {
    const entry = environmentObjects.find((item) => item.id === id);
    if (!entry) {
        return;
    }
    if (!entry.attachedTo) {
        window.alert('请先将该物体附着到小球上，再进行调整。');
        return;
    }
    if (!visualizer || typeof visualizer.setEnvironmentObjectOffset !== 'function') {
        window.alert('当前可视化器不支持偏移调整');
        return;
    }

    const offset = visualizer.getEnvironmentObjectOffset?.(id) || {
        position: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0 }
    };
    const defaultInput = [
        (offset.position.x / CM_TO_M).toFixed(2),
        (offset.position.y / CM_TO_M).toFixed(2),
        (offset.position.z / CM_TO_M).toFixed(2),
        (offset.rotation.x || 0).toFixed(1),
        (offset.rotation.y || 0).toFixed(1),
        (offset.rotation.z || 0).toFixed(1)
    ].join(', ');

    const input = window.prompt(
        '输入偏移与旋转，格式：dx, dy, dz, roll, pitch, yaw\n' +
        'dx/dy/dz 单位为厘米，roll/pitch/yaw 单位为度。\n' +
        '正方向遵循右手定则，旋转顺序为 XYZ。',
        defaultInput
    );
    if (input === null) {
        return;
    }

    const parts = input.split(/[,\s]+/).filter(Boolean);
    if (parts.length !== 6) {
        window.alert('请输入 6 个数值，格式示例：2, 0, 5, 0, 0, 90');
        return;
    }

    const numbers = parts.map((value) => Number.parseFloat(value));
    if (numbers.some((num) => Number.isNaN(num))) {
        window.alert('输入包含非数值内容，请重新输入');
        return;
    }

    const [dxCm, dyCm, dzCm, rollDeg, pitchDeg, yawDeg] = numbers;

    try {
        const updatedOffset = visualizer.setEnvironmentObjectOffset(id, {
            position: {
                x: dxCm * CM_TO_M,
                y: dyCm * CM_TO_M,
                z: dzCm * CM_TO_M
            },
            rotation: {
                x: rollDeg,
                y: pitchDeg,
                z: yawDeg
            }
        });

        entry.offset = {
            position: { ...updatedOffset.position },
            rotation: { ...updatedOffset.rotation }
        };

        if (typeof visualizer.getEnvironmentObjectWorldPosition === 'function') {
            const worldPos = visualizer.getEnvironmentObjectWorldPosition(id);
            if (worldPos) {
                entry.position = { x: worldPos.x, y: worldPos.y, z: worldPos.z };
            }
        }

        renderEnvironmentObjectList();
    } catch (error) {
        console.error('调整环境物体偏移失败:', error);
        window.alert(`调整失败：${error.message}`);
    }
}

function updateEnvironmentAttachment(id, targetValue) {
    const entry = environmentObjects.find((item) => item.id === id);
    if (!entry) {
        return;
    }

    const normalized = targetValue === 'none' ? null : targetValue;

    try {
        if (!visualizer) {
            throw new Error('可视化器尚未初始化');
        }

        if (!normalized) {
            if (typeof visualizer.detachEnvironmentObject === 'function') {
                visualizer.detachEnvironmentObject(id);
            }
            entry.attachedTo = null;
        } else if (typeof visualizer.attachEnvironmentObject === 'function') {
            visualizer.attachEnvironmentObject(id, { target: normalized });
            entry.attachedTo = normalized;
        } else {
            throw new Error('可视化器未实现环境附着功能');
        }

        if (typeof visualizer.getEnvironmentObjectWorldPosition === 'function') {
            const worldPos = visualizer.getEnvironmentObjectWorldPosition(id);
            if (worldPos) {
                entry.position = { x: worldPos.x, y: worldPos.y, z: worldPos.z };
            }
        }
        if (typeof visualizer.getEnvironmentObjectOffset === 'function') {
            const offset = visualizer.getEnvironmentObjectOffset(id);
            if (offset) {
                entry.offset = offset;
            }
        }
    } catch (error) {
        console.error('更新环境物体附着失败:', error);
        window.alert(`更新附着失败：${error.message}`);
    }

    renderEnvironmentObjectList();
}

/**
 * 添加默认环境箱体到场景
 */
function addEnvironmentBoxToScene() {
    if (!visualizer || typeof visualizer.addEnvironmentBox !== 'function') {
        throw new Error('可视化器尚未初始化，无法添加环境物体');
    }

    environmentObjectCounter += 1;

    const dimsCm = DEFAULT_ENVIRONMENT_BOX_CM;
    const dims = {
        width: Number(dimsCm.length || 0) * CM_TO_M, // X 方向
        depth: Number(dimsCm.width || 0) * CM_TO_M,  // Y 方向
        height: Number(dimsCm.height || 0) * CM_TO_M // Z 方向
    };

    const spacing = Math.max(dims.width, dims.depth) + 0.15;
    const position = {
        x: ENVIRONMENT_BOX_BASE_OFFSET_X + (environmentObjectCounter - 1) * spacing,
        y: 0,
        z: dims.height / 2
    };

    const id = `env-box-${environmentObjectCounter}`;
    const mesh = visualizer.addEnvironmentBox({
        id,
        width: dims.width,
        depth: dims.depth,
        height: dims.height,
        color: DEFAULT_ENVIRONMENT_BOX_COLOR,
        opacity: 0.92,
        position
    });

    if (!mesh) {
        environmentObjectCounter -= 1;
        throw new Error('添加环境物体失败，返回值为空');
    }

    environmentObjects.push({
        id,
        type: 'box',
        displayName: `箱子 #${environmentObjectCounter}`,
        dimensionsCm: { ...dimsCm },
        position,
        attachedTo: null,
        offset: {
            position: { x: 0, y: 0, z: 0 },
            rotation: { x: 0, y: 0, z: 0 }
        }
    });

    renderEnvironmentObjectList();
    return mesh;
}

/**
 * 移除环境物体
 * @param {string} id
 * @returns {boolean}
 */
function removeEnvironmentObjectEntry(id) {
    const index = environmentObjects.findIndex((item) => item.id === id);
    if (index === -1) {
        return false;
    }

    if (visualizer && typeof visualizer.removeEnvironmentObject === 'function') {
        visualizer.removeEnvironmentObject(id);
    }

    environmentObjects.splice(index, 1);
    renderEnvironmentObjectList();
    return true;
}

/**
 * 设置事件监听器
 */
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
            if (visualizer) {
                visualizer.toggleMDHFrames(e.target.checked);
            }
        });
    }
    
    // URDF坐标系显示开关
    const urdfFramesToggle = document.getElementById('show-urdf-frames-toggle');
    if (urdfFramesToggle) {
        urdfFramesToggle.addEventListener('change', (e) => {
            if (visualizer) {
                visualizer.toggleURDFFrames(e.target.checked);
            }
        });
    }
    
    const angleUnitToggle = document.getElementById('angle-unit-toggle');
    if (angleUnitToggle) {
        const updateAngleUnitToggleLabel = () => {
            angleUnitToggle.textContent = angleDisplayMode === 'deg' ? '切换为弧度' : '切换为角度';
        };
        updateAngleUnitToggleLabel();
        angleUnitToggle.addEventListener('click', () => {
            angleDisplayMode = angleDisplayMode === 'deg' ? 'rad' : 'deg';
            refreshJointControlDisplay();
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

                importJointAnglesFromText(inputText);
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
                addEnvironmentBoxToScene();
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

    const copyAnglesBtn = document.getElementById('copy-joint-angles-btn');
    if (copyAnglesBtn) {
        const defaultCopyText = copyAnglesBtn.textContent.trim();
        copyAnglesBtn.addEventListener('click', async () => {
            copyAnglesBtn.disabled = true;
            try {
                await copyJointAnglesToClipboard();
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
                await copyJointLimitsToClipboard();
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
        if (visualizer?.setTeachMultiplicationOrder) {
            const order = isDeltaFirst ? 'delta-first' : 'current-first';
            visualizer.setTeachMultiplicationOrder(order);
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
        if (visualizer?.setTeachSpeedPercent) {
            visualizer.setTeachSpeedPercent(p);
        } else if (visualizer) {
            visualizer.teachSpeedPercent = p;
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
            const axis = button.dataset.axis;
            const direction = button.dataset.direction;
            const type = button.dataset.type || 'translation'; // 'translation' 或 'rotation'
            
            if (!visualizer || !axis || !direction) return;
            
            // 检查是否为双臂模式
            const isDualArm = visualizer.dualArmMode;
            
            if (isDualArm) {
                // 获取控制模式选择器
                const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
                const controlMode = controlModeSelect ? controlModeSelect.value : 'absolute';
                
                if (controlMode === 'robot1') {
                    // 控制机器人1
                    if (type === 'rotation') {
                        visualizer.rotateRobot1EndEffectorAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.rotateRobot1EndEffectorAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        visualizer.moveRobot1EndEffectorAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.moveRobot1EndEffectorAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'robot2') {
                    // 控制机器人2
                    if (type === 'rotation') {
                        visualizer.rotateRobot2EndEffectorAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.rotateRobot2EndEffectorAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        visualizer.moveRobot2EndEffectorAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.moveRobot2EndEffectorAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'relative') {
                    // 控制相对位姿（保持绝对位姿不变）
                    if (type === 'rotation') {
                        visualizer.rotateDualArmRelativePoseAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.rotateDualArmRelativePoseAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        visualizer.moveDualArmRelativePoseAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.moveDualArmRelativePoseAlongAxis(axis, direction);
                        }, 20);
                    }
                } else {
                    // 默认：控制绝对位姿（保持相对位姿不变）
                    if (type === 'rotation') {
                        visualizer.rotateDualArmAbsolutePoseAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.rotateDualArmAbsolutePoseAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        visualizer.moveDualArmAbsolutePoseAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            visualizer.moveDualArmAbsolutePoseAlongAxis(axis, direction);
                        }, 20);
                    }
                }
            } else {
                // 单臂模式：控制末端执行器
                if (type === 'rotation') {
                    // 旋转控制
                    visualizer.rotateEndEffectorAroundAxis(axis, direction);
                    intervalId = setInterval(() => {
                        visualizer.rotateEndEffectorAroundAxis(axis, direction);
                    }, 20);
                } else {
                    // 平移控制
                    visualizer.moveEndEffectorAlongAxis(axis, direction);
                    intervalId = setInterval(() => {
                        visualizer.moveEndEffectorAlongAxis(axis, direction);
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
    renderEnvironmentObjectList();
    setupGlobalDragAndDrop();
    
    // 浮动面板与底部功能栏
    setupFloatingPanels();
}

/**
 * 设置拖拽区域
 */
function setupGlobalDragAndDrop() {
    dropOverlayElement = document.getElementById('drop-overlay');
    if (!dropOverlayElement) {
        return;
    }

    let dragCounter = 0;

    const isFileDrag = (event) => {
        const types = event.dataTransfer?.types;
        if (!types) {
            return false;
        }
        if (typeof types.includes === 'function') {
            return types.includes('Files');
        }
        return Array.from(types).indexOf('Files') >= 0;
    };

    window.addEventListener('dragenter', (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        event.preventDefault();
        dragCounter += 1;
        updateDropOverlay('ready', '释放以加载 URDF 文件夹', '📁');
    });

    window.addEventListener('dragover', (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        event.preventDefault();
    });

    window.addEventListener('dragleave', (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        dragCounter = Math.max(0, dragCounter - 1);
        if (dragCounter === 0) {
            hideDropOverlay();
        }
    });

    window.addEventListener('drop', async (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        event.preventDefault();
        dragCounter = 0;

        const transfer = event.dataTransfer;
        const items = transfer?.items?.length ? transfer.items : transfer?.files;
        if (!items || !items.length) {
            hideDropOverlay();
            return;
        }

        updateDropOverlay('loading', '正在加载 URDF 资源...', '⏳');
        await handleDrop(items);
    });
}

function updateDropOverlay(state, message, icon) {
    dropOverlayElement = dropOverlayElement || document.getElementById('drop-overlay');
    if (!dropOverlayElement) {
        return;
    }

    if (dropOverlayHideTimer) {
        clearTimeout(dropOverlayHideTimer);
        dropOverlayHideTimer = null;
    }

    const stateClasses = ['drop-ready', 'drop-loading', 'drop-success', 'drop-error'];
    const iconFallback = {
        ready: '📁',
        loading: '⏳',
        success: '✅',
        error: '⚠️'
    };
    const textFallback = {
        ready: '释放以加载 URDF 文件夹',
        loading: '正在加载 URDF 资源...',
        success: '加载完成',
        error: '加载失败'
    };

    dropOverlayElement.classList.remove('hidden');
    dropOverlayElement.classList.add('visible');
    stateClasses.forEach((cls) => dropOverlayElement.classList.remove(cls));
    dropOverlayElement.classList.add(`drop-${state}`);

    const iconElement = dropOverlayElement.querySelector('.drop-overlay-icon');
    const textElement = dropOverlayElement.querySelector('.drop-overlay-text');

    if (iconElement) {
        iconElement.textContent = icon || iconFallback[state] || '📁';
    }
    if (textElement) {
        textElement.textContent = message || textFallback[state] || '';
    }
}

function hideDropOverlay(delay = 0) {
    dropOverlayElement = dropOverlayElement || document.getElementById('drop-overlay');
    if (!dropOverlayElement) {
        return;
    }

    if (dropOverlayHideTimer) {
        clearTimeout(dropOverlayHideTimer);
        dropOverlayHideTimer = null;
    }

    dropOverlayHideTimer = setTimeout(() => {
        dropOverlayElement.classList.remove('visible', 'drop-ready', 'drop-loading', 'drop-success', 'drop-error');
        dropOverlayElement.classList.add('hidden');
        dropOverlayHideTimer = null;
    }, delay);
}

/**
 * 处理拖拽的文件
 */
async function handleDrop(items) {
    console.log('═══════════════════════════════════════════');
    console.log('       开始拖拽加载');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        // 在加载新模型前，先清理所有旧模型
        if (visualizer && typeof visualizer.cleanupOldRobots === 'function') {
            console.log('0️⃣ 清理旧模型...');
            visualizer.cleanupOldRobots();
            console.log('✓ 旧模型已清理');
        }
        
        console.log('1️⃣ 创建包管理器...');
        packageManager = new URDFPackageManager();
        await packageManager.addFiles(items);
        console.log(`✓ 已添加 ${packageManager.files.size} 个文件`);
        
        console.log('\n2️⃣ 加载 URDF...');
        const robot = await loadURDFFromPackage(packageManager);
        console.log('✓ URDF 对象创建成功');
        
        console.log('\n3️⃣ 读取配置...');
        customRobotConfig = packageManager.getConfig();
        if (!customRobotConfig) {
            throw new Error('未找到 robot_config.yaml 配置文件');
        }
        console.log('✓ 配置:', customRobotConfig.robot_name);
        console.log('  基座:', customRobotConfig.joint_chain?.base_link);
        console.log('  末端:', customRobotConfig.joint_chain?.end_link);
        console.log('  关节数:', customRobotConfig.joint_chain?.joints?.length || 0);
        
        const isDualArm = customRobotConfig?.dual_arm === true;
        dualArmMode = isDualArm;
        
        if (isDualArm) {
            console.log('\n4️⃣ 双臂模式：加载两个机器人...');
            const jointControlsContainer = document.getElementById('joint-controls');
            if (jointControlsContainer) {
                jointControlsContainer.innerHTML = '';
            }
            
            await loadDualArmSystem(robot, customRobotConfig);
            switchToDualArmUI();
            createDualArmJointControls(customRobotConfig);
        } else {
            console.log('\n4️⃣ 加载到 3D 场景...');
            if (!visualizer) {
                throw new Error('可视化器未初始化');
            }
            const loaded = await visualizer.loadCustomURDF(robot, null, customRobotConfig);
            if (!loaded) {
                throw new Error('可视化器加载失败');
            }
            console.log('✓ 3D 模型已添加到场景');
            switchToSingleArmUI();
            
            console.log('\n5️⃣ 提取 MDH 参数...');
            if (customRobotConfig.mdh_extraction?.auto_extract) {
                await extractMDHFromCustomRobot(robot, customRobotConfig);
                console.log('✓ MDH 提取并应用成功');
            } else {
                console.log('⊘ MDH 自动提取已禁用');
            }
        }
        
        console.log('\n6️⃣ 同步界面状态...');
    updateDropOverlay('success', `✓ 已加载: ${customRobotConfig.robot_name}`, '✅');
    hideDropOverlay(1400);
    refreshDockState();
        
        console.log('\n✅ 全部完成！');
        console.log('═══════════════════════════════════════════\n');
    } catch (error) {
        console.error('\n❌ 加载失败:', error);
        console.error('错误堆栈:', error.stack);
        console.log('═══════════════════════════════════════════\n');
        updateDropOverlay('error', '❌ 加载失败，请查看控制台了解详情', '⚠️');
        hideDropOverlay(2200);
        alert(`加载失败:\n\n${error.message}\n\n请打开控制台(F12)查看详细错误信息。`);
    }
}

/**
 * 从自定义机器人提取 MDH
 */
async function extractMDHFromCustomRobot(robot, config) {
    try {
        console.log('═══════════════════════════════════════════');
        console.log(`  提取 ${config.robot_name} 的 MDH 参数`);
        console.log('═══════════════════════════════════════════\n');
        
        const jointNames = config.joint_chain.joints;
        const baseLinkName = config.joint_chain?.base_link || null;
        
        // 检查是否使用手动指定的 MDH
        if (config.mdh_extraction && 
            config.mdh_extraction.auto_extract === false && 
            config.mdh_extraction.manual_mdh) {
            
            console.log('  ✓ 使用配置文件中的手动 MDH 参数');
            extractedMDH = config.mdh_extraction.manual_mdh;
            
        } else {
            // 从 URDF 自动提取
            console.log('  → 从 URDF 自动提取 MDH（保持 URDF 原始坐标系）');
            const endLinkName = config.joint_chain?.end_link;
            
            // 读取基座配置（如果配置了）
            let baseConfig = null;
            if (config.mdh_extraction?.base_frame) {
                const baseFrame = config.mdh_extraction.base_frame;
                baseConfig = {
                    origin: baseFrame.origin || [0, 0, 0],
                    z_axis: baseFrame.z_axis || [0, 0, 1],
                    x_axis: baseFrame.x_axis || [1, 0, 0]
                };
                console.log('  ✓ 使用配置的基座坐标系');
            }
            
            // 先不做坐标转换，直接提取原始URDF的MDH参数
            const result = extractMDHFromURDF(robot, jointNames, false, endLinkName, baseConfig, baseLinkName);
            
            // 保存提取的effectorPose供后续使用
            let extractedEffectorPose = null;
            
            // 处理新的返回格式
            if (result && typeof result === 'object' && result.mdh) {
                extractedMDH = result.mdh;
                extractedEffectorPose = result.effectorPose;
                if (result.autoBasePose) {
                    config._autoBasePose = result.autoBasePose;
                    console.log('  ✓ 已记录自动估计的基座位姿用于后续修正');
                }
                // 从MDH提取中获得的effectorPose会被后续代码使用
                if (extractedEffectorPose) {
                    console.log('  ✓ 已从MDH提取中获取末端执行器偏移');
                }
                
                // 设置MDH坐标系可视化
                if (result.allMDHFrames && visualizer) {
                    visualizer.setMDHFrames(result.allMDHFrames);
                    console.log('  ✓ 已设置MDH坐标系可视化');
                    
                    // 同步开关状态：如果开关是打开的，确保坐标系显示
                    const mdhFramesToggle = document.getElementById('show-mdh-frames-toggle');
                    if (mdhFramesToggle && mdhFramesToggle.checked) {
                        visualizer.showMDHFrames = true;
                        if (visualizer.mdhFramesGroup && !visualizer.scene.getObjectByName('MDHFrames')) {
                            visualizer.scene.add(visualizer.mdhFramesGroup);
                        }
                    }
                }
            } else {
                extractedMDH = result; // 向后兼容
            }
            
            // 将effectorPose保存到config中，供后续使用
            if (extractedEffectorPose) {
                config._extractedEffectorPose = extractedEffectorPose;
            }
        }
        
        console.log('  MDH 参数 (提取结果):');
        console.log('  关节 |    θ     |    d     |    a     |    α    ');
        console.log('  -----|----------|----------|----------|----------');
        extractedMDH.forEach((params, i) => {
            console.log(`    ${(i+1).toString().padStart(2)}  | ` +
                       `${params[0].toFixed(4).padStart(8)} | ` +
                       `${params[1].toFixed(4).padStart(8)} | ` +
                       `${params[2].toFixed(4).padStart(8)} | ` +
                       `${params[3].toFixed(4).padStart(8)}`);
        });
        
        // 如果是Franka Panda，显示参考值对比
        if (jointNames.includes('panda_joint1')) {
            console.log('\n  参考MDH参数 (Franka官方):');
            console.log('  关节 |    θ     |    d     |    a     |    α    ');
            console.log('  -----|----------|----------|----------|----------');
            const ref = [
                [0, 0.333, 0, 0],
                [0, 0, 0, -Math.PI/2],
                [0, 0.316, 0, Math.PI/2],
                [0, 0, 0.0825, Math.PI/2],
                [0, 0.384, -0.0825, -Math.PI/2],
                [0, 0, 0, Math.PI/2],
                [0, 0, 0.088, Math.PI/2]
            ];
            ref.forEach((params, i) => {
                console.log(`    ${(i+1).toString().padStart(2)}  | ` +
                           `${params[0].toFixed(4).padStart(8)} | ` +
                           `${params[1].toFixed(4).padStart(8)} | ` +
                           `${params[2].toFixed(4).padStart(8)} | ` +
                           `${params[3].toFixed(4).padStart(8)}`);
            });
            
            // 显示差异
            console.log('\n  差异分析:');
            extractedMDH.forEach((params, i) => {
                if (i < ref.length) {
                    const diffD = Math.abs(params[1] - ref[i][1]);
                    const diffA = Math.abs(params[2] - ref[i][2]);
                    const diffAlpha = Math.abs(params[3] - ref[i][3]);
                    if (diffD > 0.001 || diffA > 0.001 || diffAlpha > 0.01) {
                        console.log(`    关节${i+1}: d差${diffD.toFixed(4)}, a差${diffA.toFixed(4)}, α差${diffAlpha.toFixed(4)}`);
                    }
                }
            });
        }
        
        // C++ 期望格式: 5行 x n列
        // 第1行: [θ1, θ2, θ3, ...]
        // 第2行: [d1, d2, d3, ...]
        // 第3行: [a1, a2, a3, ...]
        // 第4行: [α1, α2, α3, ...]
        // 第5行: [0, 0, 0, ...] (关节类型: 0=旋转)
        
        const numJoints = extractedMDH.length;
        console.log(`  关节数: ${numJoints}`);
        
        // 转置：从 [[θ,d,a,α], ...] 到 [[θ...], [d...], [a...], [α...], [type...]]
        const theta_row = extractedMDH.map(p => p[0]);
        const d_row = extractedMDH.map(p => p[1]);
        const a_row = extractedMDH.map(p => p[2]);
        const alpha_row = extractedMDH.map(p => p[3]);
        const type_row = new Array(numJoints).fill(0); // 全部为旋转关节 (0=revolute)
        
        const mdhMatrix = [theta_row, d_row, a_row, alpha_row, type_row];
        
        console.log('  MDH 矩阵格式 (5行 x n列):');
        console.log(`    θ 行 (${theta_row.length}元素):`, theta_row);
        console.log(`    d 行 (${d_row.length}元素):`, d_row);
        console.log(`    a 行 (${a_row.length}元素):`, a_row);
        console.log(`    α 行 (${alpha_row.length}元素):`, alpha_row);
        console.log(`    type (${type_row.length}元素):`, type_row);
        
        // 创建基于 URDF 的机器人（MDH 已经是 Z-up 了）
        console.log('  创建 SerialManipulator (5 x', numJoints, '矩阵)...');
        urdfBasedRobot = new SerialManipulator(mdhMatrix);
        console.log('  ✓ SerialManipulator 创建成功（Z-up 坐标系）');

        // 设置末端执行器偏移
        console.log('\n  → 检查末端执行器偏移配置...');
        try {
            const endLinkName = config.joint_chain?.end_link;
            let effPose = null;
            
            // 0. 优先使用从extractMDHFromURDF中提取的effectorPose（如果在提取MDH时已获得）
            // 注意：extractedMDH是数组，我们需要从提取结果中获取effectorPose
            // 这里我们需要重新提取MDH来获取effectorPose，或者从之前的result中获取
            // 由于代码流程限制，我们需要再次提取或者改进流程
            
            // 1. 优先使用配置文件中的手动偏移（覆盖自动提取的）
            // 如果设置为null，则跳过所有末端偏移
            if (config.mdh_extraction?.effector_offset === null) {
                console.log('  ⊘ 末端偏移已禁用（配置文件设置为null）');
            } else if (config.mdh_extraction?.effector_offset) {
                const offset = config.mdh_extraction.effector_offset;
                
                // 判断是简单平移 [x,y,z] 还是完整姿态 [qw,qx,qy,qz,tx,ty,tz]
                if (offset.length === 3) {
                    // 简单平移
                    console.log(`  使用配置的偏移 (平移): (${offset[0]}, ${offset[1]}, ${offset[2]})`);
                    effPose = {
                        qw: 1, qx: 0, qy: 0, qz: 0,
                        tx: offset[0], ty: offset[1], tz: offset[2]
                    };
                } else if (offset.length === 7) {
                    // 完整姿态
                    console.log(`  使用配置的偏移 (完整姿态):`);
                    console.log(`    四元数: (${offset[0]}, ${offset[1]}, ${offset[2]}, ${offset[3]})`);
                    console.log(`    平移: (${offset[4]}, ${offset[5]}, ${offset[6]})`);
                    effPose = {
                        qw: offset[0], qx: offset[1], qy: offset[2], qz: offset[3],
                        tx: offset[4], ty: offset[5], tz: offset[6]
                    };
                } else {
                    console.warn(`  ⚠ 末端偏移格式错误，应为[x,y,z]或[qw,qx,qy,qz,tx,ty,tz]`);
                }
            }
            // 2. 如果未配置手动偏移（且不是null），尝试使用之前提取的effectorPose或重新提取
            else if (config.mdh_extraction?.effector_offset !== null) {
                // 优先使用之前从extractMDHFromURDF中提取的effectorPose
                if (config._extractedEffectorPose) {
                    effPose = config._extractedEffectorPose;
                    console.log('  ✓ 使用之前从MDH提取中获取的末端执行器偏移');
                } else {
                    console.log(`  尝试从URDF自动提取（end_link: ${endLinkName}）...`);
                    // 使用新的提取方法（会返回详细的调试信息）
                    // 读取基座配置（如果配置了）
                    let baseConfig = null;
                    if (config.mdh_extraction?.base_frame) {
                        const baseFrame = config.mdh_extraction.base_frame;
                        baseConfig = {
                            origin: baseFrame.origin || [0, 0, 0],
                            z_axis: baseFrame.z_axis || [0, 0, 1],
                            x_axis: baseFrame.x_axis || [1, 0, 0]
                        };
                        console.log('  ✓ 使用配置的基座坐标系');
                    }
                    
                    const extractResult = extractMDHFromURDF(robot, jointNames, false, endLinkName, baseConfig, baseLinkName);
                    if (extractResult && typeof extractResult === 'object' && extractResult.effectorPose) {
                        effPose = extractResult.effectorPose;
                        console.log('  ✓ 从MDH提取中获取了末端执行器偏移');
                    } else {
                        // 回退到旧方法（向后兼容）
                        effPose = computeEffectorPoseFromURDF(robot, endLinkName);
                    }
                }
            }
            
            // 应用偏移
            if (effPose) {
                console.log('  末端偏移 (Z-up):');
                console.log(`    平移: (${effPose.tx.toFixed(4)}, ${effPose.ty.toFixed(4)}, ${effPose.tz.toFixed(4)})`);
                console.log(`    四元数: (${effPose.qw.toFixed(4)}, ${effPose.qx.toFixed(4)}, ${effPose.qy.toFixed(4)}, ${effPose.qz.toFixed(4)})`);
                urdfBasedRobot.setEffectorPose(
                    effPose.qw, effPose.qx, effPose.qy, effPose.qz,
                    effPose.tx, effPose.ty, effPose.tz
                );
                console.log('  ✓ 末端偏移已应用');
            } else {
                console.log('  ⊘ 末端偏移为 0（未配置且未找到fixed关节）');
            }
        } catch (e) {
            console.warn('  ⚠ 末端偏移设置失败:', e.message);
        }

        // 应用基座偏移（如果配置中有）
        // 优先从 mdh_extraction.base_pose 读取，如果不存在则从 config.base_pose 读取
        console.log('\n  → 检查基座偏移配置...');
        console.log('    config.mdh_extraction:', config.mdh_extraction);
        console.log('    config.mdh_extraction?.base_pose:', config.mdh_extraction?.base_pose);
        console.log('    config.base_pose:', config.base_pose);
        let basePose = config.mdh_extraction?.base_pose || config.base_pose;
        console.log('    读取到的 basePose:', basePose);
        console.log('    basePose 类型:', typeof basePose);
        console.log('    basePose 是否为数组:', Array.isArray(basePose));
        
        // 如果 basePose 是字符串，尝试解析为数组
        if (typeof basePose === 'string') {
            try {
                basePose = JSON.parse(basePose);
                console.log('    解析后的 basePose:', basePose);
            } catch (e) {
                console.warn('  无法解析 base_pose 字符串:', basePose);
            }
        }
        if ((!basePose || !Array.isArray(basePose) || basePose.length < 3) && Array.isArray(config._autoBasePose)) {
            basePose = config._autoBasePose;
            console.log('    未配置基座位姿，使用自动估计的基座修正');
        }
        
        if (basePose && Array.isArray(basePose) && basePose.length >= 3) {
            if (basePose.length === 8) {
                // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                console.log(`\n  → 设置基座 (双四元数格式):`);
                console.log(`    DQ: [${basePose.map(v => v.toFixed(6)).join(', ')}]`);
                // 检查 setBaseFrameDQ 是否存在
                if (typeof urdfBasedRobot.setBaseFrameDQ === 'function') {
                    urdfBasedRobot.setBaseFrameDQ(basePose);
                    console.log('  ✓ 基座双四元数已应用');
                } else {
                    console.warn('  ⚠ setBaseFrameDQ 方法不可用，回退到其他格式');
                    // 如果方法不存在，尝试从双四元数中提取位置和旋转
                    // 这里可以添加回退逻辑，但通常应该编译 WASM 时包含该方法
                }
            } else if (basePose.length === 7) {
                // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                const [qw, qx, qy, qz, tx, ty, tz] = basePose;
                console.log(`\n  → 设置基座位姿 (Z-up):`);
                console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                // 检查 setBaseFramePose 是否存在，如果不存在则只使用平移
                if (typeof urdfBasedRobot.setBaseFramePose === 'function') {
                    urdfBasedRobot.setBaseFramePose(qw, qx, qy, qz, tx, ty, tz);
                    console.log('  ✓ 基座完整位姿已应用');
                } else {
                    console.warn('  ⚠ setBaseFramePose 方法不可用，仅使用平移部分');
                    urdfBasedRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                    console.log('  ✓ 基座平移已应用（旋转部分未应用）');
                }
            } else if (basePose.length === 4) {
                // 可能是 [qw, tx, ty, tz] 格式，取后3个
                const [, tx, ty, tz] = basePose;
                console.log(`\n  → 设置基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                urdfBasedRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                console.log('  ✓ 基座偏移已应用');
            } else {
                // 仅平移 [tx, ty, tz]
                const [tx, ty, tz] = basePose;
                console.log(`\n  → 设置基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                urdfBasedRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                console.log('  ✓ 基座偏移已应用');
            }
        } else {
            // 向后兼容：检查 base_offset（旧格式）
            if (config.mdh_extraction?.base_offset) {
                const offset = config.mdh_extraction.base_offset;
                console.log(`\n  → 应用基座偏移 (旧格式base_offset): (${offset[0]}, ${offset[1]}, ${offset[2]})`);
                urdfBasedRobot.setBaseFrame(offset[0], offset[1], offset[2]);
                console.log('  ✓ 基座偏移已应用');
            } else {
                console.log('\n  ⊘ 基座偏移未配置');
            }
        }
        
        // 如果是Franka Panda，对比内置类和URDF提取的末端位置差异
        if (jointNames.includes('panda_joint1') && numJoints === 7) {
            console.log('\n  🔍 Franka Panda 末端位置对比:');
            try {
                // 计算URDF提取的MDH在零位的末端位置
                const zeroAngles = new Array(numJoints).fill(0);
                const urdfPose = urdfBasedRobot.getEndEffectorPose(zeroAngles);
                const urdfPos = {
                    x: urdfPose.translation[1],
                    y: urdfPose.translation[2],
                    z: urdfPose.translation[3]
                };
                console.log(`  URDF提取的MDH: X=${urdfPos.x.toFixed(4)}, Y=${urdfPos.y.toFixed(4)}, Z=${urdfPos.z.toFixed(4)}`);
                
                // 计算内置FrankaPanda类在零位的末端位置
                const builtinFranka = new FrankaPanda();
                const builtinPose = builtinFranka.getEndEffectorPose(zeroAngles);
                const builtinPos = {
                    x: builtinPose.translation[1],
                    y: builtinPose.translation[2],
                    z: builtinPose.translation[3]
                };
                console.log(`  内置FrankaPanda: X=${builtinPos.x.toFixed(4)}, Y=${builtinPos.y.toFixed(4)}, Z=${builtinPos.z.toFixed(4)}`);
                
                // 计算差异
                const diff = {
                    x: builtinPos.x - urdfPos.x,
                    y: builtinPos.y - urdfPos.y,
                    z: builtinPos.z - urdfPos.z
                };
                console.log(`  末端偏移差异:    ΔX=${diff.x.toFixed(4)}, ΔY=${diff.y.toFixed(4)}, ΔZ=${diff.z.toFixed(4)}`);
                console.log(`  说明: 内置类比URDF提取的末端在X方向多了 ${(diff.x * 1000).toFixed(1)}mm`);
            } catch (e) {
                console.warn('  对比失败:', e.message);
            }
        }
        
        currentRobot = urdfBasedRobot;
        currentRobotIsYUp = false; // 暂时设为false，待调试
        
        // 更新关节控制器
        jointAngles = config.initial_joint_angles || new Array(jointNames.length).fill(0);
        
        // 限制初始角度在限位范围内
        if (jointNames && jointNames.length === jointAngles.length) {
            const urdfRobot = visualizer?.urdfRobot || null;
            jointAngles = clampJointAngles(jointAngles, jointNames, urdfRobot, config);
        }
        
        // 创建关节控制器（传入限制后的角度）
        createJointControls(jointNames.length, config);
        
        // 更新滑块显示（使用限制后的角度）
        updateJointSlidersFromAngles();
        
        console.log('\n✓ MDH 提取完成，已创建 DQ 机器人');
        
        // 更新显示（使用简单的关节角度，避免立即调用可能失败的方法）
        setTimeout(() => {
            try {
                updateRobotPose();
            } catch (error) {
                console.warn('初始位姿更新失败（可忽略）:', error.message);
            }
        }, 100);
        
    } catch (error) {
        console.error('MDH 提取失败:', error);
        console.error('错误堆栈:', error.stack);
        throw error;
    }
}

/**
 * 应用预设位姿
 */
async function applyPreset(preset) {
    const numJoints = jointAngles.length;
    
    // 获取当前配置和关节名称（用于限位检查）
    let config = null;
    let jointNames = [];
    let urdfRobot = null;
    
    // 尝试从当前加载的配置获取信息
    if (customRobotConfig) {
        config = customRobotConfig;
        jointNames = config.joint_chain?.joints || [];
        urdfRobot = visualizer?.urdfRobot || null;
    } else if (currentRobotType === 'franka') {
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
            jointAngles = new Array(numJoints).fill(0);
            break;
            
        case 'extended':
            if (currentRobotType === 'franka') {
                jointAngles = [0, -30, 0, -120, 0, 90, 45].map(deg => deg * Math.PI / 180);
            } else {
                jointAngles = [0, -45, 90, 0, 45, 0, 0].map(deg => deg * Math.PI / 180);
            }
            break;
            
        case 'folded':
            if (currentRobotType === 'franka') {
                jointAngles = [0, 45, 0, -90, 0, 135, 0].map(deg => deg * Math.PI / 180);
            } else {
                jointAngles = [0, 45, -90, 0, -45, 0, 0].map(deg => deg * Math.PI / 180);
            }
            break;
            
        case 'random':
            jointAngles = jointAngles.map(() => (Math.random() - 0.5) * Math.PI);
            break;
    }
    
    // 限制角度在限位范围内
    if (jointNames.length === jointAngles.length) {
        jointAngles = clampJointAngles(jointAngles, jointNames, urdfRobot, config);
    }
    
    if (dualArmMode) {
        updateDualArmJointSlidersFromAngles();
    } else {
        updateJointSlidersFromAngles();
    }
    
    updateRobotPose();
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
            if (dualArmMode) {
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

/**
 * 从当前 URDF 提取 MDH 参数
 */
function extractMDHFromCurrentURDF() {
    if (!visualizer || !visualizer.urdfRobot) {
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
        
        const result = extractMDHFromURDF(visualizer.urdfRobot, jointNames);
        // 处理新的返回格式
        if (result && typeof result === 'object' && result.mdh) {
            extractedMDH = result.mdh;
            
            // 设置MDH坐标系可视化
            if (result.allMDHFrames && visualizer) {
                visualizer.setMDHFrames(result.allMDHFrames);
                console.log('  ✓ 已设置MDH坐标系可视化');
            }
        } else {
            extractedMDH = result; // 向后兼容
        }
        
        // 显示状态
        const statusDiv = document.getElementById('mdh-status');
        statusDiv.innerHTML = `
            <div style="color: #10b981; font-size: 0.875rem; margin-top: 0.5rem;">
                ✓ 已提取 ${extractedMDH.length} 个关节的 MDH 参数<br>
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
    if (!visualizer || !visualizer.urdfRobot) {
        alert('⚠ URDF 模型未加载');
        return;
    }
    
    console.log('═══════════════════════════════════════════');
    console.log('       MDH 参数对比测试');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        const result = createMDHTestSuite(visualizer.urdfRobot);
        
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
    if (!extractedMDH) {
        alert('⚠ 请先提取 MDH 参数\n\n点击 "从URDF提取MDH" 按钮。');
        return;
    }
    
    try {
        console.log('使用 URDF 提取的 MDH 参数创建机器人...');
        
        // 格式化为 DQ Robotics 格式
        const formatted = formatMDHForDQ(extractedMDH);
        
        // 转置为列格式（DQ 需要的格式）
        const mdhMatrix = [];
        for (let row = 0; row < 5; row++) {
            mdhMatrix.push(formatted[row]);
        }
        
        // 创建新的串联机械臂
        urdfBasedRobot = new SerialManipulator(mdhMatrix);
        
        // 切换到使用新机器人
        currentRobot = urdfBasedRobot;
        updateGlobalVariables();
        
        const statusDiv = document.getElementById('mdh-status');
        statusDiv.innerHTML = `
            <div style="color: #3b82f6; font-size: 0.875rem; margin-top: 0.5rem;">
                ✓ 正在使用 URDF-MDH<br>
                运动学现在基于URDF参数
            </div>
        `;
        
        // 更新显示
        updateRobotPose();
        
        alert('✓ 切换成功！\n\n现在使用从 URDF 提取的 MDH 参数进行运动学计算。\n\n黄色球应该与 URDF 末端完全对齐！');
        
    } catch (error) {
        console.error('切换失败:', error);
        alert('❌ 切换失败\n\n' + error.message);
    }
}

/**
 * 设置浮动面板的折叠/展开和拖动功能
 */
function setupFloatingPanels() {
    const panels = document.querySelectorAll('.floating-panel');
    panels.forEach((panel) => {
        makePanelDraggable(panel);
        panel.addEventListener('mousedown', () => bringPanelToFront(panel));
    });

    document.querySelectorAll('.panel-toggle').forEach((toggle) => {
        const panel = toggle.closest('.floating-panel');
        if (!panel) {
            return;
        }
        toggle.addEventListener('click', (event) => {
            event.stopPropagation();
            hidePanel(panel.id);
        });
    });

    // ROS 2 通讯事件处理
    setupROS2EventListeners();
    setupInverseKinematicsControls();
    setupDQCalculatorPanel();
    
    initializeDock();
    refreshDockState();
}

/**
 * 设置 ROS 2 事件监听器
 */
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
            if (ros2Communicator && ros2Communicator.isConnected) {
                ros2Communicator.shutdown();
            }
            
            // 保存新地址
            if (ros2Communicator) {
                ros2Communicator.setWebSocketUrl(url);
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
            if (!ros2Communicator) {
                // 先读取 URL 输入框的值
                const urlInput = document.getElementById('ros2-websocket-url');
                const customUrl = urlInput ? urlInput.value.trim() : null;
                
                await initROS2Communication(customUrl);
            } else if (ros2Communicator.getStatus() === 'connected') {
                await ros2Communicator.shutdown();
                updateROS2StatusDisplay('disconnected');
            } else {
                await ros2Communicator.init();
            }
        });
    }

    updateIKRobotSelection();
    
    // ROS 2 切换控制模式按钮
    const switchModeBtn = document.getElementById('ros2-switch-mode-btn');
    if (switchModeBtn) {
        switchModeBtn.addEventListener('click', async () => {
            // 防止快速多次点击
            if (switchModeBtnProcessing) {
                return;
            }

            if (!ros2Communicator || !ros2Communicator.isInitialized) {
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
                await ros2Communicator.switchMode(selectedMode);
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
            if (ros2Communicator && ros2Communicator.isInitialized) {
                updateROS2ActiveTopics();
                addROS2LogEntry('刷新话题列表', 'info');
            }
        });
    }
    
    // ROS 2 订阅按钮
    const subscribeBtn = document.getElementById('ros2-subscribe-btn');
    if (subscribeBtn) {
        subscribeBtn.addEventListener('click', async () => {
            if (!ros2Communicator || !ros2Communicator.isInitialized) {
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
                const result = await ros2Communicator.subscribe(topicName, messageType, (message) => {
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
            if (!ros2Communicator || !ros2Communicator.isInitialized) {
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
                const message = ros2Communicator.rclnodejs.createMessageObject(messageType);
                const result = await ros2Communicator.publish(topicName, messageType, message);
                
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
            if (!ros2Communicator || !ros2Communicator.isInitialized) {
                addROS2LogEntry('ROS 2 未连接', 'error');
                return;
            }
            
            const topicName = document.getElementById('ros2-topic-name').value.trim() || '/cmd_vel';
            
            try {
                const linearX = parseFloat(document.getElementById('ros2-twist-linear-x').value) || 0;
                const linearY = parseFloat(document.getElementById('ros2-twist-linear-y').value) || 0;
                const linearZ = parseFloat(document.getElementById('ros2-twist-linear-z').value) || 0;
                const angularZ = parseFloat(document.getElementById('ros2-twist-angular-z').value) || 0;
                
                const result = await ros2Communicator.publishTwist(
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
    if (!ros2Communicator || !ros2Communicator.isInitialized) {
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
        if (dualArmMode) {
            // 双臂模式：根据URDF定义，robot1(panda_1)是右臂，robot2(panda_2)是左臂
            const angles1 = jointAngles1 || [];  // robot1 (panda_1) = 右臂
            const angles2 = jointAngles2 || [];  // robot2 (panda_2) = 左臂

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
            const angles = jointAngles || [];

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
        const result = await ros2Communicator.publishRobotArmServoMsg(topicName, messageData);
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
    if (jointPositionPublishTimer !== null) {
        clearInterval(jointPositionPublishTimer);
    }

    // 检查ROS2连接
    if (!ros2Communicator || !ros2Communicator.isInitialized) {
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
    jointPositionPublishTimer = setInterval(() => {
        publishJointPositionMessage();
    }, JOINT_POSITION_PUBLISH_INTERVAL);

    updateJointPositionStatus(`正在持续发布到: ${topicName} (${1000/JOINT_POSITION_PUBLISH_INTERVAL}Hz)`, 'success');
    addROS2LogEntry(`开始持续发布关节位置到: ${topicName}`, 'info');
}

/**
 * 停止持续发布关节位置
 */
function stopJointPositionPublishing() {
    if (jointPositionPublishTimer !== null) {
        clearInterval(jointPositionPublishTimer);
        jointPositionPublishTimer = null;
    }
    updateJointPositionStatus('已停止发布', 'info');
    addROS2LogEntry('已停止关节位置发布', 'info');
}

function setupDQCalculatorPanel() {
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

/**
 * 获取当前选中机器人的双四元数
 * @param {string} robotChoice 
 * @returns {number[]|null}
 */
function getCurrentRobotDQ(robotChoice) {
    if (!window.DQModule || !window.DQModule.DQWrapper) {
        console.warn('DQ 模块未加载');
        return null;
    }

    try {
        if (dualArmMode) {
            // 双臂：支持机器人1/2 + 绝对/相对位姿
            const da = window.dualArmSystem || dualArmSystem;
            if (robotChoice === 'absolute') {
                if (!da || !window.jointAngles1?.length || !window.jointAngles2?.length) {
                    return null;
                }
                const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
                return da.getAbsolutePose(allJointAngles);
            }
            if (robotChoice === 'relative') {
                if (!da || !window.jointAngles1?.length || !window.jointAngles2?.length) {
                    return null;
                }
                const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
                return da.getRelativePose(allJointAngles);
            }
            if (robotChoice === 'robot2') {
                if (!window.robot2 || !window.jointAngles2?.length) {
                    return null;
                }
                return window.robot2.fkm(window.jointAngles2);
            }
            if (!window.robot1 || !window.jointAngles1?.length) {
                return null;
            }
            return window.robot1.fkm(window.jointAngles1);
        }

        if (!window.currentRobot || !window.jointAngles?.length) {
            return null;
        }
        return window.currentRobot.fkm(window.jointAngles);
    } catch (error) {
        console.error('获取当前双四元数失败:', error);
        return null;
    }
}

function formatDualQuaternion(dqArray) {
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

/**
 * 更新 IK 机器人选择依据单/双臂
 */
function updateIKRobotSelection() {
    const ikSelect = document.getElementById('ik-robot-select');
    const ikStatus = document.getElementById('ik-status');
    if (!ikSelect) return;

    const prevValue = ikSelect.value;
    ikSelect.innerHTML = '';
    const opt1 = document.createElement('option');
    opt1.value = 'robot1';
    opt1.textContent = dualArmMode ? '机器人1' : '机器人';
    ikSelect.appendChild(opt1);

    if (dualArmMode) {
        const opt2 = document.createElement('option');
        opt2.value = 'robot2';
        opt2.textContent = '机器人2';
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
            ikStatus.textContent = '双臂模式：可选择机器人1/机器人2/绝对位姿/相对位姿';
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

function updateDQCalculatorRobotSelection() {
    const select = document.getElementById('dq-calculator-robot');
    if (!select) {
        return;
    }

    const prevValue = select.value;
    select.innerHTML = '';
    const opt1 = document.createElement('option');
    opt1.value = 'robot1';
    opt1.textContent = dualArmMode ? '机器人1' : '机器人';
    select.appendChild(opt1);

    if (dualArmMode) {
        const opt2 = document.createElement('option');
        opt2.value = 'robot2';
        opt2.textContent = '机器人2';
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

/**
 * 控制面板 - 逆运动学按钮
 */
function setupInverseKinematicsControls() {
    const ikSolveBtn = document.getElementById('ik-solve-btn');
    const copyDqBtn = document.getElementById('copy-dq-btn');
    const ikStatus = document.getElementById('ik-status');
    if (!ikSolveBtn && !copyDqBtn) {
        return;
    }

    const ikSelect = document.getElementById('ik-robot-select');
    const updateIkSolveAvailability = () => {
        // 双臂模式下也允许求解 absolute/relative：
        // - absolute：保持相对位姿不变
        // - relative：保持绝对位姿不变
        if (!ikStatus) return;
        if (!dualArmMode) {
            ikStatus.textContent = '单臂模式：使用雅可比伪逆更新当前机器人角度';
            ikStatus.style.color = '#94a3b8';
            return;
        }
        const choice = ikSelect?.value || 'robot1';
        if (choice === 'absolute') {
            ikStatus.textContent = '双臂绝对位姿：求解绝对位姿，保持相对位姿不变';
            ikStatus.style.color = '#94a3b8';
            return;
        }
        if (choice === 'relative') {
            ikStatus.textContent = '双臂相对位姿：求解相对位姿，保持绝对位姿不变';
            ikStatus.style.color = '#94a3b8';
            return;
        }
        ikStatus.textContent = '双臂单臂求解：仅更新机器人1或机器人2角度';
        ikStatus.style.color = '#94a3b8';
    };

    ikSolveBtn.addEventListener('click', () => {
        const dqText = document.getElementById('ik-dq-input')?.value || '';
        const dqArray = parseDualQuaternionInput(dqText);
        if (!dqArray) {
            addROS2LogEntry('请输入 8 个双四元数元素（逗号或空格分隔）', 'warn');
            if (ikStatus) {
                ikStatus.textContent = '请输入 8 个双四元数元素（逗号或空格分隔）';
                ikStatus.style.color = '#f87171';
            }
            return;
        }

        const robotChoice = document.getElementById('ik-robot-select')?.value || 'robot1';
        const vis = window.visualizer;
        if (!vis) {
            addROS2LogEntry('视觉器未初始化，无法求解逆运动学', 'error');
            if (ikStatus) {
                ikStatus.textContent = '视觉器未初始化，无法求解逆运动学';
                ikStatus.style.color = '#f87171';
            }
            return;
        }

        try {
            let success = false;
            if (!dualArmMode) {
                success = vis.updateSingleArmJointsWithPseudoInverse(dqArray) === true;
            } else if (robotChoice === 'robot1') {
                success = vis.updateRobot1JointsWithPseudoInverse(dqArray) === true;
            } else if (robotChoice === 'robot2') {
                success = vis.updateRobot2JointsWithPseudoInverse(dqArray) === true;
            } else if (robotChoice === 'absolute') {
                const allJointAngles = [...(window.jointAngles1 || []), ...(window.jointAngles2 || [])];
                success = vis.updateDualArmJointsFromAbsolutePoseTarget?.(dqArray, allJointAngles) === true;
            } else if (robotChoice === 'relative') {
                const allJointAngles = [...(window.jointAngles1 || []), ...(window.jointAngles2 || [])];
                success = vis.updateDualArmJointsFromRelativePoseTarget?.(dqArray, allJointAngles) === true;
            } else {
                // 兜底：防止误选 absolute/relative 时错误调用 robot2
                addROS2LogEntry('当前逆运动学求解仅支持机器人1/机器人2', 'warn');
                if (ikStatus) {
                    ikStatus.textContent = '当前逆运动学求解仅支持机器人1/机器人2';
                    ikStatus.style.color = '#f59e0b';
                }
                return;
            }
            if (!success) {
                addROS2LogEntry('逆运动学求解失败：伪逆求解未收敛', 'warn');
                if (ikStatus) {
                    ikStatus.textContent = '逆运动学求解失败：伪逆求解未收敛';
                    ikStatus.style.color = '#f87171';
                }
                return;
            }
            if (typeof updateJointSlidersFromAngles === 'function') {
                updateJointSlidersFromAngles();
            }
            if (typeof updateDualArmJointSlidersFromAngles === 'function') {
                updateDualArmJointSlidersFromAngles();
            }
            const label = robotChoice === 'robot1'
                ? '机器人1'
                : robotChoice === 'robot2'
                    ? '机器人2'
                    : robotChoice === 'absolute'
                        ? '绝对位姿（保持相对不变）'
                        : robotChoice === 'relative'
                            ? '相对位姿（保持绝对不变）'
                            : robotChoice;
            addROS2LogEntry(`逆运动学求解成功，已更新 ${label}`, 'info');
            if (ikStatus) {
                ikStatus.textContent = `逆运动学求解成功，已更新 ${label}`;
                ikStatus.style.color = '#10b981';
            }
        } catch (error) {
            console.error('逆运动学求解失败:', error);
            addROS2LogEntry(`逆运动学失败：${error.message}`, 'error');
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
                addROS2LogEntry('无法获取当前姿态的双四元数', 'warn');
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
                addROS2LogEntry(`已复制双四元数：${formatted}`, 'info');
                if (ikStatus) {
                    ikStatus.textContent = '已复制当前姿态的双四元数';
                    ikStatus.style.color = '#10b981';
                }
            } catch (error) {
                console.error('复制双四元数失败:', error);
                addROS2LogEntry(`复制双四元数失败：${error.message}`, 'error');
                if (ikStatus) {
                    ikStatus.textContent = `复制失败：${error.message}`;
                    ikStatus.style.color = '#f87171';
                }
            }
        });
    }
}

function setupRealJointStateControls() {
    const subscribeBtn = document.getElementById('ros2-joint-subscribe-btn');
    const topicInput = document.getElementById('ros2-joint-topic');
    ros2JointStatusElement = document.getElementById('ros2-joint-status');
    ros2JointStateButton = subscribeBtn;

    if (!subscribeBtn) {
        return;
    }

    updateJointStateSubscribeButton();
    setRos2JointStatus(ros2JointStateSubscribed ? `已订阅 ${ros2JointStateTopic}` : '未订阅真机关节状态', 'info');

    subscribeBtn.addEventListener('click', async () => {
        if (!ros2Communicator || !ros2Communicator.isInitialized) {
            setRos2JointStatus('请先连接 ROS 2', 'error');
            return;
        }

        const topic = topicInput?.value.trim() || '/arm/joint_states';

        if (!ros2JointStateSubscribed) {
            const success = await ros2Communicator.subscribe(topic, 'sensor_msgs/msg/JointState', handleRealJointStateMessage);
            if (success) {
                ros2JointStateSubscribed = true;
                ros2JointStateTopic = topic;
                updateJointStateSubscribeButton();
                setRos2JointStatus(`已订阅 ${topic}`, 'success');
                addROS2LogEntry(`已订阅真机关节: ${topic}`, 'info');
            } else {
                setRos2JointStatus('订阅失败，请检查话题', 'error');
            }
        } else {
            ros2Communicator.unsubscribe(ros2JointStateTopic);
            ros2JointStateSubscribed = false;
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

    const updated = applyJointStatesFromMessage(names, positions);
    if (updated) {
        if (typeof updateJointSlidersFromAngles === 'function') {
            updateJointSlidersFromAngles();
        }
        if (typeof updateDualArmJointSlidersFromAngles === 'function') {
            updateDualArmJointSlidersFromAngles();
        }
        if (window.updateGlobalVariables) {
            window.updateGlobalVariables();
        }
        if (window.updateRobotPose) {
            window.updateRobotPose();
        }
        setRos2JointStatus(`已接收 ${new Date().toLocaleTimeString()}`, 'success');
    }
}

function applyJointStatesFromMessage(names, positions) {
    if (dualArmMode) {
        const updatedRight = updateRobotAnglesFromJointState(names, positions, 'robot1');
        const updatedLeft = updateRobotAnglesFromJointState(names, positions, 'robot2');
        return updatedRight || updatedLeft;
    }
    return updateRobotAnglesFromJointState(names, positions, 'single');
}

function updateRobotAnglesFromJointState(names, positions, robotKey) {
    let jointAngles;
    let jointOrder;
    let urdf = null;
    let config = null;

    if (robotKey === 'single') {
        jointAngles = window.jointAngles;
        jointOrder = window.customRobotConfig?.joint_chain?.joints || null;
        urdf = window.visualizer?.urdfRobot || null;
        config = window.customRobotConfig || null;
    } else if (robotKey === 'robot1') {
        jointAngles = window.jointAngles1;
        jointOrder = window.customRobotConfig?.robot1?.joint_chain?.joints || null;
        urdf = window.visualizer?.robot1URDF || null;
        config = window.customRobotConfig?.robot1 || null;
    } else {
        jointAngles = window.jointAngles2;
        jointOrder = window.customRobotConfig?.robot2?.joint_chain?.joints || null;
        urdf = window.visualizer?.robot2URDF || null;
        config = window.customRobotConfig?.robot2 || null;
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
        if (dualArmMode) {
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

    if (jointOrder && typeof window.clampJointAngles === 'function') {
        const clamped = window.clampJointAngles(newAngles, jointOrder, urdf, config);
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
    if (ros2JointStateButton) {
        ros2JointStateButton.textContent = ros2JointStateSubscribed ? '取消订阅' : '订阅';
    }
}

function setRos2JointStatus(message, type = 'info') {
    if (!ros2JointStatusElement) {
        return;
    }
    ros2JointStatusElement.textContent = message;
    ros2JointStatusElement.style.color = type === 'success' ? '#10b981' : type === 'error' ? '#f87171' : '#94a3b8';
}

/**
 * 订阅电池状态话题
 */
async function subscribeBatteryState() {
    if (!ros2Communicator || !ros2Communicator.isInitialized) {
        console.warn('ROS 2 未初始化，无法订阅电池状态');
        return false;
    }

    if (ros2BatteryStateSubscribed) {
        console.log('电池状态已订阅');
        return true;
    }

    try {
        const success = await ros2Communicator.subscribe(
            ROS2_BATTERY_STATE_TOPIC,
            'sensor_msgs/msg/BatteryState',
            handleBatteryStateMessage
        );
        
        if (success) {
            ros2BatteryStateSubscribed = true;
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
    console.log(batteryState)
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

/**
 * 使面板可拖动
 */
function makePanelDraggable(panel) {
    const header = panel.querySelector('.panel-header');
    if (!header) return;
    
    let isDragging = false;
    let currentX;
    let currentY;
    let initialX;
    let initialY;
    let xOffset = 0;
    let yOffset = 0;
    
    // 获取当前面板的位置
    const rect = panel.getBoundingClientRect();
    xOffset = rect.left;
    yOffset = rect.top;
    
    header.addEventListener('mousedown', dragStart);
    document.addEventListener('mousemove', drag);
    document.addEventListener('mouseup', dragEnd);
    
    function dragStart(e) {
        // 如果点击的是按钮，不触发拖动
        if (e.target.classList.contains('panel-toggle')) {
            return;
        }
        
        initialX = e.clientX - xOffset;
        initialY = e.clientY - yOffset;
        
        isDragging = true;
        header.style.cursor = 'grabbing';
        bringPanelToFront(panel);
    }
    
    function drag(e) {
        if (isDragging) {
            e.preventDefault();
            
            currentX = e.clientX - initialX;
            currentY = e.clientY - initialY;
            
            xOffset = currentX;
            yOffset = currentY;
            
            // 限制在视口内
            const maxX = window.innerWidth - panel.offsetWidth;
            const maxY = window.innerHeight - panel.offsetHeight;
            
            xOffset = Math.max(0, Math.min(xOffset, maxX));
            yOffset = Math.max(0, Math.min(yOffset, maxY));
            
            setTranslate(xOffset, yOffset, panel);
        }
    }
    
    function dragEnd() {
        if (isDragging) {
            initialX = currentX;
            initialY = currentY;
            isDragging = false;
            header.style.cursor = 'grab';
        }
    }
    
    function setTranslate(xPos, yPos, el) {
        el.style.left = xPos + 'px';
        el.style.top = yPos + 'px';
        el.style.right = 'auto';
        el.style.bottom = 'auto';
    }
}

function bringPanelToFront(panel) {
    if (!panel) {
        return;
    }
    panelStackIndex += 1;
    panel.style.zIndex = panelStackIndex;
}

function initializeDock() {
    dockElement = document.getElementById('bottom-dock');
    if (!dockElement) {
        return;
    }

    const dockItems = dockElement.querySelectorAll('.dock-item');
    dockItems.forEach((item) => {
        const panelId = item.dataset.panel;
        if (!panelId) {
            return;
        }

        item.addEventListener('click', () => {
            togglePanelVisibility(panelId);
        });
    });
}

function refreshDockState() {
    if (!dockElement) {
        dockElement = document.getElementById('bottom-dock');
        if (!dockElement) {
            return;
        }
    }

    const dockItems = dockElement.querySelectorAll('.dock-item');
    dockItems.forEach((item) => {
        const panelId = item.dataset.panel;
        const panel = document.getElementById(panelId);
        const isActive = !!panel && !panel.classList.contains('hidden');
        item.classList.toggle('active', isActive);
    });
}

function updateDockItemState(panelId, isActive) {
    if (!dockElement) {
        dockElement = document.getElementById('bottom-dock');
        if (!dockElement) {
            return;
        }
    }
    const item = dockElement.querySelector(`.dock-item[data-panel="${panelId}"]`);
    if (!item) {
        return;
    }
    item.classList.toggle('active', isActive);
}

function togglePanelVisibility(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) {
        return;
    }
    if (panel.classList.contains('hidden')) {
        showPanel(panelId);
    } else {
        hidePanel(panelId);
    }
}

function showPanel(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) {
        return;
    }
    panel.classList.remove('hidden');
    panel.classList.remove('collapsed');
    const toggle = panel.querySelector('.panel-toggle');
    if (toggle) {
        toggle.textContent = '−';
    }
    bringPanelToFront(panel);
    updateDockItemState(panelId, true);
}

function hidePanel(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) {
        return;
    }
    if (!panel.classList.contains('hidden')) {
        panel.classList.add('hidden');
    }
    const toggle = panel.querySelector('.panel-toggle');
    if (toggle) {
        toggle.textContent = '+';
    }
    updateDockItemState(panelId, false);
}

/**
 * ROS 2 消息日志记录
 */
function addROS2LogEntry(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const logEntry = `[${timestamp}] ${message}`;
    ros2MessageLog.push({ message: logEntry, type });
    
    if (ros2MessageLog.length > ROS2_MAX_LOG_ENTRIES) {
        ros2MessageLog.shift();
    }
    
    updateROS2LogDisplay();
}

/**
 * 更新 ROS 2 消息日志显示
 */
function updateROS2LogDisplay() {
    const logContainer = document.getElementById('ros2-message-log');
    if (!logContainer) return;
    
    if (ros2MessageLog.length === 0) {
        logContainer.innerHTML = '<div style="color: #94a3b8; font-size: 0.85rem;">暂无消息</div>';
        return;
    }
    
    logContainer.innerHTML = ros2MessageLog.map(entry => 
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
    if (!topicsContainer || !ros2Communicator) return;
    
    const subscriptions = ros2Communicator.getActiveSubscriptions();
    const publishers = ros2Communicator.getActivePublishers();
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
    if (ros2Communicator) {
        ros2Communicator.unsubscribe(topicName);
        updateROS2ActiveTopics();
        addROS2LogEntry(`取消订阅: ${topicName}`, 'info');
    }
}

/**
 * 初始化 ROS 2 通讯
 */
async function initROS2Communication(customUrl = null) {
    try {
        // 如果提供了自定义 URL，使用它；否则让构造函数从本地存储读取
        ros2Communicator = new ROS2Communicator(customUrl);
        
        // 注册状态变化回调
        ros2Communicator.onStatusChange((status) => {
            updateROS2StatusDisplay(status);
            const statusText = {
                'disconnected': '未连接',
                'connecting': '连接中...',
                'connected': '已连接',
                'error': '错误'
            }[status] || status;
            addROS2LogEntry(`状态: ${statusText}`, 'info');
            if (status === 'disconnected' && ros2JointStateSubscribed) {
                ros2JointStateSubscribed = false;
                updateJointStateSubscribeButton();
                setRos2JointStatus('连接断开，已停止订阅', 'error');
            }
            // 如果断开连接，停止电池状态订阅
            if (status === 'disconnected' && ros2BatteryStateSubscribed) {
                ros2BatteryStateSubscribed = false;
            }
            // 如果断开连接，停止关节位置发布
            if (status === 'disconnected' || status === 'error') {
                if (jointPositionPublishTimer !== null) {
                    stopJointPositionPublishing();
                    const toggle = document.getElementById('ros2-joint-position-toggle');
                    if (toggle) toggle.checked = false;
                }
            }
        });

        // 注册模式变化回调
        ros2Communicator.onModeChange((mode) => {
            updateROS2ControlModeDisplay(mode);
            addROS2LogEntry(`当前控制模式: ${mode}`, 'info');
        });
        
        // 显示连接的 URL
        addROS2LogEntry(`连接地址: ${ros2Communicator.getWebSocketUrl()}`, 'info');
        
        // 尝试初始化
        const result = await ros2Communicator.init();
        if (result) {
            addROS2LogEntry('ROS 2 初始化成功', 'info');
            // 查询当前控制模式
            try {
                await ros2Communicator.getMode();
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
 * 切换到双臂UI显示
 */
function switchToDualArmUI() {
    // 隐藏单臂位姿显示
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) {
        singleArmPose.style.display = 'none';
    }
    
    // 显示双臂位姿显示
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'block';
        dualArmPose.classList.add('active');
    }
    
    // 显示双臂控制模式切换开关
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) {
        dualArmControlMode.style.display = 'block';
    }
    
    // 设置控制模式选择器的事件监听器
    const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const modeLabel = document.getElementById('dual-arm-mode-label');
    if (controlModeSelect && modeLabel) {
        const updateModeLabel = () => {
            const mode = controlModeSelect.value;
            if (typeof window !== 'undefined') {
                window.currentDualArmControlMode = mode;
            }
            switch (mode) {
                case 'robot1':
                    modeLabel.textContent = '机器人1模式：控制机器人1的末端执行器';
                    break;
                case 'robot2':
                    modeLabel.textContent = '机器人2模式：控制机器人2的末端执行器';
                    break;
                case 'relative':
                    modeLabel.textContent = '相对位姿模式：控制相对位姿，保持绝对位姿不变';
                    break;
                case 'absolute':
                default:
                    modeLabel.textContent = '绝对位姿模式：控制绝对位姿，保持相对位姿不变';
                    break;
            }
            renderEnvironmentObjectList();
        };
        updateModeLabel();
        controlModeSelect.addEventListener('change', updateModeLabel);
    }
    
    // 更新标签页，确保显示正确的标签内容
    if (dualArmPose && document.querySelector('[data-tab="pose"]')) {
        switchTab('pose');
    }

    renderEnvironmentObjectList();
    updateIKRobotSelection();
    updateDQCalculatorRobotSelection();
}

/**
 * 切换到单臂UI显示
 */
function switchToSingleArmUI() {
    // 显示单臂位姿显示
    const singleArmPose = document.querySelector('.single-arm-pose');
    if (singleArmPose) {
        singleArmPose.style.display = 'block';
    }
    
    // 隐藏双臂位姿显示
    const dualArmPose = document.getElementById('dual-arm-pose-tab');
    if (dualArmPose) {
        dualArmPose.style.display = 'none';
        dualArmPose.classList.remove('active');
    }
    
    // 隐藏双臂控制模式切换开关
    const dualArmControlMode = document.getElementById('dual-arm-control-mode');
    if (dualArmControlMode) {
        dualArmControlMode.style.display = 'none';
    }

    if (typeof window !== 'undefined') {
        window.currentDualArmControlMode = 'absolute';
    }

    renderEnvironmentObjectList();
    updateIKRobotSelection();
    updateDQCalculatorRobotSelection();
}

/**
 * 更新 ROS 2 控制模式显示
 */
function updateROS2ControlModeDisplay(mode) {
    const controlModeSelect = document.getElementById('ros2-control-mode');
    if (controlModeSelect) {
        controlModeSelect.value = mode;

        // 更新状态显示
        const modeStatusEl = document.getElementById('ros2-mode-status');
        if (modeStatusEl) {
            const modeText = mode === 'manual' ? '手动模式 - 可直接控制机器人关节' :
                           mode === 'teleop' ? '遥操作模式 - 通过VR设备控制' : '未知模式';
            modeStatusEl.textContent = `当前模式：${modeText}`;
        }
    }
}

// 页面加载完成后初始化
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}

