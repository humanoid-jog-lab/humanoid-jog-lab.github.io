/**
 * 关节角度剪贴板操作模块
 * 负责关节角度的复制/导入/解析，以及关节限位的剪贴板输出
 */

import * as THREE from 'three';
import { DEG_TO_RAD, RAD_TO_DEG } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { angleToDisplay, getActiveJointConfig, getInputPrecision } from '../utils/math-utils.js';

/**
 * 构建关节角度的剪贴板文本
 */
export function buildJointAnglesClipboardText() {
    const digits = appState.angleDisplayMode === 'deg' ? 3 : 5;
    const formatList = (angles = []) => {
        const list = Array.isArray(angles) ? angles : Array.from(angles || []);
        const formatted = list.map(a => parseFloat(angleToDisplay(a ?? 0).toFixed(digits)));
        return `[${formatted.join(', ')}]`;
    };

    if (appState.dualArmMode) {
        const robot1 = formatList(appState.jointAngles1);
        const robot2 = formatList(appState.jointAngles2);
        return `robot1=${robot1}\nrobot2=${robot2}`;
    }
    if (appState.humanoidMode && appState.humanoidQ.length > 0) {
        // 人形模式：使用 humanoid= 前缀，方便导入时识别
        return `humanoid=${formatList(appState.humanoidQ)}`;
    }

    return formatList(appState.jointAngles);
}

/**
 * 解析关节角度数组字符串
 */
export function parseJointAngleArray(arrayStr) {
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

/**
 * 解析关节角度输入文本（支持单臂、双臂、人形格式）
 */
export function parseJointAnglesInput(text) {
    const trimmed = (text || '').trim();
    if (!trimmed) {
        throw new Error('未提供任何角度数据');
    }

    if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
        return { single: parseJointAngleArray(trimmed) };
    }

    const result = {};
    const normalized = trimmed.replace(/\r\n/g, '\n');
    
    // 匹配 humanoid= 格式（人形机器人）
    const humanoidRegex = /humanoid\s*[:=]\s*(\[[^\]]*\])/gi;
    let humanoidMatch = humanoidRegex.exec(normalized);
    if (humanoidMatch) {
        result.humanoid = parseJointAngleArray(humanoidMatch[1]);
    }
    
    // 匹配 robot1= 和 robot2= 格式（双臂模式）
    const regex = /(robot1|robot2)\s*[:=]\s*(\[[^\]]*\])/gi;
    let match;
    while ((match = regex.exec(normalized)) !== null) {
        const key = match[1].toLowerCase();
        result[key] = parseJointAngleArray(match[2]);
    }

    if (Object.keys(result).length === 0) {
        throw new Error('未检测到有效的角度数组格式（支持: [...]、humanoid=[...]、robot1=[...] robot2=[...]）');
    }

    return result;
}

/**
 * 从文本导入关节角度
 */
export function importJointAnglesFromText(text) {
    const parsed = parseJointAnglesInput(text);
    const toRad = (value) => (appState.angleDisplayMode === 'deg' ? value * DEG_TO_RAD : value);

    // 人形模式
    if (appState.humanoidMode && appState.humanoidQ.length > 0) {
        const arr = parsed.humanoid || parsed.single;
        if (!arr) {
            throw new Error('人形模式需要提供一个角度数组（格式: humanoid=[...] 或 [...]）');
        }
        if (arr.length !== appState.humanoidQ.length) {
            throw new Error(`导入数组长度 ${arr.length} 与人形机器人关节数 ${appState.humanoidQ.length} 不匹配`);
        }

        appState.humanoidQ = arr.map(toRad);

        // 应用关节限位
        if (appState.customRobotConfig && appState.humanoidJointNames.length > 0) {
            const urdfRobot = appState.visualizer?.humanoidURDF || null;
            appState.humanoidQ = callbacks.clampJointAngles?.(appState.humanoidQ, appState.humanoidJointNames, urdfRobot, appState.customRobotConfig);
        }

        callbacks.updateGlobalVariables?.();
        callbacks.updateRobotPose?.();
        callbacks.updateHumanoidJointSlidersFromAngles?.();
        return;
    }

    // 双臂模式
    if (appState.dualArmMode) {
        const arr1 = parsed.robot1 || parsed.single;
        const arr2 = parsed.robot2;

        if (!arr1 || !arr2) {
            throw new Error('双臂模式需要同时提供 robot1 和 robot2 的角度数组');
        }

        if (arr1.length !== appState.jointAngles1.length || arr2.length !== appState.jointAngles2.length) {
            throw new Error('导入数组长度与当前机器人关节数不匹配');
        }

        appState.jointAngles1 = arr1.map(toRad);
        appState.jointAngles2 = arr2.map(toRad);

        if (window.customRobotConfig?.robot1) {
            const jointNames1 = window.customRobotConfig.robot1.joint_chain?.joints || [];
            appState.jointAngles1 = callbacks.clampJointAngles?.(appState.jointAngles1, jointNames1, appState.visualizer?.robot1URDF || null, window.customRobotConfig.robot1);
        }
        if (window.customRobotConfig?.robot2) {
            const jointNames2 = window.customRobotConfig.robot2.joint_chain?.joints || [];
            appState.jointAngles2 = callbacks.clampJointAngles?.(appState.jointAngles2, jointNames2, appState.visualizer?.robot2URDF || null, window.customRobotConfig.robot2);
        }

        callbacks.updateGlobalVariables?.();
        callbacks.updateRobotPose?.();
        callbacks.updateDualArmJointSlidersFromAngles?.();
        return;
    }
    
    // 单臂模式
    const arr = parsed.single || parsed.robot1 || parsed.robot2;
    if (!arr) {
        throw new Error('单臂模式需要提供一个角度数组');
    }
    if (arr.length !== appState.jointAngles.length) {
        throw new Error('导入数组长度与当前机器人关节数不匹配');
    }

    appState.jointAngles = arr.map(toRad);

    const config = getActiveJointConfig();
    const jointNames = config?.joint_chain?.joints || [];
    const urdfRobot = appState.visualizer?.urdfRobot || null;
    if (jointNames.length === appState.jointAngles.length) {
        appState.jointAngles = callbacks.clampJointAngles?.(appState.jointAngles, jointNames, urdfRobot, config);
    }

    callbacks.updateGlobalVariables?.();
    callbacks.updateRobotPose?.();
    callbacks.updateJointSlidersFromAngles?.();
}

/**
 * 格式化关节限位用于剪贴板
 */
export function formatJointLimitsForClipboard(lowerAngles = [], upperAngles = [], label = 'joint') {
    const listLower = Array.isArray(lowerAngles) ? lowerAngles : Array.from(lowerAngles || []);
    const listUpper = Array.isArray(upperAngles) ? upperAngles : Array.from(upperAngles || []);
    const digits = appState.angleDisplayMode === 'deg' ? 3 : 5;
    const lowerDisplay = listLower.map(a => parseFloat(angleToDisplay(a ?? 0).toFixed(digits)));
    const upperDisplay = listUpper.map(a => parseFloat(angleToDisplay(a ?? 0).toFixed(digits)));
    return `${label}_q_min: [${lowerDisplay.join(', ')}]\n${label}_q_max: [${upperDisplay.join(', ')}]`;
}

/**
 * 构建关节限位的剪贴板文本
 */
export function buildJointLimitsClipboardText() {
    const config = getActiveJointConfig();
    const formatLimits = (jointNames = [], urdfRobotRef = null, configRef = null, label = 'joint') => {
        const lower = [];
        const upper = [];
        jointNames.forEach(name => {
            const limits = callbacks.getJointLimits?.(name, urdfRobotRef, configRef) ?? { lower: -Math.PI, upper: Math.PI };
            lower.push(limits.lower);
            upper.push(limits.upper);
        });
        return formatJointLimitsForClipboard(lower, upper, label);
    };

    // 人形模式
    if (appState.humanoidMode && appState.humanoidJointNames.length > 0) {
        const urdfRobot = appState.visualizer?.humanoidURDF || null;
        return formatLimits(appState.humanoidJointNames, urdfRobot, config, 'humanoid');
    }

    // 双臂模式
    if (appState.dualArmMode && config) {
        const jointNames1 = config.robot1?.joint_chain?.joints || [];
        const jointNames2 = config.robot2?.joint_chain?.joints || [];
        const text1 = formatLimits(jointNames1, appState.visualizer?.robot1URDF || null, config.robot1, 'robot1');
        const text2 = formatLimits(jointNames2, appState.visualizer?.robot2URDF || null, config.robot2, 'robot2');
        return `${text1}\n${text2}`;
    }

    // 单臂模式
    const jointNames = config?.joint_chain?.joints || [];
    return formatLimits(jointNames, appState.visualizer?.urdfRobot || null, config, 'joint');
}

/**
 * 复制关节限位到剪贴板
 */
export async function copyJointLimitsToClipboard() {
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

/**
 * 复制关节角度到剪贴板
 */
export async function copyJointAnglesToClipboard() {
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
export function computeEffectorOffsetFromURDF(robot, endLinkName) {
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
export function computeEffectorPoseFromURDF(robot, endLinkName) {
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
 * 注册本模块的回调函数
 */
export function registerJointClipboardCallbacks() {
    callbacks.importJointAnglesFromText = importJointAnglesFromText;
    callbacks.copyJointAnglesToClipboard = copyJointAnglesToClipboard;
    callbacks.copyJointLimitsToClipboard = copyJointLimitsToClipboard;
}
