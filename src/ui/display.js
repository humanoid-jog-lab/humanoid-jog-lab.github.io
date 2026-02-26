import { appState, callbacks } from '../core/state.js';
import { getDQModule, quaternionToRotationMatrix, rotationMatrixToEuler } from '../dqrobotics.js';

/**
 * 更新位姿显示
 * @param {Object} pose - { translation, rotation }
 */
export function updatePoseDisplay(pose) {
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
 * 更新双臂/人形位姿显示
 * @param {Object} pose1 - 左臂/机器人1位姿 { translation, rotation }
 * @param {Object} pose2 - 右臂/机器人2位姿
 * @param {Object|null} optionalAbsolutePose - 可选，预计算的绝对位姿（人形时传入）
 * @param {Object|null} optionalRelativePose - 可选，预计算的相对位姿（人形时传入）
 * @param {Object|null} optionalDQData - 可选，预计算的原始 DQ 数据
 */
export function updateDualArmPoseDisplay(pose1, pose2, optionalAbsolutePose, optionalRelativePose, optionalDQData = null) {
    const isDQArray = (arr) => Array.isArray(arr) && arr.length >= 8;
    const providedDQ = optionalDQData || {};

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
    
    // 获取绝对位姿（使用absolute_pose函数或人形预计算值）
    let absX, absY, absZ, absQW, absQX, absQY, absQZ, absDQ = isDQArray(providedDQ.absoluteDQ) ? providedDQ.absoluteDQ : null;
    
    if (optionalAbsolutePose) {
        absX = optionalAbsolutePose.translation[1] || 0;
        absY = optionalAbsolutePose.translation[2] || 0;
        absZ = optionalAbsolutePose.translation[3] || 0;
        absQW = optionalAbsolutePose.rotation[0] || 1;
        absQX = optionalAbsolutePose.rotation[1] || 0;
        absQY = optionalAbsolutePose.rotation[2] || 0;
        absQZ = optionalAbsolutePose.rotation[3] || 0;
        if (!absDQ) {
            // 从位置和四元数构建DQ数组用于显示（仅在未提供原始DQ时回退）
            try {
                const dqModule = getDQModule();
                if (dqModule && dqModule.DQWrapper) {
                    const absDQObj = dqModule.DQWrapper.fromQuaternionAndTranslation(absQW, absQX, absQY, absQZ, absX, absY, absZ);
                    absDQ = dqModule.DQWrapper.toArray(absDQObj);
                }
            } catch (e) {
                console.warn('构建绝对位姿DQ失败:', e);
            }
        }
    } else if (appState.dualArmSystem && appState.jointAngles1.length > 0 && appState.jointAngles2.length > 0) {
        const allJointAngles = [...appState.jointAngles1, ...appState.jointAngles2];
        try {
            const absPose = appState.dualArmSystem.getAbsoluteWorldPose(allJointAngles);
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
            const absDQArray = appState.dualArmSystem.getAbsolutePose(allJointAngles);
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
    
    // 计算相对位姿（使用relative_pose函数或人形预计算值）
    let relX, relY, relZ, relQW, relQX, relQY, relQZ, relDQ = isDQArray(providedDQ.relativeDQ) ? providedDQ.relativeDQ : null;
    
    if (optionalRelativePose) {
        relX = optionalRelativePose.translation[1] || 0;
        relY = optionalRelativePose.translation[2] || 0;
        relZ = optionalRelativePose.translation[3] || 0;
        relQW = optionalRelativePose.rotation[0] || 1;
        relQX = optionalRelativePose.rotation[1] || 0;
        relQY = optionalRelativePose.rotation[2] || 0;
        relQZ = optionalRelativePose.rotation[3] || 0;
        if (!relDQ) {
            // 从位置和四元数构建DQ数组用于显示（仅在未提供原始DQ时回退）
            try {
                const dqModule = getDQModule();
                if (dqModule && dqModule.DQWrapper) {
                    const relDQObj = dqModule.DQWrapper.fromQuaternionAndTranslation(relQW, relQX, relQY, relQZ, relX, relY, relZ);
                    relDQ = dqModule.DQWrapper.toArray(relDQObj);
                }
            } catch (e) {
                console.warn('构建相对位姿DQ失败:', e);
            }
        }
    } else if (appState.dualArmSystem && appState.jointAngles1.length > 0 && appState.jointAngles2.length > 0) {
        const allJointAngles = [...appState.jointAngles1, ...appState.jointAngles2];
        try {
            // 获取相对位姿的DQ表示
            relDQ = appState.dualArmSystem.getRelativePose(allJointAngles);
            
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
    
    // 优先显示原始运动学DQ（与计算器“加载当前姿态”一致）
    let dq1 = isDQArray(providedDQ.robot1DQ) ? providedDQ.robot1DQ : null;
    let dq2 = isDQArray(providedDQ.robot2DQ) ? providedDQ.robot2DQ : null;
    try {
        const dqModule = getDQModule();
        const dqWrapper = dqModule?.DQWrapper;

        if ((!dq1 || !dq2) && dqWrapper && appState.humanoidMode && appState.humanoidModel && appState.humanoidQ?.length) {
            const baseX = appState.humanoidBasePose?.x || 0;
            const baseY = appState.humanoidBasePose?.y || 0;
            const baseYaw = appState.humanoidBasePose?.yaw || 0;
            const baseTrans = dqWrapper.translation(baseX, baseY, 0);
            const baseRot = dqWrapper.rotation(baseYaw, 0, 0, 1);
            const baseObj = dqWrapper.multiply(baseTrans, baseRot);

            if (!dq1) {
                const local1 = appState.humanoidModel.fkmEffector(appState.humanoidQ, 1);
                dq1 = dqWrapper.toArray(dqWrapper.multiply(baseObj, dqWrapper.createFromArray(local1)));
            }
            if (!dq2) {
                const local2 = appState.humanoidModel.fkmEffector(appState.humanoidQ, 2);
                dq2 = dqWrapper.toArray(dqWrapper.multiply(baseObj, dqWrapper.createFromArray(local2)));
            }
        }

        if ((!dq1 || !dq2) && !appState.humanoidMode) {
            if (!dq1 && appState.robot1 && appState.jointAngles1?.length) {
                dq1 = appState.robot1.fkm(appState.jointAngles1);
            }
            if (!dq2 && appState.robot2 && appState.jointAngles2?.length) {
                dq2 = appState.robot2.fkm(appState.jointAngles2);
            }
        }

        if (dqWrapper) {
            if (!dq1) {
                const dq1Obj = dqWrapper.fromQuaternionAndTranslation(qw1, qx1, qy1, qz1, x1, y1, z1);
                dq1 = dqWrapper.toArray(dq1Obj);
            }
            if (!dq2) {
                const dq2Obj = dqWrapper.fromQuaternionAndTranslation(qw2, qx2, qy2, qz2, x2, y2, z2);
                dq2 = dqWrapper.toArray(dq2Obj);
            }
        }
    } catch (error) {
        console.warn('获取左右臂DQ显示失败:', error);
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
export function updateDQDisplay() {
    try {
        const dqArray = appState.currentRobot.fkm(appState.jointAngles);
        
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
export function updateJacobianDisplay() {
    try {
        const jacobian = appState.currentRobot.poseJacobian(appState.jointAngles);
        
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
 * 注册显示回调到全局 callbacks
 */
export function registerDisplayCallbacks() {
    callbacks.updatePoseDisplay = updatePoseDisplay;
    callbacks.updateDualArmPoseDisplay = updateDualArmPoseDisplay;
    callbacks.updateDQDisplay = updateDQDisplay;
    callbacks.updateJacobianDisplay = updateJacobianDisplay;
}
