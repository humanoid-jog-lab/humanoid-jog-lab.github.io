/**
 * robot-manager.js — 机器人管理核心模块
 *
 * 负责机器人的初始化、位姿更新、URDF 加载、MDH 提取等。
 * 所有共享状态通过 appState 访问，跨模块调用通过 callbacks。
 */

import { appState, callbacks, updateWindowGlobals } from './state.js';
import {
    FrankaPanda,
    KukaLw4,
    SerialManipulator,
    DifferentialDriveRobot,
    CooperativeDualArm,
    HumanoidDualArmAdapter,
    getDQModule
} from '../dqrobotics.js';
import { RobotVisualizer } from '../visualizer.js';
import { extractMDHFromURDF, loadHumanoidChainsFromURDF } from '../urdf-to-mdh.js';
import { URDFPackageManager, loadURDFFromPackage } from '../urdf-loader.js';
import { computeEffectorPoseFromURDF } from '../features/joint-clipboard.js';

// ---------------------------------------------------------------------------
// initVisualizer
// ---------------------------------------------------------------------------

/**
 * 初始化可视化器
 */
export async function initVisualizer() {
    const canvas = document.getElementById('robot-canvas');
    appState.visualizer = new RobotVisualizer(canvas);
    window.visualizer = appState.visualizer;
    await appState.visualizer.init();

    // 根据UI初始化点动乘法顺序
    const teachOrderToggle = document.getElementById('teach-multiply-order-toggle');
    if (teachOrderToggle && appState.visualizer?.setTeachMultiplicationOrder) {
        const initialOrder = teachOrderToggle.checked ? 'delta-first' : 'current-first';
        appState.visualizer.setTeachMultiplicationOrder(initialOrder);
    }
}

// ---------------------------------------------------------------------------
// updateRobotPose
// ---------------------------------------------------------------------------

/**
 * 更新机器人位姿（支持单臂、双臂和人形模式）
 */
export function updateRobotPose() {
    if (appState.humanoidMode) {
        updateHumanoidPose();
        return;
    }
    if (appState.dualArmMode) {
        updateDualArmPose();
    } else {
        updateSingleArmPose();
    }
}

// 暴露到全局作用域
window.updateRobotPose = updateRobotPose;

// ---------------------------------------------------------------------------
// updateGlobalVariables
// ---------------------------------------------------------------------------

/**
 * 更新全局变量（当这些变量改变时）
 */
export function updateGlobalVariables() {
    // 先调用 state.js 里的 updateWindowGlobals，把 appState 同步到 window
    updateWindowGlobals();

    // 暴露 updateGlobalVariables 自身供 visualizer 调用
    window.updateGlobalVariables = updateGlobalVariables;
}

// ---------------------------------------------------------------------------
// updateSingleArmPose
// ---------------------------------------------------------------------------

/**
 * 更新单臂机器人位姿
 */
export function updateSingleArmPose() {
    try {
        // 确保使用最新的关节角度（如果 window.jointAngles 被修改，同步到本地）
        if (window.jointAngles && window.jointAngles.length === appState.jointAngles.length) {
            // 检查是否有差异，如果有则同步
            for (let i = 0; i < appState.jointAngles.length; i++) {
                if (Math.abs(appState.jointAngles[i] - window.jointAngles[i]) > 1e-8) {
                    appState.jointAngles = window.jointAngles;  // 使用 window.jointAngles 的引用
                    break;
                }
            }
        }

        // 计算正向运动学
        let pose = appState.currentRobot.getEndEffectorPose(appState.jointAngles);

        // 创建用于显示的位姿副本（可能需要坐标转换）
        let displayPose = pose;
        if (appState.currentRobotIsYUp && pose && pose.translation) {
            const x = pose.translation[1];
            const y = pose.translation[2];
            const z = pose.translation[3];
            displayPose = {
                translation: [pose.translation[0], x, z, y],
                rotation: pose.rotation
            };
        }

        // 更新位姿信息显示
        callbacks.updatePoseDisplay(displayPose);

        // 更新 DQ 显示
        callbacks.updateDQDisplay();

        // 更新雅可比矩阵显示
        callbacks.updateJacobianDisplay();

        // 更新 3D 可视化
        if (appState.visualizer) {
            const orderedNames = appState.customRobotConfig?.joint_chain?.joints || [];
            // 调试：打印 DQ 返回的原始坐标
            if (pose && pose.translation) {
                const [w, x, y, z] = pose.translation;
                console.log(`[DQ fkm] translation: [${w.toFixed(3)}, ${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}]`);
            }
            appState.visualizer.updateRobotPose(appState.jointAngles, pose, orderedNames);
        }

        // 同步关节滑块和输入框显示
        callbacks.updateJointSlidersFromAngles();
    } catch (error) {
        console.error('更新单臂位姿失败:', error);
    }
}

// ---------------------------------------------------------------------------
// updateHumanoidPose
// ---------------------------------------------------------------------------

/**
 * 更新人形机器人位姿（左臂=effector1，右臂=effector2；用于绝对/相对姿态显示）
 */
export function updateHumanoidPose() {
    try {
        if (!appState.humanoidModel || !appState.humanoidQ.length) return;
        callbacks.applyHumanoidBasePose();
        if (appState.visualizer) appState.visualizer.setHumanoidJoints(appState.humanoidQ);

        // 更新关节滑块显示
        callbacks.updateHumanoidJointSlidersFromAngles();

        const dqModule = getDQModule();
        if (!dqModule) return;
        const dq1 = appState.humanoidModel.fkmEffector(appState.humanoidQ, 1);
        const dq2 = appState.humanoidModel.fkmEffector(appState.humanoidQ, 2);
        // 注意：DQWrapper.pose 在当前 WASM 实现中等价于 R*T（平移在局部坐标）
        // 这里基座位姿需要世界坐标语义，统一使用 T*R，与规划运动模块保持一致。
        const baseX = appState.humanoidBasePose.x || 0;
        const baseY = appState.humanoidBasePose.y || 0;
        const baseYaw = appState.humanoidBasePose.yaw || 0;
        const baseTrans = dqModule.DQWrapper.translation(baseX, baseY, 0);
        const baseRot = dqModule.DQWrapper.rotation(baseYaw, 0, 0, 1);
        const baseObj = dqModule.DQWrapper.multiply(baseTrans, baseRot);
        const worldDq1 = dqModule.DQWrapper.toArray(dqModule.DQWrapper.multiply(baseObj, dqModule.DQWrapper.createFromArray(dq1)));
        const worldDq2 = dqModule.DQWrapper.toArray(dqModule.DQWrapper.multiply(baseObj, dqModule.DQWrapper.createFromArray(dq2)));
        const obj1 = dqModule.DQWrapper.createFromArray(worldDq1);
        const obj2 = dqModule.DQWrapper.createFromArray(worldDq2);
        const pose1 = { translation: dqModule.DQWrapper.getTranslation(obj1), rotation: dqModule.DQWrapper.getRotation(obj1) };
        const pose2 = { translation: dqModule.DQWrapper.getTranslation(obj2), rotation: dqModule.DQWrapper.getRotation(obj2) };

        // 使用 dualArmSystem（HumanoidDualArmAdapter）计算绝对位姿和相对位姿，保持与控制器一致
        let absPose, relPose;
        if (appState.dualArmSystem && typeof appState.dualArmSystem.getAbsolutePose === 'function') {
            const absDQLocal = appState.dualArmSystem.getAbsolutePose(appState.humanoidQ);
            const relDQArr = appState.dualArmSystem.getRelativePose(appState.humanoidQ);
            const absDQ = dqModule.DQWrapper.toArray(dqModule.DQWrapper.multiply(baseObj, dqModule.DQWrapper.createFromArray(absDQLocal)));
            const absObj = dqModule.DQWrapper.createFromArray(absDQ);
            const relObj = dqModule.DQWrapper.createFromArray(relDQArr);
            absPose = { translation: dqModule.DQWrapper.getTranslation(absObj), rotation: dqModule.DQWrapper.getRotation(absObj) };
            relPose = { translation: dqModule.DQWrapper.getTranslation(relObj), rotation: dqModule.DQWrapper.getRotation(relObj) };
        } else {
            // fallback: 简单计算
            absPose = {
                translation: [1, (pose1.translation[1] + pose2.translation[1]) / 2, (pose1.translation[2] + pose2.translation[2]) / 2, (pose1.translation[3] + pose2.translation[3]) / 2],
                rotation: [(pose1.rotation[0] + pose2.rotation[0]) / 2, (pose1.rotation[1] + pose2.rotation[1]) / 2, (pose1.rotation[2] + pose2.rotation[2]) / 2, (pose1.rotation[3] + pose2.rotation[3]) / 2]
            };
            const inv1 = dqModule.DQWrapper.inverse(obj1);
            const relDQ = dqModule.DQWrapper.multiply(obj2, inv1);
            relPose = { translation: dqModule.DQWrapper.getTranslation(relDQ), rotation: dqModule.DQWrapper.getRotation(relDQ) };
        }
        if (appState.visualizer) appState.visualizer.updateDualArmRelativeVisualizations(pose1, pose2, absPose, null);
        callbacks.updateDualArmPoseDisplay(pose1, pose2, absPose, relPose);
    } catch (error) {
        console.error('更新人形位姿失败:', error);
    }
}

// ---------------------------------------------------------------------------
// updateDualArmPose
// ---------------------------------------------------------------------------

/**
 * 更新双臂机器人位姿
 */
export function updateDualArmPose() {
    try {
        if (!appState.robot1 || !appState.robot2) {
            console.warn('双臂机器人未初始化');
            return;
        }

        // 计算两个机器人的位姿
        const pose1 = appState.robot1.getEndEffectorPose(appState.jointAngles1);
        const pose2 = appState.robot2.getEndEffectorPose(appState.jointAngles2);

        // 更新 3D 可视化
        if (appState.visualizer) {
            const jointNames1 = appState.customRobotConfig?.robot1?.joint_chain?.joints || [];
            const jointNames2 = appState.customRobotConfig?.robot2?.joint_chain?.joints || [];

            appState.visualizer.updateRobotPose(appState.jointAngles1, pose1, jointNames1, 0);
            appState.visualizer.updateRobotPose(appState.jointAngles2, pose2, jointNames2, 1);

            // 使用 absolute_pose 函数计算绝对位姿
            // 合并两个机器人的关节角度：[robot1_joints..., robot2_joints...]
            const allJointAngles = [...appState.jointAngles1, ...appState.jointAngles2];

            let absolutePose = null;

            if (appState.dualArmSystem && appState.dualArmSystem.getAbsoluteWorldPose) {
                // 使用absolute_pose函数计算
                const absPose = appState.dualArmSystem.getAbsoluteWorldPose(allJointAngles);

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
            appState.visualizer.updateDualArmRelativeVisualizations(pose1, pose2, absolutePose, null);
        }

        // 更新 UI 显示
        callbacks.updateDualArmPoseDisplay(pose1, pose2, null, null);

        // 同步双臂关节滑块和输入框显示
        callbacks.updateDualArmJointSlidersFromAngles();

    } catch (error) {
        console.error('更新双臂位姿失败:', error);
    }
}

// ---------------------------------------------------------------------------
// loadSpecificURDF
// ---------------------------------------------------------------------------

/**
 * 加载指定的URDF文件
 */
export async function loadSpecificURDF(urdfFile, urdfPath, config) {
    // 查找URDF文件
    let urdfFilePath = null;
    const allUrdfFiles = Array.from(appState.packageManager.files.keys()).filter(name =>
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
        console.log('  可用URDF文件:', Array.from(appState.packageManager.files.keys()).filter(n => n.endsWith('.urdf')));
        throw new Error(`未找到URDF文件: ${urdfFile}`);
    }

    console.log(`  ✓ 找到 URDF: ${urdfFilePath}`);

    // 创建包管理器（复用文件）
    const robotPackageManager = new URDFPackageManager();
    robotPackageManager.files = appState.packageManager.files;
    robotPackageManager.packages = appState.packageManager.packages;
    robotPackageManager.config = config;

    // 临时覆盖 findURDFFile 方法
    const originalFindURDFFile = robotPackageManager.findURDFFile;
    robotPackageManager.findURDFFile = () => urdfFilePath;

    const robotURDFObj = await loadURDFFromPackage(robotPackageManager);

    // 恢复原始方法
    robotPackageManager.findURDFFile = originalFindURDFFile;

    return robotURDFObj;
}

// ---------------------------------------------------------------------------
// loadDualArmSystem
// ---------------------------------------------------------------------------

/**
 * 加载双臂机器人系统
 */
export async function loadDualArmSystem(firstRobot, config) {
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

                appState.dualArmSystem = new CooperativeDualArm(mdhMatrix1, mdhMatrix2);

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
                        appState.dualArmSystem.setRobot1BaseDQ(robot1BasePose);
                        console.log('  ✓ 机器人1基座双四元数已应用');
                    } else if (robot1BasePose.length === 7) {
                        const [qw, qx, qy, qz, tx, ty, tz] = robot1BasePose;
                        console.log(`  设置机器人1基座位姿 (Z-up):`);
                        console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                        console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                        appState.dualArmSystem.setRobot1BasePose(qw, qx, qy, qz, tx, ty, tz);
                        console.log('  ✓ 机器人1基座完整位姿已应用');
                    } else if (robot1BasePose.length === 4) {
                        const [, tx, ty, tz] = robot1BasePose;
                        console.log(`  设置机器人1基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        appState.dualArmSystem.setRobot1Base(tx, ty, tz);
                        console.log('  ✓ 机器人1基座偏移已应用');
                    } else {
                        const [tx, ty, tz] = robot1BasePose;
                        console.log(`  设置机器人1基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        appState.dualArmSystem.setRobot1Base(tx, ty, tz);
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
                        appState.dualArmSystem.setRobot2BaseDQ(robot2BasePose);
                        console.log('  ✓ 机器人2基座双四元数已应用');
                    } else if (robot2BasePose.length === 7) {
                        const [qw, qx, qy, qz, tx, ty, tz] = robot2BasePose;
                        console.log(`  设置机器人2基座位姿 (Z-up):`);
                        console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                        console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                        appState.dualArmSystem.setRobot2BasePose(qw, qx, qy, qz, tx, ty, tz);
                        console.log('  ✓ 机器人2基座完整位姿已应用');
                    } else if (robot2BasePose.length === 4) {
                        const [, tx, ty, tz] = robot2BasePose;
                        console.log(`  设置机器人2基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        appState.dualArmSystem.setRobot2Base(tx, ty, tz);
                        console.log('  ✓ 机器人2基座偏移已应用');
                    } else {
                        const [tx, ty, tz] = robot2BasePose;
                        console.log(`  设置机器人2基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                        appState.dualArmSystem.setRobot2Base(tx, ty, tz);
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
                    appState.dualArmSystem.setRobot1EffectorPose(
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
                    appState.dualArmSystem.setRobot2EffectorPose(
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
            appState.dualArmSystem = null;
            // 即使创建失败，也要更新全局变量
            updateGlobalVariables();
        }

        // 加载到可视化器
        console.log('  → 加载到 3D 场景...');
        if (dualArmType === 'single_urdf') {
            // 单URDF格式：只传入一个URDF对象，但传递dualArmType标识
            await appState.visualizer.loadCustomURDF(robot1URDFObj, null, config, 'single_urdf');
        } else {
            // 双URDF格式：传入两个URDF对象
            await appState.visualizer.loadCustomURDF(robot1URDFObj, robot2URDFObj, config, 'two_urdf');
        }

        // 初始化关节角度
        appState.jointAngles1 = config.robot1?.initial_joint_angles || [];
        appState.jointAngles2 = robot2Config?.initial_joint_angles || [];

        // 限制初始角度在限位范围内
        const robot1JointNames = config.robot1?.joint_chain?.joints || [];
        const robot2JointNames = robot2Config?.joint_chain?.joints || [];

        if (robot1JointNames.length === appState.jointAngles1.length) {
            appState.jointAngles1 = callbacks.clampJointAngles(appState.jointAngles1, robot1JointNames, robot1URDFObj, config.robot1);
        }
        if (robot2JointNames.length === appState.jointAngles2.length) {
            appState.jointAngles2 = callbacks.clampJointAngles(appState.jointAngles2, robot2JointNames, robot2URDFObj, robot2Config);
        }

        console.log(`✓ 双臂系统加载完成`);
        console.log(`  机器人1: ${appState.jointAngles1.length} 个关节`);
        console.log(`  机器人2: ${appState.jointAngles2.length} 个关节`);

        // 保存URDF对象引用（用于更新关节角度）
        appState.robot1URDF = robot1URDFObj;
        appState.robot2URDF = robot2URDFObj;

        // 设置visualizer的URDF引用
        if (appState.visualizer) {
            appState.visualizer.robot1URDF = robot1URDFObj;
            appState.visualizer.robot2URDF = robot2URDFObj;

            // 初始化URDF坐标系显示（如果开关已打开）- 双臂模式同时初始化两个机器人
            if (appState.visualizer.showURDFFrames) {
                // 双臂模式：同时初始化两个机器人的坐标系
                const robot1Links = robot1URDFObj && robot1URDFObj.links ? Object.keys(robot1URDFObj.links) : [];
                const robot2Links = robot2URDFObj && robot2URDFObj.links ? Object.keys(robot2URDFObj.links) : [];
                appState.visualizer.initURDFFrames(null, null, robot1URDFObj, robot2URDFObj);
            }
        }

        // 设置双臂模式标志
        appState.dualArmMode = true;

        // 更新全局变量，确保所有双臂相关变量都被暴露
        updateGlobalVariables();

        // 更新双臂滑块显示（使用限制后的角度）
        // 延迟执行，确保关节控制器已创建
        setTimeout(() => {
            callbacks.updateDualArmJointSlidersFromAngles();
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

// ---------------------------------------------------------------------------
// extractMDHFromDualArmRobot
// ---------------------------------------------------------------------------

/**
 * 从双臂机器人提取 MDH（辅助函数）
 */
export async function extractMDHFromDualArmRobot(urdfRobot, robotConfig, robotIndex) {
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
            appState.robot1 = dqRobot;
        } else {
            appState.robot2 = dqRobot;
        }

        console.log(`✓ 机器人${robotIndex} MDH 提取完成`);

    } catch (error) {
        console.error(`提取机器人${robotIndex} MDH失败:`, error);
        throw error;
    }
}

// ---------------------------------------------------------------------------
// loadHumanoidSystem
// ---------------------------------------------------------------------------

/**
 * 加载人形机器人系统
 */
export async function loadHumanoidSystem(robot, config) {
    const linkNames = config.humanoid_link_names || {};
    const names = {
        base_link: linkNames.base_link || 'base_link',
        waist_start: linkNames.waist_start || linkNames.base_link || 'base_link',
        waist_end: linkNames.waist_end || 'body_link',
        left_arm_start: linkNames.left_arm_start || 'body_link',
        left_arm_end: linkNames.left_arm_end || 'lee_link',
        right_arm_start: linkNames.right_arm_start || 'body_link',
        right_arm_end: linkNames.right_arm_end || 'ree_link',
        head_start: linkNames.head_start || 'body_link',
        head_end: linkNames.head_end || 'head_pitch_link'
    };
    const chains = loadHumanoidChainsFromURDF(robot, names);
    if (!chains) {
        throw new Error('人形链提取失败，请检查 humanoid_link_names 与 URDF 中的 link 名是否一致');
    }
    const dqModule = getDQModule();
    if (!dqModule || !dqModule.HumanoidURDF) {
        throw new Error('WASM 未加载或未包含 HumanoidURDF，请重新构建 wasm');
    }
    const baseDQ = [1, 0, 0, 0, 0, 0, 0, 0];
    // 传递 effector frames 以处理尾随固定关节
    appState.humanoidModel = new dqModule.HumanoidURDF(
        baseDQ,
        chains.waistChain, chains.leftArmChain, chains.rightArmChain, chains.headChain,
        chains.waistEffector, chains.leftArmEffector, chains.rightArmEffector, chains.headEffector
    );
    appState.humanoidJointNames = [...chains.waistJointNames, ...chains.leftArmJointNames, ...chains.rightArmJointNames, ...chains.headJointNames];
    appState.humanoidQ = new Array(appState.humanoidJointNames.length).fill(0);
    if (config.initial_q && Array.isArray(config.initial_q) && config.initial_q.length === appState.humanoidQ.length) {
        appState.humanoidQ = config.initial_q.slice();
    }
    appState.humanoidMode = true;
    appState.dualArmMode = false;
    appState.dualArmSystem = new HumanoidDualArmAdapter(appState.humanoidModel);
    appState.humanoidBasePose = callbacks.getHumanoidBasePoseFromConfig(config);
    appState.humanoidBaseTargetPose = { ...appState.humanoidBasePose };
    appState.humanoidBaseWheelVelocity = { left: 0, right: 0 };
    appState.humanoidBaseConvergedCount = 0;
    appState.humanoidBaseControlParams = callbacks.getHumanoidDiffDriveParamsFromConfig(config);
    try {
        appState.humanoidDiffDriveModel = new DifferentialDriveRobot(
            appState.humanoidBaseControlParams.wheelRadius,
            appState.humanoidBaseControlParams.wheelDistance
        );
    } catch (error) {
        appState.humanoidDiffDriveModel = null;
        console.warn('未能初始化 DQ DifferentialDriveRobot，底盘将回退到解析积分:', error);
    }
    callbacks.startHumanoidBaseControlLoop();
    if (appState.visualizer) {
        const waistLen = chains.waistJointNames.length;
        const leftLen = chains.leftArmJointNames.length;
        const rightLen = chains.rightArmJointNames.length;
        const headLen = chains.headJointNames.length;
        await appState.visualizer.loadHumanoidURDF(robot, config, {
            model: appState.humanoidModel,
            jointNames: appState.humanoidJointNames,
            waistLength: waistLen,
            leftArmLength: leftLen,
            rightArmLength: rightLen,
            headLength: headLen,
            effectorLinkNames: [names.waist_end, names.left_arm_end, names.right_arm_end, names.head_end]
        });
        callbacks.applyHumanoidBasePose();
        appState.visualizer.setHumanoidJoints(appState.humanoidQ);
    }
    callbacks.createHumanoidJointControls(config);
    updateGlobalVariables();
}

// ---------------------------------------------------------------------------
// extractMDHFromCustomRobot
// ---------------------------------------------------------------------------

/**
 * 从自定义机器人提取 MDH
 */
export async function extractMDHFromCustomRobot(robot, config) {
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
            appState.extractedMDH = config.mdh_extraction.manual_mdh;

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
                appState.extractedMDH = result.mdh;
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
                if (result.allMDHFrames && appState.visualizer) {
                    appState.visualizer.setMDHFrames(result.allMDHFrames);
                    console.log('  ✓ 已设置MDH坐标系可视化');

                    // 同步开关状态：如果开关是打开的，确保坐标系显示
                    const mdhFramesToggle = document.getElementById('show-mdh-frames-toggle');
                    if (mdhFramesToggle && mdhFramesToggle.checked) {
                        appState.visualizer.showMDHFrames = true;
                        if (appState.visualizer.mdhFramesGroup && !appState.visualizer.scene.getObjectByName('MDHFrames')) {
                            appState.visualizer.scene.add(appState.visualizer.mdhFramesGroup);
                        }
                    }
                }
            } else {
                appState.extractedMDH = result; // 向后兼容
            }

            // 将effectorPose保存到config中，供后续使用
            if (extractedEffectorPose) {
                config._extractedEffectorPose = extractedEffectorPose;
            }
        }

        console.log('  MDH 参数 (提取结果):');
        console.log('  关节 |    θ     |    d     |    a     |    α    ');
        console.log('  -----|----------|----------|----------|----------');
        appState.extractedMDH.forEach((params, i) => {
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
            appState.extractedMDH.forEach((params, i) => {
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
        const numJoints = appState.extractedMDH.length;
        console.log(`  关节数: ${numJoints}`);

        // 转置：从 [[θ,d,a,α], ...] 到 [[θ...], [d...], [a...], [α...], [type...]]
        const theta_row = appState.extractedMDH.map(p => p[0]);
        const d_row = appState.extractedMDH.map(p => p[1]);
        const a_row = appState.extractedMDH.map(p => p[2]);
        const alpha_row = appState.extractedMDH.map(p => p[3]);
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
        appState.urdfBasedRobot = new SerialManipulator(mdhMatrix);
        console.log('  ✓ SerialManipulator 创建成功（Z-up 坐标系）');

        // 设置末端执行器偏移
        console.log('\n  → 检查末端执行器偏移配置...');
        try {
            const endLinkName = config.joint_chain?.end_link;
            let effPose = null;

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
                appState.urdfBasedRobot.setEffectorPose(
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
                if (typeof appState.urdfBasedRobot.setBaseFrameDQ === 'function') {
                    appState.urdfBasedRobot.setBaseFrameDQ(basePose);
                    console.log('  ✓ 基座双四元数已应用');
                } else {
                    console.warn('  ⚠ setBaseFrameDQ 方法不可用，回退到其他格式');
                }
            } else if (basePose.length === 7) {
                // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                const [qw, qx, qy, qz, tx, ty, tz] = basePose;
                console.log(`\n  → 设置基座位姿 (Z-up):`);
                console.log(`    平移: (${tx}, ${ty}, ${tz})`);
                console.log(`    四元数: (${qw}, ${qx}, ${qy}, ${qz})`);
                if (typeof appState.urdfBasedRobot.setBaseFramePose === 'function') {
                    appState.urdfBasedRobot.setBaseFramePose(qw, qx, qy, qz, tx, ty, tz);
                    console.log('  ✓ 基座完整位姿已应用');
                } else {
                    console.warn('  ⚠ setBaseFramePose 方法不可用，仅使用平移部分');
                    appState.urdfBasedRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                    console.log('  ✓ 基座平移已应用（旋转部分未应用）');
                }
            } else if (basePose.length === 4) {
                // 可能是 [qw, tx, ty, tz] 格式，取后3个
                const [, tx, ty, tz] = basePose;
                console.log(`\n  → 设置基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                appState.urdfBasedRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                console.log('  ✓ 基座偏移已应用');
            } else {
                // 仅平移 [tx, ty, tz]
                const [tx, ty, tz] = basePose;
                console.log(`\n  → 设置基座偏移 (Z-up): (${tx}, ${ty}, ${tz})`);
                appState.urdfBasedRobot.setBaseFrame(tx || 0, ty || 0, tz || 0);
                console.log('  ✓ 基座偏移已应用');
            }
        } else {
            // 向后兼容：检查 base_offset（旧格式）
            if (config.mdh_extraction?.base_offset) {
                const offset = config.mdh_extraction.base_offset;
                console.log(`\n  → 应用基座偏移 (旧格式base_offset): (${offset[0]}, ${offset[1]}, ${offset[2]})`);
                appState.urdfBasedRobot.setBaseFrame(offset[0], offset[1], offset[2]);
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
                const urdfPose = appState.urdfBasedRobot.getEndEffectorPose(zeroAngles);
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

        appState.currentRobot = appState.urdfBasedRobot;
        appState.currentRobotIsYUp = false; // 暂时设为false，待调试

        // 更新关节控制器
        appState.jointAngles = config.initial_joint_angles || new Array(jointNames.length).fill(0);

        // 限制初始角度在限位范围内
        if (jointNames && jointNames.length === appState.jointAngles.length) {
            const urdfRobot = appState.visualizer?.urdfRobot || null;
            appState.jointAngles = callbacks.clampJointAngles(appState.jointAngles, jointNames, urdfRobot, config);
        }

        // 创建关节控制器（传入限制后的角度）
        callbacks.createJointControls(jointNames.length, config);

        // 更新滑块显示（使用限制后的角度）
        callbacks.updateJointSlidersFromAngles();

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

// ---------------------------------------------------------------------------
// initRobot
// ---------------------------------------------------------------------------

/**
 * 初始化机器人（内置 Franka/Kuka）
 */
export async function initRobot(robotType) {
    appState.currentRobotType = robotType;
    let frankaConfig = null;

    if (robotType === 'franka') {
        appState.currentRobot = new FrankaPanda();
        appState.currentRobotIsYUp = false; // 内置 Franka 为 Z-up
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
        appState.currentRobot = new KukaLw4();
        appState.currentRobotIsYUp = false;
    }
    appState.frankaConfigCache = robotType === 'franka' ? frankaConfig : null;

    const numJoints = appState.currentRobot.getDimConfigurationSpace();

    // 初始化关节角度为零位或从配置文件读取
    if (frankaConfig && frankaConfig.initial_joint_angles) {
        appState.jointAngles = [...frankaConfig.initial_joint_angles];
        // 限制初始角度在限位范围内
        const jointNames = frankaConfig.joint_chain?.joints || [];
        if (jointNames.length === appState.jointAngles.length) {
            appState.jointAngles = callbacks.clampJointAngles(appState.jointAngles, jointNames, null, frankaConfig);
        }
    } else {
        appState.jointAngles = new Array(numJoints).fill(0);
    }
    updateGlobalVariables();

    // 创建关节控制滑块（传入配置以读取限位）
    callbacks.createJointControls(numJoints, frankaConfig);

    // 更新滑块显示（使用限制后的角度）
    callbacks.updateJointSlidersFromAngles();
}

// ---------------------------------------------------------------------------
// initDefaultIARobot
// ---------------------------------------------------------------------------

/**
 * 默认机器人初始化
 */
export async function initDefaultIARobot() {
    // 如果用户已经通过拖拽加载/或已经有机器人，则不覆盖
    if (appState.customRobotConfig || appState.dualArmMode || appState.humanoidMode || appState.currentRobot) {
        return;
    }

    try {
        await loadBuiltInRobotFromPublic('sailor_r1_pro_description');
        updateRobotPose();
    } catch (error) {
        console.warn('⚠ 默认加载人形机器人失败，将回退到内置 Franka:', error);
        await initRobot('franka');
        updateRobotPose();
    }
}

// ---------------------------------------------------------------------------
// loadBuiltInRobotFromPublic
// ---------------------------------------------------------------------------

/**
 * 从 public/robots/<robotDir>/ 自动加载内置 URDF 机器人（复用拖拽加载逻辑）
 * 支持 ia_robot（双臂）和 sailor_r1_pro_description（人形）
 */
export async function loadBuiltInRobotFromPublic(robotDir) {
    if (!appState.visualizer) {
        throw new Error('可视化器未初始化');
    }

    // 在加载新模型前，先清理所有旧模型
    if (appState.visualizer && typeof appState.visualizer.cleanupOldRobots === 'function') {
        appState.visualizer.cleanupOldRobots();
    }

    const baseUrl = import.meta.env.BASE_URL || '/';

    let filesInPackage = [];

    if (robotDir === 'sailor_r1_pro_description') {
        // 人形机器人：sailor_r1_pro
        filesInPackage = [
            'config/robot_config.yaml',
            'urdf/sailor_r1_pro_description.urdf',
            // meshes
            'meshes/ankle_link.STL',
            'meshes/base_link.STL',
            'meshes/body_link.STL',
            'meshes/car_link.STL',
            'meshes/head_pitch_link.STL',
            'meshes/head_yaw.STL',
            'meshes/head_yaw_link.STL',
            'meshes/hip_pitch_link.STL',
            'meshes/knee_link.STL',
            'meshes/left_wheel_link.STL',
            'meshes/lidar_link.STL',
            'meshes/llink1.STL',
            'meshes/llink2.STL',
            'meshes/llink3.STL',
            'meshes/llink4.STL',
            'meshes/llink5.STL',
            'meshes/llink6.STL',
            'meshes/llink7.STL',
            'meshes/right_wheel_link.STL',
            'meshes/rlink1.STL',
            'meshes/rlink2.STL',
            'meshes/rlink3.STL',
            'meshes/rlink4.STL',
            'meshes/rlink5.STL',
            'meshes/rlink6.STL',
            'meshes/rlink7.STL'
        ];
    } else if (robotDir === 'ia_robot') {
        // ia_robot 是双臂包
        filesInPackage = [
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
    } else {
        throw new Error(`暂不支持的内置机器人: ${robotDir}`);
    }

    appState.packageManager = new URDFPackageManager();
    const remoteEntries = filesInPackage.map((rel) => {
        const path = `${robotDir}/${rel}`;
        const url = `${baseUrl}robots/${robotDir}/${rel}`;
        return { path, url };
    });
    await appState.packageManager.addRemoteFiles(remoteEntries);

    // 读取配置
    appState.customRobotConfig = appState.packageManager.getConfig();
    if (!appState.customRobotConfig) {
        throw new Error('未找到 robot_config.yaml 配置文件');
    }

    appState.currentRobotType = robotDir;

    // 为了复用现有流程，仍然 load 一次（dualArmSystem 内部会按 config 再次加载指定 URDF）
    const robot = await loadURDFFromPackage(appState.packageManager);

    // 检查是否为人形机器人
    const isHumanoid = appState.customRobotConfig?.humanoid === true;
    if (isHumanoid) {
        await loadHumanoidSystem(robot, appState.customRobotConfig);
        callbacks.switchToHumanoidUI();
        callbacks.refreshDockState();
        return;
    }

    const isDualArm = appState.customRobotConfig?.dual_arm === true;
    appState.dualArmMode = isDualArm;

    if (isDualArm) {
        const jointControlsContainer = document.getElementById('joint-controls');
        if (jointControlsContainer) {
            jointControlsContainer.innerHTML = '';
        }

        await loadDualArmSystem(robot, appState.customRobotConfig);
        callbacks.switchToDualArmUI();
        callbacks.createDualArmJointControls(appState.customRobotConfig);
    } else {
        const loaded = await appState.visualizer.loadCustomURDF(robot, null, appState.customRobotConfig);
        if (!loaded) {
            throw new Error('可视化器加载失败');
        }
        callbacks.switchToSingleArmUI();

        if (appState.customRobotConfig.mdh_extraction?.auto_extract) {
            await extractMDHFromCustomRobot(robot, appState.customRobotConfig);
        }
    }

    callbacks.refreshDockState();
}

// ---------------------------------------------------------------------------
// registerRobotManagerCallbacks
// ---------------------------------------------------------------------------

/**
 * 将本模块的公开函数注册到 callbacks，供其他模块调用
 */
export function registerRobotManagerCallbacks() {
    callbacks.updateRobotPose = updateRobotPose;
    callbacks.updateGlobalVariables = updateGlobalVariables;
    callbacks.initRobot = initRobot;
    callbacks.loadSpecificURDF = loadSpecificURDF;
    callbacks.loadDualArmSystem = loadDualArmSystem;
    callbacks.loadHumanoidSystem = loadHumanoidSystem;
    callbacks.extractMDHFromCustomRobot = extractMDHFromCustomRobot;
    callbacks.loadBuiltInRobotFromPublic = loadBuiltInRobotFromPublic;
}
