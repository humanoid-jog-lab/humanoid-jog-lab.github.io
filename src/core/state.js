/**
 * 集中状态管理
 * 
 * 所有模块通过 import { appState } from '../core/state.js' 来访问和修改共享状态。
 * 跨模块的函数调用通过 callbacks 对象注册，避免循环依赖。
 */

import { getDQModule } from '../dqrobotics.js';

/**
 * 应用全局状态
 */
export const appState = {
    // 当前机器人
    currentRobot: null,
    currentRobotType: 'ia_robot',
    visualizer: null,
    jointAngles: [],
    extractedMDH: null,
    urdfBasedRobot: null,
    packageManager: null,
    customRobotConfig: null,
    currentRobotIsYUp: false,
    frankaConfigCache: null,

    // 角度显示模式
    angleDisplayMode: 'rad',

    // UI 元素引用
    dropOverlayElement: null,
    dropOverlayHideTimer: null,
    dockElement: null,
    panelStackIndex: 30,

    // 双臂机器人状态
    dualArmMode: false,
    robot1: null,
    robot2: null,
    robot1URDF: null,
    robot2URDF: null,
    jointAngles1: [],
    jointAngles2: [],
    dualArmSystem: null,

    // 人形机器人状态
    humanoidMode: false,
    humanoidModel: null,
    humanoidQ: [],
    humanoidJointNames: [],
    humanoidInitialQProfileId: 'default1',
    humanoidCooperativePolishingSurfaceSource: 'analytic',
    humanoidCooperativePolishingObjectPose: null,
    humanoidBasePose: { x: 0, y: 0, yaw: 0 },
    humanoidBaseTargetPose: { x: 0, y: 0, yaw: 0 },
    humanoidBaseWheelVelocity: { left: 0, right: 0 },
    humanoidBaseControlTimer: null,
    humanoidBaseControlLastTs: 0,
    humanoidBaseConvergedCount: 0,
    humanoidBaseReverseDrive: false,
    humanoidDiffDriveModel: null,
    humanoidBaseControlParams: {
        wheelRadius: 0.10,
        wheelDistance: 0.425,
        kRho: 1.2,
        kAlpha: 3.2,
        kBeta: -0.9,
        kYawFinal: 3.0,
        maxLinear: 0.8,
        maxAngular: 2.0,
        maxWheelSpeed: 18.0,
        minWheelSpeed: 0.35,
        tolPos: 0.03,
        tolYaw: 0.05,
        settleCycles: 8
    },

    // Regulated Pure Pursuit 路径跟踪
    humanoidBaseControlMode: 'point',
    humanoidBasePath: [],
    humanoidBaseRPPParams: {
        lookahead_dist: 0.4,
        use_velocity_scaled_lookahead: true,
        lookahead_time: 1.5,
        min_lookahead_dist: 0.3,
        max_lookahead_dist: 0.9,
        max_linear_vel: 0.5,
        min_linear_vel: 0.05,
        max_angular_vel: 1.8,
        max_angular_accel: 3.2,
        use_regulated_linear_velocity_scaling: true,
        regulated_linear_scaling_min_radius: 0.9,
        regulated_linear_scaling_min_speed: 0.25,
        use_rotate_to_heading: true,
        rotate_to_heading_min_angle: 0.785,
        rotate_to_heading_angular_vel: 1.8,
        goal_dist_tol: 0.10,
        approach_velocity_scaling_dist: 0.6,
        min_approach_linear_velocity: 0.05,
        allow_reversing: false,
    },
    humanoidBaseRPPState: {
        isRotatingToHeading: false,
        hasReachedGoalXY: false,
        lastCmdLinear: 0,
        lastCmdAngular: 0,
        isFinished: false,
        isRunning: false,
        carrotWorld: null,
        progressSegIdx: 0,
    },

    // 环境物体
    environmentObjects: [],
    environmentObjectCounter: 0,

    // 场景预设
    currentScenePreset: null,  // 当前预设 ID ('warehouse' 等)

    // 关节位置发布定时器
    jointPositionPublishTimer: null,
    waistPositionPublishTimer: null,

    // ROS 2 通讯
    ros2Communicator: null,
    ros2MessageLog: [],
    ros2JointStateSubscribed: false,
    ros2JointStateTopic: '/whole/joint_states',
    ros2JointStateButton: null,
    ros2JointStatusElement: null,
    ros2BatteryStateSubscribed: false,
};

/**
 * 跨模块回调注册表
 * 模块在初始化时将自己的函数注册到这里，其他模块通过 callbacks.xxx() 调用
 * 避免循环依赖
 */
export const callbacks = {
    // robot-manager.js
    updateRobotPose: null,
    updateGlobalVariables: null,
    initRobot: null,
    loadSpecificURDF: null,
    loadDualArmSystem: null,
    loadHumanoidSystem: null,
    extractMDHFromCustomRobot: null,
    loadBuiltInRobotFromPublic: null,
    computeEffectorPoseFromURDF: null,

    // joint-controls.js
    createJointControls: null,
    updateJointSlidersFromAngles: null,
    updateDualArmJointSlidersFromAngles: null,
    updateHumanoidJointSlidersFromAngles: null,
    refreshJointControlDisplay: null,
    clampJointAngles: null,
    getJointLimits: null,
    createDualArmJointControls: null,
    createHumanoidJointControls: null,

    // display.js
    updatePoseDisplay: null,
    updateDualArmPoseDisplay: null,

    // environment-panel.js
    renderEnvironmentObjectList: null,
    addEnvironmentBoxToScene: null,
    loadScenePreset: null,
    clearScenePreset: null,

    // mode-switcher.js
    switchToSingleArmUI: null,
    switchToDualArmUI: null,
    switchToHumanoidUI: null,

    // panel-manager.js
    setupFloatingPanels: null,
    refreshDockState: null,
    showPanel: null,
    hidePanel: null,

    // ik-controller.js
    updateIKRobotSelection: null,
    setupInverseKinematicsControls: null,

    // dq-calculator.js
    updateDQCalculatorRobotSelection: null,
    setupDQCalculatorPanel: null,

    // ros2-panel.js
    addROS2LogEntry: null,
    initROS2Communication: null,
    updateROS2StatusDisplay: null,
    updateROS2ControlModeDisplay: null,
    startJointPositionPublishing: null,
    stopJointPositionPublishing: null,
    setupROS2EventListeners: null,

    // humanoid/base-control.js
    stopHumanoidBaseControlLoop: null,
    startHumanoidBaseControlLoop: null,
    applyHumanoidBasePose: null,
    getHumanoidDiffDriveParamsFromConfig: null,
    getHumanoidBasePoseFromConfig: null,

    // event-handlers.js
    setupEventListeners: null,

    // joint-clipboard.js
    importJointAnglesFromText: null,
    copyJointAnglesToClipboard: null,
    copyJointLimitsToClipboard: null,
};

/**
 * 将关键状态同步到 window 对象
 * 供 visualizer.js 等通过 window.* 访问的模块使用
 */
export function updateWindowGlobals() {
    window.currentRobot = appState.currentRobot;
    window.jointAngles = appState.jointAngles;
    window.customRobotConfig = appState.customRobotConfig;
    window.DQModule = getDQModule();
    window.updateGlobalVariables = callbacks.updateGlobalVariables;
    window.clampJointAngles = callbacks.clampJointAngles;
    window.updateRobotPose = callbacks.updateRobotPose;

    // 双臂相关
    window.dualArmSystem = appState.dualArmSystem;
    window.jointAngles1 = appState.jointAngles1;
    window.jointAngles2 = appState.jointAngles2;
    window.dualArmMode = appState.dualArmMode;
    window.robot1 = appState.robot1;
    window.robot2 = appState.robot2;
    window.robot1URDF = appState.robot1URDF;
    window.robot2URDF = appState.robot2URDF;

    // 人形相关
    window.humanoidMode = appState.humanoidMode;
    window.humanoidModel = appState.humanoidModel;
    window.humanoidQ = appState.humanoidQ;
    window.humanoidJointNames = appState.humanoidJointNames;
    window.humanoidInitialQProfileId = appState.humanoidInitialQProfileId;
    window.humanoidCooperativeRuledSurfaceSource = appState.humanoidCooperativePolishingSurfaceSource;
    window.humanoidCooperativeRuledSurfaceObjectPose = appState.humanoidCooperativePolishingObjectPose;
    window.humanoidBasePose = appState.humanoidBasePose;
    window.humanoidBaseTargetPose = appState.humanoidBaseTargetPose;
    window.humanoidBaseWheelVelocity = appState.humanoidBaseWheelVelocity;
}

if (typeof window !== 'undefined') {
    window.currentDualArmControlMode = window.currentDualArmControlMode || 'absolute';
}
