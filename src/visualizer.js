/**
 * 机器人 3D 可视化器 - 使用 Three.js 和 URDF（Z-up 坐标系）
 */

import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import {
    appendEqualityConstraints,
    appendVFIErrorTubeConstraints,
    appendComponentBounds,
    appendJointLimitStepConstraints,
    buildQPNumericSolveOptions,
    projectOntoHalfspaces,
    solveQPNumericActiveSet
} from './core/vfi-utils.js';
import { minimizeLBFGS } from './core/lbfgs-utils.js';
import { buildBodyVerticalLineTask } from './core/body-vertical-utils.js';
import {
    evaluateHumanoidJogCommandOutcome,
    resolveHumanoidJogCommandResult,
    shouldRejectHumanoidJogForMismatch
} from './core/humanoid-jog-outcome.js';
import { evaluateTranslationAxisGuard } from './core/humanoid-jog-axis-guard.js';
import {
    buildCurrentCCTSMPayload,
    buildPriorityCurrentCCTSMPayload
} from './core/humanoid-jog-diagnostics.js';
import { buildHumanoidJogManifoldDiagnostics } from './core/humanoid-jog-manifold-diagnostics.js';
import { buildHumanoidJogMJDiagnostics } from './core/humanoid-jog-mj-diagnostics.js';
import { evaluateHumanoidJogManifoldAcceptance } from './core/humanoid-jog-manifold-acceptance.js';
import { resolveHumanoidJogManifoldArmOnlyFallback } from './core/humanoid-jog-manifold-fallback.js';
import {
    normalizeHumanoidJogManifoldJogMetric,
    resolveHumanoidJogManifoldJogMetric,
    resolveHumanoidJogManifoldJogMetricFallback
} from './core/humanoid-jog-manifold-jog-metric.js';
import {
    getHumanoidJogSolverDisplayName,
    getHumanoidJogSolverReadyMessage,
    normalizeHumanoidJogSolverMode
} from './core/humanoid-jog-solver-presentation.js';
import { computeWaistXVFIRows } from './core/waist-x-vfi.js';
import { enforceWVFILowerBound, selectWVFILowerBound } from './core/wvfi-guard.js';
import {
    resolveHumanoidJogVFIDefaultSecondaryMetric,
    shouldProtectCommandRowsForHumanoidVFISecondary
} from './core/humanoid-jog-vfi-secondary-metric.js';
import {
    buildEqualityToleranceCandidates,
    shouldRetryWithoutSecondaryObjective
} from './core/vfi-step-secondary-guard.js';
import { generateBranchSeeds, projectToManifold, rolloutBranch, selectBranch } from './core/predictive-cert.js';
import {
    buildManipulabilityVFIConstraints,
    computeProtectedSecondaryGradient,
    summarizeManipulabilityVFIConstraints
} from './core/vfi-secondary-objective.js';
import {
    buildHumanoidJogActiveLimitRetreatBounds,
    buildHumanoidJogActiveLimitRetreatGradient
} from './core/humanoid-jog-active-limit-bias.js';
import { buildHumanoidJogManifoldProblem } from './core/humanoid-jog-manifold-problem.js';
import { buildHumanoidJogMJProblem } from './core/humanoid-jog-mj-problem.js';
import { buildHumanoidJogMJDistanceProblem } from './core/humanoid-jog-mj-distance-problem.js';
import {
    attemptHumanoidJogMJRestoreWasm,
    attemptHumanoidJogMJRestoreWasmWorker,
    buildHumanoidJogMJRestoreWasmEnvelope
} from './core/humanoid-jog-mj-wasm.js';
import { resolveHumanoidJogMJRestoreProfile } from './core/humanoid-jog-mj-profile.js';
import { buildHumanoidJogMJRegularizationWeights } from './core/humanoid-jog-mj-regularization.js';
import { supportsHumanoidJogMJRestoreMode } from './core/humanoid-jog-mj-mode-support.js';
import { HumanoidJogMJWorkerClient } from './core/humanoid-jog-mj-worker-client.js';
import { restoreHumanoidJogManifold } from './core/humanoid-jog-manifold-restoration.js';
import { restoreHumanoidJogMJ } from './core/humanoid-jog-mj-restoration.js';
import {
    getHumanoidJogMJPreWDirection,
    mapHumanoidJogMJPreWOutcome,
    resolveHumanoidJogMJPreWMetric
} from './core/humanoid-jog-mj-pre-w.js';
import {
    shouldAttemptHumanoidJogMJPolish,
    shouldApplyHumanoidJogMJPolish
} from './core/humanoid-jog-mj-polish.js';
import { computeHumanoidJogManifoldNullspaceStep } from './core/humanoid-jog-manifold-nullspace.js';
import {
    buildHumanoidJogMJWorkerSecondaryMetricRequest,
    computeHumanoidJogMJNullspaceRefine,
    shouldComputeHumanoidJogMJInitialManipData,
    shouldComputeHumanoidJogMJTrialManipData,
    shouldEnableHumanoidJogMJNullspaceRefine,
    shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt,
    shouldUseHumanoidJogMJProjectionOnlyMetricStep
} from './core/humanoid-jog-mj-nullspace.js';
import { resolveHumanoidJogMJMetricTolerances } from './core/humanoid-jog-mj-metric-tolerances.js';
import { buildHumanoidJogManifoldStepBounds } from './core/humanoid-jog-manifold-profile.js';
import {
    evaluateHumanoidJogManifoldObjectiveDirection,
    evaluateHumanoidJogManifoldNonDegradation
} from './core/humanoid-jog-manifold-objective-direction.js';
import {
    buildHumanoidJogDirectionalAxisVector,
    evaluateHumanoidJogAbsoluteManipulability,
    evaluateHumanoidJogAbsoluteArmManipulability,
    evaluateHumanoidJogAbsoluteArmVFIManipulability,
    evaluateHumanoidJogAbsoluteHierarchicalManipulability,
    evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability,
    evaluateHumanoidJogAbsoluteVFIManipulability,
    evaluateHumanoidJogDirectionalManipulability,
    evaluateHumanoidJogProjectedManipulability
} from './core/humanoid-jog-directional-manipulability.js';
import {
    computeTwistJacobian,
    computeAngularJacobian,
    nullSpaceBasis,
    computeCCTSM,
    computeVFIAttenuationMatrix,
    computeVFICCTSM,
    computeCombinedCCTSM,
    computeCombinedVFICCTSM,
    directionalCCTSM,
    minEigenvalueCCTSM
} from './core/cctsm.js';

/** Local finite-value helper (used by predictive cert and CCTSM wiring) */
function asFinite(v, fb = 0) { return Number.isFinite(v) ? v : fb; }

export class RobotVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.robot = null;
        this.urdfRobot = null;
        this.endEffector = null;
        this.useURDF = true;
        
        // 双臂支持
        this.dualArmMode = false;
        this.robot1 = null;
        this.robot1URDF = null;
        this.robot1EndEffector = null;
        this.robot2 = null;
        this.robot2URDF = null;
        this.robot2EndEffector = null;
        this.absolutePoseMarker = null;  // 绝对位姿标记
        this.relativePoseLine = null;    // 相对位姿连线
        // 人形支持（左臂/右臂姿态控制；IK 仅动左臂+右臂，腰和头固定）
        this.humanoidMode = false;
        this.humanoidURDF = null;
        this.humanoidModel = null;
        this.humanoidJointNames = [];
        this.humanoidWaistLength = 0;
        this.humanoidLeftArmLength = 0;
        this.humanoidRightArmLength = 0;
        this.humanoidHeadLength = 0;
        this.humanoidEffectorLinkNames = [];  // [waist_end, left_arm_end, right_arm_end, head_end] 用于球体取 URDF link 世界坐标
        this.humanoidMJWorkerModelSpec = null;
        this.humanoidMJWorkerClient = null;
        this.humanoidMJWorkerScriptVersion = `${Date.now()}`;
        this._humanoidMJWorkerRequestToken = 0;
        this._humanoidMJWorkerLoopActive = false;
        this._humanoidMJWorkerPendingRequest = null;
        this.humanoidEffectorSpheres = null;  // Group: 4 球体（腰/左臂/右臂/头末端）
        this.humanoidBaseHeight = 0; // 人形底盘高度（z 方向保持固定）
        this.humanoidAbsoluteMarker = null;   // 绝对位姿球+坐标轴（左右臂中点）
        this.humanoidRelativeLine = null;     // 左臂到右臂连线
        this.humanoidLeftArmEndEffector = null;   // 左臂末端球+坐标轴
        this.humanoidRightArmEndEffector = null;  // 右臂末端球+坐标轴
        this.humanoidWaistEndEffector = null;     // 腰部末端坐标轴（DQ计算）
        this.humanoidHeadEndEffector = null;      // 头部末端坐标轴（DQ计算）
        this.lastHumanoidJogAxis = null;
        
        // 坐标系指示器
        this.axesScene = null;
        this.axesCamera = null;
        
        // MDH坐标系显示
        this.mdhFramesGroup = null;
        this.showMDHFrames = false;
        this.lastMDHFrameOrigin = null; // 最后一个MDH坐标系的原点位置
        
        // URDF坐标系显示
        this.urdfFramesGroup = null;
        this.showURDFFrames = false;
        this.urdfFrameObjects = new Map(); // robotPrefix_linkName -> frameObject (例如 "robot1_link1" 或 "link1")
        this.urdfFrameLinkNames = []; // 存储要显示的link名称列表（单臂模式）
        this.urdfFrameRobot1Links = []; // 存储机器人1的link名称列表（双臂模式）
        this.urdfFrameRobot2Links = []; // 存储机器人2的link名称列表（双臂模式）
        
        // 鼠标控制
        this.isLeftDragging = false;  // 左键拖动标志（旋转视角）
        this.isRightDragging = false; // 右键拖动标志（平移位置）
        this.previousMousePosition = { x: 0, y: 0 };
        this.cameraRotation = { 
            x: 0.9680,    // 俯仰角（弧度）
            y: 0.0100     // 方位角（弧度）
        };
        this.cameraDistance = 2.4085;  // 相机距离
        this.cameraPan = { x: -0.0001, y: -0.0020, z: 0.2679 }; // 相机平移偏移
        this.cameraViewMode = 'orbit'; // orbit | top-down
        
        // 拖动末端执行器模式
        this.dragEndEffectorMode = false;  // 拖动模式开关
        this.isDraggingEndEffector = false;  // 是否正在拖动末端执行器
        this.raycaster = new THREE.Raycaster();  // 用于鼠标拾取
        this.mouse = new THREE.Vector2();  // 鼠标位置
        this.dragPlane = null;  // 拖动平面
        this.dragStartPosition = null;  // 拖动起始位置
        this.dragCurrentPosition = null;  // 拖动当前位置
        
        // MoveIt 风格的拖动箭头
        this.endEffectorGizmo = null;  // 拖动控制组（包含平移和旋转箭头）

        // 环境物体管理
        this.environmentGroup = null;
        this.environmentObjects = new Map();

        // 场景预设管理
        this.scenePresetGroup = null;  // 当前加载的场景预设 Group
        this.currentScenePreset = null; // 当前预设 ID（如 'warehouse'）

        // 预分配临时向量，减少频繁创建
        this._tempVec3 = new THREE.Vector3();
        this._tempQuat = new THREE.Quaternion();
        this._tempScale = new THREE.Vector3();
        this._tempEuler = new THREE.Euler();
        this.translationArrows = { 
            x: { positive: null, negative: null }, 
            y: { positive: null, negative: null }, 
            z: { positive: null, negative: null } 
        };  // 平移箭头（正负方向）
        this.rotationArrows = { x: null, y: null, z: null };  // 旋转箭头
        this.selectedArrow = null;  // 当前选中的箭头
        this.arrowType = null;  // 'translation' 或 'rotation'
        this.arrowAxis = null;  // 'x', 'y', 或 'z'

        // 示教器乘法顺序（默认局部增量：Δ × 当前）
        this.teachMultiplicationOrder = 'delta-first';

        // 示教器速度百分比（影响“每一步”的平移/旋转增量，不改变按键重复频率）
        this.teachSpeedPercent = 100;

        // 人形点动求解后端：'vfi' | 'priority' | 'mj_restore'
        this.humanoidJogSolverMode = 'vfi';
        this.vfiConfig = {
            maxIter: 240,
            damping: 1e-3,
            kappaCommand: 0.6,
            kappaInvariant: 0.4,
            invariantBandBase: 1e-4,
            invariantBandGain: 0.15,
            // distance-target mode specific options
            distanceQPUseInvariantEquality: false,   // default: avoid over-constraining distance command
            distanceKeepRelativeRotation: true,      // keep relative orientation as invariant task
            distanceKeepHandOrientation: true,       // keep both hand orientations during distance jog
            distanceInvariantKappa: 0.7,
            distanceInvariantBandBase: 2e-3,
            distanceInvariantBandGain: 0.3,
            distanceInvariantSoftWeight: 6.0,
            maxStep: 0.1,
            qpMaxIter: 30,
            qpStepTol: 1e-12,
            qpGradTol: 1e-12,
            qpSolutionTolerance: 1e-12,
            qpRegularization: 1e-6,
            qpUseInvariantEquality: true,
            qpEqualityTolerance: 1e-12,
            qpOuterMaxIter: 16,
            qpMismatchTol: 1e-12,
            qpMismatchRequiredReduction: 0.8,
            qpSingleStepMismatchRequiredReduction: 0.05,
            jogTranslationAxisMinProgressRatio: 0.05,
            jogTranslationAxisMaxOrthogonalRatio: 0.1,
            jointLimitEta: 1.2,
            jointLimitManipulabilityActivationThreshold: 0.15,
            manifoldActiveLimitRetreatHoldFraction: 0.5,
            manifoldActiveLimitRetreatHoldDistance: 1e-3,
            manifoldActiveLimitRetreatTargetFraction: 0.2,
            humanoidTestDualArmLockEnabled: false,
            humanoidTestDualArmLockLimits: {
                ljoint2: { lower: -0.1, upper: 0.2 },
                rjoint2: { lower: -0.2, upper: 0.1 }
            },
            bodyVerticalKappa: 0.8,
            manifoldMaxIter: 120,
            manifoldDamping: 1e-4,
            manifoldReferenceStiffness: 1.0,
            manifoldReferenceDamping: 0.35,
            manifoldLineSearchMinScale: 1e-6,
            manifoldLineSearchReductionFactor: 0.5,
            mjRestoreMaxIter: 120,
            mjRestoreMaxStep: 0.05,
            mjRestoreDamping: 1e-4,
            mjRestoreReferenceStiffness: 1.0,
            mjRestoreReferenceDamping: 0.35,
            mjRestoreRelativeWeight: 1.0,
            mjRestoreBodyVerticalWeight: 4.0,
            mjRestoreBodyTolerance: 5e-5,
            mjRestoreHardTolerance: 1e-8,
            mjRestorePracticalCommandTolPose: 5e-4,
            mjRestoreMetricButtonCommandTolerance: 1e-3,
            mjRestoreMetricButtonInvariantTolerance: 1e-4,
            mjRestoreActiveLimitRetreatCommandTolerance: 1e-2,
            mjRestoreActiveLimitRetreatInvariantTolerance: 2e-3,
            mjRestoreBodyReferenceStiffness: 0.6,
            mjRestoreBodyReferenceDamping: 0.45,
            mjRestoreLineSearchMinScale: 1e-6,
            mjRestoreLineSearchReductionFactor: 0.5,
            mjRestoreNullspaceRestoreMaxIter: 24,
            mjRestoreNullspaceStepSize: 0.01,
            mjRestoreMetricProjectionStepSize: 0.01,
            mjRestoreMetricProjectionPasses: 4,
            mjRestoreWaistMotionRegularization: 6,
            mjRestorePolishMaxIter: 64,
            mjRestorePolishActivationResidual: 1e-4,
            mjRestorePolishDamping: 1e-6,
            mjRestoreArmOnlyFallbackMaxIter: 12,
            mjRestoreArmOnlyFallbackReferenceDamping: 0.2,
            mjRestoreArmOnlyFallbackPolishActivationResidual: 5e-4,
            mjRestoreArmOnlyFallbackPolishDamping: 1e-7,
            mjRestoreEnableNullspaceRefine: false,
            mjRestoreUseWorker: true,
            manifoldJogSecondaryMetric: 'none',
            manifoldNullspaceStepSize: 0.1,
            manifoldWVFIButtonStepSize: 0.1,
            manifoldMetricDirectionTolerance: 1e-12,
            manifoldMetricDirectionMinImprovement: 1e-12,
            manifoldNullspaceRestoreMaxIter: 24,
            manifoldWaistStepFraction: 1.0,
            manifoldRelativeWeight: 1.0,
            manifoldBodyVerticalWeight: 4.0,
            manifoldBodyTolerance: 5e-5,
            hardCommandTolPose: 1e-16,
            hardCommandTolDistanceSq: 1e-8,
            practicalCommandTolPose: 1e-16,
            practicalCommandTolDistanceSq: 1e-5,
            hardInvariantTol: 5e-5,
            practicalInvariantTol: 2.5e-4,
            practicalStepNormTol: 2e-5,
            practicalViolationTol: 1e-8,
            // Waist x-direction corridor (point-to-plane VFI, Section III-B)
            waistXCorridorEnabled: false,
            waistXMax: 0.05,            // 5 cm forward limit
            waistXMin: -Infinity,       // no rear limit by default
            waistXEta: 1.0,             // VFI decay rate
            waistXActivationThreshold: 0.12,
            manifoldWaistXCorridorTolerance: 5e-4,
            manifoldHierarchicalVfiBlend: 1,
            // CCTSM (Section IV)
            cctsmEnabled: true,
            cctsmSVDTolerance: null,    // null → auto
            // Predictive Certification Controller (Section V)
            predictiveCertEnabled: false,
            predictiveHorizon: 3,
            predictiveTimeBudgetMs: 20,
            predictiveSlackTol: 1e-6,
            predictiveBranchScales: [0, 0.4, 0.8],
            predictiveBranchProjectionMaxIter: 12,
            predictiveBranchProjectionTol: 2e-4,
            predictiveBranchProjectionMaxStep: 0.01,
            predictiveBranchProjectionDamping: 2e-3,
            predictiveHysteresisAbsMargin: 5e-4,
            predictiveHysteresisRelMargin: 0.2,
            predictiveMinHoldSteps: 2,
            predictiveWMin: 0.0,
            wvfiLowerBoundDefault: 0.001,
            wvfiLowerBoundWithWaist: 3e-2,
            wvfiGuardBacktrackScales: [1, 0.5, 0.25, 0.125, 0.0625, 0.03125],
            manipGradWeight: 1.0   // secondary task weight for protected ∇w_vfi ascent in VFI+QP
        };

        // ── Regulated Pure Pursuit 路径可视化 ──
        this.rppPathLine = null;           // THREE.Line - 路径折线
        this.rppWaypointMarkers = [];      // THREE.Mesh[] - 航点圆球
        this.rppLookaheadMarker = null;    // THREE.Mesh - 前视点标记
        this.rppPathGroup = null;          // THREE.Group - 路径可视化容器
        this.rppAddWaypointMode = false;   // 是否处于"点击添加航点"模式
        this.onGroundClick = null;         // 地面点击回调 (x, y) => void
        this._groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0); // Z-up 地面
        this._predictiveCertState = new Map();  // memoryKey → {branchIdx, dwell}
    }
    
    /**
     * 初始化 Three.js 场景
     */
    async init() {
        // 创建场景
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x2d2d3e); // 调亮背景色（从 0x1a1a2e 改为 0x2d2d3e）
        
        // 环境物体组
        this.environmentGroup = new THREE.Group();
        this.environmentGroup.name = 'EnvironmentObjects';
        this.scene.add(this.environmentGroup);
        this.environmentObjects.clear();
        
        // 确保canvas尺寸已正确设置
        if (this.canvas.clientWidth === 0 || this.canvas.clientHeight === 0) {
            // 如果canvas尺寸为0，等待一帧再继续
            await new Promise(resolve => requestAnimationFrame(resolve));
        }
        
        // 创建相机
        const aspect = this.canvas.clientWidth / this.canvas.clientHeight || 1;
        this.camera = new THREE.PerspectiveCamera(60, aspect, 0.1, 100);
        
        // 创建渲染器
        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: true,
            alpha: false
        });
        this.renderer.setSize(this.canvas.clientWidth || window.innerWidth, this.canvas.clientHeight || window.innerHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        
        // 更新相机位置（在渲染器设置之后）
        this.updateCameraPosition();
        
        // 确保相机和渲染器同步更新
        this.camera.aspect = this.canvas.clientWidth / this.canvas.clientHeight || 1;
        this.camera.updateProjectionMatrix();
        
        // 添加光源（增强亮度）
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.9); // 从 0.7 增加到 0.9
        this.scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8); // 从 0.6 增加到 0.8
        directionalLight.position.set(3, 3, 5);
        directionalLight.castShadow = true;
        this.scene.add(directionalLight);
        
        const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.5); // 从 0.3 增加到 0.5
        directionalLight2.position.set(-3, -3, 2);
        this.scene.add(directionalLight2);
        
        // 添加地面网格（Z-up: 网格在 XY 平面，调亮网格颜色）
        const gridHelper = new THREE.GridHelper(5, 20, 0x888888, 0x555555); // 从 0x666666/0x333333 调亮到 0x888888/0x555555
        gridHelper.rotation.x = Math.PI / 2; // 旋转到 XY 平面
        this.scene.add(gridHelper);
        
        // 添加世界坐标轴（Z-up）
        const worldAxes = new THREE.AxesHelper(0.5);
        this.scene.add(worldAxes);
        
        // 创建机器人
        await this.createRobot();
        
        // 创建末端执行器标记
        this.createEndEffectorMarker();
        
        // 创建左上角坐标系指示器
        this.createAxesIndicator();
        
        // 设置事件监听
        this.setupEventListeners();
        
        // 确保首次渲染时相机位置正确（在所有对象创建后）
        // 等待一帧确保DOM完全渲染
        await new Promise(resolve => requestAnimationFrame(resolve));
        
        // 再次检查并更新canvas尺寸
        const width = this.canvas.clientWidth || window.innerWidth;
        const height = this.canvas.clientHeight || window.innerHeight;
        if (width > 0 && height > 0) {
            this.renderer.setSize(width, height);
            this.camera.aspect = width / height;
            this.camera.updateProjectionMatrix();
        }
        
        // 更新相机位置
        this.updateCameraPosition();
        
        // 强制更新相机投影矩阵
        this.camera.updateProjectionMatrix();
        
        // 立即渲染一次，确保初始状态正确
        this.renderer.render(this.scene, this.camera);
        
        // 再渲染一次坐标系指示器
        if (this.axesScene && this.axesCamera) {
            const size = 120;
            this.renderer.autoClear = false;
            this.renderer.clearDepth();
            this.renderer.setViewport(20, height - size - 20, size, size);
            this.renderer.setScissor(20, height - size - 20, size, size);
            this.renderer.setScissorTest(true);
            this.renderer.render(this.axesScene, this.axesCamera);
            this.renderer.setScissorTest(false);
            this.renderer.setViewport(0, 0, width, height);
            this.renderer.autoClear = true;
        }
        
        // 绑定画布点击事件（用于路径航点添加）— 记录 mousedown 位置用于过滤拖动
        this.canvas.addEventListener('mousedown', (e) => {
            if (e.button === 0 && this.rppAddWaypointMode) {
                this._rppMouseDownPos = { x: e.clientX, y: e.clientY };
                this._rppMouseDownTime = performance.now();
            }
        }, true); // capture phase，先于其他 handler
        this.canvas.addEventListener('click', (e) => this._handleCanvasClickForWaypoint(e));
        
        // 开始渲染循环
        this.animate();
        
        console.log('✓ 可视化器初始化完成（Z-up 坐标系）');
    }

    /**
     * 设置示教器乘法顺序
     * @param {'delta-first'|'current-first'} order 
     */
    setTeachMultiplicationOrder(order) {
        if (order !== 'delta-first' && order !== 'current-first') {
            console.warn('无效的乘法顺序设置:', order);
            return;
        }
        this.teachMultiplicationOrder = order;
        console.log(`示教器乘法顺序已设置为: ${order === 'delta-first' ? 'Δ × 当前' : '当前 × Δ'}`);
    }

    /**
     * 设置示教器速度百分比（1-100）
     * @param {number} percent
     */
    setTeachSpeedPercent(percent) {
        const p = Math.max(1, Math.min(100, Number(percent) || 100));
        this.teachSpeedPercent = p;
    }

    getTeachSpeedScale() {
        return (this.teachSpeedPercent || 100) / 100;
    }

    getTeachTranslationStep() {
        // 默认每步 5mm，按速度百分比缩放
        return 0.005 * this.getTeachSpeedScale();
    }

    getTeachRotationAngle() {
        // 默认每步 π/128 rad，按速度百分比缩放
        return (Math.PI / 128) * this.getTeachSpeedScale();
    }

    /**
     * 根据乘法顺序组合双四元数
     * @param {*} dqDelta - 增量双四元数
     * @param {*} dqCurrent - 当前双四元数
     * @returns {*} 组合后的双四元数
     */
    combineDQWithTeachOrder(dqDelta, dqCurrent, deltaOnLeft = true) {
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            throw new Error('DQ Robotics 模块未加载');
        }
        const { multiply } = window.DQModule.DQWrapper;
        const isCurrentFirst = this.teachMultiplicationOrder === 'current-first';
        
        if (deltaOnLeft) {
            // 默认：Δ × 当前；切换时交换顺序
            return isCurrentFirst ? multiply(dqCurrent, dqDelta) : multiply(dqDelta, dqCurrent);
        }
        // 默认：当前 × Δ；切换时交换顺序
        return isCurrentFirst ? multiply(dqDelta, dqCurrent) : multiply(dqCurrent, dqDelta);
    }
    
    /**
     * 判断当前是否为世界坐标平移模式
     * @returns {boolean}
     */
    isWorldTranslationMode() {
        return this.teachMultiplicationOrder === 'current-first';
    }

    setHumanoidJogSolverMode(mode) {
        const normalized = normalizeHumanoidJogSolverMode(mode);
        if (mode !== normalized) {
            console.warn('无效的人形点动求解后端:', mode);
        }
        this.humanoidJogSolverMode = normalized;
        if (typeof window !== 'undefined') {
            window.humanoidJogSolverMode = normalized;
        }
        if (normalized === 'mj_restore' || normalized === 'mj_restore_pre_w') {
            void this._warmHumanoidJogMJRestoreWorkerIfNeeded();
        }
        this._publishHumanoidJogDiagnostics({
            solverMode: normalized,
            modeTag: 'idle',
            active: true,
            message: getHumanoidJogSolverReadyMessage(normalized)
        });
    }

    setHumanoidWaistXCorridorEnabled(enabled) {
        if (!this.vfiConfig) this.vfiConfig = {};
        this.vfiConfig.waistXCorridorEnabled = Boolean(enabled);
        this._publishHumanoidJogDiagnostics({
            solverMode: this._getHumanoidJogSolverMode(),
            modeTag: 'idle',
            active: this._getHumanoidJogSolverMode() !== 'priority',
            message: this.vfiConfig.waistXCorridorEnabled
                ? `腰部 X 走廊已启用（x_max=${this.vfiConfig.waistXMax.toFixed(3)}m）`
                : '腰部 X 走廊已关闭。'
        });
    }

    setHumanoidTestDualArmLockEnabled(enabled) {
        if (!this.vfiConfig) this.vfiConfig = {};
        this.vfiConfig.humanoidTestDualArmLockEnabled = Boolean(enabled);
        this._publishHumanoidJogDiagnostics({
            solverMode: this._getHumanoidJogSolverMode(),
            modeTag: 'idle',
            active: true,
            message: this.vfiConfig.humanoidTestDualArmLockEnabled
                ? '双臂测试锁已启用（ljoint2∈[-0.1,0.2], rjoint2∈[-0.2,0.1]）。'
                : '双臂测试锁已关闭。'
        });
    }

    setHumanoidManifoldJogSecondaryMetric(metric) {
        if (!this.vfiConfig) this.vfiConfig = {};
        this.vfiConfig.manifoldJogSecondaryMetric = normalizeHumanoidJogManifoldJogMetric(metric);
    }

    _getHumanoidManifoldJogSecondaryMetric() {
        return normalizeHumanoidJogManifoldJogMetric(this.vfiConfig?.manifoldJogSecondaryMetric);
    }

    setHumanoidWaistXMax(xMaxMetres) {
        if (!this.vfiConfig) this.vfiConfig = {};
        const v = Number(xMaxMetres);
        if (Number.isFinite(v)) this.vfiConfig.waistXMax = v;
    }

    setHumanoidCCTSMEnabled(enabled) {
        if (!this.vfiConfig) this.vfiConfig = {};
        this.vfiConfig.cctsmEnabled = Boolean(enabled);
    }

    setHumanoidPredictiveCertEnabled(enabled) {
        const normalized = Boolean(enabled);
        this.vfiConfig.predictiveCertEnabled = normalized;
        if (!normalized) this._predictiveCertState.clear();
        if (typeof window !== 'undefined') window.humanoidPredictiveCertEnabled = normalized;
    }

    _isPredictiveCertEnabled() {
        if (typeof window !== 'undefined' && typeof window.humanoidPredictiveCertEnabled === 'boolean')
            return window.humanoidPredictiveCertEnabled;
        return this.vfiConfig.predictiveCertEnabled === true;
    }

    _shouldUseInvariantEqualityForStep(forceInvariantEquality = null) {
        if (typeof forceInvariantEquality === 'boolean') return forceInvariantEquality;
        const cfg = this.vfiConfig || {};
        return (typeof cfg.qpUseInvariantEquality === 'boolean') ? cfg.qpUseInvariantEquality : true;
    }

    _getHumanoidJogSolverMode() {
        if (typeof window !== 'undefined' && typeof window.humanoidJogSolverMode === 'string') {
            return normalizeHumanoidJogSolverMode(window.humanoidJogSolverMode);
        }
        return normalizeHumanoidJogSolverMode(this.humanoidJogSolverMode);
    }

    _isHumanoidVFISolverEnabled() {
        return this._getHumanoidJogSolverMode() === 'vfi';
    }

    _isHumanoidMJRestoreSolverEnabled() {
        return this._getHumanoidJogSolverMode() === 'mj_restore';
    }

    _isHumanoidMJRestorePreWSolverEnabled() {
        return this._getHumanoidJogSolverMode() === 'mj_restore_pre_w';
    }

    _isHumanoidMetricOptimizationSolverEnabled() {
        const solverMode = this._getHumanoidJogSolverMode();
        return solverMode === 'mj_restore'
            || solverMode === 'mj_restore_pre_w'
            || solverMode === 'priority'
            || solverMode === 'vfi';
    }

    _publishHumanoidJogMetricOptimizationSolverMismatch(modeTag, metricName) {
        const currentSolverMode = this._getHumanoidJogSolverMode();
        const solverDisplay = getHumanoidJogSolverDisplayName(currentSolverMode);
        const modeText = `当前点动模式`;
        this._publishHumanoidJogDiagnostics({
            active: true,
            solverMode: currentSolverMode,
            modeTag,
            message: `${solverDisplay} 的 ${metricName} 按钮当前仅在 ${modeText} 求解器下生效。`,
            diagnosticLabel: `${modeTag}=solver_mismatch`,
            includeWaist: false,
            keepBodyZVertical: false,
            jointLimitAvoidance: true
        });
    }

    _signedPoseError(target, current) {
        const n = Math.min(target?.length || 0, current?.length || 0);
        if (n === 0) return [];
        let dot = 0;
        for (let i = 0; i < n; i++) dot += (target[i] || 0) * (current[i] || 0);
        const sign = dot < 0 ? -1 : 1;
        const err = new Array(n).fill(0);
        for (let i = 0; i < n; i++) err[i] = (target[i] || 0) - sign * (current[i] || 0);
        return err;
    }

    _computePoseErrorNorm(errorArray) {
        if (!Array.isArray(errorArray) || errorArray.length === 0) {
            return Number.NaN;
        }
        let sumSquares = 0;
        for (let index = 0; index < errorArray.length; index += 1) {
            const value = Number(errorArray[index]);
            if (!Number.isFinite(value)) continue;
            sumSquares += value * value;
        }
        return Math.sqrt(sumSquares);
    }

    _getHumanoidActiveSliceInfo(includeWaist = false) {
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;
        const activeCount = includeWaist
            ? (waistCount + leftArmCount + rightArmCount)
            : (leftArmCount + rightArmCount);
        return {
            waistCount,
            leftArmCount,
            rightArmCount,
            armStart,
            activeCount
        };
    }

    _sliceHumanoidActiveJoints(q, armStart, activeCount) {
        if (!Array.isArray(q) || activeCount <= 0) return [];
        return q.slice(armStart, armStart + activeCount);
    }

    _writeHumanoidActiveJoints(qBase, qActive, armStart, activeCount) {
        const qOut = Array.isArray(qBase) ? [...qBase] : [];
        for (let i = 0; i < activeCount && i < qActive.length; i++) {
            qOut[armStart + i] = qActive[i];
        }
        return qOut;
    }

    _clampHumanoidActiveJoints(q, armStart, hardLimits) {
        if (!Array.isArray(q) || !Array.isArray(hardLimits)) return q;
        for (let i = 0; i < hardLimits.length; i++) {
            const lim = hardLimits[i];
            if (!lim) continue;
            q[armStart + i] = Math.max(lim.lower, Math.min(lim.upper, q[armStart + i]));
        }
        return q;
    }

    _computeHumanoidActiveJointDistance(qA, qB, armStart, activeCount) {
        if (!Array.isArray(qA) || !Array.isArray(qB) || activeCount <= 0) return Number.POSITIVE_INFINITY;
        let norm2 = 0;
        for (let i = 0; i < activeCount; i++) {
            const diff = (qA[armStart + i] || 0) - (qB[armStart + i] || 0);
            norm2 += diff * diff;
        }
        return Math.sqrt(norm2);
    }

    // ── Waist x-corridor VFI helper ────────────────────────────────────────────

    /**
     * Compute waist x-corridor VFI rows for the active joint slice.
     * Returns { rows, rhs, vfiConstraints, p0x, d_upper } or null.
     * @param {number[]} q            full joint vector
     * @param {number}   armStart     first active joint index
     * @param {number}   activeCount  number of active joints
     */
    _getWaistXVFIRows(q, armStart, activeCount) {
        const cfg = this.vfiConfig || {};
        if (!cfg.waistXCorridorEnabled) return null;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !this.humanoidModel || typeof this.humanoidModel.fkmEffector !== 'function') return null;
        const waistCount = this.humanoidWaistLength;
        if (waistCount <= 0) return null;
        // Only relevant when waist joints are included in active range
        if (armStart !== 0) return null;

        const result = computeWaistXVFIRows({
            fkmFn: (qArg) => this.humanoidModel.fkmEffector(qArg, 0),
            q,
            waistStart: 0,
            waistCount,
            DQ,
            xMax: Number.isFinite(cfg.waistXMax) ? cfg.waistXMax : 0.05,
            xMin: Number.isFinite(cfg.waistXMin) ? cfg.waistXMin : -Infinity,
            eta: Number.isFinite(cfg.waistXEta) ? cfg.waistXEta : 1.0,
            activationThreshold: Number.isFinite(cfg.waistXActivationThreshold) ? cfg.waistXActivationThreshold : 0.12
        });
        if (!result || result.rows.length === 0) return result;

        // The rows from computeWaistXVFIRows are in waist-joint space (length waistCount).
        // We need to embed them into activeCount-length vectors (zeros elsewhere).
        const embed = (row) => {
            const out = new Array(activeCount).fill(0);
            for (let j = 0; j < waistCount && j < row.length; j++) out[j] = row[j];
            return out;
        };
        return {
            ...result,
            rows: result.rows.map(embed),
            vfiConstraints: result.vfiConstraints.map((c) => ({ ...c, row: embed(c.row) }))
        };
    }

    _buildHumanoidManifoldWaistXCorridor({ qFull, qInitialFull, armStart, activeCount, includeWaist }) {
        const cfg = this.vfiConfig || {};
        const enabled = includeWaist && cfg.waistXCorridorEnabled === true && armStart === 0;
        const vfi = enabled ? this._getWaistXVFIRows(qFull, armStart, activeCount) : null;
        return {
            enabled,
            min: Number.isFinite(cfg.waistXMin) ? cfg.waistXMin : Number.NEGATIVE_INFINITY,
            max: Number.isFinite(cfg.waistXMax) ? cfg.waistXMax : Number.POSITIVE_INFINITY,
            tolerance: Number.isFinite(cfg.manifoldWaistXCorridorTolerance)
                ? Math.max(0, cfg.manifoldWaistXCorridorTolerance)
                : 5e-4,
            vfiRows: Array.isArray(vfi?.rows) ? vfi.rows.map((row) => row.slice()) : [],
            rhs: Array.isArray(vfi?.rhs) ? vfi.rhs.slice() : [],
            evaluateX: (qNextActive) => this._evaluateHumanoidWaistXPosition(
                this._writeHumanoidActiveJoints(qInitialFull, qNextActive, armStart, activeCount)
            )
        };
    }

    _buildManipulabilityVFIConstraints(q, armStart, activeCount, activeLimits = null) {
        const cfg = this.vfiConfig || {};
        const qActive = this._sliceHumanoidActiveJoints(q, armStart, activeCount);
        const extraConstraints = this._getWaistXVFIRows(q, armStart, activeCount)?.vfiConstraints ?? null;
        return buildManipulabilityVFIConstraints({
            qActive,
            activeLimits,
            activationThreshold: Number.isFinite(cfg.jointLimitManipulabilityActivationThreshold)
                ? Math.max(0, cfg.jointLimitManipulabilityActivationThreshold)
                : 0.05,
            extraConstraints
        });
    }

    _computeProtectedManipulabilityGradient({
        q,
        armStart,
        activeCount,
        mode,
        Jcmd,
        Jinvariant = null,
        activeLimits = null,
        forceInvariantEquality = null,
        metric = 'wvfi',
        directionalVector = null,
        protectCommandRows = true
    }) {
        const cfg = this.vfiConfig || {};
        const manipGradWeight = Number.isFinite(cfg.manipGradWeight) ? cfg.manipGradWeight : 1.0;
        if (manipGradWeight <= 0) return null;

        const qActive = this._sliceHumanoidActiveJoints(q, armStart, activeCount);
        if (!Array.isArray(qActive) || qActive.length === 0) return null;

        const protectRows = [];
        if (protectCommandRows && Array.isArray(Jcmd)) {
            for (const row of Jcmd) {
                if (Array.isArray(row) && row.length === activeCount) protectRows.push(row.slice());
            }
        }
        if (this._shouldUseInvariantEqualityForStep(forceInvariantEquality) && Array.isArray(Jinvariant)) {
            for (const row of Jinvariant) {
                if (Array.isArray(row) && row.length === activeCount) protectRows.push(row.slice());
            }
        }

        const evalFn = (qAct) => {
            if (!Array.isArray(qAct) || qAct.length !== activeCount) return null;
            const qFull = this._writeHumanoidActiveJoints(q, qAct, armStart, activeCount);
            const vfiConstraints = this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, activeLimits);
            const cctsm = this._computeModeCCTSM(qFull, armStart, activeCount, mode, vfiConstraints);
            if (!cctsm) return null;
            let scalar = Number.NaN;
            if (metric === 'w') {
                scalar = asFinite(evaluateHumanoidJogProjectedManipulability(cctsm), Number.NaN);
            } else if (metric === 'w_abs') {
                scalar = asFinite(evaluateHumanoidJogAbsoluteManipulability(cctsm), Number.NaN);
            } else if (metric === 'w_abs_arm') {
                scalar = asFinite(evaluateHumanoidJogAbsoluteArmManipulability(cctsm), Number.NaN);
            } else if (metric === 'w_abs_vfi') {
                scalar = asFinite(evaluateHumanoidJogAbsoluteVFIManipulability(cctsm), Number.NaN);
            } else if (metric === 'w_abs_hier') {
                scalar = asFinite(evaluateHumanoidJogAbsoluteManipulability(cctsm), Number.NaN);
            } else if (metric === 'w_abs_hier_vfi') {
                scalar = asFinite(evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability(
                    cctsm,
                    { hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend() }
                ), Number.NaN);
            } else if (metric === 'sigma') {
                scalar = Array.isArray(directionalVector)
                    ? asFinite(directionalCCTSM(cctsm.Gram, directionalVector), Number.NaN)
                    : Number.NaN;
            } else {
                scalar = asFinite(cctsm.w_vfi, Number.NaN);
            }
            return Number.isFinite(scalar) ? { w_vfi: scalar } : null;
        };

        const projectedGrad = computeProtectedSecondaryGradient({
            evalFn,
            qActive,
            protectRows,
            eps: Number.isFinite(cfg.manipGradEps) ? Math.max(1e-7, cfg.manipGradEps) : 1e-5,
            damping: Number.isFinite(cfg.manipGradProjectionDamping)
                ? Math.max(1e-12, cfg.manipGradProjectionDamping)
                : 1e-6,
            normalize: true
        });

        if (!Array.isArray(projectedGrad) || projectedGrad.length !== activeCount) return null;
        const gradNorm = Math.sqrt(projectedGrad.reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
        return gradNorm > 1e-8 ? projectedGrad : null;
    }

    _getWVFILowerBound(includeWaist = false) {
        const cfg = this.vfiConfig || {};
        return selectWVFILowerBound({
            includeWaist,
            lowerBoundWithWaist: cfg.wvfiLowerBoundWithWaist,
            lowerBoundDefault: cfg.wvfiLowerBoundDefault
        });
    }

    _getHumanoidHierarchicalVfiBlend() {
        const cfg = this.vfiConfig || {};
        const value = Number.isFinite(cfg.manifoldHierarchicalVfiBlend)
            ? cfg.manifoldHierarchicalVfiBlend
            : Number.isFinite(cfg.hierarchicalVfiBlend)
                ? cfg.hierarchicalVfiBlend
                : 0;
        return Math.min(1, Math.max(0, value));
    }

    _getWVFIGuardBacktrackScales() {
        const configured = this.vfiConfig?.wvfiGuardBacktrackScales;
        if (Array.isArray(configured)) {
            const sanitized = configured.filter((value) => Number.isFinite(value) && value > 0);
            if (sanitized.length > 0) return sanitized;
        }
        return [1, 0.5, 0.25, 0.125, 0.0625, 0.03125];
    }

    _buildWVFILowerBoundGuard({
        q,
        armStart,
        activeCount,
        mode,
        activeLimits = null,
        includeWaist = false
    } = {}) {
        const lowerBound = this._getWVFILowerBound(includeWaist);
        return {
            lowerBound,
            apply: (dqCandidate) => enforceWVFILowerBound({
                dqCandidate,
                lowerBound,
                backtrackScales: this._getWVFIGuardBacktrackScales(),
                evaluateWVFI: (dqScaled) => {
                    const qNext = Array.isArray(q) ? q.slice() : [];
                    for (let i = 0; i < activeCount; i++) {
                        qNext[armStart + i] = (qNext[armStart + i] || 0) + (dqScaled[i] || 0);
                    }
                    this._clampHumanoidActiveJoints(qNext, armStart, activeLimits);
                    const vfiConstraints = this._buildManipulabilityVFIConstraints(qNext, armStart, activeCount, activeLimits);
                    const cctsm = this._computeModeCCTSM(qNext, armStart, activeCount, mode, vfiConstraints, { force: true });
                    return cctsm?.w_vfi ?? Number.NaN;
                }
            })
        };
    }

    // ── CCTSM computation ──────────────────────────────────────────────────────

    /**
     * Compute CCTSM (and VFI-weighted CCTSM) for the current mode and config.
     *
     * @param {number[]} q           full joint vector
     * @param {number}   armStart    first active joint index in q
     * @param {number}   activeCount number of active joints
     * @param {string}   mode        jog mode identifier
     * @param {Array}    vfiConstraints  near-active VFI constraints (for W_vfi)
     * @returns {{ w: number, w_vfi: number, N_z: number[][], G_vfi: number[][] } | null}
     */
    _computeModeCCTSM(q, armStart, activeCount, mode, vfiConstraints = [], options = {}) {
        const cfg = this.vfiConfig || {};
        const force = options?.force === true;
        if (!this._cctsmCallCount) this._cctsmCallCount = 0;
        this._cctsmCallCount++;
        if (this._cctsmCallCount <= 3) console.log('[CCTSM entry]', mode, 'call#', this._cctsmCallCount, '| cctsmEnabled:', cfg.cctsmEnabled, '| armStart:', armStart, 'activeCount:', activeCount);
        if (!cfg.cctsmEnabled && !force) return null;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) { if (this._cctsmCallCount <= 3) console.log('[CCTSM] early exit: no DQ'); return null; }
        // this.humanoidModel is only needed for waist FK; absolute/relative/distance
        // use _getHumanoidAbsolutePose etc. which rely on window.dualArmSystem.

        const svdTol = cfg.cctsmSVDTolerance ?? null;

        // Helper: slice active columns from an 8×n Jacobian
        const sliceCols = (J8full) => {
            if (!Array.isArray(J8full)) return null;
            return J8full.map((row) => row.slice(armStart, armStart + activeCount));
        };

        // Helper: slice active columns from a k×n Jacobian (any row count)
        const sliceColsK = sliceCols;

        let J_xi_y = null;
        let J_xi_z = null;

        if (mode === 'absolute' || mode === 'decoupled' || mode === 'decoupled-upright') {
            const xA = this._getHumanoidAbsolutePose(q);
            const xR = this._getHumanoidRelativePose(q);
            const JA8 = this._getHumanoidAbsolutePoseJacobian(q);
            const JR8 = this._getHumanoidRelativePoseJacobian(q);
            if (this._cctsmCallCount <= 3) console.log('[CCTSM abs] xA:', !!xA, 'xR:', !!xR, 'JA8:', Array.isArray(JA8) ? JA8.length+'×'+JA8[0]?.length : JA8, 'JR8:', Array.isArray(JR8) ? JR8.length+'×'+JR8[0]?.length : JR8);
            if (!xA || !xR || !JA8 || !JR8) { console.log('[CCTSM] early exit: null pose/jac'); return null; }
            const JA_active = sliceCols(JA8);
            const JR_active = sliceCols(JR8);
            J_xi_y = computeTwistJacobian(JA_active, xA);
            J_xi_z = computeTwistJacobian(JR_active, xR);
            if (this._cctsmCallCount <= 3) console.log('[CCTSM abs] JA_active:', Array.isArray(JA_active) ? JA_active.length+'×'+JA_active[0]?.length : JA_active, '| J_xi_y:', Array.isArray(J_xi_y) ? J_xi_y.length+'×'+J_xi_y[0]?.length : J_xi_y, '| J_xi_z:', Array.isArray(J_xi_z) ? J_xi_z.length+'×'+J_xi_z[0]?.length : J_xi_z);
        } else if (mode === 'relative' || mode === 'relative-fullbody') {
            const xA = this._getHumanoidAbsolutePose(q);
            const xR = this._getHumanoidRelativePose(q);
            const JA8 = this._getHumanoidAbsolutePoseJacobian(q);
            const JR8 = this._getHumanoidRelativePoseJacobian(q);
            if (!xA || !xR || !JA8 || !JR8) return null;
            const JA_active = sliceCols(JA8);
            const JR_active = sliceCols(JR8);
            J_xi_y = computeTwistJacobian(JR_active, xR);
            J_xi_z = computeTwistJacobian(JA_active, xA);
        } else if (mode === 'distance' || mode === 'distance-fullbody') {
            const Kinematics = window.DQModule?.Kinematics;
            if (!Kinematics?.distanceJacobian) return null;
            const xA = this._getHumanoidAbsolutePose(q);
            const xR = this._getHumanoidRelativePose(q);
            const JA8 = this._getHumanoidAbsolutePoseJacobian(q);
            const JR8 = this._getHumanoidRelativePoseJacobian(q);
            if (!xA || !xR || !JA8 || !JR8) return null;
            const JR8full = this._getHumanoidRelativePoseJacobian(q);
            const Jd2full = Kinematics.distanceJacobian(JR8full, xR); // 1×n
            const JA_active = sliceCols(JA8);
            const JR_active = sliceCols(JR8);
            const Jd2_active = sliceColsK(Jd2full);
            J_xi_y = Jd2_active; // 1×activeCount (already minimal)
            const J_xi_a = computeTwistJacobian(JA_active, xA); // 6×activeCount
            const J_omega_r = computeAngularJacobian(JR_active.slice(0, 4), xR.slice(0, 4)); // 3×activeCount
            if (!J_xi_a || !J_omega_r) return null;
            J_xi_z = [...J_xi_a, ...J_omega_r]; // 9×activeCount
        } else {
            return null; // unsupported mode
        }

        if (!J_xi_y || !J_xi_z) return null;

        // For absolute mode, use the combined-task CCTSM: stack [J_xi_y; J_xi_z] so
        // that both the commanded (absolute pose) and the preserved (relative pose)
        // tasks contribute directly to the manipulability measure (no null-space
        // projection).  For other modes, keep the original projected formulation.
        const useCombined = (mode === 'absolute' || mode === 'decoupled' || mode === 'decoupled-upright');
        const absOnlyResult = useCombined
            ? computeCombinedCCTSM(J_xi_y, [])
            : null;
        const waistActiveCount = armStart === 0 ? Math.min(this.humanoidWaistLength, activeCount) : 0;
        const armOnlyStart = Math.max(0, waistActiveCount);
        const armOnlyColumns = activeCount - armOnlyStart;
        const absArmOnlyResult = useCombined && armOnlyColumns > 0
            ? computeCombinedCCTSM(
                J_xi_y.map((row) => row.slice(armOnlyStart, armOnlyStart + armOnlyColumns)),
                []
            )
            : absOnlyResult;
        const result = useCombined
            ? computeCombinedCCTSM(J_xi_y, J_xi_z)
            : computeCCTSM(J_xi_y, J_xi_z, null);
        const projectedResult = useCombined
            ? computeCCTSM(J_xi_y, J_xi_z, null)
            : result;

        if (!result) {
            const dbgKey = `cctsm_null_${mode}`;
            if (!this[dbgKey]) { this[dbgKey] = true; console.debug('[CCTSM] computation returned null for mode', mode); }
            return null;
        }
        // One-shot detailed diagnostic
        const dbgKey3 = `cctsm_diag_${mode}`;
        if (!this[dbgKey3]) {
            this[dbgKey3] = true;
            const gramDiag = result.Gram.map((r, i) => r[i].toFixed(6));
            const Nz = result.N_z;
            console.debug('[CCTSM diag]', mode, useCombined ? '(combined)' : '(projected)',
                '| J_xi_y:', J_xi_y.length, '×', J_xi_y[0]?.length,
                '| J_xi_z:', J_xi_z.length, '×', J_xi_z[0]?.length,
                '| N_z:', Nz?.length, '×', Nz?.[0]?.length,
                '| Gram:', result.Gram.length, '×', result.Gram[0]?.length,
                '| w:', result.w.toExponential(3));
        }

        // VFI-weighted CCTSM
        const W_vfi = computeVFIAttenuationMatrix(vfiConstraints, activeCount);
        const W_vfi_arm = armOnlyColumns > 0
            ? W_vfi.slice(armOnlyStart, armOnlyStart + armOnlyColumns)
                .map((row) => Array.isArray(row) ? row.slice(armOnlyStart, armOnlyStart + armOnlyColumns) : new Array(armOnlyColumns).fill(0))
            : null;
        const absOnlyVfiResult = useCombined
            ? computeCombinedVFICCTSM(J_xi_y, W_vfi)
            : null;
        const absArmOnlyVfiResult = useCombined && armOnlyColumns > 0
            ? computeCombinedVFICCTSM(
                J_xi_y.map((row) => row.slice(armOnlyStart, armOnlyStart + armOnlyColumns)),
                W_vfi_arm
            )
            : absOnlyVfiResult;
        const vfiResult = useCombined
            ? computeCombinedVFICCTSM(result.J_full, W_vfi)
            : computeVFICCTSM(J_xi_y, result.N_z, W_vfi);

        const dbgKey2 = `cctsm_zero_${mode}`;
        if (result.w < 1e-10 && !this[dbgKey2]) {
            this[dbgKey2] = true;
            console.debug('[CCTSM] w≈0 for mode', mode, '— J_xi_y:', J_xi_y.length, '×', J_xi_y[0]?.length, 'N_z:', result.N_z?.length, '×', result.N_z?.[0]?.length, 'Gram det≈', result.w ** 2);
        }
        const hierarchicalBlend = this._getHumanoidHierarchicalVfiBlend();
        const wAbs = absOnlyResult?.w;
        const wAbsVfi = absOnlyVfiResult?.w_vfi;
        const wAbsHier = Number.isFinite(wAbs)
            ? wAbs
            : (Number.isFinite(wAbsVfi)
                ? wAbsVfi
                : Number.NaN);
        const blendedAbsHierVfi = Number.isFinite(wAbsHier) && Number.isFinite(wAbsVfi)
            ? (1 - hierarchicalBlend) * wAbsHier + hierarchicalBlend * wAbsVfi
            : Number.isFinite(wAbs)
                ? wAbsHier
                : (Number.isFinite(wAbsVfi)
                    ? wAbsVfi
                    : Number.NaN);
        const wAbsHierVfi = Number.isFinite(wAbsHier) && Number.isFinite(blendedAbsHierVfi)
            ? Math.min(wAbsHier, blendedAbsHierVfi)
            : blendedAbsHierVfi;
        return {
            w: result.w,
            w_vfi: vfiResult?.w_vfi ?? result.w,
            w_cmd: projectedResult?.w ?? result.w,
            w_abs: absOnlyResult?.w ?? Number.NaN,
            w_abs_hier: wAbsHier,
            w_abs_hier_vfi: wAbsHierVfi,
            w_abs_arm: absArmOnlyResult?.w ?? Number.NaN,
            w_abs_arm_vfi: absArmOnlyVfiResult?.w_vfi ?? Number.NaN,
            w_abs_vfi: absOnlyVfiResult?.w_vfi ?? Number.NaN,
            N_z: result.N_z,
            J_bar: result.J_bar,
            Gram: result.Gram,
            Gram_cmd: projectedResult?.Gram ?? result.Gram,
            Gram_abs: absOnlyResult?.Gram ?? null,
            G_vfi: vfiResult?.G_vfi ?? null
        };
    }

    /**
     * Per-tick manipulability-aware predictive certification (Algorithm 1).
     *
     * Replaces the standard one-step jog with a multi-branch rollout that selects
     * the branch with the strongest short-horizon certificate.
     *
     * @param {object} opts
     * @param {number[]}  opts.q              full joint vector (length n_total)
     * @param {number}    opts.armStart       first active joint index in q
     * @param {number}    opts.activeCount    number of active joints
     * @param {string}    opts.mode           'absolute' | 'relative' | 'distance'
     * @param {string}    opts.memoryKey      hysteresis memory key
     * @param {function}  opts.oneStepFn      (qActive) → { dq, slack } | null
     * @param {function}  opts.taskErrFn      (qActive) → error vector (for projection)
     * @param {function}  opts.taskJacFn      (qActive) → Jacobian (for projection)
     * @param {number[][]|null} opts.hardLimits  [[lo,hi],...] per active joint
     * @param {function}  opts.cctsmEvalFn    (qActive) → { w_vfi } | null
     * @param {number[]}  opts.jogDir         commanded jog direction unit vector (task space)
     * @returns {{ dqActive, qActive, label, h, mu, sigma, cctsmW, cctsmWvfi } | null}
     */
    _runPredictiveCert({
        q, armStart, activeCount, mode, memoryKey,
        oneStepFn, taskErrFn, taskJacFn, hardLimits,
        cctsmEvalFn, jogDir, wMinOverride = null
    }) {
        const cfg = this.vfiConfig || {};
        const horizon = Math.max(1, Math.floor(asFinite(cfg.predictiveHorizon, 3)));
        const timeBudgetMs = asFinite(cfg.predictiveTimeBudgetMs, 20);
        const slackTol = asFinite(cfg.predictiveSlackTol, 1e-4);
        const scales = Array.isArray(cfg.predictiveBranchScales) ? cfg.predictiveBranchScales : [0, 0.4, 0.8];
        const projOpts = {
            maxIter: Math.max(1, Math.floor(asFinite(cfg.predictiveBranchProjectionMaxIter, 12))),
            tol: asFinite(cfg.predictiveBranchProjectionTol, 2e-4),
            maxStep: asFinite(cfg.predictiveBranchProjectionMaxStep, 0.01),
            damping: asFinite(cfg.predictiveBranchProjectionDamping, 2e-3)
        };
        const hysteresisOpts = {
            hysteresisAbs: asFinite(cfg.predictiveHysteresisAbsMargin, 5e-4),
            hysteresisRel: asFinite(cfg.predictiveHysteresisRelMargin, 0.2),
            minHold: Math.max(1, Math.floor(asFinite(cfg.predictiveMinHoldSteps, 2))),
            horizon
        };
        const wMin = Number.isFinite(wMinOverride)
            ? asFinite(wMinOverride, 0.0)
            : asFinite(cfg.predictiveWMin, 0.0);

        const qActive = q.slice(armStart, armStart + activeCount);
        const cctsmAtCurrent = cctsmEvalFn(qActive);
        const wvfiAtCurrent = cctsmAtCurrent ? asFinite(cctsmAtCurrent.w_vfi, 0) : 0;

        // --- Stage 2a: gradient of w_m^vfi, projected through null(J_xi_z) ---
        // In absolute mode, J_xi_z = relative Jacobian (invariant).  Seeds must not
        // disturb the relative pose, so we project the manipulability gradient onto
        // null(J_xi_z) = N_z.  Seeds may deviate slightly in the commanded (absolute)
        // task; projectToManifold corrects that deviation.  Scale clamping (below)
        // bounds the deviation so the projection budget is not exceeded.
        let gProjCache = null; // cache for scale clamping below
        const gradFn = (qAct) => {
            const eps = 1e-5;
            const grad = new Array(qAct.length).fill(0);
            for (let j = 0; j < qAct.length; j++) {
                const qp = qAct.slice(); qp[j] += eps;
                const qm = qAct.slice(); qm[j] -= eps;
                const rp = cctsmEvalFn(qp);
                const rm = cctsmEvalFn(qm);
                if (rp && rm) grad[j] = (asFinite(rp.w_vfi, 0) - asFinite(rm.w_vfi, 0)) / (2 * eps);
            }
            // Project gradient into null(J_xi_z): g_proj = N_z * N_z^T * grad
            const cctsmCur = cctsmEvalFn(qAct);
            const Nz = cctsmCur?.N_z;
            if (Nz && Nz.length === qAct.length && Nz[0]?.length > 0) {
                const k = Nz[0].length, n = qAct.length;
                const alpha = new Array(k).fill(0);
                for (let ci = 0; ci < k; ci++)
                    for (let j = 0; j < n; j++) alpha[ci] += Nz[j][ci] * grad[j];
                const gProj = new Array(n).fill(0);
                for (let j = 0; j < n; j++)
                    for (let ci = 0; ci < k; ci++) gProj[j] += Nz[j][ci] * alpha[ci];
                gProjCache = { alpha, Jbar: cctsmCur?.J_bar };
                return gProj;
            }
            gProjCache = null;
            return grad;
        };

        // --- Stage 2a: generate seeds with scale clamping ---
        // Clamp each configured scale so the expected commanded-task drift at the seed
        // stays within the projection budget (maxIter × maxStep), preventing flyaway.
        const projBudget = projOpts.maxIter * projOpts.maxStep;
        // We need the gradient at qActive first to compute drift-per-unit
        const _g0 = gradFn(qActive); // populates gProjCache
        const _gNorm = Math.sqrt(_g0.reduce((s, v) => s + asFinite(v) ** 2, 0));
        let adaptedScales = scales;
        if (_gNorm > 1e-12 && gProjCache?.Jbar && Array.isArray(gProjCache.Jbar)) {
            const { alpha: alphaVec, Jbar } = gProjCache;
            const m6 = Jbar.length;
            // drift direction = J_bar * (alpha / gNorm)  [commanded-task drift per unit scale]
            const driftVec = new Array(m6).fill(0);
            for (let r = 0; r < m6; r++)
                for (let ci = 0; ci < alphaVec.length; ci++)
                    driftVec[r] += (Jbar[r][ci] || 0) * alphaVec[ci] / _gNorm;
            const driftPerUnit = Math.sqrt(driftVec.reduce((s, v) => s + v * v, 0));
            if (driftPerUnit > 1e-12) {
                const maxSafe = projBudget / driftPerUnit;
                adaptedScales = scales.map(a => a === 0 ? 0 : Math.min(a, maxSafe));
            }
        }
        const seeds = generateBranchSeeds(qActive, adaptedScales, gradFn);

        // --- Stage 2b + 3 + 4: for each branch, project → one-step QP → rollout ---
        const tStart = performance.now();
        const branchResults = [];

        for (let bi = 0; bi < seeds.length; bi++) {
            // Time budget check
            if (bi > 0 && performance.now() - tStart > timeBudgetMs) break;

            // Project seed to task-equivalent manifold
            let qBranch;
            if (bi === 0) {
                // Real branch: no projection needed
                qBranch = qActive.slice();
            } else {
                qBranch = projectToManifold(seeds[bi], taskErrFn, taskJacFn, hardLimits, projOpts);
                // Discard branch if projection did not converge within εp
                const projErr = taskErrFn(qBranch);
                const projErrNorm = projErr ? Math.sqrt(projErr.reduce((s, v) => s + asFinite(v) ** 2, 0)) : Infinity;
                if (projErrNorm > projOpts.tol) {
                    branchResults.push({ h: 0, mu: 0, sigma: 0, deltaW: -Infinity, cost: Infinity, branchIdx: bi, dq: null, qNext: qBranch });
                    continue;
                }
            }

            // One-step branch-local QP (Stage 3)
            const oneStep = oneStepFn(qBranch);
            if (!oneStep || !Array.isArray(oneStep.dq)) {
                branchResults.push({ h: 0, mu: 0, sigma: 0, deltaW: -Infinity, cost: Infinity, branchIdx: bi, dq: null, qNext: qBranch });
                continue;
            }

            const slack = asFinite(oneStep.slack, Infinity);
            if (slack > slackTol) {
                branchResults.push({ h: 0, mu: 0, sigma: 0, deltaW: -Infinity, cost: Infinity, branchIdx: bi, dq: oneStep.dq, qNext: qBranch });
                continue;
            }

            const qNext = qBranch.map((v, j) => v + asFinite(oneStep.dq[j]));

            // reserve = w_vfi - wMin.  The slack filter above already rejected branches
            // with real QP constraint violations (slack > slackTol), so we only need to
            // track the CCTSM margin here.  Mixing in hardMargin introduced spurious
            // negative reserves from tiny QP numerical errors (~1e-7), which incorrectly
            // stopped the rollout and caused branches with h=1, mu<0 to be selected.
            const cctsmAtNext = cctsmEvalFn(qNext);
            const wvfiAtNext = cctsmAtNext ? asFinite(cctsmAtNext.w_vfi, 0) : null;
            const deltaW = wvfiAtNext !== null ? (wvfiAtNext - wvfiAtCurrent) : -Infinity;
            const reserveAtNext = wvfiAtNext !== null ? wvfiAtNext - wMin : 0;
            // σ = λ_min(G_vfi): worst-case directional CCTSM across all directions.
            // Avoids the "dagger" anti-correlation where a single-direction σ is large
            // precisely when the ellipsoid is singular in perpendicular directions.
            const computeSigma = (cc) => {
                if (!cc) return 0;
                if (cc.G_vfi) return asFinite(minEigenvalueCCTSM(cc.G_vfi), 0);
                return asFinite(cc.w_vfi, 0);
            };
            const sigmaAtNext = computeSigma(cctsmAtNext);

            // Stage 4: rollout
            const rollout = rolloutBranch({
                qStart: qNext,
                horizon: horizon - 1,  // already did 1 step
                slackTol,
                stepFn: (qAct) => {
                    if (performance.now() - tStart > timeBudgetMs * 2) return null;
                    const res = oneStepFn(qAct);
                    if (!res || !Array.isArray(res.dq)) return null;
                    const qN = qAct.map((v, j) => v + asFinite(res.dq[j]));
                    const cc = cctsmEvalFn(qN);
                    const wvfi = cc ? asFinite(cc.w_vfi, 0) : null;
                    return {
                        slack: asFinite(res.slack, Infinity),
                        qNext: qN,
                        reserve: wvfi !== null ? wvfi - wMin : 0,
                        sigma: computeSigma(cc)
                    };
                }
            });

            // h = 1 (one-step passed) + rollout.h
            const totalH = 1 + rollout.h;
            const totalMu = Math.min(reserveAtNext, rollout.mu);
            const totalSigma = Math.min(sigmaAtNext, rollout.sigma);

            // Branch cost: tracking residual (use step norm as proxy) + slack
            const stepNorm = Math.sqrt(oneStep.dq.reduce((s, v) => s + asFinite(v) ** 2, 0));
            const cost = stepNorm + 10 * slack;

            branchResults.push({ h: totalH, mu: totalMu, sigma: totalSigma, deltaW, cost, branchIdx: bi, dq: oneStep.dq, qNext, qBranch });
        }

        // --- Stage 5: selection ---
        const prevState = this._predictiveCertState.get(memoryKey) || null;
        const { selected, state, label } = selectBranch(branchResults, prevState, hysteresisOpts);
        this._predictiveCertState.set(memoryKey, state);

        if (!selected || !selected.dq) {
            return { dqActive: null, qActive: null, label: 'unreachable', h: 0, mu: 0, sigma: 0 };
        }

        // Compute final CCTSM for the selected branch
        const cctsmFinal = cctsmEvalFn(selected.qNext);

        return {
            dqActive: selected.dq,
            qActive: selected.qNext,
            label,
            h: selected.h,
            mu: selected.mu,
            sigma: selected.sigma,
            cctsmW: cctsmFinal?.w ?? NaN,
            cctsmWvfi: cctsmFinal?.w_vfi ?? NaN,
            branchIdx: selected.branchIdx
        };
    }

    _getHumanoidBodyVerticalState(q, armStart, activeCount) {
        const DQ = window.DQModule?.DQWrapper;
        const Kinematics = window.DQModule?.Kinematics;
        if (
            !DQ ||
            !Kinematics?.lineJacobian ||
            !this.humanoidModel ||
            typeof this.humanoidModel.fkmEffector !== 'function' ||
            typeof this.humanoidModel.poseJacobianEffector !== 'function'
        ) {
            return null;
        }

        const bodyZLineDirection = [0, 0, 0, 1, 0, 0, 0, 0];
        const waistPose = this.humanoidModel.fkmEffector(q, 0);
        const JWaistFull = this.humanoidModel.poseJacobianEffector(q, 0);
        if (!waistPose || !JWaistFull) return null;

        const JLineFull = Kinematics.lineJacobian(JWaistFull, waistPose, bodyZLineDirection);
        const waistPoseDQ = DQ.createFromArray(waistPose);
        const rotMat = DQ.getRotationMatrix(waistPoseDQ);
        return buildBodyVerticalLineTask({
            rotMat,
            lineJacobianFull: JLineFull,
            armStart,
            activeCount
        });
    }

    _matVecProduct(M, v) {
        if (!Array.isArray(M) || M.length === 0 || !Array.isArray(v)) return [];
        const out = new Array(M.length).fill(0);
        for (let i = 0; i < M.length; i++) {
            let sum = 0;
            const row = M[i] || [];
            const n = Math.min(row.length, v.length);
            for (let j = 0; j < n; j++) sum += (row[j] || 0) * (v[j] || 0);
            out[i] = sum;
        }
        return out;
    }

    _formatDiagNumber(value, digits = 3) {
        if (!Number.isFinite(value)) return 'n/a';
        const abs = Math.abs(value);
        if (abs === 0) return '0';
        if (abs >= 1000 || abs < 1e-3) return value.toExponential(2);
        return value.toFixed(digits);
    }

    _notifyHumanoidJogUnreachable(message = '无法到达该位置') {
        if (typeof window === 'undefined' || typeof window.alert !== 'function') return;
        const now = Date.now();
        const cooldownMs = 1500;
        const last = Number.isFinite(this._lastHumanoidJogUnreachableAlertAt)
            ? this._lastHumanoidJogUnreachableAlertAt
            : 0;
        if (now - last < cooldownMs) return;
        this._lastHumanoidJogUnreachableAlertAt = now;
        window.alert(message);
    }

    _shouldFailForQPMismatch({ stopReason, finalCmdNorm, initialCmdNorm }, cfg = {}) {
        if (stopReason === 'qp_not_converged') return false;
        const mismatchTol = Number.isFinite(cfg.qpMismatchTol) ? cfg.qpMismatchTol : 1e-3;
        const requiredReduction = Number.isFinite(cfg.qpMismatchRequiredReduction)
            ? Math.min(0.99, Math.max(0, cfg.qpMismatchRequiredReduction))
            : 0.8;
        const singleStepRequiredReduction = Number.isFinite(cfg.qpSingleStepMismatchRequiredReduction)
            ? Math.min(0.99, Math.max(0, cfg.qpSingleStepMismatchRequiredReduction))
            : 0.05;
        return shouldRejectHumanoidJogForMismatch({
            stopReason,
            finalCommandNorm: finalCmdNorm,
            initialCommandNorm: initialCmdNorm,
            mismatchTol,
            mismatchRequiredReduction: requiredReduction,
            singleStepMismatchRequiredReduction: singleStepRequiredReduction,
            explicitFailureReasons: new Set(['fk_fail', 'jacobian_fail', 'body_task_fail', 'solver_fail'])
        }).shouldReject;
    }

    _applyHumanoidJointSolution(qSolved) {
        if (!Array.isArray(qSolved) || !window.humanoidQ) return false;
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        if (window.clampJointAngles && this.humanoidURDF && window.customRobotConfig) {
            const clamped = window.clampJointAngles(qSolved, this.humanoidJointNames || [], this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }

    _buildTranslationAxisGuard(delta) {
        if (!delta) return null;
        const x = Number(delta.x);
        const y = Number(delta.y);
        const z = Number(delta.z);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return null;
        return { commandDelta: [x, y, z] };
    }

    _getDQTranslationVector(dqArray) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !Array.isArray(dqArray) || dqArray.length < 8) return null;
        const translation = DQ.getTranslation(DQ.createFromArray(dqArray));
        return [
            Number(translation?.[1] || 0),
            Number(translation?.[2] || 0),
            Number(translation?.[3] || 0)
        ];
    }

    _evaluateHumanoidTranslationAxisGuard(startPoseArray, finalPoseArray, translationAxisGuard, options = {}) {
        const commandDelta = translationAxisGuard?.commandDelta;
        if (!Array.isArray(commandDelta) || commandDelta.length < 3) return null;
        const startTranslation = this._getDQTranslationVector(startPoseArray);
        const finalTranslation = this._getDQTranslationVector(finalPoseArray);
        if (!Array.isArray(startTranslation) || !Array.isArray(finalTranslation)) return null;
        const actualDelta = [
            (finalTranslation[0] || 0) - (startTranslation[0] || 0),
            (finalTranslation[1] || 0) - (startTranslation[1] || 0),
            (finalTranslation[2] || 0) - (startTranslation[2] || 0)
        ];
        const cfg = this.vfiConfig || {};
        return evaluateTranslationAxisGuard({
            commandDelta,
            actualDelta,
            minProgressRatio: Number.isFinite(cfg.jogTranslationAxisMinProgressRatio)
                ? Math.max(0, cfg.jogTranslationAxisMinProgressRatio)
                : 0.05,
            maxOrthogonalRatio: Number.isFinite(cfg.jogTranslationAxisMaxOrthogonalRatio)
                ? Math.max(0, cfg.jogTranslationAxisMaxOrthogonalRatio)
                : 0.35,
            orthogonalRatioBasis: options?.orthogonalRatioBasis === 'progress' ? 'progress' : 'command'
        });
    }

    _evaluateHumanoidWaistXPosition(qFull) {
        const DQ = window.DQModule?.DQWrapper;
        if (
            !DQ ||
            !this.humanoidModel ||
            typeof this.humanoidModel.fkmEffector !== 'function' ||
            !Array.isArray(qFull)
        ) {
            return Number.NaN;
        }
        const waistPose = this.humanoidModel.fkmEffector(qFull, 0);
        if (!waistPose) return Number.NaN;
        const translation = DQ.getTranslation(DQ.createFromArray(waistPose));
        return Number(translation?.[1] || 0);
    }

    _getHumanoidAbsoluteJogTargetArray(qCurrent, { axis, direction, type = 'translation' } = {}) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !Array.isArray(qCurrent) || qCurrent.length === 0) return null;
        const dqAbsArray = this._getHumanoidAbsolutePose(qCurrent);
        if (!dqAbsArray) return null;
        const dqAbsCurrent = DQ.createFromArray(dqAbsArray);

        if (type === 'rotation') {
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = this.getTeachRotationAngle() * directionSign;
            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return null;
            return DQ.toArray(this.combineDQWithTeachOrder(rotationDQ, dqAbsCurrent, false));
        }

        const rotationArray = DQ.getRotation(dqAbsCurrent);
        const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
        const directionSign = direction === 'positive' ? 1 : -1;
        const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(this.getTeachTranslationStep() * directionSign);
        const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
        return DQ.toArray(DQ.multiply(dqDelta, dqAbsCurrent));
    }

    _publishHumanoidJogDiagnostics(payload = {}) {
        if (typeof document === 'undefined') return;
        const panel = document.getElementById('humanoid-vfi-diagnostics-panel');
        const textEl = document.getElementById('humanoid-vfi-diagnostics-text');
        const tsEl = document.getElementById('humanoid-vfi-diagnostics-timestamp');
        if (!textEl || !tsEl) return;

        const now = new Date();
        const ms = String(now.getMilliseconds()).padStart(3, '0');
        tsEl.textContent = `${now.toLocaleTimeString('zh-CN', { hour12: false })}.${ms}`;

        const solver = normalizeHumanoidJogSolverMode(
            payload.solverMode || this._getHumanoidJogSolverMode()
        );
        const solverDisplay = getHumanoidJogSolverDisplayName(solver);
        const modeTag = payload.modeTag || 'idle';
        const active = Boolean(payload.active ?? true);
        if (panel) panel.classList.toggle('inactive', !active);

        const lines = [
            `solver: ${solverDisplay}    mode: ${modeTag}`,
            `status: ${payload.message || (active ? 'running' : 'inactive')}`
        ];
        if (typeof payload.nextMode === 'string') {
            lines.push(`next_mode: ${payload.nextMode}`);
        }

        if (Number.isFinite(payload.iterations)) {
            lines.push(`iters: ${payload.iterations}    active_joints: ${payload.activeJoints ?? 'n/a'}`);
        }
        if (Number.isFinite(payload.commandRows) || Number.isFinite(payload.invariantRows) || Number.isFinite(payload.constraintRows)) {
            lines.push(
                `rows(cmd/inv/all): ${payload.commandRows ?? 'n/a'} / ${payload.invariantRows ?? 'n/a'} / ${payload.constraintRows ?? 'n/a'}`
            );
        }
        if (
            Number.isFinite(payload.invariantConstraintRows) ||
            Number.isFinite(payload.componentRows) ||
            Number.isFinite(payload.jointLimitRows)
        ) {
            lines.push(
                `rows(invBand/box/joint): ${payload.invariantConstraintRows ?? 'n/a'} / ${payload.componentRows ?? 'n/a'} / ${payload.jointLimitRows ?? 'n/a'}`
            );
        }
        if (Number.isFinite(payload.equalityRows)) {
            lines.push(`rows(eq): ${payload.equalityRows}`);
        }
        if (Number.isFinite(payload.commandError) || Number.isFinite(payload.invariantError) || Number.isFinite(payload.stepNorm)) {
            lines.push(
                `err(cmd/inv): ${this._formatDiagNumber(payload.commandError)} / ${this._formatDiagNumber(payload.invariantError)}    step: ${this._formatDiagNumber(payload.stepNorm)}`
            );
        }
        if (
            Number.isFinite(payload.cmdTargetNorm) ||
            Number.isFinite(payload.cmdAchievedNorm) ||
            Number.isFinite(payload.cmdResidualNorm) ||
            Number.isFinite(payload.cmdAlignment)
        ) {
            lines.push(
                `cmd_fit(target/achieved/res/cos): ${this._formatDiagNumber(payload.cmdTargetNorm)} / ${this._formatDiagNumber(payload.cmdAchievedNorm)} / ${this._formatDiagNumber(payload.cmdResidualNorm)} / ${this._formatDiagNumber(payload.cmdAlignment)}`
            );
        }
        if (Number.isFinite(payload.cmdTargetFirst) || Number.isFinite(payload.cmdAchievedFirst)) {
            lines.push(
                `cmd_first(target/achieved): ${this._formatDiagNumber(payload.cmdTargetFirst)} / ${this._formatDiagNumber(payload.cmdAchievedFirst)}`
            );
        }
        if (Number.isFinite(payload.maxViolationBefore) || Number.isFinite(payload.maxViolationAfter)) {
            lines.push(
                `max_violation(before/after): ${this._formatDiagNumber(payload.maxViolationBefore)} / ${this._formatDiagNumber(payload.maxViolationAfter)}`
            );
        }
        if (Number.isFinite(payload.qpIterations) || typeof payload.qpConverged === 'boolean' || Number.isFinite(payload.qpObjective)) {
            lines.push(
                `qp(iters/converged/obj): ${payload.qpIterations ?? 'n/a'} / ${typeof payload.qpConverged === 'boolean' ? (payload.qpConverged ? 'yes' : 'no') : 'n/a'} / ${this._formatDiagNumber(payload.qpObjective)}`
            );
        }
        if (typeof payload.qpMessage === 'string' && payload.qpMessage.trim().length > 0) {
            lines.push(`qp_message: ${payload.qpMessage.trim()}`);
        }
        const optimizationLabel = typeof payload.optimizationLabel === 'string' && payload.optimizationLabel.trim().length > 0
            ? payload.optimizationLabel.trim()
            : (typeof payload.wvfiOptimizationLabel === 'string' && payload.wvfiOptimizationLabel.trim().length > 0
                ? payload.wvfiOptimizationLabel.trim()
                : '');
        if (optimizationLabel) {
            lines.push(optimizationLabel);
        }
        if (typeof payload.jogMetric === 'string' && payload.jogMetric.trim().length > 0) {
            const axisPart = typeof payload.jogMetricAxis === 'string' && payload.jogMetricAxis.trim().length > 0
                ? `  axis=${payload.jogMetricAxis.trim()}`
                : '';
            lines.push(
                `jog_metric: ${payload.jogMetric.trim()}  ${payload.jogMetricState || 'n/a'}${axisPart}  before=${this._formatDiagNumber(payload.jogMetricBefore)}  after=${this._formatDiagNumber(payload.jogMetricAfter)}  applied=${payload.jogMetricApplied ? 'yes' : 'no'}`
            );
        }
        if (Number.isFinite(payload.hierarchicalVfiBlend)) {
            lines.push(`hier_vfi_blend: alpha=${this._formatDiagNumber(payload.hierarchicalVfiBlend)}`);
        }
        if (
            Number.isFinite(payload.timingPreMs) ||
            Number.isFinite(payload.timingWorkerInitMs) ||
            Number.isFinite(payload.timingSolveWaitMs) ||
            Number.isFinite(payload.timingSolveMs) ||
            Number.isFinite(payload.timingPostMs) ||
            Number.isFinite(payload.timingCCTSMMs) ||
            Number.isFinite(payload.timingDiagnosticMs) ||
            Number.isFinite(payload.timingTotalMs)
        ) {
            lines.push(
                `timing_ms(pre/init/wait/solve/post/cctsm/diag/total): ${this._formatDiagNumber(payload.timingPreMs)} / ${this._formatDiagNumber(payload.timingWorkerInitMs)} / ${this._formatDiagNumber(payload.timingSolveWaitMs)} / ${this._formatDiagNumber(payload.timingSolveMs)} / ${this._formatDiagNumber(payload.timingPostMs)} / ${this._formatDiagNumber(payload.timingCCTSMMs)} / ${this._formatDiagNumber(payload.timingDiagnosticMs)} / ${this._formatDiagNumber(payload.timingTotalMs)}`
            );
        }
        if (typeof payload.diagnosticLabel === 'string') {
            lines.push(`diagnostic: ${payload.diagnosticLabel}`);
        }
        if (Number.isFinite(payload.feasibleHorizon) || Number.isFinite(payload.targetHorizon) || Number.isFinite(payload.minReserve)) {
            lines.push(
                `certificate(h/target/mu): ${payload.feasibleHorizon ?? 'n/a'} / ${payload.targetHorizon ?? 'n/a'} / ${this._formatDiagNumber(payload.minReserve)}`
            );
        }
        if (Number.isFinite(payload.readiness)) {
            lines.push(`readiness: ${this._formatDiagNumber(payload.readiness)}`);
        }
        if (Array.isArray(payload.degradedFlags) && payload.degradedFlags.length > 0) {
            lines.push(`degraded: ${payload.degradedFlags.join(', ')}`);
        }
        if (typeof payload.includeWaist === 'boolean' || typeof payload.keepBodyZVertical === 'boolean' || typeof payload.jointLimitAvoidance === 'boolean') {
            lines.push(
                `flags: waist=${payload.includeWaist ? 'on' : 'off'}  body_z=${payload.keepBodyZVertical ? 'on' : 'off'}  limit=${payload.jointLimitAvoidance ? 'on' : 'off'}`
            );
        }
        // CCTSM diagnostics (Section IV)
        if (Number.isFinite(payload.cctsmW) || Number.isFinite(payload.cctsmWvfi)) {
            lines.push(
                `CCTSM: w_combined=${this._formatDiagNumber(payload.cctsmW)}  w_vfi=${this._formatDiagNumber(payload.cctsmWvfi)}`
            );
        }
        if (Number.isFinite(payload.cctsmWAbs)) {
            lines.push(`CCTSM_abs: w_abs=${this._formatDiagNumber(payload.cctsmWAbs)}`);
        }
        if (Number.isFinite(payload.cctsmWAbsHier)) {
            lines.push(`CCTSM_abs_hier: w_abs_hier=${this._formatDiagNumber(payload.cctsmWAbsHier)}`);
        }
        if (Number.isFinite(payload.cctsmWAbsHierVfi)) {
            lines.push(`CCTSM_abs_hier_vfi: w_abs_hier_vfi=${this._formatDiagNumber(payload.cctsmWAbsHierVfi)}`);
        }
        if (Number.isFinite(payload.cctsmWAbsArm)) {
            lines.push(`CCTSM_abs_arm: w_abs_arm=${this._formatDiagNumber(payload.cctsmWAbsArm)}`);
        }
        if (Number.isFinite(payload.cctsmWAbsArmVfi)) {
            lines.push(`CCTSM_abs_arm_vfi: w_abs_arm_vfi=${this._formatDiagNumber(payload.cctsmWAbsArmVfi)}`);
        }
        if (Number.isFinite(payload.cctsmWAbsVfi)) {
            lines.push(`CCTSM_abs_vfi: w_abs_vfi=${this._formatDiagNumber(payload.cctsmWAbsVfi)}`);
        }
        if (Number.isFinite(payload.cctsmWCmd)) {
            lines.push(`CCTSM_cmd: w_cmd=${this._formatDiagNumber(payload.cctsmWCmd)}`);
        }
        if (
            Number.isFinite(payload.vfiJointLimitCount) ||
            Number.isFinite(payload.vfiExtraCount) ||
            Number.isFinite(payload.vfiTotalCount) ||
            Number.isFinite(payload.vfiMinDistance)
        ) {
            lines.push(
                `VFI_rows(joint/extra/total,dmin): ${Number.isFinite(payload.vfiJointLimitCount) ? payload.vfiJointLimitCount : 'n/a'} / ${Number.isFinite(payload.vfiExtraCount) ? payload.vfiExtraCount : 'n/a'} / ${Number.isFinite(payload.vfiTotalCount) ? payload.vfiTotalCount : 'n/a'} / ${this._formatDiagNumber(payload.vfiMinDistance)}`
            );
        }
        if (payload.wvfiFloorEnabled === true) {
            lines.push(
                `wvfi_floor: value=${this._formatDiagNumber(payload.wvfiFloorValue)}  min=${this._formatDiagNumber(payload.wvfiFloorLowerBound)}  slack=${this._formatDiagNumber(payload.wvfiFloorSlack)}  viol=${this._formatDiagNumber(payload.wvfiFloorViolation)}`
            );
        } else if (
            payload.solverMode === 'mj_restore' &&
            (
                Number.isFinite(payload.cctsmWvfi) ||
                Number.isFinite(payload.vfiTotalCount)
            )
        ) {
            lines.push('wvfi_floor: disabled');
        }
        if (
            Number.isFinite(payload.restoreAcceptedIterations) ||
            Number.isFinite(payload.restoreResidualNorm) ||
            Number.isFinite(payload.restoreScale)
        ) {
            lines.push(
                `restore(acc/res/scale): ${Number.isFinite(payload.restoreAcceptedIterations) ? payload.restoreAcceptedIterations : 'n/a'} / ${this._formatDiagNumber(payload.restoreResidualNorm)} / ${this._formatDiagNumber(payload.restoreScale)}`
            );
        }
        // Predictive certification diagnostics (Section V)
        if (typeof payload.certLabel === 'string') {
            const hStr = Number.isFinite(payload.certH) ? `h=${payload.certH}` : '';
            const muStr = Number.isFinite(payload.certMu) ? `μ=${this._formatDiagNumber(payload.certMu)}` : '';
            const sigStr = Number.isFinite(payload.certSigma) ? `σ=${this._formatDiagNumber(payload.certSigma)}` : '';
            lines.push(`cert: ${payload.certLabel}  ${hStr}  ${muStr}  ${sigStr}`);
        }
        // Waist x-corridor diagnostics
        if (payload.waistXEnabled === false && (
            payload.includeWaist === true ||
            Number.isFinite(payload.vfiExtraCount) ||
            Number.isFinite(payload.waistXViolation)
        )) {
            lines.push('waist_x: disabled');
        } else if (
            Number.isFinite(payload.waistXPx) ||
            Number.isFinite(payload.waistXDupper) ||
            Number.isFinite(payload.waistXDlower) ||
            Number.isFinite(payload.waistXViolation)
        ) {
            const xMax = this.vfiConfig?.waistXMax ?? 0.05;
            lines.push(
                `waist_x: p0x=${this._formatDiagNumber(payload.waistXPx)}m  d+=${this._formatDiagNumber(payload.waistXDupper)}m  d-=${this._formatDiagNumber(payload.waistXDlower)}m  viol=${this._formatDiagNumber(payload.waistXViolation)}m  limit=${xMax.toFixed(3)}m`
            );
        }
        this.lastHumanoidJogDiagnostics = {
            ...payload,
            renderedLines: [...lines]
        };
        textEl.textContent = lines.join('\n');
    }

    _extractHumanoidWaistXCorridorDiagnostics(waistXCorridor = null) {
        const enabled = waistXCorridor?.enabled === true;
        const value = Number.isFinite(waistXCorridor?.value) ? waistXCorridor.value : Number.NaN;
        const min = Number.isFinite(waistXCorridor?.min) ? waistXCorridor.min : Number.NaN;
        const max = Number.isFinite(waistXCorridor?.max) ? waistXCorridor.max : Number.NaN;
        return {
            waistXEnabled: enabled,
            waistXPx: enabled ? value : Number.NaN,
            waistXDupper: enabled && Number.isFinite(value) && Number.isFinite(max) ? (max - value) : Number.NaN,
            waistXDlower: enabled && Number.isFinite(value) && Number.isFinite(min) ? (value - min) : Number.NaN,
            waistXViolation: enabled && Number.isFinite(waistXCorridor?.violation) ? waistXCorridor.violation : Number.NaN
        };
    }

    _extractHumanoidWVFIConstraintDiagnostics(wvfiFloor = null) {
        const enabled = wvfiFloor?.enabled === true;
        return {
            wvfiFloorEnabled: enabled,
            wvfiFloorValue: enabled && Number.isFinite(wvfiFloor?.value) ? wvfiFloor.value : Number.NaN,
            wvfiFloorLowerBound: enabled && Number.isFinite(wvfiFloor?.lowerBound) ? wvfiFloor.lowerBound : Number.NaN,
            wvfiFloorSlack: enabled && Number.isFinite(wvfiFloor?.slack) ? wvfiFloor.slack : Number.NaN,
            wvfiFloorViolation: enabled && Number.isFinite(wvfiFloor?.violation) ? wvfiFloor.violation : Number.NaN
        };
    }

    _buildPriorityCurrentCCTSMPayload({
        solverMode = 'priority',
        modeTag,
        mode,
        q,
        armStart,
        activeCount,
        activeLimits = null,
        includeWaist = false,
        keepBodyZVertical = false,
        jointLimitAvoidance = true,
            message = `当前为${getHumanoidJogSolverDisplayName(solverMode)}求解器，显示当前 CCTSM。`
    } = {}) {
        const cctsmResult = Array.isArray(q) && Number.isFinite(armStart) && Number.isFinite(activeCount)
            ? this._computeModeCCTSM(
                q,
                armStart,
                activeCount,
                mode,
                this._buildManipulabilityVFIConstraints(q, armStart, activeCount, activeLimits)
            )
            : null;

        return buildCurrentCCTSMPayload({
            solverMode,
            modeTag,
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance,
            message,
            cctsmResult
        });
    }

    _publishCurrentPriorityCCTSMDiagnostics(options = {}) {
        this._publishHumanoidJogDiagnostics(this._buildPriorityCurrentCCTSMPayload(options));
    }

    _publishHumanoidJogInputFailure({
        modeTag = 'absolute-target',
        includeWaist = false,
        keepBodyZVertical = false,
        message = '点动输入失败。',
        reason = 'input_failure'
    } = {}) {
        const solverMode = this._getHumanoidJogSolverMode();
        this._publishHumanoidJogDiagnostics({
            solverMode,
            modeTag,
            active: true,
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true,
            message,
            diagnosticLabel: reason
        });
        this._setIKFailureStatus({
            solver: solverMode,
            mode: `humanoid_${modeTag}`,
            reason,
            message
        });
        return false;
    }

    _setIKFailureStatus(status = {}) {
        this.lastIKSolveStatus = {
            converged: false,
            finalResidual: Number.POSITIVE_INFINITY,
            ...status
        };
        return false;
    }

    _solveVFICommandStep({
        Jcmd,
        eCmd,
        Jinvariant,
        eInvariant,
        qActive,
        activeLimits = null,
        maxStep = 0.003,
        damping = 1e-3,
        forceInvariantEquality = null,
        invariantKappaOverride = null,
        invariantBandBaseOverride = null,
        invariantBandGainOverride = null,
        invariantSoftWeightOverride = null,
        extraVFIRows = null,  // { rows: number[][], rhs: number[] } for additional VFI constraints
        jointRegWeights = null, // per-joint absolute regularization (length n); defaults to lambda for all
        jointMaxSteps = null,   // per-joint step-size hard bounds (length n); defaults to maxStep for all
        nullManipGrad = null,   // null(J_cmd)-projected ∇w_vfi for secondary manipulability task
        allowSecondaryFallback = true,
        wvfiGuard = null
    }) {
        if (!Array.isArray(Jcmd) || Jcmd.length === 0 || !Array.isArray(Jcmd[0])) return null;
        const n = Jcmd[0].length;
        if (n <= 0) return null;

        const cfg = this.vfiConfig || {};
        const kappaCmd = Number.isFinite(cfg.kappaCommand) ? cfg.kappaCommand : 0.6;
        const kappaInv = Number.isFinite(invariantKappaOverride)
            ? Math.max(0, invariantKappaOverride)
            : (Number.isFinite(cfg.kappaInvariant) ? cfg.kappaInvariant : 0.9);
        const bandBase = Number.isFinite(invariantBandBaseOverride)
            ? Math.max(0, invariantBandBaseOverride)
            : (Number.isFinite(cfg.invariantBandBase) ? cfg.invariantBandBase : 1e-4);
        const bandGain = Number.isFinite(invariantBandGainOverride)
            ? Math.max(0, invariantBandGainOverride)
            : (Number.isFinite(cfg.invariantBandGain) ? cfg.invariantBandGain : 0.15);
        const useInvariantEqualityInQP = this._shouldUseInvariantEqualityForStep(forceInvariantEquality);
        const eqTol = Number.isFinite(cfg.qpEqualityTolerance) ? cfg.qpEqualityTolerance : 1e-12;
        const eta = Number.isFinite(cfg.jointLimitEta) ? cfg.jointLimitEta : 1.0;
        const cmdTarget = Array.isArray(eCmd) ? eCmd.map((v) => kappaCmd * (v || 0)) : [];
        const invTarget = Array.isArray(eInvariant) ? eInvariant.map((v) => kappaInv * (v || 0)) : [];
        const invariantSoftWeight = Number.isFinite(invariantSoftWeightOverride)
            ? Math.max(0, invariantSoftWeightOverride)
            : 0;

        const Jpinv = this._dampedPseudoInverse(Jcmd, damping);
        let dqSeed = new Array(n).fill(0);
        if (Jpinv && cmdTarget.length > 0) {
            dqSeed = this._matVecProduct(Jpinv, cmdTarget);
        }

        const Wineq = [];
        const bineq = [];
        const Weq = [];
        const beq = [];
        const invariantRows = Array.isArray(Jinvariant) ? Jinvariant.length : 0;
        if (Array.isArray(Jinvariant) && Jinvariant.length > 0 && Array.isArray(eInvariant) && eInvariant.length > 0) {
            if (useInvariantEqualityInQP) {
                for (let i = 0; i < Jinvariant.length; i++) {
                    const row = Array.isArray(Jinvariant[i]) ? Jinvariant[i] : null;
                    if (!Array.isArray(row) || row.length === 0) continue;
                    Weq.push([...row]);
                    beq.push(invTarget[i] || 0);
                }
            } else {
                // VFI-style inequality tube around invariants (aligned with velocity-damper formulation).
                appendVFIErrorTubeConstraints(Wineq, bineq, Jinvariant, eInvariant, {
                    eta: kappaInv,
                    marginBase: bandBase,
                    marginGain: bandGain,
                    zeta: 0
                });
            }
        }

        const rowsAfterInvariant = Wineq.length;
        // Per-joint step bounds: waist joints may have a tighter limit than arms
        if (Array.isArray(jointMaxSteps) && jointMaxSteps.length >= n) {
            for (let i = 0; i < n; i++) {
                const s = Number.isFinite(jointMaxSteps[i]) ? Math.max(0, jointMaxSteps[i]) : maxStep;
                const rowP = new Array(n).fill(0); rowP[i] = 1;
                const rowN = new Array(n).fill(0); rowN[i] = -1;
                Wineq.push(rowP); bineq.push(s);
                Wineq.push(rowN); bineq.push(s);
            }
        } else {
            appendComponentBounds(Wineq, bineq, n, -maxStep, maxStep);
        }
        const componentRows = Wineq.length - rowsAfterInvariant;
        const rowsAfterBox = Wineq.length;
        if (Array.isArray(activeLimits) && Array.isArray(qActive) && qActive.length === n) {
            appendJointLimitStepConstraints(Wineq, bineq, qActive, activeLimits, { eta, maxStep });
        }
        const jointLimitRows = Wineq.length - rowsAfterBox;
        // Extra VFI constraints (e.g. waist x-corridor, eq. 861–866)
        if (extraVFIRows && Array.isArray(extraVFIRows.rows) && Array.isArray(extraVFIRows.rhs)) {
            for (let i = 0; i < extraVFIRows.rows.length; i++) {
                Wineq.push([...(extraVFIRows.rows[i] || [])]);
                bineq.push(extraVFIRows.rhs[i] || 0);
            }
        }

        const Abase = Wineq.map((row) => [...row]);
        const bbase = [...bineq];
        let Aactive = Abase;
        let bactive = bbase;

        const maxViolation = (Aset, bset, x) => {
            let maxV = 0;
            for (let i = 0; i < Aset.length; i++) {
                const row = Aset[i];
                if (!Array.isArray(row)) continue;
                let lhs = 0;
                const m = Math.min(row.length, x.length);
                for (let j = 0; j < m; j++) lhs += (row[j] || 0) * (x[j] || 0);
                const vio = lhs - (bset[i] || 0);
                if (vio > maxV) maxV = vio;
            }
            return maxV;
        };

        let dq = dqSeed;
        let qpIterations = null;
        let qpConverged = null;
        let qpObjective = null;
        let qpMessage = null;
        let equalityRowsApplied = 0;

        const addLeastSquaresTerm = (H, f, J, y, weight = 1) => {
            if (!Array.isArray(J) || J.length === 0) return;
            const w = Number.isFinite(weight) ? Math.max(0, weight) : 1;
            if (w <= 0) return;
            for (let k = 0; k < J.length; k++) {
                const row = Array.isArray(J[k]) ? J[k] : [];
                const yk = y?.[k] || 0;
                for (let i = 0; i < n; i++) {
                    const Ji = row[i] || 0;
                    if (!Ji) continue;
                    f[i] -= w * Ji * yk;
                    for (let j = 0; j < n; j++) {
                        H[i][j] += w * Ji * (row[j] || 0);
                    }
                }
            }
        };

        const lambda = Number.isFinite(cfg.qpRegularization) ? cfg.qpRegularization : 1e-6;
        const qpSolveOptions = buildQPNumericSolveOptions(cfg);

        const H = new Array(n).fill(0).map(() => new Array(n).fill(0));
        const f = new Array(n).fill(0);
        addLeastSquaresTerm(H, f, Jcmd, cmdTarget, 1.0);
        for (let i = 0; i < n; i++) {
            // jointRegWeights[i]: absolute diagonal regularization per joint.
            // Defaults to lambda. Higher values discourage motion for that joint.
            H[i][i] += (Array.isArray(jointRegWeights) && Number.isFinite(jointRegWeights[i]))
                ? Math.max(0, jointRegWeights[i]) : lambda;
        }
        if (
            !useInvariantEqualityInQP &&
            invariantSoftWeight > 0 &&
            Array.isArray(Jinvariant) &&
            Jinvariant.length > 0 &&
            invTarget.length > 0
        ) {
            addLeastSquaresTerm(H, f, Jinvariant, invTarget, invariantSoftWeight);
        }
        // Secondary task: maximise w_vfi in null(J_cmd).
        // nullManipGrad is already projected into null(J_cmd) by the caller,
        // so subtracting it from f pushes dq toward higher manipulability
        // without disturbing the primary task (quadratic term H is unchanged).
        const manipGradW = Number.isFinite(cfg.manipGradWeight) ? cfg.manipGradWeight : 1.0;
        if (manipGradW > 0 && Array.isArray(nullManipGrad)) {
            for (let i = 0; i < n; i++) {
                f[i] -= manipGradW * (nullManipGrad[i] || 0);
            }
        }

        const solveOnce = (Aset, bset, Hset, fset) => {
            Aactive = Aset;
            bactive = bset;
            return solveQPNumericActiveSet(Hset, fset, Aset, bset, dqSeed, qpSolveOptions);
        };

        let qpResult = null;
        let relaxedEqTolUsed = null;
        let usedSoftInvariantFallback = false;
        if (useInvariantEqualityInQP && Weq.length > 0) {
            const eqTolCandidates = buildEqualityToleranceCandidates(eqTol);
            for (let idx = 0; idx < eqTolCandidates.length; idx++) {
                const tol = eqTolCandidates[idx];
                const Atrial = Abase.map((row) => [...row]);
                const btrial = [...bbase];
                appendEqualityConstraints(Atrial, btrial, Weq, beq, { tolerance: tol });
                qpResult = solveOnce(Atrial, btrial, H, f);
                equalityRowsApplied = Weq.length;
                relaxedEqTolUsed = tol;
                if (qpResult?.converged) break;
                const msg = String(qpResult?.solverMessage || '').toLowerCase();
                const infeasible = msg.includes('inconsistent') || msg.includes('no solution');
                if (!infeasible) break;
            }

            if (Number.isFinite(relaxedEqTolUsed) && relaxedEqTolUsed > eqTol + 1e-16) {
                const extra = `eq_tol_relaxed=${relaxedEqTolUsed.toExponential(1)}`;
                qpMessage = qpMessage ? `${qpMessage}; ${extra}` : extra;
            }
        } else {
            qpResult = solveOnce(Abase, bbase, H, f);
        }

        if (!qpResult) {
            qpResult = solveOnce(Abase, bbase, H, f);
        }

        dq = Array.isArray(qpResult?.x) ? qpResult.x : dqSeed;
        qpIterations = qpResult?.iterations ?? null;
        qpConverged = Boolean(qpResult?.converged);
        qpObjective = Number.isFinite(qpResult?.objective) ? qpResult.objective : null;
        const baseMessage = (typeof qpResult?.solverMessage === 'string' && qpResult.solverMessage.trim().length > 0)
            ? qpResult.solverMessage.trim()
            : null;
        if (baseMessage) {
            qpMessage = qpMessage ? `${baseMessage}; ${qpMessage}` : baseMessage;
        }

        const maxViolationBefore = Aactive.length > 0 ? maxViolation(Aactive, bactive, dqSeed) : 0;
        // Hard post-solve clamp: enforce per-joint limits regardless of QP quality.
        // This prevents limit violations when the QP solver produces a slightly infeasible solution.
        for (let i = 0; i < n; i++) {
            let upBound = maxStep;
            let dnBound = maxStep;
            if (Array.isArray(activeLimits) && Array.isArray(qActive) && qActive.length === n) {
                const lim = activeLimits[i];
                if (lim) {
                    const qi = qActive[i] || 0;
                    if (Number.isFinite(lim.upper)) upBound = Math.min(maxStep, Math.max(0, lim.upper - qi));
                    if (Number.isFinite(lim.lower)) dnBound = Math.min(maxStep, Math.max(0, qi - lim.lower));
                }
            }
            dq[i] = Math.max(-dnBound, Math.min(upBound, dq[i] || 0));
        }
        let wvfiGuardAccepted = null;
        let wvfiGuardScale = Number.NaN;
        let wvfiGuardValue = Number.NaN;
        let wvfiGuardLowerBound = Number.NaN;
        let wvfiGuardReason = null;
        if (wvfiGuard && typeof wvfiGuard.apply === 'function') {
            const guardResult = wvfiGuard.apply(dq);
            wvfiGuardAccepted = guardResult?.accepted === true;
            wvfiGuardScale = Number.isFinite(guardResult?.scale) ? guardResult.scale : Number.NaN;
            wvfiGuardValue = Number.isFinite(guardResult?.wvfi) ? guardResult.wvfi : Number.NaN;
            wvfiGuardLowerBound = Number.isFinite(guardResult?.lowerBound)
                ? guardResult.lowerBound
                : (Number.isFinite(wvfiGuard.lowerBound) ? wvfiGuard.lowerBound : Number.NaN);
            wvfiGuardReason = typeof guardResult?.reason === 'string' ? guardResult.reason : null;
            if (wvfiGuardAccepted && Array.isArray(guardResult?.dq)) {
                dq = guardResult.dq;
            } else {
                dq = new Array(n).fill(0);
            }
            if (wvfiGuardReason) {
                const lowerStr = Number.isFinite(wvfiGuardLowerBound) ? wvfiGuardLowerBound.toExponential(1) : 'n/a';
                const valueStr = Number.isFinite(wvfiGuardValue) ? wvfiGuardValue.toExponential(1) : 'n/a';
                const scaleStr = Number.isFinite(wvfiGuardScale) ? wvfiGuardScale.toExponential(1) : 'n/a';
                const extra = `wvfi_guard=${wvfiGuardReason}(scale=${scaleStr},w=${valueStr},min=${lowerStr})`;
                qpMessage = qpMessage ? `${qpMessage}; ${extra}` : extra;
            }
        }
        const cmdAchieved = this._matVecProduct(Jcmd, dq);
        const cmdTargetEval = Array.isArray(cmdTarget) ? cmdTarget : [];
        let cmdTargetNorm2 = 0;
        let cmdAchievedNorm2 = 0;
        let cmdResidualNorm2 = 0;
        let cmdDot = 0;
        const cmdN = Math.min(cmdTargetEval.length, cmdAchieved.length);
        for (let i = 0; i < cmdN; i++) {
            const t = cmdTargetEval[i] || 0;
            const a = cmdAchieved[i] || 0;
            const r = a - t;
            cmdTargetNorm2 += t * t;
            cmdAchievedNorm2 += a * a;
            cmdResidualNorm2 += r * r;
            cmdDot += t * a;
        }
        const cmdTargetNorm = Math.sqrt(cmdTargetNorm2);
        const cmdAchievedNorm = Math.sqrt(cmdAchievedNorm2);
        const cmdResidualNorm = Math.sqrt(cmdResidualNorm2);
        const cmdAlignment = (cmdTargetNorm > 0 && cmdAchievedNorm > 0)
            ? (cmdDot / (cmdTargetNorm * cmdAchievedNorm))
            : Number.NaN;
        let stepNorm2 = 0;
        for (let i = 0; i < dq.length; i++) stepNorm2 += (dq[i] || 0) * (dq[i] || 0);
        const maxViolationAfter = Aactive.length > 0 ? maxViolation(Aactive, bactive, dq) : 0;
        if (qpConverged === false) {
            const feasibleAcceptanceTol = Number.isFinite(cfg.qpFeasibleAcceptanceTol)
                ? Math.max(0, cfg.qpFeasibleAcceptanceTol)
                : 1e-8;
            if (maxViolationAfter <= feasibleAcceptanceTol) {
                qpConverged = true;
                const extra = `accepted_feasible_warning(tol=${feasibleAcceptanceTol.toExponential(1)})`;
                qpMessage = qpMessage ? `${qpMessage}; ${extra}` : extra;
            }
        }

        const hasSecondaryObjective = Array.isArray(nullManipGrad)
            && nullManipGrad.some((value) => Math.abs(value || 0) > 1e-10);
        const equalityToleranceRelaxed = (
            useInvariantEqualityInQP &&
            Number.isFinite(relaxedEqTolUsed) &&
            relaxedEqTolUsed > eqTol + 1e-16
        );
        const minCommandAlignment = Number.isFinite(cfg.qpSecondaryFallbackMinAlignment)
            ? cfg.qpSecondaryFallbackMinAlignment
            : 0;
        const shouldRetryForEqualityProtection = shouldRetryWithoutSecondaryObjective({
            useInvariantEqualityInQP,
            hasSecondaryObjective,
            qpConverged,
            equalityToleranceRelaxed,
            usedSoftInvariantFallback
        });
        const shouldFallbackWithoutSecondary = (
            allowSecondaryFallback &&
            hasSecondaryObjective &&
            (
                shouldRetryForEqualityProtection ||
                (Number.isFinite(cmdAlignment) && cmdAlignment < minCommandAlignment)
            )
        );
        if (shouldFallbackWithoutSecondary) {
            const fallbackResult = this._solveVFICommandStep({
                Jcmd,
                eCmd,
                Jinvariant,
                eInvariant,
                qActive,
                activeLimits,
                maxStep,
                damping,
                forceInvariantEquality,
                invariantKappaOverride,
                invariantBandBaseOverride,
                invariantBandGainOverride,
                invariantSoftWeightOverride,
                extraVFIRows,
                jointRegWeights,
                jointMaxSteps,
                nullManipGrad: null,
                allowSecondaryFallback: false,
                wvfiGuard
            });
            if (fallbackResult?.stats) {
                let reason = qpConverged === false ? 'solver' : `cmd_alignment=${cmdAlignment.toFixed(3)}`;
                if (shouldRetryForEqualityProtection) {
                    if (usedSoftInvariantFallback && equalityToleranceRelaxed) reason = 'eq_protection(soft_invariant+relaxed_tol)';
                    else if (usedSoftInvariantFallback) reason = 'eq_protection(soft_invariant)';
                    else if (equalityToleranceRelaxed) reason = 'eq_protection(relaxed_tol)';
                    else if (qpConverged === false) reason = 'eq_protection(solver)';
                }
                const extra = `secondary_fallback=disabled_manip_grad(${reason})`;
                fallbackResult.stats.qpMessage = fallbackResult.stats.qpMessage
                    ? `${fallbackResult.stats.qpMessage}; ${extra}`
                    : extra;
            }
            return fallbackResult;
        }

        return {
            dq,
            stats: {
                commandRows: Jcmd.length,
                invariantRows,
                invariantConstraintRows: rowsAfterInvariant,
                equalityRows: equalityRowsApplied,
                componentRows,
                jointLimitRows,
                constraintRows: Aactive.length,
                maxViolationBefore,
                maxViolationAfter,
                stepNorm: Math.sqrt(stepNorm2),
                cmdTargetNorm,
                cmdAchievedNorm,
                cmdResidualNorm,
                cmdAlignment,
                cmdTargetFirst: cmdTargetEval.length > 0 ? cmdTargetEval[0] : Number.NaN,
                cmdAchievedFirst: cmdAchieved.length > 0 ? cmdAchieved[0] : Number.NaN,
                qpIterations,
                qpConverged,
                qpObjective,
                qpMessage,
                wvfiGuardAccepted,
                wvfiGuardScale,
                wvfiGuardValue,
                wvfiGuardLowerBound,
                wvfiGuardReason
            }
        };
    }

    /**
     * 根据轴名称返回单位基向量
     * @param {string} axis - 'x' | 'y' | 'z'
     * @returns {THREE.Vector3}
     */
    getAxisBasisVector(axis) {
        switch (axis) {
            case 'x':
                return new THREE.Vector3(1, 0, 0);
            case 'y':
                return new THREE.Vector3(0, 1, 0);
            case 'z':
                return new THREE.Vector3(0, 0, 1);
            default:
                throw new Error(`无效的轴名称: ${axis}`);
        }
    }

    /**
     * 获取平移方向（世界坐标系向量），根据当前模式决定是否使用末端旋转
     * @param {string} axis - 'x' | 'y' | 'z'
     * @param {THREE.Quaternion|null} referenceQuaternion - 末端或参考坐标系四元数
     * @returns {THREE.Vector3}
     */
    getTranslationDirection(axis, referenceQuaternion = null) {
        const axisVec = this.getAxisBasisVector(axis);
        if (!this.isWorldTranslationMode() && referenceQuaternion) {
            axisVec.applyQuaternion(referenceQuaternion);
        }
        return axisVec;
    }
    
    /**
     * 创建机器人模型
     */
    async createRobot() {
        // 不再默认加载 ia_robot，由 main.js 的 initDefaultIARobot() 统一加载机器人
        // 如果需要显示占位模型，可以创建简化几何模型
        console.log('✓ 可视化器初始化完成，等待加载机器人...');
    }
    
    /**
     * 加载 URDF 机器人模型
     */
    async loadURDFRobot() {
        try {
            const loader = new URDFLoader();
            const baseUrl = import.meta.env.BASE_URL || '/';
            loader.packages = {
                // 默认改为 ia_robot（同时保留常见别名映射）
                'ia_robot': `${baseUrl}robots/ia_robot`,
                'robot_arm_description': `${baseUrl}robots/ia_robot`
            };
            
            this.urdfRobot = await new Promise((resolve, reject) => {
                loader.load(
                    `${baseUrl}robots/ia_robot/right_arm/right_arm.urdf`,
                    (robot) => {
                        console.log('✓ URDF 加载成功');
                        console.log('关节数:', Object.keys(robot.joints).length);
                        resolve(robot);
                    },
                    undefined,
                    (error) => {
                        console.error('URDF 加载失败:', error);
                        reject(error);
                    }
                );
            });
            
            // URDF 模型保持原始 Y-up，直接添加到场景
            this.scene.add(this.urdfRobot);
            return true;
            
        } catch (error) {
            console.warn('无法加载 URDF:', error);
            return false;
        }
    }
    
    /**
     * 创建简化的几何机器人模型
     */
    createSimplifiedRobot() {
        this.robot = new THREE.Group();
        
        // 材质
        const linkMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xeeeeee,
            shininess: 60
        });
        const jointMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x666666,
            shininess: 80
        });
        
        // 基座（Z-up坐标系，所以使用Z轴）
        const base = new THREE.Mesh(
            new THREE.CylinderGeometry(0.12, 0.15, 0.1, 32),
            jointMaterial
        );
        base.rotation.x = Math.PI / 2; // 旋转使其Z轴向上
        base.position.z = 0.05;
        this.robot.add(base);
        
        // 立柱
        const column = new THREE.Mesh(
            new THREE.CylinderGeometry(0.08, 0.08, 0.3, 16),
            linkMaterial
        );
        column.rotation.x = Math.PI / 2;
        column.position.z = 0.25;
        this.robot.add(column);
        
        // 上臂
        const upperArm = new THREE.Mesh(
            new THREE.BoxGeometry(0.08, 0.08, 0.3),
            linkMaterial
        );
        upperArm.position.set(0.1, 0, 0.5);
        this.robot.add(upperArm);
        
        // 前臂
        const forearm = new THREE.Mesh(
            new THREE.BoxGeometry(0.06, 0.06, 0.25),
            linkMaterial
        );
        forearm.position.set(0.25, 0, 0.6);
        this.robot.add(forearm);
        
        this.scene.add(this.robot);
        console.log('✓ 简化模型已创建');
    }
    
    /**
     * 创建末端执行器标记
     */
    createEndEffectorMarker() {
        const geometry = new THREE.SphereGeometry(0.02, 32, 32);
        const material = new THREE.MeshPhongMaterial({ 
            color: 0xfbbf24,
            emissive: 0xfbbf24,
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.9,
            depthTest: false,  // 不进行深度测试，始终显示在最前面
            depthWrite: false  // 不写入深度缓冲
        });
        this.endEffector = new THREE.Mesh(geometry, material);
        this.endEffector.position.set(0.3, 0, 0.6);
        this.endEffector.renderOrder = 999;  // 设置高渲染优先级，最后渲染
        
        // 添加末端坐标轴（也设置为始终显示在前面）
        const endAxes = new THREE.AxesHelper(0.1);
        endAxes.material.depthTest = false;
        endAxes.material.depthWrite = false;
        endAxes.renderOrder = 999;
        this.endEffector.add(endAxes);
        
        this.scene.add(this.endEffector);
        console.log('✓ 末端执行器标记已创建（始终显示在最前面）');
    }
    
    /**
     * 创建左上角坐标系指示器
     */
    createAxesIndicator() {
        this.axesScene = new THREE.Scene();
        
        // 创建相机
        this.axesCamera = new THREE.OrthographicCamera(-1.5, 1.5, 1.5, -1.5, 0.1, 10);
        this.axesCamera.position.set(3, 3, 3);
        this.axesCamera.lookAt(0, 0, 0);
        
        // 创建坐标轴
        const axesHelper = new THREE.AxesHelper(1);
        this.axesScene.add(axesHelper);
        
        // 创建文字标签
        const createTextSprite = (text, color, position) => {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 128;
            canvas.height = 128;
            
            context.fillStyle = color;
            context.font = 'Bold 90px Arial';
            context.textAlign = 'center';
            context.textBaseline = 'middle';
            context.fillText(text, 64, 64);
            
            const texture = new THREE.CanvasTexture(canvas);
            const spriteMaterial = new THREE.SpriteMaterial({ 
                map: texture,
                depthTest: false
            });
            const sprite = new THREE.Sprite(spriteMaterial);
            sprite.position.copy(position);
            sprite.scale.set(0.4, 0.4, 1);
            
            return sprite;
        };
        
        // 添加 XYZ 标签
        this.axesScene.add(createTextSprite('X', '#ef4444', new THREE.Vector3(1.2, 0, 0)));
        this.axesScene.add(createTextSprite('Y', '#10b981', new THREE.Vector3(0, 1.2, 0)));
        this.axesScene.add(createTextSprite('Z', '#3b82f6', new THREE.Vector3(0, 0, 1.2)));
        
        console.log('✓ 坐标系指示器已创建');
    }
    
    /**
     * 更新相机位置
     */
    updateCameraPosition() {
        // 计算目标点（应用平移偏移）
        const targetX = this.cameraPan.x;
        const targetY = this.cameraPan.y;
        const targetZ = 0.5 + this.cameraPan.z;

        if (this.cameraViewMode === 'top-down') {
            const distance = Math.max(0.5, Math.min(10, this.cameraDistance));
            this.camera.position.set(targetX, targetY, targetZ + distance);
            // 垂直俯视时需要使用非 Z 轴 up，避免 lookAt 奇异
            this.camera.up.set(0, 1, 0);
            this.camera.lookAt(targetX, targetY, targetZ);

            if (this.axesCamera) {
                this.axesCamera.position.copy(this.camera.position).normalize().multiplyScalar(4);
                this.axesCamera.lookAt(0, 0, 0);
                this.axesCamera.up.set(0, 0, 1);
            }
            return;
        }

        const theta = this.cameraRotation.y;
        const phi = this.cameraRotation.x;
        
        const x = this.cameraDistance * Math.sin(phi) * Math.cos(theta);
        const y = this.cameraDistance * Math.sin(phi) * Math.sin(theta);
        const z = this.cameraDistance * Math.cos(phi);
        
        // 应用相机位置和平移偏移
        this.camera.position.set(x + this.cameraPan.x, y + this.cameraPan.y, z + this.cameraPan.z);

        this.camera.lookAt(targetX, targetY, targetZ);
        this.camera.up.set(0, 0, 1); // Z轴向上
        
        // Debug: 打印视角参数（控制台执行 window.debugCameraLog = true 开启）
        if (window.debugCameraLog) {
            console.log(`📷 Camera: rotation.x=${phi.toFixed(4)}, rotation.y=${theta.toFixed(4)}, distance=${this.cameraDistance.toFixed(4)}, pan=(${this.cameraPan.x.toFixed(4)}, ${this.cameraPan.y.toFixed(4)}, ${this.cameraPan.z.toFixed(4)})`);
        }
        
        // 同步坐标系指示器
        if (this.axesCamera) {
            this.axesCamera.position.copy(this.camera.position).normalize().multiplyScalar(4);
            this.axesCamera.lookAt(0, 0, 0);
            this.axesCamera.up.set(0, 0, 1);
        }
    }

    /**
     * 切换到垂直俯视视角（沿 -Z 方向观察 XY 平面）
     */
    setTopDownView() {
        this.cameraViewMode = 'top-down';
        this.updateCameraPosition();
    }
    
    /**
     * 设置事件监听器
     */
    setupEventListeners() {
        // 窗口大小调整
        window.addEventListener('resize', () => {
            const width = this.canvas.clientWidth;
            const height = this.canvas.clientHeight;
            
            this.camera.aspect = width / height;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(width, height);
        });
        
        // 鼠标拖动
        // 左键：旋转视角
        // 右键：平移位置
        this.canvas.addEventListener('mousedown', (e) => {
            // 如果拖动末端执行器模式开启，优先处理拖动
            if (this.dragEndEffectorMode && e.button === 0) {
                this.handleDragEndEffectorMouseDown(e);
                if (this.isDraggingEndEffector) {
                    return; // 如果正在拖动末端执行器，不处理相机旋转
                }
            }
            
            if (e.button === 0) {
                // 左键：旋转视角
                if (this.cameraViewMode === 'top-down') {
                    // 进入手动旋转时回到轨道视角
                    this.cameraViewMode = 'orbit';
                    this.cameraRotation.x = 0.1;
                }
                this.isLeftDragging = true;
                this.previousMousePosition = { x: e.clientX, y: e.clientY };
            } else if (e.button === 2) {
                // 右键：平移位置
                this.isRightDragging = true;
                this.previousMousePosition = { x: e.clientX, y: e.clientY };
                // 阻止默认的右键菜单
                e.preventDefault();
            }
        });
        
        this.canvas.addEventListener('mousemove', (e) => {
            // 如果拖动末端执行器模式开启，优先处理拖动
            if (this.dragEndEffectorMode && this.isDraggingEndEffector) {
                this.handleDragEndEffectorMouseMove(e);
                return; // 如果正在拖动末端执行器，不处理相机旋转
            }
            
            const deltaX = e.clientX - this.previousMousePosition.x;
            const deltaY = e.clientY - this.previousMousePosition.y;
            
            if (this.isLeftDragging) {
                // 左键拖动：旋转视角
                this.cameraRotation.y += deltaX * 0.01;
                this.cameraRotation.x -= deltaY * 0.01;
                
                // 限制角度
                this.cameraRotation.x = Math.max(0.1, Math.min(Math.PI - 0.1, this.cameraRotation.x));
                
                this.updateCameraPosition();
            } else if (this.isRightDragging) {
                // 右键拖动：平移位置
                // 计算相机右方向和上方向
                const right = new THREE.Vector3();
                const up = new THREE.Vector3();
                const forward = new THREE.Vector3();
                
                forward.subVectors(
                    new THREE.Vector3(this.cameraPan.x, this.cameraPan.y, 0.5 + this.cameraPan.z),
                    this.camera.position
                ).normalize();

                right.crossVectors(forward, new THREE.Vector3(0, 0, 1));
                if (right.lengthSq() < 1e-8) {
                    // 俯视时 forward 与 Z 轴共线，改用 Y 轴构造右方向
                    right.crossVectors(forward, new THREE.Vector3(0, 1, 0));
                }
                right.normalize();
                up.crossVectors(right, forward).normalize();
                
                // 根据鼠标移动量平移相机
                const panSpeed = this.cameraDistance * 0.001;
                this.cameraPan.x -= right.x * deltaX * panSpeed;
                this.cameraPan.y -= right.y * deltaX * panSpeed;
                this.cameraPan.z += up.z * deltaY * panSpeed;
                
                this.updateCameraPosition();
            }
            
            if (this.isLeftDragging || this.isRightDragging) {
                this.previousMousePosition = { x: e.clientX, y: e.clientY };
            }
        });
        
        this.canvas.addEventListener('mouseup', (e) => {
            // 如果拖动末端执行器模式开启，优先处理拖动
            if (this.dragEndEffectorMode && e.button === 0) {
                this.handleDragEndEffectorMouseUp(e);
            }
            
            if (e.button === 0) {
                this.isLeftDragging = false;
            } else if (e.button === 2) {
                this.isRightDragging = false;
            }
        });
        
        this.canvas.addEventListener('mouseleave', () => {
            this.isLeftDragging = false;
            this.isRightDragging = false;
            this.isDraggingEndEffector = false;
        });
        
        // 阻止右键菜单（在canvas上）
        this.canvas.addEventListener('contextmenu', (e) => {
            e.preventDefault();
        });
        
        // 滚轮缩放
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            this.cameraDistance *= (1 + e.deltaY * 0.001);
            this.cameraDistance = Math.max(0.5, Math.min(10, this.cameraDistance));
            this.updateCameraPosition();
        }, { passive: false });
    }
    
    /**
     * 动画循环
     */
    animate() {
        requestAnimationFrame(() => this.animate());
        
        // 渲染主场景
        this.renderer.render(this.scene, this.camera);
        
        // 渲染左上角坐标系指示器
        if (this.axesScene && this.axesCamera) {
            this.renderer.autoClear = false;
            this.renderer.clearDepth();
            
            const width = this.canvas.clientWidth;
            const height = this.canvas.clientHeight;
            const size = 120;
            
            this.renderer.setViewport(20, height - size - 20, size, size);
            this.renderer.setScissor(20, height - size - 20, size, size);
            this.renderer.setScissorTest(true);
            
            this.renderer.render(this.axesScene, this.axesCamera);
            
            this.renderer.setViewport(0, 0, width, height);
            this.renderer.setScissorTest(false);
            this.renderer.autoClear = true;
        }
    }
    
    /**
     * 加载自定义 URDF 模型（单臂或双臂）
     * @param {Object} robot - urdf-loader 加载的机器人对象
     * @param {Object} robot2 - 可选的第二个机器人对象（双臂模式，two_urdf格式）或null（single_urdf格式）
     * @param {Object} config - 配置对象，包含双臂模式和基座位姿等
     * @param {string} dualArmType - 双臂格式类型：'two_urdf' 或 'single_urdf'
     */
    async loadCustomURDF(robot, robot2 = null, config = null, dualArmType = 'two_urdf') {
        try {
            console.log('  → 开始加载自定义 URDF 到场景...');
            
            // 检查是否为双臂模式
            const isDualArm = config?.dual_arm === true;
            this.dualArmMode = isDualArm;
            this.dualArmType = dualArmType; // 存储双臂格式类型
            
            if (isDualArm) {
                console.log(`  → 双臂模式：格式=${dualArmType}`);
                return await this.loadDualArmURDF(robot, robot2, config, dualArmType);
            } else {
                console.log('  → 单臂模式：加载单个机器人');
                return await this.loadSingleArmURDF(robot, config);
            }
        } catch (error) {
            console.error('❌ 加载自定义 URDF 失败:', error);
            console.error('错误堆栈:', error.stack);
            return false;
        }
    }
    
    /**
     * 彻底清理旧机器人模型（包括释放资源）
     * @param {THREE.Object3D} object - 要清理的对象
     */
    disposeRobotObject(object) {
        if (!object) return;
        
        // 先收集所有需要清理的子对象
        const objectsToDispose = [];
        object.traverse((child) => {
            objectsToDispose.push(child);
        });
        
        // 遍历所有子对象并释放资源
        objectsToDispose.forEach((child) => {
            if (child instanceof THREE.Mesh || child instanceof THREE.Line || child instanceof THREE.Points) {
                // 释放几何体
                if (child.geometry) {
                    child.geometry.dispose();
                }
                // 释放材质
                if (child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach(mat => {
                            if (mat.map) mat.map.dispose();
                            if (mat.normalMap) mat.normalMap.dispose();
                            if (mat.bumpMap) mat.bumpMap.dispose();
                            if (mat.roughnessMap) mat.roughnessMap.dispose();
                            if (mat.metalnessMap) mat.metalnessMap.dispose();
                            if (mat.aoMap) mat.aoMap.dispose();
                            if (mat.emissiveMap) mat.emissiveMap.dispose();
                            if (mat.dispose) mat.dispose();
                        });
                    } else {
                        if (child.material.map) child.material.map.dispose();
                        if (child.material.normalMap) child.material.normalMap.dispose();
                        if (child.material.bumpMap) child.material.bumpMap.dispose();
                        if (child.material.roughnessMap) child.material.roughnessMap.dispose();
                        if (child.material.metalnessMap) child.material.metalnessMap.dispose();
                        if (child.material.aoMap) child.material.aoMap.dispose();
                        if (child.material.emissiveMap) child.material.emissiveMap.dispose();
                        if (child.material.dispose) child.material.dispose();
                    }
                }
            }
        });
        
        // 从场景中移除（如果还在场景中）
        if (object.parent) {
            object.parent.remove(object);
        } else if (this.scene && this.scene.children.includes(object)) {
            this.scene.remove(object);
        }
    }
    
    /**
     * 清理所有旧机器人模型
     */
    cleanupOldRobots() {
        // 确保先解除环境物体的附着，以免随旧锚点一起被移除
        this.detachAllEnvironmentObjects();
        if (this.humanoidMJWorkerClient) {
            this.humanoidMJWorkerClient.close();
            this.humanoidMJWorkerClient = null;
        }
        this.humanoidMJWorkerModelSpec = null;
        this._humanoidMJWorkerRequestToken = 0;
        this._humanoidMJWorkerLoopActive = false;
        this._humanoidMJWorkerPendingRequest = null;
        this._humanoidMJWorkerLoopActive = false;
        this._humanoidMJWorkerPendingRequest = null;

        // 清理单臂模型
        if (this.urdfRobot) {
            this.disposeRobotObject(this.urdfRobot);
            this.urdfRobot = null;
        }
        if (this.robot) {
            this.disposeRobotObject(this.robot);
            this.robot = null;
        }
        
        // 清理双臂模型
        if (this.robot1URDF) {
            this.disposeRobotObject(this.robot1URDF);
            this.robot1URDF = null;
        }
        if (this.robot2URDF) {
            this.disposeRobotObject(this.robot2URDF);
            this.robot2URDF = null;
        }
        
        // 清理末端执行器标记
        if (this.endEffector) {
            this.disposeRobotObject(this.endEffector);
            this.endEffector = null;
        }
        if (this.robot1EndEffector) {
            this.disposeRobotObject(this.robot1EndEffector);
            this.robot1EndEffector = null;
        }
        if (this.robot2EndEffector) {
            this.disposeRobotObject(this.robot2EndEffector);
            this.robot2EndEffector = null;
        }
        
        // 清理双臂可视化元素
        if (this.absolutePoseMarker) {
            this.disposeRobotObject(this.absolutePoseMarker);
            this.absolutePoseMarker = null;
        }
        if (this.relativePoseLine) {
            this.disposeRobotObject(this.relativePoseLine);
            this.relativePoseLine = null;
        }
        // 清理人形
        this.humanoidMode = false;
        this.humanoidModel = null;
        this.humanoidJointNames = [];
        this.humanoidWaistLength = 0;
        this.humanoidLeftArmLength = 0;
        this.humanoidRightArmLength = 0;
        this.humanoidHeadLength = 0;
        if (this.humanoidEffectorSpheres) {
            this.scene.remove(this.humanoidEffectorSpheres);
            this.humanoidEffectorSpheres.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
            this.humanoidEffectorSpheres = null;
        }
        if (this.humanoidAbsoluteMarker) {
            this.scene.remove(this.humanoidAbsoluteMarker);
            this.humanoidAbsoluteMarker.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
            this.humanoidAbsoluteMarker = null;
        }
        if (this.humanoidRelativeLine) {
            this.scene.remove(this.humanoidRelativeLine);
            if (this.humanoidRelativeLine.geometry) this.humanoidRelativeLine.geometry.dispose();
            if (this.humanoidRelativeLine.material) this.humanoidRelativeLine.material.dispose();
            this.humanoidRelativeLine = null;
        }
        if (this.humanoidLeftArmEndEffector) {
            this.scene.remove(this.humanoidLeftArmEndEffector);
            this.humanoidLeftArmEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
            this.humanoidLeftArmEndEffector = null;
        }
        if (this.humanoidWaistEndEffector) {
            this.scene.remove(this.humanoidWaistEndEffector);
            this.humanoidWaistEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
            this.humanoidWaistEndEffector = null;
        }
        if (this.humanoidHeadEndEffector) {
            this.scene.remove(this.humanoidHeadEndEffector);
            this.humanoidHeadEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
            this.humanoidHeadEndEffector = null;
        }
        if (this.humanoidRightArmEndEffector) {
            this.scene.remove(this.humanoidRightArmEndEffector);
            this.humanoidRightArmEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
            this.humanoidRightArmEndEffector = null;
        }
        if (this.humanoidURDF) {
            this.disposeRobotObject(this.humanoidURDF);
            this.humanoidURDF = null;
        }
        
        // 清理拖动控制
        if (this.endEffectorGizmo) {
            this.disposeRobotObject(this.endEffectorGizmo);
            this.endEffectorGizmo = null;
        }
        
        // 额外检查：遍历场景，移除任何可能残留的机器人相关对象
        // 通过检查对象的类型或名称来识别
        const objectsToRemove = [];
        const processedObjects = new Set(); // 避免重复处理
        
        // 先检查场景的直接子对象
        this.scene.children.forEach((object) => {
            if (processedObjects.has(object)) return;
            
            // 检查是否是URDF相关的对象
            const isURDFRobot = object && typeof object === 'object' && 
                (object.joints || object.links || 
                 (object.name && (object.name.toLowerCase().includes('robot') || 
                                  object.name.toLowerCase().includes('urdf') || 
                                  object.name.toLowerCase().includes('panda') ||
                                  object.name.toLowerCase().includes('franka'))));
            
            // 确保不是环境物体、坐标系、网格、光源、相机等
            if (isURDFRobot && 
                object !== this.environmentGroup && 
                object !== this.mdhFramesGroup && 
                object !== this.urdfFramesGroup &&
                object !== this.axesScene &&
                object !== this.camera &&
                !object.isGridHelper &&
                !object.isAxesHelper &&
                !object.isLight &&
                !object.isAmbientLight &&
                !object.isDirectionalLight &&
                object.parent === this.scene) {
                objectsToRemove.push(object);
                processedObjects.add(object);
            }
        });
        
        // 移除找到的残留对象
        objectsToRemove.forEach(obj => {
            console.log(`  清理残留对象: ${obj.name || obj.type || obj.constructor.name || 'unknown'}`);
            this.disposeRobotObject(obj);
        });
        
        if (objectsToRemove.length > 0) {
            console.log(`✓ 已清理 ${objectsToRemove.length} 个残留的机器人对象`);
        }
        
        console.log('✓ 已清理所有旧机器人模型');
    }
    
    /**
     * 加载单臂 URDF 模型
     * @param {Object} robot - urdf-loader 加载的机器人对象
     * @param {Object} config - 配置对象，包含基座位姿等
     */
    async loadSingleArmURDF(robot, config = null) {
        // 彻底清理所有旧模型
        this.cleanupOldRobots();
        
        // 设置双臂模式为false
        this.dualArmMode = false;
        
        // 应用基座位姿（如果配置中有）
        // 优先从 mdh_extraction.base_pose 读取，如果不存在则从 config.base_pose 读取
        let basePose = config?.mdh_extraction?.base_pose || config?.base_pose;
        if (basePose && Array.isArray(basePose) && basePose.length >= 3) {
            console.log('  设置单臂基座位姿:', basePose);
            if (basePose.length === 8) {
                // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                // 使用 DQModule 提取旋转和平移
                if (window.DQModule && window.DQModule.DQWrapper) {
                    try {
                        console.log(`    原始双四元数: [${basePose.map(v => v.toFixed(6)).join(', ')}]`);
                        
                        // 创建双四元数并归一化（必须归一化才能提取旋转和平移）
                        let dq = window.DQModule.DQWrapper.createFromArray(basePose);
                        dq = window.DQModule.DQWrapper.normalize(dq);
                        
                        const rotation = window.DQModule.DQWrapper.getRotation(dq);
                        const translation = window.DQModule.DQWrapper.getTranslation(dq);
                        
                        console.log(`    提取的旋转: [${rotation.map(v => v.toFixed(6)).join(', ')}]`);
                        console.log(`    提取的平移: [${translation.map(v => v.toFixed(6)).join(', ')}]`);
                        
                        // rotation: [qw, qx, qy, qz]
                        // translation: [w, x, y, z] (w=0, xyz是位置)
                        const [qw, qx, qy, qz] = rotation;
                        const [, tx, ty, tz] = translation;
                        
                        robot.position.set(tx || 0, ty || 0, tz || 0);
                        robot.quaternion.set(qx, qy, qz, qw);
                        console.log(`    应用的位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                        console.log(`    应用的旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                    } catch (error) {
                        console.warn('  无法从双四元数提取位姿，回退到前4个元素作为旋转:', error);
                        // 回退：假设前4个是旋转，后4个包含平移信息
                        const [qw, qx, qy, qz] = basePose.slice(0, 4);
                        robot.quaternion.set(qx, qy, qz, qw);
                        console.log(`    旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                    }
                } else {
                    console.warn('  DQModule 不可用，无法从双四元数提取位姿');
                }
            } else if (basePose.length === 7) {
                // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                const [qw, qx, qy, qz, tx, ty, tz] = basePose;
                robot.position.set(tx || 0, ty || 0, tz || 0);
                // Three.js 四元数顺序是 (x, y, z, w)
                robot.quaternion.set(qx, qy, qz, qw);
                console.log(`    位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                console.log(`    旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
            } else if (basePose.length >= 3) {
                // 仅平移 [tx, ty, tz] 或 [tx, ty, tz, ...]
                const [tx, ty, tz] = basePose;
                robot.position.set(tx || 0, ty || 0, tz || 0);
                console.log(`    位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
            }
        }
        
        // 添加新模型
        this.urdfRobot = robot;
        this.scene.add(robot);
        
        // 重新创建单臂末端执行器标记（如果之前没有）
        if (!this.endEffector) {
            this.createEndEffectorMarker();
        }
        
        // 初始化URDF坐标系显示（如果开关已打开）
        if (this.showURDFFrames && robot && robot.links) {
            const linkNames = Object.keys(robot.links);
            this.initURDFFrames(robot, linkNames, null, null);
        }
        
        console.log('✓ 单臂 URDF 加载完成');
        return true;
    }
    
    /**
     * 加载人形 URDF 模型（单棵 URDF 树 + WASM HumanoidURDF；姿态控制以左臂、右臂为主）
     * @param {Object} robot - urdf-loader 加载的机器人对象
     * @param {Object} config - 配置（含 humanoid_link_names 等）
     * @param {Object} data - { model: WASM HumanoidURDF, jointNames: string[] }
     */
    async loadHumanoidURDF(robot, config, data) {
        this.cleanupOldRobots();
        this.dualArmMode = false;
        this.humanoidMode = true;
        this.humanoidURDF = robot;
        this.humanoidModel = data.model;
        this.humanoidJointNames = data.jointNames || [];
        this.humanoidWaistLength = data.waistLength ?? 0;
        this.humanoidLeftArmLength = data.leftArmLength ?? 0;
        this.humanoidRightArmLength = data.rightArmLength ?? 0;
        this.humanoidHeadLength = data.headLength ?? 0;
        this.humanoidEffectorLinkNames = Array.isArray(data.effectorLinkNames) ? data.effectorLinkNames : [];
        this.humanoidMJWorkerModelSpec = data?.mjRestoreWorkerModelSpec ?? null;
        if (this._getHumanoidJogSolverMode() === 'mj_restore') {
            void this._warmHumanoidJogMJRestoreWorkerIfNeeded();
        }
        const basePose = config?.base_pose || config?.mdh_extraction?.base_pose || [0, 0, 0];
        if (Array.isArray(basePose) && basePose.length >= 3) {
            const [tx, ty, tz] = basePose;
            robot.position.set(tx || 0, ty || 0, tz || 0);
        }
        this.humanoidBaseHeight = robot.position.z || 0;
        this.scene.add(robot);
        this._createHumanoidEffectorSpheres();
        this._createHumanoidDualArmVisualizations();
        
        // 初始化URDF坐标系显示（如果开关已打开）
        if (this.showURDFFrames && robot && robot.links) {
            const linkNames = Object.keys(robot.links);
            this.initURDFFrames(robot, linkNames, null, null);
        }
        
        console.log('✓ 人形 URDF 加载完成，关节数:', this.humanoidJointNames.length, '（腰/左臂/右臂/头）仅左臂+右臂参与 IK');
        return true;
    }

    _canUseHumanoidJogMJRestoreWorker() {
        const cfg = this.vfiConfig || {};
        return cfg.mjRestoreUseWorker !== false &&
            !!this.humanoidMJWorkerModelSpec &&
            typeof Worker === 'function';
    }

    _getHumanoidJogMJWorkerClient() {
        if (!this._canUseHumanoidJogMJRestoreWorker()) return null;
        if (!this.humanoidMJWorkerClient) {
            this.humanoidMJWorkerClient = new HumanoidJogMJWorkerClient({
                workerScriptVersion: this.humanoidMJWorkerScriptVersion
            });
        }
        return this.humanoidMJWorkerClient;
    }

    _warmHumanoidJogMJRestoreWorkerIfNeeded() {
        if (!this._canUseHumanoidJogMJRestoreWorker()) return null;
        if (!this.humanoidMJWorkerModelSpec) return null;
        if (this._humanoidMJWorkerWarmPromise) return this._humanoidMJWorkerWarmPromise;
        const client = this._getHumanoidJogMJWorkerClient();
        if (!client || typeof client.ensureModel !== 'function') return null;
        this._humanoidMJWorkerWarmPromise = Promise.resolve()
            .then(() => client.ensureModel(this.humanoidMJWorkerModelSpec))
            .catch((error) => {
                console.warn('MuJoCo 风格恢复 Worker 预热失败:', error);
                return false;
            })
            .finally(() => {
                this._humanoidMJWorkerWarmPromise = null;
            });
        return this._humanoidMJWorkerWarmPromise;
    }

    _enqueueHumanoidJogMJRestoreWorkerRequest(request) {
        this._humanoidMJWorkerPendingRequest = request;
        if (this._humanoidMJWorkerLoopActive) {
            return true;
        }
        this._humanoidMJWorkerLoopActive = true;
        const drainQueue = async () => {
            try {
                while (this._humanoidMJWorkerPendingRequest) {
                    const nextRequest = this._humanoidMJWorkerPendingRequest;
                    this._humanoidMJWorkerPendingRequest = null;
                    const requestToken = ++this._humanoidMJWorkerRequestToken;
                    await this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
                        nextRequest.dqAbsoluteTargetArray,
                        nextRequest.currentQ,
                        nextRequest.enableJointLimitAvoidance,
                        nextRequest.includeWaist,
                        nextRequest.keepBodyZVertical,
                        nextRequest.fixedRelativeTargetArray,
                        {
                            solverMode: 'mj_restore',
                            translationAxisGuard: nextRequest.translationAxisGuard,
                            requestToken,
                            preferWorker: true,
                            jogMetricOverride: nextRequest.jogMetricOverride,
                            metricDirection: nextRequest.metricDirection
                        }
                    );
                }
            } catch (error) {
                console.error('MuJoCo 风格恢复 Worker 队列异常:', error);
            } finally {
                this._humanoidMJWorkerLoopActive = false;
                if (this._humanoidMJWorkerPendingRequest) {
                    this._enqueueHumanoidJogMJRestoreWorkerRequest(this._humanoidMJWorkerPendingRequest);
                }
            }
        };
        void drainQueue();
        return true;
    }

    /**
     * 设置人形底盘平面位姿（x/y/yaw）
     * @param {{x:number,y:number,yaw:number}} basePose
     */
    setHumanoidBasePose(basePose = {}) {
        if (!this.humanoidURDF) return;
        const x = Number(basePose.x) || 0;
        const y = Number(basePose.y) || 0;
        const yaw = Number(basePose.yaw) || 0;
        const z = Number(this.humanoidBaseHeight) || 0;

        this.humanoidURDF.position.set(x, y, z);
        this.humanoidURDF.quaternion.set(0, 0, Math.sin(yaw / 2), Math.cos(yaw / 2));

        if (typeof this.humanoidURDF.updateMatrixWorld === 'function') {
            this.humanoidURDF.updateMatrixWorld(true);
        }
        if (this.showURDFFrames) {
            this.updateAllURDFFrames();
        }
        this._updateHumanoidEffectorSpheres(window?.humanoidQ || []);
    }

    // ──────────────────────────────────────────────────────────────
    // Regulated Pure Pursuit 路径可视化
    // ──────────────────────────────────────────────────────────────

    /**
     * 初始化路径可视化的 Three.js 组（在 scene 已创建后调用）
     */
    _initRPPPathGroup() {
        if (this.rppPathGroup) return;
        this.rppPathGroup = new THREE.Group();
        this.rppPathGroup.name = 'RPPPathVisualization';
        this.scene.add(this.rppPathGroup);
    }

    /**
     * 设置路径航点并更新可视化
     * @param {Array<{x:number, y:number, yaw?:number}>} waypoints
     */
    setPathWaypoints(waypoints = []) {
        this._initRPPPathGroup();

        // 清除旧的路径线
        if (this.rppPathLine) {
            this.rppPathGroup.remove(this.rppPathLine);
            this.rppPathLine.geometry.dispose();
            this.rppPathLine.material.dispose();
            this.rppPathLine = null;
        }

        // 清除旧的航点标记
        for (const m of this.rppWaypointMarkers) {
            this.rppPathGroup.remove(m);
            m.geometry.dispose();
            m.material.dispose();
        }
        this.rppWaypointMarkers = [];

        if (waypoints.length === 0) return;

        // 路径折线（Z-up，路径放在地面稍上方 z=0.005）
        const linePoints = waypoints.map(wp => new THREE.Vector3(wp.x, wp.y, 0.005));
        const lineGeom = new THREE.BufferGeometry().setFromPoints(linePoints);
        const lineMat = new THREE.LineBasicMaterial({ color: 0x22d3ee, linewidth: 2, transparent: true, opacity: 0.85 });
        this.rppPathLine = new THREE.Line(lineGeom, lineMat);
        this.rppPathGroup.add(this.rppPathLine);

        // 航点圆球
        const sphereGeom = new THREE.SphereGeometry(0.025, 12, 8);
        for (let i = 0; i < waypoints.length; i++) {
            const wp = waypoints[i];
            const isLast = i === waypoints.length - 1;
            const color = isLast ? 0xf97316 : 0x22d3ee; // 最后一个点为橙色
            const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 });
            const sphere = new THREE.Mesh(sphereGeom.clone(), mat);
            sphere.position.set(wp.x, wp.y, 0.025);
            this.rppPathGroup.add(sphere);
            this.rppWaypointMarkers.push(sphere);
        }

        // 最后一个航点如有 yaw，画一个短箭头指示朝向
        const lastWP = waypoints[waypoints.length - 1];
        if (lastWP.yaw !== undefined) {
            const arrowLen = 0.12;
            const dirX = Math.cos(lastWP.yaw) * arrowLen;
            const dirY = Math.sin(lastWP.yaw) * arrowLen;
            const arrowPts = [
                new THREE.Vector3(lastWP.x, lastWP.y, 0.01),
                new THREE.Vector3(lastWP.x + dirX, lastWP.y + dirY, 0.01)
            ];
            const arrowGeom = new THREE.BufferGeometry().setFromPoints(arrowPts);
            const arrowMat = new THREE.LineBasicMaterial({ color: 0xf97316, linewidth: 2 });
            const arrowLine = new THREE.Line(arrowGeom, arrowMat);
            this.rppPathGroup.add(arrowLine);
            this.rppWaypointMarkers.push(arrowLine); // 便于统一清除
        }
    }

    /**
     * 设置前视点标记位置
     * @param {{x:number, y:number}|null} point
     */
    setLookaheadPoint(point) {
        this._initRPPPathGroup();

        if (!point) {
            if (this.rppLookaheadMarker) this.rppLookaheadMarker.visible = false;
            return;
        }

        if (!this.rppLookaheadMarker) {
            const geom = new THREE.SphereGeometry(0.035, 16, 12);
            const mat = new THREE.MeshBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.9 });
            this.rppLookaheadMarker = new THREE.Mesh(geom, mat);
            this.rppLookaheadMarker.name = 'RPPLookaheadMarker';
            this.rppPathGroup.add(this.rppLookaheadMarker);
        }

        this.rppLookaheadMarker.visible = true;
        this.rppLookaheadMarker.position.set(point.x, point.y, 0.035);
    }

    /**
     * 清除所有路径可视化
     */
    clearPathVisualization() {
        if (this.rppPathLine) {
            this.rppPathGroup?.remove(this.rppPathLine);
            this.rppPathLine.geometry.dispose();
            this.rppPathLine.material.dispose();
            this.rppPathLine = null;
        }
        for (const m of this.rppWaypointMarkers) {
            this.rppPathGroup?.remove(m);
            if (m.geometry) m.geometry.dispose();
            if (m.material) m.material.dispose();
        }
        this.rppWaypointMarkers = [];
        if (this.rppLookaheadMarker) {
            this.rppLookaheadMarker.visible = false;
        }
    }

    /**
     * 启用/禁用"点击地面添加航点"模式
     */
    setAddWaypointMode(enabled) {
        this.rppAddWaypointMode = !!enabled;
        if (this.canvas) {
            this.canvas.style.cursor = enabled ? 'crosshair' : '';
        }
    }

    /**
     * 处理画布点击事件（在添加航点模式下 raycast 到地面）
     * 通过 mousedown/mouseup 距离判断是否为"点击"而非"拖动"
     */
    _handleCanvasClickForWaypoint(e) {
        if (!this.rppAddWaypointMode || !this.onGroundClick) return;

        // 排除拖动操作
        if (this.isDraggingEndEffector) return;

        // 通过 mousedown → click 的时间差和距离判断是否为拖动（>5px 或 >300ms 判为拖动）
        if (this._rppMouseDownPos) {
            const dx = e.clientX - this._rppMouseDownPos.x;
            const dy = e.clientY - this._rppMouseDownPos.y;
            const dt = performance.now() - (this._rppMouseDownTime || 0);
            if (Math.hypot(dx, dy) > 5 || dt > 400) return;
        }

        const rect = this.canvas.getBoundingClientRect();
        const mouse = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width) * 2 - 1,
            -((e.clientY - rect.top) / rect.height) * 2 + 1
        );
        this.raycaster.setFromCamera(mouse, this.camera);

        const intersection = new THREE.Vector3();
        const hit = this.raycaster.ray.intersectPlane(this._groundPlane, intersection);
        if (hit) {
            this.onGroundClick(intersection.x, intersection.y);
        }
    }

    _createHumanoidDualArmVisualizations() {
        if (this.humanoidAbsoluteMarker) {
            this.scene.remove(this.humanoidAbsoluteMarker);
            this.humanoidAbsoluteMarker.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
        }
        if (this.humanoidRelativeLine) {
            this.scene.remove(this.humanoidRelativeLine);
            if (this.humanoidRelativeLine.geometry) this.humanoidRelativeLine.geometry.dispose();
            if (this.humanoidRelativeLine.material) this.humanoidRelativeLine.material.dispose();
        }
        if (this.humanoidLeftArmEndEffector) {
            this.scene.remove(this.humanoidLeftArmEndEffector);
            this.humanoidLeftArmEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
        }
        if (this.humanoidRightArmEndEffector) {
            this.scene.remove(this.humanoidRightArmEndEffector);
            this.humanoidRightArmEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
        }
        if (this.humanoidWaistEndEffector) {
            this.scene.remove(this.humanoidWaistEndEffector);
            this.humanoidWaistEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
        }
        if (this.humanoidHeadEndEffector) {
            this.scene.remove(this.humanoidHeadEndEffector);
            this.humanoidHeadEndEffector.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
        }
        const size = 0.02;
        const colorAbs = 0xfbbf24;
        const color1 = 0x3b82f6;
        const color2 = 0x10b981;
        const colorLine = 0xef4444;
        this.humanoidAbsoluteMarker = new THREE.Group();
        const sphereGeo = new THREE.SphereGeometry(0.02, 16, 16);
        const sphereMat = new THREE.MeshPhongMaterial({ color: colorAbs, emissive: colorAbs, emissiveIntensity: 0.7, transparent: true, opacity: 0.9 });
        sphereMat.depthTest = false;
        sphereMat.depthWrite = false;
        const absSphere = new THREE.Mesh(sphereGeo, sphereMat);
        absSphere.renderOrder = 998;
        this.humanoidAbsoluteMarker.add(absSphere);
        const absAxes = new THREE.AxesHelper(0.08);
        absAxes.renderOrder = 1001;
        // AxesHelper uses array of materials for each axis color
        if (Array.isArray(absAxes.material)) {
            absAxes.material.forEach(mat => { mat.depthTest = false; mat.depthWrite = false; mat.transparent = true; mat.opacity = 1.0; });
        } else if (absAxes.material) {
            absAxes.material.depthTest = false; absAxes.material.depthWrite = false; absAxes.material.transparent = true; absAxes.material.opacity = 1.0;
        }
        this.humanoidAbsoluteMarker.add(absAxes);
        this.scene.add(this.humanoidAbsoluteMarker);
        const lineGeo = new THREE.BufferGeometry();
        lineGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
        this.humanoidRelativeLine = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: colorLine, transparent: true, opacity: 0.6 }));
        this.humanoidRelativeLine.renderOrder = 997;
        this.scene.add(this.humanoidRelativeLine);
        const geo1 = new THREE.SphereGeometry(size, 32, 32);
        const mat1 = new THREE.MeshPhongMaterial({ color: color1, emissive: color1, emissiveIntensity: 0.5, transparent: true, opacity: 0.9 });
        mat1.depthTest = false;
        mat1.depthWrite = false;
        this.humanoidLeftArmEndEffector = new THREE.Mesh(geo1, mat1);
        this.humanoidLeftArmEndEffector.renderOrder = 999;
        const axes1 = new THREE.AxesHelper(size * 2);
        if (axes1.material) { axes1.material.depthTest = false; axes1.material.depthWrite = false; axes1.material.transparent = true; axes1.material.opacity = 0.95; }
        this.humanoidLeftArmEndEffector.add(axes1);
        this.keepMeshWorldScale(this.humanoidLeftArmEndEffector);
        this.scene.add(this.humanoidLeftArmEndEffector);
        const geo2 = new THREE.SphereGeometry(size, 32, 32);
        const mat2 = new THREE.MeshPhongMaterial({ color: color2, emissive: color2, emissiveIntensity: 0.5, transparent: true, opacity: 0.9 });
        mat2.depthTest = false;
        mat2.depthWrite = false;
        this.humanoidRightArmEndEffector = new THREE.Mesh(geo2, mat2);
        this.humanoidRightArmEndEffector.renderOrder = 999;
        const axes2 = new THREE.AxesHelper(size * 2);
        if (axes2.material) { axes2.material.depthTest = false; axes2.material.depthWrite = false; axes2.material.transparent = true; axes2.material.opacity = 0.95; }
        this.humanoidRightArmEndEffector.add(axes2);
        this.keepMeshWorldScale(this.humanoidRightArmEndEffector);
        this.scene.add(this.humanoidRightArmEndEffector);
        
        // 腰部末端坐标轴（灰色，仅显示坐标轴）
        const colorWaist = 0x94a3b8;
        this.humanoidWaistEndEffector = new THREE.Group();
        this.humanoidWaistEndEffector.renderOrder = 999;
        const axesWaist = new THREE.AxesHelper(size * 2);
        if (axesWaist.material) { axesWaist.material.depthTest = false; axesWaist.material.depthWrite = false; axesWaist.material.transparent = true; axesWaist.material.opacity = 0.95; }
        this.humanoidWaistEndEffector.add(axesWaist);
        this.scene.add(this.humanoidWaistEndEffector);
        
        // 头部末端坐标轴（橙色，仅显示坐标轴）
        const colorHead = 0xf59e0b;
        this.humanoidHeadEndEffector = new THREE.Group();
        this.humanoidHeadEndEffector.renderOrder = 999;
        const axesHead = new THREE.AxesHelper(size * 2);
        if (axesHead.material) { axesHead.material.depthTest = false; axesHead.material.depthWrite = false; axesHead.material.transparent = true; axesHead.material.opacity = 0.95; }
        this.humanoidHeadEndEffector.add(axesHead);
        this.scene.add(this.humanoidHeadEndEffector);
    }
    _createHumanoidEffectorSpheres() {
        if (this.humanoidEffectorSpheres) {
            this.scene.remove(this.humanoidEffectorSpheres);
            this.humanoidEffectorSpheres.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); });
        }
        const radius = 0.025;
        const colors = [0x94a3b8, 0x3b82f6, 0x10b981, 0xf59e0b];
        const names = ['waist', 'left_arm', 'right_arm', 'head'];
        this.humanoidEffectorSpheres = new THREE.Group();
        this.humanoidEffectorSpheres.name = 'HumanoidEffectorSpheres';
        for (let i = 0; i < 4; i++) {
            const geo = new THREE.SphereGeometry(radius, 16, 16);
            const mat = new THREE.MeshPhongMaterial({ color: colors[i], transparent: true, opacity: 0.9 });
            mat.depthTest = false;
            mat.depthWrite = false;
            const mesh = new THREE.Mesh(geo, mat);
            mesh.name = names[i];
            mesh.renderOrder = 998;
            this.humanoidEffectorSpheres.add(mesh);
        }
        this.scene.add(this.humanoidEffectorSpheres);
    }
    
    /**
     * 设置人形关节角（与 humanoidJointNames 顺序一致）
     * @param {number[]} q - 全关节角 [腰, 左臂, 右臂, 头]
     */
    setHumanoidJoints(q) {
        if (!this.humanoidURDF || !this.humanoidURDF.joints || !this.humanoidJointNames.length) return;
        for (let i = 0; i < this.humanoidJointNames.length; i++) {
            const name = this.humanoidJointNames[i];
            const joint = this.humanoidURDF.joints[name];
            if (joint && typeof joint.setJointValue === 'function') {
                joint.setJointValue(q[i] ?? 0);
            }
        }
        if (typeof this.humanoidURDF.updateMatrixWorld === 'function') {
            this.humanoidURDF.updateMatrixWorld(true);
        }
        this._updateHumanoidEffectorSpheres(q);
    }
    _updateHumanoidEffectorSpheres(q) {
        if (!this.humanoidEffectorSpheres || !this.humanoidURDF) return;
        const robot = this.humanoidURDF;
        const linkNames = this.humanoidEffectorLinkNames;
        const basePose = (typeof window !== 'undefined')
            ? (window.humanoidBasePose || window.humanoidBaseTargetPose || { x: 0, y: 0, yaw: 0 })
            : { x: 0, y: 0, yaw: 0 };
        let leftPos = null;
        let rightPos = null;
        if (Array.isArray(linkNames) && linkNames.length >= 4 && robot.links) {
            for (let i = 0; i < 4; i++) {
                const linkName = linkNames[i];
                const link = linkName ? robot.links[linkName] : null;
                if (link && typeof link.getWorldPosition === 'function') {
                    link.getWorldPosition(this._tempVec3);
                    const mesh = this.humanoidEffectorSpheres.children[i];
                    if (mesh) mesh.position.copy(this._tempVec3);
                    if (i === 1) leftPos = this._tempVec3.clone();
                    if (i === 2) rightPos = this._tempVec3.clone();
                }
            }
            if (leftPos && rightPos) {
                if (this.humanoidAbsoluteMarker && window.humanoidQ?.length) {
                    // 使用 DQ 公式计算的绝对位姿：x2 * pow(relative, 0.5)
                    const dqAbsLocalArray = this._getHumanoidAbsolutePose(window.humanoidQ);
                    const dqAbsArray = dqAbsLocalArray ? (this._composeHumanoidWorldPose(basePose, dqAbsLocalArray) || dqAbsLocalArray) : null;
                    if (dqAbsArray) {
                        const DQ = window.DQModule?.DQWrapper;
                        if (DQ) {
                            const dqAbs = DQ.createFromArray(dqAbsArray);
                            // 提取平移
                            const transArray = DQ.getTranslation(dqAbs);
                            this.humanoidAbsoluteMarker.position.set(transArray[1], transArray[2], transArray[3]);
                            // 提取旋转
                            const rotArray = DQ.getRotation(dqAbs);
                            // DQ 四元数格式 [w, x, y, z]，THREE.Quaternion 格式 (x, y, z, w)
                            this.humanoidAbsoluteMarker.quaternion.set(rotArray[1], rotArray[2], rotArray[3], rotArray[0]);
                        }
                    }
                }
                if (this.humanoidRelativeLine && this.humanoidRelativeLine.geometry?.attributes?.position) {
                    const pos = this.humanoidRelativeLine.geometry.attributes.position;
                    pos.array[0] = leftPos.x; pos.array[1] = leftPos.y; pos.array[2] = leftPos.z;
                    pos.array[3] = rightPos.x; pos.array[4] = rightPos.y; pos.array[5] = rightPos.z;
                    pos.needsUpdate = true;
                }
                
                // 使用 DQ 计算更新所有末端执行器坐标轴
                const DQ = window.DQModule?.DQWrapper;
                if (DQ && window.humanoidQ?.length && this.humanoidModel) {
                    // 腰部末端 (effector index 0)
                    if (this.humanoidWaistEndEffector) {
                        const pose0Local = this.humanoidModel.fkmEffector(window.humanoidQ, 0);
                        const pose0 = pose0Local ? (this._composeHumanoidWorldPose(basePose, pose0Local) || pose0Local) : null;
                        if (pose0?.length >= 8) {
                            const dq0 = DQ.createFromArray(pose0);
                            const t0 = DQ.getTranslation(dq0);
                            const r0 = DQ.getRotation(dq0);
                            this.humanoidWaistEndEffector.position.set(t0[1], t0[2], t0[3]);
                            this.humanoidWaistEndEffector.quaternion.set(r0[1], r0[2], r0[3], r0[0]);
                        }
                    }
                    
                    // 左臂末端 (effector index 1)
                    if (this.humanoidLeftArmEndEffector) {
                        const pose1Local = this.humanoidModel.fkmEffector(window.humanoidQ, 1);
                        const pose1 = pose1Local ? (this._composeHumanoidWorldPose(basePose, pose1Local) || pose1Local) : null;
                        if (pose1?.length >= 8) {
                            const dq1 = DQ.createFromArray(pose1);
                            const t1 = DQ.getTranslation(dq1);
                            const r1 = DQ.getRotation(dq1);
                            this.humanoidLeftArmEndEffector.position.set(t1[1], t1[2], t1[3]);
                            this.humanoidLeftArmEndEffector.quaternion.set(r1[1], r1[2], r1[3], r1[0]);
                            
                            // 调试：比较 URDF link 和 DQ fkm 的位置
                            if (!this._debugLoggedOnce) {
                                const link = robot.links[linkNames[1]];
                                if (link) {
                                    link.getWorldPosition(this._tempVec3);
                                    link.getWorldQuaternion(this._tempQuat);
                                    console.log('=== 左臂末端位置比较 ===');
                                    console.log('URDF link 位置:', this._tempVec3.x.toFixed(4), this._tempVec3.y.toFixed(4), this._tempVec3.z.toFixed(4));
                                    console.log('DQ fkm 位置:', t1[1].toFixed(4), t1[2].toFixed(4), t1[3].toFixed(4));
                                    console.log('URDF link 旋转 (xyzw):', this._tempQuat.x.toFixed(4), this._tempQuat.y.toFixed(4), this._tempQuat.z.toFixed(4), this._tempQuat.w.toFixed(4));
                                    console.log('DQ fkm 旋转 (wxyz):', r1[0].toFixed(4), r1[1].toFixed(4), r1[2].toFixed(4), r1[3].toFixed(4));
                                }
                            }
                        }
                    }
                    
                    // 右臂末端 (effector index 2)
                    if (this.humanoidRightArmEndEffector) {
                        const pose2Local = this.humanoidModel.fkmEffector(window.humanoidQ, 2);
                        const pose2 = pose2Local ? (this._composeHumanoidWorldPose(basePose, pose2Local) || pose2Local) : null;
                        if (pose2?.length >= 8) {
                            const dq2 = DQ.createFromArray(pose2);
                            const t2 = DQ.getTranslation(dq2);
                            const r2 = DQ.getRotation(dq2);
                            this.humanoidRightArmEndEffector.position.set(t2[1], t2[2], t2[3]);
                            this.humanoidRightArmEndEffector.quaternion.set(r2[1], r2[2], r2[3], r2[0]);
                            
                            // 调试：比较 URDF link 和 DQ fkm 的位置
                            if (!this._debugLoggedOnce) {
                                const link = robot.links[linkNames[2]];
                                if (link) {
                                    link.getWorldPosition(this._tempVec3);
                                    link.getWorldQuaternion(this._tempQuat);
                                    console.log('=== 右臂末端位置比较 ===');
                                    console.log('URDF link 位置:', this._tempVec3.x.toFixed(4), this._tempVec3.y.toFixed(4), this._tempVec3.z.toFixed(4));
                                    console.log('DQ fkm 位置:', t2[1].toFixed(4), t2[2].toFixed(4), t2[3].toFixed(4));
                                    console.log('URDF link 旋转 (xyzw):', this._tempQuat.x.toFixed(4), this._tempQuat.y.toFixed(4), this._tempQuat.z.toFixed(4), this._tempQuat.w.toFixed(4));
                                    console.log('DQ fkm 旋转 (wxyz):', r2[0].toFixed(4), r2[1].toFixed(4), r2[2].toFixed(4), r2[3].toFixed(4));
                                    this._debugLoggedOnce = true;
                                }
                            }
                        }
                    }
                    
                    // 头部末端 (effector index 3)
                    if (this.humanoidHeadEndEffector) {
                        const pose3Local = this.humanoidModel.fkmEffector(window.humanoidQ, 3);
                        const pose3 = pose3Local ? (this._composeHumanoidWorldPose(basePose, pose3Local) || pose3Local) : null;
                        if (pose3?.length >= 8) {
                            const dq3 = DQ.createFromArray(pose3);
                            const t3 = DQ.getTranslation(dq3);
                            const r3 = DQ.getRotation(dq3);
                            this.humanoidHeadEndEffector.position.set(t3[1], t3[2], t3[3]);
                            this.humanoidHeadEndEffector.quaternion.set(r3[1], r3[2], r3[3], r3[0]);
                        }
                    }
                }
            }
            return;
        }
        if (!this.humanoidModel || !q || q.length === 0) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        for (let i = 0; i < 4; i++) {
            const pose = this.humanoidModel.fkmEffector(q, i);
            if (!pose || pose.length < 8) continue;
            const dq = DQ.createFromArray(pose);
            const t = DQ.getTranslation(dq);
            const x = t[1] ?? 0, y = t[2] ?? 0, z = t[3] ?? 0;
            this._tempVec3.set(x, y, z).applyQuaternion(robot.quaternion).add(robot.position);
            const mesh = this.humanoidEffectorSpheres.children[i];
            if (mesh) mesh.position.copy(this._tempVec3);
        }
    }
    
    /**
     * 加载双臂 URDF 模型
     * @param {Object} robot1 - 第一个机器人对象（two_urdf格式）或共享URDF对象（single_urdf格式）
     * @param {Object} robot2 - 第二个机器人对象（two_urdf格式）或null（single_urdf格式）
     * @param {Object} config - 配置对象
     * @param {string} dualArmType - 双臂格式类型：'two_urdf' 或 'single_urdf'
     */
    async loadDualArmURDF(robot1, robot2, config, dualArmType = 'two_urdf') {
        // 彻底清理所有旧模型
        this.cleanupOldRobots();

        // 设置双臂模式为true
        this.dualArmMode = true;
        
        if (dualArmType === 'single_urdf') {
            // 单URDF格式：只添加一个URDF对象（包含两条链）
            console.log('  → 单URDF格式：只添加一个共享URDF模型');
            this.robot1URDF = robot1;
            this.robot2URDF = robot1; // 两个机器人使用同一个URDF对象
            
            // 对于single_urdf格式，通常不需要设置base_pose（因为两条链共享base_link）
            // 但如果配置中有偏移，可以应用
            const basePose1 = config?.robot1?.base_pose || [0, 0, 0];
            if (Array.isArray(basePose1) && basePose1.some(v => v !== 0)) {
                console.log('  应用机器人1基座偏移:', basePose1);
                if (basePose1.length >= 3) {
                    const [tx, ty, tz] = basePose1;
                    robot1.position.set(tx || 0, ty || 0, tz || 0);
                }
            }
            
            this.scene.add(robot1);
            console.log('  ✓ 共享URDF模型已添加到场景，场景对象数:', this.scene.children.length);
        } else {
            // 双URDF格式：添加两个独立的URDF对象
            console.log('  → 双URDF格式：添加两个独立的URDF模型');
            
            // 设置机器人1
            this.robot1URDF = robot1;
            const basePose1 = config?.robot1?.base_pose || [0, 0, 0];
            console.log('  设置机器人1基座位姿:', basePose1);
            if (Array.isArray(basePose1)) {
                if (basePose1.length === 8) {
                    // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                    if (window.DQModule && window.DQModule.DQWrapper) {
                        try {
                            console.log(`  机器人1原始双四元数: [${basePose1.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            // 创建双四元数并归一化（必须归一化才能提取旋转和平移）
                            let dq = window.DQModule.DQWrapper.createFromArray(basePose1);
                            dq = window.DQModule.DQWrapper.normalize(dq);
                            
                            const rotation = window.DQModule.DQWrapper.getRotation(dq);
                            const translation = window.DQModule.DQWrapper.getTranslation(dq);
                            
                            console.log(`  机器人1提取的旋转: [${rotation.map(v => v.toFixed(6)).join(', ')}]`);
                            console.log(`  机器人1提取的平移: [${translation.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            const [qw, qx, qy, qz] = rotation;
                            const [, tx, ty, tz] = translation;
                            
                            robot1.position.set(tx || 0, ty || 0, tz || 0);
                            robot1.quaternion.set(qx, qy, qz, qw);
                            console.log(`  机器人1应用的位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                            console.log(`  机器人1应用的旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                        } catch (error) {
                            console.warn('  无法从双四元数提取位姿:', error);
                        }
                    } else {
                        console.warn('  DQModule 不可用，无法从双四元数提取位姿');
                    }
                } else if (basePose1.length === 7) {
                    // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                    const [qw, qx, qy, qz, tx, ty, tz] = basePose1;
                    robot1.position.set(tx || 0, ty || 0, tz || 0);
                    robot1.quaternion.set(qx, qy, qz, qw);
                    console.log(`  机器人1位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                    console.log(`  机器人1旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                } else if (basePose1.length >= 3) {
                    // 仅平移 [tx, ty, tz] 或 [tx, ty, tz, ...]
                    const [tx, ty, tz] = basePose1;
                    robot1.position.set(tx || 0, ty || 0, tz || 0);
                    console.log(`  机器人1位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                }
            }
            this.scene.add(robot1);
            console.log('  ✓ 机器人1已添加到场景，场景对象数:', this.scene.children.length);
            
            // 设置机器人2
            this.robot2URDF = robot2;
            const basePose2 = config?.robot2?.base_pose || [0, 0, 0];
            console.log('  设置机器人2基座位姿:', basePose2);
            if (Array.isArray(basePose2)) {
                if (basePose2.length === 8) {
                    // 双四元数格式 [qw, qx, qy, qz, qw', qx', qy', qz']
                    if (window.DQModule && window.DQModule.DQWrapper) {
                        try {
                            console.log(`  机器人2原始双四元数: [${basePose2.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            // 创建双四元数并归一化（必须归一化才能提取旋转和平移）
                            let dq = window.DQModule.DQWrapper.createFromArray(basePose2);
                            dq = window.DQModule.DQWrapper.normalize(dq);
                            
                            const rotation = window.DQModule.DQWrapper.getRotation(dq);
                            const translation = window.DQModule.DQWrapper.getTranslation(dq);
                            
                            console.log(`  机器人2提取的旋转: [${rotation.map(v => v.toFixed(6)).join(', ')}]`);
                            console.log(`  机器人2提取的平移: [${translation.map(v => v.toFixed(6)).join(', ')}]`);
                            
                            const [qw, qx, qy, qz] = rotation;
                            const [, tx, ty, tz] = translation;
                            
                            robot2.position.set(tx || 0, ty || 0, tz || 0);
                            robot2.quaternion.set(qx, qy, qz, qw);
                            console.log(`  机器人2应用的位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                            console.log(`  机器人2应用的旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                        } catch (error) {
                            console.warn('  无法从双四元数提取位姿:', error);
                        }
                    } else {
                        console.warn('  DQModule 不可用，无法从双四元数提取位姿');
                    }
                } else if (basePose2.length === 7) {
                    // 完整位姿 [qw, qx, qy, qz, tx, ty, tz]
                    const [qw, qx, qy, qz, tx, ty, tz] = basePose2;
                    robot2.position.set(tx || 0, ty || 0, tz || 0);
                    robot2.quaternion.set(qx, qy, qz, qw);
                    console.log(`  机器人2位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                    console.log(`  机器人2旋转: (${qw}, ${qx}, ${qy}, ${qz})`);
                } else if (basePose2.length >= 3) {
                    // 仅平移 [tx, ty, tz] 或 [tx, ty, tz, ...]
                    const [tx, ty, tz] = basePose2;
                    robot2.position.set(tx || 0, ty || 0, tz || 0);
                    console.log(`  机器人2位置: (${tx || 0}, ${ty || 0}, ${tz || 0})`);
                }
            }
            this.scene.add(robot2);
            console.log('  ✓ 机器人2已添加到场景，场景对象数:', this.scene.children.length);
            console.log('  场景中的所有对象:', this.scene.children.map(c => c.name || c.type || c.constructor.name));
        }
        
        // 创建末端执行器标记
        this.createDualArmEndEffectors(config);
        
        // 创建绝对位姿和相对位姿可视化
        this.createDualArmVisualizations(config);
        
        console.log('✓ 双臂 URDF 加载完成，最终场景对象数:', this.scene.children.length);
        return true;
    }
    
    /**
     * 创建双臂末端执行器标记
     */
    createDualArmEndEffectors(config) {
        // 先清理旧的末端执行器标记
        if (this.robot1EndEffector) {
            this.scene.remove(this.robot1EndEffector);
            this.robot1EndEffector = null;
        }
        if (this.robot2EndEffector) {
            this.scene.remove(this.robot2EndEffector);
            this.robot2EndEffector = null;
        }
        
        const visConfig = config?.visualization || {};
        const size = visConfig.end_effector_size || 0.02;
        
        // 处理颜色（可能是字符串 "#3b82f6" 或数字 0x3b82f6）
        let color1 = visConfig.robot1_color || 0x3b82f6;
        let color2 = visConfig.robot2_color || 0x10b981;
        
        if (typeof color1 === 'string' && color1.startsWith('#')) {
            color1 = parseInt(color1.substring(1), 16);
        }
        if (typeof color2 === 'string' && color2.startsWith('#')) {
            color2 = parseInt(color2.substring(1), 16);
        }
        
        // 机器人1末端执行器
        const geometry1 = new THREE.SphereGeometry(size, 32, 32);
        const material1 = new THREE.MeshPhongMaterial({ 
            color: color1,
            emissive: color1,
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.9
        });
        material1.depthTest = false;
        material1.depthWrite = false;
        this.robot1EndEffector = new THREE.Mesh(geometry1, material1);
        this.robot1EndEffector.renderOrder = 999;
        const axes1 = new THREE.AxesHelper(size * 2);
        if (axes1.material) {
            axes1.material.depthTest = false;
            axes1.material.depthWrite = false;
            axes1.material.transparent = true;
            axes1.material.opacity = 0.95;
        }
        this.robot1EndEffector.add(axes1);
        this.keepMeshWorldScale(this.robot1EndEffector);
        this.scene.add(this.robot1EndEffector);
        
        // 机器人2末端执行器
        const geometry2 = new THREE.SphereGeometry(size, 32, 32);
        const material2 = new THREE.MeshPhongMaterial({ 
            color: color2,
            emissive: color2,
            emissiveIntensity: 0.5,
            transparent: true,
            opacity: 0.9
        });
        material2.depthTest = false;
        material2.depthWrite = false;
        this.robot2EndEffector = new THREE.Mesh(geometry2, material2);
        this.robot2EndEffector.renderOrder = 999;
        const axes2 = new THREE.AxesHelper(size * 2);
        if (axes2.material) {
            axes2.material.depthTest = false;
            axes2.material.depthWrite = false;
            axes2.material.transparent = true;
            axes2.material.opacity = 0.95;
        }
        this.robot2EndEffector.add(axes2);
        this.keepMeshWorldScale(this.robot2EndEffector);
        this.scene.add(this.robot2EndEffector);
    }
    
    /**
     * 创建双臂可视化元素（绝对位姿、相对位姿）
     */
    createDualArmVisualizations(config) {
        // 先清理旧的可视化元素
        if (this.absolutePoseMarker) {
            this.scene.remove(this.absolutePoseMarker);
            this.absolutePoseMarker = null;
        }
        if (this.relativePoseLine) {
            this.scene.remove(this.relativePoseLine);
            this.relativePoseLine = null;
        }
        
        const visConfig = config?.visualization || {};
        
        // 绝对位姿标记（两个末端执行器的中点）
        if (visConfig.show_absolute_pose !== false) {
            let color = visConfig.absolute_pose_color || 0xfbbf24;
            if (typeof color === 'string' && color.startsWith('#')) {
                color = parseInt(color.substring(1), 16);
            }
            
            // 创建标记组
            this.absolutePoseMarker = new THREE.Group();
            // 确保Group的scale始终为(1,1,1)，避免变形
            this.absolutePoseMarker.scale.set(1, 1, 1);
            
            // 创建球体
            const sphereGeometry = new THREE.SphereGeometry(0.02, 16, 16);
            const sphereMaterial = new THREE.MeshPhongMaterial({ 
                color: color,
                emissive: color,
                emissiveIntensity: 0.7,
                transparent: true,
                opacity: 0.9
            });
            sphereMaterial.depthTest = false;
            sphereMaterial.depthWrite = false;
            const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
            sphere.renderOrder = 998;
            // 确保球体scale为(1,1,1)
            sphere.scale.set(1, 1, 1);
            this.absolutePoseMarker.add(sphere);
            
            // 添加坐标轴（稍大一些以便清晰看到）
            const axesHelper = new THREE.AxesHelper(0.08);
            axesHelper.renderOrder = 999;
            // 设置坐标轴线条更粗一些
            axesHelper.material.linewidth = 2;
            if (axesHelper.material) {
                axesHelper.material.depthTest = false;
                axesHelper.material.depthWrite = false;
                axesHelper.material.transparent = true;
                axesHelper.material.opacity = 0.95;
            }
            // 确保坐标轴scale为(1,1,1)
            axesHelper.scale.set(1, 1, 1);
            this.absolutePoseMarker.add(axesHelper);
            
            this.absolutePoseMarker.renderOrder = 998;
            this.scene.add(this.absolutePoseMarker);
            
            console.log('  ✓ 绝对位姿标记已创建（带坐标轴）');
        }
        
        // 相对位姿连线（从robot1到robot2）
        if (visConfig.show_relative_pose !== false) {
            let color = visConfig.relative_pose_color || 0xef4444;
            if (typeof color === 'string' && color.startsWith('#')) {
                color = parseInt(color.substring(1), 16);
            }
            const geometry = new THREE.BufferGeometry();
            const positions = new Float32Array(6); // 两个点，每个3个坐标
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            const material = new THREE.LineBasicMaterial({ 
                color: color,
                linewidth: 2,
                transparent: true,
                opacity: 0.6
            });
            this.relativePoseLine = new THREE.Line(geometry, material);
            this.relativePoseLine.renderOrder = 997;
            this.scene.add(this.relativePoseLine);
        }
    }

    keepMeshWorldScale(mesh) {
        if (!mesh) {
            return;
        }
        mesh.scale.set(1, 1, 1);
        mesh.onBeforeRender = () => {
            const worldScale = new THREE.Vector3();
            mesh.getWorldScale(worldScale);
            mesh.scale.set(
                worldScale.x ? 1 / worldScale.x : 1,
                worldScale.y ? 1 / worldScale.y : 1,
                worldScale.z ? 1 / worldScale.z : 1
            );
        };
    }
    
    /**
     * 更新机器人位姿（支持单臂和双臂模式）
     */
    updateRobotPose(jointAngles, pose, jointNamesOrder, robotIndex = 0) {
        if (this.dualArmMode) {
            // 双臂模式：robotIndex 指定更新哪个机器人 (0 或 1)
            this.updateDualArmPose(jointAngles, pose, jointNamesOrder, robotIndex);
        } else {
            // 单臂模式：保持原有逻辑
            this.updateSingleArmPose(jointAngles, pose, jointNamesOrder);
        }
    }
    
    /**
     * 更新单臂机器人位姿
     */
    updateSingleArmPose(jointAngles, pose, jointNamesOrder) {
        if (!pose || !pose.translation || !pose.rotation) {
            return;
        }
        
        try {
            // 获取 DQ 位置 (索引 1,2,3 = x,y,z)
            // DQ translation 返回格式: [w, x, y, z]，其中 w=0（纯平移）
            // 注意：DQ Robotics 使用的是 Z-up 坐标系，与 Three.js 一致
            let x = pose.translation[1] || 0;
            let y = pose.translation[2] || 0;
            let z = pose.translation[3] || 0;
            
            // 调试：打印原始坐标（仅在开发模式下）
            // 注意：DQ Robotics 的 fkm() 返回的坐标是相对于 reference_frame 的
            // 如果设置了 set_reference_frame(base)，那么返回的坐标是相对于基座坐标系的
            // 如果基座有旋转，这里的坐标需要变换到世界坐标系
            // 但是，由于我们同时设置了 set_base_frame 和 set_reference_frame 为同一个值，
            // fkm() 应该返回相对于世界坐标系的坐标（因为 reference_frame 已经包含了基座变换）
            
            // 更新末端执行器位置
            // 使用DQ Robotics通过MDH参数计算的正向运动学结果
            if (this.endEffector) {
                // 使用正向运动学计算的位置（已经包含了末端偏移，如果设置了的话）
                // DQ Robotics 返回的坐标格式：[w, x, y, z]，其中 w=0（纯平移）
                // 索引 1,2,3 分别对应 x, y, z
                this.endEffector.position.set(x, y, z);
                
                // 更新旋转
                const qw = pose.rotation[0] || 1;
                const qx = pose.rotation[1] || 0;
                const qy = pose.rotation[2] || 0;
                const qz = pose.rotation[3] || 0;
                
                this.endEffector.quaternion.set(qx, qy, qz, qw);
            }
            
            // 更新 URDF 关节
            if (this.urdfRobot && jointAngles) {
                this.updateURDFJoints(this.urdfRobot, jointAngles, jointNamesOrder);
            }
            
            // 更新URDF坐标系显示
            if (this.showURDFFrames) {
                this.updateAllURDFFrames();
            }
            
            // 更新拖动控制位置（确保一直跟随末端执行器）
            if (this.dragEndEffectorMode) {
                this.updateGizmoPosition();
            }
            
        } catch (error) {
            console.error('更新单臂位姿失败:', error);
        }
    }
    
    /**
     * 更新双臂机器人位姿
     */
    updateDualArmPose(jointAngles, pose, jointNamesOrder, robotIndex) {
        if (!pose || !pose.translation || !pose.rotation) {
            return;
        }
        
        try {
            const x = pose.translation[1] || 0;
            const y = pose.translation[2] || 0;
            const z = pose.translation[3] || 0;
            const qw = pose.rotation[0] || 1;
            const qx = pose.rotation[1] || 0;
            const qy = pose.rotation[2] || 0;
            const qz = pose.rotation[3] || 0;
            
            if (robotIndex === 0) {
                // 更新机器人1
                if (this.robot1EndEffector) {
                    this.robot1EndEffector.position.set(x, y, z);
                    this.robot1EndEffector.quaternion.set(qx, qy, qz, qw);
                }
                if (this.robot1URDF && jointAngles) {
                    this.updateURDFJoints(this.robot1URDF, jointAngles, jointNamesOrder);
                }
                // 更新所有URDF坐标系显示
                if (this.showURDFFrames) {
                    this.updateAllURDFFrames();
                }
            } else if (robotIndex === 1) {
                // 更新机器人2
                if (this.robot2EndEffector) {
                    this.robot2EndEffector.position.set(x, y, z);
                    this.robot2EndEffector.quaternion.set(qx, qy, qz, qw);
                }
                if (this.robot2URDF && jointAngles) {
                    this.updateURDFJoints(this.robot2URDF, jointAngles, jointNamesOrder);
                }
                // 更新所有URDF坐标系显示（已在上面统一更新）
            }
        } catch (error) {
            console.error(`更新机器人${robotIndex + 1}位姿失败:`, error);
        }
    }
    
    /**
     * 更新双臂绝对位姿和相对位姿可视化
     */
    updateDualArmRelativeVisualizations(pose1, pose2, absolutePose = null, relativePose = null) {
        if (!this.dualArmMode) return;
        
        try {
            // 更新绝对位姿标记（两个末端执行器的中点）
            if (this.absolutePoseMarker && pose1 && pose2) {
                const x1 = pose1.translation[1] || 0;
                const y1 = pose1.translation[2] || 0;
                const z1 = pose1.translation[3] || 0;
                const x2 = pose2.translation[1] || 0;
                const y2 = pose2.translation[2] || 0;
                const z2 = pose2.translation[3] || 0;
                
                const midX = (x1 + x2) / 2;
                const midY = (y1 + y2) / 2;
                const midZ = (z1 + z2) / 2;
                
                // 更新位置
                this.absolutePoseMarker.position.set(midX, midY, midZ);
                
                // 确保scale始终为(1,1,1)，避免变形为椭球
                this.absolutePoseMarker.scale.set(1, 1, 1);
                
                // 更新旋转（如果有计算好的绝对位姿四元数）
                if (absolutePose && absolutePose.rotation) {
                    const qw = absolutePose.rotation[0] || 1;
                    const qx = absolutePose.rotation[1] || 0;
                    const qy = absolutePose.rotation[2] || 0;
                    const qz = absolutePose.rotation[3] || 0;
                    
                    // 归一化四元数，确保旋转正确
                    const len = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                    if (len > 0.001) {
                        this.absolutePoseMarker.quaternion.set(
                            qx / len, 
                            qy / len, 
                            qz / len, 
                            qw / len
                        );
                    } else {
                        // 如果四元数无效，使用单位四元数
                        this.absolutePoseMarker.quaternion.set(0, 0, 0, 1);
                    }
                } else {
                    // 如果没有提供绝对位姿旋转，使用两个末端的平均旋转
                    const qw1 = pose1.rotation[0] || 1;
                    const qx1 = pose1.rotation[1] || 0;
                    const qy1 = pose1.rotation[2] || 0;
                    const qz1 = pose1.rotation[3] || 0;
                    const qw2 = pose2.rotation[0] || 1;
                    const qx2 = pose2.rotation[1] || 0;
                    const qy2 = pose2.rotation[2] || 0;
                    const qz2 = pose2.rotation[3] || 0;
                    
                    // 平均四元数（简单平均）
                    let avgQW = (qw1 + qw2) / 2;
                    let avgQX = (qx1 + qx2) / 2;
                    let avgQY = (qy1 + qy2) / 2;
                    let avgQZ = (qz1 + qz2) / 2;
                    
                    // 归一化
                    const len = Math.sqrt(avgQW * avgQW + avgQX * avgQX + avgQY * avgQY + avgQZ * avgQZ);
                    if (len > 0.001) {
                        this.absolutePoseMarker.quaternion.set(
                            avgQX / len, 
                            avgQY / len, 
                            avgQZ / len, 
                            avgQW / len
                        );
                    } else {
                        // 如果平均后接近零，使用默认单位四元数
                        this.absolutePoseMarker.quaternion.set(0, 0, 0, 1);
                    }
                }
            }
            
            // 更新相对位姿连线
            if (this.relativePoseLine && pose1 && pose2) {
                const x1 = pose1.translation[1] || 0;
                const y1 = pose1.translation[2] || 0;
                const z1 = pose1.translation[3] || 0;
                const x2 = pose2.translation[1] || 0;
                const y2 = pose2.translation[2] || 0;
                const z2 = pose2.translation[3] || 0;
                
                const positions = this.relativePoseLine.geometry.attributes.position;
                positions.array[0] = x1;
                positions.array[1] = y1;
                positions.array[2] = z1;
                positions.array[3] = x2;
                positions.array[4] = y2;
                positions.array[5] = z2;
                positions.needsUpdate = true;
            }
        } catch (error) {
            console.error('更新双臂可视化失败:', error);
        }
    }
    
    /**
     * 更新 URDF 关节角度（通用方法）
     */
    updateURDFJoints(urdfRobot, jointAngles, jointNamesOrder) {
        if (!urdfRobot || !jointAngles) return;
        
        if (Array.isArray(jointNamesOrder) && jointNamesOrder.length > 0) {
            jointNamesOrder.forEach((name, i) => {
                const j = urdfRobot.joints[name];
                if (j && (j.jointType === 'revolute' || j.jointType === 'continuous' || j.jointType === 'prismatic')) {
                    j.setJointValue(jointAngles[i] || 0);
                }
            });
        } else {
            // 回退：自动查找所有旋转关节
            const allJoints = Object.keys(urdfRobot.joints);
            const revoluteJoints = allJoints.filter(name => {
                const joint = urdfRobot.joints[name];
                return joint.jointType === 'revolute' || joint.jointType === 'continuous';
            });
            revoluteJoints.forEach((name, i) => {
                if (i < jointAngles.length && urdfRobot.joints[name]) {
                    urdfRobot.joints[name].setJointValue(jointAngles[i]);
                }
            });
        }
        
        // 更新URDF变换矩阵
        if (typeof urdfRobot.updateMatrixWorld === 'function') {
            urdfRobot.updateMatrixWorld(true);
        }
        
        // 更新URDF坐标系显示（更新所有坐标系，因为可能影响两个机器人）
        if (this.showURDFFrames) {
            this.updateAllURDFFrames();
        }
    }
    
    /**
     * 显示/隐藏MDH坐标系
     * @param {Array} mdhFrames - MDH坐标系数组，每个元素包含 {origin, xAxis, yAxis, zAxis}
     */
    setMDHFrames(mdhFrames) {
        console.log('setMDHFrames 被调用，mdhFrames数量:', mdhFrames ? mdhFrames.length : 0);
        console.log('当前 showMDHFrames 状态:', this.showMDHFrames);
        
        // 先移除旧的MDH坐标系
        if (this.mdhFramesGroup) {
            this.scene.remove(this.mdhFramesGroup);
            this.mdhFramesGroup = null;
        }
        
        if (!mdhFrames || mdhFrames.length === 0) {
            console.warn('MDH坐标系数据为空，无法创建');
            return;
        }
        
        // 存储最后一个MDH坐标系的位置，用于更新末端执行器
        if (mdhFrames.length > 0) {
            const lastFrame = mdhFrames[mdhFrames.length - 1];
            this.lastMDHFrameOrigin = lastFrame.origin.clone();
            console.log(`存储最后一个MDH坐标系位置: (${this.lastMDHFrameOrigin.x.toFixed(4)}, ${this.lastMDHFrameOrigin.y.toFixed(4)}, ${this.lastMDHFrameOrigin.z.toFixed(4)})`);
        }
        
        // 创建MDH坐标系组
        this.mdhFramesGroup = new THREE.Group();
        this.mdhFramesGroup.name = 'MDHFrames';
        this.mdhFramesGroup.renderOrder = 1000; // 设置高渲染优先级，确保在机器人之前渲染
        
        const axisLength = 0.1; // 正常大小的坐标轴长度
        
        mdhFrames.forEach((frame, index) => {
            const { origin, xAxis, yAxis, zAxis } = frame;
            
            // 创建坐标系组
            const frameGroup = new THREE.Group();
            frameGroup.position.copy(origin);
            frameGroup.renderOrder = 1000; // 高渲染优先级
            
            // 创建旋转矩阵（从X、Y、Z轴构建）
            const rotationMatrix = new THREE.Matrix4();
            rotationMatrix.makeBasis(xAxis, yAxis, zAxis);
            frameGroup.setRotationFromMatrix(rotationMatrix);
            
            // 使用AxesHelper创建坐标轴（正常粗细）
            const axesHelper = new THREE.AxesHelper(axisLength);
            // 设置线条材质，禁用深度测试以确保始终显示在前面
            axesHelper.material.linewidth = 2;
            axesHelper.material.depthTest = false; // 禁用深度测试，始终显示在前面
            axesHelper.material.depthWrite = false;
            axesHelper.renderOrder = 1000; // 确保在机器人之前渲染
            frameGroup.add(axesHelper);
            
            // 添加标签（使用Sprite）
            const createAxisLabel = (text, color, offset) => {
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');
                canvas.width = 64;
                canvas.height = 64;
                
                context.fillStyle = color;
                context.font = 'Bold 48px Arial';
                context.textAlign = 'center';
                context.textBaseline = 'middle';
                context.fillText(text, 32, 32);
                
                const texture = new THREE.CanvasTexture(canvas);
                const spriteMaterial = new THREE.SpriteMaterial({ 
                    map: texture,
                    depthTest: false, // 禁用深度测试，始终显示在前面
                    depthWrite: false,
                    transparent: true
                });
                const sprite = new THREE.Sprite(spriteMaterial);
                sprite.position.copy(offset);
                sprite.scale.set(0.02, 0.02, 1);
                sprite.renderOrder = 1001; // 设置更高的渲染顺序，确保在最前面
                frameGroup.add(sprite);
            };
            
            createAxisLabel('X', '#ef4444', new THREE.Vector3(0, axisLength + 0.02, 0));
            createAxisLabel('Y', '#10b981', new THREE.Vector3(-axisLength - 0.02, 0, 0));
            createAxisLabel('Z', '#3b82f6', new THREE.Vector3(0, 0, axisLength + 0.02));
            
            // 添加坐标系索引标签
            const createFrameLabel = (text) => {
                const canvas = document.createElement('canvas');
                const context = canvas.getContext('2d');
                // 增大canvas尺寸，确保文本完全显示
                canvas.width = 512;
                canvas.height = 256;
                
                // 清除背景（透明）
                context.clearRect(0, 0, canvas.width, canvas.height);
                
                context.fillStyle = '#ffffff';
                context.font = 'Bold 120px Arial'; // 增大字体
                context.textAlign = 'center';
                context.textBaseline = 'middle';
                // 在canvas中心绘制文本
                context.fillText(text, canvas.width / 2, canvas.height / 2);
                
                const texture = new THREE.CanvasTexture(canvas);
                texture.needsUpdate = true;
                const spriteMaterial = new THREE.SpriteMaterial({ 
                    map: texture,
                    depthTest: false, // 禁用深度测试，始终显示在前面
                    depthWrite: false,
                    transparent: true
                });
                const sprite = new THREE.Sprite(spriteMaterial);
                // Frame标签位置：在Y轴负方向（在坐标系本地空间中）
                sprite.position.set(0, -axisLength - 0.05, 0);
                sprite.scale.set(0.08, 0.04, 1); // 相应增大sprite尺寸
                sprite.renderOrder = 1001; // 设置更高的渲染顺序，确保在最前面
                frameGroup.add(sprite);
            };
            
            createFrameLabel(`Frame ${index}`);
            
            this.mdhFramesGroup.add(frameGroup);
        });
        
        // 根据显示状态决定是否添加到场景
        // 确保MDH坐标系在机器人之前添加到场景（渲染顺序）
        if (this.showMDHFrames) {
            console.log('showMDHFrames 为 true，将MDH坐标系添加到场景');
            // 如果场景中已有MDH坐标系组，先移除
            const existingFrames = this.scene.getObjectByName('MDHFrames');
            if (existingFrames) {
                this.scene.remove(existingFrames);
            }
            // 将MDH坐标系添加到场景
            this.scene.add(this.mdhFramesGroup);
            console.log('MDH坐标系已添加到场景，场景对象数:', this.scene.children.length);
        } else {
            console.log('showMDHFrames 为 false，MDH坐标系已创建但未添加到场景');
            console.log('提示：请打开"显示MDH坐标系"开关来显示坐标系');
        }
        
        console.log(`✓ 已创建 ${mdhFrames.length} 个MDH坐标系`);
    }
    
    /**
     * 切换MDH坐标系显示
     * @param {boolean} show - 是否显示
     */
    toggleMDHFrames(show) {
        this.showMDHFrames = show !== undefined ? show : !this.showMDHFrames;
        console.log('toggleMDHFrames 被调用，新状态:', this.showMDHFrames);
        
        if (this.mdhFramesGroup) {
            if (this.showMDHFrames) {
                // 检查是否已经在场景中
                if (!this.scene.getObjectByName('MDHFrames')) {
                    this.scene.add(this.mdhFramesGroup);
                    console.log('MDH坐标系已添加到场景');
                } else {
                    console.log('MDH坐标系已在场景中');
                }
            } else {
                this.scene.remove(this.mdhFramesGroup);
                console.log('MDH坐标系已从场景移除');
            }
        } else {
            console.warn('MDH坐标系组不存在，无法切换显示');
            console.warn('提示：请先提取MDH参数以创建坐标系');
        }
    }
    
    /**
     * 初始化URDF坐标系显示（支持单臂和双臂模式）
     * @param {Object} urdfRobot - URDF机器人对象（单臂模式）或null（双臂模式）
     * @param {Array<string>} linkNames - 要显示的link名称列表（单臂模式，可选）
     * @param {Object} robot1URDF - 机器人1的URDF对象（双臂模式，可选）
     * @param {Object} robot2URDF - 机器人2的URDF对象（双臂模式，可选）
     */
    initURDFFrames(urdfRobot = null, linkNames = null, robot1URDF = null, robot2URDF = null) {
        // 先移除旧的坐标系
        if (this.urdfFramesGroup) {
            this.scene.remove(this.urdfFramesGroup);
            this.urdfFramesGroup = null;
        }
        this.urdfFrameObjects.clear();
        
        // 创建URDF坐标系组
        this.urdfFramesGroup = new THREE.Group();
        this.urdfFramesGroup.name = 'URDFFrames';
        this.urdfFramesGroup.renderOrder = 1000;
        
        const axisLength = 0.08; // 坐标轴长度
        
        // 判断是单臂还是双臂模式
        const isDualArm = this.dualArmMode && (robot1URDF || this.robot1URDF) && (robot2URDF || this.robot2URDF);
        
        if (isDualArm) {
            // 双臂模式：同时初始化两个机器人的坐标系
            const r1URDF = robot1URDF || this.robot1URDF;
            const r2URDF = robot2URDF || this.robot2URDF;
            
            if (!r1URDF || !r1URDF.links || !r2URDF || !r2URDF.links) {
                console.warn('双臂模式下URDF机器人对象无效，无法创建坐标系');
                return;
            }
            
            // 获取两个机器人的link列表
            const robot1Links = Object.keys(r1URDF.links);
            const robot2Links = Object.keys(r2URDF.links);
            
            this.urdfFrameRobot1Links = robot1Links.slice();
            this.urdfFrameRobot2Links = robot2Links.slice();
            
            // 创建机器人1的坐标系（使用robot1前缀）
            robot1Links.forEach((linkName) => {
                const link = r1URDF.links[linkName];
                if (!link) return;
                
                const frameKey = `robot1_${linkName}`;
                const frameGroup = this.createFrameGroup(frameKey, linkName, axisLength, '#3b82f6'); // 蓝色
                this.urdfFrameObjects.set(frameKey, frameGroup);
                this.urdfFramesGroup.add(frameGroup);
            });
            
            // 创建机器人2的坐标系（使用robot2前缀）
            robot2Links.forEach((linkName) => {
                const link = r2URDF.links[linkName];
                if (!link) return;
                
                const frameKey = `robot2_${linkName}`;
                const frameGroup = this.createFrameGroup(frameKey, linkName, axisLength, '#10b981'); // 绿色
                this.urdfFrameObjects.set(frameKey, frameGroup);
                this.urdfFramesGroup.add(frameGroup);
            });
            
            console.log(`✓ 已创建双臂URDF坐标系: 机器人1(${robot1Links.length}个), 机器人2(${robot2Links.length}个)`);
        } else {
            // 单臂或人形模式：优先使用 humanoidURDF
            const robot = urdfRobot || this.humanoidURDF || this.urdfRobot;
            if (!robot || !robot.links) {
                console.warn('URDF机器人对象无效，无法创建坐标系');
                return;
            }
            
            // 确定要显示的link列表
            let linksToDisplay = [];
            if (linkNames && Array.isArray(linkNames) && linkNames.length > 0) {
                linksToDisplay = linkNames;
            } else {
                linksToDisplay = Object.keys(robot.links);
            }
            
            this.urdfFrameLinkNames = linksToDisplay.slice();
            
            linksToDisplay.forEach((linkName) => {
                const link = robot.links[linkName];
                if (!link) {
                    console.warn(`Link ${linkName} 未找到`);
                    return;
                }
                
                const frameGroup = this.createFrameGroup(linkName, linkName, axisLength, '#ffffff'); // 白色
                this.urdfFrameObjects.set(linkName, frameGroup);
                this.urdfFramesGroup.add(frameGroup);
            });
            
            console.log(`✓ 已创建 ${linksToDisplay.length} 个URDF坐标系`);
        }
        
        // 根据显示状态决定是否添加到场景
        if (this.showURDFFrames) {
            this.scene.add(this.urdfFramesGroup);
        }
        
        // 初始化时更新一次位置
        this.updateAllURDFFrames();
    }
    
    /**
     * 创建单个坐标系组
     * @param {string} frameKey - 坐标系键名（用于存储）
     * @param {string} labelText - 标签文本
     * @param {number} axisLength - 坐标轴长度
     * @param {string} labelColor - 标签颜色（可选，默认白色）
     * @returns {THREE.Group} 坐标系组
     */
    createFrameGroup(frameKey, labelText, axisLength, labelColor = '#ffffff') {
        const frameGroup = new THREE.Group();
        frameGroup.name = `URDFFrame_${frameKey}`;
        frameGroup.renderOrder = 1000;
        
        // 使用AxesHelper创建坐标轴
        const axesHelper = new THREE.AxesHelper(axisLength);
        axesHelper.material.linewidth = 2;
        axesHelper.material.depthTest = false;
        axesHelper.material.depthWrite = false;
        axesHelper.renderOrder = 1000;
        frameGroup.add(axesHelper);
        
        // 添加标签
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 512;
        canvas.height = 128;
        
        context.clearRect(0, 0, canvas.width, canvas.height);
        context.fillStyle = labelColor;
        context.font = 'Bold 64px Arial';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(labelText, canvas.width / 2, canvas.height / 2);
        
        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;
        const spriteMaterial = new THREE.SpriteMaterial({
            map: texture,
            depthTest: false,
            depthWrite: false,
            transparent: true
        });
        const sprite = new THREE.Sprite(spriteMaterial);
        sprite.position.set(0, -axisLength - 0.03, 0);
        sprite.scale.set(0.1, 0.05, 1);
        sprite.renderOrder = 1001;
        frameGroup.add(sprite);
        
        return frameGroup;
    }
    
    /**
     * 更新所有URDF坐标系位置（当关节移动时调用）
     * 自动判断单臂或双臂模式
     */
    updateAllURDFFrames() {
        if (!this.urdfFramesGroup) {
            return;
        }
        
        if (this.dualArmMode && this.robot1URDF && this.robot2URDF) {
            // 双臂模式：同时更新两个机器人的坐标系
            this.updateURDFFramesForRobot(this.robot1URDF, 'robot1', this.urdfFrameRobot1Links);
            this.updateURDFFramesForRobot(this.robot2URDF, 'robot2', this.urdfFrameRobot2Links);
        } else {
            // 单臂或人形模式：优先使用 humanoidURDF
            const urdfRobot = this.humanoidURDF || this.urdfRobot;
            if (urdfRobot && urdfRobot.links) {
                this.updateURDFFramesForRobot(urdfRobot, null, this.urdfFrameLinkNames);
            }
        }
    }
    
    /**
     * 更新指定机器人的URDF坐标系位置
     * @param {Object} urdfRobot - URDF机器人对象
     * @param {string} robotPrefix - 机器人前缀（'robot1', 'robot2' 或 null表示单臂）
     * @param {Array<string>} linkNames - 要更新的link名称列表
     */
    updateURDFFramesForRobot(urdfRobot, robotPrefix, linkNames) {
        if (!urdfRobot || !urdfRobot.links || !linkNames || linkNames.length === 0) {
            return;
        }
        
        // 确保变换矩阵是最新的
        if (typeof urdfRobot.updateMatrixWorld === 'function') {
            urdfRobot.updateMatrixWorld(true);
        }
        
        // 更新每个link的坐标系位置
        linkNames.forEach((linkName) => {
            const link = urdfRobot.links[linkName];
            if (!link) {
                return;
            }
            
            // 构建frame key
            const frameKey = robotPrefix ? `${robotPrefix}_${linkName}` : linkName;
            const frameGroup = this.urdfFrameObjects.get(frameKey);
            
            if (!frameGroup) {
                return;
            }
            
            // 获取link的世界变换矩阵
            if (link.matrixWorld) {
                // 提取位置和旋转
                const position = new THREE.Vector3();
                const quaternion = new THREE.Quaternion();
                const scale = new THREE.Vector3();
                
                link.matrixWorld.decompose(position, quaternion, scale);
                
                // 更新坐标系的位置和旋转
                frameGroup.position.copy(position);
                frameGroup.quaternion.copy(quaternion);
            } else {
                // 如果matrixWorld不存在，尝试使用position和quaternion
                if (link.position) {
                    frameGroup.position.copy(link.position);
                }
                if (link.quaternion) {
                    frameGroup.quaternion.copy(link.quaternion);
                }
            }
        });
    }
    
    /**
     * 更新URDF坐标系位置（当关节移动时调用，保持向后兼容）
     * @param {Object} urdfRobot - URDF机器人对象
     */
    updateURDFFrames(urdfRobot) {
        // 向后兼容：如果提供了urdfRobot，更新对应的坐标系
        // 但更推荐使用updateAllURDFFrames()来自动处理所有坐标系
        if (!this.urdfFramesGroup) {
            return;
        }
        
        if (this.dualArmMode) {
            // 双臂模式：根据提供的urdfRobot判断是哪个机器人
            if (urdfRobot === this.robot1URDF) {
                this.updateURDFFramesForRobot(urdfRobot, 'robot1', this.urdfFrameRobot1Links);
            } else if (urdfRobot === this.robot2URDF) {
                this.updateURDFFramesForRobot(urdfRobot, 'robot2', this.urdfFrameRobot2Links);
            }
        } else {
            // 单臂模式
            if (urdfRobot && urdfRobot.links) {
                this.updateURDFFramesForRobot(urdfRobot, null, this.urdfFrameLinkNames);
            }
        }
    }
    
    /**
     * 切换URDF坐标系显示
     * @param {boolean} show - 是否显示
     */
    toggleURDFFrames(show) {
        this.showURDFFrames = show !== undefined ? show : !this.showURDFFrames;
        console.log('toggleURDFFrames 被调用，新状态:', this.showURDFFrames);
        
        if (this.showURDFFrames) {
            // 如果坐标系还没有初始化，尝试初始化
            if (!this.urdfFramesGroup) {
                // 判断是单臂还是双臂还是人形模式
                if (this.dualArmMode && this.robot1URDF && this.robot2URDF) {
                    // 双臂模式：同时初始化两个机器人
                    this.initURDFFrames(null, null, this.robot1URDF, this.robot2URDF);
                    console.log('URDF坐标系已自动初始化（双臂模式）');
                } else {
                    // 单臂或人形模式：优先使用 humanoidURDF，其次 urdfRobot
                    const urdfRobot = this.humanoidURDF || this.urdfRobot;
                    if (urdfRobot && urdfRobot.links) {
                        const linkNames = Object.keys(urdfRobot.links);
                        this.initURDFFrames(urdfRobot, linkNames, null, null);
                        console.log(`URDF坐标系已自动初始化（${this.humanoidMode ? '人形' : '单臂'}模式，${linkNames.length}个link）`);
                    } else {
                        console.warn('无法自动初始化URDF坐标系：URDF机器人对象无效');
                        return;
                    }
                }
            }
            
            // 检查是否已经在场景中
            if (this.urdfFramesGroup && !this.scene.getObjectByName('URDFFrames')) {
                this.scene.add(this.urdfFramesGroup);
                console.log('URDF坐标系已添加到场景');
            }
            
            // 更新一次位置（更新所有坐标系）
            this.updateAllURDFFrames();
        } else {
            if (this.urdfFramesGroup) {
                this.scene.remove(this.urdfFramesGroup);
                console.log('URDF坐标系已从场景移除');
            }
        }
    }
    
    /**
     * 设置拖动末端执行器模式
     * @param {boolean} enabled - 是否启用拖动模式
     */
    setDragEndEffectorMode(enabled) {
        this.dragEndEffectorMode = enabled;
        if (enabled) {
            this.createEndEffectorGizmo();
        } else {
            this.removeEndEffectorGizmo();
        }
        console.log(`拖动末端执行器模式: ${enabled ? '启用' : '禁用'}`);
    }
    
    /**
     * 创建末端执行器拖动控制（MoveIt 风格）
     */
    createEndEffectorGizmo() {
        if (this.endEffectorGizmo) {
            this.scene.remove(this.endEffectorGizmo);
        }
        
        this.endEffectorGizmo = new THREE.Group();
        this.endEffectorGizmo.name = 'EndEffectorGizmo';
        
        const arrowLength = 0.3;   // 增大箭头长度
        const arrowRadius = 0.04;  // 增大箭头半径（更粗，更容易点击）
        const coneLength = 0.1;    // 增大箭头头部长度
        const coneRadius = 0.08;   // 增大箭头头部半径（更粗）
        const arrowOffset = 0.1;   // 箭头之间的间隔（从中心点偏移，增大间隔避免重叠）
        
        // 创建平移箭头（X, Y, Z 轴）
        const colors = {
            x: 0xff0000,  // 红色 - X轴
            y: 0x00ff00,  // 绿色 - Y轴
            z: 0x0000ff   // 蓝色 - Z轴
        };
        
        const axes = [
            { name: 'x', dir: new THREE.Vector3(1, 0, 0) },
            { name: 'y', dir: new THREE.Vector3(0, 1, 0) },
            { name: 'z', dir: new THREE.Vector3(0, 0, 1) }
        ];
        
        axes.forEach((axis) => {
            // 创建正方向箭头（从中心点向正方向偏移）
            const positiveStart = axis.dir.clone().multiplyScalar(arrowOffset);
            const arrowHelperPositive = new THREE.ArrowHelper(
                axis.dir,
                positiveStart,  // 起始位置偏移，避免重叠
                arrowLength,
                colors[axis.name],
                coneLength,
                arrowRadius
            );
            
            // 设置箭头材质（确保显示在最前端）
            arrowHelperPositive.line.material.transparent = true;
            arrowHelperPositive.line.material.opacity = 0.8;
            arrowHelperPositive.line.material.depthTest = false;  // 不进行深度测试，始终显示在前面
            arrowHelperPositive.line.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperPositive.line.renderOrder = 1000;  // 高渲染优先级
            
            arrowHelperPositive.cone.material.transparent = true;
            arrowHelperPositive.cone.material.opacity = 0.8;
            arrowHelperPositive.cone.material.depthTest = false;  // 不进行深度测试
            arrowHelperPositive.cone.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperPositive.cone.renderOrder = 1000;  // 高渲染优先级
            
            // 设置整个箭头组的渲染顺序
            arrowHelperPositive.renderOrder = 1000;
            
            arrowHelperPositive.userData = { 
                type: 'translation', 
                axis: axis.name, 
                direction: 'positive' 
            };
            
            // 创建负方向箭头（从中心点向负方向偏移）
            const negativeStart = axis.dir.clone().negate().multiplyScalar(arrowOffset);
            const arrowHelperNegative = new THREE.ArrowHelper(
                axis.dir.clone().negate(),  // 反向
                negativeStart,  // 起始位置偏移，避免重叠
                arrowLength,
                colors[axis.name],
                coneLength,
                arrowRadius
            );
            
            // 设置箭头材质（确保显示在最前端）
            arrowHelperNegative.line.material.transparent = true;
            arrowHelperNegative.line.material.opacity = 0.8;
            arrowHelperNegative.line.material.depthTest = false;  // 不进行深度测试，始终显示在前面
            arrowHelperNegative.line.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperNegative.line.renderOrder = 1000;  // 高渲染优先级
            
            arrowHelperNegative.cone.material.transparent = true;
            arrowHelperNegative.cone.material.opacity = 0.8;
            arrowHelperNegative.cone.material.depthTest = false;  // 不进行深度测试
            arrowHelperNegative.cone.material.depthWrite = false;  // 不写入深度缓冲
            arrowHelperNegative.cone.renderOrder = 1000;  // 高渲染优先级
            
            // 设置整个箭头组的渲染顺序
            arrowHelperNegative.renderOrder = 1000;
            
            arrowHelperNegative.userData = { 
                type: 'translation', 
                axis: axis.name, 
                direction: 'negative' 
            };
            
            // 存储箭头（使用对象存储正负两个方向）
            this.translationArrows[axis.name] = {
                positive: arrowHelperPositive,
                negative: arrowHelperNegative
            };
            
            this.endEffectorGizmo.add(arrowHelperPositive);
            this.endEffectorGizmo.add(arrowHelperNegative);
        });
        
        // 暂时禁用旋转箭头功能（只保留平移功能）
        // 创建旋转箭头（围绕 X, Y, Z 轴）
        // const rotationRadius = 0.12;
        // const rotationThickness = 0.008;
        // 
        // axes.forEach((axis) => {
        //     // 创建圆环（旋转箭头）
        //     const torusGeometry = new THREE.TorusGeometry(rotationRadius, rotationThickness, 16, 64);
        //     const torusMaterial = new THREE.MeshBasicMaterial({ 
        //         color: colors[axis.name],
        //         transparent: true,
        //         opacity: 0.6,
        //         side: THREE.DoubleSide
        //     });
        //     const torus = new THREE.Mesh(torusGeometry, torusMaterial);
        //     
        //     // 根据轴方向旋转圆环
        //     if (axis.name === 'x') {
        //         torus.rotateZ(Math.PI / 2);
        //     } else if (axis.name === 'y') {
        //         torus.rotateX(Math.PI / 2);
        //     }
        //     // z轴不需要旋转
        //     
        //     torus.userData = { type: 'rotation', axis: axis.name };
        //     
        //     this.rotationArrows[axis.name] = torus;
        //     this.endEffectorGizmo.add(torus);
        // });
        
        // 确保整个 gizmo 组显示在最前端
        this.endEffectorGizmo.renderOrder = 1000;
        
        // 遍历所有子对象，确保都设置正确的渲染顺序和深度测试
        this.endEffectorGizmo.traverse((child) => {
            if (child instanceof THREE.Mesh || child instanceof THREE.Line) {
                if (child.material) {
                    child.material.depthTest = false;
                    child.material.depthWrite = false;
                }
                child.renderOrder = 1000;
            }
        });
        
        this.scene.add(this.endEffectorGizmo);
        
        // 更新位置到末端执行器
        this.updateGizmoPosition();
    }
    
    /**
     * 移除拖动控制
     */
    removeEndEffectorGizmo() {
        if (this.endEffectorGizmo) {
            this.scene.remove(this.endEffectorGizmo);
            this.endEffectorGizmo = null;
            this.translationArrows = { x: null, y: null, z: null };
            this.rotationArrows = { x: null, y: null, z: null };
        }
    }
    
    /**
     * 更新拖动控制的位置（跟随末端执行器）
     */
    updateGizmoPosition() {
        if (this.endEffectorGizmo && this.endEffector) {
            this.endEffectorGizmo.position.copy(this.endEffector.position);
            this.endEffectorGizmo.quaternion.copy(this.endEffector.quaternion);
        }
    }
    
    /**
     * 处理拖动末端执行器的鼠标按下事件
     */
    handleDragEndEffectorMouseDown(e) {
        if (!this.dragEndEffectorMode || this.dualArmMode || !this.endEffectorGizmo) return;
        if (e.button !== 0) return; // 只处理左键
        
        // 更新鼠标位置
        this.updateMousePosition(e);
        
        // 检测是否点击在箭头上（只检测平移箭头）
        // 收集所有箭头（包括正负方向）
        const allArrows = [];
        Object.values(this.translationArrows).forEach(arrowPair => {
            if (arrowPair && arrowPair.positive) {
                allArrows.push(arrowPair.positive);
            }
            if (arrowPair && arrowPair.negative) {
                allArrows.push(arrowPair.negative);
            }
        });
        
        const intersects = [];
        allArrows.forEach(arrow => {
            const arrowIntersects = this.raycaster.intersectObject(arrow, true);
            arrowIntersects.forEach(intersect => {
                intersect.object = arrow;
                intersects.push(intersect);
            });
        });
        
        if (intersects.length > 0) {
            // 找到最近的交点
            intersects.sort((a, b) => a.distance - b.distance);
            const closest = intersects[0];
            
            // 获取箭头信息
            const arrowObject = closest.object;
            let arrowGroup = arrowObject;
            
            // 如果是箭头的一部分（ArrowHelper 的 line 或 cone），找到父组
            while (arrowGroup && arrowGroup.parent && arrowGroup.parent !== this.endEffectorGizmo) {
                arrowGroup = arrowGroup.parent;
            }
            
            // 如果还是没有找到，检查是否是 ArrowHelper 本身或其父对象
            if (!arrowGroup || !arrowGroup.userData || !arrowGroup.userData.type) {
                // 检查是否是 ArrowHelper 的子对象
                if (arrowObject.parent && arrowObject.parent.userData && arrowObject.parent.userData.type) {
                    arrowGroup = arrowObject.parent;
                } else if (arrowObject instanceof THREE.ArrowHelper && arrowObject.userData && arrowObject.userData.type) {
                    arrowGroup = arrowObject;
                }
            }
            
            if (arrowGroup && arrowGroup.userData && arrowGroup.userData.type) {
                this.selectedArrow = arrowGroup;
                this.arrowType = arrowGroup.userData.type;
                this.arrowAxis = arrowGroup.userData.axis;
                this.isDraggingEndEffector = true;
                
                // 存储箭头方向信息，确保在拖动过程中也能访问
                this.selectedArrowDirection = arrowGroup.userData.direction || 'positive';
                
                // 高亮选中的箭头
                this.highlightArrow(arrowGroup, true);
                
                // 创建拖动平面（根据箭头类型）
                if (this.arrowType === 'translation') {
                    // 平移：平面垂直于箭头方向
                    const axisDir = this.getAxisDirection(this.arrowAxis);
                    // 根据方向（正/负）决定箭头方向
                    const arrowDirection = this.selectedArrowDirection === 'negative' ? -1 : 1;
                    const worldAxisDir = axisDir.clone().multiplyScalar(arrowDirection).applyQuaternion(this.endEffectorGizmo.quaternion);
                    
                    // 【锁定方向】存储世界轴方向，供拖动时使用（在整个拖动过程中保持不变）
                    this.worldAxisDirection = worldAxisDir.clone().normalize();
                    
                    // 【锁定拖动平面】创建拖动平面：平面应该垂直于箭头方向
                    // 平面的法向量应该垂直于箭头方向，这样鼠标在平面上的移动可以投影到箭头方向
                    // 使用相机方向作为平面的法向量（如果相机方向与箭头方向平行，使用相机到末端执行器的方向）
                    const cameraDirection = new THREE.Vector3();
                    this.camera.getWorldDirection(cameraDirection);
                    
                    // 计算垂直于箭头方向且接近相机方向的平面法向量
                    let planeNormal = cameraDirection.clone();
                    
                    // 如果相机方向与箭头方向平行（点积接近1或-1），使用相机到末端执行器的方向
                    const dotProduct = Math.abs(planeNormal.dot(this.worldAxisDirection));
                    if (dotProduct > 0.9) {
                        // 相机方向与箭头方向太接近平行，使用相机到末端执行器的方向
                        const cameraToEndEffector = new THREE.Vector3().subVectors(this.endEffector.position, this.camera.position).normalize();
                        planeNormal = cameraToEndEffector;
                    }
                    
                    // 确保平面法向量与箭头方向垂直（通过从相机方向减去箭头方向的投影）
                    const projection = planeNormal.dot(this.worldAxisDirection);
                    planeNormal.sub(this.worldAxisDirection.clone().multiplyScalar(projection)).normalize();
                    
                    // 如果平面法向量为零（说明相机方向与箭头方向完全平行），使用一个默认的垂直方向
                    if (planeNormal.length() < 0.1) {
                        // 选择一个与箭头方向垂直的默认方向
                        if (Math.abs(this.worldAxisDirection.x) < 0.9) {
                            planeNormal.set(1, 0, 0).sub(this.worldAxisDirection.clone().multiplyScalar(this.worldAxisDirection.x)).normalize();
                        } else {
                            planeNormal.set(0, 1, 0).sub(this.worldAxisDirection.clone().multiplyScalar(this.worldAxisDirection.y)).normalize();
                        }
                    }
                    
                    // 【锁定平面法向量】存储平面法向量，确保在整个拖动过程中保持不变
                    this.lockedPlaneNormal = planeNormal.clone();
                    
                    // 调试信息
                    console.log(`✓ 选中箭头: ${this.arrowAxis} ${this.selectedArrowDirection}`);
                    console.log(`  局部轴方向: (${axisDir.x}, ${axisDir.y}, ${axisDir.z})`);
                    console.log(`  箭头方向系数: ${arrowDirection}`);
                    console.log(`  箭头方向（世界坐标系）: (${worldAxisDir.x.toFixed(4)}, ${worldAxisDir.y.toFixed(4)}, ${worldAxisDir.z.toFixed(4)})`);
                    console.log(`  平面法向量（垂直于箭头）: (${planeNormal.x.toFixed(4)}, ${planeNormal.y.toFixed(4)}, ${planeNormal.z.toFixed(4)})`);
                    console.log(`  验证垂直性: ${planeNormal.dot(worldAxisDir).toFixed(6)} (应该接近0)`);
                    
                    // 【锁定拖动平面】创建平面：法向量垂直于箭头方向，经过末端执行器位置
                    // Plane(normal, constant) 其中 constant = -normal.dot(point)
                    // 使用锁定的平面法向量，确保在整个拖动过程中保持不变
                    const planeConstant = -this.lockedPlaneNormal.dot(this.endEffector.position);
                    this.dragPlane = new THREE.Plane(this.lockedPlaneNormal, planeConstant);
                    
                    // 初始化拖动起始位置（在拖动平面上）
                    const initialIntersection = new THREE.Vector3();
                    this.updateMousePosition(e);
                    this.raycaster.ray.intersectPlane(this.dragPlane, initialIntersection);
                    
                    this.dragStartPosition = initialIntersection.clone();
                    this.dragCurrentPosition = initialIntersection.clone();
                    this.previousMousePosition = { x: e.clientX, y: e.clientY };
                    
                    console.log(`  拖动起始位置（平面交点）: (${initialIntersection.x.toFixed(4)}, ${initialIntersection.y.toFixed(4)}, ${initialIntersection.z.toFixed(4)})`);
                } else {
                    // 旋转：平面垂直于相机视线
                    const cameraDirection = new THREE.Vector3();
                    this.camera.getWorldDirection(cameraDirection);
                    this.dragPlane = new THREE.Plane(cameraDirection, -cameraDirection.dot(this.endEffector.position));
                    
                    this.dragStartPosition = this.endEffector.position.clone();
                    this.dragCurrentPosition = this.endEffector.position.clone();
                    this.previousMousePosition = { x: e.clientX, y: e.clientY };
                }
                
                e.preventDefault();
                e.stopPropagation();
            }
        }
    }
    
    /**
     * 获取轴方向向量
     */
    getAxisDirection(axis) {
        const directions = {
            x: new THREE.Vector3(1, 0, 0),
            y: new THREE.Vector3(0, 1, 0),
            z: new THREE.Vector3(0, 0, 1)
        };
        return directions[axis] || new THREE.Vector3(0, 0, 1);
    }
    
    /**
     * 高亮箭头
     */
    highlightArrow(arrow, highlight) {
        if (!arrow) return;
        
        // 如果是 ArrowHelper，特殊处理
        if (arrow instanceof THREE.ArrowHelper) {
            // 处理 line（LineBasicMaterial）
            if (arrow.line && arrow.line.material) {
                if (!arrow.line.userData.originalColor) {
                    arrow.line.userData.originalColor = arrow.line.material.color.clone();
                    arrow.line.userData.originalOpacity = arrow.line.material.opacity;
                }
                if (highlight) {
                    arrow.line.material.color.setHex(0xffffff);
                    arrow.line.material.opacity = 1.0;
                } else {
                    arrow.line.material.color.copy(arrow.line.userData.originalColor);
                    arrow.line.material.opacity = arrow.line.userData.originalOpacity || 0.8;
                }
            }
            
            // 处理 cone（MeshBasicMaterial）
            if (arrow.cone && arrow.cone.material) {
                if (!arrow.cone.userData.originalOpacity) {
                    arrow.cone.userData.originalOpacity = arrow.cone.material.opacity;
                }
                if (arrow.cone.material.emissive) {
                    if (highlight) {
                        arrow.cone.material.emissive.setHex(0xffffff);
                        if (arrow.cone.material.emissiveIntensity !== undefined) {
                            arrow.cone.material.emissiveIntensity = 0.5;
                        }
                        arrow.cone.material.opacity = 1.0;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1.3, 1.3, 1.3);
                        }
                    } else {
                        arrow.cone.material.emissive.setHex(0x000000);
                        if (arrow.cone.material.emissiveIntensity !== undefined) {
                            arrow.cone.material.emissiveIntensity = 0;
                        }
                        arrow.cone.material.opacity = arrow.cone.userData.originalOpacity || 0.8;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1, 1, 1);
                        }
                    }
                } else {
                    // 如果没有 emissive，使用 color
                    if (!arrow.cone.userData.originalColor) {
                        arrow.cone.userData.originalColor = arrow.cone.material.color.clone();
                    }
                    if (highlight) {
                        arrow.cone.material.color.setHex(0xffffff);
                        arrow.cone.material.opacity = 1.0;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1.3, 1.3, 1.3);
                        }
                    } else {
                        arrow.cone.material.color.copy(arrow.cone.userData.originalColor);
                        arrow.cone.material.opacity = arrow.cone.userData.originalOpacity || 0.8;
                        if (arrow.cone.scale) {
                            arrow.cone.scale.set(1, 1, 1);
                        }
                    }
                }
            }
            return;
        }
        
        // 处理其他类型的箭头（Mesh）
        arrow.traverse((child) => {
            if (child instanceof THREE.Mesh && child.material) {
                if (!child.userData.originalOpacity) {
                    child.userData.originalOpacity = child.material.opacity;
                    if (child.material.color) {
                        child.userData.originalColor = child.material.color.clone();
                    }
                }
                
                if (highlight) {
                    // 优先使用 emissive，如果没有则使用 color
                    if (child.material.emissive) {
                        child.material.emissive.setHex(0xffffff);
                        if (child.material.emissiveIntensity !== undefined) {
                            child.material.emissiveIntensity = 0.5;
                        }
                    } else if (child.material.color) {
                        child.material.color.setHex(0xffffff);
                    }
                    child.material.opacity = 1.0;
                    if (child.scale) {
                        child.scale.set(1.3, 1.3, 1.3);
                    }
                } else {
                    // 恢复原始状态
                    if (child.material.emissive) {
                        child.material.emissive.setHex(0x000000);
                        if (child.material.emissiveIntensity !== undefined) {
                            child.material.emissiveIntensity = 0;
                        }
                    } else if (child.material.color && child.userData.originalColor) {
                        child.material.color.copy(child.userData.originalColor);
                    }
                    child.material.opacity = child.userData.originalOpacity || 0.8;
                    if (child.scale) {
                        child.scale.set(1, 1, 1);
                    }
                }
            }
        });
    }
    
    /**
     * 处理拖动末端执行器的鼠标移动事件
     */
    handleDragEndEffectorMouseMove(e) {
        if (!this.dragEndEffectorMode || !this.isDraggingEndEffector || this.dualArmMode || !this.endEffector || !this.selectedArrow || !this.dragPlane) return;
        
        // 更新鼠标位置
        this.updateMousePosition(e);
        
        if (this.arrowType === 'translation') {
            // 平移箭头拖动
            const intersection = new THREE.Vector3();
            const hasIntersection = this.raycaster.ray.intersectPlane(this.dragPlane, intersection);
            
            if (hasIntersection) {
                // 【使用锁定的方向】计算沿箭头方向的位移
                // 使用存储的世界轴方向（在鼠标按下时已经锁定，在整个拖动过程中保持不变）
                // 确保方向完全锁定，不重新计算
                if (!this.worldAxisDirection) {
                    console.error('拖动方向未锁定！');
                    return;
                }
                const worldAxisDir = this.worldAxisDirection.clone();
                
                // 计算鼠标在拖动平面上的移动（从上次位置到当前位置）
                // 注意：这里需要先克隆，避免修改原始向量
                const deltaPos = intersection.clone().sub(this.dragCurrentPosition);
                
                // 计算鼠标移动在箭头方向上的投影长度
                // 这个投影长度就是沿箭头方向的位移
                const projectionLength = deltaPos.dot(worldAxisDir);
                
                // 计算沿箭头方向的位移向量
                // projectedDelta 的方向就是 worldAxisDir 的方向
                // 如果投影长度为正，说明沿箭头正方向移动；为负则沿负方向移动
                const projectedDelta = worldAxisDir.clone().multiplyScalar(projectionLength);
                
                // 调试信息（限制频率）
                if (!this._debugLogged || (Date.now() - (this._lastDebugTime || 0)) > 500) {
                    console.log(`拖动: 箭头=${this.arrowAxis} ${this.selectedArrowDirection}`);
                    console.log(`  交点: (${intersection.x.toFixed(4)}, ${intersection.y.toFixed(4)}, ${intersection.z.toFixed(4)})`);
                    console.log(`  上次位置: (${this.dragCurrentPosition.x.toFixed(4)}, ${this.dragCurrentPosition.y.toFixed(4)}, ${this.dragCurrentPosition.z.toFixed(4)})`);
                    console.log(`  鼠标移动: (${deltaPos.x.toFixed(4)}, ${deltaPos.y.toFixed(4)}, ${deltaPos.z.toFixed(4)})`);
                    console.log(`  箭头方向: (${worldAxisDir.x.toFixed(4)}, ${worldAxisDir.y.toFixed(4)}, ${worldAxisDir.z.toFixed(4)})`);
                    console.log(`  投影长度: ${projectionLength.toFixed(4)}`);
                    console.log(`  计算位移: (${projectedDelta.x.toFixed(4)}, ${projectedDelta.y.toFixed(4)}, ${projectedDelta.z.toFixed(4)})`);
                    this._debugLogged = true;
                    this._lastDebugTime = Date.now();
                }
                
                // 更新拖动当前位置
                this.dragCurrentPosition = intersection.clone();
                
                // 使用雅可比伪逆更新关节角度
                this.updateJointsFromEndEffectorDelta(projectedDelta);
            }
        } else if (this.arrowType === 'rotation') {
            // 旋转箭头拖动
            const intersection = new THREE.Vector3();
            const hasIntersection = this.raycaster.ray.intersectPlane(this.dragPlane, intersection);
            
            if (hasIntersection) {
                // 计算旋转角度
                const axisDir = this.getAxisDirection(this.arrowAxis);
                const worldAxisDir = axisDir.clone().applyQuaternion(this.endEffectorGizmo.quaternion);
                
                // 计算从末端执行器到交点的向量
                const toIntersection = intersection.sub(this.endEffector.position).normalize();
                const toPrevious = this.dragCurrentPosition.sub(this.endEffector.position).normalize();
                
                // 计算角度变化（使用叉积和点积）
                const cross = new THREE.Vector3().crossVectors(toPrevious, toIntersection);
                const dot = toPrevious.dot(toIntersection);
                const angle = Math.atan2(cross.dot(worldAxisDir), dot);
                
                // 将角度转换为旋转四元数
                const rotationQuat = new THREE.Quaternion().setFromAxisAngle(worldAxisDir, angle);
                
                // 计算旋转后的末端位置变化
                const currentPos = this.endEffector.position.clone();
                const rotatedPos = currentPos.clone().applyQuaternion(rotationQuat);
                const deltaPos = rotatedPos.sub(currentPos);
                
                this.dragCurrentPosition = intersection.clone();
                
                // 使用雅可比伪逆更新关节角度
                this.updateJointsFromEndEffectorDelta(deltaPos);
            }
        }
    }
    
    /**
     * 处理拖动末端执行器的鼠标释放事件
     */
    handleDragEndEffectorMouseUp(e) {
        if (!this.dragEndEffectorMode || e.button !== 0) return;
        
        // 取消高亮
        if (this.selectedArrow) {
            this.highlightArrow(this.selectedArrow, false);
        }
        
        this.isDraggingEndEffector = false;
        this.selectedArrow = null;
        this.arrowType = null;
        this.arrowAxis = null;
        this.selectedArrowDirection = null;  // 清除方向信息
        this.worldAxisDirection = null;  // 清除锁定的方向
        this.lockedPlaneNormal = null;  // 清除锁定的平面法向量
        this.dragPlane = null;
        this._debugLogged = false;  // 重置调试标志
    }
    
    /**
     * 更新鼠标位置（用于 Raycaster）
     */
    updateMousePosition(e) {
        const rect = this.canvas.getBoundingClientRect();
        this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        this.raycaster.setFromCamera(this.mouse, this.camera);
    }

    _getHumanoidMetricDirectionTolerance() {
        const cfg = this.vfiConfig || {};
        const directionTolerance = Number.isFinite(cfg.manifoldMetricDirectionTolerance)
            ? cfg.manifoldMetricDirectionTolerance
            : 1e-12;
        const minImprovement = Number.isFinite(cfg.manifoldMetricDirectionMinImprovement)
            ? cfg.manifoldMetricDirectionMinImprovement
            : 1e-12;
        return {
            direction: Math.max(0, directionTolerance),
            minImprovement: Math.max(0, minImprovement)
        };
    }

    _isHumanoidMetricRestoreUsable(refineResult) {
        const acceptedIterations = Number.isFinite(refineResult?.acceptedIterations) ? refineResult.acceptedIterations : 0;
        return (refineResult?.converged === true || acceptedIterations > 0)
            && Array.isArray(refineResult?.qApplied);
    }

    _evaluateHumanoidJogMetricDirectionWithFallback({ before, after, direction, tolerance, minImprovement }) {
        const primary = evaluateHumanoidJogManifoldObjectiveDirection({
            before,
            after,
            direction,
            tolerance
        });
        if (primary.applied) {
            return primary;
        }
        const fallback = evaluateHumanoidJogManifoldNonDegradation({
            before,
            after,
            direction,
            tolerance
        });
        const delta = Number.isFinite(primary?.delta) ? Math.abs(primary.delta) : Number.NaN;
        if (fallback.applied && Number.isFinite(delta) && delta <= minImprovement) {
            return {
                ...fallback,
                reason: 'not_improved',
                applied: true
            };
        }
        return primary;
    }
    
    /**
     * 沿指定轴方向移动双臂绝对位姿（示教器模式，使用零空间保持相对位姿不变）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveDualArmAbsolutePoseAlongAxis(axis, direction) {
        if (this.humanoidMode && window.humanoidModel && window.humanoidQ?.length) {
            try {
                const DQ = window.DQModule?.DQWrapper;
                if (!DQ) {
                    return this._publishHumanoidJogInputFailure({
                        modeTag: 'absolute-target',
                        includeWaist: false,
                        keepBodyZVertical: false,
                        reason: 'dq_unavailable',
                        message: 'DQ 模块未加载，无法执行 absolute 点动。'
                    });
                }
                const stepSize = this.getTeachTranslationStep();
                const directionSign = direction === 'positive' ? 1 : -1;
                
                // 获取当前绝对位姿
                const dqAbsArray = this._getHumanoidAbsolutePose(window.humanoidQ);
                if (!dqAbsArray) {
                    return this._publishHumanoidJogInputFailure({
                        modeTag: 'absolute-target',
                        includeWaist: false,
                        keepBodyZVertical: false,
                        reason: 'fk_unavailable',
                        message: '当前 cooperative absolute pose 不可用，无法执行 absolute 点动。'
                    });
                }
                const dqAbsCurrent = DQ.createFromArray(dqAbsArray);
                const rotationArray = DQ.getRotation(dqAbsCurrent);
                const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
                
                // 计算平移增量
                const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
                const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
                const translationAxisGuard = this._buildTranslationAxisGuard(delta);
                
                // 计算目标绝对位姿
                const dqTargetDQ = DQ.multiply(dqDelta, dqAbsCurrent);
                const dqTargetArray = DQ.toArray(dqTargetDQ);
                
                // 获取关节限位避免开关状态
                const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
                
                // 使用相对位姿优先的零空间控制更新（相对位姿不变 > 绝对位姿跟踪 > 关节限位）
                this.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
                    dqTargetArray,
                    window.humanoidQ,
                    enableLimitAvoidance,
                    false,
                    false,
                    null,
                    translationAxisGuard
                );
            } catch (e) {
                console.error('移动人形绝对位姿失败:', e);
                return this._publishHumanoidJogInputFailure({
                    modeTag: 'absolute-target',
                    includeWaist: false,
                    keepBodyZVertical: false,
                    reason: 'jog_exception',
                    message: `absolute 点动异常：${e?.message || 'unknown'}`
                });
            }
            return;
        }
        if (!this.dualArmMode) {
            console.warn('双臂绝对位姿控制仅支持双臂或人形模式');
            return;
        }
        if (!window.dualArmSystem) { console.warn('双臂系统未初始化'); return; }
        if (!window.jointAngles1 || !window.jointAngles2 || window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化'); return;
        }
        if (!window.DQModule || !window.DQModule.DQWrapper) { console.error('DQ Robotics 模块未加载'); return; }
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = this.getTranslationDirection(axis).multiplyScalar(stepSize * directionSign);
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            this.updateDualArmJointsFromAbsolutePoseDelta(delta, allJointAngles);
        } catch (error) { console.error('移动双臂绝对位姿失败:', error); }
    }

    optimizeHumanoidWVFI(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('wvfi-only', 'w_vfi');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_vfi');
    }

    optimizeHumanoidManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-only', 'w_cmd');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_cmd');
    }

    optimizeHumanoidAbsoluteManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-abs-only', 'w_abs');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_abs');
    }

    optimizeHumanoidAbsoluteArmManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-abs-arm-only', 'w_abs_arm');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_abs_arm');
    }

    optimizeHumanoidHierarchicalAbsoluteManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-abs-hier-only', 'w_abs_hier');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_abs_hier');
    }

    optimizeHumanoidHierarchicalAbsoluteVFIManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-abs-hier-vfi-only', 'w_abs_hier_vfi');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_abs_hier_vfi');
    }

    optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityIterative(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-abs-hier-vfi-iter-only', 'w_abs_hier_vfi_iter');
            return false;
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const currentQ = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
        const dqAbsoluteTargetArray = this._getHumanoidAbsolutePose(currentQ);
        const fixedRelativeTargetArray = this._getHumanoidRelativePose(currentQ);
        if (!dqAbsoluteTargetArray || !fixedRelativeTargetArray) {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_absolute',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const requestDirection = direction === 'decrease' ? 'decrease' : 'increase';
        try {
            const result = this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
                dqAbsoluteTargetArray.slice(),
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray,
                {
                    solverMode: 'mj_restore',
                    preferWorker: false,
                    jogMetricOverride: 'w_abs_hier_vfi',
                    metricDirection: requestDirection,
                    iterativeMetricProjectionPlan: {
                        metricIterations: 15,
                        restoreIterations: 35
                    }
                }
            );
            if (result && typeof result.then === 'function') {
                void result.catch((error) => {
                    console.error('w_abs_hier_vfi_iter 优化失败:', error);
                    this._setIKFailureStatus({
                        solver: 'mj_restore',
                        mode: 'humanoid_absolute',
                        reason: 'async_exception',
                        message: `MuJoCo 风格恢复 w_abs_hier_vfi_iter 优化异常：${error?.message || 'unknown'}`
                    });
                });
                return true;
            }
            return result === true;
        } catch (error) {
            console.error('w_abs_hier_vfi_iter 优化异常:', error);
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_absolute',
                reason: 'exception',
                message: `MuJoCo 风格恢复 w_abs_hier_vfi_iter 优化异常：${error?.message || 'unknown'}`
            });
        }
    }

    optimizeHumanoidAbsoluteVFIManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('w-abs-vfi-only', 'w_abs_vfi');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'w_abs_vfi');
    }

    optimizeHumanoidDirectionalManipulability(direction = 'increase') {
        if (!this.humanoidMode || !window.humanoidQ?.length) return false;
        if (!this._isHumanoidMetricOptimizationSolverEnabled()) {
            this._publishHumanoidJogMetricOptimizationSolverMismatch('sigma-only', 'sigma');
            return false;
        }
        return this._optimizeHumanoidMetricViaMJRestore(direction, 'sigma');
    }

    _optimizeHumanoidMetricViaMJRestore(direction = 'increase', metric = 'w_vfi') {
        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const currentQ = Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : [];
        const dqAbsoluteTargetArray = this._getHumanoidAbsolutePose(currentQ);
        const fixedRelativeTargetArray = this._getHumanoidRelativePose(currentQ);
        if (!dqAbsoluteTargetArray || !fixedRelativeTargetArray) {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_absolute',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const requestMetric = normalizeHumanoidJogManifoldJogMetric(metric);
        if (requestMetric === 'none') {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_absolute',
                reason: 'invalid_metric',
                message: `无效的可操作度指标：${metric}`
            });
        }

        const requestDirection = direction === 'decrease' ? 'decrease' : 'increase';
        try {
            const result = this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
                dqAbsoluteTargetArray.slice(),
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray,
                null,
                {
                    solverMode: 'mj_restore',
                    preferWorker: true,
                    jogMetricOverride: requestMetric,
                    metricDirection: requestDirection
                }
            );
            if (result && typeof result.then === 'function') {
                void result.catch((error) => {
                    console.error('w 指标优化（MJ 恢复）失败:', error);
                    this._setIKFailureStatus({
                        solver: 'mj_restore',
                        mode: 'humanoid_absolute',
                        reason: 'async_exception',
                        message: `MuJoCo 风格恢复 w 指标优化异常：${error?.message || 'unknown'}`
                    });
                });
                return true;
            }
            return result === true;
        } catch (error) {
            console.error('w 指标优化（MJ 恢复）异常:', error);
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_absolute',
                reason: 'exception',
                message: `MuJoCo 风格恢复 w 指标优化异常：${error?.message || 'unknown'}`
            });
        }
    }

    _getHumanoidMJRestorePreWControlMode(includeWaist = false, keepBodyZVertical = false) {
        if (keepBodyZVertical) return 'decoupled-upright';
        if (includeWaist) return 'decoupled';
        return 'absolute';
    }

    async _runHumanoidMJRestorePreWStage({
        q0,
        dqAbsoluteProtectedTargetArray,
        fixedRelativeProtectedTargetArray,
        enableJointLimitAvoidance,
        includeWaist,
        keepBodyZVertical,
        jogMetric,
        metricDirection
    }) {
        return this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
            dqAbsoluteProtectedTargetArray,
            q0,
            enableJointLimitAvoidance,
            includeWaist,
            keepBodyZVertical,
            fixedRelativeProtectedTargetArray,
            {
                solverMode: 'mj_restore',
                preferWorker: false,
                suppressFailureNotification: true,
                candidateOnly: true,
                allowArmOnlyFallback: false,
                jogMetricOverride: jogMetric,
                metricDirection
            }
        );
    }

    async _updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestorePreW(
        dqAbsoluteTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        keepBodyZVertical = false,
        fixedRelativeTargetArray = null,
        translationAxisGuard = null
    ) {
        const q0 = Array.isArray(currentQ) ? currentQ.slice() : [];
        const dqAbsoluteProtectedTargetArray = this._getHumanoidAbsolutePose(q0);
        const relCurrentArray = this._getHumanoidRelativePose(q0);
        if (!dqAbsoluteProtectedTargetArray || !relCurrentArray) {
            return this._setIKFailureStatus({
                solver: 'mj_restore_pre_w',
                mode: 'humanoid_absolute',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const fixedRelativeProtectedTargetArray = (
            Array.isArray(fixedRelativeTargetArray) &&
            fixedRelativeTargetArray.length === 8 &&
            fixedRelativeTargetArray.every((value) => Number.isFinite(Number(value)))
        )
            ? fixedRelativeTargetArray.map((value) => Number(value))
            : relCurrentArray.slice();
        const controlMode = this._getHumanoidMJRestorePreWControlMode(includeWaist, keepBodyZVertical);
        const jogMetric = resolveHumanoidJogMJPreWMetric(this._getHumanoidManifoldJogSecondaryMetric());
        const metricDirection = getHumanoidJogMJPreWDirection();

        let preWResult = null;
        try {
            preWResult = await this._runHumanoidMJRestorePreWStage({
                controlMode,
                q0,
                dqAbsoluteProtectedTargetArray: dqAbsoluteProtectedTargetArray.slice(),
                fixedRelativeProtectedTargetArray: fixedRelativeProtectedTargetArray.slice(),
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                jogMetric,
                metricDirection
            });
        } catch (error) {
            preWResult = {
                thrown: true,
                error
            };
        }

        const preWOutcome = mapHumanoidJogMJPreWOutcome(preWResult);
        const qStage2 = (
            preWOutcome.qPreEligible === true &&
            Array.isArray(preWResult?.qApplied) &&
            preWResult.qApplied.length > 0
        )
            ? preWResult.qApplied.slice()
            : q0.slice();
        return this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
            dqAbsoluteTargetArray,
            qStage2,
            enableJointLimitAvoidance,
            includeWaist,
            keepBodyZVertical,
            fixedRelativeProtectedTargetArray,
            translationAxisGuard,
            null
        );
    }

    recordHumanoidJogAxisIntent(axis) {
        if (!this.humanoidMode) return;
        if (typeof axis !== 'string' || axis.trim().length === 0) return;
        this.lastHumanoidJogAxis = axis.trim().toLowerCase();
    }

    _evaluateHumanoidJogManifoldMetricValue(metric, cctsmResult, axis = '') {
        if (metric === 'w_cmd' || metric === 'w') {
            return evaluateHumanoidJogProjectedManipulability(cctsmResult);
        }
        if (metric === 'w_abs') {
            return evaluateHumanoidJogAbsoluteManipulability(cctsmResult);
        }
        if (metric === 'w_abs_arm') {
            return evaluateHumanoidJogAbsoluteArmManipulability(cctsmResult);
        }
        if (metric === 'w_abs_arm_vfi') {
            return evaluateHumanoidJogAbsoluteArmVFIManipulability(cctsmResult);
        }
        if (metric === 'w_abs_vfi') {
            return evaluateHumanoidJogAbsoluteVFIManipulability(cctsmResult);
        }
        if (metric === 'w_abs_hier') {
            return evaluateHumanoidJogAbsoluteHierarchicalManipulability(cctsmResult);
        }
        if (metric === 'w_abs_hier_vfi') {
            return evaluateHumanoidJogAbsoluteHierarchicalVFIManipulability(
                cctsmResult,
                { hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend() }
            );
        }
        if (metric === 'sigma') {
            return evaluateHumanoidJogDirectionalManipulability({
                axis,
                cctsmResult
            });
        }
        return Number.isFinite(cctsmResult?.w_vfi) ? cctsmResult.w_vfi : Number.NaN;
    }

    _computeHumanoidJogManifoldMetricGradient({
        problem = null,
        metric = 'w_vfi',
        axis = '',
        armStart = 0,
        activeCount = 0,
        activeLimits = null
    } = {}) {
        if (!problem?.metadata?.qFull || !Array.isArray(problem?.metadata?.Jabs) || !Array.isArray(problem?.metadata?.protectedRows)) {
            return null;
        }
        if (metric === 'w_vfi') {
            return Array.isArray(problem?.inequalities?.wvfiFloor?.gradient)
                ? problem.inequalities.wvfiFloor.gradient.slice()
                : null;
        }
        const gradientMetric = metric === 'w_cmd' ? 'w' : metric;
        let directionalVector = null;
        if (metric === 'sigma') {
            directionalVector = buildHumanoidJogDirectionalAxisVector({
                axis,
                gramSize: Array.isArray(problem?.metadata?.cctsm?.Gram) ? problem.metadata.cctsm.Gram.length : 12
            });
            if (!Array.isArray(directionalVector)) return null;
        }
        return this._computeProtectedManipulabilityGradient({
            q: problem.metadata.qFull,
            armStart,
            activeCount,
            mode: 'absolute',
            Jcmd: problem.metadata.Jabs,
            Jinvariant: problem.metadata.protectedRows,
            activeLimits,
            forceInvariantEquality: true,
            metric: gradientMetric,
            directionalVector
        });
    }

    async _runHumanoidIterativeMetricProjectionPlan({
        baseProblem = null,
        baseQActive = [],
        metric = 'w_abs_hier_vfi',
        direction = 'increase',
        axis = '',
        evaluateProblem = null,
        armStart = 0,
        activeCount = 0,
        activeLimits = null,
        lowerBounds = [],
        upperBounds = [],
        stepBounds = [],
        wvfiMin = 0,
        restoreMaxIterations = 1,
        damping = 1e-4,
        minScale = 1e-6,
        reductionFactor = 0.5,
        equalityTolerance = 1e-12,
        inequalityTolerance = 1e-12,
        stepSize = 0.01,
        restoreFn = restoreHumanoidJogManifold,
        restoreConfig = null,
        metricIterations = 15,
        restoreIterations = 35
    } = {}) {
        const resolved = resolveHumanoidJogManifoldJogMetric(metric);
        const axisLabel = resolved.family === 'directional' ? axis : '';
        if (!baseProblem || typeof evaluateProblem !== 'function' || !Array.isArray(baseQActive)) {
            return {
                metric: resolved.metric,
                state: 'skipped(no_problem)',
                before: Number.NaN,
                after: Number.NaN,
                applied: false,
                axisLabel,
                nullspaceResult: null,
                refineResult: null,
                finalProblem: baseProblem,
                directionResult: null,
                iterativePlan: null
            };
        }

        let currentProblem = baseProblem;
        let currentQActive = baseQActive.slice();
        const before = this._evaluateHumanoidJogManifoldMetricValue(resolved.metric, currentProblem?.metadata?.cctsm, axisLabel);
        const { direction: directionTolerance, minImprovement } = this._getHumanoidMetricDirectionTolerance();
        const resolvedRestoreConfig = (restoreConfig && typeof restoreConfig === 'object')
            ? restoreConfig
            : {
                referenceStiffness: Number.isFinite(this.vfiConfig?.manifoldReferenceStiffness)
                    ? Math.max(0, this.vfiConfig.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(this.vfiConfig?.manifoldReferenceDamping)
                    ? Math.max(0, this.vfiConfig.manifoldReferenceDamping)
                    : 0.35
            };

        let lastMetricAttempt = null;
        let lastRestoreResult = null;
        let appliedMetricIterations = 0;
        let appliedRestoreIterations = 0;
        let totalIterations = 0;
        let totalAcceptedIterations = 0;
        const metricLoopCount = Math.max(0, Math.floor(metricIterations));
        const restoreLoopCount = Math.max(0, Math.floor(restoreIterations));

        for (let i = 0; i < metricLoopCount; i += 1) {
            const attempt = await this._attemptHumanoidJogManifoldMetricRefinement({
                baseProblem: currentProblem,
                baseQActive: currentQActive,
                metric: resolved.metric,
                direction,
                axis: axisLabel,
                evaluateProblem,
                armStart,
                activeCount,
                activeLimits,
                lowerBounds,
                upperBounds,
                stepBounds,
                wvfiMin,
                restoreMaxIterations,
                damping,
                minScale,
                reductionFactor,
                equalityTolerance,
                inequalityTolerance,
                stepSize,
                preferWVFIFallbackOnClip: resolved.metric === 'w_abs_hier_vfi',
                preferActiveLimitRetreat: resolved.metric === 'w_abs_hier_vfi',
                projectionOnly: true,
                projectionPasses: 1,
                restoreFn,
                restoreConfig: resolvedRestoreConfig
            });
            lastMetricAttempt = attempt;
            if (attempt?.applied !== true || !Array.isArray(attempt?.refineResult?.qApplied) || !attempt?.finalProblem) {
                continue;
            }
            currentQActive = attempt.refineResult.qApplied.slice();
            currentProblem = attempt.finalProblem;
            appliedMetricIterations += 1;
            totalIterations += Number.isFinite(attempt?.refineResult?.iterations)
                ? attempt.refineResult.iterations
                : 0;
            totalAcceptedIterations += Number.isFinite(attempt?.refineResult?.acceptedIterations)
                ? attempt.refineResult.acceptedIterations
                : 1;
        }

        for (let i = 0; i < restoreLoopCount; i += 1) {
            const repairResult = await restoreFn({
                qInitial: currentQActive,
                evaluateProblem,
                maxIterations: 1,
                damping,
                ...resolvedRestoreConfig,
                initialScale: 1,
                minScale,
                reductionFactor,
                equalityTolerance,
                inequalityTolerance
            });
            lastRestoreResult = repairResult;
            if (!this._isHumanoidMetricRestoreUsable(repairResult)) {
                continue;
            }
            const repairedProblem = repairResult?.currentProblem || evaluateProblem(repairResult.qApplied);
            if (!repairedProblem) continue;
            currentQActive = repairResult.qApplied.slice();
            currentProblem = repairedProblem;
            appliedRestoreIterations += 1;
            totalIterations += Number.isFinite(repairResult?.iterations) ? repairResult.iterations : 0;
            totalAcceptedIterations += Number.isFinite(repairResult?.acceptedIterations)
                ? repairResult.acceptedIterations
                : 1;
        }

        const after = this._evaluateHumanoidJogManifoldMetricValue(resolved.metric, currentProblem?.metadata?.cctsm, axisLabel);
        const directionResult = this._evaluateHumanoidJogMetricDirectionWithFallback({
            before,
            after,
            direction,
            tolerance: directionTolerance,
            minImprovement
        });
        const applied = directionResult.applied === true &&
            (appliedMetricIterations > 0 || appliedRestoreIterations > 0);
        const state = applied
            ? `active(iterative_${appliedMetricIterations}/${metricLoopCount}_${appliedRestoreIterations}/${restoreLoopCount})`
            : `skipped(iterative_${appliedMetricIterations}/${metricLoopCount}_${appliedRestoreIterations}/${restoreLoopCount})`;
        const currentProblemFallback = currentProblem || baseProblem;
        const refineResult = {
            converged: lastRestoreResult?.converged === true,
            reason: 'iterative_projection_restore',
            qApplied: currentQActive.slice(),
            iterations: totalIterations,
            acceptedIterations: totalAcceptedIterations,
            lastAcceptedScale: Number.isFinite(lastRestoreResult?.lastAcceptedScale)
                ? lastRestoreResult.lastAcceptedScale
                : (Number.isFinite(lastMetricAttempt?.refineResult?.lastAcceptedScale)
                    ? lastMetricAttempt.refineResult.lastAcceptedScale
                    : 1),
            currentProblem: currentProblemFallback
        };
        return {
            metric: resolved.metric,
            state,
            before,
            after,
            applied,
            axisLabel,
            nullspaceResult: lastMetricAttempt?.nullspaceResult ?? null,
            refineResult,
            finalProblem: currentProblemFallback,
            directionResult,
            iterativePlan: {
                metricIterations: metricLoopCount,
                restoreIterations: restoreLoopCount,
                appliedMetricIterations,
                appliedRestoreIterations
            }
        };
    }

    async _attemptHumanoidJogManifoldMetricRefinement({
        baseProblem = null,
        baseQActive = [],
        metric = 'w_vfi',
        direction = 'increase',
        axis = '',
        evaluateProblem = null,
        armStart = 0,
        activeCount = 0,
        activeLimits = null,
        lowerBounds = [],
        upperBounds = [],
        stepBounds = [],
        wvfiMin = 0,
        restoreMaxIterations = 24,
        damping = 1e-4,
        minScale = 1e-6,
        reductionFactor = 0.5,
        equalityTolerance = 1e-12,
        inequalityTolerance = 1e-12,
        stepSize = 0.1,
        preferWVFIFallbackOnClip = false,
        directionFallbackMetric = '',
        preferActiveLimitRetreat = false,
        projectionOnly = false,
        projectionPasses = 1,
        restoreFn = restoreHumanoidJogManifold,
        restoreConfig = null
    } = {}) {
        const resolved = resolveHumanoidJogManifoldJogMetric(metric);
        const axisLabel = resolved.family === 'directional' ? axis : '';
        if (!baseProblem) {
            return {
                metric: resolved.metric,
                state: 'skipped(no_problem)',
                before: Number.NaN,
                after: Number.NaN,
                applied: false,
                axisLabel,
                nullspaceResult: null,
                refineResult: null,
                finalProblem: null
            };
        }
        if (resolved.shouldSkip) {
            const value = this._evaluateHumanoidJogManifoldMetricValue(resolved.metric, baseProblem?.metadata?.cctsm, axisLabel);
            return {
                metric: resolved.metric,
                state: 'skipped(none)',
                before: value,
                after: value,
                applied: false,
                axisLabel,
                nullspaceResult: null,
                refineResult: null,
                finalProblem: baseProblem
            };
        }
        if (resolved.family === 'directional' && (!axisLabel || axisLabel.length === 0)) {
            return {
                metric: resolved.metric,
                state: 'skipped(no_reference_axis)',
                before: Number.NaN,
                after: Number.NaN,
                applied: false,
                axisLabel,
                nullspaceResult: null,
                refineResult: null,
                finalProblem: baseProblem
            };
        }

        const before = this._evaluateHumanoidJogManifoldMetricValue(resolved.metric, baseProblem?.metadata?.cctsm, axisLabel);
        const summarizeProblemResidualNorm = (problem) => {
            if (!problem || typeof problem !== 'object') return Number.POSITIVE_INFINITY;
            if (Number.isFinite(problem?.equalities?.activeResidualNorm)) {
                return problem.equalities.activeResidualNorm;
            }
            const residual = Array.isArray(problem?.equalities?.weightedResidual)
                ? problem.equalities.weightedResidual
                : (Array.isArray(problem?.equalities?.residual) ? problem.equalities.residual : []);
            return Math.sqrt(residual.reduce((sum, value) => {
                const numericValue = Number(value);
                return Number.isFinite(numericValue) ? (sum + numericValue * numericValue) : sum;
            }, 0));
        };
        const summarizeProblemInequalityViolation = (problem) => {
            if (!problem || typeof problem !== 'object') return Number.POSITIVE_INFINITY;
            return Math.max(
                problem?.inequalities?.wvfiFloor?.violation ?? 0,
                problem?.inequalities?.waistXCorridor?.violation ?? 0
            );
        };
        let nullspaceResult = null;
        const evaluateFinalProblemDirection = (finalProblem) => {
            const after = this._evaluateHumanoidJogManifoldMetricValue(resolved.metric, finalProblem?.metadata?.cctsm, axisLabel);
            const { direction: directionTolerance, minImprovement } = this._getHumanoidMetricDirectionTolerance();
            let directionResult = this._evaluateHumanoidJogMetricDirectionWithFallback({
                before,
                after,
                direction,
                tolerance: directionTolerance,
                minImprovement
            });
            if (directionResult.applied !== true &&
                nullspaceResult?.gradientSource === 'wvfi' &&
                typeof directionFallbackMetric === 'string' &&
                directionFallbackMetric.trim().length > 0) {
                const fallbackMetric = directionFallbackMetric.trim();
                const fallbackBefore = this._evaluateHumanoidJogManifoldMetricValue(
                    fallbackMetric,
                    baseProblem?.metadata?.cctsm,
                    axisLabel
                );
                const fallbackAfter = this._evaluateHumanoidJogManifoldMetricValue(
                    fallbackMetric,
                    finalProblem?.metadata?.cctsm,
                    axisLabel
                );
                const fallbackDirection = this._evaluateHumanoidJogMetricDirectionWithFallback({
                    before: fallbackBefore,
                    after: fallbackAfter,
                    direction,
                    tolerance: directionTolerance,
                    minImprovement
                });
                if (fallbackDirection.applied === true) {
                    directionResult = {
                        ...fallbackDirection,
                        reason: `fallback_metric(${fallbackMetric})`
                    };
                } else {
                    const safetyBefore = this._evaluateHumanoidJogManifoldMetricValue(
                        'w_vfi',
                        baseProblem?.metadata?.cctsm,
                        axisLabel
                    );
                    const safetyAfter = this._evaluateHumanoidJogManifoldMetricValue(
                        'w_vfi',
                        finalProblem?.metadata?.cctsm,
                        axisLabel
                    );
                    const safetyDirection = this._evaluateHumanoidJogMetricDirectionWithFallback({
                        before: safetyBefore,
                        after: safetyAfter,
                        direction,
                        tolerance: directionTolerance,
                        minImprovement
                    });
                    const safetyRegressionTolerance = Math.max(minImprovement, 1e-6);
                    const requestedNonDegraded = Number.isFinite(before) && Number.isFinite(after)
                        ? (direction === 'decrease'
                            ? after <= before + safetyRegressionTolerance
                            : after + safetyRegressionTolerance >= before)
                        : false;
                    if (safetyDirection.applied === true && requestedNonDegraded) {
                        directionResult = {
                            ...safetyDirection,
                            reason: 'fallback_metric(w_vfi)'
                        };
                    }
                }
            }
            if (directionResult.applied !== true && preferActiveLimitRetreat === true) {
                const summarizeVFIMinDistance = (problem) => {
                    const constraints = this._buildManipulabilityVFIConstraints(
                        Array.isArray(problem?.metadata?.qFull) ? problem.metadata.qFull : [],
                        armStart,
                        activeCount,
                        activeLimits
                    );
                    const summary = summarizeManipulabilityVFIConstraints(constraints);
                    if (Number.isFinite(summary.minDistance)) return summary.minDistance;
                    const activationThreshold = Number.isFinite(this.vfiConfig?.jointLimitManipulabilityActivationThreshold)
                        ? Math.max(0, this.vfiConfig.jointLimitManipulabilityActivationThreshold)
                        : 0.15;
                    return activationThreshold;
                };
                const retreatBefore = summarizeVFIMinDistance(baseProblem);
                const retreatAfter = summarizeVFIMinDistance(finalProblem);
                const retreatDirection = this._evaluateHumanoidJogMetricDirectionWithFallback({
                    before: retreatBefore,
                    after: retreatAfter,
                    direction: 'increase',
                    tolerance: directionTolerance,
                    minImprovement
                });
                const retreatRegressionTolerance = Math.max(minImprovement, 1e-4);
                const requestedNonDegraded = Number.isFinite(before) && Number.isFinite(after)
                    ? (direction === 'decrease'
                        ? after <= before + retreatRegressionTolerance
                        : after + retreatRegressionTolerance >= before)
                    : false;
                const projectionRetreatAccepted = projectionOnly === true && retreatDirection.applied === true;
                if (retreatDirection.applied === true && (requestedNonDegraded || projectionRetreatAccepted)) {
                    directionResult = {
                        ...retreatDirection,
                        reason: 'active_limit_retreat'
                    };
                }
            }
            return {
                after,
                directionResult
            };
        };
        const resolvedProjectionPasses = projectionOnly === true && Number.isFinite(projectionPasses)
            ? Math.max(1, Math.floor(projectionPasses))
            : 1;
        if (projectionOnly === true && resolvedProjectionPasses > 1) {
            let currentProblem = baseProblem;
            let currentQActive = Array.isArray(baseQActive) ? baseQActive.slice() : [];
            let lastAppliedAttempt = null;
            let lastDirectionResult = null;
            let acceptedPasses = 0;
            let accumulatedIterations = 0;
            let accumulatedAcceptedIterations = 0;

            for (let passIndex = 0; passIndex < resolvedProjectionPasses; passIndex += 1) {
                const passAttempt = await this._attemptHumanoidJogManifoldMetricRefinement({
                    baseProblem: currentProblem,
                    baseQActive: currentQActive,
                    metric,
                    direction,
                    axis,
                    evaluateProblem,
                    armStart,
                    activeCount,
                    activeLimits,
                    lowerBounds,
                    upperBounds,
                    stepBounds,
                    wvfiMin,
                    restoreMaxIterations,
                    damping,
                    minScale,
                    reductionFactor,
                    equalityTolerance,
                    inequalityTolerance,
                    stepSize,
                    preferWVFIFallbackOnClip,
                    directionFallbackMetric,
                    preferActiveLimitRetreat,
                    projectionOnly: true,
                    projectionPasses: 1,
                    restoreFn,
                    restoreConfig
                });
                if (passAttempt.applied !== true ||
                    !Array.isArray(passAttempt?.refineResult?.qApplied) ||
                    !passAttempt.finalProblem) {
                    if (acceptedPasses === 0) {
                        return passAttempt;
                    }
                    break;
                }
                acceptedPasses += 1;
                accumulatedIterations += Number.isFinite(passAttempt?.refineResult?.iterations)
                    ? passAttempt.refineResult.iterations
                    : 0;
                accumulatedAcceptedIterations += Number.isFinite(passAttempt?.refineResult?.acceptedIterations)
                    ? passAttempt.refineResult.acceptedIterations
                    : 1;
                currentProblem = passAttempt.finalProblem;
                currentQActive = passAttempt.refineResult.qApplied.slice();
                lastAppliedAttempt = passAttempt;
                lastDirectionResult = passAttempt.directionResult ?? null;
            }

            const finalDirection = evaluateFinalProblemDirection(currentProblem);
            const effectiveDirection = finalDirection.directionResult?.applied === true
                ? finalDirection.directionResult
                : lastDirectionResult;
            return {
                metric: resolved.metric,
                state: 'active',
                before,
                after: finalDirection.after,
                applied: true,
                axisLabel,
                nullspaceResult: lastAppliedAttempt?.nullspaceResult ?? null,
                refineResult: {
                    converged: true,
                    reason: acceptedPasses > 1
                        ? 'projected_integration'
                        : (lastAppliedAttempt?.refineResult?.reason ?? 'projected_candidate'),
                    qApplied: currentQActive.slice(),
                    iterations: accumulatedIterations,
                    acceptedIterations: accumulatedAcceptedIterations || acceptedPasses,
                    lastAcceptedScale: lastAppliedAttempt?.refineResult?.lastAcceptedScale ?? 1,
                    currentProblem
                },
                finalProblem: currentProblem,
                directionResult: effectiveDirection
            };
        }
        let objectiveGradient = this._computeHumanoidJogManifoldMetricGradient({
            problem: baseProblem,
            metric: resolved.metric,
            axis: axisLabel,
            armStart,
            activeCount,
            activeLimits
        });
        let adjustedStepSize = stepSize;
        let activeLimitConstraints = [];
        if (preferActiveLimitRetreat === true && Array.isArray(objectiveGradient) && objectiveGradient.length > 0) {
            const qFull = Array.isArray(baseProblem?.metadata?.qFull)
                ? baseProblem.metadata.qFull.slice()
                : this._writeHumanoidActiveJoints([], baseQActive, armStart, activeCount);
            activeLimitConstraints = this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, activeLimits);
            const activeLimitGradient = this._computeJointLimitGradient(baseQActive, activeLimits, activeCount);
            const retreatBlend = buildHumanoidJogActiveLimitRetreatGradient({
                objectiveGradient,
                jointLimitGradient: activeLimitGradient,
                wvfiGradient: Array.isArray(baseProblem?.inequalities?.wvfiFloor?.gradient)
                    ? baseProblem.inequalities.wvfiFloor.gradient.slice()
                    : [],
                constraints: activeLimitConstraints
            });
            if (Array.isArray(retreatBlend.gradient) && retreatBlend.gradient.length === activeCount) {
                objectiveGradient = retreatBlend.gradient;
            }
            if (Number.isFinite(retreatBlend.stepScale) && retreatBlend.stepScale > 1) {
                adjustedStepSize *= projectionOnly === true ? 1 : retreatBlend.stepScale;
            }
        }
        if (!Array.isArray(objectiveGradient) || objectiveGradient.length === 0) {
            return {
                metric: resolved.metric,
                state: 'skipped(no_gradient)',
                before,
                after: before,
                applied: false,
                axisLabel,
                nullspaceResult: null,
                refineResult: null,
                finalProblem: baseProblem
            };
        }

        const stepNorm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));

        nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: Array.isArray(baseQActive) ? baseQActive.slice() : [],
            equalityJacobian: baseProblem?.equalities?.jacobian ?? [],
            objectiveGradient,
            wvfiGradient: Array.isArray(baseProblem?.inequalities?.wvfiFloor?.gradient)
                ? baseProblem.inequalities.wvfiFloor.gradient.slice()
                : new Array(baseQActive.length).fill(0),
            stepSize: adjustedStepSize,
            lowerBounds,
            upperBounds,
            stepBounds,
            evaluateWVFI: (qNextActive) => evaluateProblem(qNextActive)?.inequalities?.wvfiFloor?.value ?? Number.NaN,
            wvfiMin,
            preferWVFIFallbackOnClip
        });
        nullspaceResult.stepNorm = stepNorm(nullspaceResult?.step ?? []);
        if (nullspaceResult.accepted !== true) {
            return {
                metric: resolved.metric,
                state: `skipped(${nullspaceResult?.reason ?? 'no_nullspace'})`,
                before,
                after: before,
                applied: false,
                axisLabel,
                nullspaceResult,
                refineResult: null,
                finalProblem: baseProblem
            };
        }

        const retreatHoldFraction = Number.isFinite(this.vfiConfig?.manifoldActiveLimitRetreatHoldFraction)
            ? this.vfiConfig.manifoldActiveLimitRetreatHoldFraction
            : 0.5;
        const retreatHoldDistance = Number.isFinite(this.vfiConfig?.manifoldActiveLimitRetreatHoldDistance)
            ? this.vfiConfig.manifoldActiveLimitRetreatHoldDistance
            : 1e-3;
        const retreatTargetFraction = Number.isFinite(this.vfiConfig?.manifoldActiveLimitRetreatTargetFraction)
            ? this.vfiConfig.manifoldActiveLimitRetreatTargetFraction
            : 0.4;
        const projectionOnlyRetreatDistanceCap = projectionOnly === true
            ? Math.max(retreatHoldDistance, stepSize)
            : Number.POSITIVE_INFINITY;
        const retreatBounds = preferActiveLimitRetreat === true
            ? buildHumanoidJogActiveLimitRetreatBounds({
                qCurrent: Array.isArray(baseQActive) ? baseQActive.slice() : [],
                step: Array.isArray(nullspaceResult?.step) ? nullspaceResult.step.slice() : [],
                activeLimits,
                constraints: activeLimitConstraints,
                holdFraction: retreatHoldFraction,
                minRetreatDistance: retreatHoldDistance,
                targetThresholdFraction: retreatTargetFraction,
                maxRetreatDistance: projectionOnlyRetreatDistanceCap
            })
            : { active: false };
        const refineEvaluateProblem = retreatBounds.active === true
            ? (qTrial) => {
                const qSource = Array.isArray(qTrial) ? qTrial.slice() : [];
                const qClamped = qSource.slice();
                for (let i = 0; i < qClamped.length; i += 1) {
                    const lockedLower = Number.isFinite(retreatBounds.lowerBounds?.[i])
                        ? retreatBounds.lowerBounds[i]
                        : Number.NEGATIVE_INFINITY;
                    const lockedUpper = Number.isFinite(retreatBounds.upperBounds?.[i])
                        ? retreatBounds.upperBounds[i]
                        : Number.POSITIVE_INFINITY;
                    qClamped[i] = Math.min(lockedUpper, Math.max(lockedLower, qClamped[i]));
                }
                const problem = evaluateProblem(qClamped);
                if (!problem) return problem;
                if (problem.bounds) {
                    const effectiveLower = Array.isArray(problem.bounds.effectiveLower)
                        ? problem.bounds.effectiveLower.slice()
                        : new Array(qClamped.length).fill(Number.NEGATIVE_INFINITY);
                    const effectiveUpper = Array.isArray(problem.bounds.effectiveUpper)
                        ? problem.bounds.effectiveUpper.slice()
                        : new Array(qClamped.length).fill(Number.POSITIVE_INFINITY);
                    for (let i = 0; i < qClamped.length; i += 1) {
                        if (Number.isFinite(retreatBounds.lowerBounds?.[i])) {
                            effectiveLower[i] = Math.max(effectiveLower[i], retreatBounds.lowerBounds[i]);
                        }
                        if (Number.isFinite(retreatBounds.upperBounds?.[i])) {
                            effectiveUpper[i] = Math.min(effectiveUpper[i], retreatBounds.upperBounds[i]);
                        }
                    }
                    problem.bounds.effectiveLower = effectiveLower;
                    problem.bounds.effectiveUpper = effectiveUpper;
                }
                problem.q = qClamped.slice();
                return problem;
            }
            : evaluateProblem;
        const clampProjectedCandidateQ = (qCandidate) => {
            const qClamped = Array.isArray(qCandidate) ? qCandidate.slice() : [];
            for (let i = 0; i < qClamped.length; i += 1) {
                const qBase = Number(baseQActive?.[i] ?? 0);
                const lower = Number.isFinite(lowerBounds?.[i]) ? lowerBounds[i] : Number.NEGATIVE_INFINITY;
                const upper = Number.isFinite(upperBounds?.[i]) ? upperBounds[i] : Number.POSITIVE_INFINITY;
                const stepLimit = Number.isFinite(stepBounds?.[i]) ? Math.max(0, stepBounds[i]) : Number.POSITIVE_INFINITY;
                const qMin = Math.max(lower, qBase - stepLimit);
                const qMax = Math.min(upper, qBase + stepLimit);
                qClamped[i] = Math.min(qMax, Math.max(qMin, Number(qClamped[i] ?? qBase)));
            }
            return qClamped;
        };
        const projectVector = (matrix, vector) => {
            const rowCount = Array.isArray(matrix) ? matrix.length : 0;
            const out = new Array(rowCount).fill(0);
            for (let i = 0; i < rowCount; i += 1) {
                const row = Array.isArray(matrix?.[i]) ? matrix[i] : [];
                let sum = 0;
                for (let j = 0; j < row.length; j += 1) {
                    sum += Number(row[j] ?? 0) * Number(vector?.[j] ?? 0);
                }
                out[i] = sum;
            }
            return out;
        };
        const buildProblemObjectiveGradient = (problem) => {
            let gradient = this._computeHumanoidJogManifoldMetricGradient({
                problem,
                metric: resolved.metric,
                axis: axisLabel,
                armStart,
                activeCount,
                activeLimits
            });
            let currentStepSize = stepSize;
            if (preferActiveLimitRetreat === true && Array.isArray(gradient) && gradient.length > 0) {
                const qActiveProblem = Array.isArray(problem?.q) ? problem.q.slice() : [];
                const qFull = Array.isArray(problem?.metadata?.qFull)
                    ? problem.metadata.qFull.slice()
                    : this._writeHumanoidActiveJoints([], qActiveProblem, armStart, activeCount);
                const constraints = this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, activeLimits);
                const activeLimitGradient = this._computeJointLimitGradient(qActiveProblem, activeLimits, activeCount);
                const retreatBlend = buildHumanoidJogActiveLimitRetreatGradient({
                    objectiveGradient: gradient,
                    jointLimitGradient: activeLimitGradient,
                    wvfiGradient: Array.isArray(problem?.inequalities?.wvfiFloor?.gradient)
                        ? problem.inequalities.wvfiFloor.gradient.slice()
                        : [],
                    constraints
                });
                if (Array.isArray(retreatBlend.gradient) && retreatBlend.gradient.length === activeCount) {
                    gradient = retreatBlend.gradient;
                }
                if (Number.isFinite(retreatBlend.stepScale) && retreatBlend.stepScale > 1) {
                    currentStepSize *= projectionOnly === true ? 1 : retreatBlend.stepScale;
                }
            }
            return {
                gradient: Array.isArray(gradient) ? gradient.slice() : [],
                stepSize: currentStepSize
            };
        };
        const projectedCandidateScales = [1, 0.5, 0.25, 0.125, 0.0625];
        for (const scale of projectedCandidateScales) {
            const qProjected = Array.isArray(baseQActive) ? baseQActive.slice() : [];
            for (let i = 0; i < qProjected.length; i += 1) {
                qProjected[i] += scale * (nullspaceResult?.step?.[i] ?? 0);
            }
            const projectedProblem = refineEvaluateProblem(clampProjectedCandidateQ(qProjected));
            const projectedResidualNorm = summarizeProblemResidualNorm(projectedProblem);
            const projectedInequalityViolation = summarizeProblemInequalityViolation(projectedProblem);
            if (projectedResidualNorm > equalityTolerance || projectedInequalityViolation > inequalityTolerance) {
                continue;
            }
            const projectedDirection = evaluateFinalProblemDirection(projectedProblem);
            if (projectedDirection.directionResult.applied !== true) {
                continue;
            }
            const qApplied = Array.isArray(projectedProblem?.q)
                ? projectedProblem.q.slice()
                : qProjected.slice();
            const projectedReason = scale >= 1 - 1e-12
                ? 'projected_candidate'
                : 'projected_candidate_backtracked';
            return {
                metric: resolved.metric,
                state: 'active',
                before,
                after: projectedDirection.after,
                applied: true,
                axisLabel,
                nullspaceResult,
                refineResult: {
                    converged: true,
                    reason: projectedReason,
                    qApplied,
                    iterations: 0,
                    acceptedIterations: 1,
                    lastAcceptedScale: scale,
                    currentProblem: projectedProblem
                },
                finalProblem: projectedProblem,
                directionResult: projectedDirection.directionResult
            };
        }
        const projectedCorrectionIterations = 8;
        for (const scale of projectedCandidateScales) {
            let qProjected = Array.isArray(baseQActive) ? baseQActive.slice() : [];
            for (let i = 0; i < qProjected.length; i += 1) {
                qProjected[i] += scale * (nullspaceResult?.step?.[i] ?? 0);
            }
            qProjected = clampProjectedCandidateQ(qProjected);
            for (let iter = 0; iter < projectedCorrectionIterations; iter += 1) {
                const projectedProblem = refineEvaluateProblem(qProjected);
                const projectedResidualNorm = summarizeProblemResidualNorm(projectedProblem);
                const projectedInequalityViolation = summarizeProblemInequalityViolation(projectedProblem);
                if (projectedResidualNorm <= equalityTolerance && projectedInequalityViolation <= inequalityTolerance) {
                    const projectedDirection = evaluateFinalProblemDirection(projectedProblem);
                    if (projectedDirection.directionResult.applied === true) {
                        return {
                            metric: resolved.metric,
                            state: 'active',
                            before,
                            after: projectedDirection.after,
                            applied: true,
                            axisLabel,
                            nullspaceResult,
                            refineResult: {
                                converged: true,
                                reason: scale >= 1 - 1e-12
                                    ? 'projected_correction'
                                    : 'projected_correction_backtracked',
                                qApplied: Array.isArray(projectedProblem?.q)
                                    ? projectedProblem.q.slice()
                                    : qProjected.slice(),
                                iterations: iter + 1,
                                acceptedIterations: 1,
                                lastAcceptedScale: scale,
                                currentProblem: projectedProblem
                            },
                            finalProblem: projectedProblem,
                            directionResult: projectedDirection.directionResult
                        };
                    }
                }
                const equalityJacobian = Array.isArray(projectedProblem?.equalities?.weightedJacobian) &&
                    projectedProblem.equalities.weightedJacobian.length > 0
                    ? projectedProblem.equalities.weightedJacobian
                    : (Array.isArray(projectedProblem?.equalities?.jacobian) ? projectedProblem.equalities.jacobian : []);
                const equalityResidual = Array.isArray(projectedProblem?.equalities?.weightedResidual) &&
                    projectedProblem.equalities.weightedResidual.length > 0
                    ? projectedProblem.equalities.weightedResidual
                    : (Array.isArray(projectedProblem?.equalities?.residual) ? projectedProblem.equalities.residual : []);
                if (!Array.isArray(equalityJacobian) || equalityJacobian.length === 0) break;
                const correctionPseudoInverse = this._dampedPseudoInverse(equalityJacobian, Math.max(1e-8, damping));
                if (!Array.isArray(correctionPseudoInverse) || correctionPseudoInverse.length !== activeCount) break;
                const correctionStep = new Array(activeCount).fill(0);
                for (let i = 0; i < activeCount; i += 1) {
                    let sum = 0;
                    for (let j = 0; j < equalityResidual.length; j += 1) {
                        sum -= Number(correctionPseudoInverse?.[i]?.[j] ?? 0) * Number(equalityResidual[j] ?? 0);
                    }
                    correctionStep[i] = sum;
                }
                const correctionProjector = this._computeNullspaceProjection(correctionPseudoInverse, equalityJacobian, activeCount);
                const { gradient: currentObjectiveGradient, stepSize: currentStepSize } = buildProblemObjectiveGradient(projectedProblem);
                const projectedSecondary = projectVector(correctionProjector, currentObjectiveGradient);
                const projectedSecondaryNorm = stepNorm(projectedSecondary);
                const correctionOnlyPhase = iter >= Math.floor(projectedCorrectionIterations / 2);
                const secondaryGain = correctionOnlyPhase
                    ? 0
                    : ((currentStepSize * scale) / (1 + 200 * projectedResidualNorm));
                const secondaryScale = projectedSecondaryNorm > 1e-10
                    ? (secondaryGain / projectedSecondaryNorm)
                    : 0;
                let updated = false;
                const qNext = Array.isArray(qProjected) ? qProjected.slice() : [];
                for (let i = 0; i < activeCount; i += 1) {
                    const dq = correctionStep[i] + secondaryScale * (projectedSecondary[i] ?? 0);
                    if (Math.abs(dq) > 1e-10) updated = true;
                    qNext[i] += dq;
                }
                if (!updated) break;
                const qNextClamped = clampProjectedCandidateQ(qNext);
                const nextDeltaNorm = stepNorm(qNextClamped.map((value, index) => value - qProjected[index]));
                qProjected = qNextClamped;
                if (!(nextDeltaNorm > 1e-10)) break;
            }
        }
        const shouldUseRetreatSeed = retreatBounds.active === true &&
            retreatBounds.lockedIndices.some((index) => {
                const qNextValue = nullspaceResult?.qNext?.[index];
                const lockedLower = Number.isFinite(retreatBounds.lowerBounds?.[index])
                    ? retreatBounds.lowerBounds[index]
                    : Number.NEGATIVE_INFINITY;
                const lockedUpper = Number.isFinite(retreatBounds.upperBounds?.[index])
                    ? retreatBounds.upperBounds[index]
                    : Number.POSITIVE_INFINITY;
                return (Number.isFinite(lockedLower) && qNextValue < lockedLower - 1e-9) ||
                    (Number.isFinite(lockedUpper) && qNextValue > lockedUpper + 1e-9);
            });
        const refineInitial = shouldUseRetreatSeed
            ? (() => {
                const qSeed = Array.isArray(baseQActive) ? baseQActive.slice() : [];
                for (let i = 0; i < qSeed.length; i += 1) {
                    const lockedLower = Number.isFinite(retreatBounds.lowerBounds?.[i])
                        ? retreatBounds.lowerBounds[i]
                        : Number.NEGATIVE_INFINITY;
                    const lockedUpper = Number.isFinite(retreatBounds.upperBounds?.[i])
                        ? retreatBounds.upperBounds[i]
                        : Number.POSITIVE_INFINITY;
                    qSeed[i] = Math.min(lockedUpper, Math.max(lockedLower, qSeed[i]));
                }
                return qSeed;
            })()
            : nullspaceResult.qNext;
        const directRetreatSeedProblem = shouldUseRetreatSeed && preferActiveLimitRetreat === true
            ? refineEvaluateProblem(refineInitial)
            : null;
        const directRetreatSeedResidualNorm = summarizeProblemResidualNorm(directRetreatSeedProblem);
        const directRetreatSeedInequalityViolation = directRetreatSeedProblem
            ? summarizeProblemInequalityViolation(directRetreatSeedProblem)
            : Number.POSITIVE_INFINITY;
        const shouldSkipRetreatSeedRestore = shouldUseRetreatSeed === true &&
            preferActiveLimitRetreat === true &&
            !!directRetreatSeedProblem &&
            directRetreatSeedResidualNorm <= equalityTolerance &&
            directRetreatSeedInequalityViolation <= inequalityTolerance;

        const resolvedRestoreConfig = (restoreConfig && typeof restoreConfig === 'object')
            ? restoreConfig
            : {
                referenceStiffness: Number.isFinite(this.vfiConfig?.manifoldReferenceStiffness)
                    ? Math.max(0, this.vfiConfig.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(this.vfiConfig?.manifoldReferenceDamping)
                    ? Math.max(0, this.vfiConfig.manifoldReferenceDamping)
                    : 0.35
            };

        const refineRestoreFn = retreatBounds.active === true &&
            preferActiveLimitRetreat === true &&
            projectionOnly !== true &&
            restoreFn !== restoreHumanoidJogManifold
            ? restoreHumanoidJogMJ
            : restoreFn;
        if (projectionOnly === true) {
            if (!(shouldUseRetreatSeed === true && preferActiveLimitRetreat === true)) {
                return {
                    metric: resolved.metric,
                    state: 'skipped(projection_only_no_progress)',
                    before,
                    after: before,
                    applied: false,
                    axisLabel,
                    nullspaceResult,
                    refineResult: null,
                    finalProblem: baseProblem
                };
            }
            const projectionRestoreResult = await refineRestoreFn({
                qInitial: refineInitial,
                evaluateProblem,
                maxIterations: Math.min(restoreMaxIterations, 8),
                damping,
                referenceStiffness: resolvedRestoreConfig.referenceStiffness,
                referenceDamping: resolvedRestoreConfig.referenceDamping,
                initialScale: 1.0,
                minScale,
                reductionFactor,
                equalityTolerance,
                inequalityTolerance
            });
            if (this._isHumanoidMetricRestoreUsable(projectionRestoreResult)) {
                const projectionProblem = evaluateProblem(projectionRestoreResult.qApplied);
                const projectionResidualNorm = summarizeProblemResidualNorm(projectionProblem);
                const projectionInequalityViolation = summarizeProblemInequalityViolation(projectionProblem);
                if (projectionResidualNorm <= equalityTolerance &&
                    projectionInequalityViolation <= inequalityTolerance) {
                    const projectionDirection = evaluateFinalProblemDirection(projectionProblem);
                    if (projectionDirection.directionResult.applied === true) {
                        return {
                            metric: resolved.metric,
                            state: 'active',
                            before,
                            after: projectionDirection.after,
                            applied: true,
                            axisLabel,
                            nullspaceResult,
                            refineResult: {
                                converged: projectionRestoreResult.converged === true,
                                reason: 'projection_retreat_restore',
                                qApplied: Array.isArray(projectionProblem?.q)
                                    ? projectionProblem.q.slice()
                                    : projectionRestoreResult.qApplied.slice(),
                                iterations: Number.isFinite(projectionRestoreResult.iterations)
                                    ? projectionRestoreResult.iterations
                                    : 0,
                                acceptedIterations: Number.isFinite(projectionRestoreResult.acceptedIterations)
                                    ? projectionRestoreResult.acceptedIterations
                                    : 1,
                                lastAcceptedScale: Number.isFinite(projectionRestoreResult.lastAcceptedScale)
                                    ? projectionRestoreResult.lastAcceptedScale
                                    : 1,
                                currentProblem: projectionProblem
                            },
                            finalProblem: projectionProblem,
                            directionResult: projectionDirection.directionResult
                        };
                    }
                }
            }
            return {
                metric: resolved.metric,
                state: 'skipped(projection_only_no_progress)',
                before,
                after: before,
                applied: false,
                axisLabel,
                nullspaceResult,
                refineResult: null,
                finalProblem: baseProblem
            };
        }
        const refineResult = shouldSkipRetreatSeedRestore
            ? {
                converged: true,
                reason: 'retreat_seed',
                qApplied: Array.isArray(refineInitial) ? refineInitial.slice() : [],
                iterations: 0,
                acceptedIterations: 1,
                lastAcceptedScale: 1,
                currentProblem: directRetreatSeedProblem
            }
            : await refineRestoreFn({
                qInitial: refineInitial,
                evaluateProblem: refineEvaluateProblem,
                maxIterations: restoreMaxIterations,
                damping,
                ...resolvedRestoreConfig,
                initialScale: 1,
                minScale,
                reductionFactor,
                equalityTolerance,
                inequalityTolerance
            });
        const finalProblem = refineResult?.currentProblem || (Array.isArray(refineResult?.qApplied) ? refineEvaluateProblem(refineResult.qApplied) : null);
        const after = this._evaluateHumanoidJogManifoldMetricValue(resolved.metric, finalProblem?.metadata?.cctsm, axisLabel);

        if (!this._isHumanoidMetricRestoreUsable(refineResult)) {
            return {
                metric: resolved.metric,
                state: 'rejected(hard_constraints)',
                before,
                after,
                applied: false,
                axisLabel,
                nullspaceResult,
                refineResult,
                finalProblem
            };
        }
        const { after: evaluatedAfter, directionResult } = evaluateFinalProblemDirection(finalProblem);
        return {
            metric: resolved.metric,
            state: directionResult.applied ? 'active' : `rejected(${directionResult.reason})`,
            before,
            after: evaluatedAfter,
            applied: directionResult.applied === true,
            axisLabel,
            nullspaceResult,
            refineResult,
            finalProblem,
            directionResult
        };
    }
    
    /**
     * 绕指定轴旋转双臂绝对位姿（示教器模式，使用零空间保持相对位姿不变）
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateDualArmAbsolutePoseAroundAxis(axis, direction) {
        if (this.humanoidMode && window.humanoidModel && window.humanoidQ?.length) {
            try {
                const DQ = window.DQModule?.DQWrapper;
                if (!DQ) return;
                const angle = this.getTeachRotationAngle();
                const directionSign = direction === 'positive' ? 1 : -1;
                const rotationAngle = angle * directionSign;
                const dqAbsoluteCurrentArray = this._getHumanoidAbsolutePose(window.humanoidQ);
                if (!dqAbsoluteCurrentArray) return;
                const dqAbsoluteCurrent = DQ.createFromArray(dqAbsoluteCurrentArray);
                let rotationDQ;
                if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
                else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
                else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
                else return;
                const dqAbsoluteTarget = this.combineDQWithTeachOrder(rotationDQ, dqAbsoluteCurrent, false);
                
                // 获取关节限位避免开关状态
                const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
                
                // 使用相对位姿优先的零空间控制更新（相对位姿不变 > 绝对位姿跟踪 > 关节限位）
                this.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(DQ.toArray(dqAbsoluteTarget), window.humanoidQ, enableLimitAvoidance);
            } catch (e) { console.error('旋转人形绝对位姿失败:', e); }
            return;
        }
        if (!this.dualArmMode) { console.warn('双臂绝对位姿控制仅支持双臂或人形模式'); return; }
        if (!window.dualArmSystem) { console.warn('双臂系统未初始化'); return; }
        if (!window.jointAngles1 || !window.jointAngles2 || window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化'); return;
        }
        if (!window.DQModule || !window.DQModule.DQWrapper) { console.error('DQ Robotics 模块未加载'); return; }
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(allJointAngles);
            const dqAbsoluteCurrent = window.DQModule.DQWrapper.createFromArray(dqAbsoluteCurrentArray);
            
            // 创建旋转双四元数
            let rotationDQ;
            if (axis === 'rx') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            // 计算目标绝对位姿，根据乘法顺序切换
            const dqAbsoluteTarget = this.combineDQWithTeachOrder(rotationDQ, dqAbsoluteCurrent, false);
            const dqAbsoluteTargetArray = window.DQModule.DQWrapper.toArray(dqAbsoluteTarget);
            
            // 使用零空间方法更新关节角度（保持相对位姿不变）
            this.updateDualArmJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, allJointAngles);
        } catch (error) {
            console.error('旋转双臂绝对位姿失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从绝对位姿增量更新双臂关节角度（保持相对位姿不变）
     * 参考 MATLAB cdts_bucket_kuka.m：
     * - 使用完整雅可比矩阵：[J_abs; J_rel]
     * - 误差向量：[err_abs; 0]（相对位姿误差为0）
     * - 更新：q = q + pinv(J) * damping * err
     * 
     * @param {THREE.Vector3} delta - 绝对位姿的位移（世界坐标系，单位：米）
     * @param {number[]} currentJointAngles - 当前关节角度 [robot1_joints..., robot2_joints...]
     */
    updateDualArmJointsFromAbsolutePoseDelta(delta, currentJointAngles) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return;
        
        try {
            // 获取当前绝对位姿和相对位姿
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            
            // 创建位移的平移双四元数
            const dqAbsoluteCurrent = window.DQModule.DQWrapper.createFromArray(dqAbsoluteCurrentArray);
            const { multiply } = window.DQModule.DQWrapper;
            const worldMode = this.isWorldTranslationMode();
            let dqDelta;
            let dqAbsoluteTarget;
            if (worldMode) {
                dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
                dqAbsoluteTarget = multiply(dqDelta, dqAbsoluteCurrent);
            } else {
                const rotationArray = window.DQModule.DQWrapper.getRotation(dqAbsoluteCurrent);
                const rotationQuat = new THREE.Quaternion(
                    rotationArray[1] || 0,
                    rotationArray[2] || 0,
                    rotationArray[3] || 0,
                    rotationArray[0] || 1
                );
                const deltaLocal = delta.clone().applyQuaternion(rotationQuat.clone().conjugate());
                dqDelta = window.DQModule.DQWrapper.translation(deltaLocal.x, deltaLocal.y, deltaLocal.z);
                dqAbsoluteTarget = multiply(dqAbsoluteCurrent, dqDelta);
            }
            const dqAbsoluteTargetArray = window.DQModule.DQWrapper.toArray(dqAbsoluteTarget);
            
            // 使用迭代求解
            this.updateDualArmJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, currentJointAngles, dqRelativeCurrentArray);
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从绝对位姿目标更新双臂关节角度（保持相对位姿不变）
     * 
     * @param {number[]} dqAbsoluteTargetArray - 目标绝对位姿（8元素双四元数）
     * @param {number[]} currentJointAngles - 当前关节角度
     * @param {number[]} dqRelativeCurrentArray - 当前相对位姿（可选，如果不提供则从当前关节角度计算）
     */
    updateDualArmJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, currentJointAngles, dqRelativeCurrentArray = null) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return false;
        
        try {
            // 获取当前相对位姿（如果未提供）
            if (!dqRelativeCurrentArray) {
                dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            }
            
            // 使用 L-BFGS 最小化 0.5*||abs_pose - abs_target||^2 + 0.5*||rel_pose - rel_fixed||^2
            const f = (q) => {
                const absPose = window.dualArmSystem.getAbsolutePose(q);
                const relPose = window.dualArmSystem.getRelativePose(q);
                let sum = 0;
                for (let i = 0; i < 8; i++) {
                    sum += (absPose[i] - dqAbsoluteTargetArray[i]) ** 2;
                    sum += (relPose[i] - dqRelativeCurrentArray[i]) ** 2;
                }
                return 0.5 * sum;
            };
            const grad = (q, out) => {
                const absPose = window.dualArmSystem.getAbsolutePose(q);
                const relPose = window.dualArmSystem.getRelativePose(q);
                const errAbs = [];
                const errRel = [];
                for (let i = 0; i < 8; i++) {
                    errAbs.push(absPose[i] - dqAbsoluteTargetArray[i]);
                    errRel.push(relPose[i] - dqRelativeCurrentArray[i]);
                }
                const Jabs = window.dualArmSystem.getAbsolutePoseJacobian(q);
                const Jrel = window.dualArmSystem.getRelativePoseJacobian(q);
                const n = out.length;
                for (let i = 0; i < n; i++) {
                    let s = 0;
                    for (let j = 0; j < 8; j++) {
                        s += (Jabs[j][i] || 0) * errAbs[j];
                        s += (Jrel[j][i] || 0) * errRel[j];
                    }
                    out[i] = s;
                }
            };
            const poseTol = 1e-4;
            const result = this.minimizeLBFGS(f, grad, [...currentJointAngles], {
                maxIter: 120,
                gradTol: 1e-8,
                costTol: 0.5 * poseTol * poseTol,
                m: 10
            });
            const { x: qSol, converged } = result;
            
            const robot1JointCount = window.jointAngles1.length;
            const robot2JointCount = window.jointAngles2.length;
            let q = qSol.slice(0, robot1JointCount + robot2JointCount);
            
            if (window.customRobotConfig && window.clampJointAngles) {
                const jointNames1 = window.customRobotConfig?.robot1?.joint_chain?.joints || [];
                const jointNames2 = window.customRobotConfig?.robot2?.joint_chain?.joints || [];
                const q1 = q.slice(0, robot1JointCount);
                const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
                const clampedQ1 = window.clampJointAngles(q1, jointNames1, this.robot1URDF, window.customRobotConfig?.robot1);
                const clampedQ2 = window.clampJointAngles(q2, jointNames2, this.robot2URDF, window.customRobotConfig?.robot2);
                q = [...clampedQ1, ...clampedQ2];
            }
            
            const q1 = q.slice(0, robot1JointCount);
            const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
            
            for (let i = 0; i < window.jointAngles1.length && i < q1.length; i++) {
                window.jointAngles1[i] = q1[i];
            }
            for (let i = 0; i < window.jointAngles2.length && i < q2.length; i++) {
                window.jointAngles2[i] = q2[i];
            }
            
            // 同步到 main.js
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 更新机器人位姿
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
            const finalAbsResidual = this._computeArrayResidualNorm(window.dualArmSystem.getAbsolutePose(q), dqAbsoluteTargetArray);
            const finalRelResidual = this._computeArrayResidualNorm(window.dualArmSystem.getRelativePose(q), dqRelativeCurrentArray);
            this.lastIKSolveStatus = {
                solver: 'lbfgs',
                mode: 'dual_absolute',
                ...result,
                finalResidual: Math.max(finalAbsResidual, finalRelResidual),
                finalAbsoluteResidual: finalAbsResidual,
                finalInvariantResidual: finalRelResidual
            };
            return converged || (finalAbsResidual <= poseTol && finalRelResidual <= poseTol);
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
            this.lastIKSolveStatus = {
                solver: 'lbfgs',
                mode: 'dual_absolute',
                converged: false,
                reason: 'exception',
                message: error.message || 'dual absolute IK exception',
                finalResidual: Number.POSITIVE_INFINITY
            };
            return false;
        }
    }
    

    /**
     * 沿指定轴方向移动双臂相对位姿（示教器模式，使用零空间保持绝对位姿不变）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveDualArmRelativePoseAlongAxis(axis, direction) {
        if (this.humanoidMode && window.humanoidModel && window.humanoidQ?.length) {
            try {
                const DQ = window.DQModule?.DQWrapper;
                if (!DQ) return;
                const stepSize = this.getTeachTranslationStep();
                const directionSign = direction === 'positive' ? 1 : -1;
                
                // 获取当前相对位姿
                const dqRelArray = this._getHumanoidRelativePose(window.humanoidQ);
                if (!dqRelArray) return;
                const dqRelCurrent = DQ.createFromArray(dqRelArray);
                const rotationArray = DQ.getRotation(dqRelCurrent);
                const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
                
                // 计算平移增量
                const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
                const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
                const translationAxisGuard = this._buildTranslationAxisGuard(delta);
                
                // 计算目标相对位姿
                const worldMode = this.isWorldTranslationMode?.() ?? true;
                const dqTargetDQ = worldMode ? DQ.multiply(dqDelta, dqRelCurrent) : DQ.multiply(dqRelCurrent, dqDelta);
                const dqTargetArray = DQ.toArray(dqTargetDQ);
                
                // 获取关节限位避免开关状态
                const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
                
                // 使用绝对位姿优先的零空间控制更新（绝对位姿不变 > 相对位姿跟踪 > 关节限位）
                this.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(
                    dqTargetArray,
                    window.humanoidQ,
                    enableLimitAvoidance,
                    false,
                    translationAxisGuard
                );
            } catch (e) { console.error('移动人形相对位姿失败:', e); }
            return;
        }
        if (!this.dualArmMode) { console.warn('双臂相对位姿控制仅支持双臂或人形模式'); return; }
        if (!window.dualArmSystem) { console.warn('双臂系统未初始化'); return; }
        if (!window.jointAngles1 || !window.jointAngles2 || window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化'); return;
        }
        if (!window.DQModule || !window.DQModule.DQWrapper) { console.error('DQ Robotics 模块未加载'); return; }
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = this.getTranslationDirection(axis).multiplyScalar(stepSize * directionSign);
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            this.updateDualArmJointsFromRelativePoseDelta(delta, allJointAngles);
        } catch (error) { console.error('移动双臂相对位姿失败:', error); }
    }

    /**
     * 调整双臂末端相对距离（示教器模式）
     * 约束：
     * - 绝对位姿保持不变
     * - 相对姿态（相对旋转四元数）保持不变
     *
     * @param {string} direction - 'positive'（distance+）或 'negative'（distance-）
     * @param {boolean} includeWaist - 是否包含腰部关节，默认为false
     */
    moveDualArmRelativeDistance(direction, includeWaist = false) {
        if (this.humanoidMode && window.humanoidModel && window.humanoidQ?.length) {
            try {
                const stepSize = this.getTeachTranslationStep();
                const directionSign = direction === 'positive' ? 1 : -1;
                const deltaDistance = stepSize * directionSign;
                
                // 获取关节限位避免开关状态
                const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
                
                this.updateHumanoidJointsFromRelativeDistanceDelta(deltaDistance, window.humanoidQ, enableLimitAvoidance, includeWaist);
            } catch (e) { console.error('调整人形相对距离失败:', e); }
            return;
        }
        if (!this.dualArmMode) { console.warn('相对距离控制仅支持双臂或人形模式'); return; }
        if (!window.dualArmSystem) { console.warn('双臂系统未初始化'); return; }
        if (!window.jointAngles1 || !window.jointAngles2 || window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化'); return;
        }
        if (!window.DQModule || !window.DQModule.DQWrapper) { console.error('DQ Robotics 模块未加载'); return; }
        if (!window.DQModule.Kinematics || typeof window.DQModule.Kinematics.distanceJacobian !== 'function') {
            console.error('WASM 未导出 Kinematics.distanceJacobian'); return;
        }
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            const deltaDistance = stepSize * directionSign;
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            this.updateDualArmJointsFromRelativeDistanceDelta(deltaDistance, allJointAngles);
        } catch (error) { console.error('调整双臂相对距离失败:', error); }
    }
    
    /**
     * 绕指定轴旋转双臂相对位姿（示教器模式，使用零空间保持绝对位姿不变）
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateDualArmRelativePoseAroundAxis(axis, direction) {
        if (this.humanoidMode && window.humanoidModel && window.humanoidQ?.length) {
            try {
                const DQ = window.DQModule?.DQWrapper;
                if (!DQ) return;
                const angle = this.getTeachRotationAngle();
                const directionSign = direction === 'positive' ? 1 : -1;
                const rotationAngle = angle * directionSign;
                
                const dqRelativeCurrentArray = this._getHumanoidRelativePose(window.humanoidQ);
                if (!dqRelativeCurrentArray) return;
                const dqRelativeCurrent = DQ.createFromArray(dqRelativeCurrentArray);
                
                let rotationDQ;
                if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
                else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
                else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
                else return;
                
                const dqRelativeTarget = this.combineDQWithTeachOrder(rotationDQ, dqRelativeCurrent, false);
                
                // 获取关节限位避免开关状态
                const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
                
                // 使用绝对位姿优先的零空间控制更新（绝对位姿不变 > 相对位姿跟踪 > 关节限位）
                this.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(DQ.toArray(dqRelativeTarget), window.humanoidQ, enableLimitAvoidance);
            } catch (e) { console.error('旋转人形相对位姿失败:', e); }
            return;
        }
        if (!this.dualArmMode) { console.warn('双臂相对位姿控制仅支持双臂或人形模式'); return; }
        if (!window.dualArmSystem) { console.warn('双臂系统未初始化'); return; }
        if (!window.jointAngles1 || !window.jointAngles2 || window.jointAngles1.length === 0 || window.jointAngles2.length === 0) {
            console.warn('关节角度未初始化'); return;
        }
        if (!window.DQModule || !window.DQModule.DQWrapper) { console.error('DQ Robotics 模块未加载'); return; }
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            const allJointAngles = [...window.jointAngles1, ...window.jointAngles2];
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(allJointAngles);
            const dqRelativeCurrent = window.DQModule.DQWrapper.createFromArray(dqRelativeCurrentArray);
            
            // 创建旋转双四元数
            let rotationDQ;
            if (axis === 'rx') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            // 计算目标相对位姿，根据乘法顺序切换
            const dqRelativeTarget = this.combineDQWithTeachOrder(rotationDQ, dqRelativeCurrent);
            const dqRelativeTargetArray = window.DQModule.DQWrapper.toArray(dqRelativeTarget);
            
            // 使用零空间方法更新关节角度（保持绝对位姿不变）
            this.updateDualArmJointsFromRelativePoseTarget(dqRelativeTargetArray, allJointAngles);
        } catch (error) {
            console.error('旋转双臂相对位姿失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从相对位姿增量更新双臂关节角度（保持绝对位姿不变）
     * 
     * @param {THREE.Vector3} delta - 相对位姿的位移（世界坐标系，单位：米）
     * @param {number[]} currentJointAngles - 当前关节角度 [robot1_joints..., robot2_joints...]
     */
    updateDualArmJointsFromRelativePoseDelta(delta, currentJointAngles) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return;
        
        try {
            // 获取当前绝对位姿和相对位姿
            const dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            const dqRelativeCurrentArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            
            // 创建位移的平移双四元数
            const dqRelativeCurrent = window.DQModule.DQWrapper.createFromArray(dqRelativeCurrentArray);
            const { multiply } = window.DQModule.DQWrapper;
            const worldMode = this.isWorldTranslationMode();
            let dqDelta;
            let dqRelativeTarget;
            if (worldMode) {
                dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
                dqRelativeTarget = multiply(dqDelta, dqRelativeCurrent);
            } else {
                const rotationArray = window.DQModule.DQWrapper.getRotation(dqRelativeCurrent);
                const rotationQuat = new THREE.Quaternion(
                    rotationArray[1] || 0,
                    rotationArray[2] || 0,
                    rotationArray[3] || 0,
                    rotationArray[0] || 1
                );
                const deltaLocal = delta.clone().applyQuaternion(rotationQuat.clone().conjugate());
                dqDelta = window.DQModule.DQWrapper.translation(deltaLocal.x, deltaLocal.y, deltaLocal.z);
                dqRelativeTarget = multiply(dqRelativeCurrent, dqDelta);
            }
            const dqRelativeTargetArray = window.DQModule.DQWrapper.toArray(dqRelativeTarget);
            
            // 使用迭代求解
            this.updateDualArmJointsFromRelativePoseTarget(dqRelativeTargetArray, currentJointAngles, dqAbsoluteCurrentArray);
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
        }
    }
    
    /**
     * 使用零空间方法从相对位姿目标更新双臂关节角度（保持绝对位姿不变）
     * 
     * @param {number[]} dqRelativeTargetArray - 目标相对位姿（8元素双四元数）
     * @param {number[]} currentJointAngles - 当前关节角度
     * @param {number[]} dqAbsoluteCurrentArray - 当前绝对位姿（可选，如果不提供则从当前关节角度计算）
     */
    updateDualArmJointsFromRelativePoseTarget(dqRelativeTargetArray, currentJointAngles, dqAbsoluteCurrentArray = null) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return false;
        
        try {
            // 获取当前绝对位姿（如果未提供）
            if (!dqAbsoluteCurrentArray) {
                dqAbsoluteCurrentArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            }
            
            // 使用 L-BFGS 最小化 0.5*||abs_pose - abs_fixed||^2 + 0.5*||rel_pose - rel_target||^2
            const f = (q) => {
                const absPose = window.dualArmSystem.getAbsolutePose(q);
                const relPose = window.dualArmSystem.getRelativePose(q);
                let sum = 0;
                for (let i = 0; i < 8; i++) {
                    sum += (absPose[i] - dqAbsoluteCurrentArray[i]) ** 2;
                    sum += (relPose[i] - dqRelativeTargetArray[i]) ** 2;
                }
                return 0.5 * sum;
            };
            const grad = (q, out) => {
                const absPose = window.dualArmSystem.getAbsolutePose(q);
                const relPose = window.dualArmSystem.getRelativePose(q);
                const errAbs = [];
                const errRel = [];
                for (let i = 0; i < 8; i++) {
                    errAbs.push(absPose[i] - dqAbsoluteCurrentArray[i]);
                    errRel.push(relPose[i] - dqRelativeTargetArray[i]);
                }
                const Jabs = window.dualArmSystem.getAbsolutePoseJacobian(q);
                const Jrel = window.dualArmSystem.getRelativePoseJacobian(q);
                const n = out.length;
                for (let i = 0; i < n; i++) {
                    let s = 0;
                    for (let j = 0; j < 8; j++) {
                        s += (Jabs[j][i] || 0) * errAbs[j];
                        s += (Jrel[j][i] || 0) * errRel[j];
                    }
                    out[i] = s;
                }
            };
            const poseTol = 1e-4;
            const result = this.minimizeLBFGS(f, grad, [...currentJointAngles], {
                maxIter: 120,
                gradTol: 1e-8,
                costTol: 0.5 * poseTol * poseTol,
                m: 10
            });
            const { x: qSol, converged } = result;
            
            const robot1JointCount = window.jointAngles1.length;
            const robot2JointCount = window.jointAngles2.length;
            let q = qSol.slice(0, robot1JointCount + robot2JointCount);
            if (window.customRobotConfig && window.clampJointAngles) {
                const jointNames1 = window.customRobotConfig?.robot1?.joint_chain?.joints || [];
                const jointNames2 = window.customRobotConfig?.robot2?.joint_chain?.joints || [];
                const q1 = q.slice(0, robot1JointCount);
                const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
                const clampedQ1 = window.clampJointAngles(q1, jointNames1, this.robot1URDF, window.customRobotConfig?.robot1);
                const clampedQ2 = window.clampJointAngles(q2, jointNames2, this.robot2URDF, window.customRobotConfig?.robot2);
                q = [...clampedQ1, ...clampedQ2];
            }
            
            // 限制关节角度
            if (window.customRobotConfig) {
                const jointNames1 = window.customRobotConfig?.robot1?.joint_chain?.joints || [];
                const jointNames2 = window.customRobotConfig?.robot2?.joint_chain?.joints || [];
                
                const q1 = q.slice(0, robot1JointCount);
                const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
                
                if (window.clampJointAngles) {
                    const clampedQ1 = window.clampJointAngles(q1, jointNames1, this.robot1URDF, window.customRobotConfig?.robot1);
                    const clampedQ2 = window.clampJointAngles(q2, jointNames2, this.robot2URDF, window.customRobotConfig?.robot2);
                    q = [...clampedQ1, ...clampedQ2];
                }
            }
            
            // 更新全局关节角度
            const q1 = q.slice(0, robot1JointCount);
            const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
            
            for (let i = 0; i < window.jointAngles1.length && i < q1.length; i++) {
                window.jointAngles1[i] = q1[i];
            }
            for (let i = 0; i < window.jointAngles2.length && i < q2.length; i++) {
                window.jointAngles2[i] = q2[i];
            }
            
            // 同步到 main.js
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 更新机器人位姿
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
            const finalAbsResidual = this._computeArrayResidualNorm(window.dualArmSystem.getAbsolutePose(q), dqAbsoluteCurrentArray);
            const finalRelResidual = this._computeArrayResidualNorm(window.dualArmSystem.getRelativePose(q), dqRelativeTargetArray);
            this.lastIKSolveStatus = {
                solver: 'lbfgs',
                mode: 'dual_relative',
                ...result,
                finalResidual: Math.max(finalAbsResidual, finalRelResidual),
                finalAbsoluteResidual: finalAbsResidual,
                finalInvariantResidual: finalRelResidual
            };
            return converged || (finalAbsResidual <= poseTol && finalRelResidual <= poseTol);
        } catch (error) {
            console.error('更新双臂关节角度失败:', error);
            this.lastIKSolveStatus = {
                solver: 'lbfgs',
                mode: 'dual_relative',
                converged: false,
                reason: 'exception',
                message: error.message || 'dual relative IK exception',
                finalResidual: Number.POSITIVE_INFINITY
            };
            return false;
        }
    }

    /**
     * 使用雅可比法调整相对距离（保持绝对位姿 + 相对姿态不变）
     *
     * 误差向量：
     * - 绝对位姿（8）
     * - 相对旋转（4，取 relativePose 的前 4 个分量）
     * - 相对距离平方（1，使用 WASM distanceJacobian，对应 dot(d^2)）
     *
     * @param {number} deltaDistance - 距离增量（米）
     * @param {number[]} currentJointAngles - 当前关节角度 [robot1..., robot2...]
     */
    updateDualArmJointsFromRelativeDistanceDelta(deltaDistance, currentJointAngles) {
        if (!window.dualArmSystem || !currentJointAngles || currentJointAngles.length === 0) return false;
        if (!window.DQModule?.Kinematics || typeof window.DQModule.Kinematics.distanceJacobian !== 'function') return false;

        try {
            // 固定约束目标：绝对位姿（8）+ 相对姿态（4）
            const dqAbsoluteTargetArray = window.dualArmSystem.getAbsolutePose(currentJointAngles);
            const dqRelativeStartArray = window.dualArmSystem.getRelativePose(currentJointAngles);
            const relativeRotTarget = dqRelativeStartArray.slice(0, 4);

            // 目标距离（用平移向量范数）
            const dqRelStart = window.DQModule.DQWrapper.createFromArray(dqRelativeStartArray);
            const tStart = window.DQModule.DQWrapper.getTranslation(dqRelStart); // [0,x,y,z]
            const dStart = Math.sqrt((tStart[1] || 0) ** 2 + (tStart[2] || 0) ** 2 + (tStart[3] || 0) ** 2);
            const dTarget = Math.max(0.0, dStart + deltaDistance);
            const d2Target = dTarget * dTarget;

            if (dStart < 1e-10 && Math.abs(deltaDistance) > 0) {
                console.warn('当前相对距离接近 0，distanceJacobian 可能退化，无法可靠调整距离');
            }

            // 迭代求解参数（与相对位姿一致：步长小、限制单步，避免飞车）
            const epsilon = 0.000001;
            const maxIterations = 80;
            const damping = 0.15;
            const maxJointStepPerIter = 0.08;

            let q = [...currentJointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;

            while (errNorm > epsilon && iteration < maxIterations) {
                const dqAbsoluteIterArray = window.dualArmSystem.getAbsolutePose(q);
                const dqRelativeIterArray = window.dualArmSystem.getRelativePose(q);

                // 绝对位姿误差（8）
                const errAbsolute = [];
                for (let i = 0; i < 8; i++) {
                    errAbsolute.push(dqAbsoluteTargetArray[i] - dqAbsoluteIterArray[i]);
                }

                // 相对旋转误差（4）
                const errRelRot = [];
                for (let i = 0; i < 4; i++) {
                    errRelRot.push(relativeRotTarget[i] - dqRelativeIterArray[i]);
                }

                // 相对距离平方误差（1）
                const dqRelIter = window.DQModule.DQWrapper.createFromArray(dqRelativeIterArray);
                const tIter = window.DQModule.DQWrapper.getTranslation(dqRelIter);
                const d2Iter = ((tIter[1] || 0) ** 2 + (tIter[2] || 0) ** 2 + (tIter[3] || 0) ** 2);
                const errD2 = d2Target - d2Iter;

                const err = [...errAbsolute, ...errRelRot, errD2];
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                if (errNorm <= epsilon) break;

                // 雅可比：J_abs(8xN) + J_rel_rot(4xN) + J_d2(1xN)
                const jacobianAbsolute = window.dualArmSystem.getAbsolutePoseJacobian(q);
                const jacobianRelative = window.dualArmSystem.getRelativePoseJacobian(q);

                // 相对旋转雅可比：取相对位姿雅可比的前 4 行
                const jacobianRelRot = jacobianRelative.slice(0, 4).map(row => [...row]);

                // 距离平方雅可比：由 WASM distanceJacobian 从相对位姿雅可比 + 相对位姿计算
                const Jd2 = window.DQModule.Kinematics.distanceJacobian(jacobianRelative, dqRelativeIterArray);
                const jacobianD2Row = Array.isArray(Jd2) && Array.isArray(Jd2[0]) ? [...Jd2[0]] : null;
                if (!jacobianD2Row) {
                    console.error('distanceJacobian 返回值格式错误');
                    return false;
                }

                const jacobian = [];
                for (let i = 0; i < jacobianAbsolute.length; i++) jacobian.push([...jacobianAbsolute[i]]);
                for (let i = 0; i < jacobianRelRot.length; i++) jacobian.push([...jacobianRelRot[i]]);
                jacobian.push(jacobianD2Row);

                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    const clamped = Math.max(-maxJointStepPerIter, Math.min(maxJointStepPerIter, sum));
                    deltaJoints.push(clamped);
                }
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                iteration++;
            }

            // 分离关节角度
            const robot1JointCount = window.jointAngles1.length;
            const robot2JointCount = window.jointAngles2.length;

            // 限制关节角度
            if (window.customRobotConfig) {
                const jointNames1 = window.customRobotConfig?.robot1?.joint_chain?.joints || [];
                const jointNames2 = window.customRobotConfig?.robot2?.joint_chain?.joints || [];
                const q1raw = q.slice(0, robot1JointCount);
                const q2raw = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
                if (window.clampJointAngles) {
                    const clampedQ1 = window.clampJointAngles(q1raw, jointNames1, this.robot1URDF, window.customRobotConfig?.robot1);
                    const clampedQ2 = window.clampJointAngles(q2raw, jointNames2, this.robot2URDF, window.customRobotConfig?.robot2);
                    q = [...clampedQ1, ...clampedQ2];
                }
            }

            // 更新全局关节角度
            const q1 = q.slice(0, robot1JointCount);
            const q2 = q.slice(robot1JointCount, robot1JointCount + robot2JointCount);
            for (let i = 0; i < window.jointAngles1.length && i < q1.length; i++) window.jointAngles1[i] = q1[i];
            for (let i = 0; i < window.jointAngles2.length && i < q2.length; i++) window.jointAngles2[i] = q2[i];

            if (window.updateGlobalVariables) window.updateGlobalVariables();
            if (window.updateRobotPose) window.updateRobotPose();
            return true;
        } catch (error) {
            console.error('更新双臂关节角度（相对距离）失败:', error);
            return false;
        }
    }
    
    // ---------- 人形：左臂=robot1，右臂=robot2，相对/绝对位姿与距离控制 ----------
    /** 人形模式下 cooperative dual-arm 只能走 WASM 解析接口；缺失时 fail-fast */
    _getHumanoidDualArmSystem() {
        if (!window.humanoidMode || !window.dualArmSystem) return null;
        const dualArmSystem = window.dualArmSystem;
        const hasFullCooperativeApi =
            typeof dualArmSystem.getAbsolutePose === 'function' &&
            typeof dualArmSystem.getRelativePose === 'function' &&
            typeof dualArmSystem.getAbsolutePoseJacobian === 'function' &&
            typeof dualArmSystem.getRelativePoseJacobian === 'function';
        return hasFullCooperativeApi ? dualArmSystem : null;
    }
    _getHumanoidPose1(q) {
        if (!this.humanoidModel || !q || q.length === 0) return null;
        return this.humanoidModel.fkmEffector(q, 1);
    }
    _getHumanoidPose2(q) {
        if (!this.humanoidModel || !q || q.length === 0) return null;
        return this.humanoidModel.fkmEffector(q, 2);
    }
    /** 绝对位姿：只使用 HumanoidDualArmAdapter（C++ cooperative 公式 x2*pow(relative,0.5)） */
    _getHumanoidAbsolutePose(q) {
        const da = this._getHumanoidDualArmSystem();
        if (!da) return null;
        try {
            return da.getAbsolutePose(q);
        } catch (error) {
            console.warn('Humanoid cooperative absolute pose unavailable:', error);
            return null;
        }
    }
    /** 相对位姿：只使用 HumanoidDualArmAdapter（C++ cooperative 公式 conj(x2)*x1） */
    _getHumanoidRelativePose(q) {
        const da = this._getHumanoidDualArmSystem();
        if (!da) return null;
        try {
            return da.getRelativePose(q);
        } catch (error) {
            console.warn('Humanoid cooperative relative pose unavailable:', error);
            return null;
        }
    }
    _makeHumanoidBaseDQ(basePose = {}) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return null;
        const x = Number(basePose.x) || 0;
        const y = Number(basePose.y) || 0;
        const yaw = Number(basePose.yaw) || 0;
        const h = 0.5 * yaw;
        const c = Math.cos(h);
        const s = Math.sin(h);
        const rot = DQ.createFromArray([c, 0, 0, s, 0, 0, 0, 0]);
        const trans = DQ.translation(x, y, 0);
        return DQ.multiply(trans, rot);
    }
    _composeHumanoidWorldPose(basePose, localPoseArray) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !Array.isArray(localPoseArray) || localPoseArray.length !== 8) return null;
        const baseDQ = this._makeHumanoidBaseDQ(basePose);
        if (!baseDQ) return null;
        const localDQ = DQ.createFromArray(localPoseArray);
        return DQ.toArray(DQ.multiply(baseDQ, localDQ));
    }
    _dqLeftMultiplicationMatrix(leftDQArray) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !Array.isArray(leftDQArray) || leftDQArray.length !== 8) return null;
        const leftDQ = DQ.createFromArray(leftDQArray);
        const L = [];
        for (let i = 0; i < 8; i++) L.push(new Array(8).fill(0));
        for (let j = 0; j < 8; j++) {
            const basis = new Array(8).fill(0);
            basis[j] = 1;
            const col = DQ.toArray(DQ.multiply(leftDQ, DQ.createFromArray(basis)));
            for (let i = 0; i < 8; i++) L[i][j] = col[i];
        }
        return L;
    }
    /** 绝对位姿雅可比：只使用 C++ cooperative 解析雅可比 */
    _getHumanoidAbsolutePoseJacobian(q, eps = 1e-7) {
        const da = this._getHumanoidDualArmSystem();
        if (!da) return null;
        try {
            return da.getAbsolutePoseJacobian(q);
        } catch (error) {
            console.warn('Humanoid cooperative absolute jacobian unavailable:', error);
            return null;
        }
    }
    /** 相对位姿雅可比：只使用 C++ cooperative 解析雅可比 */
    _getHumanoidRelativePoseJacobian(q, eps = 1e-7) {
        const da = this._getHumanoidDualArmSystem();
        if (!da) return null;
        try {
            return da.getRelativePoseJacobian(q);
        } catch (error) {
            console.warn('Humanoid cooperative relative jacobian unavailable:', error);
            return null;
        }
    }
    
    _humanoidFullQFromArms(currentQ, xArms) {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        const r = this.humanoidRightArmLength;
        const armEnd = w + l + r;
        return [...currentQ.slice(0, w), ...xArms, ...currentQ.slice(armEnd)];
    }
    _humanoidArmIndices() {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        const r = this.humanoidRightArmLength;
        return { start: w, end: w + l + r, count: l + r };
    }
    
    /**
     * 获取WBC控制的关节索引范围（腰部+左臂+右臂，不含头部）
     * @returns {{start: number, end: number, count: number}}
     */
    _humanoidWBCIndices() {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        const r = this.humanoidRightArmLength;
        return { start: 0, end: w + l + r, count: w + l + r };
    }
    
    /**
     * 从WBC关节构建完整关节向量
     * @param {number[]} currentQ - 当前完整关节角
     * @param {number[]} xWBC - WBC关节（腰+左臂+右臂）
     * @returns {number[]} 完整关节向量
     */
    _humanoidFullQFromWBC(currentQ, xWBC) {
        const { end: wbcEnd } = this._humanoidWBCIndices();
        return [...xWBC, ...currentQ.slice(wbcEnd)];
    }
    
    /**
     * WBC控制：腰部+双臂协同控制绝对位姿（保持相对位姿不变）
     * @param {number[]} dqAbsoluteTargetArray - 目标绝对位姿 DQ 8元素
     * @param {number[]} currentQ - 当前全关节角
     * @param {number[]} dqRelativeCurrentArray - 当前相对位姿（可选）
     * @returns {boolean}
     */
    updateHumanoidJointsFromAbsolutePoseTargetWBC(dqAbsoluteTargetArray, currentQ, dqRelativeCurrentArray = null) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const relCurrent = dqRelativeCurrentArray || this._getHumanoidRelativePose(currentQ);
        if (!relCurrent) return false;
        
        const { start: wbcStart, end: wbcEnd, count: wbcCount } = this._humanoidWBCIndices();
        if (wbcCount <= 0) return false;
        
        const x0 = currentQ.slice(wbcStart, wbcEnd);
        
        const f = (x) => {
            const q = this._humanoidFullQFromWBC(currentQ, x);
            const absPose = this._getHumanoidAbsolutePose(q);
            const relPose = this._getHumanoidRelativePose(q);
            if (!absPose || !relPose) return 1e10;
            let sum = 0;
            for (let i = 0; i < 8; i++) {
                sum += (absPose[i] - dqAbsoluteTargetArray[i]) ** 2;
                sum += (relPose[i] - relCurrent[i]) ** 2;
            }
            return 0.5 * sum;
        };
        
        const grad = (x, out) => {
            const q = this._humanoidFullQFromWBC(currentQ, x);
            const absPose = this._getHumanoidAbsolutePose(q);
            const relPose = this._getHumanoidRelativePose(q);
            const Jabs = this._getHumanoidAbsolutePoseJacobian(q);
            const Jrel = this._getHumanoidRelativePoseJacobian(q);
            if (!Jabs || !Jrel) return;
            for (let i = 0; i < wbcCount; i++) {
                let s = 0;
                for (let j = 0; j < 8; j++) {
                    s += Jabs[j][wbcStart + i] * (absPose[j] - dqAbsoluteTargetArray[j]);
                    s += Jrel[j][wbcStart + i] * (relPose[j] - relCurrent[j]);
                }
                out[i] = s;
            }
        };
        
        const { x: xSol } = this.minimizeLBFGS(f, grad, x0, { maxIter: 80, tol: 1e-6, m: 10 });
        const qSol = this._humanoidFullQFromWBC(currentQ, xSol);
        
        for (let i = 0; i < window.humanoidQ.length && i < qSol.length; i++) {
            window.humanoidQ[i] = qSol[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(qSol, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }
    
    /**
     * WBC控制：从绝对位姿增量更新关节（迭代雅可比伪逆法）
     * @param {THREE.Vector3} delta - 平移增量（世界坐标系）
     * @param {number[]} currentQ - 当前全关节角
     */
    updateHumanoidJointsFromAbsolutePoseDeltaWBC(delta, currentQ) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !currentQ || currentQ.length === 0) return;
        
        const { start: wbcStart, end: wbcEnd, count: wbcCount } = this._humanoidWBCIndices();
        if (wbcCount <= 0) return;
        
        // 获取当前绝对位姿
        const dqAbsArray = this._getHumanoidAbsolutePose(currentQ);
        if (!dqAbsArray) return;
        
        // 计算目标位姿
        const dqAbsCurrent = DQ.createFromArray(dqAbsArray);
        const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
        const dqTargetDQ = DQ.multiply(dqDelta, dqAbsCurrent);
        const dqTargetArray = DQ.toArray(dqTargetDQ);
        
        // 保存当前相对位姿作为约束目标
        const relTarget = this._getHumanoidRelativePose(currentQ);
        if (!relTarget) return;
        
        // 迭代参数
        const maxIter = 50;
        const tol = 1e-16;
        const damping = 0.8;
        
        const qInitial = [...currentQ];
        let q = [...qInitial];
        
        for (let iter = 0; iter < maxIter; iter++) {
            // 获取当前位姿
            const absNow = this._getHumanoidAbsolutePose(q);
            const relNow = this._getHumanoidRelativePose(q);
            if (!absNow || !relNow) break;
            
            // 计算误差
            const errAbs = [];
            const errRel = [];
            for (let i = 0; i < 8; i++) {
                errAbs.push(dqTargetArray[i] - absNow[i]);
                errRel.push(relTarget[i] - relNow[i]);
            }
            
            // 检查收敛
            const errNorm = Math.sqrt(errAbs.reduce((s, e) => s + e * e, 0) + errRel.reduce((s, e) => s + e * e, 0));
            if (errNorm < tol) break;
            
            // 获取雅可比
            const Jabs = this._getHumanoidAbsolutePoseJacobian(q);
            const Jrel = this._getHumanoidRelativePoseJacobian(q);
            if (!Jabs || !Jrel) break;
            
            // 提取WBC关节对应的列
            const JabsWBC = [];
            const JrelWBC = [];
            for (let i = 0; i < 8; i++) {
                JabsWBC.push(Jabs[i].slice(wbcStart, wbcEnd));
                JrelWBC.push(Jrel[i].slice(wbcStart, wbcEnd));
            }
            
            // 堆叠雅可比矩阵和误差
            const Jstack = [...JabsWBC, ...JrelWBC];
            const errStack = [...errAbs, ...errRel];
            
            // 伪逆求解
            const JstackPinv = this.pseudoInverse(Jstack);
            
            // 计算关节增量
            for (let i = 0; i < wbcCount; i++) {
                let sum = 0;
                for (let j = 0; j < 16; j++) {
                    sum += JstackPinv[i][j] * errStack[j];
                }
                q[wbcStart + i] += sum * damping;
            }
        }
        
        // 更新
        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(q, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
    }
    
    /**
     * WBC模式：沿轴平移绝对位姿（腰部+双臂协同）
     */
    moveHumanoidAbsolutePoseAlongAxisWBC(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            
            // 获取当前绝对位姿的旋转，用于局部坐标系模式
            const dqAbsArray = this._getHumanoidAbsolutePose(window.humanoidQ);
            if (!dqAbsArray) return;
            const dqAbsCurrent = DQ.createFromArray(dqAbsArray);
            const rotationArray = DQ.getRotation(dqAbsCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
            
            // getTranslationDirection 会根据 isWorldTranslationMode 决定是否应用旋转
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            this.updateHumanoidJointsFromAbsolutePoseDeltaWBC(delta, window.humanoidQ);
        } catch (e) {
            console.error('WBC移动绝对位姿失败:', e);
        }
    }
    
    /**
     * WBC模式：绕轴旋转绝对位姿（腰部+双臂协同）
     */
    rotateHumanoidAbsolutePoseAroundAxisWBC(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            const dqAbsoluteCurrentArray = this._getHumanoidAbsolutePose(window.humanoidQ);
            if (!dqAbsoluteCurrentArray) return;
            
            const dqAbsoluteCurrent = DQ.createFromArray(dqAbsoluteCurrentArray);
            
            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return;
            
            const dqAbsoluteTarget = this.combineDQWithTeachOrder(rotationDQ, dqAbsoluteCurrent, false);
            this.updateHumanoidJointsFromAbsolutePoseTargetWBC(DQ.toArray(dqAbsoluteTarget), window.humanoidQ);
        } catch (e) {
            console.error('WBC旋转绝对位姿失败:', e);
        }
    }
    
    // ==================== 零空间优先级控制：绝对位姿变化，保持相对位姿不变 ====================
    // 策略：使用零空间投影的优先级任务控制
    // 任务1（最高优先级）：左臂末端跟踪目标位姿
    // 任务2（次优先级）：右臂保持相对位姿
    // 任务3（最低优先级）：关节限位避免
    //
    // 公式：dq = J1^+ * e1 + P1 * J2^+ * e2 + P12 * grad_limit
    // 其中 P1 = I - J1^+ * J1 是任务1的零空间投影
    //      P12 = I - J12^+ * J12 是任务1和2的增广雅可比的零空间投影
    
    /**
     * 获取左链（腰部+左臂）的关节索引
     */
    _humanoidLeftChainIndices() {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        return { start: 0, end: w + l, count: w + l };
    }
    
    /**
     * 获取右链（只有右臂，不含腰部）的关节索引
     */
    _humanoidRightChainIndices() {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        const r = this.humanoidRightArmLength;
        return { start: w + l, end: w + l + r, count: r };
    }
    
    /**
     * 从左链关节（腰+左臂）构建完整关节向量
     */
    _humanoidFullQFromLeftChain(currentQ, xLeftChain) {
        const { end: leftChainEnd } = this._humanoidLeftChainIndices();
        return [...xLeftChain, ...currentQ.slice(leftChainEnd)];
    }
    
    /**
     * 从右链关节（只有右臂）构建完整关节向量
     */
    _humanoidFullQFromRightChain(currentQ, xRightChain) {
        const { start: rightStart, end: rightEnd } = this._humanoidRightChainIndices();
        return [...currentQ.slice(0, rightStart), ...xRightChain, ...currentQ.slice(rightEnd)];
    }
    
    /**
     * 零空间优先级控制：根据目标绝对位姿更新关节（保持相对位姿不变）
     * 
     * 策略：使用零空间投影的优先级任务堆叠
     * dq = J1^+ * κ1 * e1 + P1 * J2^+ * κ2 * e2 + P12 * grad_limit
     * 
     * 任务1（最高优先级）：左臂末端跟踪目标位姿
     * 任务2（次优先级）：右臂保持相对位姿
     * 任务3（最低优先级）：关节限位避免（可选）
     * 
     * @param {number[]} dqAbsoluteTargetArray - 目标绝对位姿 DQ 8元素
     * @param {number[]} currentQ - 当前全关节角
     * @param {boolean} enableJointLimitAvoidance - 是否启用关节限位避免，默认为 true
     * @returns {boolean}
     */
    updateHumanoidJointsFromAbsolutePoseTargetDecoupled(dqAbsoluteTargetArray, currentQ, enableJointLimitAvoidance = true) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return false;
        
        // 获取当前状态
        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relCurrentArray = this._getHumanoidRelativePose(currentQ);
        const x1CurrentArray = this._getHumanoidPose1(currentQ);
        const x2CurrentArray = this._getHumanoidPose2(currentQ);
        if (!absCurrentArray || !relCurrentArray || !x1CurrentArray || !x2CurrentArray) return false;
        
        const absCurrent = DQ.createFromArray(absCurrentArray);
        const absTarget = DQ.createFromArray(dqAbsoluteTargetArray);
        const relCurrent = DQ.createFromArray(relCurrentArray);
        const x1Current = DQ.createFromArray(x1CurrentArray);
        
        // 计算绝对位姿变化量：delta = target * inv(current)
        const absCurrentInv = DQ.inverse(absCurrent);
        const delta = DQ.multiply(absTarget, absCurrentInv);
        
        // 左臂新目标位姿：x1_target = delta * x1_current
        const x1Target = DQ.multiply(delta, x1Current);
        let x1TargetArray = DQ.toArray(x1Target);
        
        // 处理DQ双覆盖：选择与当前位姿符号一致的目标
        let dot = 0;
        for (let i = 0; i < 8; i++) dot += x1TargetArray[i] * x1CurrentArray[i];
        if (dot < 0) x1TargetArray = x1TargetArray.map(v => -v);
        
        // 获取关节限位
        const jointLimits = this._getHumanoidJointLimits();
        
        // 获取关节索引
        const n = currentQ.length;  // 全部关节数
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const activeCount = waistCount + leftArmCount + rightArmCount;  // 腰+左臂+右臂
        
        // 增益参数
        const kappa1 = 1.0;     // 任务1增益（左臂跟踪）
        const kappa2 = 0.8;     // 任务2增益（右臂相对位姿）
        const kappa3 = 0.02;    // 任务3增益（关节限位避免，降低以减少冲击）
        const waistGain = 0.5;  // 腰部权重（让腰部动得少）
        const armGain = 1.0;    // 手臂权重
        
        // 迭代参数
        const maxIter = 100;
        const tol = 1e-16;
        const damping = 0.01;   // 阻尼伪逆的阻尼系数
        const maxStep = 0.003;   // 单步最大关节变化（降低以减少冲击）
        
        const qInitial = [...currentQ];
        let q = [...qInitial];
        
        for (let iter = 0; iter < maxIter; iter++) {
            // === 计算任务1误差：左臂末端位姿跟踪 ===
            const x1Now = this._getHumanoidPose1(q);
            if (!x1Now) break;
            
            // 处理DQ双覆盖
            let dot1 = 0;
            for (let i = 0; i < 8; i++) dot1 += x1TargetArray[i] * x1Now[i];
            const sign1 = dot1 < 0 ? -1 : 1;
            
            const e1 = [];
            let e1Norm2 = 0;
            for (let i = 0; i < 8; i++) {
                const err = x1TargetArray[i] - sign1 * x1Now[i];
                e1.push(err);
                e1Norm2 += err * err;
            }
            
            // === 计算任务2误差：右臂相对位姿保持 ===
            // 目标：x2_target = x1_now * conj(rel_original)
            const x1NowDQ = DQ.createFromArray(x1Now.map((v, i) => sign1 * v));
            const relConj = DQ.conjugate(relCurrent);
            const x2Target = DQ.multiply(x1NowDQ, relConj);
            let x2TargetArray = DQ.toArray(x2Target);
            
            const x2Now = this._getHumanoidPose2(q);
            if (!x2Now) break;
            
            // 处理DQ双覆盖
            let dot2 = 0;
            for (let i = 0; i < 8; i++) dot2 += x2TargetArray[i] * x2Now[i];
            if (dot2 < 0) x2TargetArray = x2TargetArray.map(v => -v);
            
            const e2 = [];
            let e2Norm2 = 0;
            for (let i = 0; i < 8; i++) {
                const err = x2TargetArray[i] - x2Now[i];
                e2.push(err);
                e2Norm2 += err * err;
            }
            
            // 收敛检查
            if (e1Norm2 < tol && e2Norm2 < tol) {
                console.log('零空间控制: 迭代收敛, iter=', iter, 'e1=', Math.sqrt(e1Norm2).toFixed(6), 'e2=', Math.sqrt(e2Norm2).toFixed(6));
                break;
            }
            
            if (iter === 0 || iter === maxIter - 1) {
                console.log('零空间控制: iter=', iter, 'e1=', Math.sqrt(e1Norm2).toFixed(6), 'e2=', Math.sqrt(e2Norm2).toFixed(6));
            }
            
            // === 计算雅可比矩阵 ===
            // J1: 左臂位姿对所有活动关节的雅可比 (8 x activeCount)
            const J1Full = this._getHumanoidPose1Jacobian(q);
            if (!J1Full) break;
            const J1 = [];
            for (let i = 0; i < 8; i++) {
                J1.push(J1Full[i].slice(0, activeCount));
            }
            
            // J2: 右臂位姿对所有活动关节的雅可比 (8 x activeCount)
            const J2Full = this._getHumanoidPose2Jacobian(q);
            if (!J2Full) break;
            const J2 = [];
            for (let i = 0; i < 8; i++) {
                J2.push(J2Full[i].slice(0, activeCount));
            }
            
            // === 计算关节权重矩阵 W (对角阵，用数组表示) ===
            const W = [];
            for (let i = 0; i < activeCount; i++) {
                if (i < waistCount) {
                    W.push(waistGain);  // 腰部权重小
                } else {
                    W.push(armGain);    // 手臂权重大
                }
            }
            
            // === 计算任务1的加权伪逆和零空间投影 ===
            // J1_pinv = W^-1 * J1^T * (J1 * W^-1 * J1^T + λI)^-1
            const J1Pinv = this._weightedDampedPseudoInverse(J1, W, damping);
            if (!J1Pinv) break;
            
            // P1 = I - J1_pinv * J1 (零空间投影矩阵)
            const P1 = this._computeNullspaceProjection(J1Pinv, J1, activeCount);
            
            // === 计算任务2在任务1零空间中的伪逆 ===
            // J2_bar = J2 * P1
            const J2Bar = this._matMul(J2, P1);
            const J2BarPinv = this._weightedDampedPseudoInverse(J2Bar, W, damping);
            if (!J2BarPinv) break;
            
            // === 计算增广雅可比的零空间投影 P12 ===
            // P12 = P1 - J2Bar_pinv * J2Bar
            const P12 = this._computeNullspaceProjection2(P1, J2BarPinv, J2Bar, activeCount);
            
            // === 计算任务3：关节限位避免梯度 ===
            const gradLimit = this._computeJointLimitGradient(q, jointLimits, activeCount);
            
            // === 计算关节速度增量 ===
            // dq = J1^+ * κ1 * e1 + P1 * J2^+ * κ2 * e2 + P12 * κ3 * gradLimit
            const dq = new Array(activeCount).fill(0);
            
            // 任务1贡献：J1^+ * κ1 * e1
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < 8; j++) {
                    dq[i] += J1Pinv[i][j] * kappa1 * e1[j];
                }
            }
            
            // 任务2贡献：P1 * J2Bar^+ * κ2 * e2
            // 先计算 J2Bar^+ * e2
            const dq2_temp = new Array(activeCount).fill(0);
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < 8; j++) {
                    dq2_temp[i] += J2BarPinv[i][j] * kappa2 * e2[j];
                }
            }
            // 再乘以 P1 投影到零空间
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < activeCount; j++) {
                    dq[i] += P1[i][j] * dq2_temp[j];
                }
            }
            
            // 任务3贡献：P12 * κ3 * gradLimit（仅在启用关节限位避免时）
            if (enableJointLimitAvoidance) {
                for (let i = 0; i < activeCount; i++) {
                    for (let j = 0; j < activeCount; j++) {
                        dq[i] += P12[i][j] * kappa3 * gradLimit[j];
                    }
                }
            }
            
            // 应用关节权重并限制步长
            for (let i = 0; i < activeCount; i++) {
                let step = dq[i] * W[i];
                step = Math.max(-maxStep, Math.min(maxStep, step));
                q[i] += step;
            }
            
            // 应用关节限位（硬限位始终生效）
            if (jointLimits) {
                for (let i = 0; i < activeCount && i < jointLimits.length; i++) {
                    const lim = jointLimits[i];
                    if (lim) {
                        q[i] = Math.max(lim.lower, Math.min(lim.upper, q[i]));
                    }
                }
            }
        }
        
        // 应用结果
        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }

    async _updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
        dqAbsoluteTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        keepBodyZVertical = false,
        fixedRelativeTargetArray = null,
        internalOptions = null
    ) {
        const solverMode = internalOptions?.solverMode === 'mj_restore' ? 'mj_restore' : 'manifold';
        const solverDisplayName = getHumanoidJogSolverDisplayName(solverMode);
        const isMJRestore = solverMode === 'mj_restore';
        const candidateOnly = internalOptions?.candidateOnly === true;
        const translationAxisGuard = getHumanoidJogTranslationAxisGuard(internalOptions);
        const allowArmOnlyFallback = internalOptions?.allowArmOnlyFallback !== false;
        const suppressFailureNotification = internalOptions?.suppressFailureNotification === true;
        const requestToken = Number.isFinite(internalOptions?.requestToken) ? internalOptions.requestToken : 0;
        const preferWorker = internalOptions?.preferWorker === true;
        const jogMetricDirection = internalOptions?.metricDirection === 'decrease' ? 'decrease' : 'increase';
        const inheritedFallbackReason = typeof internalOptions?.fallbackReason === 'string'
            ? internalOptions.fallbackReason
            : null;
        const requestedJogMetric = typeof internalOptions?.jogMetricOverride === 'string'
            ? internalOptions.jogMetricOverride
            : '';
        const iterativeMetricProjectionPlan = (
            internalOptions?.iterativeMetricProjectionPlan &&
            typeof internalOptions.iterativeMetricProjectionPlan === 'object'
        )
            ? {
                metricIterations: Number.isFinite(internalOptions.iterativeMetricProjectionPlan.metricIterations)
                    ? Math.max(0, Math.floor(internalOptions.iterativeMetricProjectionPlan.metricIterations))
                    : 0,
                restoreIterations: Number.isFinite(internalOptions.iterativeMetricProjectionPlan.restoreIterations)
                    ? Math.max(0, Math.floor(internalOptions.iterativeMetricProjectionPlan.restoreIterations))
                    : 0
            }
            : null;
        const mjRestoreProfile = isMJRestore
            ? resolveHumanoidJogMJRestoreProfile({
                includeWaist,
                fallbackReason: inheritedFallbackReason,
                config: this.vfiConfig || {}
            })
            : null;
        if (!this.humanoidModel || !window.humanoidQ || !Array.isArray(currentQ) || currentQ.length === 0) {
            return this._setIKFailureStatus({
                solver: solverMode,
                mode: 'humanoid_absolute',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relCurrentArray = this._getHumanoidRelativePose(currentQ);
        if (!absCurrentArray || !relCurrentArray) {
            return this._setIKFailureStatus({
                solver: solverMode,
                mode: 'humanoid_absolute',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const relConstraintArray = (
            Array.isArray(fixedRelativeTargetArray) &&
            fixedRelativeTargetArray.length === 8 &&
            fixedRelativeTargetArray.every((value) => Number.isFinite(Number(value)))
        )
            ? fixedRelativeTargetArray.map((value) => Number(value))
            : relCurrentArray.slice();

        const { waistCount, armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: solverMode,
                mode: 'humanoid_absolute',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const manipActiveLimits = enableJointLimitAvoidance ? hardLimits : null;
        const qInitialFull = currentQ.slice();
        const qInitialActive = this._sliceHumanoidActiveJoints(qInitialFull, armStart, activeCount);
        const maxStep = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreMaxStep) ? cfg.mjRestoreMaxStep : (Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.05))
            : (Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1);
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const wvfiMin = this._getWVFILowerBound(includeWaist);
        const restorationWVFILowerBound = isMJRestore ? 0 : wvfiMin;
        const hardTol = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreHardTolerance) ? Math.max(0, cfg.mjRestoreHardTolerance) : 1e-8)
            : 1e-12;
        const bodyTol = keepBodyZVertical
            ? (isMJRestore
                ? (Number.isFinite(cfg.mjRestoreBodyTolerance) ? cfg.mjRestoreBodyTolerance : 5e-5)
                : (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5))
            : hardTol;
        const restorationMaxIterations = isMJRestore
            ? mjRestoreProfile.maxIterations
            : (Number.isFinite(cfg.manifoldMaxIter) ? Math.max(1, Math.floor(cfg.manifoldMaxIter)) : 120);
        const restorationDamping = isMJRestore
            ? mjRestoreProfile.damping
            : (Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4);
        const restorationReferenceStiffness = isMJRestore
            ? mjRestoreProfile.referenceStiffness
            : (Number.isFinite(cfg.manifoldReferenceStiffness) ? Math.max(0, cfg.manifoldReferenceStiffness) : 1.0);
        const restorationReferenceDamping = isMJRestore
            ? mjRestoreProfile.referenceDamping
            : (Number.isFinite(cfg.manifoldReferenceDamping) ? Math.max(0, cfg.manifoldReferenceDamping) : 0.35);
        const restorationMinScale = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreLineSearchMinScale) ? Math.max(1e-8, cfg.mjRestoreLineSearchMinScale) : 1e-6)
            : (Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6);
        const restorationReductionFactor = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreLineSearchReductionFactor) ? cfg.mjRestoreLineSearchReductionFactor : 0.5)
            : (Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5);
        const nullspaceRestoreMaxIterations = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreNullspaceRestoreMaxIter) ? Math.max(1, Math.floor(cfg.mjRestoreNullspaceRestoreMaxIter)) : 24)
            : (Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter) ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter)) : 24);
        const mjPolishMaxIterations = Number.isFinite(cfg.mjRestorePolishMaxIter)
            ? (isMJRestore ? mjRestoreProfile.polishMaxIterations : Math.max(1, Math.floor(cfg.mjRestorePolishMaxIter)))
            : (isMJRestore ? mjRestoreProfile.polishMaxIterations : 64);
        const mjPolishActivationResidual = isMJRestore
            ? mjRestoreProfile.polishActivationResidual
            : (Number.isFinite(cfg.mjRestorePolishActivationResidual)
                ? Math.max(0, cfg.mjRestorePolishActivationResidual)
                : 1e-4);
        const mjPolishDamping = isMJRestore
            ? mjRestoreProfile.polishDamping
            : (Number.isFinite(cfg.mjRestorePolishDamping)
                ? Math.max(1e-12, cfg.mjRestorePolishDamping)
                : 1e-6);
        const relativeEqualityWeight = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreRelativeWeight) ? cfg.mjRestoreRelativeWeight : 1.0)
            : (Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0);
        const bodyEqualityWeight = keepBodyZVertical
            ? (isMJRestore
                ? (Number.isFinite(cfg.mjRestoreBodyVerticalWeight) ? cfg.mjRestoreBodyVerticalWeight : 4.0)
                : (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0))
            : 1.0;
        const bodyReferenceStiffness = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreBodyReferenceStiffness) ? Math.max(0, cfg.mjRestoreBodyReferenceStiffness) : 0.6)
            : restorationReferenceStiffness * 0.6;
        const bodyReferenceDamping = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreBodyReferenceDamping) ? Math.max(0, cfg.mjRestoreBodyReferenceDamping) : 0.45)
            : restorationReferenceDamping + 0.1;

        const norm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
        const nowMs = () => (typeof performance !== 'undefined' && typeof performance.now === 'function')
            ? performance.now()
            : Date.now();
        const mjTimingStartMs = isMJRestore ? nowMs() : Number.NaN;

        const selectedJogMetric = requestedJogMetric.trim().length > 0
            ? normalizeHumanoidJogManifoldJogMetric(requestedJogMetric)
            : this._getHumanoidManifoldJogSecondaryMetric();
        const resolvedJogMetric = resolveHumanoidJogManifoldJogMetric(selectedJogMetric, {
            hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend()
        });
        const jogMetricDiagnostic = resolvedJogMetric.diagnosticMetric || resolvedJogMetric.metric;
        const isHierarchicalVFIJogMetric = (
            jogMetricDiagnostic === 'w_abs_hier_vfi' &&
            resolvedJogMetric.metric !== 'w_abs'
        );
        const isPosePreservingVFIMetric = (
            isHierarchicalVFIJogMetric === true ||
            resolvedJogMetric.metric === 'w_abs_vfi'
        );
        const absoluteTargetError = Array.isArray(dqAbsoluteTargetArray)
            ? this._computePoseErrorNorm(this._signedPoseError(absCurrentArray, dqAbsoluteTargetArray))
            : Number.POSITIVE_INFINITY;
        const relativeTargetError = Array.isArray(relConstraintArray)
            ? this._computePoseErrorNorm(this._signedPoseError(relCurrentArray, relConstraintArray))
            : Number.POSITIVE_INFINITY;
        const isMetricOnlyJogRequest = (
            isMJRestore &&
            resolvedJogMetric.shouldSkip !== true &&
            translationAxisGuard === null &&
            absoluteTargetError <= 1e-12 &&
            relativeTargetError <= 1e-12
        );
        const useProjectionOnlyMetricJog = shouldUseHumanoidJogMJProjectionOnlyMetricStep({
            isMetricOnlyJogRequest,
            requestedMetric: jogMetricDiagnostic,
            stageMetric: jogMetricDiagnostic
        });
        const mjRestoreRegularizationWeights = isMJRestore
            ? buildHumanoidJogMJRegularizationWeights({
                activeCount,
                waistActiveCount: armStart === 0 ? Math.min(waistCount, activeCount) : 0,
                includeWaist,
                keepBodyZVertical,
                isMetricOnlyJogRequest,
                metricShouldSkip: resolvedJogMetric.shouldSkip === true,
                config: cfg
            })
            : null;
        const useLocalRegularizedMJRestore = Array.isArray(mjRestoreRegularizationWeights) &&
            mjRestoreRegularizationWeights.length === activeCount;
        const metricButtonCommandTolerance = isMetricOnlyJogRequest
            ? (Number.isFinite(cfg.mjRestoreMetricButtonCommandTolerance)
                ? Math.max(0, cfg.mjRestoreMetricButtonCommandTolerance)
                : 1e-3)
            : hardTol;
        const metricButtonInvariantTolerance = isMetricOnlyJogRequest
            ? (Number.isFinite(cfg.mjRestoreMetricButtonInvariantTolerance)
                ? Math.max(0, cfg.mjRestoreMetricButtonInvariantTolerance)
                : 1e-4)
            : hardTol;
        const metricToleranceProfile = resolveHumanoidJogMJMetricTolerances({
            isMetricOnlyJogRequest,
            isHierarchicalVFIJogMetric: isPosePreservingVFIMetric,
            hardTolerance: hardTol,
            metricButtonCommandTolerance,
            metricButtonInvariantTolerance,
            activeLimitRetreatCommandTolerance: Number.isFinite(cfg.mjRestoreActiveLimitRetreatCommandTolerance)
                ? cfg.mjRestoreActiveLimitRetreatCommandTolerance
                : 5e-3,
            activeLimitRetreatInvariantTolerance: Number.isFinite(cfg.mjRestoreActiveLimitRetreatInvariantTolerance)
                ? cfg.mjRestoreActiveLimitRetreatInvariantTolerance
                : 1e-3
        });
        const activeLimitRetreatCommandTolerance = metricToleranceProfile.activeLimitRetreatCommandTolerance;
        const activeLimitRetreatInvariantTolerance = metricToleranceProfile.activeLimitRetreatInvariantTolerance;
        const metricRefinementTolerance = metricToleranceProfile.metricRefinementTolerance;
        const practicalCommandTolerance = !isMetricOnlyJogRequest
            ? (Number.isFinite(cfg.mjRestorePracticalCommandTolPose)
                ? Math.max(0, cfg.mjRestorePracticalCommandTolPose)
                : 1e-4)
            : Number.NaN;
        const practicalInvariantTolerance = !isMetricOnlyJogRequest
            ? (Number.isFinite(cfg.practicalInvariantTol) ? Math.max(0, cfg.practicalInvariantTol) : 2.5e-4)
            : Number.NaN;
        const mjEnableNullspaceRefine = isMJRestore
            ? shouldEnableHumanoidJogMJNullspaceRefine({
                enabled: cfg.mjRestoreEnableNullspaceRefine === true,
                metric: resolvedJogMetric.requestedMetric || resolvedJogMetric.metric
            })
            : true;
        const mjNullspaceStepSize = isMJRestore
            ? (Number.isFinite(cfg.mjRestoreNullspaceStepSize)
                ? Math.max(0, cfg.mjRestoreNullspaceStepSize)
                : (Number.isFinite(cfg.manifoldNullspaceStepSize) ? Math.max(0, cfg.manifoldNullspaceStepSize) : 0.01))
            : (Number.isFinite(cfg.manifoldNullspaceStepSize) ? Math.max(0, cfg.manifoldNullspaceStepSize) : 0.002);
        const mjProjectionOnlyStepSize = useProjectionOnlyMetricJog
            ? Math.min(
                mjNullspaceStepSize,
                Number.isFinite(cfg.mjRestoreMetricProjectionStepSize)
                    ? Math.max(1e-6, cfg.mjRestoreMetricProjectionStepSize)
                    : 0.002
            )
            : mjNullspaceStepSize;
        const mjProjectionOnlyPasses = useProjectionOnlyMetricJog
            ? (Number.isFinite(cfg.mjRestoreMetricProjectionPasses)
                ? Math.max(1, Math.floor(cfg.mjRestoreMetricProjectionPasses))
                : 16)
            : 1;
        const workerAvailable = isMJRestore &&
            preferWorker === true &&
            useLocalRegularizedMJRestore !== true &&
            this._canUseHumanoidJogMJRestoreWorker();
        const mjWorkerSecondaryMetricRequest = buildHumanoidJogMJWorkerSecondaryMetricRequest({
            metric: resolvedJogMetric.metric,
            preferWorker,
            workerAvailable,
            nullspaceRefineEnabled: mjEnableNullspaceRefine === true,
            stepSize: mjNullspaceStepSize,
            restoreMaxIterations: nullspaceRestoreMaxIterations,
            waistActiveCount: armStart === 0 ? Math.min(waistCount, activeCount) : 0,
            hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend()
        });
        const mjWorkerHandlesMetric = isMJRestore &&
            mjWorkerSecondaryMetricRequest !== null &&
            workerAvailable;
        const shouldComputeMJTrialManipData = shouldComputeHumanoidJogMJTrialManipData({
            metric: resolvedJogMetric.metric,
            nullspaceRefineEnabled: mjEnableNullspaceRefine,
            wvfiLowerBound: restorationWVFILowerBound
        });
        const shouldComputeMJInitialManipData = shouldComputeHumanoidJogMJInitialManipData({
            metric: resolvedJogMetric.metric,
            nullspaceRefineEnabled: mjEnableNullspaceRefine,
            wvfiLowerBound: restorationWVFILowerBound,
            workerHandlesMetric: mjWorkerHandlesMetric
        });
        const evaluateProblemVFIMinDistance = (problem) => {
            const qFull = Array.isArray(problem?.metadata?.qFull) ? problem.metadata.qFull : null;
            if (!qFull) return Number.NaN;
            const constraints = this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits);
            return summarizeManipulabilityVFIConstraints(constraints).minDistance;
        };

        const evaluateProblem = (qActiveEval, options = {}) => {
            const qFull = this._writeHumanoidActiveJoints(qInitialFull, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const shouldComputeManipData = typeof options.computeManipData === 'boolean'
                ? options.computeManipData
                : (!isMJRestore || shouldComputeMJTrialManipData);
            const cctsm = shouldComputeManipData
                ? this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
                )
                : null;
            const wvfiGradient = shouldComputeManipData
                ? this._computeProtectedManipulabilityGradient({
                    q: qFull,
                    armStart,
                    activeCount,
                    mode: 'absolute',
                    Jcmd: Jabs,
                    Jinvariant: protectedRows,
                    activeLimits: hardLimits,
                    forceInvariantEquality: true
                })
                : null;

            const problemBuilder = isMJRestore ? buildHumanoidJogMJProblem : buildHumanoidJogManifoldProblem;
            const problem = problemBuilder({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: dqAbsoluteTargetArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: relativeEqualityWeight,
                bodyWeight: bodyEqualityWeight,
                wvfiState: shouldComputeManipData
                    ? () => ({
                        value: cctsm?.w_vfi ?? Number.NaN,
                        gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                    })
                    : null,
                wvfiLowerBound: restorationWVFILowerBound,
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull,
                    armStart,
                    activeCount,
                    includeWaist
                }),
                referenceModel: isMJRestore
                    ? {
                        absolute: {
                            stiffness: restorationReferenceStiffness,
                            damping: restorationReferenceDamping
                        },
                        relative: {
                            stiffness: restorationReferenceStiffness,
                            damping: restorationReferenceDamping
                        },
                        bodyZ: {
                            stiffness: bodyReferenceStiffness,
                            damping: bodyReferenceDamping
                        }
                    }
                    : undefined
            });
            problem.metadata = {
                qFull,
                cctsm,
                Jabs,
                protectedRows
            };
            return problem;
        };
        const initialProblem = evaluateProblem(qInitialActive, {
            computeManipData: !isMJRestore || shouldComputeMJInitialManipData
        });
        if (!initialProblem) {
            return this._setIKFailureStatus({
                solver: solverMode,
                mode: 'humanoid_absolute',
                reason: 'problem_eval_failed',
                message: `${solverDisplayName} 问题构建失败。`
            });
        }

        const restoreFn = isMJRestore
            ? async ({
                qInitial,
                evaluateProblem: restoreEvaluateProblem,
                maxIterations,
                damping,
                referenceStiffness,
                referenceDamping,
                initialScale,
                minScale,
                reductionFactor,
                equalityTolerance,
                inequalityTolerance
            } = {}) => {
                const jointLimitActivationThreshold = Number.isFinite(cfg.jointLimitManipulabilityActivationThreshold)
                    ? Math.max(0, cfg.jointLimitManipulabilityActivationThreshold)
                    : 0.05;
                const referenceModel = {
                    absolute: {
                        stiffness: Number.isFinite(referenceStiffness)
                            ? Math.max(0, referenceStiffness)
                            : restorationReferenceStiffness,
                        damping: Number.isFinite(referenceDamping)
                            ? Math.max(0, referenceDamping)
                            : restorationReferenceDamping
                    },
                    relative: {
                        stiffness: Number.isFinite(referenceStiffness)
                            ? Math.max(0, referenceStiffness)
                            : restorationReferenceStiffness,
                        damping: Number.isFinite(referenceDamping)
                            ? Math.max(0, referenceDamping)
                            : restorationReferenceDamping
                    },
                    bodyZ: {
                        stiffness: bodyReferenceStiffness,
                        damping: bodyReferenceDamping
                    }
                };
                const commonWasmRequest = {
                    qInitial,
                    evaluateProblem: restoreEvaluateProblem,
                    activeStart: armStart,
                    activeCount,
                    absoluteTarget: dqAbsoluteTargetArray,
                    relativeTarget: relConstraintArray,
                    keepBodyZVertical,
                    bodyEqualityTolerance: bodyTol,
                    absoluteWeight: 1.0,
                    relativeWeight: relativeEqualityWeight,
                    bodyWeight: bodyEqualityWeight,
                    lowerBounds,
                    upperBounds,
                    stepBounds: jointMaxSteps,
                    wvfiLowerBound: restorationWVFILowerBound,
                    jointLimitActivationThreshold,
                    referenceModel,
                    maxIterations,
                    damping,
                    initialScale,
                    minScale,
                    reductionFactor,
                    equalityTolerance,
                    inequalityTolerance
                };
                let wasmAttempt = null;
                if (preferWorker && useLocalRegularizedMJRestore !== true) {
                    try {
                        wasmAttempt = await attemptHumanoidJogMJRestoreWasmWorker({
                            workerClient: this._getHumanoidJogMJWorkerClient(),
                            modelSpec: this.humanoidMJWorkerModelSpec,
                            secondaryMetricRequest: qInitial === qInitialActive ? mjWorkerSecondaryMetricRequest : null,
                            ...commonWasmRequest
                        });
                    } catch (error) {
                        console.warn('MuJoCo 风格恢复 Worker 求解失败，回退同步 WASM/JS:', error);
                    }
                }
                if (!wasmAttempt && useLocalRegularizedMJRestore !== true) {
                    wasmAttempt = attemptHumanoidJogMJRestoreWasm({
                        humanoidModel: this.humanoidModel,
                        ...commonWasmRequest
                    });
                }
                if (wasmAttempt) {
                    return buildHumanoidJogMJRestoreWasmEnvelope({
                        attempt: wasmAttempt,
                        qInitial,
                        evaluateProblem: restoreEvaluateProblem
                    });
                }
                return restoreHumanoidJogMJ({
                    qInitial,
                    evaluateProblem: restoreEvaluateProblem,
                    maxIterations,
                    damping,
                    regularizationWeights: mjRestoreRegularizationWeights,
                    referenceStiffness,
                    referenceDamping,
                    initialScale,
                    minScale,
                    reductionFactor,
                    equalityTolerance,
                    inequalityTolerance
                });
            }
            : restoreHumanoidJogManifold;
        const restorationResult = await restoreFn({
            qInitial: qInitialActive,
            evaluateProblem,
            maxIterations: restorationMaxIterations,
            damping: restorationDamping,
            referenceStiffness: restorationReferenceStiffness,
            referenceDamping: restorationReferenceDamping,
            initialScale: 1,
            minScale: restorationMinScale,
            reductionFactor: restorationReductionFactor,
            equalityTolerance: hardTol,
            inequalityTolerance: hardTol
        });

        let effectiveRestorationResult = restorationResult;
        let mjPolishResult = null;
        let mjPolishState = 'skipped';
        if (isMJRestore && shouldAttemptHumanoidJogMJPolish({
            restorationResult,
            activationResidual: mjPolishActivationResidual
        })) {
            mjPolishResult = restoreHumanoidJogManifold({
                qInitial: restorationResult.qApplied,
                evaluateProblem,
                maxIterations: mjPolishMaxIterations,
                damping: mjPolishDamping,
                referenceStiffness: 1,
                referenceDamping: 0,
                initialScale: 1,
                minScale: restorationMinScale,
                reductionFactor: restorationReductionFactor,
                equalityTolerance: hardTol,
                inequalityTolerance: hardTol
            });
            const polishImproved = shouldApplyHumanoidJogMJPolish({
                restorationResult,
                polishResult: mjPolishResult
            });
            mjPolishState = mjPolishResult?.converged === true
                ? 'converged'
                : (polishImproved ? 'best_effort' : 'not_improved');
            if (polishImproved) {
                effectiveRestorationResult = {
                    ...mjPolishResult,
                    reason: mjPolishResult.converged === true ? 'polish_converged' : 'polish_best_effort',
                    iterations: restorationResult.iterations + (Number.isFinite(mjPolishResult.iterations) ? mjPolishResult.iterations : 0),
                    acceptedIterations: restorationResult.acceptedIterations + (Number.isFinite(mjPolishResult.acceptedIterations) ? mjPolishResult.acceptedIterations : 0)
                };
            }
        }

        let effectiveResult = effectiveRestorationResult;
        let nullspaceResult = null;
        let nullspaceRefineResult = null;
        let fallbackReason = inheritedFallbackReason;
        const restoredProblem = effectiveRestorationResult.currentProblem || evaluateProblem(effectiveRestorationResult.qApplied);
        const restoredWVFI = restoredProblem?.inequalities?.wvfiFloor?.value;
        let jogMetricState = effectiveRestorationResult.converged === true ? 'skipped(none)' : 'skipped(waiting_for_restore)';
        let jogMetricBefore = Number.NaN;
        let jogMetricAfter = Number.NaN;
        let jogMetricApplied = false;
        let jogMetricAxis = resolvedJogMetric.family === 'directional' ? (this.lastHumanoidJogAxis || '') : '';
        const workerMetricAttempt = isMJRestore
            ? effectiveRestorationResult?.secondaryMetricAttempt ?? null
            : null;
        const retryRejectedWorkerMetricLocally = shouldRetryHumanoidJogMJLocalRefineAfterWorkerMetricAttempt({
            metric: resolvedJogMetric.metric,
            isMetricOnlyJogRequest,
            workerMetricAttempt
        });
        if (effectiveRestorationResult.converged &&
            restoredProblem &&
            workerMetricAttempt &&
            workerMetricAttempt.metric === resolvedJogMetric.metric &&
            retryRejectedWorkerMetricLocally !== true) {
            jogMetricState = workerMetricAttempt.state || 'unknown';
            jogMetricBefore = workerMetricAttempt.before;
            jogMetricAfter = workerMetricAttempt.after;
            jogMetricApplied = workerMetricAttempt.applied === true;
        } else if (effectiveRestorationResult.converged && restoredProblem && mjEnableNullspaceRefine) {
            const nullspaceStepSize = mjNullspaceStepSize;
            if (
                iterativeMetricProjectionPlan &&
                resolvedJogMetric.metric === 'w_abs_hier_vfi' &&
                (iterativeMetricProjectionPlan.metricIterations > 0 || iterativeMetricProjectionPlan.restoreIterations > 0)
            ) {
                const metricAttempt = await this._runHumanoidIterativeMetricProjectionPlan({
                    baseProblem: restoredProblem,
                    baseQActive: effectiveRestorationResult.qApplied,
                    metric: resolvedJogMetric.metric,
                    direction: jogMetricDirection,
                    axis: jogMetricAxis,
                    evaluateProblem,
                    armStart,
                    activeCount,
                    activeLimits: hardLimits,
                    lowerBounds,
                    upperBounds,
                    stepBounds: jointMaxSteps,
                    wvfiMin: restorationWVFILowerBound,
                    restoreMaxIterations: 1,
                    damping: restorationDamping,
                    minScale: restorationMinScale,
                    reductionFactor: restorationReductionFactor,
                    equalityTolerance: metricRefinementTolerance,
                    inequalityTolerance: metricRefinementTolerance,
                    stepSize: mjProjectionOnlyStepSize,
                    restoreFn,
                    restoreConfig: {
                        referenceStiffness: restorationReferenceStiffness,
                        referenceDamping: restorationReferenceDamping
                    },
                    metricIterations: iterativeMetricProjectionPlan.metricIterations,
                    restoreIterations: iterativeMetricProjectionPlan.restoreIterations
                });
                jogMetricState = metricAttempt.state;
                jogMetricBefore = metricAttempt.before;
                jogMetricAfter = metricAttempt.after;
                jogMetricApplied = metricAttempt.applied === true;
                nullspaceResult = metricAttempt.nullspaceResult;
                nullspaceRefineResult = metricAttempt.refineResult;
                if (metricAttempt.applied === true && metricAttempt.refineResult) {
                    effectiveResult = metricAttempt.refineResult;
                }
            } else if (resolvedJogMetric.family === 'hierarchical') {
                const primaryMetric = resolvedJogMetric.metric === 'w_abs_hier_vfi'
                    ? 'w_abs_arm_vfi'
                    : 'w_abs_arm';
                const primaryAttempt = await this._attemptHumanoidJogManifoldMetricRefinement({
                    baseProblem: restoredProblem,
                    baseQActive: effectiveRestorationResult.qApplied,
                    metric: primaryMetric,
                    direction: jogMetricDirection,
                    axis: jogMetricAxis,
                    evaluateProblem,
                    armStart,
                    activeCount,
                    activeLimits: hardLimits,
                    lowerBounds,
                    upperBounds,
                    stepBounds: jointMaxSteps,
                    wvfiMin: restorationWVFILowerBound,
                    restoreMaxIterations: nullspaceRestoreMaxIterations,
                    damping: restorationDamping,
                    minScale: restorationMinScale,
                    reductionFactor: restorationReductionFactor,
                    equalityTolerance: metricRefinementTolerance,
                    inequalityTolerance: metricRefinementTolerance,
                    stepSize: shouldUseHumanoidJogMJProjectionOnlyMetricStep({
                        isMetricOnlyJogRequest,
                        requestedMetric: jogMetricDiagnostic,
                        stageMetric: primaryMetric
                    }) ? mjProjectionOnlyStepSize : nullspaceStepSize,
                    preferWVFIFallbackOnClip: isPosePreservingVFIMetric,
                    directionFallbackMetric: isHierarchicalVFIJogMetric
                        ? 'w_abs_hier_vfi'
                        : '',
                    preferActiveLimitRetreat: isPosePreservingVFIMetric,
                    projectionOnly: shouldUseHumanoidJogMJProjectionOnlyMetricStep({
                        isMetricOnlyJogRequest,
                        requestedMetric: jogMetricDiagnostic,
                        stageMetric: primaryMetric
                    }),
                    projectionPasses: shouldUseHumanoidJogMJProjectionOnlyMetricStep({
                        isMetricOnlyJogRequest,
                        requestedMetric: jogMetricDiagnostic,
                        stageMetric: primaryMetric
                    }) ? mjProjectionOnlyPasses : 1,
                    restoreFn,
                    restoreConfig: {
                        referenceStiffness: restorationReferenceStiffness,
                        referenceDamping: restorationReferenceDamping
                    }
                });
                jogMetricBefore = this._evaluateHumanoidJogManifoldMetricValue(
                    resolvedJogMetric.metric,
                    restoredProblem?.metadata?.cctsm,
                    jogMetricAxis
                );
                jogMetricAfter = jogMetricBefore;
                jogMetricState = primaryAttempt.state;
                if (primaryAttempt.applied === true) {
                    nullspaceResult = primaryAttempt.nullspaceResult;
                    nullspaceRefineResult = primaryAttempt.refineResult;
                    effectiveResult = primaryAttempt.refineResult;
                    const primaryProblem = primaryAttempt.finalProblem || (primaryAttempt.refineResult?.currentProblem ?? null);
                    const secondaryMetric = resolvedJogMetric.metric === 'w_abs_hier_vfi' ? 'w_abs_vfi' : 'w_abs';
                    const secondaryAttempt = await this._attemptHumanoidJogManifoldMetricRefinement({
                        baseProblem: primaryProblem,
                        baseQActive: primaryAttempt.refineResult?.qApplied ?? effectiveRestorationResult.qApplied,
                        metric: resolvedJogMetric.metric,
                        direction: jogMetricDirection,
                        axis: jogMetricAxis,
                        evaluateProblem,
                        armStart,
                        activeCount,
                        activeLimits: hardLimits,
                        lowerBounds,
                        upperBounds,
                        stepBounds: jointMaxSteps,
                        wvfiMin: restorationWVFILowerBound,
                        restoreMaxIterations: nullspaceRestoreMaxIterations,
                        damping: restorationDamping,
                        minScale: restorationMinScale,
                        reductionFactor: restorationReductionFactor,
                        equalityTolerance: metricRefinementTolerance,
                        inequalityTolerance: metricRefinementTolerance,
                        stepSize: shouldUseHumanoidJogMJProjectionOnlyMetricStep({
                            isMetricOnlyJogRequest,
                            requestedMetric: jogMetricDiagnostic,
                            stageMetric: resolvedJogMetric.metric
                        }) ? mjProjectionOnlyStepSize : nullspaceStepSize,
                        preferWVFIFallbackOnClip: isHierarchicalVFIJogMetric,
                        preferActiveLimitRetreat: isHierarchicalVFIJogMetric,
                        projectionOnly: shouldUseHumanoidJogMJProjectionOnlyMetricStep({
                            isMetricOnlyJogRequest,
                            requestedMetric: jogMetricDiagnostic,
                            stageMetric: resolvedJogMetric.metric
                        }),
                        projectionPasses: shouldUseHumanoidJogMJProjectionOnlyMetricStep({
                            isMetricOnlyJogRequest,
                            requestedMetric: jogMetricDiagnostic,
                            stageMetric: resolvedJogMetric.metric
                        }) ? mjProjectionOnlyPasses : 1,
                        restoreFn,
                        restoreConfig: {
                            referenceStiffness: restorationReferenceStiffness,
                            referenceDamping: restorationReferenceDamping
                        }
                    });
                    const armGuard = evaluateHumanoidJogManifoldNonDegradation({
                        before: this._evaluateHumanoidJogManifoldMetricValue(primaryMetric, primaryProblem?.metadata?.cctsm, jogMetricAxis),
                        after: this._evaluateHumanoidJogManifoldMetricValue(primaryMetric, secondaryAttempt.finalProblem?.metadata?.cctsm, jogMetricAxis),
                        direction: jogMetricDirection
                    });
                    const vfiDistanceGuard = resolvedJogMetric.metric === 'w_abs_hier_vfi'
                        ? evaluateHumanoidJogManifoldNonDegradation({
                            before: evaluateProblemVFIMinDistance(primaryProblem),
                            after: evaluateProblemVFIMinDistance(secondaryAttempt.finalProblem),
                            direction: 'increase'
                        })
                        : { applied: true, reason: 'protected' };
                    jogMetricAfter = this._evaluateHumanoidJogManifoldMetricValue(
                        resolvedJogMetric.metric,
                        (secondaryAttempt.applied === true && armGuard.applied === true && vfiDistanceGuard.applied === true
                            ? secondaryAttempt.finalProblem?.metadata?.cctsm
                            : primaryProblem?.metadata?.cctsm),
                        jogMetricAxis
                    );
                    jogMetricApplied = true;
                    if (secondaryAttempt.applied === true && armGuard.applied === true && vfiDistanceGuard.applied === true) {
                        jogMetricState = 'active';
                        nullspaceResult = secondaryAttempt.nullspaceResult;
                        nullspaceRefineResult = secondaryAttempt.refineResult;
                        effectiveResult = secondaryAttempt.refineResult;
                    } else {
                        jogMetricState = secondaryAttempt.applied !== true
                            ? `active(primary_only; secondary=${secondaryAttempt.state})`
                            : (armGuard.applied !== true
                                ? 'active(primary_only; secondary=arm_guard)'
                                : 'active(primary_only; secondary=vfi_guard)');
                    }
                }
            } else if (!resolvedJogMetric.shouldSkip) {
                const metricAttempt = await this._attemptHumanoidJogManifoldMetricRefinement({
                    baseProblem: restoredProblem,
                    baseQActive: effectiveRestorationResult.qApplied,
                    metric: resolvedJogMetric.metric,
                    direction: jogMetricDirection,
                    axis: jogMetricAxis,
                    evaluateProblem,
                    armStart,
                    activeCount,
                    activeLimits: hardLimits,
                    lowerBounds,
                    upperBounds,
                    stepBounds: jointMaxSteps,
                    wvfiMin: restorationWVFILowerBound,
                    restoreMaxIterations: nullspaceRestoreMaxIterations,
                    damping: restorationDamping,
                    minScale: restorationMinScale,
                    reductionFactor: restorationReductionFactor,
                    equalityTolerance: metricRefinementTolerance,
                    inequalityTolerance: metricRefinementTolerance,
                    stepSize: useProjectionOnlyMetricJog ? mjProjectionOnlyStepSize : nullspaceStepSize,
                    preferWVFIFallbackOnClip: resolvedJogMetric.metric === 'w_abs_vfi',
                    preferActiveLimitRetreat: isPosePreservingVFIMetric,
                    projectionOnly: useProjectionOnlyMetricJog,
                    projectionPasses: mjProjectionOnlyPasses,
                    restoreFn,
                    restoreConfig: {
                        referenceStiffness: restorationReferenceStiffness,
                        referenceDamping: restorationReferenceDamping
                    }
                });
                jogMetricState = metricAttempt.state;
                jogMetricBefore = metricAttempt.before;
                jogMetricAfter = metricAttempt.after;
                jogMetricApplied = metricAttempt.applied === true;
                nullspaceResult = metricAttempt.nullspaceResult;
                nullspaceRefineResult = metricAttempt.refineResult;
                if (metricAttempt.applied === true && metricAttempt.refineResult) {
                    effectiveResult = metricAttempt.refineResult;
                }
            } else {
                jogMetricBefore = this._evaluateHumanoidJogManifoldMetricValue(
                    resolvedJogMetric.metric,
                    restoredProblem?.metadata?.cctsm,
                    jogMetricAxis
                );
                jogMetricAfter = jogMetricBefore;
                jogMetricState = 'skipped(none)';
            }
        } else if (effectiveRestorationResult.converged && restoredProblem && !mjEnableNullspaceRefine) {
            jogMetricBefore = this._evaluateHumanoidJogManifoldMetricValue(
                resolvedJogMetric.metric,
                restoredProblem?.metadata?.cctsm,
                jogMetricAxis
            );
            jogMetricAfter = jogMetricBefore;
            jogMetricState = resolvedJogMetric.shouldSkip
                ? 'skipped(none)'
                : 'skipped(disabled_for_mj_restore)';
        }

        const evaluateAppliedManifoldResult = (appliedResult) => {
            const qCandidateActive = Array.isArray(appliedResult?.qApplied) ? appliedResult.qApplied.slice() : qInitialActive.slice();
            const qCandidate = this._writeHumanoidActiveJoints(qInitialFull, qCandidateActive, armStart, activeCount);
            this._clampHumanoidActiveJoints(qCandidate, armStart, hardLimits);
            const finalProblem = appliedResult?.currentProblem || evaluateProblem(qCandidateActive);
            const finalAbsoluteResidual = finalProblem?.equalities?.blocks?.absolute?.residual ?? [];
            const finalRelativeResidual = finalProblem?.equalities?.blocks?.relative?.residual ?? [];
            const finalBodyResidual = keepBodyZVertical ? (finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []) : [];
            const finalCmdNorm = norm(finalAbsoluteResidual);
            const finalRelativeNorm = norm(finalRelativeResidual);
            const finalBodyNorm = norm(finalBodyResidual);
            const finalInvariantNorm = Math.hypot(finalRelativeNorm, finalBodyNorm);
            const finalInequalityViolation = Math.max(
                finalProblem?.inequalities?.wvfiFloor?.violation ?? 0,
                finalProblem?.inequalities?.waistXCorridor?.violation ?? 0
            );
            const restorationAccepted = this._isHumanoidMetricRestoreUsable(appliedResult);
            const commandTolerance = metricToleranceProfile.acceptanceCommandTolerance;
            const relativeTolerance = metricToleranceProfile.acceptanceRelativeTolerance;
            const acceptance = evaluateHumanoidJogManifoldAcceptance({
                restorationConverged: appliedResult?.converged === true,
                restorationAccepted,
                absoluteResidual: finalAbsoluteResidual,
                relativeResidual: finalRelativeResidual,
                bodyResidual: finalBodyResidual,
                inequalityViolation: finalInequalityViolation,
                commandTolerance,
                relativeTolerance,
                practicalCommandTolerance,
                practicalRelativeTolerance: practicalInvariantTolerance,
                bodyTolerance: bodyTol,
                inequalityTolerance: hardTol
            });
            return {
                qCandidateActive,
                qCandidate,
                finalProblem,
                finalAbsoluteResidual,
                finalRelativeResidual,
                finalBodyResidual,
                finalCmdNorm,
                finalRelativeNorm,
                finalBodyNorm,
                finalInvariantNorm,
                finalInequalityViolation,
                acceptance
            };
        };

        let evaluatedResult = evaluateAppliedManifoldResult(effectiveResult);
        let {
            qCandidateActive,
            qCandidate,
            finalProblem,
            finalAbsoluteResidual,
            finalRelativeResidual,
            finalBodyResidual,
            finalCmdNorm,
            finalRelativeNorm,
            finalBodyNorm,
            finalInvariantNorm,
            finalInequalityViolation,
            acceptance: manifoldAcceptance
        } = evaluatedResult;
        let converged = manifoldAcceptance.converged;

        const jogMetricFallback = resolveHumanoidJogManifoldJogMetricFallback({
            metric: resolvedJogMetric.metric,
            restorationAccepted: this._isHumanoidMetricRestoreUsable(effectiveRestorationResult),
            effectiveAccepted: converged,
            jogMetricState
        });
        if (jogMetricFallback.shouldFallback) {
            effectiveResult = effectiveRestorationResult;
            jogMetricState = jogMetricFallback.state;
            jogMetricApplied = false;
            evaluatedResult = evaluateAppliedManifoldResult(effectiveRestorationResult);
            ({
                qCandidateActive,
                qCandidate,
                finalProblem,
                finalAbsoluteResidual,
                finalRelativeResidual,
                finalBodyResidual,
                finalCmdNorm,
                finalRelativeNorm,
                finalBodyNorm,
                finalInvariantNorm,
                finalInequalityViolation,
                acceptance: manifoldAcceptance
            } = evaluatedResult);
            converged = manifoldAcceptance.converged;
        }

        if (!converged && allowArmOnlyFallback && !candidateOnly) {
            const initialBodyNorm = keepBodyZVertical
                ? norm(initialProblem?.equalities?.blocks?.bodyZ?.residual ?? [])
                : 0;
            const initialWaistCorridor = initialProblem?.inequalities?.waistXCorridor ?? null;
            const fallbackDecision = resolveHumanoidJogManifoldArmOnlyFallback({
                includeWaist,
                converged,
                keepBodyZVertical,
                currentBodyResidualNorm: finalBodyNorm,
                initialBodyResidualNorm: initialBodyNorm,
                bodyTolerance: bodyTol,
                waistXCorridorEnabled: initialWaistCorridor?.enabled === true,
                waistXPosition: initialWaistCorridor?.value,
                waistXMin: initialWaistCorridor?.min,
                waistXMax: initialWaistCorridor?.max,
                waistXActivationThreshold: Number.isFinite(cfg.waistXActivationThreshold)
                    ? cfg.waistXActivationThreshold
                    : 0.12,
                commandDelta: translationAxisGuard?.commandDelta ?? null
            });
            fallbackReason = fallbackDecision.reason;
            if (fallbackDecision.shouldFallback) {
                const fallbackSucceeded = await this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
                    dqAbsoluteTargetArray,
                    currentQ,
                    enableJointLimitAvoidance,
                    false,
                    fallbackDecision.keepBodyZVertical,
                    fixedRelativeTargetArray,
                    {
                        allowArmOnlyFallback: false,
                        suppressFailureNotification: true,
                        fallbackReason: fallbackDecision.reason,
                        solverMode,
                        requestToken,
                        preferWorker
                    }
                ) && this.lastIKSolveStatus?.solver === solverMode && this.lastIKSolveStatus?.converged === true;
                if (fallbackSucceeded) return true;
            }
        }

        if (isMJRestore && requestToken > 0 && requestToken !== this._humanoidMJWorkerRequestToken) {
            return false;
        }

        const qApplied = converged ? qCandidate : qInitialFull.slice();
        if (!converged && !suppressFailureNotification) {
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }

        const cctsmStartMs = isMJRestore ? nowMs() : Number.NaN;
        const vfiConstraintsForDiagnostics = this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits);
        const vfiDiagnostics = summarizeManipulabilityVFIConstraints(vfiConstraintsForDiagnostics);
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            vfiConstraintsForDiagnostics
        );
        const cctsmMs = isMJRestore ? (nowMs() - cctsmStartMs) : Number.NaN;
        const diagnosticBuildStartMs = isMJRestore ? nowMs() : Number.NaN;
        const diagnosticsPayload = isMJRestore
            ? buildHumanoidJogMJDiagnostics({
                modeTag: 'absolute-target',
                includeWaist,
                keepBodyZVertical,
                restorationResult,
                polishState: mjPolishState,
                polishReason: mjPolishResult?.reason ?? '',
                polishResidualNorm: mjPolishResult?.currentResidualNorm,
                nullspaceResult: computeHumanoidJogMJNullspaceRefine({
                    enabled: mjEnableNullspaceRefine,
                    restorationConverged: effectiveRestorationResult.converged === true,
                    metricAttempt: {
                        applied: jogMetricApplied,
                        state: jogMetricState,
                        before: jogMetricBefore,
                        after: jogMetricAfter,
                        axisLabel: jogMetricAxis,
                        nullspaceResult,
                        refineResult: nullspaceRefineResult,
                        finalProblem
                    }
                }),
                referenceModel: initialProblem?.referenceModel,
                jogMetric: jogMetricDiagnostic,
                jogMetricState,
                jogMetricBefore,
                jogMetricAfter,
                jogMetricApplied,
                jogMetricAxis,
                hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend(),
                relativeResidualNorm: finalRelativeNorm,
                bodyResidualNorm: finalBodyNorm,
                vfiJointLimitCount: vfiDiagnostics.jointLimitCount,
                vfiExtraCount: vfiDiagnostics.extraCount,
                vfiTotalCount: vfiDiagnostics.totalCount,
                vfiMinDistance: vfiDiagnostics.minDistance,
                ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
                ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
            })
            : buildHumanoidJogManifoldDiagnostics({
                modeTag: 'absolute-target',
                includeWaist,
                keepBodyZVertical,
                jointLimitAvoidance: enableJointLimitAvoidance,
                restorationResult: effectiveResult,
                nullspaceResult,
                optimizationMetric: '',
                restoredWVFI,
                refinedWVFI: nullspaceRefineResult?.currentProblem?.inequalities?.wvfiFloor?.value,
                wvfiOptimizationApplied: effectiveResult === nullspaceRefineResult,
                jogMetric: jogMetricDiagnostic,
                jogMetricState,
                jogMetricBefore,
                jogMetricAfter,
                jogMetricApplied,
                jogMetricAxis,
                fallbackReason,
                cctsmResult,
                relativeResidualNorm: finalRelativeNorm,
                bodyResidualNorm: finalBodyNorm,
                ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
                ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
            });
        const diagnosticLabel = [
            `restore=${restorationResult.reason}`,
            isMJRestore ? `polish=${mjPolishState}${mjPolishResult?.reason ? `(${mjPolishResult.reason})` : ''}` : null,
            `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
            nullspaceRefineResult ? `nullspace_restore=${nullspaceRefineResult.reason}` : null,
            `jog_metric=${jogMetricDiagnostic}`,
            `jog_metric_opt=${jogMetricState}`,
            diagnosticsPayload.fallbackReason ? `fallback=${diagnosticsPayload.fallbackReason}` : null,
            isMJRestore && diagnosticsPayload.referenceSummary ? diagnosticsPayload.referenceSummary : null,
            `rel=${this._formatDiagNumber(finalRelativeNorm)}`,
            keepBodyZVertical ? `body=${this._formatDiagNumber(finalBodyNorm)}` : null
        ].filter(Boolean).join('; ');
        const diagnosticMs = isMJRestore ? (nowMs() - diagnosticBuildStartMs) : Number.NaN;
        const totalMs = isMJRestore ? (nowMs() - mjTimingStartMs) : Number.NaN;
        if (isMJRestore) {
            diagnosticsPayload.timingPreMs = Number.isFinite(restorationResult?.timings?.preMs)
                ? restorationResult.timings.preMs
                : Number.NaN;
            diagnosticsPayload.timingWorkerInitMs = Number.isFinite(restorationResult?.timings?.workerInitMs)
                ? restorationResult.timings.workerInitMs
                : Number.NaN;
            diagnosticsPayload.timingSolveWaitMs = Number.isFinite(restorationResult?.timings?.solveWaitMs)
                ? restorationResult.timings.solveWaitMs
                : Number.NaN;
            diagnosticsPayload.timingSolveMs = Number.isFinite(restorationResult?.timings?.solveMs)
                ? restorationResult.timings.solveMs
                : Number.NaN;
            diagnosticsPayload.timingPostMs = Number.isFinite(restorationResult?.timings?.postMs)
                ? restorationResult.timings.postMs
                : Number.NaN;
            diagnosticsPayload.timingCCTSMMs = cctsmMs;
            diagnosticsPayload.timingDiagnosticMs = diagnosticMs;
            diagnosticsPayload.timingTotalMs = totalMs;
        }

        this._publishHumanoidJogDiagnostics({
            ...diagnosticsPayload,
            active: true,
            message: converged
                ? (manifoldAcceptance.reason === 'practical_converged'
                    ? `${solverDisplayName} 达到实用收敛阈值。`
                    : `${solverDisplayName} 单步更新完成。`)
                : '无法到达该位置（目标未达成）。',
            iterations: restorationResult.iterations + (nullspaceRefineResult?.iterations ?? 0),
            activeJoints: activeCount,
            commandError: finalCmdNorm,
            invariantError: finalInvariantNorm,
            stepNorm: converged ? this._computeHumanoidActiveJointDistance(qApplied, qInitialFull, armStart, activeCount) : 0,
            diagnosticLabel,
            cctsmW: cctsmResult?.w ?? Number.NaN,
            cctsmWvfi: cctsmResult?.w_vfi ?? Number.NaN,
            cctsmWAbs: cctsmResult?.w_abs ?? Number.NaN,
            cctsmWAbsHier: cctsmResult?.w_abs_hier ?? Number.NaN,
            cctsmWAbsHierVfi: cctsmResult?.w_abs_hier_vfi ?? Number.NaN,
            cctsmWAbsArm: cctsmResult?.w_abs_arm ?? Number.NaN,
            cctsmWAbsArmVfi: cctsmResult?.w_abs_arm_vfi ?? Number.NaN,
            cctsmWAbsVfi: cctsmResult?.w_abs_vfi ?? Number.NaN,
            cctsmWCmd: cctsmResult?.w_cmd ?? Number.NaN,
            vfiJointLimitCount: vfiDiagnostics.jointLimitCount,
            vfiExtraCount: vfiDiagnostics.extraCount,
            vfiTotalCount: vfiDiagnostics.totalCount,
            vfiMinDistance: vfiDiagnostics.minDistance
        });
        this.lastIKSolveStatus = {
            solver: solverMode,
            mode: 'humanoid_absolute',
            converged,
            reason: converged ? (manifoldAcceptance.reason || 'converged') : 'hard_tol_not_reached',
            message: converged
                ? (manifoldAcceptance.reason === 'practical_converged'
                    ? `${solverDisplayName} 达到实用收敛阈值。`
                    : `${solverDisplayName} 单步更新完成。`)
                : '无法到达该位置（目标未达成）。',
            finalResidual: finalCmdNorm,
            invariantResidual: finalInvariantNorm
        };
        if (candidateOnly) {
            return {
                state: jogMetricState,
                acceptedCandidate: converged === true && jogMetricApplied === true,
                qApplied: qApplied.slice(),
                currentProblem: finalProblem,
                converged,
                reason: this.lastIKSolveStatus.reason
            };
        }
        this._applyHumanoidJointSolution(qApplied);
        return true;
    }

    _updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
        dqAbsoluteTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        keepBodyZVertical = false,
        fixedRelativeTargetArray = null,
        translationAxisGuard = null,
        internalOptions = null
    ) {
        const forwardedJogMetricOverride = typeof internalOptions?.jogMetricOverride === 'string'
            ? internalOptions.jogMetricOverride
            : 'none';
        const requestedJogMetric = typeof internalOptions?.jogMetricOverride === 'string'
            ? normalizeHumanoidJogManifoldJogMetric(internalOptions.jogMetricOverride)
            : 'none';
        const resolvedJogMetric = resolveHumanoidJogManifoldJogMetric(requestedJogMetric, {
            hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend()
        });
        const jogMetricDiagnostic = resolvedJogMetric.diagnosticMetric || resolvedJogMetric.metric;
        const isLocalMetricOnlyRequest = (
            jogMetricDiagnostic === 'w_abs_vfi' &&
            resolvedJogMetric.shouldSkip !== true
        );
        const absCurrentArray = Array.isArray(currentQ) && currentQ.length > 0
            ? this._getHumanoidAbsolutePose(currentQ)
            : null;
        const relCurrentArray = Array.isArray(currentQ) && currentQ.length > 0
            ? this._getHumanoidRelativePose(currentQ)
            : null;
        const relConstraintArray = (
            Array.isArray(fixedRelativeTargetArray) &&
            fixedRelativeTargetArray.length === 8 &&
            fixedRelativeTargetArray.every((value) => Number.isFinite(Number(value)))
        )
            ? fixedRelativeTargetArray.map((value) => Number(value))
            : (Array.isArray(relCurrentArray) ? relCurrentArray.slice() : null);
        const absoluteTargetError = Array.isArray(dqAbsoluteTargetArray) && Array.isArray(absCurrentArray)
            ? this._computePoseErrorNorm(this._signedPoseError(absCurrentArray, dqAbsoluteTargetArray))
            : Number.POSITIVE_INFINITY;
        const relativeTargetError = Array.isArray(relConstraintArray) && Array.isArray(relCurrentArray)
            ? this._computePoseErrorNorm(this._signedPoseError(relCurrentArray, relConstraintArray))
            : Number.POSITIVE_INFINITY;
        const { waistCount, armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        const isMetricOnlyJogRequest = (
            resolvedJogMetric.shouldSkip !== true &&
            translationAxisGuard === null &&
            absoluteTargetError <= 1e-12 &&
            relativeTargetError <= 1e-12
        );
        const shouldBypassWorkerForRegularizedRestore = Array.isArray(buildHumanoidJogMJRegularizationWeights({
            activeCount,
            waistActiveCount: armStart === 0 ? Math.min(waistCount, activeCount) : 0,
            includeWaist,
            keepBodyZVertical,
            isMetricOnlyJogRequest,
            metricShouldSkip: resolvedJogMetric.shouldSkip === true,
            config: this.vfiConfig || {}
        }));
        const shouldBypassWorkerQueue = this._canUseHumanoidJogMJRestoreWorker() &&
            (
                (isLocalMetricOnlyRequest &&
                    translationAxisGuard === null &&
                    absoluteTargetError <= 1e-12 &&
                    relativeTargetError <= 1e-12) ||
                shouldBypassWorkerForRegularizedRestore
            );
        if (shouldBypassWorkerQueue) {
            return this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
                dqAbsoluteTargetArray,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray,
                {
                    solverMode: 'mj_restore',
                    translationAxisGuard,
                    preferWorker: false,
                    jogMetricOverride: forwardedJogMetricOverride,
                    metricDirection: internalOptions?.metricDirection
                }
            );
        }
        if (this._canUseHumanoidJogMJRestoreWorker()) {
            return this._enqueueHumanoidJogMJRestoreWorkerRequest({
                dqAbsoluteTargetArray: Array.isArray(dqAbsoluteTargetArray) ? dqAbsoluteTargetArray.slice() : [],
                currentQ: Array.isArray(currentQ) ? currentQ.slice() : [],
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray: Array.isArray(fixedRelativeTargetArray) ? fixedRelativeTargetArray.slice() : null,
                translationAxisGuard,
                jogMetricOverride: forwardedJogMetricOverride,
                metricDirection: internalOptions?.metricDirection
            });
        }
        return this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
            dqAbsoluteTargetArray,
            currentQ,
            enableJointLimitAvoidance,
            includeWaist,
            keepBodyZVertical,
            fixedRelativeTargetArray,
            {
                solverMode: 'mj_restore',
                translationAxisGuard,
                jogMetricOverride: forwardedJogMetricOverride,
                metricDirection: internalOptions?.metricDirection
            }
        );
    }

    _updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
        deltaDistance,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        internalOptions = null
    ) {
        if (!this.humanoidModel || !window.humanoidQ || !Array.isArray(currentQ) || currentQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_distance',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }
        const DQ = window.DQModule?.DQWrapper;
        const Kinematics = window.DQModule?.Kinematics;
        if (!DQ || !Kinematics?.distanceJacobian) {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_distance',
                reason: 'dq_unavailable',
                message: 'DQ 距离雅可比模块未加载。'
            });
        }

        const { waistCount, armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_distance',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relStartArray = this._getHumanoidRelativePose(currentQ);
        const pose1StartArray = this._getHumanoidPose1(currentQ);
        const pose2StartArray = this._getHumanoidPose2(currentQ);
        if (!absCurrentArray || !relStartArray || !pose1StartArray || !pose2StartArray) {
            return this._setIKFailureStatus({
                solver: 'mj_restore',
                mode: 'humanoid_distance',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const relRotTarget = relStartArray.slice(0, 4);
        const hand1RotTarget = pose1StartArray.slice(0, 4);
        const hand2RotTarget = pose2StartArray.slice(0, 4);
        const tStart = DQ.getTranslation(DQ.createFromArray(relStartArray));
        const dStart = Math.sqrt((tStart[1] || 0) ** 2 + (tStart[2] || 0) ** 2 + (tStart[3] || 0) ** 2);
        const dTarget = Math.max(0.01, dStart + deltaDistance);
        const d2Target = dTarget * dTarget;

        const cfg = this.vfiConfig || {};
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const manipActiveLimits = enableJointLimitAvoidance ? hardLimits : null;
        const qInitialFull = currentQ.slice();
        const qInitialActive = this._sliceHumanoidActiveJoints(qInitialFull, armStart, activeCount);
        const mjRestoreProfile = resolveHumanoidJogMJRestoreProfile({
            includeWaist,
            fallbackReason: null,
            config: cfg
        });
        const maxStep = Number.isFinite(cfg.mjRestoreMaxStep)
            ? cfg.mjRestoreMaxStep
            : (Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.05);
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical: false,
            waistCount,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolDistanceSq) ? cfg.hardCommandTolDistanceSq : 1e-8;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const inequalityTol = Number.isFinite(cfg.mjRestoreHardTolerance) ? Math.max(0, cfg.mjRestoreHardTolerance) : 1e-8;
        const equalityTol = Math.min(hardCmdTol, hardInvariantTol);
        const damping = mjRestoreProfile.damping;
        const referenceStiffness = mjRestoreProfile.referenceStiffness;
        const referenceDamping = mjRestoreProfile.referenceDamping;
        const minScale = Number.isFinite(cfg.mjRestoreLineSearchMinScale) ? Math.max(1e-8, cfg.mjRestoreLineSearchMinScale) : 1e-6;
        const reductionFactor = Number.isFinite(cfg.mjRestoreLineSearchReductionFactor) ? cfg.mjRestoreLineSearchReductionFactor : 0.5;
        const restoreMaxIterations = mjRestoreProfile.maxIterations;
        const polishActivationResidual = mjRestoreProfile.polishActivationResidual;
        const polishMaxIterations = mjRestoreProfile.polishMaxIterations;
        const polishDamping = mjRestoreProfile.polishDamping;
        const distanceKeepRelativeRotation = (typeof cfg.distanceKeepRelativeRotation === 'boolean')
            ? cfg.distanceKeepRelativeRotation
            : true;
        const distanceKeepHandOrientation = (typeof cfg.distanceKeepHandOrientation === 'boolean')
            ? cfg.distanceKeepHandOrientation
            : true;
        const norm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => {
            const numericValue = Number(value);
            return Number.isFinite(numericValue) ? (sum + numericValue * numericValue) : sum;
        }, 0));
        const nowMs = () => (typeof performance !== 'undefined' && typeof performance.now === 'function')
            ? performance.now()
            : Date.now();
        const mjTimingStartMs = nowMs();

        const requestedJogMetric = typeof internalOptions?.jogMetricOverride === 'string'
            ? normalizeHumanoidJogManifoldJogMetric(internalOptions.jogMetricOverride)
            : 'none';
        const resolvedJogMetric = resolveHumanoidJogManifoldJogMetric(requestedJogMetric, {
            hierarchicalVfiBlend: this._getHumanoidHierarchicalVfiBlend()
        });
        const jogMetricDiagnostic = resolvedJogMetric.diagnosticMetric || resolvedJogMetric.metric;
        const shouldComputeManipData = shouldComputeHumanoidJogMJTrialManipData({
            metric: resolvedJogMetric.metric,
            nullspaceRefineEnabled: false,
            wvfiLowerBound: 0
        });

        const extractActiveColumns = (rows) => (Array.isArray(rows) ? rows : []).map((row) => {
            const out = new Array(activeCount).fill(0);
            if (!Array.isArray(row)) return out;
            for (let i = 0; i < activeCount; i++) out[i] = row[armStart + i] || 0;
            return out;
        });

        const evaluateProblem = (qActiveEval, options = {}) => {
            const qFull = this._writeHumanoidActiveJoints(qInitialFull, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const Jd2Full = Kinematics.distanceJacobian(JrelFull, relPose);
            const Jd2Row = Array.isArray(Jd2Full) && Array.isArray(Jd2Full[0]) ? Jd2Full[0] : null;
            if (!Jd2Row) return null;

            const absResidual = this._signedPoseError(absCurrentArray, absPose);
            const absJacobian = extractActiveColumns(JabsFull);

            const relativeResidual = [];
            const relativeJacobian = [];
            if (distanceKeepRelativeRotation) {
                const relRotNow = relPose.slice(0, 4);
                relativeResidual.push(...this._signedPoseError(relRotTarget, relRotNow));
                relativeJacobian.push(...extractActiveColumns(JrelFull.slice(0, 4)));
            }
            if (distanceKeepHandOrientation) {
                const pose1Now = this._getHumanoidPose1(qFull);
                const pose2Now = this._getHumanoidPose2(qFull);
                const Jpose1Full = this._getHumanoidSingleArmPoseJacobian(qFull, 1) || this._getHumanoidPose1Jacobian(qFull);
                const Jpose2Full = this._getHumanoidSingleArmPoseJacobian(qFull, 2) || this._getHumanoidPose2Jacobian(qFull);
                if (!pose1Now || !pose2Now || !Jpose1Full || !Jpose2Full) return null;
                relativeResidual.push(...this._signedPoseError(hand1RotTarget, pose1Now.slice(0, 4)));
                relativeResidual.push(...this._signedPoseError(hand2RotTarget, pose2Now.slice(0, 4)));
                relativeJacobian.push(...extractActiveColumns(Jpose1Full.slice(0, 4)));
                relativeJacobian.push(...extractActiveColumns(Jpose2Full.slice(0, 4)));
            }

            const tNow = DQ.getTranslation(DQ.createFromArray(relPose));
            const d2Now = (tNow[1] || 0) ** 2 + (tNow[2] || 0) ** 2 + (tNow[3] || 0) ** 2;
            const commandJacobian = [extractActiveColumns([Jd2Row])[0]];

            const computeManipData = typeof options.computeManipData === 'boolean'
                ? options.computeManipData
                : shouldComputeManipData;
            const vfiConstraints = computeManipData
                ? this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, manipActiveLimits)
                : null;
            const cctsm = computeManipData
                ? this._computeModeCCTSM(qFull, armStart, activeCount, 'distance', vfiConstraints)
                : null;
            const wvfiGradient = computeManipData
                ? this._computeProtectedManipulabilityGradient({
                    q: qFull,
                    armStart,
                    activeCount,
                    mode: 'distance',
                    Jcmd: commandJacobian,
                    Jinvariant: [...absJacobian, ...relativeJacobian],
                    activeLimits: manipActiveLimits,
                    forceInvariantEquality: true
                })
                : null;

            const problem = buildHumanoidJogMJDistanceProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                distanceTarget: d2Target,
                currentDistance: d2Now,
                distanceJacobian: () => commandJacobian,
                absoluteTarget: absCurrentArray,
                absolutePose: () => absPose,
                absoluteJacobian: () => absJacobian,
                relativeRotationTarget: [
                    ...(distanceKeepRelativeRotation ? relRotTarget : []),
                    ...(distanceKeepHandOrientation ? hand1RotTarget : []),
                    ...(distanceKeepHandOrientation ? hand2RotTarget : [])
                ],
                relativeRotation: () => [
                    ...(distanceKeepRelativeRotation ? relPose.slice(0, 4) : []),
                    ...(distanceKeepHandOrientation ? this._getHumanoidPose1(qFull)?.slice(0, 4) ?? [] : []),
                    ...(distanceKeepHandOrientation ? this._getHumanoidPose2(qFull)?.slice(0, 4) ?? [] : [])
                ],
                relativeRotationJacobian: () => relativeJacobian,
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                wvfiState: computeManipData
                    ? () => ({
                        value: cctsm?.w_vfi ?? Number.NaN,
                        gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                    })
                    : null,
                wvfiLowerBound: 0,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull,
                    armStart,
                    activeCount,
                    includeWaist
                }),
                referenceModel: {
                    absolute: {
                        stiffness: referenceStiffness,
                        damping: referenceDamping
                    },
                    relative: {
                        stiffness: referenceStiffness,
                        damping: referenceDamping
                    },
                    bodyZ: {
                        stiffness: 0,
                        damping: 0
                    }
                }
            });
            problem.metadata = {
                qFull,
                cctsm,
                Jcmd: commandJacobian,
                protectedRows: [...absJacobian, ...relativeJacobian]
            };
            return problem;
        };

        const restorationResult = restoreHumanoidJogMJ({
            qInitial: qInitialActive,
            evaluateProblem,
            maxIterations: restoreMaxIterations,
            damping,
            referenceStiffness,
            referenceDamping,
            initialScale: 1,
            minScale,
            reductionFactor,
            equalityTolerance: equalityTol,
            inequalityTolerance: inequalityTol
        });

        let effectiveRestorationResult = restorationResult;
        let mjPolishResult = null;
        let mjPolishState = 'skipped';
        if (shouldAttemptHumanoidJogMJPolish({
            restorationResult,
            activationResidual: polishActivationResidual
        })) {
            mjPolishResult = restoreHumanoidJogManifold({
                qInitial: restorationResult.qApplied,
                evaluateProblem,
                maxIterations: polishMaxIterations,
                damping: polishDamping,
                referenceStiffness: 1,
                referenceDamping: 0,
                initialScale: 1,
                minScale,
                reductionFactor,
                equalityTolerance: equalityTol,
                inequalityTolerance: inequalityTol
            });
            const polishImproved = shouldApplyHumanoidJogMJPolish({
                restorationResult,
                polishResult: mjPolishResult
            });
            mjPolishState = mjPolishResult?.converged === true
                ? 'converged'
                : (polishImproved ? 'best_effort' : 'not_improved');
            if (polishImproved) {
                effectiveRestorationResult = {
                    ...mjPolishResult,
                    reason: mjPolishResult.converged === true ? 'polish_converged' : 'polish_best_effort',
                    iterations: restorationResult.iterations + (Number.isFinite(mjPolishResult.iterations) ? mjPolishResult.iterations : 0),
                    acceptedIterations: restorationResult.acceptedIterations + (Number.isFinite(mjPolishResult.acceptedIterations) ? mjPolishResult.acceptedIterations : 0)
                };
            }
        }

        let effectiveResult = effectiveRestorationResult;
        let nullspaceResult = null;
        let nullspaceRefineResult = null;
        const restoredProblem = effectiveRestorationResult.currentProblem || evaluateProblem(effectiveRestorationResult.qApplied);
        const jogMetricState = effectiveRestorationResult.converged === true
            ? (resolvedJogMetric.shouldSkip ? 'skipped(none)' : 'skipped(disabled_for_distance_target)')
            : 'skipped(waiting_for_restore)';
        const jogMetricBefore = Number.NaN;
        const jogMetricAfter = Number.NaN;
        const jogMetricApplied = false;
        const jogMetricAxis = resolvedJogMetric.family === 'directional' ? (this.lastHumanoidJogAxis || '') : '';

        return this._finalizeHumanoidMJDistanceResult({
            qInitialFull,
            armStart,
            activeCount,
            includeWaist,
            enableJointLimitAvoidance,
            hardLimits,
            manipActiveLimits,
            evaluateProblem,
            effectiveResult,
            restorationResult,
            mjPolishState,
            mjPolishResult,
            nullspaceResult,
            nullspaceRefineResult,
            resolvedJogMetric,
            jogMetricState,
            jogMetricBefore,
            jogMetricAfter,
            jogMetricApplied,
            jogMetricAxis,
            referenceModel: restoredProblem?.referenceModel ?? null,
            hardCmdTol,
            hardInvariantTol,
            inequalityTol,
            mjTimingStartMs
        });
    }

    _finalizeHumanoidMJDistanceResult({
        qInitialFull,
        armStart,
        activeCount,
        includeWaist,
        enableJointLimitAvoidance,
        hardLimits,
        manipActiveLimits,
        evaluateProblem,
        effectiveResult,
        restorationResult,
        mjPolishState,
        mjPolishResult,
        nullspaceResult,
        nullspaceRefineResult,
        resolvedJogMetric,
        jogMetricState,
        jogMetricBefore,
        jogMetricAfter,
        jogMetricApplied,
        jogMetricAxis,
        referenceModel,
        hardCmdTol,
        hardInvariantTol,
        inequalityTol,
        mjTimingStartMs
    }) {
        const nowMs = () => (typeof performance !== 'undefined' && typeof performance.now === 'function')
            ? performance.now()
            : Date.now();
        const jogMetricDiagnostic = resolvedJogMetric?.diagnosticMetric || resolvedJogMetric?.metric || 'none';
        const norm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => {
            const numericValue = Number(value);
            return Number.isFinite(numericValue) ? (sum + numericValue * numericValue) : sum;
        }, 0));

        const qCandidateActive = Array.isArray(effectiveResult?.qApplied) ? effectiveResult.qApplied.slice() : [];
        const qCandidate = this._writeHumanoidActiveJoints(qInitialFull, qCandidateActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qCandidate, armStart, hardLimits);
        const finalProblem = effectiveResult?.currentProblem || evaluateProblem(qCandidateActive, { computeManipData: true });
        const finalCommandResidual = finalProblem?.equalities?.blocks?.command?.residual ?? [];
        const finalAbsoluteResidual = finalProblem?.equalities?.blocks?.absolute?.residual ?? [];
        const finalRelativeResidual = finalProblem?.equalities?.blocks?.relative?.residual ?? [];
        const finalCmdNorm = norm(finalCommandResidual);
        const finalInvariantNorm = Math.hypot(norm(finalAbsoluteResidual), norm(finalRelativeResidual));
        const finalInequalityViolation = Math.max(
            finalProblem?.inequalities?.wvfiFloor?.violation ?? 0,
            finalProblem?.inequalities?.waistXCorridor?.violation ?? 0
        );
        const converged = effectiveResult?.converged === true &&
            finalCmdNorm <= hardCmdTol &&
            finalInvariantNorm <= hardInvariantTol &&
            finalInequalityViolation <= inequalityTol;
        const qApplied = converged ? qCandidate : qInitialFull.slice();
        if (!converged) {
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }

        const cctsmStartMs = nowMs();
        const vfiConstraintsForDiagnostics = this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, manipActiveLimits);
        const vfiDiagnostics = summarizeManipulabilityVFIConstraints(vfiConstraintsForDiagnostics);
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'distance',
            vfiConstraintsForDiagnostics
        );
        const cctsmMs = nowMs() - cctsmStartMs;
        const diagnosticBuildStartMs = nowMs();
        const diagnosticsPayload = buildHumanoidJogMJDiagnostics({
            modeTag: 'distance-target',
            includeWaist,
            keepBodyZVertical: false,
            restorationResult,
            polishState: mjPolishState,
            polishReason: mjPolishResult?.reason ?? '',
            polishResidualNorm: mjPolishResult?.currentResidualNorm,
            nullspaceResult: computeHumanoidJogMJNullspaceRefine({
                enabled: true,
                restorationConverged: effectiveResult?.converged === true,
                metricAttempt: {
                    applied: jogMetricApplied,
                    state: jogMetricState,
                    before: jogMetricBefore,
                    after: jogMetricAfter,
                    axisLabel: jogMetricAxis,
                    nullspaceResult,
                    refineResult: nullspaceRefineResult,
                    finalProblem
                }
            }),
            referenceModel,
            jogMetric: jogMetricDiagnostic,
            jogMetricState,
            jogMetricBefore,
            jogMetricAfter,
            jogMetricApplied,
            jogMetricAxis,
            relativeResidualNorm: finalInvariantNorm,
            bodyResidualNorm: Number.NaN,
            vfiJointLimitCount: vfiDiagnostics.jointLimitCount,
            vfiExtraCount: vfiDiagnostics.extraCount,
            vfiTotalCount: vfiDiagnostics.totalCount,
            vfiMinDistance: vfiDiagnostics.minDistance,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        const diagnosticLabel = [
            `restore=${restorationResult.reason}`,
            `polish=${mjPolishState}${mjPolishResult?.reason ? `(${mjPolishResult.reason})` : ''}`,
            `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
            nullspaceRefineResult ? `nullspace_restore=${nullspaceRefineResult.reason}` : null,
            `jog_metric=${jogMetricDiagnostic}`,
            `jog_metric_opt=${jogMetricState}`,
            diagnosticsPayload.referenceSummary ? diagnosticsPayload.referenceSummary : null,
            `inv=${this._formatDiagNumber(finalInvariantNorm)}`
        ].filter(Boolean).join('; ');
        const diagnosticMs = nowMs() - diagnosticBuildStartMs;
        const totalMs = nowMs() - mjTimingStartMs;
        diagnosticsPayload.timingPreMs = Number.isFinite(restorationResult?.timings?.preMs) ? restorationResult.timings.preMs : Number.NaN;
        diagnosticsPayload.timingWorkerInitMs = Number.isFinite(restorationResult?.timings?.workerInitMs) ? restorationResult.timings.workerInitMs : Number.NaN;
        diagnosticsPayload.timingSolveWaitMs = Number.isFinite(restorationResult?.timings?.solveWaitMs) ? restorationResult.timings.solveWaitMs : Number.NaN;
        diagnosticsPayload.timingSolveMs = Number.isFinite(restorationResult?.timings?.solveMs) ? restorationResult.timings.solveMs : Number.NaN;
        diagnosticsPayload.timingPostMs = Number.isFinite(restorationResult?.timings?.postMs) ? restorationResult.timings.postMs : Number.NaN;
        diagnosticsPayload.timingCCTSMMs = cctsmMs;
        diagnosticsPayload.timingDiagnosticMs = diagnosticMs;
        diagnosticsPayload.timingTotalMs = totalMs;

        this._publishHumanoidJogDiagnostics({
            ...diagnosticsPayload,
            active: true,
            message: converged ? 'MuJoCo 风格恢复 单步更新完成。' : '无法到达该位置（目标未达成）。',
            iterations: restorationResult.iterations + (nullspaceRefineResult?.iterations ?? 0),
            activeJoints: activeCount,
            commandError: finalCmdNorm,
            invariantError: finalInvariantNorm,
            stepNorm: converged ? this._computeHumanoidActiveJointDistance(qApplied, qInitialFull, armStart, activeCount) : 0,
            diagnosticLabel,
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance,
            cctsmW: cctsmResult?.w ?? Number.NaN,
            cctsmWvfi: cctsmResult?.w_vfi ?? Number.NaN,
            cctsmWAbs: cctsmResult?.w_abs ?? Number.NaN,
            cctsmWAbsHier: cctsmResult?.w_abs_hier ?? Number.NaN,
            cctsmWAbsHierVfi: cctsmResult?.w_abs_hier_vfi ?? Number.NaN,
            cctsmWAbsArm: cctsmResult?.w_abs_arm ?? Number.NaN,
            cctsmWAbsArmVfi: cctsmResult?.w_abs_arm_vfi ?? Number.NaN,
            cctsmWAbsVfi: cctsmResult?.w_abs_vfi ?? Number.NaN,
            cctsmWCmd: cctsmResult?.w_cmd ?? Number.NaN,
            vfiJointLimitCount: vfiDiagnostics.jointLimitCount,
            vfiExtraCount: vfiDiagnostics.extraCount,
            vfiTotalCount: vfiDiagnostics.totalCount,
            vfiMinDistance: vfiDiagnostics.minDistance
        });
        this.lastIKSolveStatus = {
            solver: 'mj_restore',
            mode: 'humanoid_distance',
            converged,
            reason: converged ? 'converged' : 'hard_tol_not_reached',
            message: converged ? 'MuJoCo 风格恢复 单步更新完成。' : '无法到达该位置（目标未达成）。',
            finalResidual: finalCmdNorm,
            invariantResidual: finalInvariantNorm
        };
        this._applyHumanoidJointSolution(qApplied);
        return true;
    }

    _optimizeHumanoidWVFIManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_wvfi_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_wvfi_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const currentQ = window.humanoidQ.slice();
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1;
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount: includeWaist ? this.humanoidWaistLength : 0,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const qInitialActive = this._sliceHumanoidActiveJoints(currentQ, armStart, activeCount);
        const relConstraintArray = this._getHumanoidRelativePose(currentQ);
        const absConstraintArray = this._getHumanoidAbsolutePose(currentQ);
        if (!relConstraintArray || !absConstraintArray) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_wvfi_only',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const evaluateProblem = (qActiveEval) => {
            const qFull = this._writeHumanoidActiveJoints(currentQ, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const cctsm = this._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
            );
            const wvfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true
            });

            const problem = buildHumanoidJogManifoldProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: absConstraintArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0,
                bodyWeight: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0)
                    : 1.0,
                wvfiState: () => ({
                    value: cctsm?.w_vfi ?? Number.NaN,
                    gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                }),
                wvfiLowerBound: this._getWVFILowerBound(includeWaist),
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull: currentQ,
                    armStart,
                    activeCount,
                    includeWaist
                })
            });
            problem.metadata = { qFull, cctsm };
            return problem;
        };

        const restoredProblem = evaluateProblem(qInitialActive);
        if (!restoredProblem) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_wvfi_only',
                reason: 'problem_eval_failed',
                message: '约束流形 w_vfi-only 问题构建失败。'
            });
        }

        const buttonStepSize = Number.isFinite(cfg.manifoldWVFIButtonStepSize)
            ? cfg.manifoldWVFIButtonStepSize
            : 0.01;
        const signedStep = Math.abs(buttonStepSize)
            * (direction === 'decrease' ? -1 : 1);
        const nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: qInitialActive,
            equalityJacobian: restoredProblem?.equalities?.jacobian ?? [],
            wvfiGradient: restoredProblem?.inequalities?.wvfiFloor?.gradient ?? new Array(activeCount).fill(0),
            stepSize: signedStep,
            lowerBounds,
            upperBounds,
            stepBounds: jointMaxSteps,
            evaluateWVFI: (qNextActive) => {
                const qFull = this._writeHumanoidActiveJoints(currentQ, qNextActive, armStart, activeCount);
                const cctsm = this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                    { force: true }
                );
                return cctsm?.w_vfi ?? Number.NaN;
            },
            wvfiMin: this._getWVFILowerBound(includeWaist)
        });

        let effectiveActive = qInitialActive.slice();
        let refineResult = null;
        if (nullspaceResult.accepted === true) {
            refineResult = restoreHumanoidJogManifold({
                qInitial: nullspaceResult.qNext,
                evaluateProblem,
                maxIterations: Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter)
                    ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter))
                    : 24,
                damping: Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4,
                referenceStiffness: Number.isFinite(cfg.manifoldReferenceStiffness)
                    ? Math.max(0, cfg.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(cfg.manifoldReferenceDamping)
                    ? Math.max(0, cfg.manifoldReferenceDamping)
                    : 0.35,
                initialScale: 1,
                minScale: Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6,
                reductionFactor: Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5,
                equalityTolerance: 1e-12,
                inequalityTolerance: 1e-12
            });
            if (this._isHumanoidMetricRestoreUsable(refineResult)) {
                effectiveActive = refineResult.qApplied.slice();
            }
        }

        const qApplied = this._writeHumanoidActiveJoints(currentQ, effectiveActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qApplied, armStart, hardLimits);
        const finalProblem = evaluateProblem(effectiveActive);
        const relNorm = Math.sqrt((finalProblem?.equalities?.blocks?.relative?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const bodyNorm = Math.sqrt((finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits)
        );
        const { direction: directionTolerance, minImprovement } = this._getHumanoidMetricDirectionTolerance();
        const wvfiDirection = this._evaluateHumanoidJogMetricDirectionWithFallback({
            before: restoredProblem?.inequalities?.wvfiFloor?.value,
            after: finalProblem?.inequalities?.wvfiFloor?.value,
            direction,
            tolerance: directionTolerance,
            minImprovement
        });
        const restoreUsable = this._isHumanoidMetricRestoreUsable(refineResult);
        const wvfiApplied = nullspaceResult.accepted === true && restoreUsable && wvfiDirection.applied;
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'wvfi-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: refineResult?.converged === true,
                iterations: refineResult?.iterations ?? 0,
                acceptedIterations: refineResult?.acceptedIterations ?? 0,
                currentResidualNorm: refineResult?.currentResidualNorm ?? Number.NaN,
                lastAcceptedScale: refineResult?.lastAcceptedScale ?? Number.NaN,
                reason: refineResult?.reason ?? 'skipped'
            },
            nullspaceResult,
            restoredWVFI: restoredProblem?.inequalities?.wvfiFloor?.value,
            refinedWVFI: finalProblem?.inequalities?.wvfiFloor?.value,
            wvfiOptimizationApplied: wvfiApplied,
            cctsmResult,
            relativeResidualNorm: relNorm,
            bodyResidualNorm: bodyNorm,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: wvfiApplied
                ? '约束流形 w_vfi 单步更新完成。'
                : '无法到达该位置（w_vfi 优化未通过硬约束）。',
            iterations: (refineResult?.iterations ?? 0) + (nullspaceResult?.accepted ? 1 : 0),
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(relNorm, bodyNorm),
            stepNorm: wvfiApplied
                ? this._computeHumanoidActiveJointDistance(qApplied, currentQ, armStart, activeCount)
                : 0,
            diagnosticLabel: [
                `restore=${refineResult?.reason ?? 'skipped'}`,
                `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
                `metric=${wvfiDirection.reason}`,
                `rel=${this._formatDiagNumber(relNorm)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(bodyNorm)}` : null
            ].filter(Boolean).join('; ')
        });
        if (wvfiApplied) {
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_wvfi_only',
                converged: true,
                reason: 'wvfi_only_converged',
                message: '约束流形 w_vfi 单步更新完成。',
                finalResidual: 0,
                invariantResidual: Math.hypot(relNorm, bodyNorm)
            };
            this._applyHumanoidJointSolution(qApplied);
            return true;
        }

        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_wvfi_only',
            converged: false,
            reason: 'wvfi_only_failed',
            message: '无法到达该位置（w_vfi 优化未通过硬约束）。',
            finalResidual: 0,
            invariantResidual: Math.hypot(relNorm, bodyNorm)
        };
        return true;
    }

    _optimizeHumanoidWManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const currentQ = window.humanoidQ.slice();
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1;
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount: includeWaist ? this.humanoidWaistLength : 0,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const qInitialActive = this._sliceHumanoidActiveJoints(currentQ, armStart, activeCount);
        const relConstraintArray = this._getHumanoidRelativePose(currentQ);
        const absConstraintArray = this._getHumanoidAbsolutePose(currentQ);
        if (!relConstraintArray || !absConstraintArray) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_only',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const evaluateProblem = (qActiveEval) => {
            const qFull = this._writeHumanoidActiveJoints(currentQ, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const cctsm = this._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
            );
            const wvfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'wvfi'
            });
            const wGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'w'
            });

            const problem = buildHumanoidJogManifoldProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: absConstraintArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0,
                bodyWeight: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0)
                    : 1.0,
                wvfiState: () => ({
                    value: cctsm?.w_vfi ?? Number.NaN,
                    gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                }),
                wvfiLowerBound: this._getWVFILowerBound(includeWaist),
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull: currentQ,
                    armStart,
                    activeCount,
                    includeWaist
                })
            });
            problem.metadata = {
                qFull,
                cctsm,
                wGradient: Array.isArray(wGradient) ? wGradient.slice() : new Array(activeCount).fill(0)
            };
            return problem;
        };

        const restoredProblem = evaluateProblem(qInitialActive);
        if (!restoredProblem) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_only',
                reason: 'problem_eval_failed',
                message: '约束流形 w-only 问题构建失败。'
            });
        }

        const buttonStepSize = Number.isFinite(cfg.manifoldWVFIButtonStepSize)
            ? cfg.manifoldWVFIButtonStepSize
            : 0.01;
        const signedStep = Math.abs(buttonStepSize)
            * (direction === 'decrease' ? -1 : 1);
        const nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: qInitialActive,
            equalityJacobian: restoredProblem?.equalities?.jacobian ?? [],
            objectiveGradient: restoredProblem?.metadata?.wGradient ?? new Array(activeCount).fill(0),
            wvfiGradient: restoredProblem?.inequalities?.wvfiFloor?.gradient ?? new Array(activeCount).fill(0),
            stepSize: signedStep,
            lowerBounds,
            upperBounds,
            stepBounds: jointMaxSteps,
            evaluateWVFI: (qNextActive) => {
                const qFull = this._writeHumanoidActiveJoints(currentQ, qNextActive, armStart, activeCount);
                const cctsm = this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                    { force: true }
                );
                return cctsm?.w_vfi ?? Number.NaN;
            },
            wvfiMin: this._getWVFILowerBound(includeWaist)
        });

        let effectiveActive = qInitialActive.slice();
        let refineResult = null;
        if (nullspaceResult.accepted === true) {
            refineResult = restoreHumanoidJogManifold({
                qInitial: nullspaceResult.qNext,
                evaluateProblem,
                maxIterations: Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter)
                    ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter))
                    : 24,
                damping: Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4,
                referenceStiffness: Number.isFinite(cfg.manifoldReferenceStiffness)
                    ? Math.max(0, cfg.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(cfg.manifoldReferenceDamping)
                    ? Math.max(0, cfg.manifoldReferenceDamping)
                    : 0.35,
                initialScale: 1,
                minScale: Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6,
                reductionFactor: Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5,
                equalityTolerance: 1e-12,
                inequalityTolerance: 1e-12
            });
            if (this._isHumanoidMetricRestoreUsable(refineResult)) {
                effectiveActive = refineResult.qApplied.slice();
            }
        }

        const qApplied = this._writeHumanoidActiveJoints(currentQ, effectiveActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qApplied, armStart, hardLimits);
        const finalProblem = evaluateProblem(effectiveActive);
        const relNorm = Math.sqrt((finalProblem?.equalities?.blocks?.relative?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const bodyNorm = Math.sqrt((finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits)
        );
        const { direction: directionTolerance, minImprovement } = this._getHumanoidMetricDirectionTolerance();
        const wDirection = this._evaluateHumanoidJogMetricDirectionWithFallback({
            before: evaluateHumanoidJogProjectedManipulability(restoredProblem?.metadata?.cctsm),
            after: evaluateHumanoidJogProjectedManipulability(finalProblem?.metadata?.cctsm),
            direction,
            tolerance: directionTolerance,
            minImprovement
        });
        const restoreUsable = this._isHumanoidMetricRestoreUsable(refineResult);
        const wApplied = nullspaceResult.accepted === true && restoreUsable && wDirection.applied === true;
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'w-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: refineResult?.converged === true,
                iterations: refineResult?.iterations ?? 0,
                acceptedIterations: refineResult?.acceptedIterations ?? 0,
                currentResidualNorm: refineResult?.currentResidualNorm ?? Number.NaN,
                lastAcceptedScale: refineResult?.lastAcceptedScale ?? Number.NaN,
                reason: refineResult?.reason ?? 'skipped'
            },
            nullspaceResult,
            optimizationMetric: 'w_cmd',
            restoredWVFI: evaluateHumanoidJogProjectedManipulability(restoredProblem?.metadata?.cctsm),
            refinedWVFI: evaluateHumanoidJogProjectedManipulability(finalProblem?.metadata?.cctsm),
            wvfiOptimizationApplied: wApplied,
            cctsmResult,
            relativeResidualNorm: relNorm,
            bodyResidualNorm: bodyNorm,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: wApplied
                ? '约束流形 w 单步更新完成。'
                : '无法到达该位置（w 优化未通过硬约束）。',
            iterations: (refineResult?.iterations ?? 0) + (nullspaceResult?.accepted ? 1 : 0),
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(relNorm, bodyNorm),
            stepNorm: wApplied
                ? this._computeHumanoidActiveJointDistance(qApplied, currentQ, armStart, activeCount)
                : 0,
            diagnosticLabel: [
                `restore=${refineResult?.reason ?? 'skipped'}`,
                `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
                `metric=${wDirection.reason}`,
                `rel=${this._formatDiagNumber(relNorm)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(bodyNorm)}` : null
            ].filter(Boolean).join('; ')
        });
        if (wApplied) {
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_w_only',
                converged: true,
                reason: 'w_only_converged',
                message: '约束流形 w 单步更新完成。',
                finalResidual: 0,
                invariantResidual: Math.hypot(relNorm, bodyNorm)
            };
            this._applyHumanoidJointSolution(qApplied);
            return true;
        }

        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_w_only',
            converged: false,
            reason: 'w_only_failed',
            message: '无法到达该位置（w 优化未通过硬约束）。',
            finalResidual: 0,
            invariantResidual: Math.hypot(relNorm, bodyNorm)
        };
        return true;
    }

    _optimizeHumanoidAbsoluteManipulabilityManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const currentQ = window.humanoidQ.slice();
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1;
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount: includeWaist ? this.humanoidWaistLength : 0,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const qInitialActive = this._sliceHumanoidActiveJoints(currentQ, armStart, activeCount);
        const relConstraintArray = this._getHumanoidRelativePose(currentQ);
        const absConstraintArray = this._getHumanoidAbsolutePose(currentQ);
        if (!relConstraintArray || !absConstraintArray) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_only',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const evaluateProblem = (qActiveEval) => {
            const qFull = this._writeHumanoidActiveJoints(currentQ, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const cctsm = this._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
            );
            const wvfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'wvfi'
            });
            const wAbsGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'w_abs'
            });

            const problem = buildHumanoidJogManifoldProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: absConstraintArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0,
                bodyWeight: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0)
                    : 1.0,
                wvfiState: () => ({
                    value: cctsm?.w_vfi ?? Number.NaN,
                    gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                }),
                wvfiLowerBound: this._getWVFILowerBound(includeWaist),
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull: currentQ,
                    armStart,
                    activeCount,
                    includeWaist
                })
            });
            problem.metadata = {
                qFull,
                cctsm,
                wAbsGradient: Array.isArray(wAbsGradient) ? wAbsGradient.slice() : new Array(activeCount).fill(0)
            };
            return problem;
        };

        const restoredProblem = evaluateProblem(qInitialActive);
        if (!restoredProblem) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_only',
                reason: 'problem_eval_failed',
                message: '约束流形 w_abs-only 问题构建失败。'
            });
        }

        const buttonStepSize = Number.isFinite(cfg.manifoldWVFIButtonStepSize)
            ? cfg.manifoldWVFIButtonStepSize
            : 0.01;
        const signedStep = Math.abs(buttonStepSize)
            * (direction === 'decrease' ? -1 : 1);
        const nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: qInitialActive,
            equalityJacobian: restoredProblem?.equalities?.jacobian ?? [],
            objectiveGradient: restoredProblem?.metadata?.wAbsGradient ?? new Array(activeCount).fill(0),
            wvfiGradient: restoredProblem?.inequalities?.wvfiFloor?.gradient ?? new Array(activeCount).fill(0),
            stepSize: signedStep,
            lowerBounds,
            upperBounds,
            stepBounds: jointMaxSteps,
            evaluateWVFI: (qNextActive) => {
                const qFull = this._writeHumanoidActiveJoints(currentQ, qNextActive, armStart, activeCount);
                const cctsm = this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                    { force: true }
                );
                return cctsm?.w_vfi ?? Number.NaN;
            },
            wvfiMin: this._getWVFILowerBound(includeWaist)
        });

        let effectiveActive = qInitialActive.slice();
        let refineResult = null;
        if (nullspaceResult.accepted === true) {
            refineResult = restoreHumanoidJogManifold({
                qInitial: nullspaceResult.qNext,
                evaluateProblem,
                maxIterations: Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter)
                    ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter))
                    : 24,
                damping: Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4,
                referenceStiffness: Number.isFinite(cfg.manifoldReferenceStiffness)
                    ? Math.max(0, cfg.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(cfg.manifoldReferenceDamping)
                    ? Math.max(0, cfg.manifoldReferenceDamping)
                    : 0.35,
                initialScale: 1,
                minScale: Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6,
                reductionFactor: Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5,
                equalityTolerance: 1e-12,
                inequalityTolerance: 1e-12
            });
            if (this._isHumanoidMetricRestoreUsable(refineResult)) {
                effectiveActive = refineResult.qApplied.slice();
            }
        }

        const qApplied = this._writeHumanoidActiveJoints(currentQ, effectiveActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qApplied, armStart, hardLimits);
        const finalProblem = evaluateProblem(effectiveActive);
        const relNorm = Math.sqrt((finalProblem?.equalities?.blocks?.relative?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const bodyNorm = Math.sqrt((finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits)
        );
        const wAbsDirection = evaluateHumanoidJogManifoldObjectiveDirection({
            before: evaluateHumanoidJogAbsoluteManipulability(restoredProblem?.metadata?.cctsm),
            after: evaluateHumanoidJogAbsoluteManipulability(finalProblem?.metadata?.cctsm),
            direction
        });
        const wAbsUsable = this._isHumanoidMetricRestoreUsable(refineResult);
        const wAbsApplied = nullspaceResult.accepted === true && wAbsUsable && wAbsDirection.applied === true;
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'w-abs-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: refineResult?.converged === true,
                iterations: refineResult?.iterations ?? 0,
                acceptedIterations: refineResult?.acceptedIterations ?? 0,
                currentResidualNorm: refineResult?.currentResidualNorm ?? Number.NaN,
                lastAcceptedScale: refineResult?.lastAcceptedScale ?? Number.NaN,
                reason: refineResult?.reason ?? 'skipped'
            },
            nullspaceResult,
            optimizationMetric: 'w_abs',
            restoredWVFI: evaluateHumanoidJogAbsoluteManipulability(restoredProblem?.metadata?.cctsm),
            refinedWVFI: evaluateHumanoidJogAbsoluteManipulability(finalProblem?.metadata?.cctsm),
            wvfiOptimizationApplied: wAbsApplied,
            cctsmResult,
            relativeResidualNorm: relNorm,
            bodyResidualNorm: bodyNorm,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: wAbsApplied
                ? '约束流形 w_abs 单步更新完成。'
                : '无法到达该位置（w_abs 优化未通过硬约束）。',
            iterations: (refineResult?.iterations ?? 0) + (nullspaceResult?.accepted ? 1 : 0),
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(relNorm, bodyNorm),
            stepNorm: wAbsApplied
                ? this._computeHumanoidActiveJointDistance(qApplied, currentQ, armStart, activeCount)
                : 0,
            diagnosticLabel: [
                `restore=${refineResult?.reason ?? 'skipped'}`,
                `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
                `metric=${wAbsDirection.reason}`,
                `rel=${this._formatDiagNumber(relNorm)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(bodyNorm)}` : null
            ].filter(Boolean).join('; ')
        });
        if (wAbsApplied) {
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_w_abs_only',
                converged: true,
                reason: 'w_abs_only_converged',
                message: '约束流形 w_abs 单步更新完成。',
                finalResidual: 0,
                invariantResidual: Math.hypot(relNorm, bodyNorm)
            };
            this._applyHumanoidJointSolution(qApplied);
            return true;
        }

        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_w_abs_only',
            converged: false,
            reason: 'w_abs_only_failed',
            message: '无法到达该位置（w_abs 优化未通过硬约束）。',
            finalResidual: 0,
            invariantResidual: Math.hypot(relNorm, bodyNorm)
        };
        return true;
    }

    _optimizeHumanoidAbsoluteArmManipulabilityManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_arm_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_arm_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const currentQ = window.humanoidQ.slice();
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1;
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount: includeWaist ? this.humanoidWaistLength : 0,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const qInitialActive = this._sliceHumanoidActiveJoints(currentQ, armStart, activeCount);
        const relConstraintArray = this._getHumanoidRelativePose(currentQ);
        const absConstraintArray = this._getHumanoidAbsolutePose(currentQ);
        if (!relConstraintArray || !absConstraintArray) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_arm_only',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const evaluateProblem = (qActiveEval) => {
            const qFull = this._writeHumanoidActiveJoints(currentQ, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const cctsm = this._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
            );
            const wvfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'wvfi'
            });
            const wAbsArmGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'w_abs_arm'
            });

            const problem = buildHumanoidJogManifoldProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: absConstraintArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0,
                bodyWeight: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0)
                    : 1.0,
                wvfiState: () => ({
                    value: cctsm?.w_vfi ?? Number.NaN,
                    gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                }),
                wvfiLowerBound: this._getWVFILowerBound(includeWaist),
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull: currentQ,
                    armStart,
                    activeCount,
                    includeWaist
                })
            });
            problem.metadata = {
                qFull,
                cctsm,
                wAbsArmGradient: Array.isArray(wAbsArmGradient) ? wAbsArmGradient.slice() : new Array(activeCount).fill(0)
            };
            return problem;
        };

        const restoredProblem = evaluateProblem(qInitialActive);
        if (!restoredProblem) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_arm_only',
                reason: 'problem_eval_failed',
                message: '约束流形 w_abs_arm-only 问题构建失败。'
            });
        }

        const buttonStepSize = Number.isFinite(cfg.manifoldWVFIButtonStepSize)
            ? cfg.manifoldWVFIButtonStepSize
            : 0.01;
        const signedStep = Math.abs(buttonStepSize)
            * (direction === 'decrease' ? -1 : 1);
        const nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: qInitialActive,
            equalityJacobian: restoredProblem?.equalities?.jacobian ?? [],
            objectiveGradient: restoredProblem?.metadata?.wAbsArmGradient ?? new Array(activeCount).fill(0),
            wvfiGradient: restoredProblem?.inequalities?.wvfiFloor?.gradient ?? new Array(activeCount).fill(0),
            stepSize: signedStep,
            lowerBounds,
            upperBounds,
            stepBounds: jointMaxSteps,
            evaluateWVFI: (qNextActive) => {
                const qFull = this._writeHumanoidActiveJoints(currentQ, qNextActive, armStart, activeCount);
                const cctsm = this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                    { force: true }
                );
                return cctsm?.w_vfi ?? Number.NaN;
            },
            wvfiMin: this._getWVFILowerBound(includeWaist)
        });

        let effectiveActive = qInitialActive.slice();
        let refineResult = null;
        if (nullspaceResult.accepted === true) {
            refineResult = restoreHumanoidJogManifold({
                qInitial: nullspaceResult.qNext,
                evaluateProblem,
                maxIterations: Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter)
                    ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter))
                    : 24,
                damping: Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4,
                referenceStiffness: Number.isFinite(cfg.manifoldReferenceStiffness)
                    ? Math.max(0, cfg.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(cfg.manifoldReferenceDamping)
                    ? Math.max(0, cfg.manifoldReferenceDamping)
                    : 0.35,
                initialScale: 1,
                minScale: Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6,
                reductionFactor: Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5,
                equalityTolerance: 1e-12,
                inequalityTolerance: 1e-12
            });
            if (this._isHumanoidMetricRestoreUsable(refineResult)) {
                effectiveActive = refineResult.qApplied.slice();
            }
        }

        const qApplied = this._writeHumanoidActiveJoints(currentQ, effectiveActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qApplied, armStart, hardLimits);
        const finalProblem = evaluateProblem(effectiveActive);
        const relNorm = Math.sqrt((finalProblem?.equalities?.blocks?.relative?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const bodyNorm = Math.sqrt((finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits)
        );
        const wAbsArmDirection = evaluateHumanoidJogManifoldObjectiveDirection({
            before: evaluateHumanoidJogAbsoluteArmManipulability(restoredProblem?.metadata?.cctsm),
            after: evaluateHumanoidJogAbsoluteArmManipulability(finalProblem?.metadata?.cctsm),
            direction
        });
        const wAbsArmUsable = this._isHumanoidMetricRestoreUsable(refineResult);
        const wAbsArmApplied = nullspaceResult.accepted === true && wAbsArmUsable && wAbsArmDirection.applied === true;
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'w-abs-arm-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: refineResult?.converged === true,
                iterations: refineResult?.iterations ?? 0,
                acceptedIterations: refineResult?.acceptedIterations ?? 0,
                currentResidualNorm: refineResult?.currentResidualNorm ?? Number.NaN,
                lastAcceptedScale: refineResult?.lastAcceptedScale ?? Number.NaN,
                reason: refineResult?.reason ?? 'skipped'
            },
            nullspaceResult,
            optimizationMetric: 'w_abs_arm',
            restoredWVFI: evaluateHumanoidJogAbsoluteArmManipulability(restoredProblem?.metadata?.cctsm),
            refinedWVFI: evaluateHumanoidJogAbsoluteArmManipulability(finalProblem?.metadata?.cctsm),
            wvfiOptimizationApplied: wAbsArmApplied,
            cctsmResult,
            relativeResidualNorm: relNorm,
            bodyResidualNorm: bodyNorm,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: wAbsArmApplied
                ? '约束流形 w_abs_arm 单步更新完成。'
                : '无法到达该位置（w_abs_arm 优化未通过硬约束）。',
            iterations: (refineResult?.iterations ?? 0) + (nullspaceResult?.accepted ? 1 : 0),
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(relNorm, bodyNorm),
            stepNorm: wAbsArmApplied
                ? this._computeHumanoidActiveJointDistance(qApplied, currentQ, armStart, activeCount)
                : 0,
            diagnosticLabel: [
                `restore=${refineResult?.reason ?? 'skipped'}`,
                `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
                `metric=${wAbsArmDirection.reason}`,
                `rel=${this._formatDiagNumber(relNorm)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(bodyNorm)}` : null
            ].filter(Boolean).join('; ')
        });
        if (wAbsArmApplied) {
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_w_abs_arm_only',
                converged: true,
                reason: 'w_abs_arm_only_converged',
                message: '约束流形 w_abs_arm 单步更新完成。',
                finalResidual: 0,
                invariantResidual: Math.hypot(relNorm, bodyNorm)
            };
            this._applyHumanoidJointSolution(qApplied);
            return true;
        }

        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_w_abs_arm_only',
            converged: false,
            reason: 'w_abs_arm_only_failed',
            message: '无法到达该位置（w_abs_arm 优化未通过硬约束）。',
            finalResidual: 0,
            invariantResidual: Math.hypot(relNorm, bodyNorm)
        };
        return true;
    }

    _optimizeHumanoidHierarchicalAbsoluteManipulabilityManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_hier_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_hier_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const initialQ = window.humanoidQ.slice();
        const initialAbs = this._getHumanoidAbsolutePose(initialQ);
        const initialRel = this._getHumanoidRelativePose(initialQ);
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const computeCCTSM = (qFull) => this._computeModeCCTSM(
            qFull,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
        );
        const computeResiduals = (qFull) => {
            const absNow = this._getHumanoidAbsolutePose(qFull);
            const relNow = this._getHumanoidRelativePose(qFull);
            const absError = (initialAbs && absNow)
                ? this._computePoseErrorNorm(this._signedPoseError(initialAbs, absNow))
                : Number.NaN;
            const relError = (initialRel && relNow)
                ? this._computePoseErrorNorm(this._signedPoseError(initialRel, relNow))
                : Number.NaN;
            let bodyError = 0;
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                bodyError = Math.sqrt((bodyState?.error ?? []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
            }
            return { absError, relError, bodyError };
        };

        const initialCCTSM = computeCCTSM(initialQ);
        const armBefore = evaluateHumanoidJogAbsoluteArmManipulability(initialCCTSM);
        const fullBefore = evaluateHumanoidJogAbsoluteManipulability(initialCCTSM);

        this._optimizeHumanoidAbsoluteArmManipulabilityManifold(direction);
        const armStageStatus = { ...(this.lastIKSolveStatus || {}) };
        const armStageDiag = { ...(this.lastHumanoidJogDiagnostics || {}) };
        const armStageApplied = armStageStatus.reason === 'w_abs_arm_only_converged';

        if (!armStageApplied) {
            this._publishHumanoidJogDiagnostics({
                ...armStageDiag,
                active: true,
                solverMode: 'manifold',
                modeTag: 'w-abs-hier-only',
                message: '无法到达该位置（w_abs_hier 主级优化未通过硬约束）。',
                diagnosticLabel: 'primary=not_improved; secondary=skipped'
            });
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_w_abs_hier_only',
                converged: false,
                reason: 'w_abs_hier_primary_failed',
                message: '无法到达该位置（w_abs_hier 主级优化未通过硬约束）。',
                finalResidual: 0,
                invariantResidual: Number.isFinite(armStageStatus.invariantResidual) ? armStageStatus.invariantResidual : Number.NaN
            };
            return true;
        }

        const qAfterPrimary = window.humanoidQ.slice();
        const primaryCCTSM = computeCCTSM(qAfterPrimary);
        const armAfterPrimary = evaluateHumanoidJogAbsoluteArmManipulability(primaryCCTSM);
        const primaryDirection = evaluateHumanoidJogManifoldObjectiveDirection({
            before: armBefore,
            after: armAfterPrimary,
            direction
        });

        this._optimizeHumanoidAbsoluteManipulabilityManifold(direction);
        const fullStageStatus = { ...(this.lastIKSolveStatus || {}) };
        const fullStageDiag = { ...(this.lastHumanoidJogDiagnostics || {}) };
        const fullStageApplied = fullStageStatus.reason === 'w_abs_only_converged';

        let finalQ = qAfterPrimary.slice();
        let finalCCTSM = primaryCCTSM;
        let finalAbsMetric = evaluateHumanoidJogAbsoluteManipulability(primaryCCTSM);
        let secondaryDirection = {
            before: finalAbsMetric,
            after: finalAbsMetric,
            delta: 0,
            applied: false,
            reason: 'skipped'
        };
        let armGuard = {
            before: armAfterPrimary,
            after: armAfterPrimary,
            delta: 0,
            applied: true,
            reason: 'protected'
        };
        let hierarchicalApplied = true;
        let hierarchicalReason = 'primary_only';

        if (fullStageApplied) {
            const qAfterSecondary = window.humanoidQ.slice();
            const secondaryCCTSM = computeCCTSM(qAfterSecondary);
            const fullBeforeSecondary = evaluateHumanoidJogAbsoluteManipulability(primaryCCTSM);
            const fullAfterSecondary = evaluateHumanoidJogAbsoluteManipulability(secondaryCCTSM);
            secondaryDirection = evaluateHumanoidJogManifoldObjectiveDirection({
                before: fullBeforeSecondary,
                after: fullAfterSecondary,
                direction
            });
            armGuard = evaluateHumanoidJogManifoldNonDegradation({
                before: armAfterPrimary,
                after: evaluateHumanoidJogAbsoluteArmManipulability(secondaryCCTSM),
                direction
            });

            if (secondaryDirection.applied === true && armGuard.applied === true) {
                finalQ = qAfterSecondary.slice();
                finalCCTSM = secondaryCCTSM;
                finalAbsMetric = fullAfterSecondary;
                hierarchicalReason = 'hierarchical_accepted';
            } else {
                this._applyHumanoidJointSolution(qAfterPrimary);
            }
        }

        const finalResiduals = computeResiduals(finalQ);
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'w-abs-hier-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: true,
                iterations: (Number.isFinite(armStageDiag.iterations) ? armStageDiag.iterations : 0)
                    + (Number.isFinite(fullStageDiag.iterations) ? fullStageDiag.iterations : 0),
                acceptedIterations: 0,
                currentResidualNorm: Math.hypot(finalResiduals.relError, finalResiduals.bodyError),
                lastAcceptedScale: Number.NaN,
                reason: hierarchicalReason
            },
            nullspaceResult: {
                accepted: true,
                reason: hierarchicalReason,
                stepNorm: this._computeHumanoidActiveJointDistance(finalQ, initialQ, armStart, activeCount)
            },
            optimizationMetric: 'w_abs_hier',
            restoredWVFI: fullBefore,
            refinedWVFI: finalAbsMetric,
            wvfiOptimizationApplied: hierarchicalApplied,
            cctsmResult: finalCCTSM,
            relativeResidualNorm: finalResiduals.relError,
            bodyResidualNorm: finalResiduals.bodyError
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: hierarchicalReason === 'hierarchical_accepted'
                ? '约束流形 w_abs_hier 单步更新完成。'
                : '约束流形 w_abs_hier 主级更新完成。',
            iterations: payload.restoreIterations,
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(finalResiduals.relError, finalResiduals.bodyError),
            stepNorm: this._computeHumanoidActiveJointDistance(finalQ, initialQ, armStart, activeCount),
            diagnosticLabel: [
                `primary=${primaryDirection.reason}`,
                `secondary=${secondaryDirection.reason}`,
                `arm_guard=${armGuard.reason}`,
                `rel=${this._formatDiagNumber(finalResiduals.relError)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(finalResiduals.bodyError)}` : null
            ].filter(Boolean).join('; ')
        });
        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_w_abs_hier_only',
            converged: true,
            reason: hierarchicalReason === 'hierarchical_accepted'
                ? 'w_abs_hier_only_converged'
                : 'w_abs_hier_only_primary_only',
            message: hierarchicalReason === 'hierarchical_accepted'
                ? '约束流形 w_abs_hier 单步更新完成。'
                : '约束流形 w_abs_hier 主级更新完成。',
            finalResidual: 0,
            invariantResidual: Math.hypot(finalResiduals.relError, finalResiduals.bodyError)
        };
        return true;
    }

    _optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_hier_vfi_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_hier_vfi_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const initialQ = window.humanoidQ.slice();
        const initialAbs = this._getHumanoidAbsolutePose(initialQ);
        const initialRel = this._getHumanoidRelativePose(initialQ);
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const computeCCTSM = (qFull) => this._computeModeCCTSM(
            qFull,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
        );
        const computeResiduals = (qFull) => {
            const absNow = this._getHumanoidAbsolutePose(qFull);
            const relNow = this._getHumanoidRelativePose(qFull);
            const absError = (initialAbs && absNow)
                ? this._computePoseErrorNorm(this._signedPoseError(initialAbs, absNow))
                : Number.NaN;
            const relError = (initialRel && relNow)
                ? this._computePoseErrorNorm(this._signedPoseError(initialRel, relNow))
                : Number.NaN;
            let bodyError = 0;
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                bodyError = Math.sqrt((bodyState?.error ?? []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
            }
            return { absError, relError, bodyError };
        };

        const initialCCTSM = computeCCTSM(initialQ);
        const armBefore = evaluateHumanoidJogAbsoluteArmManipulability(initialCCTSM);
        const fullBefore = evaluateHumanoidJogAbsoluteVFIManipulability(initialCCTSM);

        this._optimizeHumanoidAbsoluteArmManipulabilityManifold(direction);
        const armStageStatus = { ...(this.lastIKSolveStatus || {}) };
        const armStageDiag = { ...(this.lastHumanoidJogDiagnostics || {}) };
        const armStageApplied = armStageStatus.reason === 'w_abs_arm_only_converged';

        if (!armStageApplied) {
            this._publishHumanoidJogDiagnostics({
                ...armStageDiag,
                active: true,
                solverMode: 'manifold',
                modeTag: 'w-abs-hier-vfi-only',
                message: '无法到达该位置（w_abs_hier_vfi 主级优化未通过硬约束）。',
                diagnosticLabel: 'primary=not_improved; secondary=skipped'
            });
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_w_abs_hier_vfi_only',
                converged: false,
                reason: 'w_abs_hier_vfi_primary_failed',
                message: '无法到达该位置（w_abs_hier_vfi 主级优化未通过硬约束）。',
                finalResidual: 0,
                invariantResidual: Number.isFinite(armStageStatus.invariantResidual) ? armStageStatus.invariantResidual : Number.NaN
            };
            return true;
        }

        const qAfterPrimary = window.humanoidQ.slice();
        const primaryCCTSM = computeCCTSM(qAfterPrimary);
        const armAfterPrimary = evaluateHumanoidJogAbsoluteArmManipulability(primaryCCTSM);
        const primaryDirection = evaluateHumanoidJogManifoldObjectiveDirection({
            before: armBefore,
            after: armAfterPrimary,
            direction
        });

        this._optimizeHumanoidAbsoluteVFIManipulabilityManifold(direction);
        const fullStageStatus = { ...(this.lastIKSolveStatus || {}) };
        const fullStageDiag = { ...(this.lastHumanoidJogDiagnostics || {}) };
        const fullStageApplied = fullStageStatus.reason === 'w_abs_vfi_only_converged';

        let finalQ = qAfterPrimary.slice();
        let finalCCTSM = primaryCCTSM;
        let finalAbsMetric = evaluateHumanoidJogAbsoluteVFIManipulability(primaryCCTSM);
        let secondaryDirection = {
            before: finalAbsMetric,
            after: finalAbsMetric,
            delta: 0,
            applied: false,
            reason: 'skipped'
        };
        let armGuard = {
            before: armAfterPrimary,
            after: armAfterPrimary,
            delta: 0,
            applied: true,
            reason: 'protected'
        };
        let hierarchicalReason = 'primary_only';

        if (fullStageApplied) {
            const qAfterSecondary = window.humanoidQ.slice();
            const secondaryCCTSM = computeCCTSM(qAfterSecondary);
            const fullBeforeSecondary = evaluateHumanoidJogAbsoluteVFIManipulability(primaryCCTSM);
            const fullAfterSecondary = evaluateHumanoidJogAbsoluteVFIManipulability(secondaryCCTSM);
            secondaryDirection = evaluateHumanoidJogManifoldObjectiveDirection({
                before: fullBeforeSecondary,
                after: fullAfterSecondary,
                direction
            });
            armGuard = evaluateHumanoidJogManifoldNonDegradation({
                before: armAfterPrimary,
                after: evaluateHumanoidJogAbsoluteArmManipulability(secondaryCCTSM),
                direction
            });

            if (secondaryDirection.applied === true && armGuard.applied === true) {
                finalQ = qAfterSecondary.slice();
                finalCCTSM = secondaryCCTSM;
                finalAbsMetric = fullAfterSecondary;
                hierarchicalReason = 'hierarchical_accepted';
            } else {
                this._applyHumanoidJointSolution(qAfterPrimary);
            }
        }

        const finalResiduals = computeResiduals(finalQ);
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'w-abs-hier-vfi-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: true,
                iterations: (Number.isFinite(armStageDiag.iterations) ? armStageDiag.iterations : 0)
                    + (Number.isFinite(fullStageDiag.iterations) ? fullStageDiag.iterations : 0),
                acceptedIterations: 0,
                currentResidualNorm: Math.hypot(finalResiduals.relError, finalResiduals.bodyError),
                lastAcceptedScale: Number.NaN,
                reason: hierarchicalReason
            },
            nullspaceResult: {
                accepted: true,
                reason: hierarchicalReason,
                stepNorm: this._computeHumanoidActiveJointDistance(finalQ, initialQ, armStart, activeCount)
            },
            optimizationMetric: 'w_abs_hier_vfi',
            restoredWVFI: fullBefore,
            refinedWVFI: finalAbsMetric,
            wvfiOptimizationApplied: true,
            cctsmResult: finalCCTSM,
            relativeResidualNorm: finalResiduals.relError,
            bodyResidualNorm: finalResiduals.bodyError
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: hierarchicalReason === 'hierarchical_accepted'
                ? '约束流形 w_abs_hier_vfi 单步更新完成。'
                : '约束流形 w_abs_hier_vfi 主级更新完成。',
            iterations: payload.restoreIterations,
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(finalResiduals.relError, finalResiduals.bodyError),
            stepNorm: this._computeHumanoidActiveJointDistance(finalQ, initialQ, armStart, activeCount),
            diagnosticLabel: [
                `primary=${primaryDirection.reason}`,
                `secondary=${secondaryDirection.reason}`,
                `arm_guard=${armGuard.reason}`,
                `rel=${this._formatDiagNumber(finalResiduals.relError)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(finalResiduals.bodyError)}` : null
            ].filter(Boolean).join('; ')
        });
        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_w_abs_hier_vfi_only',
            converged: true,
            reason: hierarchicalReason === 'hierarchical_accepted'
                ? 'w_abs_hier_vfi_only_converged'
                : 'w_abs_hier_vfi_only_primary_only',
            message: hierarchicalReason === 'hierarchical_accepted'
                ? '约束流形 w_abs_hier_vfi 单步更新完成。'
                : '约束流形 w_abs_hier_vfi 主级更新完成。',
            finalResidual: 0,
            invariantResidual: Math.hypot(finalResiduals.relError, finalResiduals.bodyError)
        };
        return true;
    }

    _optimizeHumanoidAbsoluteVFIManipulabilityManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_vfi_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_vfi_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const currentQ = window.humanoidQ.slice();
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1;
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount: includeWaist ? this.humanoidWaistLength : 0,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const qInitialActive = this._sliceHumanoidActiveJoints(currentQ, armStart, activeCount);
        const relConstraintArray = this._getHumanoidRelativePose(currentQ);
        const absConstraintArray = this._getHumanoidAbsolutePose(currentQ);
        if (!relConstraintArray || !absConstraintArray) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_vfi_only',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const evaluateProblem = (qActiveEval) => {
            const qFull = this._writeHumanoidActiveJoints(currentQ, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const cctsm = this._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
            );
            const wvfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'wvfi'
            });
            const wAbsVfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'w_abs_vfi'
            });

            const problem = buildHumanoidJogManifoldProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: absConstraintArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0,
                bodyWeight: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0)
                    : 1.0,
                wvfiState: () => ({
                    value: cctsm?.w_vfi ?? Number.NaN,
                    gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                }),
                wvfiLowerBound: this._getWVFILowerBound(includeWaist),
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull: currentQ,
                    armStart,
                    activeCount,
                    includeWaist
                })
            });
            problem.metadata = {
                qFull,
                cctsm,
                wAbsVfiGradient: Array.isArray(wAbsVfiGradient) ? wAbsVfiGradient.slice() : new Array(activeCount).fill(0)
            };
            return problem;
        };

        const restoredProblem = evaluateProblem(qInitialActive);
        if (!restoredProblem) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_w_abs_vfi_only',
                reason: 'problem_eval_failed',
                message: '约束流形 w_abs_vfi-only 问题构建失败。'
            });
        }

        const buttonStepSize = Number.isFinite(cfg.manifoldWVFIButtonStepSize)
            ? cfg.manifoldWVFIButtonStepSize
            : 0.01;
        const signedStep = Math.abs(buttonStepSize)
            * (direction === 'decrease' ? -1 : 1);
        const nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: qInitialActive,
            equalityJacobian: restoredProblem?.equalities?.jacobian ?? [],
            objectiveGradient: restoredProblem?.metadata?.wAbsVfiGradient ?? new Array(activeCount).fill(0),
            wvfiGradient: restoredProblem?.inequalities?.wvfiFloor?.gradient ?? new Array(activeCount).fill(0),
            stepSize: signedStep,
            lowerBounds,
            upperBounds,
            stepBounds: jointMaxSteps,
            evaluateWVFI: (qNextActive) => {
                const qFull = this._writeHumanoidActiveJoints(currentQ, qNextActive, armStart, activeCount);
                const cctsm = this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                    { force: true }
                );
                return cctsm?.w_vfi ?? Number.NaN;
            },
            wvfiMin: this._getWVFILowerBound(includeWaist)
        });

        let effectiveActive = qInitialActive.slice();
        let refineResult = null;
        if (nullspaceResult.accepted === true) {
            refineResult = restoreHumanoidJogManifold({
                qInitial: nullspaceResult.qNext,
                evaluateProblem,
                maxIterations: Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter)
                    ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter))
                    : 24,
                damping: Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4,
                referenceStiffness: Number.isFinite(cfg.manifoldReferenceStiffness)
                    ? Math.max(0, cfg.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(cfg.manifoldReferenceDamping)
                    ? Math.max(0, cfg.manifoldReferenceDamping)
                    : 0.35,
                initialScale: 1,
                minScale: Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6,
                reductionFactor: Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5,
                equalityTolerance: 1e-12,
                inequalityTolerance: 1e-12
            });
            if (this._isHumanoidMetricRestoreUsable(refineResult)) {
                effectiveActive = refineResult.qApplied.slice();
            }
        }

        const qApplied = this._writeHumanoidActiveJoints(currentQ, effectiveActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qApplied, armStart, hardLimits);
        const finalProblem = evaluateProblem(effectiveActive);
        const relNorm = Math.sqrt((finalProblem?.equalities?.blocks?.relative?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const bodyNorm = Math.sqrt((finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits)
        );
        const wAbsVFIDirection = evaluateHumanoidJogManifoldObjectiveDirection({
            before: evaluateHumanoidJogAbsoluteVFIManipulability(restoredProblem?.metadata?.cctsm),
            after: evaluateHumanoidJogAbsoluteVFIManipulability(finalProblem?.metadata?.cctsm),
            direction
        });
        const wAbsVFIUsable = this._isHumanoidMetricRestoreUsable(refineResult);
        const wAbsVFIApplied = nullspaceResult.accepted === true && wAbsVFIUsable && wAbsVFIDirection.applied === true;
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'w-abs-vfi-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: refineResult?.converged === true,
                iterations: refineResult?.iterations ?? 0,
                acceptedIterations: refineResult?.acceptedIterations ?? 0,
                currentResidualNorm: refineResult?.currentResidualNorm ?? Number.NaN,
                lastAcceptedScale: refineResult?.lastAcceptedScale ?? Number.NaN,
                reason: refineResult?.reason ?? 'skipped'
            },
            nullspaceResult,
            optimizationMetric: 'w_abs_vfi',
            restoredWVFI: evaluateHumanoidJogAbsoluteVFIManipulability(restoredProblem?.metadata?.cctsm),
            refinedWVFI: evaluateHumanoidJogAbsoluteVFIManipulability(finalProblem?.metadata?.cctsm),
            wvfiOptimizationApplied: wAbsVFIApplied,
            cctsmResult,
            relativeResidualNorm: relNorm,
            bodyResidualNorm: bodyNorm,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: wAbsVFIApplied
                ? '约束流形 w_abs_vfi 单步更新完成。'
                : '无法到达该位置（w_abs_vfi 优化未通过硬约束）。',
            iterations: (refineResult?.iterations ?? 0) + (nullspaceResult?.accepted ? 1 : 0),
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(relNorm, bodyNorm),
            stepNorm: wAbsVFIApplied
                ? this._computeHumanoidActiveJointDistance(qApplied, currentQ, armStart, activeCount)
                : 0,
            diagnosticLabel: [
                `restore=${refineResult?.reason ?? 'skipped'}`,
                `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
                `metric=${wAbsVFIDirection.reason}`,
                `rel=${this._formatDiagNumber(relNorm)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(bodyNorm)}` : null
            ].filter(Boolean).join('; ')
        });
        if (wAbsVFIApplied) {
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_w_abs_vfi_only',
                converged: true,
                reason: 'w_abs_vfi_only_converged',
                message: '约束流形 w_abs_vfi 单步更新完成。',
                finalResidual: 0,
                invariantResidual: Math.hypot(relNorm, bodyNorm)
            };
            this._applyHumanoidJointSolution(qApplied);
            return true;
        }

        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_w_abs_vfi_only',
            converged: false,
            reason: 'w_abs_vfi_only_failed',
            message: '无法到达该位置（w_abs_vfi 优化未通过硬约束）。',
            finalResidual: 0,
            invariantResidual: Math.hypot(relNorm, bodyNorm)
        };
        return true;
    }

    _optimizeHumanoidDirectionalManipulabilityManifold(direction = 'increase') {
        if (!this.humanoidModel || !window.humanoidQ || window.humanoidQ.length === 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_sigma_only',
                reason: 'invalid_state',
                message: '人形状态未初始化。'
            });
        }

        const axis = typeof this.lastHumanoidJogAxis === 'string' ? this.lastHumanoidJogAxis : '';
        const directionVector = buildHumanoidJogDirectionalAxisVector({
            axis,
            gramSize: 12
        });
        if (!Array.isArray(directionVector)) {
            this._publishHumanoidJogDiagnostics({
                active: true,
                solverMode: 'manifold',
                modeTag: 'sigma-only',
                message: '请先按一次 X/Y/Z/Rx/Ry/Rz 点动按钮，再使用 sigma 按钮。',
                diagnosticLabel: 'sigma_only=no_reference_axis',
                includeWaist: false,
                keepBodyZVertical: false,
                jointLimitAvoidance: true
            });
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_sigma_only',
                converged: false,
                reason: 'no_reference_axis',
                message: '请先按一次 X/Y/Z/Rx/Ry/Rz 点动按钮，再使用 sigma 按钮。',
                finalResidual: Number.NaN,
                invariantResidual: Number.NaN
            };
            return false;
        }

        const controlMode = document.getElementById('dual-arm-control-mode-select')?.value || 'absolute';
        const includeWaist = controlMode === 'decoupled' || controlMode === 'decoupled-upright';
        const keepBodyZVertical = controlMode === 'decoupled-upright';
        const enableJointLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

        const { armStart, activeCount } = this._getHumanoidActiveSliceInfo(includeWaist);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_sigma_only',
                reason: 'no_active_joints',
                message: '当前模式下没有可用活动关节。'
            });
        }

        const cfg = this.vfiConfig || {};
        const currentQ = window.humanoidQ.slice();
        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.1;
        const jointMaxSteps = buildHumanoidJogManifoldStepBounds({
            maxStep,
            includeWaist,
            keepBodyZVertical,
            waistCount: includeWaist ? this.humanoidWaistLength : 0,
            activeCount,
            waistStepFraction: Number.isFinite(cfg.manifoldWaistStepFraction)
                ? cfg.manifoldWaistStepFraction
                : 1.0
        });
        const lowerBounds = hardLimits
            ? hardLimits.map((limit) => limit?.lower ?? -Math.PI)
            : new Array(activeCount).fill(-Math.PI);
        const upperBounds = hardLimits
            ? hardLimits.map((limit) => limit?.upper ?? Math.PI)
            : new Array(activeCount).fill(Math.PI);
        const qInitialActive = this._sliceHumanoidActiveJoints(currentQ, armStart, activeCount);
        const relConstraintArray = this._getHumanoidRelativePose(currentQ);
        const absConstraintArray = this._getHumanoidAbsolutePose(currentQ);
        if (!relConstraintArray || !absConstraintArray) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_sigma_only',
                reason: 'fk_unavailable',
                message: '当前人形位姿计算失败。'
            });
        }

        const evaluateProblem = (qActiveEval) => {
            const qFull = this._writeHumanoidActiveJoints(currentQ, qActiveEval, armStart, activeCount);
            const absPose = this._getHumanoidAbsolutePose(qFull);
            const relPose = this._getHumanoidRelativePose(qFull);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(qFull);
            const JrelFull = this._getHumanoidRelativePoseJacobian(qFull);
            if (!absPose || !relPose || !JabsFull || !JrelFull) return null;

            const sliceRows = (rows, rowCount = rows.length) => {
                const out = [];
                for (let i = 0; i < rowCount; i++) {
                    const row = Array.isArray(rows?.[i]) ? rows[i].slice(armStart, armStart + activeCount) : null;
                    if (!row) return null;
                    out.push(row);
                }
                return out;
            };

            const Jabs = sliceRows(JabsFull, 8);
            const Jrel = sliceRows(JrelFull, 8);
            if (!Jabs || !Jrel) return null;

            let bodyConstraint = null;
            const protectedRows = Jrel.map((row) => row.slice());
            if (keepBodyZVertical) {
                const bodyState = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                if (!bodyState) return null;
                bodyConstraint = {
                    residual: Array.isArray(bodyState.error) ? bodyState.error.slice() : [Number(bodyState.error || 0)],
                    jacobian: Array.isArray(bodyState.Jrows) ? bodyState.Jrows.map((row) => row.slice()) : []
                };
                protectedRows.push(...bodyConstraint.jacobian.map((row) => row.slice()));
            }

            const cctsm = this._computeModeCCTSM(
                qFull,
                armStart,
                activeCount,
                'absolute',
                this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits)
            );
            const wvfiGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'wvfi'
            });
            const sigmaGradient = this._computeProtectedManipulabilityGradient({
                q: qFull,
                armStart,
                activeCount,
                mode: 'absolute',
                Jcmd: Jabs,
                Jinvariant: protectedRows,
                activeLimits: hardLimits,
                forceInvariantEquality: true,
                metric: 'sigma',
                directionalVector: directionVector.slice()
            });
            const sigmaValue = evaluateHumanoidJogDirectionalManipulability({
                axis,
                cctsmResult: cctsm
            });

            const problem = buildHumanoidJogManifoldProblem({
                q: qActiveEval,
                qCurrent: qInitialActive,
                absoluteTarget: absConstraintArray,
                relativeTarget: relConstraintArray,
                absolutePose: () => absPose,
                relativePose: () => relPose,
                absoluteJacobian: () => Jabs,
                relativeJacobian: () => Jrel,
                includeBodyZ: keepBodyZVertical,
                bodyZConstraint: keepBodyZVertical
                    ? () => ({
                        residual: bodyConstraint?.residual ?? [],
                        jacobian: bodyConstraint?.jacobian ?? []
                    })
                    : null,
                bodyEqualityTolerance: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyTolerance) ? cfg.manifoldBodyTolerance : 5e-5)
                    : 0,
                absoluteWeight: 1.0,
                relativeWeight: Number.isFinite(cfg.manifoldRelativeWeight) ? cfg.manifoldRelativeWeight : 1.0,
                bodyWeight: keepBodyZVertical
                    ? (Number.isFinite(cfg.manifoldBodyVerticalWeight) ? cfg.manifoldBodyVerticalWeight : 4.0)
                    : 1.0,
                wvfiState: () => ({
                    value: cctsm?.w_vfi ?? Number.NaN,
                    gradient: Array.isArray(wvfiGradient) ? wvfiGradient.slice() : new Array(activeCount).fill(0)
                }),
                wvfiLowerBound: this._getWVFILowerBound(includeWaist),
                lowerBounds,
                upperBounds,
                stepBounds: jointMaxSteps,
                waistXCorridor: this._buildHumanoidManifoldWaistXCorridor({
                    qFull,
                    qInitialFull: currentQ,
                    armStart,
                    activeCount,
                    includeWaist
                })
            });
            problem.metadata = {
                qFull,
                cctsm,
                sigmaGradient: Array.isArray(sigmaGradient) ? sigmaGradient.slice() : new Array(activeCount).fill(0),
                sigmaValue
            };
            return problem;
        };

        const restoredProblem = evaluateProblem(qInitialActive);
        if (!restoredProblem) {
            return this._setIKFailureStatus({
                solver: 'manifold',
                mode: 'humanoid_sigma_only',
                reason: 'problem_eval_failed',
                message: '约束流形 sigma-only 问题构建失败。'
            });
        }

        const buttonStepSize = Number.isFinite(cfg.manifoldWVFIButtonStepSize)
            ? cfg.manifoldWVFIButtonStepSize
            : 0.01;
        const signedStep = Math.abs(buttonStepSize)
            * (direction === 'decrease' ? -1 : 1);
        const nullspaceResult = computeHumanoidJogManifoldNullspaceStep({
            qCurrent: qInitialActive,
            equalityJacobian: restoredProblem?.equalities?.jacobian ?? [],
            objectiveGradient: restoredProblem?.metadata?.sigmaGradient ?? new Array(activeCount).fill(0),
            wvfiGradient: restoredProblem?.inequalities?.wvfiFloor?.gradient ?? new Array(activeCount).fill(0),
            stepSize: signedStep,
            lowerBounds,
            upperBounds,
            stepBounds: jointMaxSteps,
            evaluateWVFI: (qNextActive) => {
                const qFull = this._writeHumanoidActiveJoints(currentQ, qNextActive, armStart, activeCount);
                const cctsm = this._computeModeCCTSM(
                    qFull,
                    armStart,
                    activeCount,
                    'absolute',
                    this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits),
                    { force: true }
                );
                return cctsm?.w_vfi ?? Number.NaN;
            },
            wvfiMin: this._getWVFILowerBound(includeWaist)
        });

        let effectiveActive = qInitialActive.slice();
        let refineResult = null;
        if (nullspaceResult.accepted === true) {
            refineResult = restoreHumanoidJogManifold({
                qInitial: nullspaceResult.qNext,
                evaluateProblem,
                maxIterations: Number.isFinite(cfg.manifoldNullspaceRestoreMaxIter)
                    ? Math.max(1, Math.floor(cfg.manifoldNullspaceRestoreMaxIter))
                    : 24,
                damping: Number.isFinite(cfg.manifoldDamping) ? Math.max(1e-12, cfg.manifoldDamping) : 1e-4,
                referenceStiffness: Number.isFinite(cfg.manifoldReferenceStiffness)
                    ? Math.max(0, cfg.manifoldReferenceStiffness)
                    : 1.0,
                referenceDamping: Number.isFinite(cfg.manifoldReferenceDamping)
                    ? Math.max(0, cfg.manifoldReferenceDamping)
                    : 0.35,
                initialScale: 1,
                minScale: Number.isFinite(cfg.manifoldLineSearchMinScale) ? Math.max(1e-8, cfg.manifoldLineSearchMinScale) : 1e-6,
                reductionFactor: Number.isFinite(cfg.manifoldLineSearchReductionFactor) ? cfg.manifoldLineSearchReductionFactor : 0.5,
                equalityTolerance: 1e-12,
                inequalityTolerance: 1e-12
            });
            if (this._isHumanoidMetricRestoreUsable(refineResult)) {
                effectiveActive = refineResult.qApplied.slice();
            }
        }

        const qApplied = this._writeHumanoidActiveJoints(currentQ, effectiveActive, armStart, activeCount);
        this._clampHumanoidActiveJoints(qApplied, armStart, hardLimits);
        const finalProblem = evaluateProblem(effectiveActive);
        const relNorm = Math.sqrt((finalProblem?.equalities?.blocks?.relative?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const bodyNorm = Math.sqrt((finalProblem?.equalities?.blocks?.bodyZ?.residual ?? []).reduce((s, v) => s + (v || 0) * (v || 0), 0));
        const cctsmResult = this._computeModeCCTSM(
            qApplied,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(qApplied, armStart, activeCount, hardLimits)
        );
        const sigmaDirection = evaluateHumanoidJogManifoldObjectiveDirection({
            before: restoredProblem?.metadata?.sigmaValue,
            after: finalProblem?.metadata?.sigmaValue,
            direction
        });
        const sigmaUsable = this._isHumanoidMetricRestoreUsable(refineResult);
        const sigmaApplied = nullspaceResult.accepted === true && sigmaUsable && sigmaDirection.applied === true;
        const payload = buildHumanoidJogManifoldDiagnostics({
            modeTag: 'sigma-only',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            restorationResult: {
                converged: refineResult?.converged === true,
                iterations: refineResult?.iterations ?? 0,
                acceptedIterations: refineResult?.acceptedIterations ?? 0,
                currentResidualNorm: refineResult?.currentResidualNorm ?? Number.NaN,
                lastAcceptedScale: refineResult?.lastAcceptedScale ?? Number.NaN,
                reason: refineResult?.reason ?? 'skipped'
            },
            nullspaceResult,
            optimizationMetric: 'sigma',
            optimizationAxis: axis,
            restoredWVFI: restoredProblem?.metadata?.sigmaValue,
            refinedWVFI: finalProblem?.metadata?.sigmaValue,
            wvfiOptimizationApplied: sigmaApplied,
            cctsmResult,
            relativeResidualNorm: relNorm,
            bodyResidualNorm: bodyNorm,
            ...this._extractHumanoidWVFIConstraintDiagnostics(finalProblem?.inequalities?.wvfiFloor),
            ...this._extractHumanoidWaistXCorridorDiagnostics(finalProblem?.inequalities?.waistXCorridor)
        });
        this._publishHumanoidJogDiagnostics({
            ...payload,
            active: true,
            message: sigmaApplied
                ? '约束流形 sigma 单步更新完成。'
                : '无法到达该位置（sigma 优化未通过硬约束）。',
            iterations: (refineResult?.iterations ?? 0) + (nullspaceResult?.accepted ? 1 : 0),
            activeJoints: activeCount,
            commandError: 0,
            invariantError: Math.hypot(relNorm, bodyNorm),
            stepNorm: sigmaApplied
                ? this._computeHumanoidActiveJointDistance(qApplied, currentQ, armStart, activeCount)
                : 0,
            diagnosticLabel: [
                `restore=${refineResult?.reason ?? 'skipped'}`,
                `nullspace=${nullspaceResult?.reason ?? 'skipped'}`,
                `metric=${sigmaDirection.reason}`,
                `axis=${axis}`,
                `rel=${this._formatDiagNumber(relNorm)}`,
                keepBodyZVertical ? `body=${this._formatDiagNumber(bodyNorm)}` : null
            ].filter(Boolean).join('; ')
        });
        if (sigmaApplied) {
            this.lastIKSolveStatus = {
                solver: 'manifold',
                mode: 'humanoid_sigma_only',
                converged: true,
                reason: 'sigma_only_converged',
                message: '约束流形 sigma 单步更新完成。',
                finalResidual: 0,
                invariantResidual: Math.hypot(relNorm, bodyNorm)
            };
            this._applyHumanoidJointSolution(qApplied);
            return true;
        }

        this.lastIKSolveStatus = {
            solver: 'manifold',
            mode: 'humanoid_sigma_only',
            converged: false,
            reason: 'sigma_only_failed',
            message: '无法到达该位置（sigma 优化未通过硬约束）。',
            finalResidual: 0,
            invariantResidual: Math.hypot(relNorm, bodyNorm)
        };
        return true;
    }

    _updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstVFI(
        dqAbsoluteTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        keepBodyZVertical = false,
        fixedRelativeTargetArray = null,
        translationAxisGuard = null
    ) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_absolute', reason: 'invalid_state', message: '人形状态未初始化。' });
        }
        const DQ = window.DQModule?.DQWrapper;
        const Kinematics = window.DQModule?.Kinematics;
        if (!DQ) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_absolute', reason: 'dq_unavailable', message: 'DQ 模块未加载。' });
        }

        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relCurrentArray = this._getHumanoidRelativePose(currentQ);
        if (!absCurrentArray || !relCurrentArray) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_absolute', reason: 'fk_unavailable', message: '当前人形位姿计算失败。' });
        }
        const relConstraintArray = (
            Array.isArray(fixedRelativeTargetArray) &&
            fixedRelativeTargetArray.length === 8 &&
            fixedRelativeTargetArray.every(v => Number.isFinite(Number(v)))
        )
            ? fixedRelativeTargetArray.map(v => Number(v))
            : relCurrentArray.slice();

        const jointLimits = this._getHumanoidJointLimits();
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;
        const activeCount = includeWaist ? (waistCount + leftArmCount + rightArmCount) : (leftArmCount + rightArmCount);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_absolute', reason: 'no_active_joints', message: '当前模式下没有可用活动关节。' });
        }

        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const softLimits = enableJointLimitAvoidance ? hardLimits : null;
        const cfg = this.vfiConfig || {};
        const maxIter = Number.isFinite(cfg.qpOuterMaxIter) ? cfg.qpOuterMaxIter : 16;
        const damping = Number.isFinite(cfg.damping) ? cfg.damping : 1e-3;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.003;
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolPose) ? cfg.hardCommandTolPose : 1e-6;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const practicalCmdTol = Number.isFinite(cfg.practicalCommandTolPose) ? cfg.practicalCommandTolPose : 1e-4;
        const practicalInvariantTol = Number.isFinite(cfg.practicalInvariantTol) ? cfg.practicalInvariantTol : 2.5e-4;
        const practicalStepTol = Number.isFinite(cfg.practicalStepNormTol) ? cfg.practicalStepNormTol : 2e-5;
        const practicalViolationTol = Number.isFinite(cfg.practicalViolationTol) ? cfg.practicalViolationTol : 1e-8;
        const bodyKappa = Number.isFinite(cfg.bodyVerticalKappa) ? cfg.bodyVerticalKappa : 0.8;

        // Per-joint step-size hard bounds: waist joints are limited to a fraction of
        // maxStep so they contribute smoothly without dominating over arm joints.
        // This is a hard QP constraint (not cost-based), so the effect is
        // configuration-independent and avoids jerky bang-bang behavior.
        const lambda = Number.isFinite(cfg.qpRegularization) ? cfg.qpRegularization : 1e-6;
        // In body-vertical mode the waist must move freely to correct orientation,
        // so waistStepFraction is not applied (fraction=1). Without this the QP
        // becomes infeasible (step bound < required waist correction) and cert
        // reports 'unreachable' even when the target is within reach.
        const waistStepFraction = keepBodyZVertical
            ? 1.0
            : (Number.isFinite(cfg.waistStepFraction) ? Math.max(0, Math.min(1, cfg.waistStepFraction)) : 0.3);
        const jointMaxSteps = includeWaist && waistCount > 0
            ? Array.from({ length: activeCount }, (_, i) => i < waistCount ? maxStep * waistStepFraction : maxStep)
            : null;
        const jointRegWeights = null; // regularization is uniform; step bounds handle waist limiting

        const enableBodyVerticalTask = keepBodyZVertical && includeWaist && waistCount > 0
            && Kinematics && typeof Kinematics.lineJacobian === 'function'
            && typeof this.humanoidModel.fkmEffector === 'function'
            && typeof this.humanoidModel.poseJacobianEffector === 'function';

        const qInitial = [...currentQ];
        let q = [...qInitial];
        let executedIterations = 0;
        let stopReason = 'max_iter';
        let initialCmdNorm = Number.NaN;
        let lastCmdNorm = Number.NaN;
        let lastInvariantNorm = Number.NaN;
        let lastStats = null;
        let certFallbackReason = null;
        for (let iter = 0; iter < maxIter; iter++) {
            const relNow = this._getHumanoidRelativePose(q);
            const absNow = this._getHumanoidAbsolutePose(q);
            if (!relNow || !absNow) {
                stopReason = 'fk_fail';
                break;
            }

            const eInvariantCurrent = this._signedPoseError(relConstraintArray, relNow);
            const eCmd = this._signedPoseError(dqAbsoluteTargetArray, absNow);
            const cmdNorm = Math.sqrt(eCmd.reduce((s, v) => s + v * v, 0));
            lastCmdNorm = cmdNorm;
            if (!Number.isFinite(initialCmdNorm)) initialCmdNorm = cmdNorm;

            if (enableBodyVerticalTask) {
                const bodyStateCurrent = this._getHumanoidBodyVerticalState(q, armStart, activeCount);
                if (!bodyStateCurrent) {
                    stopReason = 'body_task_fail';
                    break;
                }
                eInvariantCurrent.push(bodyKappa * (bodyStateCurrent.error[0] || 0));
            }
            lastInvariantNorm = Math.sqrt(eInvariantCurrent.reduce((s, v) => s + v * v, 0));
            if (cmdNorm < hardCmdTol && lastInvariantNorm < hardInvariantTol) {
                stopReason = 'converged';
                break;
            }

            const JrelFull = this._getHumanoidRelativePoseJacobian(q);
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(q);
            if (!JrelFull || !JabsFull) { stopReason = 'solver_fail'; break; }
            const Jinvariant = JrelFull.map((row) => row.slice(armStart, armStart + activeCount));
            const Jcmd = JabsFull.map((row) => row.slice(armStart, armStart + activeCount));
            if (enableBodyVerticalTask) {
                const bodyState = this._getHumanoidBodyVerticalState(q, armStart, activeCount);
                if (!bodyState) { stopReason = 'body_task_fail'; break; }
                Jinvariant.push(bodyState.Jrows[0]);
            }
            const qActive = q.slice(armStart, armStart + activeCount);

            // If predictive cert is enabled, use Algorithm 1 instead of single-step QP
            if (this._isPredictiveCertEnabled() && this._isHumanoidVFISolverEnabled()) {
                const certResult = this._runPredictiveCert({
                    q, armStart, activeCount,
                    mode: 'absolute',
                    memoryKey: `cert|absolute|w:${includeWaist ? 1 : 0}`,
                    oneStepFn: (qAct) => {
                        const qFull = q.slice();
                        for (let j = 0; j < activeCount; j++) qFull[armStart + j] = qAct[j];
                        const JabsF = this._getHumanoidAbsolutePoseJacobian(qFull);
                        const JrelF = this._getHumanoidRelativePoseJacobian(qFull);
                        if (!JabsF || !JrelF) return null;
                        const absAtBranch = this._getHumanoidAbsolutePose(qFull);
                        const relAtBranch = this._getHumanoidRelativePose(qFull);
                        const eCmdLocal = absAtBranch ? this._signedPoseError(dqAbsoluteTargetArray, absAtBranch) : eCmd.slice();
                        const eInvLocal = relAtBranch ? this._signedPoseError(relConstraintArray, relAtBranch) : eInvariantCurrent.slice();
                        const JcmdLocal = JabsF.map(r => r.slice(armStart, armStart + activeCount));
                        const JinvLocal = JrelF.map(r => r.slice(armStart, armStart + activeCount));
                        if (enableBodyVerticalTask) {
                            const bodyAtBranch = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                            if (bodyAtBranch) {
                                eInvLocal.push(bodyKappa * (bodyAtBranch.error[0] || 0));
                                JinvLocal.push(bodyAtBranch.Jrows[0]);
                            }
                        }
                        const res = this._solveVFICommandStep({
                            Jcmd: JcmdLocal, eCmd: eCmdLocal,
                            Jinvariant: JinvLocal, eInvariant: eInvLocal,
                            qActive: qAct, activeLimits: softLimits, maxStep, damping,
                            extraVFIRows: this._getWaistXVFIRows(qFull, armStart, activeCount),
                            jointMaxSteps,
                            nullManipGrad: null,
                            wvfiGuard: this._buildWVFILowerBoundGuard({
                                q: qFull,
                                armStart,
                                activeCount,
                                mode: 'absolute',
                                activeLimits: hardLimits,
                                includeWaist
                            })
                        });
                        if (!res || !res.dq || res.stats?.wvfiGuardAccepted === false) return null;
                        // Use QP convergence, not maxViolationAfter, as the feasibility signal.
                        // maxViolationAfter is computed AFTER a joint post-clamp that modifies dq
                        // independently of the QP solution; near joint limits this inflates the
                        // equality-constraint violation by ~1e-3, falsely marking all branches as
                        // infeasible even when the target is reachable.  qpConverged=true means
                        // the solver found a feasible dq (before the clamp altered it).
                        const slack = res.stats?.qpConverged === false ? Infinity : 0;
                        return { dq: res.dq, slack };
                    },
                    taskErrFn: (qAct) => {
                        const qFull = q.slice(); for (let j = 0; j < activeCount; j++) qFull[armStart + j] = qAct[j];
                        const xA = this._getHumanoidAbsolutePose(qFull);
                        const xR = this._getHumanoidRelativePose(qFull);
                        const xARef = this._getHumanoidAbsolutePose(q);
                        const xRRef = this._getHumanoidRelativePose(q);
                        if (!xA || !xR || !xARef || !xRRef) return new Array(16).fill(0);
                        const errA = this._signedPoseError(xARef, xA);
                        const errR = this._signedPoseError(xRRef, xR);
                        const err = [...errA, ...errR];
                        if (enableBodyVerticalTask) {
                            const bodyRef = this._getHumanoidBodyVerticalState(q, armStart, activeCount);
                            const bodyBranch = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                            if (bodyRef && bodyBranch) {
                                err.push(bodyKappa * ((bodyRef.error[0] || 0) - (bodyBranch.error[0] || 0)));
                            }
                        }
                        return err;
                    },
                    taskJacFn: (qAct) => {
                        const qFull = q.slice(); for (let j = 0; j < activeCount; j++) qFull[armStart + j] = qAct[j];
                        const JA = this._getHumanoidAbsolutePoseJacobian(qFull);
                        const JR = this._getHumanoidRelativePoseJacobian(qFull);
                        if (!JA || !JR) return [];
                        const rows = [...JA.map(r => r.slice(armStart, armStart + activeCount)), ...JR.map(r => r.slice(armStart, armStart + activeCount))];
                        if (enableBodyVerticalTask) {
                            const bodyBranch = this._getHumanoidBodyVerticalState(qFull, armStart, activeCount);
                            if (bodyBranch) rows.push(bodyBranch.Jrows[0]);
                        }
                        return rows;
                    },
                    hardLimits: hardLimits ? hardLimits.map(lim => lim ? [lim.lower, lim.upper] : null) : null,
                    cctsmEvalFn: (qAct) => {
                        const qFull = q.slice(); for (let j = 0; j < activeCount; j++) qFull[armStart + j] = qAct[j];
                        const vfiConstraints = this._buildManipulabilityVFIConstraints(qFull, armStart, activeCount, hardLimits);
                        return this._computeModeCCTSM(qFull, armStart, activeCount, 'absolute', vfiConstraints);
                    },
                    jogDir: eCmd,
                    wMinOverride: this._getWVFILowerBound(includeWaist)
                });
                if (certResult && certResult.qActive) {
                    const qNext = [...q];
                    for (let i = 0; i < activeCount; i++) {
                        let val = asFinite(certResult.qActive[i]);
                        const lim = hardLimits?.[i];
                        if (lim) val = Math.max(lim.lower, Math.min(lim.upper, val));
                        qNext[armStart + i] = val;
                    }
                    // Cmd progress check: if the cert step made the command error worse,
                    // discard it.  This catches "strange-place" moves where the QP is
                    // constraint-feasible (slack ≤ slackTol) but actually moved the arm
                    // away from the target (e.g. pure constraint-avoidance direction).
                    let acceptCertStep = true;
                    const absAfterCert = this._getHumanoidAbsolutePose(qNext);
                    if (absAfterCert && Number.isFinite(initialCmdNorm) && initialCmdNorm > 1e-4) {
                        const eCmdAfter = this._signedPoseError(dqAbsoluteTargetArray, absAfterCert);
                        const certCmdNorm = Math.sqrt(eCmdAfter.reduce((s, v) => s + v * v, 0));
                        if (certCmdNorm > initialCmdNorm * 1.05) {
                            // Step made the primary command worse. Fall back to the
                            // standard reactive QP step instead of declaring failure.
                            if (!lastStats) lastStats = {};
                            lastStats._certLabel = 'cmd_regression';
                            certFallbackReason = 'cmd_regression';
                            acceptCertStep = false;
                        }
                    }
                    if (acceptCertStep) {
                        const certAxisGuard = this._evaluateHumanoidTranslationAxisGuard(absCurrentArray, absAfterCert, translationAxisGuard);
                        if (certAxisGuard?.violated) {
                            if (!lastStats) lastStats = {};
                            lastStats._certLabel = 'axis_guard';
                            certFallbackReason = `axis_guard_${certAxisGuard.reason}`;
                            acceptCertStep = false;
                        }
                    }
                    if (acceptCertStep) {
                        q = qNext;
                        executedIterations = iter + 1;
                        // Store cert info on lastStats for diagnostics
                        if (!lastStats) lastStats = {};
                        lastStats._certLabel = certResult.label;
                        lastStats._certH = certResult.h;
                        lastStats._certMu = certResult.mu;
                        lastStats._certSigma = certResult.sigma;
                        lastStats._cctsmW = certResult.cctsmW;
                        lastStats._cctsmWvfi = certResult.cctsmWvfi;
                        // Predictive cert always runs one effective step then exits
                        stopReason = 'single_step';
                        break;
                    }
                } else if (certResult?.label === 'unreachable') {
                    // All branches infeasible according to the predictive filter.
                    // Fall back to the standard reactive QP step before declaring the
                    // target unreachable, because the branch feasibility screen can be
                    // numerically stricter than the underlying one-step controller.
                    if (!lastStats) lastStats = {};
                    lastStats._certLabel = 'unreachable';
                    certFallbackReason = 'unreachable';
                }
                // Fall through to standard step only if cert failed to evaluate (CCTSM unavailable etc.)
            }

            const stepResult = this._solveVFICommandStep({
                Jcmd,
                eCmd,
                Jinvariant,
                eInvariant: eInvariantCurrent,
                qActive,
                activeLimits: softLimits,
                maxStep,
                damping,
                extraVFIRows: this._getWaistXVFIRows(q, armStart, activeCount),
                jointMaxSteps,
                nullManipGrad: this._computeProtectedManipulabilityGradient({
                    q,
                    armStart,
                    activeCount,
                    mode: 'absolute',
                    Jcmd,
                    Jinvariant,
                    activeLimits: hardLimits,
                    metric: resolveHumanoidJogVFIDefaultSecondaryMetric('absolute'),
                    protectCommandRows: shouldProtectCommandRowsForHumanoidVFISecondary('absolute')
                }),
                wvfiGuard: this._buildWVFILowerBoundGuard({
                    q,
                    armStart,
                    activeCount,
                    mode: 'absolute',
                    activeLimits: hardLimits,
                    includeWaist
                })
            });
            if (!stepResult) { stopReason = 'solver_fail'; break; }
            const stats = stepResult.stats;
            if (stats?.wvfiGuardAccepted === false) { stopReason = 'wvfi_lower_bound'; break; }
            if (!Array.isArray(stepResult.dq)) { stopReason = 'solver_fail'; break; }
            if (stats && certFallbackReason) {
                if (certFallbackReason === 'cmd_regression') {
                    stats._certLabel = 'cmd_regression_fallback';
                } else if (certFallbackReason.startsWith('axis_guard_')) {
                    stats._certLabel = 'axis_guard_fallback';
                } else {
                    stats._certLabel = 'reactive_fallback';
                }
            }
            if (stats) lastStats = stats;
            lastCmdNorm = cmdNorm;
            if (stats?.qpConverged === false) {
                stopReason = 'qp_not_converged';
                break;
            }
            const practicalConverged = (
                Number.isFinite(lastCmdNorm) &&
                lastCmdNorm <= practicalCmdTol &&
                Number.isFinite(lastInvariantNorm) &&
                lastInvariantNorm <= practicalInvariantTol &&
                Number.isFinite(stats?.stepNorm) &&
                stats.stepNorm <= practicalStepTol &&
                Number.isFinite(stats?.maxViolationAfter) &&
                stats.maxViolationAfter <= practicalViolationTol
            );

            const qNext = [...q];
            for (let i = 0; i < activeCount; i++) {
                let val = (q[armStart + i] || 0) + (stepResult.dq[i] || 0);
                const lim = hardLimits?.[i];
                if (lim) val = Math.max(lim.lower, Math.min(lim.upper, val));
                qNext[armStart + i] = val;
            }
            q = qNext;
            executedIterations = iter + 1;
            certFallbackReason = null;
            if (practicalConverged) {
                stopReason = 'practical_converged';
                break;
            }
        }
        if (stopReason === 'max_iter' && maxIter <= 1) {
            stopReason = 'single_step';
        }
        let finalCmdNorm = Number.NaN;
        const absEnd = this._getHumanoidAbsolutePose(q);
        if (absEnd) {
            const eCmdEnd = this._signedPoseError(dqAbsoluteTargetArray, absEnd);
            finalCmdNorm = Math.sqrt(eCmdEnd.reduce((s, v) => s + v * v, 0));
        }
        const translationGuardResult = this._evaluateHumanoidTranslationAxisGuard(absCurrentArray, absEnd, translationAxisGuard);
        if (translationGuardResult?.violated) {
            stopReason = 'command_mismatch';
            if (!lastStats) lastStats = {};
            const extra = `axis_guard=${translationGuardResult.reason}(proj=${translationGuardResult.projectedProgress.toExponential(2)},orth=${translationGuardResult.orthogonalDrift.toExponential(2)})`;
            lastStats.qpMessage = lastStats.qpMessage ? `${lastStats.qpMessage}; ${extra}` : extra;
        } else if (stopReason !== 'wvfi_lower_bound' && this._shouldFailForQPMismatch({
            stopReason,
            finalCmdNorm,
            initialCmdNorm
        }, cfg)) {
            stopReason = 'command_mismatch';
        }
        if (stopReason === 'qp_not_converged' || stopReason === 'command_mismatch' || stopReason === 'wvfi_lower_bound') {
            q = [...qInitial];
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }
        const cctsmResult_abs = this._computeModeCCTSM(
            q,
            armStart,
            activeCount,
            'absolute',
            this._buildManipulabilityVFIConstraints(q, armStart, activeCount, hardLimits)
        );
        const messageMap = {
            converged: '命令误差已收敛。',
            practical_converged: '达到实用收敛阈值。',
            single_step: 'new qpJ 单步更新完成。',
            qp_not_converged: lastStats?._certLabel === 'unreachable'
                ? '无法到达该位置（预测证书：所有分支均不可行）。'
                : '无法到达该位置（new qpJ 未收敛）。',
            command_mismatch: '无法到达该位置（目标未达成）。',
            wvfi_lower_bound: '无法到达该位置（w_vfi 下界不可行）。',
            max_iter: '达到最大迭代次数。',
            fk_fail: '姿态计算失败，提前终止。',
            jacobian_fail: '雅可比计算失败，提前终止。',
            body_task_fail: '腰部竖直约束构建失败。',
            solver_fail: 'new qpJ 求解失败。'
        };
        this._publishHumanoidJogDiagnostics({
            solverMode: 'vfi',
            modeTag: 'absolute-target',
            active: true,
            message: messageMap[stopReason] || 'new qpJ 已执行。',
            iterations: executedIterations,
            activeJoints: activeCount,
            commandRows: lastStats?.commandRows,
            invariantRows: lastStats?.invariantRows,
            constraintRows: lastStats?.constraintRows,
            invariantConstraintRows: lastStats?.invariantConstraintRows,
            equalityRows: lastStats?.equalityRows,
            componentRows: lastStats?.componentRows,
            jointLimitRows: lastStats?.jointLimitRows,
            commandError: Number.isFinite(finalCmdNorm) ? finalCmdNorm : lastCmdNorm,
            invariantError: lastInvariantNorm,
            stepNorm: lastStats?.stepNorm,
            cmdTargetNorm: lastStats?.cmdTargetNorm,
            cmdAchievedNorm: lastStats?.cmdAchievedNorm,
            cmdResidualNorm: lastStats?.cmdResidualNorm,
            cmdAlignment: lastStats?.cmdAlignment,
            cmdTargetFirst: lastStats?.cmdTargetFirst,
            cmdAchievedFirst: lastStats?.cmdAchievedFirst,
            maxViolationBefore: lastStats?.maxViolationBefore,
            maxViolationAfter: lastStats?.maxViolationAfter,
            qpIterations: lastStats?.qpIterations,
            qpConverged: lastStats?.qpConverged,
            qpObjective: lastStats?.qpObjective,
            qpMessage: lastStats?.qpMessage,
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            cctsmW: lastStats?._cctsmW !== undefined ? lastStats._cctsmW : (cctsmResult_abs?.w ?? NaN),
            cctsmWvfi: lastStats?._cctsmWvfi !== undefined ? lastStats._cctsmWvfi : (cctsmResult_abs?.w_vfi ?? NaN),
            certLabel: lastStats?._certLabel,
            certH: lastStats?._certH,
            certMu: lastStats?._certMu,
            certSigma: lastStats?._certSigma
        });
        this.lastIKSolveStatus = {
            solver: 'vfi',
            mode: 'humanoid_absolute',
            converged: stopReason === 'converged' || stopReason === 'practical_converged' || stopReason === 'single_step',
            reason: stopReason,
            message: messageMap[stopReason] || 'new qpJ 已执行。',
            finalResidual: Number.isFinite(finalCmdNorm) ? finalCmdNorm : lastCmdNorm,
            invariantResidual: lastInvariantNorm,
            qpConverged: lastStats?.qpConverged,
            qpMessage: lastStats?.qpMessage
        };

        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }
    
    /**
     * 零空间优先级控制：根据目标绝对位姿更新关节（相对位姿优先）
     * 
     * 策略：使用零空间投影的优先级任务堆叠
     * dq = J_rel^+ * κ1 * e_rel + P1 * J_abs^+ * κ2 * e_abs + P12 * κ3 * grad_limit
     * 
     * 任务1（最高优先级）：保持相对位姿不变
     * 任务2（次优先级）：绝对位姿跟踪目标
     * 任务3（最低优先级）：关节限位避免（可选）
     * 
     * @param {number[]} dqAbsoluteTargetArray - 目标绝对位姿 DQ 8元素
     * @param {number[]} currentQ - 当前全关节角
     * @param {boolean} enableJointLimitAvoidance - 是否启用关节限位避免，默认为 true
     * @param {boolean} includeWaist - 是否包含腰部关节，默认为 false（只控制双臂）。
     * @param {boolean} keepBodyZVertical - 是否使用 line_jacobian 约束 body_link 的 z 轴竖直地面。
     * @param {number[]|null} fixedRelativeTargetArray - 可选，外部指定“固定相对位姿”约束目标（8元素 DQ）。
     * @returns {boolean}
     */
    updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
        dqAbsoluteTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        keepBodyZVertical = false,
        fixedRelativeTargetArray = null,
        translationAxisGuard = null
    ) {
        const settleAsyncResult = (result, solverLabel) => {
            if (result && typeof result.then === 'function') {
                void result.catch((error) => {
                    console.error(`${solverLabel} absolute 点动异步求解失败:`, error);
                    this._setIKFailureStatus({
                        solver: solverLabel,
                        mode: 'humanoid_absolute',
                        reason: 'async_exception',
                        message: `${getHumanoidJogSolverDisplayName(solverLabel)} absolute 点动异常：${error?.message || 'unknown'}`
                    });
                });
                return true;
            }
            return result;
        };
        if (this._isHumanoidMJRestorePreWSolverEnabled()) {
            return settleAsyncResult(this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestorePreW(
                dqAbsoluteTargetArray,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray,
                translationAxisGuard
            ), 'mj_restore_pre_w');
        }
        if (this._isHumanoidMJRestoreSolverEnabled()) {
            return settleAsyncResult(this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
                dqAbsoluteTargetArray,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray,
                translationAxisGuard
            ), 'mj_restore');
        }
        if (this._isHumanoidVFISolverEnabled()) {
            return this._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstVFI(
                dqAbsoluteTargetArray,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                keepBodyZVertical,
                fixedRelativeTargetArray,
                translationAxisGuard
            );
        }
        this._publishHumanoidJogDiagnostics({
            solverMode: 'priority',
            modeTag: 'absolute-target',
            active: true,
            message: '当前为零空间投影求解器。',
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance
        });
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) {
            return this._setIKFailureStatus({ solver: 'priority', mode: 'humanoid_absolute', reason: 'invalid_state', message: '人形状态未初始化。' });
        }
        const DQ = window.DQModule?.DQWrapper;
        const Kinematics = window.DQModule?.Kinematics;
        if (!DQ) {
            return this._setIKFailureStatus({ solver: 'priority', mode: 'humanoid_absolute', reason: 'dq_unavailable', message: 'DQ 模块未加载。' });
        }

        // Debug log（节流）：用于观察“相对位姿是否发生偏移”
        // 使用方式：在控制台执行 `window.debugRelativeFirstLog = true` 开启；设为 false 关闭。
        const debugLogEnabled = window.debugRelativeFirstLog ?? true;
        const nowMs = Date.now();
        const shouldLog = debugLogEnabled && (!this._lastRelativeFirstLogTime || (nowMs - this._lastRelativeFirstLogTime) > 300);
        if (shouldLog) this._lastRelativeFirstLogTime = nowMs;

        const clamp01 = (x) => Math.max(-1, Math.min(1, x));
        const quatDotAbs = (qA, qB) => {
            // q = [w, x, y, z]
            const d = (qA[0] || 0) * (qB[0] || 0) + (qA[1] || 0) * (qB[1] || 0) + (qA[2] || 0) * (qB[2] || 0) + (qA[3] || 0) * (qB[3] || 0);
            return Math.abs(d);
        };
        const quatAngleRad = (qA, qB) => {
            const d = clamp01(quatDotAbs(qA, qB));
            return 2 * Math.acos(d);
        };
        const vecNorm = (v) => Math.sqrt((v[0] || 0) ** 2 + (v[1] || 0) ** 2 + (v[2] || 0) ** 2);
        const vecSub = (a, b) => [(a[0] || 0) - (b[0] || 0), (a[1] || 0) - (b[1] || 0), (a[2] || 0) - (b[2] || 0)];
        
        // 获取当前状态
        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relCurrentArray = this._getHumanoidRelativePose(currentQ);
        if (!absCurrentArray || !relCurrentArray) {
            return this._setIKFailureStatus({ solver: 'priority', mode: 'humanoid_absolute', reason: 'fk_unavailable', message: '当前人形位姿计算失败。' });
        }
        const relConstraintArray = (
            Array.isArray(fixedRelativeTargetArray) &&
            fixedRelativeTargetArray.length === 8 &&
            fixedRelativeTargetArray.every(v => Number.isFinite(Number(v)))
        )
            ? fixedRelativeTargetArray.map(v => Number(v))
            : relCurrentArray.slice();

        // 缓存相对位姿起点（用于末尾计算偏移量）
        const relStartArray = relConstraintArray.slice();
        
        // 目标：绝对位姿跟踪 dqAbsoluteTargetArray，同时保持相对位姿不变 (relConstraintArray)
        let absTargetArray = [...dqAbsoluteTargetArray];
        
        // 处理DQ双覆盖：选择与当前绝对位姿符号一致的目标
        let dot = 0;
        for (let i = 0; i < 8; i++) dot += absTargetArray[i] * absCurrentArray[i];
        if (dot < 0) absTargetArray = absTargetArray.map(v => -v);
        
        // 获取关节限位
        const jointLimits = this._getHumanoidJointLimits();
        
        // 获取关节索引 - 根据 includeWaist 参数决定是否包含腰部
        const n = currentQ.length;  // 全部关节数
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;  // 包含腰部则从0开始，否则跳过腰部
        const activeCount = includeWaist ? (waistCount + leftArmCount + rightArmCount) : (leftArmCount + rightArmCount);
        
        // 增益参数
        const kappa1 = 0.5;     // 任务1增益（相对位姿保持 - 最高优先级）
        const kappa2 = 0.5;     // 任务2增益（绝对位姿跟踪）
        const kappa3 = 0.3;    // 任务3基础增益（关节限位避免，实际使用自适应缩放）
        const kappaBodyZ = 0.8; // body_link z 轴竖直约束增益（line_jacobian）
        const waistGain = 1.0;  // 腰部权重（更小=腰部动得更少）
        const armGain = 1.0;    // 手臂权重
        const enableBodyVerticalTask = keepBodyZVertical && includeWaist && waistCount > 0
            && Kinematics && typeof Kinematics.lineJacobian === 'function'
            && typeof this.humanoidModel.fkmEffector === 'function'
            && typeof this.humanoidModel.poseJacobianEffector === 'function';
        if (keepBodyZVertical && includeWaist && waistCount > 0 && !enableBodyVerticalTask && !this._missingBodyZConstraintWarned) {
            console.warn('全身绝对位姿控制：未启用 body_link z 轴竖直约束（缺少 Kinematics.lineJacobian 或腰部接口）');
            this._missingBodyZConstraintWarned = true;
        }
        const tolBodyZ = 1e-14;
        const cfg = this.vfiConfig || {};
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolPose) ? cfg.hardCommandTolPose : 1e-6;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const practicalCmdTol = Number.isFinite(cfg.practicalCommandTolPose) ? cfg.practicalCommandTolPose : 1e-4;
        const practicalInvariantTol = Number.isFinite(cfg.practicalInvariantTol) ? cfg.practicalInvariantTol : 2.5e-4;
        const mismatchTol = Number.isFinite(cfg.qpMismatchTol) ? cfg.qpMismatchTol : 1e-3;
        const mismatchRequiredReduction = Number.isFinite(cfg.qpMismatchRequiredReduction) ? cfg.qpMismatchRequiredReduction : 0.8;
        const vectorNorm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
        const computeBodyVerticalResidual = (qEval) => {
            if (!enableBodyVerticalTask) return 0;
            const bodyState = this._getHumanoidBodyVerticalState(qEval, armStart, activeCount);
            return bodyState ? (bodyState.errorNorm ?? Math.abs(bodyState.error?.[0] || 0)) : Number.POSITIVE_INFINITY;
        };
        const initialCmdNorm = vectorNorm(this._signedPoseError(absTargetArray, absCurrentArray));
        
        // 迭代参数
        const maxIter = 200;
        const tol = 1e-16;
        const damping = 0.001;   // 阻尼伪逆的阻尼系数
        const maxStep = 0.003;   // 单步最大关节变化（与其他控制器保持一致，减少振荡）
        const maxNullspaceStep = maxStep * 0.3;  // 任务3单独限幅，避免抢占主任务步长
        
        const qInitial = [...currentQ];
        let q = [...qInitial];

        // 用于日志统计
        let lastERel = Number.NaN;
        let lastEAbs = Number.NaN;
        let lastEBodyZ = Number.NaN;
        let maxERel = 0;
        let maxEAbs = 0;
        let maxEBodyZ = 0;
        let iterEnd = 0;
        let converged = false;
        let stopReason = 'max_iter';
        
        for (let iter = 0; iter < maxIter; iter++) {
            iterEnd = iter;
            // === 计算任务1误差：相对位姿保持（最高优先级）===
            const relNow = this._getHumanoidRelativePose(q);
            if (!relNow) {
                stopReason = 'fk_fail';
                break;
            }
            
            // 处理DQ双覆盖
            let dotRel = 0;
            for (let i = 0; i < 8; i++) dotRel += relConstraintArray[i] * relNow[i];
            const signRel = dotRel < 0 ? -1 : 1;
            
            const e1 = [];  // 相对位姿误差
            let e1Norm2 = 0;
            for (let i = 0; i < 8; i++) {
                const err = relConstraintArray[i] - signRel * relNow[i];
                e1.push(err);
                e1Norm2 += err * err;
            }

            // === 任务1附加约束：body_link 的 z 轴保持垂直地面（世界 Z）===
            // 通过 line_jacobian 构造 Plucker line 方向误差 f=(l-l_z)^T(l-l_z)
            let JBodyZ = null;
            let eBodyZNorm2 = 0;
            if (enableBodyVerticalTask) {
                const bodyState = this._getHumanoidBodyVerticalState(q, armStart, activeCount);
                if (!bodyState) {
                    stopReason = 'body_task_fail';
                    break;
                }
                e1.push(bodyState.error[0] || 0);
                eBodyZNorm2 = (bodyState.errorNorm || 0) ** 2;
                e1Norm2 += eBodyZNorm2;
                JBodyZ = [bodyState.Jrows[0]];
            }
            
            // === 计算任务2误差：绝对位姿跟踪 ===
            const absNow = this._getHumanoidAbsolutePose(q);
            if (!absNow) {
                stopReason = 'fk_fail';
                break;
            }
            
            // 处理DQ双覆盖
            let dotAbs = 0;
            for (let i = 0; i < 8; i++) dotAbs += absTargetArray[i] * absNow[i];
            const signAbs = dotAbs < 0 ? -1 : 1;
            
            const e2 = [];  // 绝对位姿误差
            let e2Norm2 = 0;
            for (let i = 0; i < 8; i++) {
                const err = absTargetArray[i] - signAbs * absNow[i];
                e2.push(err);
                e2Norm2 += err * err;
            }
            
            // 收敛检查（主要看绝对位姿是否跟上，相对位姿由于是优先级应该一直很小）
            if (e1Norm2 < tol && e2Norm2 < tol && (!enableBodyVerticalTask || eBodyZNorm2 < tolBodyZ)) {
                console.log('零空间控制(相对优先): 迭代收敛, iter=', iter, 'e_rel=', Math.sqrt(e1Norm2).toFixed(6), 'e_abs=', Math.sqrt(e2Norm2).toFixed(6));
                converged = true;
                stopReason = 'converged';
                break;
            }

            lastERel = Math.sqrt(e1Norm2);
            lastEAbs = Math.sqrt(e2Norm2);
            lastEBodyZ = Math.sqrt(eBodyZNorm2);
            if (lastERel > maxERel) maxERel = lastERel;
            if (lastEAbs > maxEAbs) maxEAbs = lastEAbs;
            if (lastEBodyZ > maxEBodyZ) maxEBodyZ = lastEBodyZ;
            
            if (iter === 0 || iter === maxIter - 1) {
                console.log('零空间控制(相对优先): iter=', iter, 'e_rel=', Math.sqrt(e1Norm2).toFixed(6), 'e_abs=', Math.sqrt(e2Norm2).toFixed(6), 'e_body_z=', Math.sqrt(eBodyZNorm2).toFixed(6));
            }
            
            // === 计算雅可比矩阵 ===
            // J_rel: 相对位姿对活动关节的雅可比 (8 x activeCount)
            const JrelFull = this._getHumanoidRelativePoseJacobian(q);
            if (!JrelFull) {
                stopReason = 'jacobian_fail';
                break;
            }
            const J1 = [];  // 任务1的雅可比（相对位姿）
            for (let i = 0; i < 8; i++) {
                J1.push(JrelFull[i].slice(armStart, armStart + activeCount));
            }
            if (JBodyZ) {
                J1.push(JBodyZ[0]);
            }
            
            // J_abs: 绝对位姿对活动关节的雅可比 (8 x activeCount)
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(q);
            if (!JabsFull) {
                stopReason = 'jacobian_fail';
                break;
            }
            const J2 = [];  // 任务2的雅可比（绝对位姿）
            for (let i = 0; i < 8; i++) {
                J2.push(JabsFull[i].slice(armStart, armStart + activeCount));
            }
            
            // === 计算关节权重矩阵 W (对角阵，用数组表示) ===
            const W = [];
            for (let i = 0; i < activeCount; i++) {
                const isWaistJoint = includeWaist && i < waistCount;
                W.push(isWaistJoint ? waistGain : armGain);
            }
            
            // === 计算任务1（相对位姿）的加权伪逆和零空间投影 ===
            // J1_pinv = W^-1 * J1^T * (J1 * W^-1 * J1^T + λI)^-1
            const J1Pinv = this._weightedDampedPseudoInverse(J1, W, damping);
            if (!J1Pinv) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            // P1 = I - J1_pinv * J1 (相对位姿任务的零空间投影矩阵)
            const P1 = this._computeNullspaceProjection(J1Pinv, J1, activeCount);
            
            // === 计算任务2在任务1零空间中的伪逆 ===
            // J2_bar = J2 * P1
            const J2Bar = this._matMul(J2, P1);
            const J2BarPinv = this._weightedDampedPseudoInverse(J2Bar, W, damping);
            if (!J2BarPinv) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            // === 计算增广雅可比的零空间投影 P12 ===
            // P12 = P1 - J2Bar_pinv * J2Bar
            const P12 = this._computeNullspaceProjection2(P1, J2BarPinv, J2Bar, activeCount);
            
            // === 计算任务3：关节限位避免梯度（只计算双臂）===
            const armLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
            const gradLimit = this._computeJointLimitGradient(q.slice(armStart, armStart + activeCount), armLimits, activeCount);
            
            // === 计算关节速度增量 ===
            // dq = J1^+ * κ1 * e1 + P1 * J2Bar^+ * κ2 * e2 + P12 * κ3 * gradLimit
            const dq = new Array(activeCount).fill(0);
            
            // 任务1贡献：J1^+ * κ1 * e1 （相对位姿保持 - 最高优先级）
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < e1.length; j++) {
                    const gain = (enableBodyVerticalTask && j >= 8) ? kappaBodyZ : kappa1;
                    dq[i] += J1Pinv[i][j] * gain * e1[j];
                }
            }
            
            // 任务2贡献：P1 * J2Bar^+ * κ2 * e2 （绝对位姿跟踪 - 在相对位姿零空间内）
            // 先计算 J2Bar^+ * e2
            const dq2_temp = new Array(activeCount).fill(0);
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < 8; j++) {
                    dq2_temp[i] += J2BarPinv[i][j] * kappa2 * e2[j];
                }
            }
            // 再乘以 P1 投影到相对位姿的零空间
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < activeCount; j++) {
                    dq[i] += P1[i][j] * dq2_temp[j];
                }
            }
            
            // 任务3贡献：P12 * κ3 * gradLimit（仅在启用关节限位避免时）
            // 阶段策略：前15次迭代启用任务3，后续迭代任务3归零
            const kappa3ActiveIters = 15;
            if (enableJointLimitAvoidance && iter < kappa3ActiveIters) {
                const primaryErr = Math.sqrt(e1Norm2 + e2Norm2);
                const kappa3Eff = kappa3 / (1 + 20 * primaryErr);
                const dq3 = new Array(activeCount).fill(0);
                for (let i = 0; i < activeCount; i++) {
                    for (let j = 0; j < activeCount; j++) {
                        dq3[i] += P12[i][j] * kappa3Eff * gradLimit[j];
                    }
                }
                
                // 对任务3单独限幅，避免其主导每步更新
                let dq3Inf = 0;
                for (let i = 0; i < activeCount; i++) {
                    const a = Math.abs(dq3[i]);
                    if (a > dq3Inf) dq3Inf = a;
                }
                const dq3Scale = dq3Inf > maxNullspaceStep ? (maxNullspaceStep / dq3Inf) : 1;
                for (let i = 0; i < activeCount; i++) {
                    dq[i] += dq3[i] * dq3Scale;
                }
            }
            
            // 应用关节权重并限制步长
            for (let i = 0; i < activeCount; i++) {
                let step = dq[i] * W[i];
                step = Math.max(-maxStep, Math.min(maxStep, step));
                q[armStart + i] += step;  // 应用到双臂关节（跳过腰部）
            }
            
            // 应用关节限位（硬限位始终生效）
            if (armLimits) {
                for (let i = 0; i < activeCount && i < armLimits.length; i++) {
                    const lim = armLimits[i];
                    if (lim) {
                        q[armStart + i] = Math.max(lim.lower, Math.min(lim.upper, q[armStart + i]));
                    }
                }
            }
        }

        // 末尾：打印“相对位姿偏移”摘要（平移偏移、旋转偏移、误差范数）
        if (shouldLog) {
            try {
                const relEndArrayRaw = this._getHumanoidRelativePose(q);
                const absEndArrayRaw = this._getHumanoidAbsolutePose(q);

                // 相对位姿：对齐双覆盖
                let relEndArray = relEndArrayRaw ? relEndArrayRaw.slice() : null;
                if (relEndArray) {
                    let dotRel = 0;
                    for (let i = 0; i < 8; i++) dotRel += relStartArray[i] * relEndArray[i];
                    if (dotRel < 0) relEndArray = relEndArray.map(v => -v);
                }

                // 绝对位姿：对齐双覆盖
                let absEndArray = absEndArrayRaw ? absEndArrayRaw.slice() : null;
                if (absEndArray) {
                    let dotAbs = 0;
                    for (let i = 0; i < 8; i++) dotAbs += absTargetArray[i] * absEndArray[i];
                    if (dotAbs < 0) absEndArray = absEndArray.map(v => -v);
                }

                let relDt = NaN;
                let relDAngDeg = NaN;
                if (relEndArray) {
                    const dqRel0 = DQ.createFromArray(relStartArray);
                    const dqRel1 = DQ.createFromArray(relEndArray);
                    const t0 = DQ.getTranslation(dqRel0);
                    const t1 = DQ.getTranslation(dqRel1);
                    const r0 = DQ.getRotation(dqRel0);
                    const r1 = DQ.getRotation(dqRel1);
                    relDt = vecNorm(vecSub(t1, t0));
                    relDAngDeg = (quatAngleRad(r0, r1) * 180) / Math.PI;
                }

                let absErrT = NaN;
                let absErrAngDeg = NaN;
                if (absEndArray) {
                    const dqAbsT = DQ.createFromArray(absTargetArray);
                    const dqAbsE = DQ.createFromArray(absEndArray);
                    const tT = DQ.getTranslation(dqAbsT);
                    const tE = DQ.getTranslation(dqAbsE);
                    const rT = DQ.getRotation(dqAbsT);
                    const rE = DQ.getRotation(dqAbsE);
                    absErrT = vecNorm(vecSub(tE, tT));
                    absErrAngDeg = (quatAngleRad(rT, rE) * 180) / Math.PI;
                }

                // 打印相对位姿四元数的起始和结束值，便于对比
                const r0Str = relStartArray ? `[${relStartArray.slice(0,4).map(v=>v.toFixed(4)).join(',')}]` : 'n/a';
                const r1Str = relEndArray ? `[${relEndArray.slice(0,4).map(v=>v.toFixed(4)).join(',')}]` : 'n/a';
                
                console.log('[RelFirst] iter=', iterEnd,
                    'converged=', converged,
                    'e_rel(last/max)=', (Number.isFinite(lastERel) ? lastERel.toExponential(3) : 'n/a'), '/', maxERel.toExponential(3),
                    'e_abs(last/max)=', (Number.isFinite(lastEAbs) ? lastEAbs.toExponential(3) : 'n/a'), '/', maxEAbs.toExponential(3),
                    'e_body_z(last/max)=', (Number.isFinite(lastEBodyZ) ? lastEBodyZ.toExponential(3) : 'n/a'), '/', maxEBodyZ.toExponential(3),
                    'relΔt(m)=', Number.isFinite(relDt) ? relDt.toExponential(3) : 'n/a',
                    'relΔang(deg)=', Number.isFinite(relDAngDeg) ? relDAngDeg.toFixed(3) : 'n/a',
                    'absErrT(m)=', Number.isFinite(absErrT) ? absErrT.toExponential(3) : 'n/a',
                    'absErrAng(deg)=', Number.isFinite(absErrAngDeg) ? absErrAngDeg.toFixed(3) : 'n/a'
                );
                console.log('[RelFirst] relQuat: start=', r0Str, 'end=', r1Str);
            } catch (e) {
                console.warn('[RelFirst] debug log failed:', e);
            }
        }
        const relEndArray = this._getHumanoidRelativePose(q);
        const absEndArray = this._getHumanoidAbsolutePose(q);
        const finalCmdNorm = absEndArray ? vectorNorm(this._signedPoseError(absTargetArray, absEndArray)) : Number.POSITIVE_INFINITY;
        let finalInvariantNorm = relEndArray ? vectorNorm(this._signedPoseError(relConstraintArray, relEndArray)) : Number.POSITIVE_INFINITY;
        if (enableBodyVerticalTask) {
            const bodyResidual = computeBodyVerticalResidual(q);
            finalInvariantNorm = Math.sqrt(finalInvariantNorm * finalInvariantNorm + bodyResidual * bodyResidual);
        }
        const translationGuardResult = this._evaluateHumanoidTranslationAxisGuard(absCurrentArray, absEndArray, translationAxisGuard);
        if (translationGuardResult?.violated) {
            stopReason = 'command_mismatch';
        }
        const outcome = evaluateHumanoidJogCommandOutcome({
            stopReason,
            initialCommandNorm: initialCmdNorm,
            finalCommandNorm: finalCmdNorm,
            finalInvariantNorm,
            hardCommandTol: hardCmdTol,
            hardInvariantTol: hardInvariantTol,
            practicalCommandTol: practicalCmdTol,
            practicalInvariantTol: practicalInvariantTol,
            mismatchTol,
            mismatchRequiredReduction,
            singleStepMismatchRequiredReduction: Number.isFinite(cfg.qpSingleStepMismatchRequiredReduction)
                ? Math.min(0.99, Math.max(0, cfg.qpSingleStepMismatchRequiredReduction))
                : 0.05
        });
        const messageMap = {
            converged: '命令误差已收敛。',
            practical_converged: '达到实用收敛阈值。',
            command_mismatch: '无法到达该位置（目标未达成）。',
            max_iter: '达到最大迭代次数。',
            fk_fail: '姿态计算失败，提前终止。',
            jacobian_fail: '雅可比计算失败，提前终止。',
            body_task_fail: '腰部竖直约束构建失败。'
        };
        const qApplied = outcome.applyResult ? q : qInitial;
        if (outcome.reason === 'command_mismatch') {
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }
        this.lastIKSolveStatus = {
            solver: 'priority',
            mode: 'humanoid_absolute',
            converged: outcome.converged,
            reason: outcome.reason,
            message: messageMap[outcome.reason] || '零空间投影 已执行。',
            finalResidual: finalCmdNorm,
            invariantResidual: finalInvariantNorm
        };
        
        // 应用结果
        for (let i = 0; i < window.humanoidQ.length && i < qApplied.length; i++) {
            window.humanoidQ[i] = qApplied[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        this._publishCurrentPriorityCCTSMDiagnostics({
            modeTag: 'absolute-target',
            mode: 'absolute',
            q: Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : qApplied,
            armStart,
            activeCount,
            activeLimits: jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null,
            includeWaist,
            keepBodyZVertical,
            jointLimitAvoidance: enableJointLimitAvoidance,
            message: this.lastIKSolveStatus.message
        });
        return outcome.converged;
    }
    
    /**
     * 零空间优先级控制：从相对位姿目标更新人形机器人关节（绝对位姿优先）
     * 优先级：绝对位姿保持（最高）> 相对位姿跟踪 > 关节限位避免
     * 
     * @param {number[]} dqRelativeTargetArray - 目标相对位姿 DQ 8元素
     * @param {number[]} currentQ - 当前全关节角
     * @param {boolean} enableJointLimitAvoidance - 是否启用关节限位避免
     * @param {boolean} includeWaist - 是否包含腰部关节，默认为 false（只控制双臂）
     * @returns {boolean}
     */
    _updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstVFI(
        dqRelativeTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        translationAxisGuard = null
    ) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_relative', reason: 'invalid_state', message: '人形状态未初始化。' });
        }
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_relative', reason: 'dq_unavailable', message: 'DQ 模块未加载。' });
        }

        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relCurrentArray = this._getHumanoidRelativePose(currentQ);
        if (!absCurrentArray || !relCurrentArray) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_relative', reason: 'fk_unavailable', message: '当前人形位姿计算失败。' });
        }

        const jointLimits = this._getHumanoidJointLimits();
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;
        const activeCount = includeWaist ? (waistCount + leftArmCount + rightArmCount) : (leftArmCount + rightArmCount);
        if (activeCount <= 0) {
            return this._setIKFailureStatus({ solver: 'vfi', mode: 'humanoid_relative', reason: 'no_active_joints', message: '当前模式下没有可用活动关节。' });
        }

        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const softLimits = enableJointLimitAvoidance ? hardLimits : null;
        const cfg = this.vfiConfig || {};
        const maxIter = Number.isFinite(cfg.qpOuterMaxIter) ? cfg.qpOuterMaxIter : 16;
        const damping = Number.isFinite(cfg.damping) ? cfg.damping : 1e-3;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.003;
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolPose) ? cfg.hardCommandTolPose : 1e-6;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const practicalCmdTol = Number.isFinite(cfg.practicalCommandTolPose) ? cfg.practicalCommandTolPose : 1e-4;
        const practicalInvariantTol = Number.isFinite(cfg.practicalInvariantTol) ? cfg.practicalInvariantTol : 2.5e-4;
        const practicalStepTol = Number.isFinite(cfg.practicalStepNormTol) ? cfg.practicalStepNormTol : 2e-5;
        const practicalViolationTol = Number.isFinite(cfg.practicalViolationTol) ? cfg.practicalViolationTol : 1e-8;

        const qInitial = [...currentQ];
        let q = [...qInitial];
        let executedIterations = 0;
        let stopReason = 'max_iter';
        let initialCmdNorm = Number.NaN;
        let lastCmdNorm = Number.NaN;
        let lastInvariantNorm = Number.NaN;
        let lastStats = null;
        for (let iter = 0; iter < maxIter; iter++) {
            const absNow = this._getHumanoidAbsolutePose(q);
            const relNow = this._getHumanoidRelativePose(q);
            if (!absNow || !relNow) {
                stopReason = 'fk_fail';
                break;
            }

            const eInvariant = this._signedPoseError(absCurrentArray, absNow);
            const eCmd = this._signedPoseError(dqRelativeTargetArray, relNow);
            const cmdNorm = Math.sqrt(eCmd.reduce((s, v) => s + v * v, 0));
            lastCmdNorm = cmdNorm;
            if (!Number.isFinite(initialCmdNorm)) initialCmdNorm = cmdNorm;

            lastInvariantNorm = Math.sqrt(eInvariant.reduce((s, v) => s + v * v, 0));
            if (cmdNorm < hardCmdTol && lastInvariantNorm < hardInvariantTol) {
                stopReason = 'converged';
                break;
            }

            const JabsFull = this._getHumanoidAbsolutePoseJacobian(q);
            const JrelFull = this._getHumanoidRelativePoseJacobian(q);
            if (!JabsFull || !JrelFull) { stopReason = 'solver_fail'; break; }
            const Jinvariant = JabsFull.map((row) => row.slice(armStart, armStart + activeCount));
            const Jcmd = JrelFull.map((row) => row.slice(armStart, armStart + activeCount));
            const qActive = q.slice(armStart, armStart + activeCount);
            const stepResult = this._solveVFICommandStep({
                Jcmd,
                eCmd,
                Jinvariant,
                eInvariant,
                qActive,
                activeLimits: softLimits,
                maxStep,
                damping,
                extraVFIRows: this._getWaistXVFIRows(q, armStart, activeCount),
                nullManipGrad: this._computeProtectedManipulabilityGradient({
                    q,
                    armStart,
                    activeCount,
                    mode: 'relative',
                    Jcmd,
                    Jinvariant,
                    activeLimits: hardLimits
                }),
                wvfiGuard: this._buildWVFILowerBoundGuard({
                    q,
                    armStart,
                    activeCount,
                    mode: 'relative',
                    activeLimits: hardLimits,
                    includeWaist
                })
            });
            if (!stepResult) { stopReason = 'solver_fail'; break; }
            const stats = stepResult.stats;
            if (stats?.wvfiGuardAccepted === false) { stopReason = 'wvfi_lower_bound'; break; }
            if (!Array.isArray(stepResult.dq)) { stopReason = 'solver_fail'; break; }
            if (stats) lastStats = stats;
            lastCmdNorm = cmdNorm;
            if (stats?.qpConverged === false) {
                stopReason = 'qp_not_converged';
                break;
            }
            const practicalConverged = (
                Number.isFinite(lastCmdNorm) &&
                lastCmdNorm <= practicalCmdTol &&
                Number.isFinite(lastInvariantNorm) &&
                lastInvariantNorm <= practicalInvariantTol &&
                Number.isFinite(stats?.stepNorm) &&
                stats.stepNorm <= practicalStepTol &&
                Number.isFinite(stats?.maxViolationAfter) &&
                stats.maxViolationAfter <= practicalViolationTol
            );

            const qNext = [...q];
            for (let i = 0; i < activeCount; i++) {
                let val = (q[armStart + i] || 0) + (stepResult.dq[i] || 0);
                const lim = hardLimits?.[i];
                if (lim) val = Math.max(lim.lower, Math.min(lim.upper, val));
                qNext[armStart + i] = val;
            }
            q = qNext;
            executedIterations = iter + 1;
            if (practicalConverged) {
                stopReason = 'practical_converged';
                break;
            }
        }
        if (stopReason === 'max_iter' && maxIter <= 1) {
            stopReason = 'single_step';
        }
        let finalCmdNorm = Number.NaN;
        const relEnd = this._getHumanoidRelativePose(q);
        if (relEnd) {
            const eCmdEnd = this._signedPoseError(dqRelativeTargetArray, relEnd);
            finalCmdNorm = Math.sqrt(eCmdEnd.reduce((s, v) => s + v * v, 0));
        }
        const translationGuardResult = this._evaluateHumanoidTranslationAxisGuard(relCurrentArray, relEnd, translationAxisGuard);
        if (translationGuardResult?.violated) {
            stopReason = 'command_mismatch';
            if (!lastStats) lastStats = {};
            const extra = `axis_guard=${translationGuardResult.reason}(proj=${translationGuardResult.projectedProgress.toExponential(2)},orth=${translationGuardResult.orthogonalDrift.toExponential(2)})`;
            lastStats.qpMessage = lastStats.qpMessage ? `${lastStats.qpMessage}; ${extra}` : extra;
        } else if (stopReason !== 'wvfi_lower_bound' && this._shouldFailForQPMismatch({
            stopReason,
            finalCmdNorm,
            initialCmdNorm
        }, cfg)) {
            stopReason = 'command_mismatch';
        }
        if (stopReason === 'qp_not_converged' || stopReason === 'command_mismatch' || stopReason === 'wvfi_lower_bound') {
            q = [...qInitial];
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }
        const cctsmResult_rel = this._computeModeCCTSM(
            q,
            armStart,
            activeCount,
            'relative',
            this._buildManipulabilityVFIConstraints(q, armStart, activeCount, hardLimits)
        );
        const messageMap = {
            converged: '命令误差已收敛。',
            practical_converged: '达到实用收敛阈值。',
            single_step: 'new qpJ 单步更新完成。',
            qp_not_converged: '无法到达该位置（new qpJ 未收敛）。',
            command_mismatch: '无法到达该位置（目标未达成）。',
            wvfi_lower_bound: '无法到达该位置（w_vfi 下界不可行）。',
            max_iter: '达到最大迭代次数。',
            fk_fail: '姿态计算失败，提前终止。',
            jacobian_fail: '雅可比计算失败，提前终止。',
            solver_fail: 'new qpJ 求解失败。'
        };
        this._publishHumanoidJogDiagnostics({
            solverMode: 'vfi',
            modeTag: 'relative-target',
            active: true,
            message: messageMap[stopReason] || 'new qpJ 已执行。',
            iterations: executedIterations,
            activeJoints: activeCount,
            commandRows: lastStats?.commandRows,
            invariantRows: lastStats?.invariantRows,
            constraintRows: lastStats?.constraintRows,
            invariantConstraintRows: lastStats?.invariantConstraintRows,
            equalityRows: lastStats?.equalityRows,
            componentRows: lastStats?.componentRows,
            jointLimitRows: lastStats?.jointLimitRows,
            commandError: Number.isFinite(finalCmdNorm) ? finalCmdNorm : lastCmdNorm,
            invariantError: lastInvariantNorm,
            stepNorm: lastStats?.stepNorm,
            cmdTargetNorm: lastStats?.cmdTargetNorm,
            cmdAchievedNorm: lastStats?.cmdAchievedNorm,
            cmdResidualNorm: lastStats?.cmdResidualNorm,
            cmdAlignment: lastStats?.cmdAlignment,
            cmdTargetFirst: lastStats?.cmdTargetFirst,
            cmdAchievedFirst: lastStats?.cmdAchievedFirst,
            maxViolationBefore: lastStats?.maxViolationBefore,
            maxViolationAfter: lastStats?.maxViolationAfter,
            qpIterations: lastStats?.qpIterations,
            qpConverged: lastStats?.qpConverged,
            qpObjective: lastStats?.qpObjective,
            qpMessage: lastStats?.qpMessage,
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance,
            cctsmW: cctsmResult_rel?.w ?? NaN,
            cctsmWvfi: cctsmResult_rel?.w_vfi ?? NaN
        });
        this.lastIKSolveStatus = {
            solver: 'vfi',
            mode: 'humanoid_relative',
            converged: stopReason === 'converged' || stopReason === 'practical_converged' || stopReason === 'single_step',
            reason: stopReason,
            message: messageMap[stopReason] || 'new qpJ 已执行。',
            finalResidual: Number.isFinite(finalCmdNorm) ? finalCmdNorm : lastCmdNorm,
            invariantResidual: lastInvariantNorm,
            qpConverged: lastStats?.qpConverged,
            qpMessage: lastStats?.qpMessage
        };

        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }

    updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(
        dqRelativeTargetArray,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false,
        translationAxisGuard = null
    ) {
        if (this._isHumanoidMJRestoreSolverEnabled()) {
            const fallbackMessage = 'MuJoCo 风格恢复 当前仅支持 absolute-target，其它模式已回退到 new qpJ。';
            const ok = this._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstVFI(
                dqRelativeTargetArray,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                translationAxisGuard
            );
            const payload = this.lastHumanoidJogDiagnostics
                ? {
                    ...this.lastHumanoidJogDiagnostics,
                    solverMode: 'mj_restore',
                    modeTag: 'relative-target',
                    message: fallbackMessage,
                    nextMode: getHumanoidJogSolverDisplayName('vfi'),
                    diagnosticLabel: 'fallback_to_new_qpJ'
                }
                : {
                    solverMode: 'mj_restore',
                    modeTag: 'relative-target',
                    active: true,
                    message: fallbackMessage,
                    nextMode: getHumanoidJogSolverDisplayName('vfi')
                };
            this._publishHumanoidJogDiagnostics(payload);
            return ok;
        }
        if (this._isHumanoidVFISolverEnabled()) {
            return this._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstVFI(
                dqRelativeTargetArray,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist,
                translationAxisGuard
            );
        }
        this._publishHumanoidJogDiagnostics({
            solverMode: 'priority',
            modeTag: 'relative-target',
            active: true,
            message: '当前为零空间投影求解器。',
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance
        });
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) {
            return this._setIKFailureStatus({ solver: 'priority', mode: 'humanoid_relative', reason: 'invalid_state', message: '人形状态未初始化。' });
        }
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) {
            return this._setIKFailureStatus({ solver: 'priority', mode: 'humanoid_relative', reason: 'dq_unavailable', message: 'DQ 模块未加载。' });
        }

        // Debug log（节流）
        const debugLogEnabled = window.debugAbsoluteFirstLog ?? false;
        const nowMs = Date.now();
        const shouldLog = debugLogEnabled && (!this._lastAbsoluteFirstLogTime || (nowMs - this._lastAbsoluteFirstLogTime) > 300);
        if (shouldLog) this._lastAbsoluteFirstLogTime = nowMs;

        const clamp01 = (x) => Math.max(-1, Math.min(1, x));
        const quatDotAbs = (qA, qB) => {
            const d = (qA[0] || 0) * (qB[0] || 0) + (qA[1] || 0) * (qB[1] || 0) + (qA[2] || 0) * (qB[2] || 0) + (qA[3] || 0) * (qB[3] || 0);
            return Math.abs(d);
        };
        const quatAngleRad = (qA, qB) => {
            const d = clamp01(quatDotAbs(qA, qB));
            return 2 * Math.acos(d);
        };
        const vecNorm = (v) => Math.sqrt((v[0] || 0) ** 2 + (v[1] || 0) ** 2 + (v[2] || 0) ** 2);
        const vecSub = (a, b) => [(a[0] || 0) - (b[0] || 0), (a[1] || 0) - (b[1] || 0), (a[2] || 0) - (b[2] || 0)];
        
        // 获取当前状态
        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relCurrentArray = this._getHumanoidRelativePose(currentQ);
        if (!absCurrentArray || !relCurrentArray) {
            return this._setIKFailureStatus({ solver: 'priority', mode: 'humanoid_relative', reason: 'fk_unavailable', message: '当前人形位姿计算失败。' });
        }

        // 缓存绝对位姿起点（用于末尾计算偏移量）
        const absStartArray = absCurrentArray.slice();
        
        // 目标：相对位姿跟踪 dqRelativeTargetArray，同时保持绝对位姿不变 (absCurrentArray)
        let relTargetArray = [...dqRelativeTargetArray];
        
        // 处理DQ双覆盖：选择与当前相对位姿符号一致的目标
        let dot = 0;
        for (let i = 0; i < 8; i++) dot += relTargetArray[i] * relCurrentArray[i];
        if (dot < 0) relTargetArray = relTargetArray.map(v => -v);
        
        // 获取关节限位
        const jointLimits = this._getHumanoidJointLimits();
        
        // 获取关节索引 - 根据 includeWaist 参数决定是否包含腰部
        const n = currentQ.length;
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;  // 包含腰部则从0开始，否则跳过腰部
        const activeCount = includeWaist ? (waistCount + leftArmCount + rightArmCount) : (leftArmCount + rightArmCount);
        
        // 增益参数
        const kappa1 = 0.2;     // 任务1增益（绝对位姿保持 - 最高优先级）
        const kappa2 = 0.4;     // 任务2增益（相对位姿跟踪）
        const kappa3 = 0.00;    // 任务3增益（关节限位避免）
        const waistGain = 0.5;  // 腰部权重（更小=腰部动得更少）
        const armGain = 1.0;    // 手臂权重
        const cfg = this.vfiConfig || {};
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolPose) ? cfg.hardCommandTolPose : 1e-6;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const practicalCmdTol = Number.isFinite(cfg.practicalCommandTolPose) ? cfg.practicalCommandTolPose : 1e-4;
        const practicalInvariantTol = Number.isFinite(cfg.practicalInvariantTol) ? cfg.practicalInvariantTol : 2.5e-4;
        const mismatchTol = Number.isFinite(cfg.qpMismatchTol) ? cfg.qpMismatchTol : 1e-3;
        const mismatchRequiredReduction = Number.isFinite(cfg.qpMismatchRequiredReduction) ? cfg.qpMismatchRequiredReduction : 0.8;
        const vectorNorm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
        const initialCmdNorm = vectorNorm(this._signedPoseError(relTargetArray, relCurrentArray));
        
        // 迭代参数
        const maxIter = 200;
        const tol = 1e-16;
        const damping = 0.00001;
        const maxStep = 0.0003;
        
        const qInitial = [...currentQ];
        let q = [...qInitial];

        // 用于日志统计
        let lastEAbs = Number.NaN;
        let lastERel = Number.NaN;
        let maxEAbs = 0;
        let maxERel = 0;
        let iterEnd = 0;
        let converged = false;
        let stopReason = 'max_iter';
        
        for (let iter = 0; iter < maxIter; iter++) {
            iterEnd = iter;
            // === 计算任务1误差：绝对位姿保持（最高优先级）===
            const absNow = this._getHumanoidAbsolutePose(q);
            if (!absNow) {
                stopReason = 'fk_fail';
                break;
            }
            
            // 处理DQ双覆盖
            let dotAbs = 0;
            for (let i = 0; i < 8; i++) dotAbs += absCurrentArray[i] * absNow[i];
            const signAbs = dotAbs < 0 ? -1 : 1;
            
            const e1 = [];  // 绝对位姿误差
            let e1Norm2 = 0;
            for (let i = 0; i < 8; i++) {
                const err = absCurrentArray[i] - signAbs * absNow[i];
                e1.push(err);
                e1Norm2 += err * err;
            }
            
            // === 计算任务2误差：相对位姿跟踪 ===
            const relNow = this._getHumanoidRelativePose(q);
            if (!relNow) {
                stopReason = 'fk_fail';
                break;
            }
            
            // 处理DQ双覆盖
            let dotRel = 0;
            for (let i = 0; i < 8; i++) dotRel += relTargetArray[i] * relNow[i];
            const signRel = dotRel < 0 ? -1 : 1;
            
            const e2 = [];  // 相对位姿误差
            let e2Norm2 = 0;
            for (let i = 0; i < 8; i++) {
                const err = relTargetArray[i] - signRel * relNow[i];
                e2.push(err);
                e2Norm2 += err * err;
            }
            
            // 收敛检查
            if (e1Norm2 < tol && e2Norm2 < tol) {
                console.log('零空间控制(绝对优先): 迭代收敛, iter=', iter, 'e_abs=', Math.sqrt(e1Norm2).toFixed(6), 'e_rel=', Math.sqrt(e2Norm2).toFixed(6));
                converged = true;
                stopReason = 'converged';
                break;
            }

            lastEAbs = Math.sqrt(e1Norm2);
            lastERel = Math.sqrt(e2Norm2);
            if (lastEAbs > maxEAbs) maxEAbs = lastEAbs;
            if (lastERel > maxERel) maxERel = lastERel;
            
            if (iter === 0 || iter === maxIter - 1) {
                console.log('零空间控制(绝对优先): iter=', iter, 'e_abs=', Math.sqrt(e1Norm2).toFixed(6), 'e_rel=', Math.sqrt(e2Norm2).toFixed(6));
            }
            
            // === 计算雅可比矩阵 ===
            // J_abs: 绝对位姿对关节的雅可比 (8 x activeCount)
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(q);
            if (!JabsFull) {
                stopReason = 'jacobian_fail';
                break;
            }
            const J1 = [];  // 任务1的雅可比（绝对位姿）
            for (let i = 0; i < 8; i++) {
                J1.push(JabsFull[i].slice(armStart, armStart + activeCount));
            }
            
            // J_rel: 相对位姿对关节的雅可比 (8 x activeCount)
            const JrelFull = this._getHumanoidRelativePoseJacobian(q);
            if (!JrelFull) {
                stopReason = 'jacobian_fail';
                break;
            }
            const J2 = [];  // 任务2的雅可比（相对位姿）
            for (let i = 0; i < 8; i++) {
                J2.push(JrelFull[i].slice(armStart, armStart + activeCount));
            }
            
            // === 计算关节权重矩阵 W (对角阵，用数组表示) ===
            const W = [];
            for (let i = 0; i < activeCount; i++) {
                const isWaistJoint = includeWaist && i < waistCount;
                W.push(isWaistJoint ? waistGain : armGain);
            }
            
            // === 计算任务1（绝对位姿）的加权伪逆和零空间投影 ===
            const J1Pinv = this._weightedDampedPseudoInverse(J1, W, damping);
            if (!J1Pinv) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            // P1 = I - J1_pinv * J1 (绝对位姿任务的零空间投影矩阵)
            const P1 = this._computeNullspaceProjection(J1Pinv, J1, activeCount);
            
            // === 计算任务2在任务1零空间中的伪逆 ===
            // J2_bar = J2 * P1
            const J2Bar = this._matMul(J2, P1);
            const J2BarPinv = this._weightedDampedPseudoInverse(J2Bar, W, damping);
            if (!J2BarPinv) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            // === 计算增广雅可比的零空间投影 P12 ===
            const P12 = this._computeNullspaceProjection2(P1, J2BarPinv, J2Bar, activeCount);
            
            // === 计算任务3：关节限位避免梯度 ===
            const activeLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
            const gradLimit = this._computeJointLimitGradient(q.slice(armStart, armStart + activeCount), activeLimits, activeCount);
            
            // === 计算关节速度增量 ===
            // dq = J1^+ * κ1 * e1 + P1 * J2Bar^+ * κ2 * e2 + P12 * κ3 * gradLimit
            const dq = new Array(activeCount).fill(0);
            
            // 任务1贡献：J1^+ * κ1 * e1 （绝对位姿保持 - 最高优先级）
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < 8; j++) {
                    dq[i] += J1Pinv[i][j] * kappa1 * e1[j];
                }
            }
            
            // 任务2贡献：P1 * J2Bar^+ * κ2 * e2 （相对位姿跟踪 - 在绝对位姿零空间内）
            const dq2_temp = new Array(activeCount).fill(0);
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < 8; j++) {
                    dq2_temp[i] += J2BarPinv[i][j] * kappa2 * e2[j];
                }
            }
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < activeCount; j++) {
                    dq[i] += P1[i][j] * dq2_temp[j];
                }
            }
            
            // 任务3贡献：P12 * κ3 * gradLimit（仅在启用关节限位避免时）
            if (enableJointLimitAvoidance) {
                for (let i = 0; i < activeCount; i++) {
                    for (let j = 0; j < activeCount; j++) {
                        dq[i] += P12[i][j] * kappa3 * gradLimit[j];
                    }
                }
            }
            
            // 应用关节权重并限制步长
            for (let i = 0; i < activeCount; i++) {
                let step = dq[i] * W[i];
                step = Math.max(-maxStep, Math.min(maxStep, step));
                q[armStart + i] += step;
            }
            
            // 应用关节限位（硬限位始终生效）
            if (activeLimits) {
                for (let i = 0; i < activeCount && i < activeLimits.length; i++) {
                    const lim = activeLimits[i];
                    if (lim) {
                        q[armStart + i] = Math.max(lim.lower, Math.min(lim.upper, q[armStart + i]));
                    }
                }
            }
        }

        // 末尾：打印摘要日志
        if (shouldLog) {
            try {
                const absEndArrayRaw = this._getHumanoidAbsolutePose(q);
                const relEndArrayRaw = this._getHumanoidRelativePose(q);

                let absEndArray = absEndArrayRaw ? absEndArrayRaw.slice() : null;
                if (absEndArray) {
                    let dotAbs = 0;
                    for (let i = 0; i < 8; i++) dotAbs += absStartArray[i] * absEndArray[i];
                    if (dotAbs < 0) absEndArray = absEndArray.map(v => -v);
                }

                let relEndArray = relEndArrayRaw ? relEndArrayRaw.slice() : null;
                if (relEndArray) {
                    let dotRel = 0;
                    for (let i = 0; i < 8; i++) dotRel += relTargetArray[i] * relEndArray[i];
                    if (dotRel < 0) relEndArray = relEndArray.map(v => -v);
                }

                let absDt = NaN;
                let absDAngDeg = NaN;
                if (absEndArray) {
                    const dqAbs0 = DQ.createFromArray(absStartArray);
                    const dqAbs1 = DQ.createFromArray(absEndArray);
                    const t0 = DQ.getTranslation(dqAbs0);
                    const t1 = DQ.getTranslation(dqAbs1);
                    const r0 = DQ.getRotation(dqAbs0);
                    const r1 = DQ.getRotation(dqAbs1);
                    absDt = vecNorm(vecSub(t1, t0));
                    absDAngDeg = (quatAngleRad(r0, r1) * 180) / Math.PI;
                }

                let relErrT = NaN;
                let relErrAngDeg = NaN;
                if (relEndArray) {
                    const dqRelT = DQ.createFromArray(relTargetArray);
                    const dqRelE = DQ.createFromArray(relEndArray);
                    const tT = DQ.getTranslation(dqRelT);
                    const tE = DQ.getTranslation(dqRelE);
                    const rT = DQ.getRotation(dqRelT);
                    const rE = DQ.getRotation(dqRelE);
                    relErrT = vecNorm(vecSub(tE, tT));
                    relErrAngDeg = (quatAngleRad(rT, rE) * 180) / Math.PI;
                }

                console.log('[AbsFirst] iter=', iterEnd,
                    'converged=', converged,
                    'e_abs(last/max)=', (Number.isFinite(lastEAbs) ? lastEAbs.toExponential(3) : 'n/a'), '/', maxEAbs.toExponential(3),
                    'e_rel(last/max)=', (Number.isFinite(lastERel) ? lastERel.toExponential(3) : 'n/a'), '/', maxERel.toExponential(3),
                    'absΔt(m)=', Number.isFinite(absDt) ? absDt.toExponential(3) : 'n/a',
                    'absΔang(deg)=', Number.isFinite(absDAngDeg) ? absDAngDeg.toFixed(3) : 'n/a',
                    'relErrT(m)=', Number.isFinite(relErrT) ? relErrT.toExponential(3) : 'n/a',
                    'relErrAng(deg)=', Number.isFinite(relErrAngDeg) ? relErrAngDeg.toFixed(3) : 'n/a'
                );
            } catch (e) {
                console.warn('[AbsFirst] debug log failed:', e);
            }
        }
        const absEndArray = this._getHumanoidAbsolutePose(q);
        const relEndArray = this._getHumanoidRelativePose(q);
        const finalCmdNorm = relEndArray ? vectorNorm(this._signedPoseError(relTargetArray, relEndArray)) : Number.POSITIVE_INFINITY;
        const finalInvariantNorm = absEndArray ? vectorNorm(this._signedPoseError(absCurrentArray, absEndArray)) : Number.POSITIVE_INFINITY;
        const translationGuardResult = this._evaluateHumanoidTranslationAxisGuard(relCurrentArray, relEndArray, translationAxisGuard);
        if (translationGuardResult?.violated) {
            stopReason = 'command_mismatch';
        }
        const outcome = evaluateHumanoidJogCommandOutcome({
            stopReason,
            initialCommandNorm: initialCmdNorm,
            finalCommandNorm: finalCmdNorm,
            finalInvariantNorm,
            hardCommandTol: hardCmdTol,
            hardInvariantTol: hardInvariantTol,
            practicalCommandTol: practicalCmdTol,
            practicalInvariantTol: practicalInvariantTol,
            mismatchTol,
            mismatchRequiredReduction,
            singleStepMismatchRequiredReduction: Number.isFinite(cfg.qpSingleStepMismatchRequiredReduction)
                ? Math.min(0.99, Math.max(0, cfg.qpSingleStepMismatchRequiredReduction))
                : 0.05
        });
        const messageMap = {
            converged: '命令误差已收敛。',
            practical_converged: '达到实用收敛阈值。',
            command_mismatch: '无法到达该位置（目标未达成）。',
            max_iter: '达到最大迭代次数。',
            fk_fail: '姿态计算失败，提前终止。',
            jacobian_fail: '雅可比计算失败，提前终止。'
        };
        const qApplied = outcome.applyResult ? q : qInitial;
        if (outcome.reason === 'command_mismatch') {
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }
        this.lastIKSolveStatus = {
            solver: 'priority',
            mode: 'humanoid_relative',
            converged: outcome.converged,
            reason: outcome.reason,
            message: messageMap[outcome.reason] || '零空间投影 已执行。',
            finalResidual: finalCmdNorm,
            invariantResidual: finalInvariantNorm
        };
        
        // 应用结果
        for (let i = 0; i < window.humanoidQ.length && i < qApplied.length; i++) {
            window.humanoidQ[i] = qApplied[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        this._publishCurrentPriorityCCTSMDiagnostics({
            modeTag: 'relative-target',
            mode: 'relative',
            q: Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : qApplied,
            armStart,
            activeCount,
            activeLimits: jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null,
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance,
            message: this.lastIKSolveStatus.message
        });
        return outcome.converged;
    }
    
    /**
     * 获取humanoid的关节限位
     */
    _getHumanoidJointLimits() {
        if (!this.humanoidURDF || !this.humanoidJointNames) return null;
        const limits = [];
        for (const name of this.humanoidJointNames) {
            const joint = this.humanoidURDF.joints?.[name];
            if (joint && joint.limit) {
                limits.push({
                    lower: joint.limit.lower ?? -Math.PI,
                    upper: joint.limit.upper ?? Math.PI
                });
            } else {
                limits.push({ lower: -Math.PI, upper: Math.PI });
            }
        }
        if (this.vfiConfig?.humanoidTestDualArmLockEnabled === true) {
            const overrides = this.vfiConfig?.humanoidTestDualArmLockLimits;
            if (overrides && typeof overrides === 'object') {
                for (let i = 0; i < this.humanoidJointNames.length; i++) {
                    const name = this.humanoidJointNames[i];
                    const override = overrides[name];
                    if (!override) continue;
                    const lower = Number.isFinite(override.lower) ? override.lower : limits[i].lower;
                    const upper = Number.isFinite(override.upper) ? override.upper : limits[i].upper;
                    limits[i] = {
                        lower: Math.min(lower, upper),
                        upper: Math.max(lower, upper)
                    };
                }
            }
        }
        return limits;
    }
    
    /**
     * 计算关节限位避免梯度
     * 当关节接近限位时，产生推离力
     */
    _computeJointLimitGradient(q, limits, n) {
        const grad = new Array(n).fill(0);
        if (!limits) return grad;
        
        const margin = 0.1;  // 10% 的边界开始产生梯度
        
        for (let i = 0; i < n && i < limits.length; i++) {
            const lim = limits[i];
            if (!lim) continue;
            
            const range = lim.upper - lim.lower;
            const threshold = range * margin;
            const center = (lim.upper + lim.lower) / 2;
            
            // 接近下限
            if (q[i] < lim.lower + threshold) {
                const d = lim.lower + threshold - q[i];
                grad[i] = d / threshold;  // 推向中心
            }
            // 接近上限
            else if (q[i] > lim.upper - threshold) {
                const d = q[i] - (lim.upper - threshold);
                grad[i] = -d / threshold;  // 推向中心
            }
        }
        return grad;
    }
    
    /**
     * 计算加权阻尼伪逆
     * J_pinv = W^-1 * J^T * (J * W^-1 * J^T + λI)^-1
     */
    _weightedDampedPseudoInverse(J, W, lambda) {
        const m = J.length;      // 行数 (8)
        const n = J[0].length;   // 列数 (关节数)
        
        // 计算 W^-1 (对角阵逆就是元素取倒数)
        const Winv = W.map(w => w > 1e-10 ? 1 / w : 0);
        
        // 计算 J * W^-1 * J^T + λI  (m x m 矩阵)
        const JWJt = [];
        for (let i = 0; i < m; i++) {
            JWJt.push(new Array(m).fill(0));
            for (let j = 0; j < m; j++) {
                let sum = 0;
                for (let k = 0; k < n; k++) {
                    sum += J[i][k] * Winv[k] * J[j][k];
                }
                JWJt[i][j] = sum + (i === j ? lambda : 0);
            }
        }
        
        // 求逆 (J * W^-1 * J^T + λI)^-1
        const JWJtInv = this._matrixInverse(JWJt);
        if (!JWJtInv) return null;
        
        // 计算 W^-1 * J^T * (J * W^-1 * J^T + λI)^-1
        // 结果是 n x m 矩阵
        const Jpinv = [];
        for (let i = 0; i < n; i++) {
            Jpinv.push(new Array(m).fill(0));
            for (let j = 0; j < m; j++) {
                let sum = 0;
                for (let k = 0; k < m; k++) {
                    sum += Winv[i] * J[k][i] * JWJtInv[k][j];
                }
                Jpinv[i][j] = sum;
            }
        }
        return Jpinv;
    }
    
    /**
     * 计算零空间投影矩阵 P = I - J_pinv * J
     * @param {number[][]} Jpinv - 伪逆矩阵 (n x m)
     * @param {number[][]} J - 雅可比矩阵 (m x n)
     * @param {number} n - 关节数（列数）
     */
    _computeNullspaceProjection(Jpinv, J, n) {
        const m = J.length;  // 行数（任务维度）
        const P = [];
        for (let i = 0; i < n; i++) {
            P.push(new Array(n).fill(0));
            P[i][i] = 1.0;  // 初始化为单位阵
            for (let j = 0; j < n; j++) {
                for (let k = 0; k < m; k++) {
                    P[i][j] -= Jpinv[i][k] * J[k][j];
                }
            }
        }
        return P;
    }
    
    /**
     * 计算第二层零空间投影 P12 = P1 - J2Bar_pinv * J2Bar
     */
    _computeNullspaceProjection2(P1, J2BarPinv, J2Bar, n) {
        const m = J2Bar.length;  // 8
        const P12 = [];
        for (let i = 0; i < n; i++) {
            P12.push([...P1[i]]);  // 复制 P1
            for (let j = 0; j < n; j++) {
                for (let k = 0; k < m; k++) {
                    P12[i][j] -= J2BarPinv[i][k] * J2Bar[k][j];
                }
            }
        }
        return P12;
    }
    
    /**
     * 阻尼伪逆 J^+ = J^T * (J * J^T + λI)^-1
     * 用于处理秩亏或接近奇异的雅可比矩阵
     * @param {number[][]} J - 雅可比矩阵 (m x n)
     * @param {number} lambda - 阻尼系数
     * @returns {number[][] | null} - 伪逆矩阵 (n x m)
     */
    _dampedPseudoInverse(J, lambda = 0.01) {
        const m = J.length;      // 行数
        const n = J[0].length;   // 列数
        
        // 计算 J * J^T + λI  (m x m 矩阵)
        const JJt = [];
        for (let i = 0; i < m; i++) {
            JJt.push(new Array(m).fill(0));
            for (let j = 0; j < m; j++) {
                let sum = 0;
                for (let k = 0; k < n; k++) {
                    sum += J[i][k] * J[j][k];
                }
                JJt[i][j] = sum + (i === j ? lambda : 0);
            }
        }
        
        // 求逆 (J * J^T + λI)^-1
        const JJtInv = this._matrixInverse(JJt);
        if (!JJtInv) return null;
        
        // 计算 J^T * (J * J^T + λI)^-1
        // 结果是 n x m 矩阵
        const Jpinv = [];
        for (let i = 0; i < n; i++) {
            Jpinv.push(new Array(m).fill(0));
            for (let j = 0; j < m; j++) {
                let sum = 0;
                for (let k = 0; k < m; k++) {
                    sum += J[k][i] * JJtInv[k][j];
                }
                Jpinv[i][j] = sum;
            }
        }
        return Jpinv;
    }
    
    /**
     * 矩阵乘法 C = A * B
     */
    _matMul(A, B) {
        const m = A.length;
        const n = B[0].length;
        const p = A[0].length;
        const C = [];
        for (let i = 0; i < m; i++) {
            C.push(new Array(n).fill(0));
            for (let j = 0; j < n; j++) {
                for (let k = 0; k < p; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }
    
    /**
     * 矩阵求逆 (Gauss-Jordan消元法)
     */
    _matrixInverse(A) {
        const n = A.length;
        // 创建增广矩阵 [A | I]
        const aug = [];
        for (let i = 0; i < n; i++) {
            aug.push([...A[i]]);
            for (let j = 0; j < n; j++) {
                aug[i].push(i === j ? 1 : 0);
            }
        }
        
        // Gauss-Jordan 消元
        for (let i = 0; i < n; i++) {
            // 找主元
            let maxRow = i;
            for (let k = i + 1; k < n; k++) {
                if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) {
                    maxRow = k;
                }
            }
            [aug[i], aug[maxRow]] = [aug[maxRow], aug[i]];
            
            if (Math.abs(aug[i][i]) < 1e-12) return null;  // 奇异矩阵
            
            // 归一化主元行
            const pivot = aug[i][i];
            for (let j = 0; j < 2 * n; j++) {
                aug[i][j] /= pivot;
            }
            
            // 消元
            for (let k = 0; k < n; k++) {
                if (k !== i) {
                    const factor = aug[k][i];
                    for (let j = 0; j < 2 * n; j++) {
                        aug[k][j] -= factor * aug[i][j];
                    }
                }
            }
        }
        
        // 提取逆矩阵
        const inv = [];
        for (let i = 0; i < n; i++) {
            inv.push(aug[i].slice(n));
        }
        return inv;
    }
    
    /**
     * 获取左臂位姿的雅可比矩阵（数值微分）
     */
    _getHumanoidPose1Jacobian(q, eps = 1e-7) {
        const n = q.length;
        const J = [];
        for (let i = 0; i < 8; i++) J.push(new Array(n).fill(0));
        const r0 = this._getHumanoidPose1(q);
        if (!r0) return null;
        for (let j = 0; j < n; j++) {
            const qp = q.slice(); qp[j] += eps;
            const qm = q.slice(); qm[j] -= eps;
            const rp = this._getHumanoidPose1(qp);
            const rm = this._getHumanoidPose1(qm);
            if (!rp || !rm) return null;
            for (let i = 0; i < 8; i++) J[i][j] = (rp[i] - rm[i]) / (2 * eps);
        }
        return J;
    }
    
    /**
     * 获取右臂位姿的雅可比矩阵（数值微分）
     */
    _getHumanoidPose2Jacobian(q, eps = 1e-7) {
        const n = q.length;
        const J = [];
        for (let i = 0; i < 8; i++) J.push(new Array(n).fill(0));
        const r0 = this._getHumanoidPose2(q);
        if (!r0) return null;
        for (let j = 0; j < n; j++) {
            const qp = q.slice(); qp[j] += eps;
            const qm = q.slice(); qm[j] -= eps;
            const rp = this._getHumanoidPose2(qp);
            const rm = this._getHumanoidPose2(qm);
            if (!rp || !rm) return null;
            for (let i = 0; i < 8; i++) J[i][j] = (rp[i] - rm[i]) / (2 * eps);
        }
        return J;
    }
    
    /**
     * 全身控制：沿轴平移绝对位姿（腰部+双臂联动，不含底盘）
     */
    moveHumanoidAbsolutePoseAlongAxisDecoupled(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) {
            this._publishHumanoidJogInputFailure({
                modeTag: 'absolute-target',
                includeWaist: true,
                keepBodyZVertical: false,
                reason: 'dq_unavailable',
                message: 'DQ 模块未加载，无法执行全身 absolute 点动。'
            });
            return;
        }
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;

            // 获取当前绝对位姿
            const dqAbsArray = this._getHumanoidAbsolutePose(window.humanoidQ);
            if (!dqAbsArray) {
                this._publishHumanoidJogInputFailure({
                    modeTag: 'absolute-target',
                    includeWaist: true,
                    keepBodyZVertical: false,
                    reason: 'fk_unavailable',
                    message: '当前 cooperative absolute pose 不可用，无法执行全身 absolute 点动。'
                });
                return;
            }
            const dqAbsCurrent = DQ.createFromArray(dqAbsArray);
            const rotationArray = DQ.getRotation(dqAbsCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);

            // 计算平移增量
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
            const translationAxisGuard = this._buildTranslationAxisGuard(delta);
            
            // 计算目标绝对位姿
            const dqTargetDQ = DQ.multiply(dqDelta, dqAbsCurrent);
            const dqTargetArray = DQ.toArray(dqTargetDQ);
            
            // 获取关节限位避免开关状态
            const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

            // 不带底盘：仅腰部+双臂联动
            this.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
                dqTargetArray,
                window.humanoidQ,
                enableLimitAvoidance,
                true,
                false,
                null,
                translationAxisGuard
            );
        } catch (e) {
            console.error('全身控制移动绝对位姿失败:', e);
            this._publishHumanoidJogInputFailure({
                modeTag: 'absolute-target',
                includeWaist: true,
                keepBodyZVertical: false,
                reason: 'jog_exception',
                message: `全身 absolute 点动异常：${e?.message || 'unknown'}`
            });
        }
    }
    
    /**
     * 全身控制：绕轴旋转绝对位姿（腰部+双臂联动，不含底盘）
     */
    rotateHumanoidAbsolutePoseAroundAxisDecoupled(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;

            const dqAbsoluteCurrentArray = this._getHumanoidAbsolutePose(window.humanoidQ);
            if (!dqAbsoluteCurrentArray) return;

            const dqAbsoluteCurrent = DQ.createFromArray(dqAbsoluteCurrentArray);

            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return;
            
            const dqAbsoluteTarget = this.combineDQWithTeachOrder(rotationDQ, dqAbsoluteCurrent, false);
            
            // 获取关节限位避免开关状态
            const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

            // 不带底盘：仅腰部+双臂联动
            this.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(DQ.toArray(dqAbsoluteTarget), window.humanoidQ, enableLimitAvoidance, true);
        } catch (e) {
            console.error('全身控制旋转绝对位姿失败:', e);
        }
    }

    /**
     * 全身控制（腰部竖直约束）：沿轴平移绝对位姿（腰部+双臂联动，不含底盘）
     */
    moveHumanoidAbsolutePoseAlongAxisDecoupledUpright(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) {
            this._publishHumanoidJogInputFailure({
                modeTag: 'absolute-target',
                includeWaist: true,
                keepBodyZVertical: true,
                reason: 'dq_unavailable',
                message: 'DQ 模块未加载，无法执行全身竖直 absolute 点动。'
            });
            return;
        }

        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;

            const dqAbsArray = this._getHumanoidAbsolutePose(window.humanoidQ);
            if (!dqAbsArray) {
                this._publishHumanoidJogInputFailure({
                    modeTag: 'absolute-target',
                    includeWaist: true,
                    keepBodyZVertical: true,
                    reason: 'fk_unavailable',
                    message: '当前 cooperative absolute pose 不可用，无法执行全身竖直 absolute 点动。'
                });
                return;
            }
            const dqAbsCurrent = DQ.createFromArray(dqAbsArray);
            const rotationArray = DQ.getRotation(dqAbsCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
            
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
            const dqTargetDQ = DQ.multiply(dqDelta, dqAbsCurrent);
            const dqTargetArray = DQ.toArray(dqTargetDQ);
            const translationAxisGuard = this._buildTranslationAxisGuard(delta);
            
            const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

            this.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
                dqTargetArray,
                window.humanoidQ,
                enableLimitAvoidance,
                true,
                true,
                null,
                translationAxisGuard
            );
        } catch (e) {
            console.error('全身控制（腰部竖直约束）移动绝对位姿失败:', e);
            this._publishHumanoidJogInputFailure({
                modeTag: 'absolute-target',
                includeWaist: true,
                keepBodyZVertical: true,
                reason: 'jog_exception',
                message: `全身竖直 absolute 点动异常：${e?.message || 'unknown'}`
            });
        }
    }
    
    /**
     * 全身控制（腰部竖直约束）：绕轴旋转绝对位姿（腰部+双臂联动，不含底盘）
     */
    rotateHumanoidAbsolutePoseAroundAxisDecoupledUpright(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;

        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;

            const dqAbsoluteCurrentArray = this._getHumanoidAbsolutePose(window.humanoidQ);
            if (!dqAbsoluteCurrentArray) return;
            
            const dqAbsoluteCurrent = DQ.createFromArray(dqAbsoluteCurrentArray);
            
            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return;
            
            const dqAbsoluteTarget = this.combineDQWithTeachOrder(rotationDQ, dqAbsoluteCurrent, false);
            const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;

            this.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(DQ.toArray(dqAbsoluteTarget), window.humanoidQ, enableLimitAvoidance, true, true);
        } catch (e) {
            console.error('全身控制（腰部竖直约束）旋转绝对位姿失败:', e);
        }
    }

    /**
     * 全身相对位姿控制：沿轴平移相对位姿（腰部+双臂联动）
     */
    moveHumanoidRelativePoseAlongAxisFullbody(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            
            // 获取当前相对位姿
            const dqRelArray = this._getHumanoidRelativePose(window.humanoidQ);
            if (!dqRelArray) return;
            const dqRelCurrent = DQ.createFromArray(dqRelArray);
            const rotationArray = DQ.getRotation(dqRelCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
            
            // 计算平移增量
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
            const translationAxisGuard = this._buildTranslationAxisGuard(delta);
            
            // 计算目标相对位姿
            const worldMode = this.isWorldTranslationMode?.() ?? true;
            const dqTargetDQ = worldMode ? DQ.multiply(dqDelta, dqRelCurrent) : DQ.multiply(dqRelCurrent, dqDelta);
            const dqTargetArray = DQ.toArray(dqTargetDQ);
            
            // 获取关节限位避免开关状态
            const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
            
            // 使用绝对位姿优先的零空间控制更新（绝对位姿不变 > 相对位姿跟踪 > 关节限位）
            // 全身控制：包含腰部 (includeWaist=true)
            this.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(
                dqTargetArray,
                window.humanoidQ,
                enableLimitAvoidance,
                true,
                translationAxisGuard
            );
        } catch (e) {
            console.error('全身相对位姿平移失败:', e);
        }
    }
    
    /**
     * 全身相对位姿控制：绕轴旋转相对位姿（腰部+双臂联动）
     */
    rotateHumanoidRelativePoseAroundAxisFullbody(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            const dqRelativeCurrentArray = this._getHumanoidRelativePose(window.humanoidQ);
            if (!dqRelativeCurrentArray) return;
            const dqRelativeCurrent = DQ.createFromArray(dqRelativeCurrentArray);
            
            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return;
            
            const dqRelativeTarget = this.combineDQWithTeachOrder(rotationDQ, dqRelativeCurrent, false);
            
            // 获取关节限位避免开关状态
            const enableLimitAvoidance = document.getElementById('joint-limit-avoidance-toggle')?.checked ?? true;
            
            // 使用绝对位姿优先的零空间控制更新（绝对位姿不变 > 相对位姿跟踪 > 关节限位）
            // 全身控制：包含腰部 (includeWaist=true)
            this.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(DQ.toArray(dqRelativeTarget), window.humanoidQ, enableLimitAvoidance, true);
        } catch (e) {
            console.error('全身相对位姿旋转失败:', e);
        }
    }
    
    updateHumanoidJointsFromAbsolutePoseTarget(dqAbsoluteTargetArray, currentQ, dqRelativeCurrentArray = null, enableJointLimitAvoidance = true) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const relCurrent = dqRelativeCurrentArray || this._getHumanoidRelativePose(currentQ);
        if (!relCurrent) return false;
        const { start: armStart, end: armEnd, count: armCount } = this._humanoidArmIndices();
        if (armCount <= 0) return false;
        
        // 获取关节限位
        const jointLimits = this._getHumanoidJointLimits();
        const armLimits = jointLimits ? jointLimits.slice(armStart, armEnd) : null;
        const limitWeight = 0.02;  // 限位惩罚权重（降低以减少冲击）
        const margin = 0.1;  // 边界百分比
        
        const x0 = currentQ.slice(armStart, armEnd);
        const f = (x) => {
            const q = this._humanoidFullQFromArms(currentQ, x);
            const absPose = this._getHumanoidAbsolutePose(q);
            const relPose = this._getHumanoidRelativePose(q);
            if (!absPose || !relPose) return 1e10;
            let sum = 0;
            for (let i = 0; i < 8; i++) {
                sum += (absPose[i] - dqAbsoluteTargetArray[i]) ** 2;
                sum += (relPose[i] - relCurrent[i]) ** 2;
            }
            // 添加关节限位惩罚
            if (enableJointLimitAvoidance && armLimits) {
                for (let i = 0; i < armCount && i < armLimits.length; i++) {
                    const lim = armLimits[i];
                    if (!lim) continue;
                    const range = lim.upper - lim.lower;
                    const threshold = range * margin;
                    // 接近下限惩罚
                    if (x[i] < lim.lower + threshold) {
                        const d = lim.lower + threshold - x[i];
                        sum += limitWeight * d * d;
                    }
                    // 接近上限惩罚
                    if (x[i] > lim.upper - threshold) {
                        const d = x[i] - (lim.upper - threshold);
                        sum += limitWeight * d * d;
                    }
                }
            }
            return 0.5 * sum;
        };
        const grad = (x, out) => {
            const q = this._humanoidFullQFromArms(currentQ, x);
            const absPose = this._getHumanoidAbsolutePose(q);
            const relPose = this._getHumanoidRelativePose(q);
            const Jabs = this._getHumanoidAbsolutePoseJacobian(q);
            const Jrel = this._getHumanoidRelativePoseJacobian(q);
            if (!Jabs || !Jrel) return;
            for (let i = 0; i < armCount; i++) {
                let s = 0;
                for (let j = 0; j < 8; j++) {
                    s += Jabs[j][armStart + i] * (absPose[j] - dqAbsoluteTargetArray[j]);
                    s += Jrel[j][armStart + i] * (relPose[j] - relCurrent[j]);
                }
                // 添加关节限位梯度
                if (enableJointLimitAvoidance && armLimits && i < armLimits.length) {
                    const lim = armLimits[i];
                    if (lim) {
                        const range = lim.upper - lim.lower;
                        const threshold = range * margin;
                        if (x[i] < lim.lower + threshold) {
                            const d = lim.lower + threshold - x[i];
                            s -= limitWeight * d;
                        }
                        if (x[i] > lim.upper - threshold) {
                            const d = x[i] - (lim.upper - threshold);
                            s += limitWeight * d;
                        }
                    }
                }
                out[i] = s;
            }
        };
        const { x: xSol } = this.minimizeLBFGS(f, grad, x0, { maxIter: 80, tol: 1e-6, m: 10 });
        const qSol = this._humanoidFullQFromArms(currentQ, xSol);
        for (let i = 0; i < window.humanoidQ.length && i < qSol.length; i++) window.humanoidQ[i] = qSol[i];
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(qSol, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) window.humanoidQ[i] = clamped[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }
    
    updateHumanoidJointsFromRelativePoseTarget(dqRelativeTargetArray, currentQ, dqAbsoluteCurrentArray = null) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const absCurrent = dqAbsoluteCurrentArray || this._getHumanoidAbsolutePose(currentQ);
        if (!absCurrent) return false;
        const { start: armStart, end: armEnd, count: armCount } = this._humanoidArmIndices();
        if (armCount <= 0) return false;
        const x0 = currentQ.slice(armStart, armEnd);
        const f = (x) => {
            const q = this._humanoidFullQFromArms(currentQ, x);
            const absPose = this._getHumanoidAbsolutePose(q);
            const relPose = this._getHumanoidRelativePose(q);
            if (!absPose || !relPose) return 1e10;
            let sum = 0;
            for (let i = 0; i < 8; i++) {
                sum += (absPose[i] - absCurrent[i]) ** 2;
                sum += (relPose[i] - dqRelativeTargetArray[i]) ** 2;
            }
            return 0.5 * sum;
        };
        const grad = (x, out) => {
            const q = this._humanoidFullQFromArms(currentQ, x);
            const absPose = this._getHumanoidAbsolutePose(q);
            const relPose = this._getHumanoidRelativePose(q);
            const Jabs = this._getHumanoidAbsolutePoseJacobian(q);
            const Jrel = this._getHumanoidRelativePoseJacobian(q);
            if (!Jabs || !Jrel) return;
            for (let i = 0; i < armCount; i++) {
                let s = 0;
                for (let j = 0; j < 8; j++) {
                    s += Jabs[j][armStart + i] * (absPose[j] - absCurrent[j]);
                    s += Jrel[j][armStart + i] * (relPose[j] - dqRelativeTargetArray[j]);
                }
                out[i] = s;
            }
        };
        const { x: xSol } = this.minimizeLBFGS(f, grad, x0, { maxIter: 80, tol: 1e-6, m: 10 });
        const qSol = this._humanoidFullQFromArms(currentQ, xSol);
        for (let i = 0; i < window.humanoidQ.length && i < qSol.length; i++) window.humanoidQ[i] = qSol[i];
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(qSol, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) window.humanoidQ[i] = clamped[i];
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }
    
    /**
     * 零空间优先级控制：相对距离控制
     * 优先级：绝对位姿保持（最高）> 相对距离跟踪 > 关节限位避免
     * 
     * @param {number} deltaDistance - 距离增量
     * @param {number[]} currentQ - 当前全关节角
     * @param {boolean} enableJointLimitAvoidance - 是否启用关节限位避免
     * @param {boolean} includeWaist - 是否包含腰部关节
     * @returns {boolean}
     */
    _updateHumanoidJointsFromRelativeDistanceDeltaVFI(
        deltaDistance,
        currentQ,
        enableJointLimitAvoidance = true,
        includeWaist = false
    ) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const DQ = window.DQModule?.DQWrapper;
        const Kinematics = window.DQModule?.Kinematics;
        if (!DQ || !Kinematics?.distanceJacobian) return false;

        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;
        const activeCount = includeWaist ? (waistCount + leftArmCount + rightArmCount) : (leftArmCount + rightArmCount);
        if (activeCount <= 0) return false;

        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const relStartArray = this._getHumanoidRelativePose(currentQ);
        const pose1StartArray = this._getHumanoidPose1(currentQ);
        const pose2StartArray = this._getHumanoidPose2(currentQ);
        if (!absCurrentArray || !relStartArray || !pose1StartArray || !pose2StartArray) return false;
        const relRotTarget = relStartArray.slice(0, 4);
        const hand1RotTarget = pose1StartArray.slice(0, 4);
        const hand2RotTarget = pose2StartArray.slice(0, 4);
        const tStart = DQ.getTranslation(DQ.createFromArray(relStartArray));
        const dStart = Math.sqrt((tStart[1] || 0) ** 2 + (tStart[2] || 0) ** 2 + (tStart[3] || 0) ** 2);
        const dTarget = Math.max(0.01, dStart + deltaDistance);
        const d2Target = dTarget * dTarget;

        const jointLimits = this._getHumanoidJointLimits();
        const hardLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
        const softLimits = enableJointLimitAvoidance ? hardLimits : null;
        const cfg = this.vfiConfig || {};
        const maxIter = Number.isFinite(cfg.qpOuterMaxIter) ? cfg.qpOuterMaxIter : 16;
        const damping = Number.isFinite(cfg.damping) ? cfg.damping : 1e-3;
        const maxStep = Number.isFinite(cfg.maxStep) ? cfg.maxStep : 0.003;
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolDistanceSq) ? cfg.hardCommandTolDistanceSq : 1e-8;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const practicalCmdTol = Number.isFinite(cfg.practicalCommandTolDistanceSq) ? cfg.practicalCommandTolDistanceSq : 1e-5;
        const practicalInvariantTol = Number.isFinite(cfg.practicalInvariantTol) ? cfg.practicalInvariantTol : 2.5e-4;
        const practicalStepTol = Number.isFinite(cfg.practicalStepNormTol) ? cfg.practicalStepNormTol : 2e-5;
        const practicalViolationTol = Number.isFinite(cfg.practicalViolationTol) ? cfg.practicalViolationTol : 1e-8;
        const distanceUseInvariantEquality = (typeof cfg.distanceQPUseInvariantEquality === 'boolean')
            ? cfg.distanceQPUseInvariantEquality
            : false;
        const distanceKeepRelativeRotation = (typeof cfg.distanceKeepRelativeRotation === 'boolean')
            ? cfg.distanceKeepRelativeRotation
            : true;
        const distanceKeepHandOrientation = (typeof cfg.distanceKeepHandOrientation === 'boolean')
            ? cfg.distanceKeepHandOrientation
            : true;
        const distanceInvariantKappa = Number.isFinite(cfg.distanceInvariantKappa)
            ? Math.max(0, cfg.distanceInvariantKappa)
            : 0.7;
        const distanceInvariantBandBase = Number.isFinite(cfg.distanceInvariantBandBase)
            ? Math.max(0, cfg.distanceInvariantBandBase)
            : 2e-3;
        const distanceInvariantBandGain = Number.isFinite(cfg.distanceInvariantBandGain)
            ? Math.max(0, cfg.distanceInvariantBandGain)
            : 0.3;
        const distanceInvariantSoftWeight = Number.isFinite(cfg.distanceInvariantSoftWeight)
            ? Math.max(0, cfg.distanceInvariantSoftWeight)
            : 6.0;

        const qInitial = [...currentQ];
        let q = [...qInitial];
        let executedIterations = 0;
        let stopReason = 'max_iter';
        let initialCmdNorm = Number.NaN;
        let lastCmdNorm = Number.NaN;
        let lastInvariantNorm = Number.NaN;
        let lastStats = null;
        const extractActiveColumns = (row) => {
            const out = new Array(activeCount).fill(0);
            if (!Array.isArray(row)) return out;
            for (let i = 0; i < activeCount; i++) {
                out[i] = row[armStart + i] || 0;
            }
            return out;
        };
        for (let iter = 0; iter < maxIter; iter++) {
            const absNow = this._getHumanoidAbsolutePose(q);
            const relNow = this._getHumanoidRelativePose(q);
            if (!absNow || !relNow) {
                stopReason = 'fk_fail';
                break;
            }

            const eAbs = this._signedPoseError(absCurrentArray, absNow);
            const relRotNow = relNow.slice(0, 4);
            let dotRot = 0;
            for (let i = 0; i < 4; i++) dotRot += (relRotTarget[i] || 0) * (relRotNow[i] || 0);
            const signRot = dotRot < 0 ? -1 : 1;
            const eRot = relRotTarget.map((v, i) => (v || 0) - signRot * (relRotNow[i] || 0));
            let eHand1Rot = [];
            let eHand2Rot = [];
            let Jhand1Rot = [];
            let Jhand2Rot = [];
            if (distanceKeepHandOrientation) {
                const pose1Now = this._getHumanoidPose1(q);
                const pose2Now = this._getHumanoidPose2(q);
                if (!pose1Now || !pose2Now) {
                    stopReason = 'fk_fail';
                    break;
                }
                const hand1RotNow = pose1Now.slice(0, 4);
                const hand2RotNow = pose2Now.slice(0, 4);
                let dotHand1 = 0;
                let dotHand2 = 0;
                for (let i = 0; i < 4; i++) {
                    dotHand1 += (hand1RotTarget[i] || 0) * (hand1RotNow[i] || 0);
                    dotHand2 += (hand2RotTarget[i] || 0) * (hand2RotNow[i] || 0);
                }
                const signHand1 = dotHand1 < 0 ? -1 : 1;
                const signHand2 = dotHand2 < 0 ? -1 : 1;
                eHand1Rot = hand1RotTarget.map((v, i) => (v || 0) - signHand1 * (hand1RotNow[i] || 0));
                eHand2Rot = hand2RotTarget.map((v, i) => (v || 0) - signHand2 * (hand2RotNow[i] || 0));
                const Jpose1Full = this._getHumanoidSingleArmPoseJacobian(q, 1) || this._getHumanoidPose1Jacobian(q);
                const Jpose2Full = this._getHumanoidSingleArmPoseJacobian(q, 2) || this._getHumanoidPose2Jacobian(q);
                if (!Jpose1Full || !Jpose2Full) { stopReason = 'solver_fail'; break; }
                Jhand1Rot = Jpose1Full.slice(0, 4).map((row) => extractActiveColumns(row));
                Jhand2Rot = Jpose2Full.slice(0, 4).map((row) => extractActiveColumns(row));
            }
            const eInvariant = [...eAbs];
            if (distanceKeepRelativeRotation) eInvariant.push(...eRot);
            if (distanceKeepHandOrientation) eInvariant.push(...eHand1Rot, ...eHand2Rot);
            lastInvariantNorm = Math.sqrt(eInvariant.reduce((s, v) => s + v * v, 0));

            const tNow = DQ.getTranslation(DQ.createFromArray(relNow));
            const d2Now = (tNow[1] || 0) ** 2 + (tNow[2] || 0) ** 2 + (tNow[3] || 0) ** 2;
            const eCmd = [d2Target - d2Now];
            lastCmdNorm = Math.abs(eCmd[0]);
            if (!Number.isFinite(initialCmdNorm)) initialCmdNorm = lastCmdNorm;
            if (lastCmdNorm < hardCmdTol && lastInvariantNorm < hardInvariantTol) {
                stopReason = 'converged';
                break;
            }

            const JabsFull = this._getHumanoidAbsolutePoseJacobian(q);
            const JrelFull = this._getHumanoidRelativePoseJacobian(q);
            if (!JabsFull || !JrelFull) { stopReason = 'solver_fail'; break; }
            const Jd2Full = Kinematics.distanceJacobian(JrelFull, relNow);
            const Jd2Row = Array.isArray(Jd2Full) && Array.isArray(Jd2Full[0]) ? Jd2Full[0] : null;
            if (!Jd2Row) { stopReason = 'solver_fail'; break; }
            const Jinvariant = [...JabsFull.map((row) => extractActiveColumns(row))];
            if (distanceKeepRelativeRotation) {
                Jinvariant.push(...JrelFull.slice(0, 4).map((row) => extractActiveColumns(row)));
            }
            if (distanceKeepHandOrientation) {
                Jinvariant.push(...Jhand1Rot, ...Jhand2Rot);
            }
            const qActive = q.slice(armStart, armStart + activeCount);
            const stepResult = this._solveVFICommandStep({
                Jcmd: [extractActiveColumns(Jd2Row)],
                eCmd,
                Jinvariant,
                eInvariant,
                qActive,
                activeLimits: softLimits,
                maxStep,
                damping,
                forceInvariantEquality: distanceUseInvariantEquality,
                invariantKappaOverride: distanceInvariantKappa,
                invariantBandBaseOverride: distanceInvariantBandBase,
                invariantBandGainOverride: distanceInvariantBandGain,
                invariantSoftWeightOverride: distanceInvariantSoftWeight,
                extraVFIRows: this._getWaistXVFIRows(q, armStart, activeCount),
                nullManipGrad: this._computeProtectedManipulabilityGradient({
                    q,
                    armStart,
                    activeCount,
                    mode: 'distance',
                    Jcmd: [extractActiveColumns(Jd2Row)],
                    Jinvariant,
                    activeLimits: hardLimits,
                    forceInvariantEquality: distanceUseInvariantEquality
                }),
                wvfiGuard: this._buildWVFILowerBoundGuard({
                    q,
                    armStart,
                    activeCount,
                    mode: 'distance',
                    activeLimits: hardLimits,
                    includeWaist
                })
            });
            if (!stepResult) { stopReason = 'solver_fail'; break; }
            const stats = stepResult.stats;
            if (stats?.wvfiGuardAccepted === false) { stopReason = 'wvfi_lower_bound'; break; }
            if (!Array.isArray(stepResult.dq)) { stopReason = 'solver_fail'; break; }
            if (stats) lastStats = stats;
            if (stats?.qpConverged === false) {
                stopReason = 'qp_not_converged';
                break;
            }
            const practicalConverged = (
                Number.isFinite(lastCmdNorm) &&
                lastCmdNorm <= practicalCmdTol &&
                Number.isFinite(lastInvariantNorm) &&
                lastInvariantNorm <= practicalInvariantTol &&
                Number.isFinite(stats?.stepNorm) &&
                stats.stepNorm <= practicalStepTol &&
                Number.isFinite(stats?.maxViolationAfter) &&
                stats.maxViolationAfter <= practicalViolationTol
            );

            const qNext = [...q];
            for (let i = 0; i < activeCount; i++) {
                let val = (q[armStart + i] || 0) + (stepResult.dq[i] || 0);
                const lim = hardLimits?.[i];
                if (lim) val = Math.max(lim.lower, Math.min(lim.upper, val));
                qNext[armStart + i] = val;
            }
            q = qNext;
            executedIterations = iter + 1;
            if (practicalConverged) {
                stopReason = 'practical_converged';
                break;
            }
        }
        if (stopReason === 'max_iter' && maxIter <= 1) {
            stopReason = 'single_step';
        }
        let finalCmdNorm = Number.NaN;
        const relEndDistance = this._getHumanoidRelativePose(q);
        if (relEndDistance) {
            const tEnd = DQ.getTranslation(DQ.createFromArray(relEndDistance));
            const d2End = (tEnd[1] || 0) ** 2 + (tEnd[2] || 0) ** 2 + (tEnd[3] || 0) ** 2;
            finalCmdNorm = Math.abs(d2Target - d2End);
        }
        if (stopReason !== 'wvfi_lower_bound' && this._shouldFailForQPMismatch({
            stopReason,
            finalCmdNorm,
            initialCmdNorm
        }, cfg)) {
            stopReason = 'command_mismatch';
        }
        if (stopReason === 'qp_not_converged' || stopReason === 'command_mismatch' || stopReason === 'wvfi_lower_bound') {
            q = [...qInitial];
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }
        const cctsmResult_dist = this._computeModeCCTSM(
            q,
            armStart,
            activeCount,
            'distance',
            this._buildManipulabilityVFIConstraints(q, armStart, activeCount, hardLimits)
        );
        const messageMap = {
            converged: '距离误差已收敛。',
            practical_converged: '距离任务达到实用收敛阈值。',
            single_step: 'new qpJ 单步更新完成。',
            qp_not_converged: '无法到达该位置（new qpJ 未收敛）。',
            command_mismatch: '无法到达该位置（目标未达成）。',
            wvfi_lower_bound: '无法到达该位置（w_vfi 下界不可行）。',
            max_iter: '达到最大迭代次数。',
            fk_fail: '姿态计算失败，提前终止。',
            jacobian_fail: '雅可比计算失败，提前终止。',
            solver_fail: 'new qpJ 求解失败。'
        };
        this._publishHumanoidJogDiagnostics({
            solverMode: 'vfi',
            modeTag: 'distance-target',
            active: true,
            message: messageMap[stopReason] || 'new qpJ 已执行。',
            iterations: executedIterations,
            activeJoints: activeCount,
            commandRows: lastStats?.commandRows,
            invariantRows: lastStats?.invariantRows,
            constraintRows: lastStats?.constraintRows,
            invariantConstraintRows: lastStats?.invariantConstraintRows,
            equalityRows: lastStats?.equalityRows,
            componentRows: lastStats?.componentRows,
            jointLimitRows: lastStats?.jointLimitRows,
            commandError: Number.isFinite(finalCmdNorm) ? finalCmdNorm : lastCmdNorm,
            invariantError: lastInvariantNorm,
            stepNorm: lastStats?.stepNorm,
            cmdTargetNorm: lastStats?.cmdTargetNorm,
            cmdAchievedNorm: lastStats?.cmdAchievedNorm,
            cmdResidualNorm: lastStats?.cmdResidualNorm,
            cmdAlignment: lastStats?.cmdAlignment,
            cmdTargetFirst: lastStats?.cmdTargetFirst,
            cmdAchievedFirst: lastStats?.cmdAchievedFirst,
            maxViolationBefore: lastStats?.maxViolationBefore,
            maxViolationAfter: lastStats?.maxViolationAfter,
            qpIterations: lastStats?.qpIterations,
            qpConverged: lastStats?.qpConverged,
            qpObjective: lastStats?.qpObjective,
            qpMessage: lastStats?.qpMessage,
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance,
            cctsmW: cctsmResult_dist?.w ?? NaN,
            cctsmWvfi: cctsmResult_dist?.w_vfi ?? NaN
        });
        this.lastIKSolveStatus = {
            solver: 'vfi',
            mode: 'humanoid_distance',
            converged: stopReason === 'converged' || stopReason === 'practical_converged' || stopReason === 'single_step',
            reason: stopReason,
            message: messageMap[stopReason] || 'new qpJ 已执行。',
            finalResidual: Number.isFinite(finalCmdNorm) ? finalCmdNorm : lastCmdNorm,
            invariantResidual: lastInvariantNorm,
            qpConverged: lastStats?.qpConverged,
            qpMessage: lastStats?.qpMessage
        };

        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        if (window.clampJointAngles && this.humanoidURDF && window.customRobotConfig) {
            const clamped = window.clampJointAngles(q, this.humanoidJointNames || [], this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }

    updateHumanoidJointsFromRelativeDistanceDelta(deltaDistance, currentQ, enableJointLimitAvoidance = true, includeWaist = false) {
        if (this._isHumanoidMJRestoreSolverEnabled()) {
            if (supportsHumanoidJogMJRestoreMode('distance-target')) {
                return this._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
                    deltaDistance,
                    currentQ,
                    enableJointLimitAvoidance,
                    includeWaist
                );
            }
            const fallbackMessage = 'MuJoCo 风格恢复 当前仅支持 absolute-target，其它模式已回退到 new qpJ。';
            const ok = this._updateHumanoidJointsFromRelativeDistanceDeltaVFI(
                deltaDistance,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist
            );
            const payload = this.lastHumanoidJogDiagnostics
                ? {
                    ...this.lastHumanoidJogDiagnostics,
                    solverMode: 'mj_restore',
                    modeTag: 'distance-target',
                    message: fallbackMessage,
                    nextMode: getHumanoidJogSolverDisplayName('vfi'),
                    diagnosticLabel: 'fallback_to_new_qpJ'
                }
                : {
                    solverMode: 'mj_restore',
                    modeTag: 'distance-target',
                    active: true,
                    message: fallbackMessage,
                    nextMode: getHumanoidJogSolverDisplayName('vfi')
                };
            this._publishHumanoidJogDiagnostics(payload);
            return ok;
        }
        if (this._isHumanoidVFISolverEnabled()) {
            return this._updateHumanoidJointsFromRelativeDistanceDeltaVFI(
                deltaDistance,
                currentQ,
                enableJointLimitAvoidance,
                includeWaist
            );
        }
        this._publishHumanoidJogDiagnostics({
            solverMode: 'priority',
            modeTag: 'distance-target',
            active: true,
            message: '当前为零空间投影求解器。',
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance
        });
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !window.DQModule?.Kinematics?.distanceJacobian) return false;
        
        // 获取关节索引 - 根据 includeWaist 参数决定是否包含腰部
        const waistCount = this.humanoidWaistLength;
        const leftArmCount = this.humanoidLeftArmLength;
        const rightArmCount = this.humanoidRightArmLength;
        const armStart = includeWaist ? 0 : waistCount;
        const activeCount = includeWaist ? (waistCount + leftArmCount + rightArmCount) : (leftArmCount + rightArmCount);
        if (activeCount <= 0) return false;
        
        // 保存当前绝对位姿和相对姿态作为约束目标（任务1）
        const absCurrentArray = this._getHumanoidAbsolutePose(currentQ);
        const dqRelStart = this._getHumanoidRelativePose(currentQ);
        if (!absCurrentArray || !dqRelStart) return false;
        
        // 提取相对姿态的旋转部分（四元数，前4个元素）
        const relRotTarget = dqRelStart.slice(0, 4);
        
        // 计算目标距离
        const dqRelObj = DQ.createFromArray(dqRelStart);
        const tStart = DQ.getTranslation(dqRelObj);
        const dStart = Math.sqrt((tStart[1] || 0) ** 2 + (tStart[2] || 0) ** 2 + (tStart[3] || 0) ** 2);
        const dTarget = Math.max(0.01, dStart + deltaDistance);  // 最小距离0.01m防止奇异
        const d2Target = dTarget * dTarget;

        // 获取关节限位
        const jointLimits = this._getHumanoidJointLimits();
        const cfg = this.vfiConfig || {};
        const hardCmdTol = Number.isFinite(cfg.hardCommandTolDistanceSq) ? cfg.hardCommandTolDistanceSq : 1e-8;
        const hardInvariantTol = Number.isFinite(cfg.hardInvariantTol) ? cfg.hardInvariantTol : 5e-5;
        const practicalCmdTol = Number.isFinite(cfg.practicalCommandTolDistanceSq) ? cfg.practicalCommandTolDistanceSq : 1e-5;
        const practicalInvariantTol = Number.isFinite(cfg.practicalInvariantTol) ? cfg.practicalInvariantTol : 2.5e-4;
        const mismatchTol = Number.isFinite(cfg.qpMismatchTol) ? cfg.qpMismatchTol : 1e-3;
        const mismatchRequiredReduction = Number.isFinite(cfg.qpMismatchRequiredReduction) ? cfg.qpMismatchRequiredReduction : 0.8;
        const vectorNorm = (values) => Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => sum + (value || 0) * (value || 0), 0));
        const initialCmdNorm = Math.abs(d2Target - dStart * dStart);
        
        // 增益参数
        const kappa1 = 0.2;     // 任务1增益（绝对位姿+相对姿态保持 - 最高优先级）
        const kappa2 = 0.4;     // 任务2增益（相对距离跟踪）
        const kappa3 = 0.0005;    // 任务3增益（关节限位避免）
        const waistGain = 0.5;  // 腰部权重（更小=腰部动得更少）
        const armGain = 1.0;    // 手臂权重
        
        // 迭代参数
        const maxIter = 200;
        const tol = 1e-16;
        const damping = 0.00001;
        const maxStep = 0.003;
        
        const qInitial = [...currentQ];
        let q = [...qInitial];
        let stopReason = 'max_iter';
        
        for (let iter = 0; iter < maxIter; iter++) {
            // === 计算任务1误差：绝对位姿 + 相对姿态保持（最高优先级）===
            const absNow = this._getHumanoidAbsolutePose(q);
            const relNow = this._getHumanoidRelativePose(q);
            if (!absNow || !relNow) {
                stopReason = 'fk_fail';
                break;
            }
            
            // 处理DQ双覆盖 - 绝对位姿
            let dotAbs = 0;
            for (let i = 0; i < 8; i++) dotAbs += absCurrentArray[i] * absNow[i];
            const signAbs = dotAbs < 0 ? -1 : 1;
            
            // 处理四元数双覆盖 - 相对姿态
            const relRotNow = relNow.slice(0, 4);
            let dotRot = 0;
            for (let i = 0; i < 4; i++) dotRot += relRotTarget[i] * relRotNow[i];
            const signRot = dotRot < 0 ? -1 : 1;
            
            // 任务1误差：绝对位姿(8维) + 相对姿态(4维) = 12维
            const e1 = [];
            let e1Norm2 = 0;
            // 绝对位姿误差
            for (let i = 0; i < 8; i++) {
                const err = absCurrentArray[i] - signAbs * absNow[i];
                e1.push(err);
                e1Norm2 += err * err;
            }
            // 相对姿态误差（旋转四元数部分）
            for (let i = 0; i < 4; i++) {
                const err = relRotTarget[i] - signRot * relRotNow[i];
                e1.push(err);
                e1Norm2 += err * err;
            }
            
            // === 计算任务2误差：相对距离跟踪 ===
            const dqRelObjNow = DQ.createFromArray(relNow);
            const tNow = DQ.getTranslation(dqRelObjNow);
            const d2Now = (tNow[1] || 0) ** 2 + (tNow[2] || 0) ** 2 + (tNow[3] || 0) ** 2;
            const e2 = d2Target - d2Now;  // 距离平方误差（标量）
            const e2Norm2 = e2 * e2;
            
            // 收敛检查
            if (e1Norm2 < tol && e2Norm2 < tol) {
                stopReason = 'converged';
                break;
            }
            
            if (iter === 0 || iter === maxIter - 1) {
                console.log('零空间距离控制: iter=', iter, 'e_abs+rot=', Math.sqrt(e1Norm2).toFixed(6), 'e_d2=', Math.abs(e2).toFixed(6));
            }
            
            // === 计算雅可比矩阵 ===
            // J1: 绝对位姿雅可比(8行) + 相对姿态雅可比(4行) = 12 x activeCount
            const JabsFull = this._getHumanoidAbsolutePoseJacobian(q);
            const JrelFull = this._getHumanoidRelativePoseJacobian(q);
            if (!JabsFull || !JrelFull) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            const J1 = [];
            // 添加绝对位姿雅可比 (8行)
            for (let i = 0; i < 8; i++) {
                J1.push(JabsFull[i].slice(armStart, armStart + activeCount));
            }
            // 添加相对姿态雅可比 (前4行，对应旋转部分)
            for (let i = 0; i < 4; i++) {
                J1.push(JrelFull[i].slice(armStart, armStart + activeCount));
            }
            
            // J2: 距离雅可比 (1 x activeCount)
            const Jd2Full = window.DQModule.Kinematics.distanceJacobian(JrelFull, relNow);
            const jacobianD2Row = Array.isArray(Jd2Full) && Jd2Full[0] ? [...Jd2Full[0]] : null;
            if (!jacobianD2Row) {
                stopReason = 'jacobian_fail';
                break;
            }
            const J2 = [jacobianD2Row.slice(armStart, armStart + activeCount)];
            
            // === 计算关节权重矩阵 W ===
            const W = [];
            for (let i = 0; i < activeCount; i++) {
                const isWaistJoint = includeWaist && i < waistCount;
                W.push(isWaistJoint ? waistGain : armGain);
            }
            
            // === 计算任务1（绝对位姿+相对姿态）的加权伪逆和零空间投影 ===
            const J1Pinv = this._weightedDampedPseudoInverse(J1, W, damping);
            if (!J1Pinv) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            // P1 = I - J1_pinv * J1
            const P1 = this._computeNullspaceProjection(J1Pinv, J1, activeCount);
            
            // === 计算任务2在任务1零空间中的伪逆 ===
            // J2_bar = J2 * P1
            const J2Bar = this._matMul(J2, P1);
            const J2BarPinv = this._weightedDampedPseudoInverse(J2Bar, W, damping);
            if (!J2BarPinv) {
                stopReason = 'jacobian_fail';
                break;
            }
            
            // === 计算增广雅可比的零空间投影 P12 ===
            const P12 = this._computeNullspaceProjection2(P1, J2BarPinv, J2Bar, activeCount);
            
            // === 计算任务3：关节限位避免梯度 ===
            const activeLimits = jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null;
            const gradLimit = this._computeJointLimitGradient(q.slice(armStart, armStart + activeCount), activeLimits, activeCount);
            
            // === 计算关节速度增量 ===
            // dq = J1^+ * κ1 * e1 + P1 * J2Bar^+ * κ2 * e2 + P12 * κ3 * gradLimit
            const dq = new Array(activeCount).fill(0);
            
            // 任务1贡献：J1^+ * κ1 * e1 (J1Pinv是 activeCount x 12)
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < 12; j++) {
                    dq[i] += J1Pinv[i][j] * kappa1 * e1[j];
                }
            }
            
            // 任务2贡献：P1 * J2Bar^+ * κ2 * e2
            const dq2_temp = new Array(activeCount).fill(0);
            for (let i = 0; i < activeCount; i++) {
                // J2BarPinv 是 activeCount x 1，e2 是标量
                dq2_temp[i] = J2BarPinv[i][0] * kappa2 * e2;
            }
            for (let i = 0; i < activeCount; i++) {
                for (let j = 0; j < activeCount; j++) {
                    dq[i] += P1[i][j] * dq2_temp[j];
                }
            }
            
            // 任务3贡献：P12 * κ3 * gradLimit
            if (enableJointLimitAvoidance) {
                for (let i = 0; i < activeCount; i++) {
                    for (let j = 0; j < activeCount; j++) {
                        dq[i] += P12[i][j] * kappa3 * gradLimit[j];
                    }
                }
            }
            
            // 应用关节权重并限制步长
            for (let i = 0; i < activeCount; i++) {
                let step = dq[i] * W[i];
                step = Math.max(-maxStep, Math.min(maxStep, step));
                q[armStart + i] += step;
            }
            
            // 应用关节限位（硬限位始终生效）
            if (activeLimits) {
                for (let i = 0; i < activeCount && i < activeLimits.length; i++) {
                    const lim = activeLimits[i];
                    if (lim) {
                        q[armStart + i] = Math.max(lim.lower, Math.min(lim.upper, q[armStart + i]));
                    }
                }
            }
        }

        const absEndArray = this._getHumanoidAbsolutePose(q);
        const relEndArray = this._getHumanoidRelativePose(q);
        const finalCmdNorm = relEndArray
            ? Math.abs(d2Target - (() => {
                const tEnd = DQ.getTranslation(DQ.createFromArray(relEndArray));
                return (tEnd[1] || 0) ** 2 + (tEnd[2] || 0) ** 2 + (tEnd[3] || 0) ** 2;
            })())
            : Number.POSITIVE_INFINITY;
        const finalInvariantNorm = (() => {
            const invariantTerms = [];
            if (absEndArray) {
                invariantTerms.push(...this._signedPoseError(absCurrentArray, absEndArray));
            }
            if (relEndArray) {
                const relRotEnd = relEndArray.slice(0, 4);
                let dotRotEnd = 0;
                for (let i = 0; i < 4; i++) dotRotEnd += (relRotTarget[i] || 0) * (relRotEnd[i] || 0);
                const signRotEnd = dotRotEnd < 0 ? -1 : 1;
                for (let i = 0; i < 4; i++) {
                    invariantTerms.push((relRotTarget[i] || 0) - signRotEnd * (relRotEnd[i] || 0));
                }
            }
            return vectorNorm(invariantTerms);
        })();
        const messageMap = {
            converged: '距离误差已收敛。',
            practical_converged: '距离任务达到实用收敛阈值。',
            command_mismatch: '无法到达该位置（目标未达成）。',
            max_iter: '达到最大迭代次数。',
            fk_fail: '姿态计算失败，提前终止。',
            jacobian_fail: '雅可比计算失败，提前终止。'
        };
        const resolved = resolveHumanoidJogCommandResult({
            stopReason,
            initialCommandNorm: initialCmdNorm,
            finalCommandNorm: finalCmdNorm,
            finalInvariantNorm,
            hardCommandTol: hardCmdTol,
            hardInvariantTol: hardInvariantTol,
            practicalCommandTol: practicalCmdTol,
            practicalInvariantTol: practicalInvariantTol,
            mismatchTol,
            mismatchRequiredReduction,
            singleStepMismatchRequiredReduction: Number.isFinite(cfg.qpSingleStepMismatchRequiredReduction)
                ? Math.min(0.99, Math.max(0, cfg.qpSingleStepMismatchRequiredReduction))
                : 0.05,
            q,
            qInitial,
            messageMap,
            defaultMessage: '零空间投影 已执行。'
        });
        if (resolved.outcome.reason === 'command_mismatch') {
            this._notifyHumanoidJogUnreachable('无法到达该位置');
        }
        this.lastIKSolveStatus = {
            solver: 'priority',
            mode: 'humanoid_distance',
            converged: resolved.outcome.converged,
            reason: resolved.outcome.reason,
            message: resolved.message,
            finalResidual: finalCmdNorm,
            invariantResidual: finalInvariantNorm
        };

        // 更新
        for (let i = 0; i < window.humanoidQ.length && i < resolved.qApplied.length; i++) {
            window.humanoidQ[i] = resolved.qApplied[i];
        }
        if (window.clampJointAngles && this.humanoidURDF && window.customRobotConfig) {
            const clamped = window.clampJointAngles(resolved.qApplied, this.humanoidJointNames || [], this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        this._publishCurrentPriorityCCTSMDiagnostics({
            modeTag: 'distance-target',
            mode: 'distance',
            q: Array.isArray(window.humanoidQ) ? window.humanoidQ.slice() : resolved.qApplied,
            armStart,
            activeCount,
            activeLimits: jointLimits ? jointLimits.slice(armStart, armStart + activeCount) : null,
            includeWaist,
            jointLimitAvoidance: enableJointLimitAvoidance,
            message: resolved.message
        });
        return true;
    }
    
    updateHumanoidJointsFromAbsolutePoseDelta(delta, currentQ, enableJointLimitAvoidance = true) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !currentQ || currentQ.length === 0) return;
        
        const { start: armStart, end: armEnd, count: armCount } = this._humanoidArmIndices();
        if (armCount <= 0) return;
        
        // 获取当前绝对位姿
        const dqAbsArray = this._getHumanoidAbsolutePose(currentQ);
        if (!dqAbsArray) return;
        
        // 计算目标位姿
        const dqAbsCurrent = DQ.createFromArray(dqAbsArray);
        const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
        const dqTargetDQ = DQ.multiply(dqDelta, dqAbsCurrent);
        const dqTargetArray = DQ.toArray(dqTargetDQ);
        
        // 保存当前相对位姿作为约束目标
        const relTarget = this._getHumanoidRelativePose(currentQ);
        if (!relTarget) return;
        
        // 获取关节限位（用于硬限位约束）
        const jointLimits = this._getHumanoidJointLimits();
        
        // 迭代参数
        const maxIter = 50;
        const tol = 1e-16;
        const damping = 0.8;
        const kappa3 = 0.05;  // 关节限位避免增益
        const lambdaReg = 0.01;  // 正则化系数，处理DQ冗余
        
        let q = [...currentQ];
        
        for (let iter = 0; iter < maxIter; iter++) {
            // 获取当前位姿
            const absNow = this._getHumanoidAbsolutePose(q);
            const relNow = this._getHumanoidRelativePose(q);
            if (!absNow || !relNow) break;
            
            // 计算误差
            const errAbs = [];
            const errRel = [];
            for (let i = 0; i < 8; i++) {
                errAbs.push(dqTargetArray[i] - absNow[i]);
                errRel.push(relTarget[i] - relNow[i]);
            }
            
            // 检查收敛
            const errNorm = Math.sqrt(errAbs.reduce((s, e) => s + e * e, 0) + errRel.reduce((s, e) => s + e * e, 0));
            if (errNorm < tol) break;
            
            // 获取雅可比
            const Jabs = this._getHumanoidAbsolutePoseJacobian(q);
            const Jrel = this._getHumanoidRelativePoseJacobian(q);
            if (!Jabs || !Jrel) break;
            
            // 提取双臂关节对应的列
            const JabsArm = [];
            const JrelArm = [];
            for (let i = 0; i < 8; i++) {
                JabsArm.push(Jabs[i].slice(armStart, armEnd));
                JrelArm.push(Jrel[i].slice(armStart, armEnd));
            }
            
            // 堆叠雅可比矩阵和误差
            const Jstack = [...JabsArm, ...JrelArm];
            const errStack = [...errAbs, ...errRel];
            
            // 使用阻尼伪逆求解（处理DQ的8维表示冗余）
            // DQ实际是6维，所以16维表示中有4维冗余，需要正则化
            const JstackPinv = this._dampedPseudoInverse(Jstack, lambdaReg);
            if (!JstackPinv) break;
            
            // 计算基础关节增量
            const dqBase = new Array(armCount).fill(0);
            for (let i = 0; i < armCount; i++) {
                for (let j = 0; j < 16; j++) {
                    dqBase[i] += JstackPinv[i][j] * errStack[j];
                }
            }
            
            // 零空间投影做关节限位避免
            // 实际任务是12维（6+6），关节14维，有2维零空间
            // 由于DQ冗余，雅可比矩阵实际秩约为12
            if (enableJointLimitAvoidance && jointLimits) {
                const P = this._computeNullspaceProjection(JstackPinv, Jstack, armCount);
                
                // 计算关节限位梯度
                const gradLimit = this._computeJointLimitGradient(
                    q.slice(armStart, armEnd), 
                    jointLimits.slice(armStart, armEnd), 
                    armCount
                );
                
                // 在零空间中添加限位避免（使用较小的增益防止泄漏）
                const nullspaceContrib = new Array(armCount).fill(0);
                for (let i = 0; i < armCount; i++) {
                    for (let j = 0; j < armCount; j++) {
                        nullspaceContrib[i] += P[i][j] * kappa3 * gradLimit[j];
                    }
                }
                
                // 验证零空间运动不会显著影响相对位姿
                // 计算 Jrel * nullspaceContrib 看是否接近零
                let relImpact = 0;
                for (let i = 0; i < 8; i++) {
                    let sum = 0;
                    for (let j = 0; j < armCount; j++) {
                        sum += JrelArm[i][j] * nullspaceContrib[j];
                    }
                    relImpact += sum * sum;
                }
                
                // 只有当对相对位姿影响很小时才应用零空间运动
                if (Math.sqrt(relImpact) < 0.01) {
                    for (let i = 0; i < armCount; i++) {
                        dqBase[i] += nullspaceContrib[i];
                    }
                }
            }
            
            // 应用关节增量
            for (let i = 0; i < armCount; i++) {
                q[armStart + i] += dqBase[i] * damping;
            }
            
            // 硬限位
            if (jointLimits) {
                for (let i = armStart; i < armEnd && i < jointLimits.length; i++) {
                    const lim = jointLimits[i];
                    if (lim) {
                        q[i] = Math.max(lim.lower, Math.min(lim.upper, q[i]));
                    }
                }
            }
        }
        
        // === 最终相对位姿补偿 ===
        // 检查相对位姿是否有偏移，如果有则额外修正
        const relFinal = this._getHumanoidRelativePose(q);
        if (relFinal) {
            let relErr2 = 0;
            for (let i = 0; i < 8; i++) {
                relErr2 += (relTarget[i] - relFinal[i]) ** 2;
            }
            
            // 如果相对位姿偏差超过阈值，进行额外的修正迭代
            if (Math.sqrt(relErr2) > 1e-6) {
                const correctionIter = 20;
                const correctionDamping = 0.5;
                
                for (let iter = 0; iter < correctionIter; iter++) {
                    const relNow = this._getHumanoidRelativePose(q);
                    if (!relNow) break;
                    
                    // 只修正相对位姿
                    const errRel = [];
                    let errNorm2 = 0;
                    for (let i = 0; i < 8; i++) {
                        const e = relTarget[i] - relNow[i];
                        errRel.push(e);
                        errNorm2 += e * e;
                    }
                    if (errNorm2 < 1e-12) break;
                    
                    const Jrel = this._getHumanoidRelativePoseJacobian(q);
                    if (!Jrel) break;
                    
                    const JrelArm = [];
                    for (let i = 0; i < 8; i++) {
                        JrelArm.push(Jrel[i].slice(armStart, armEnd));
                    }
                    
                    // 只用相对位姿雅可比求解
                    const JrelPinv = this._dampedPseudoInverse(JrelArm, 0.01);
                    if (!JrelPinv) break;
                    
                    for (let i = 0; i < armCount; i++) {
                        let sum = 0;
                        for (let j = 0; j < 8; j++) {
                            sum += JrelPinv[i][j] * errRel[j];
                        }
                        q[armStart + i] += sum * correctionDamping;
                    }
                    
                    // 硬限位
                    if (jointLimits) {
                        for (let i = armStart; i < armEnd && i < jointLimits.length; i++) {
                            const lim = jointLimits[i];
                            if (lim) {
                                q[i] = Math.max(lim.lower, Math.min(lim.upper, q[i]));
                            }
                        }
                    }
                }
            }
        }
        
        // 更新
        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(q, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
    }
    
    updateHumanoidJointsFromRelativePoseDelta(delta, currentQ) {
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !currentQ || currentQ.length === 0) return;
        
        const { start: armStart, end: armEnd, count: armCount } = this._humanoidArmIndices();
        if (armCount <= 0) return;
        
        // 获取当前相对位姿
        const dqRelArray = this._getHumanoidRelativePose(currentQ);
        if (!dqRelArray) return;
        
        // 计算目标相对位姿
        const dqRelCurrent = DQ.createFromArray(dqRelArray);
        const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
        const dqTargetDQ = DQ.multiply(dqDelta, dqRelCurrent);
        const dqTargetArray = DQ.toArray(dqTargetDQ);
        
        // 保存当前绝对位姿作为约束目标
        const absTarget = this._getHumanoidAbsolutePose(currentQ);
        if (!absTarget) return;
        
        // 迭代参数
        const maxIter = 50;
        const tol = 1e-16;
        const damping = 0.8;
        
        let q = [...currentQ];
        
        for (let iter = 0; iter < maxIter; iter++) {
            // 获取当前位姿
            const relNow = this._getHumanoidRelativePose(q);
            const absNow = this._getHumanoidAbsolutePose(q);
            if (!relNow || !absNow) break;
            
            // 计算误差
            const errRel = [];
            const errAbs = [];
            for (let i = 0; i < 8; i++) {
                errRel.push(dqTargetArray[i] - relNow[i]);
                errAbs.push(absTarget[i] - absNow[i]);
            }
            
            // 检查收敛
            const errNorm = Math.sqrt(errRel.reduce((s, e) => s + e * e, 0) + errAbs.reduce((s, e) => s + e * e, 0));
            if (errNorm < tol) break;
            
            // 获取雅可比
            const Jrel = this._getHumanoidRelativePoseJacobian(q);
            const Jabs = this._getHumanoidAbsolutePoseJacobian(q);
            if (!Jrel || !Jabs) break;
            
            // 提取双臂关节对应的列
            const JrelArm = [];
            const JabsArm = [];
            for (let i = 0; i < 8; i++) {
                JrelArm.push(Jrel[i].slice(armStart, armEnd));
                JabsArm.push(Jabs[i].slice(armStart, armEnd));
            }
            
            // 堆叠雅可比矩阵和误差
            const Jstack = [...JrelArm, ...JabsArm];
            const errStack = [...errRel, ...errAbs];
            
            // 伪逆求解
            const JstackPinv = this.pseudoInverse(Jstack);
            
            // 计算关节增量
            for (let i = 0; i < armCount; i++) {
                let sum = 0;
                for (let j = 0; j < 16; j++) {
                    sum += JstackPinv[i][j] * errStack[j];
                }
                q[armStart + i] += sum * damping;
            }
        }
        
        // 更新
        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(q, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
    }
    
    // ---------- 人形局部控制（腰部=effector0，左臂=effector1，右臂=effector2） ----------

    /**
     * 获取人形机器人腰部关节索引范围
     * @returns {{start: number, end: number, count: number}}
     */
    _humanoidWaistIndices() {
        const w = this.humanoidWaistLength;
        return { start: 0, end: w, count: w };
    }

    /**
     * 人形机器人腰部 body_link 位姿控制（仅移动腰部关节）
     * @param {number[]} dqTargetArray - 目标 body_link 位姿 DQ 8元素
     * @param {number[]} currentQ - 当前全关节角
     * @returns {boolean}
     */
    updateHumanoidWaistPoseTarget(dqTargetArray, currentQ) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        if (typeof this.humanoidModel.fkmEffector !== 'function' || typeof this.humanoidModel.poseJacobianEffector !== 'function') return false;

        const { start: waistStart, end: waistEnd, count: waistCount } = this._humanoidWaistIndices();
        if (waistCount <= 0) return false;

        const jointLimits = this._getHumanoidJointLimits();
        const waistLimits = jointLimits ? jointLimits.slice(waistStart, waistEnd) : null;

        const epsilon = 1e-5;
        const maxIterations = 80;
        const damping = 0.5;
        const maxJointStepPerIter = 0.08;

        let q = [...currentQ];

        for (let iter = 0; iter < maxIterations; iter++) {
            const currentPose = this.humanoidModel.fkmEffector(q, 0);
            if (!currentPose) return false;

            // DQ 双覆盖处理：目标与当前符号对齐，避免误差突变。
            let dot = 0;
            for (let i = 0; i < 8; i++) dot += (dqTargetArray[i] || 0) * (currentPose[i] || 0);
            const sign = dot < 0 ? -1 : 1;

            const err = [];
            let errNorm2 = 0;
            for (let i = 0; i < 8; i++) {
                const e = (dqTargetArray[i] || 0) - sign * (currentPose[i] || 0);
                err.push(e);
                errNorm2 += e * e;
            }
            if (Math.sqrt(errNorm2) <= epsilon) break;

            const J = this.humanoidModel.poseJacobianEffector(q, 0);
            if (!J) return false;

            const Jwaist = [];
            for (let i = 0; i < 8; i++) {
                Jwaist.push(J[i].slice(waistStart, waistEnd));
            }

            const JwaistPseudoInv = this.pseudoInverse(Jwaist);
            if (!JwaistPseudoInv) return false;

            for (let i = 0; i < waistCount; i++) {
                let step = 0;
                for (let j = 0; j < 8; j++) {
                    step += (JwaistPseudoInv[i][j] || 0) * err[j] * damping;
                }
                step = Math.max(-maxJointStepPerIter, Math.min(maxJointStepPerIter, step));
                q[waistStart + i] += step;
            }

            if (waistLimits) {
                for (let i = 0; i < waistCount && i < waistLimits.length; i++) {
                    const lim = waistLimits[i];
                    if (lim) {
                        q[waistStart + i] = Math.max(lim.lower, Math.min(lim.upper, q[waistStart + i]));
                    }
                }
            }
        }

        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(q, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }
    
    /**
     * 获取人形机器人单臂的关节索引范围
     * @param {number} armIndex - 1=左臂, 2=右臂
     * @returns {{start: number, end: number, count: number}}
     */
    _humanoidSingleArmIndices(armIndex) {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        const r = this.humanoidRightArmLength;
        if (armIndex === 1) {
            return { start: w, end: w + l, count: l };
        } else {
            return { start: w + l, end: w + l + r, count: r };
        }
    }

    /**
     * 获取单臂控制的活动关节索引（可选包含腰部）
     * @param {number} armIndex - 1=左臂, 2=右臂
     * @param {boolean} includeWaist - 是否包含腰部关节
     * @returns {number[]}
     */
    _humanoidSingleArmActiveIndices(armIndex, includeWaist = false) {
        const w = this.humanoidWaistLength;
        const l = this.humanoidLeftArmLength;
        const r = this.humanoidRightArmLength;
        const indices = [];

        if (includeWaist && w > 0) {
            for (let i = 0; i < w; i++) indices.push(i);
        }

        if (armIndex === 1) {
            for (let i = 0; i < l; i++) indices.push(w + i);
        } else {
            for (let i = 0; i < r; i++) indices.push(w + l + i);
        }

        return indices;
    }
    
    /**
     * 获取单臂位姿雅可比（仅该臂关节列非零）
     * @param {number[]} q - 全关节角
     * @param {number} armIndex - 1=左臂, 2=右臂
     * @returns {number[][]} 8xN 雅可比矩阵
     */
    _getHumanoidSingleArmPoseJacobian(q, armIndex) {
        if (!this.humanoidModel || typeof this.humanoidModel.poseJacobianEffector !== 'function') {
            return null;
        }
        try {
            return this.humanoidModel.poseJacobianEffector(q, armIndex);
        } catch (e) {
            console.warn('获取单臂雅可比失败:', e);
            return null;
        }
    }

    _humanoidFullQFromSingleArm(currentQ, xArm, armIndex, includeWaist = false) {
        const q = [...currentQ];
        const activeIndices = this._humanoidSingleArmActiveIndices(armIndex, includeWaist);
        for (let i = 0; i < activeIndices.length && i < xArm.length; i++) {
            q[activeIndices[i]] = xArm[i];
        }
        return q;
    }

    updateHumanoidSingleArmPoseTargetLBFGS(dqTargetArray, currentQ, armIndex, includeWaist = false) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) {
            return this._setIKFailureStatus({ solver: 'lbfgs', mode: `humanoid_arm_${armIndex}`, reason: 'invalid_state', message: '人形状态未初始化。' });
        }

        const getPose = armIndex === 1
            ? (q) => this._getHumanoidPose1(q)
            : (q) => this._getHumanoidPose2(q);
        const JfullGetter = (q) => this._getHumanoidSingleArmPoseJacobian(q, armIndex);
        const activeIndices = this._humanoidSingleArmActiveIndices(armIndex, includeWaist);
        if (activeIndices.length === 0) {
            return this._setIKFailureStatus({ solver: 'lbfgs', mode: `humanoid_arm_${armIndex}`, reason: 'no_active_joints', message: '当前单臂没有可用活动关节。' });
        }

        const x0 = activeIndices.map((idx) => currentQ[idx]);
        const poseTol = 1e-4;
        const f = (x) => {
            const q = this._humanoidFullQFromSingleArm(currentQ, x, armIndex, includeWaist);
            const pose = getPose(q);
            if (!pose) return 1e10;
            let sum = 0;
            for (let i = 0; i < 8; i++) {
                const e = (pose[i] || 0) - (dqTargetArray[i] || 0);
                sum += e * e;
            }
            return 0.5 * sum;
        };
        const grad = (x, out) => {
            const q = this._humanoidFullQFromSingleArm(currentQ, x, armIndex, includeWaist);
            const pose = getPose(q);
            const Jfull = JfullGetter(q);
            if (!pose || !Jfull || Jfull.length !== 8) {
                for (let i = 0; i < out.length; i++) out[i] = 0;
                return;
            }
            for (let i = 0; i < activeIndices.length; i++) {
                let s = 0;
                const jointCol = activeIndices[i];
                for (let j = 0; j < 8; j++) {
                    s += (Jfull[j][jointCol] || 0) * ((pose[j] || 0) - (dqTargetArray[j] || 0));
                }
                out[i] = s;
            }
        };

        const result = this.minimizeLBFGS(f, grad, x0, {
            maxIter: 120,
            gradTol: 1e-8,
            costTol: 0.5 * poseTol * poseTol,
            m: 10
        });

        const qSol = this._humanoidFullQFromSingleArm(currentQ, result.x, armIndex, includeWaist);
        for (let i = 0; i < window.humanoidQ.length && i < qSol.length; i++) {
            window.humanoidQ[i] = qSol[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(qSol, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();

        const finalResidual = this._computeArrayResidualNorm(getPose(window.humanoidQ), dqTargetArray);
        this.lastIKSolveStatus = {
            solver: 'lbfgs',
            mode: `humanoid_arm_${armIndex}`,
            ...result,
            finalResidual
        };
        return result.converged || finalResidual <= poseTol;
    }
    
    /**
     * 人形机器人单臂位姿控制（仅移动指定手臂，其他关节固定）
     * @param {number[]} dqTargetArray - 目标位姿 DQ 8元素
     * @param {number[]} currentQ - 当前全关节角
     * @param {number} armIndex - 1=左臂, 2=右臂
     * @param {boolean} includeWaist - 是否包含腰部关节，默认 false
     * @param {boolean} keepBodyZVertical - 是否保持 body_link z 轴竖直（仅 includeWaist=true 时有效）
     * @returns {boolean}
     */
    updateHumanoidSingleArmPoseTarget(dqTargetArray, currentQ, armIndex, includeWaist = false, keepBodyZVertical = false) {
        if (!this.humanoidModel || !window.humanoidQ || !currentQ || currentQ.length === 0) return false;
        const DQ = window.DQModule?.DQWrapper;
        const Kinematics = window.DQModule?.Kinematics;
        if (!DQ) return false;

        const activeIndices = this._humanoidSingleArmActiveIndices(armIndex, includeWaist);
        const activeCount = activeIndices.length;
        if (activeCount <= 0) return false;
        
        const epsilon = 1e-5;
        const maxIterations = 50;
        const damping = 0.5;
        const maxJointStepPerIter = 0.1;
        const kappaBodyZ = 0.8;
        const tolBodyZ = 1e-5;
        const waistCount = this.humanoidWaistLength;
        const enableBodyVerticalTask = keepBodyZVertical && includeWaist && waistCount > 0
            && Kinematics && typeof Kinematics.lineJacobian === 'function'
            && typeof this.humanoidModel.fkmEffector === 'function'
            && typeof this.humanoidModel.poseJacobianEffector === 'function';
        if (keepBodyZVertical && includeWaist && waistCount > 0 && !enableBodyVerticalTask && !this._missingSingleArmBodyZConstraintWarned) {
            console.warn('单臂+腰部控制：未启用 body_link z 轴竖直约束（缺少 Kinematics.lineJacobian 或腰部接口）');
            this._missingSingleArmBodyZConstraintWarned = true;
        }
        
        let q = [...currentQ];
        
        for (let iter = 0; iter < maxIterations; iter++) {
            // 获取当前位姿
            const currentPose = armIndex === 1 ? this._getHumanoidPose1(q) : this._getHumanoidPose2(q);
            if (!currentPose) return false;
            
            // 计算末端误差（处理 DQ 双覆盖，避免符号翻转导致突跳）
            let dot = 0;
            for (let i = 0; i < 8; i++) dot += (dqTargetArray[i] || 0) * (currentPose[i] || 0);
            const sign = dot < 0 ? -1 : 1;
            const err = [];
            for (let i = 0; i < 8; i++) {
                err.push((dqTargetArray[i] || 0) - sign * (currentPose[i] || 0));
            }

            // 获取雅可比
            const J = this._getHumanoidSingleArmPoseJacobian(q, armIndex);
            if (!J) return false;
            
            // 提取活动关节对应的列（8 x activeCount）
            const Jactive = [];
            for (let i = 0; i < 8; i++) {
                const row = [];
                for (let j = 0; j < activeCount; j++) {
                    row.push(J[i][activeIndices[j]] || 0);
                }
                Jactive.push(row);
            }

            let bodyErrNorm = 0;
            let errStack = err.slice();
            if (enableBodyVerticalTask) {
                const bodyState = this._getHumanoidBodyVerticalState(q, 0, currentQ.length);
                if (!bodyState) return false;
                bodyErrNorm = bodyState.errorNorm || 0;
                errStack.push(kappaBodyZ * (bodyState.error[0] || 0));

                const jBody = [];
                for (let j = 0; j < activeCount; j++) {
                    const qIdx = activeIndices[j];
                    jBody.push((bodyState.Jrows[0]?.[qIdx] || 0) * kappaBodyZ);
                }
                Jactive.push(jBody);
            }

            const armErrNorm = Math.sqrt(err.reduce((s, e) => s + e * e, 0));
            if (armErrNorm <= epsilon && (!enableBodyVerticalTask || bodyErrNorm <= tolBodyZ)) break;
            
            // 伪逆求解
            const JactivePseudoInv = this.pseudoInverse(Jactive);
            
            // 计算关节增量
            const deltaJoints = [];
            for (let i = 0; i < activeCount; i++) {
                let sum = 0;
                for (let j = 0; j < errStack.length; j++) {
                    sum += (JactivePseudoInv[i][j] || 0) * errStack[j] * damping;
                }
                deltaJoints.push(Math.max(-maxJointStepPerIter, Math.min(maxJointStepPerIter, sum)));
            }
            
            // 更新活动关节（含腰部或不含腰部）
            for (let i = 0; i < activeCount; i++) {
                q[activeIndices[i]] += deltaJoints[i];
            }
        }
        
        // 应用结果
        for (let i = 0; i < window.humanoidQ.length && i < q.length; i++) {
            window.humanoidQ[i] = q[i];
        }
        if (window.clampJointAngles && window.customRobotConfig && this.humanoidURDF) {
            const names = this.humanoidJointNames || [];
            const clamped = window.clampJointAngles(q, names, this.humanoidURDF, window.customRobotConfig);
            for (let i = 0; i < window.humanoidQ.length && i < clamped.length; i++) {
                window.humanoidQ[i] = clamped[i];
            }
        }
        this.setHumanoidJoints(window.humanoidQ);
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return true;
    }

    /**
     * 人形机器人腰部 body_link 平移控制（仅支持 X/Z）
     * @param {string} axis - 'x' | 'z'
     * @param {string} direction - 'positive' | 'negative'
     */
    moveHumanoidWaistBodyLinkAlongAxis(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        if (axis !== 'x' && axis !== 'z') return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ || !this.humanoidModel || typeof this.humanoidModel.fkmEffector !== 'function') return;

        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;

            const currentPose = this.humanoidModel.fkmEffector(window.humanoidQ, 0); // body_link
            if (!currentPose) return;

            const dqCurrent = DQ.createFromArray(currentPose);
            const rotationArray = DQ.getRotation(dqCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);

            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
            const worldMode = this.isWorldTranslationMode?.() ?? true;
            const dqTarget = worldMode ? DQ.multiply(dqDelta, dqCurrent) : DQ.multiply(dqCurrent, dqDelta);

            this.updateHumanoidWaistPoseTarget(DQ.toArray(dqTarget), window.humanoidQ);
        } catch (e) {
            console.error('移动腰部 body_link 失败:', e);
        }
    }
    
    /**
     * 人形机器人左臂平移控制
     */
    moveHumanoidLeftArmAlongAxis(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            
            // 获取左臂当前位姿
            const currentPose = this._getHumanoidPose1(window.humanoidQ);
            if (!currentPose) return;
            
            const dqCurrent = DQ.createFromArray(currentPose);
            const rotationArray = DQ.getRotation(dqCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
            
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
            
            const worldMode = this.isWorldTranslationMode?.() ?? true;
            const dqTarget = worldMode ? DQ.multiply(dqDelta, dqCurrent) : DQ.multiply(dqCurrent, dqDelta);
            
            this.updateHumanoidSingleArmPoseTarget(DQ.toArray(dqTarget), window.humanoidQ, 1);
        } catch (e) {
            console.error('移动左臂失败:', e);
        }
    }
    
    /**
     * 人形机器人右臂平移控制
     */
    moveHumanoidRightArmAlongAxis(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            
            // 获取右臂当前位姿
            const currentPose = this._getHumanoidPose2(window.humanoidQ);
            if (!currentPose) return;
            
            const dqCurrent = DQ.createFromArray(currentPose);
            const rotationArray = DQ.getRotation(dqCurrent);
            const rotationQuat = new THREE.Quaternion(rotationArray[1], rotationArray[2], rotationArray[3], rotationArray[0]);
            
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            const dqDelta = DQ.translation(delta.x, delta.y, delta.z);
            
            const worldMode = this.isWorldTranslationMode?.() ?? true;
            const dqTarget = worldMode ? DQ.multiply(dqDelta, dqCurrent) : DQ.multiply(dqCurrent, dqDelta);
            
            this.updateHumanoidSingleArmPoseTarget(DQ.toArray(dqTarget), window.humanoidQ, 2);
        } catch (e) {
            console.error('移动右臂失败:', e);
        }
    }
    
    /**
     * 人形机器人左臂旋转控制
     */
    rotateHumanoidLeftArmAroundAxis(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            const currentPose = this._getHumanoidPose1(window.humanoidQ);
            if (!currentPose) return;
            
            const dqCurrent = DQ.createFromArray(currentPose);
            
            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return;
            
            const dqTarget = this.combineDQWithTeachOrder(rotationDQ, dqCurrent, false);
            this.updateHumanoidSingleArmPoseTarget(DQ.toArray(dqTarget), window.humanoidQ, 1);
        } catch (e) {
            console.error('旋转左臂失败:', e);
        }
    }
    
    /**
     * 人形机器人右臂旋转控制
     */
    rotateHumanoidRightArmAroundAxis(axis, direction) {
        if (!this.humanoidMode || !window.humanoidQ?.length) return;
        const DQ = window.DQModule?.DQWrapper;
        if (!DQ) return;
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            const currentPose = this._getHumanoidPose2(window.humanoidQ);
            if (!currentPose) return;
            
            const dqCurrent = DQ.createFromArray(currentPose);
            
            let rotationDQ;
            if (axis === 'rx') rotationDQ = DQ.rotation(rotationAngle, 1, 0, 0);
            else if (axis === 'ry') rotationDQ = DQ.rotation(rotationAngle, 0, 1, 0);
            else if (axis === 'rz') rotationDQ = DQ.rotation(rotationAngle, 0, 0, 1);
            else return;
            
            const dqTarget = this.combineDQWithTeachOrder(rotationDQ, dqCurrent, false);
            this.updateHumanoidSingleArmPoseTarget(DQ.toArray(dqTarget), window.humanoidQ, 2);
        } catch (e) {
            console.error('旋转右臂失败:', e);
        }
    }
    
    /**
     * 沿指定轴方向移动末端执行器（示教器模式）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveEndEffectorAlongAxis(axis, direction) {
        if (this.dualArmMode || !this.endEffector) {
            console.warn('示教器控制仅支持单臂模式');
            return;
        }
        
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) {
            console.warn('机器人未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationArray = window.currentRobot?.getEndEffectorPose
                ? window.currentRobot.getEndEffectorPose(window.jointAngles).rotation
                : window.DQModule.DQWrapper.getRotation(window.DQModule.DQWrapper.createFromArray(window.currentRobot.fkm(window.jointAngles)));
            const rotationQuat = new THREE.Quaternion(
                rotationArray[1] || 0,
                rotationArray[2] || 0,
                rotationArray[3] || 0,
                rotationArray[0] || 1
            );
            const delta = this.getTranslationDirection(axis, rotationQuat).multiplyScalar(stepSize * directionSign);
            this.updateJointsFromEndEffectorDelta(delta);
        } catch (error) {
            console.error('移动末端执行器失败:', error);
        }
    }
    
    /**
     * 绕指定轴旋转末端执行器（示教器模式）
     * 参考 MATLAB: dqad = DQ([cos(pi/16);0;sin(pi/16);0]) .* dqad_ant;
     * 
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateEndEffectorAroundAxis(axis, direction) {
        if (this.dualArmMode || !this.endEffector) {
            console.warn('示教器控制仅支持单臂模式');
            return;
        }
        
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) {
            console.warn('机器人未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            // 旋转角度（弧度），pi/16 ≈ 11.25度
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            // 获取当前末端执行器位姿
            const dqCurrentArray = window.currentRobot.fkm(window.jointAngles);
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            
            // 创建旋转双四元数（在末端执行器局部坐标系中）
            // 根据用户提供的MATLAB代码：DQ([cos(pi/16);0;sin(pi/16);0])
            // 这是绕Y轴旋转，格式为 [w, x, y, z]
            let rotationDQ;
            if (axis === 'rx') {
                // 绕X轴旋转：[cos(θ), sin(θ), 0, 0]
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                // 绕Y轴旋转：[cos(θ), 0, sin(θ), 0]
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                // 绕Z轴旋转：[cos(θ), 0, 0, sin(θ)]
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            // 计算目标位姿：dq_target = dq_rotation * dq_current
            // 参考 MATLAB: dqad = DQ([cos(pi/16);0;sin(pi/16);0]) .* dqad_ant;
            const dqTarget = this.combineDQWithTeachOrder(rotationDQ, dqCurrent, false);
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            // 迭代求解逆运动学（参考 MATLAB 的 while 循环）
            const epsilon = 0.00001;  // 停止条件：误差范数小于此值
            const maxIterations = 50;  // 最大迭代次数
            const damping = 0.5;  // 阻尼系数（与MATLAB一致）
            
            // 使用局部变量进行迭代
            let q = [...window.jointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;
            
            while (errNorm > epsilon && iteration < maxIterations) {
                // 获取当前位姿
                const dqCurrentIterArray = window.currentRobot.fkm(q);
                const dqCurrentIter = window.DQModule.DQWrapper.createFromArray(dqCurrentIterArray);
                
                // 计算误差向量
                const err = [];
                for (let i = 0; i < 8; i++) {
                    err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
                }
                
                // 计算误差范数
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                
                if (errNorm <= epsilon) {
                    break;  // 误差足够小，退出循环
                }
                
                // 获取雅可比矩阵
                const jacobian = window.currentRobot.poseJacobian(q);
                if (!jacobian || jacobian.length !== 8) {
                    console.error('雅可比矩阵维度错误');
                    return;
                }
                
                // 计算雅可比伪逆
                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                
                // 计算关节角度增量
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    deltaJoints.push(sum);
                }
                
                // 更新关节角度
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                
                iteration++;
            }
            
            // 限制关节角度在限位范围内
            if (window.customRobotConfig && window.customRobotConfig.joint_chain) {
                const jointNames = window.customRobotConfig.joint_chain.joints || [];
                q = window.clampJointAngles(q, jointNames, this.urdfRobot, window.customRobotConfig);
            }
            
            // 更新全局关节角度
            if (!window.jointAngles || window.jointAngles.length !== q.length) {
                if (window.updateGlobalVariables) {
                    window.updateGlobalVariables();
                }
            }
            
            // 更新关节角度
            for (let i = 0; i < window.jointAngles.length && i < q.length; i++) {
                window.jointAngles[i] = q[i];
            }
            
            // 同步到 main.js
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 更新机器人位姿
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
            
            // 确保拖动控制跟随末端执行器
            this.updateGizmoPosition();
            
        } catch (error) {
            console.error('旋转末端执行器失败:', error);
        }
    }
    
    /**
     * 沿指定轴方向移动机器人1末端执行器（双臂模式下的单臂控制）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveRobot1EndEffectorAlongAxis(axis, direction) {
        if (!this.dualArmMode || !window.robot1 || !window.jointAngles1 || window.jointAngles1.length === 0) {
            console.warn('机器人1未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = this.getTranslationDirection(axis).multiplyScalar(stepSize * directionSign);
            this.updateRobot1JointsFromEndEffectorDelta(delta);
        } catch (error) {
            console.error('移动机器人1末端执行器失败:', error);
        }
    }
    
    /**
     * 绕指定轴旋转机器人1末端执行器（双臂模式下的单臂控制）
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateRobot1EndEffectorAroundAxis(axis, direction) {
        if (!this.dualArmMode || !window.robot1 || !window.jointAngles1 || window.jointAngles1.length === 0) {
            console.warn('机器人1未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            const dqCurrentArray = window.robot1.fkm(window.jointAngles1);
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            
            let rotationDQ;
            if (axis === 'rx') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            const dqTarget = this.combineDQWithTeachOrder(rotationDQ, dqCurrent, false);
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            this.updateRobot1JointsWithPseudoInverse(dqTargetArray);
        } catch (error) {
            console.error('旋转机器人1末端执行器失败:', error);
        }
    }
    
    /**
     * 沿指定轴方向移动机器人2末端执行器（双臂模式下的单臂控制）
     * @param {string} axis - 轴名称：'x', 'y', 或 'z'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    moveRobot2EndEffectorAlongAxis(axis, direction) {
        if (!this.dualArmMode || !window.robot2 || !window.jointAngles2 || window.jointAngles2.length === 0) {
            console.warn('机器人2未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            const stepSize = this.getTeachTranslationStep();
            const directionSign = direction === 'positive' ? 1 : -1;
            const delta = this.getTranslationDirection(axis).multiplyScalar(stepSize * directionSign);
            this.updateRobot2JointsFromEndEffectorDelta(delta);
        } catch (error) {
            console.error('移动机器人2末端执行器失败:', error);
        }
    }
    
    /**
     * 绕指定轴旋转机器人2末端执行器（双臂模式下的单臂控制）
     * @param {string} axis - 旋转轴名称：'rx', 'ry', 或 'rz'
     * @param {string} direction - 方向：'positive' 或 'negative'
     */
    rotateRobot2EndEffectorAroundAxis(axis, direction) {
        if (!this.dualArmMode || !window.robot2 || !window.jointAngles2 || window.jointAngles2.length === 0) {
            console.warn('机器人2未初始化');
            return;
        }
        
        if (!window.DQModule || !window.DQModule.DQWrapper) {
            console.error('DQ Robotics 模块未加载');
            return;
        }
        
        try {
            const angle = this.getTeachRotationAngle();
            const directionSign = direction === 'positive' ? 1 : -1;
            const rotationAngle = angle * directionSign;
            
            const dqCurrentArray = window.robot2.fkm(window.jointAngles2);
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            
            let rotationDQ;
            if (axis === 'rx') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 1, 0, 0);
            } else if (axis === 'ry') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 1, 0);
            } else if (axis === 'rz') {
                rotationDQ = window.DQModule.DQWrapper.rotation(rotationAngle, 0, 0, 1);
            } else {
                console.error('无效的旋转轴名称:', axis);
                return;
            }
            
            const dqTarget = this.combineDQWithTeachOrder(rotationDQ, dqCurrent, false);
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            this.updateRobot2JointsWithPseudoInverse(dqTargetArray);
        } catch (error) {
            console.error('旋转机器人2末端执行器失败:', error);
        }
    }
    
    /**
     * 从机器人1末端执行器位移更新关节角度
     * @param {THREE.Vector3} delta - 末端执行器的位移（世界坐标系，单位：米）
     */
    updateRobot1JointsFromEndEffectorDelta(delta) {
        if (!window.robot1 || !window.jointAngles1 || window.jointAngles1.length === 0) return;
        
        try {
            if (!window.DQModule || !window.DQModule.DQWrapper) {
                console.error('DQ Robotics 模块未加载');
                return;
            }
            
            const dqCurrentArray = window.robot1.fkm(window.jointAngles1);
            if (!dqCurrentArray || dqCurrentArray.length !== 8) {
                console.error('无法获取当前位姿');
                return;
            }
            
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            const { multiply } = window.DQModule.DQWrapper;
            const worldMode = this.isWorldTranslationMode();
            let dqDelta;
            let dqTarget;
            if (worldMode) {
                dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
                dqTarget = multiply(dqDelta, dqCurrent);
            } else {
                const rotationArray = window.DQModule.DQWrapper.getRotation(dqCurrent);
                const rotationQuat = new THREE.Quaternion(
                    rotationArray[1] || 0,
                    rotationArray[2] || 0,
                    rotationArray[3] || 0,
                    rotationArray[0] || 1
                );
                const deltaLocal = delta.clone().applyQuaternion(rotationQuat.clone().conjugate());
                dqDelta = window.DQModule.DQWrapper.translation(deltaLocal.x, deltaLocal.y, deltaLocal.z);
                dqTarget = multiply(dqCurrent, dqDelta);
            }
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            this.updateRobot1JointsWithPseudoInverse(dqTargetArray);
        } catch (error) {
            console.error('更新机器人1关节角度失败:', error);
        }
    }
    
    /**
     * 从机器人2末端执行器位移更新关节角度
     * @param {THREE.Vector3} delta - 末端执行器的位移（世界坐标系，单位：米）
     */
    updateRobot2JointsFromEndEffectorDelta(delta) {
        if (!window.robot2 || !window.jointAngles2 || window.jointAngles2.length === 0) return;
        
        try {
            if (!window.DQModule || !window.DQModule.DQWrapper) {
                console.error('DQ Robotics 模块未加载');
                return;
            }
            
            const dqCurrentArray = window.robot2.fkm(window.jointAngles2);
            if (!dqCurrentArray || dqCurrentArray.length !== 8) {
                console.error('无法获取当前位姿');
                return;
            }
            
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            const { multiply } = window.DQModule.DQWrapper;
            const worldMode = this.isWorldTranslationMode();
            let dqDelta;
            let dqTarget;
            if (worldMode) {
                dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
                dqTarget = multiply(dqDelta, dqCurrent);
            } else {
                const rotationArray = window.DQModule.DQWrapper.getRotation(dqCurrent);
                const rotationQuat = new THREE.Quaternion(
                    rotationArray[1] || 0,
                    rotationArray[2] || 0,
                    rotationArray[3] || 0,
                    rotationArray[0] || 1
                );
                const deltaLocal = delta.clone().applyQuaternion(rotationQuat.clone().conjugate());
                dqDelta = window.DQModule.DQWrapper.translation(deltaLocal.x, deltaLocal.y, deltaLocal.z);
                dqTarget = multiply(dqCurrent, dqDelta);
            }
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            this.updateRobot2JointsWithPseudoInverse(dqTargetArray);
        } catch (error) {
            console.error('更新机器人2关节角度失败:', error);
        }
    }
    
    /**
     * 使用伪逆更新机器人1关节角度
     * @param {number[]} dqTargetArray - 目标位姿（8元素双四元数）
     */
    updateRobot1JointsWithPseudoInverse(dqTargetArray) {
        if (!window.robot1 || !window.jointAngles1 || window.jointAngles1.length === 0) return false;
        
        const epsilon = 0.00001;
        const maxIterations = 50;
        const damping = 0.5;
        
        let q = [...window.jointAngles1];
        let errNorm = epsilon + 1;
        let iteration = 0;
        
        while (errNorm > epsilon && iteration < maxIterations) {
            const dqCurrentIterArray = window.robot1.fkm(q);
            const err = [];
            for (let i = 0; i < 8; i++) {
                err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
            }
            
            errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
            if (errNorm <= epsilon) break;
            
            const jacobian = window.robot1.poseJacobian(q);
            if (!jacobian || jacobian.length !== 8) {
                console.error('雅可比矩阵维度错误');
                return false;
            }
            
            const jacobianPseudoInv = this.pseudoInverse(jacobian);
            const deltaJoints = [];
            for (let i = 0; i < jacobianPseudoInv.length; i++) {
                let sum = 0;
                for (let j = 0; j < err.length; j++) {
                    sum += jacobianPseudoInv[i][j] * err[j] * damping;
                }
                deltaJoints.push(sum);
            }
            
            for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                q[i] += deltaJoints[i];
            }
            
            iteration++;
        }
        
        if (window.customRobotConfig?.robot1) {
            const jointNames = window.customRobotConfig.robot1.joint_chain?.joints || [];
            if (jointNames.length === q.length && window.clampJointAngles) {
                q = window.clampJointAngles(q, jointNames, this.robot1URDF, window.customRobotConfig.robot1);
            }
        }
        
        for (let i = 0; i < window.jointAngles1.length && i < q.length; i++) {
            window.jointAngles1[i] = q[i];
        }
        
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return errNorm <= epsilon;
    }

    /**
     * 机器人1逆运动学：L-BFGS 最小化 0.5*||pose1(q)-target||^2
     */
    updateRobot1JointsWithLBFGS(dqTargetArray) {
        if (!window.robot1 || !window.jointAngles1 || window.jointAngles1.length === 0) return false;
        const robot = window.robot1;
        const target = dqTargetArray;
        const poseTol = 1e-4;
        const f = (q) => {
            const pose = robot.fkm(q);
            let sum = 0;
            for (let i = 0; i < 8; i++) sum += (pose[i] - target[i]) ** 2;
            return 0.5 * sum;
        };
        const grad = (q, out) => {
            const pose = robot.fkm(q);
            const err = [];
            for (let i = 0; i < 8; i++) err.push(pose[i] - target[i]);
            const J = robot.poseJacobian(q);
            if (!J || J.length !== 8) { for (let i = 0; i < out.length; i++) out[i] = 0; return; }
            for (let i = 0; i < out.length; i++) {
                let s = 0;
                for (let j = 0; j < 8; j++) s += (J[j][i] || 0) * err[j];
                out[i] = s;
            }
        };
        const result = this.minimizeLBFGS(f, grad, [...window.jointAngles1], {
            maxIter: 120,
            gradTol: 1e-8,
            costTol: 0.5 * poseTol * poseTol,
            m: 10
        });
        const { x, converged } = result;
        for (let i = 0; i < window.jointAngles1.length && i < x.length; i++) window.jointAngles1[i] = x[i];
        if (window.customRobotConfig?.robot1 && window.clampJointAngles) {
            const names = window.customRobotConfig.robot1.joint_chain?.joints || [];
            const clamped = window.clampJointAngles(x, names, this.robot1URDF, window.customRobotConfig.robot1);
            for (let i = 0; i < window.jointAngles1.length && i < clamped.length; i++) window.jointAngles1[i] = clamped[i];
        }
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        this.lastIKSolveStatus = {
            solver: 'lbfgs',
            ...result,
            finalResidual: this._computePoseResidualNorm(robot, window.jointAngles1, target)
        };
        return converged || this.lastIKSolveStatus.finalResidual <= poseTol;
    }
    
    /**
     * 使用伪逆更新机器人2关节角度
     * @param {number[]} dqTargetArray - 目标位姿（8元素双四元数）
     */
    updateRobot2JointsWithPseudoInverse(dqTargetArray) {
        if (!window.robot2 || !window.jointAngles2 || window.jointAngles2.length === 0) return false;
        
        const epsilon = 0.00001;
        const maxIterations = 50;
        const damping = 0.5;
        
        let q = [...window.jointAngles2];
        let errNorm = epsilon + 1;
        let iteration = 0;
        
        while (errNorm > epsilon && iteration < maxIterations) {
            const dqCurrentIterArray = window.robot2.fkm(q);
            const err = [];
            for (let i = 0; i < 8; i++) {
                err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
            }
            
            errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
            if (errNorm <= epsilon) break;
            
            const jacobian = window.robot2.poseJacobian(q);
            if (!jacobian || jacobian.length !== 8) {
                console.error('雅可比矩阵维度错误');
                return false;
            }
            
            const jacobianPseudoInv = this.pseudoInverse(jacobian);
            const deltaJoints = [];
            for (let i = 0; i < jacobianPseudoInv.length; i++) {
                let sum = 0;
                for (let j = 0; j < err.length; j++) {
                    sum += jacobianPseudoInv[i][j] * err[j] * damping;
                }
                deltaJoints.push(sum);
            }
            
            for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                q[i] += deltaJoints[i];
            }
            
            iteration++;
        }
        
        if (window.customRobotConfig?.robot2) {
            const jointNames = window.customRobotConfig.robot2.joint_chain?.joints || [];
            if (jointNames.length === q.length && window.clampJointAngles) {
                q = window.clampJointAngles(q, jointNames, this.robot2URDF, window.customRobotConfig.robot2);
            }
        }
        
        for (let i = 0; i < window.jointAngles2.length && i < q.length; i++) {
            window.jointAngles2[i] = q[i];
        }
        
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return errNorm <= epsilon;
    }

    /**
     * 机器人2逆运动学：L-BFGS 最小化 0.5*||pose2(q)-target||^2
     */
    updateRobot2JointsWithLBFGS(dqTargetArray) {
        if (!window.robot2 || !window.jointAngles2 || window.jointAngles2.length === 0) return false;
        const robot = window.robot2;
        const target = dqTargetArray;
        const poseTol = 1e-4;
        const f = (q) => {
            const pose = robot.fkm(q);
            let sum = 0;
            for (let i = 0; i < 8; i++) sum += (pose[i] - target[i]) ** 2;
            return 0.5 * sum;
        };
        const grad = (q, out) => {
            const pose = robot.fkm(q);
            const err = [];
            for (let i = 0; i < 8; i++) err.push(pose[i] - target[i]);
            const J = robot.poseJacobian(q);
            if (!J || J.length !== 8) { for (let i = 0; i < out.length; i++) out[i] = 0; return; }
            for (let i = 0; i < out.length; i++) {
                let s = 0;
                for (let j = 0; j < 8; j++) s += (J[j][i] || 0) * err[j];
                out[i] = s;
            }
        };
        const result = this.minimizeLBFGS(f, grad, [...window.jointAngles2], {
            maxIter: 120,
            gradTol: 1e-8,
            costTol: 0.5 * poseTol * poseTol,
            m: 10
        });
        const { x, converged } = result;
        for (let i = 0; i < window.jointAngles2.length && i < x.length; i++) window.jointAngles2[i] = x[i];
        if (window.customRobotConfig?.robot2 && window.clampJointAngles) {
            const names = window.customRobotConfig.robot2.joint_chain?.joints || [];
            const clamped = window.clampJointAngles(x, names, this.robot2URDF, window.customRobotConfig.robot2);
            for (let i = 0; i < window.jointAngles2.length && i < clamped.length; i++) window.jointAngles2[i] = clamped[i];
        }
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        this.lastIKSolveStatus = {
            solver: 'lbfgs',
            ...result,
            finalResidual: this._computePoseResidualNorm(robot, window.jointAngles2, target)
        };
        return converged || this.lastIKSolveStatus.finalResidual <= poseTol;
    }

    /**
     * 单臂模式下，使用伪逆更新关节角度
     * @param {number[]} dqTargetArray
     */
    updateSingleArmJointsWithPseudoInverse(dqTargetArray) {
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) {
            return false;
        }

        const epsilon = 0.00001;
        const maxIterations = 50;
        const damping = 0.5;

        let q = [...window.jointAngles];
        let errNorm = epsilon + 1;
        let iteration = 0;

        while (errNorm > epsilon && iteration < maxIterations) {
            const dqCurrentIterArray = window.currentRobot.fkm(q);
            const err = [];
            for (let i = 0; i < 8; i++) {
                err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
            }

            errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
            if (errNorm <= epsilon) break;

            const jacobian = window.currentRobot.poseJacobian(q);
            if (!jacobian || jacobian.length !== 8) {
                console.error('雅可比矩阵维度错误');
                return false;
            }

            const jacobianPseudoInv = this.pseudoInverse(jacobian);
            const deltaJoints = [];
            for (let i = 0; i < jacobianPseudoInv.length; i++) {
                let sum = 0;
                for (let j = 0; j < err.length; j++) {
                    sum += jacobianPseudoInv[i][j] * err[j] * damping;
                }
                deltaJoints.push(sum);
            }

            for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                q[i] += deltaJoints[i];
            }

            iteration++;
        }

        for (let i = 0; i < window.jointAngles.length && i < q.length; i++) {
            window.jointAngles[i] = q[i];
        }

        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        return errNorm <= epsilon;
    }

    /**
     * 单臂逆运动学：使用 L-BFGS 最小化 0.5*||pose(q)-target||^2
     * @param {number[]} dqTargetArray - 目标位姿（8 元素双四元数）
     * @returns {boolean} 是否收敛
     */
    updateSingleArmJointsWithLBFGS(dqTargetArray) {
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) {
            return false;
        }
        const robot = window.currentRobot;
        const target = dqTargetArray;
        const poseTol = 1e-4;
        const f = (q) => {
            const pose = robot.fkm(q);
            let sum = 0;
            for (let i = 0; i < 8; i++) {
                const e = pose[i] - target[i];
                sum += e * e;
            }
            return 0.5 * sum;
        };
        const grad = (q, out) => {
            const pose = robot.fkm(q);
            const err = [];
            for (let i = 0; i < 8; i++) err.push(pose[i] - target[i]);
            const J = robot.poseJacobian(q);
            if (!J || J.length !== 8) {
                for (let i = 0; i < out.length; i++) out[i] = 0;
                return;
            }
            const n = out.length;
            for (let i = 0; i < n; i++) {
                let s = 0;
                for (let j = 0; j < 8; j++) s += (J[j][i] || 0) * err[j];
                out[i] = s;
            }
        };
        const result = this.minimizeLBFGS(f, grad, [...window.jointAngles], {
            maxIter: 120,
            gradTol: 1e-8,
            costTol: 0.5 * poseTol * poseTol,
            m: 10
        });
        const { x, converged } = result;
        for (let i = 0; i < window.jointAngles.length && i < x.length; i++) {
            window.jointAngles[i] = x[i];
        }
        if (window.customRobotConfig && window.clampJointAngles) {
            const names = window.customRobotConfig?.joint_chain?.joints || [];
            const urdf = this.urdfRobot || null;
            const clamped = window.clampJointAngles(
                window.jointAngles,
                names,
                urdf,
                window.customRobotConfig
            );
            for (let i = 0; i < window.jointAngles.length && i < clamped.length; i++) {
                window.jointAngles[i] = clamped[i];
            }
        }
        if (window.updateGlobalVariables) window.updateGlobalVariables();
        if (window.updateRobotPose) window.updateRobotPose();
        this.lastIKSolveStatus = {
            solver: 'lbfgs',
            ...result,
            finalResidual: this._computePoseResidualNorm(robot, window.jointAngles, target)
        };
        return converged || this.lastIKSolveStatus.finalResidual <= poseTol;
    }
    
    /**
     * 使用双四元数方式和雅可比伪逆从末端执行器位移更新关节角度
     * 
     * 参考 MATLAB 代码 cdts_bucket_kuka.m 的实现方式：
     * 1. 获取当前末端执行器位姿（双四元数）：dq_current = robot.fkm(jointAngles)
     * 2. 计算位移的平移双四元数：dq_delta = 1 + DQ.E*0.5*DQ([0, dx, dy, dz])
     *    使用 DQ.translation(dx, dy, dz) 创建：1 + 0.5*E*DQ(0, dx, dy, dz)
     * 3. 计算目标位姿：dq_target = dq_delta * dq_current（在世界坐标系中）
     * 4. 计算8元素误差向量：err = vec8(dq_target) - vec8(dq_current)
     * 5. 获取完整的8xN位姿雅可比矩阵（8行对应双四元数的8个元素，N列对应关节数）
     * 6. 使用伪逆更新关节角度：q = q + pinv(J) * 0.5 * err
     * 
     * 注意：poseJacobian 返回的是 8xN 矩阵，前4行是旋转四元数的雅可比，后4行是平移的雅可比（与旋转耦合）
     * 
     * @param {THREE.Vector3} delta - 末端执行器的位移（世界坐标系，单位：米）
     */
    updateJointsFromEndEffectorDelta(delta) {
        if (!window.currentRobot || !window.jointAngles || window.jointAngles.length === 0) return;
        
        try {
            // 检查 DQModule 是否可用
            if (!window.DQModule || !window.DQModule.DQWrapper) {
                console.error('DQ Robotics 模块未加载');
                return;
            }
            
            // 1. 获取当前末端执行器位姿（双四元数，8元素数组）
            // 参考 MATLAB: dqad_ant = two_arms.absolute_pose(q);
            const dqCurrentArray = window.currentRobot.fkm(window.jointAngles);
            if (!dqCurrentArray || dqCurrentArray.length !== 8) {
                console.error('无法获取当前位姿');
                return;
            }
            
            // 2. 创建位移的平移双四元数：dq_delta = 1 + DQ.E*0.5*DQ([0, dx, dy, dz])
            // 参考 MATLAB: dqad = (1+DQ.E*0.5*(-0.1*DQ.i-0.1*DQ.j-0.1*DQ.k)) * dqad_ant
            // 在 JavaScript 中，使用 DQWrapper.translation(dx, dy, dz) 创建
            const dqCurrent = window.DQModule.DQWrapper.createFromArray(dqCurrentArray);
            const { multiply } = window.DQModule.DQWrapper;
            const worldMode = this.isWorldTranslationMode();
            let dqDelta;
            let dqTarget;
            if (worldMode) {
                dqDelta = window.DQModule.DQWrapper.translation(delta.x, delta.y, delta.z);
                dqTarget = multiply(dqDelta, dqCurrent);
            } else {
                const rotationArray = window.DQModule.DQWrapper.getRotation(dqCurrent);
                const rotationQuat = new THREE.Quaternion(
                    rotationArray[1] || 0,
                    rotationArray[2] || 0,
                    rotationArray[3] || 0,
                    rotationArray[0] || 1
                );
                const deltaLocal = delta.clone().applyQuaternion(rotationQuat.clone().conjugate());
                dqDelta = window.DQModule.DQWrapper.translation(deltaLocal.x, deltaLocal.y, deltaLocal.z);
                dqTarget = multiply(dqCurrent, dqDelta);
            }
            
            // 4. 保存目标位姿数组
            const dqTargetArray = window.DQModule.DQWrapper.toArray(dqTarget);
            
            // 5. 迭代求解逆运动学（参考 MATLAB 的 while 循环）
            const epsilon = 0.0001;  // 停止条件：误差范数小于此值
            const maxIterations = 50;  // 最大迭代次数，防止无限循环
            const damping = 0.5;  // 阻尼系数（与MATLAB一致）
            
            // 使用局部变量进行迭代，避免频繁修改全局 jointAngles
            let q = [...window.jointAngles];
            let errNorm = epsilon + 1;
            let iteration = 0;
            
            while (errNorm > epsilon && iteration < maxIterations) {
                // 获取当前位姿
                const dqCurrentIterArray = window.currentRobot.fkm(q);
                
                // 计算误差向量
                const err = [];
                for (let i = 0; i < 8; i++) {
                    err.push(dqTargetArray[i] - dqCurrentIterArray[i]);
                }
                
                // 计算误差范数
                errNorm = Math.sqrt(err.reduce((sum, e) => sum + e * e, 0));
                
                if (errNorm <= epsilon) {
                    break;  // 误差足够小，退出循环
                }
                
                // 获取雅可比矩阵
                const jacobian = window.currentRobot.poseJacobian(q);
                if (!jacobian || jacobian.length !== 8) {
                    console.error('雅可比矩阵维度错误');
                    return;
                }
                
                // 计算雅可比伪逆
                const jacobianPseudoInv = this.pseudoInverse(jacobian);
                
                // 计算关节角度增量
                const deltaJoints = [];
                for (let i = 0; i < jacobianPseudoInv.length; i++) {
                    let sum = 0;
                    for (let j = 0; j < err.length; j++) {
                        sum += jacobianPseudoInv[i][j] * err[j] * damping;
                    }
                    deltaJoints.push(sum);
                }
                
                // 更新关节角度
                for (let i = 0; i < q.length && i < deltaJoints.length; i++) {
                    q[i] += deltaJoints[i];
                }
                
                iteration++;
            }
            
            // 6. 限制关节角度在限位范围内
            if (window.customRobotConfig && window.customRobotConfig.joint_chain) {
                const jointNames = window.customRobotConfig.joint_chain.joints || [];
                q = window.clampJointAngles(q, jointNames, this.urdfRobot, window.customRobotConfig);
            }
            
            // 7. 更新全局关节角度
            if (!window.jointAngles || window.jointAngles.length !== q.length) {
                if (window.updateGlobalVariables) {
                    window.updateGlobalVariables();
                }
            }
            
            // 更新关节角度
            for (let i = 0; i < window.jointAngles.length && i < q.length; i++) {
                window.jointAngles[i] = q[i];
            }
            
            // 同步到 main.js 的 jointAngles（确保引用一致）
            if (window.updateGlobalVariables) {
                window.updateGlobalVariables();
            }
            
            // 立即同步更新机器人位姿（避免异步问题）
            if (window.updateRobotPose) {
                window.updateRobotPose();
            }
            
            // 确保拖动控制跟随末端执行器（在拖动过程中也要更新）
            this.updateGizmoPosition();
            
        } catch (error) {
            console.error('更新关节角度失败:', error);
        }
    }
    
    /**
     * L-BFGS 无约束最小化：min f(x)，需提供 f(x) 与 grad(x)。
     * @param {function(number[]): number} f - 标量目标函数
     * @param {function(number[], number[]): void} grad - 梯度函数 grad(x, out)，结果写入 out
     * @param {number[]} x0 - 初始点
     * @param {{ maxIter?: number, tol?: number, m?: number, maxLineSearch?: number }} options
     * @returns {{ x: number[], converged: boolean, cost: number }}
     */
    minimizeLBFGS(f, grad, x0, options = {}) {
        return minimizeLBFGS(f, grad, x0, options);
    }

    _computePoseResidualNorm(robot, jointAngles, target) {
        const pose = robot?.fkm?.(jointAngles);
        if (!pose || pose.length !== target.length) return Number.POSITIVE_INFINITY;
        return this._computeArrayResidualNorm(pose, target);
    }

    _computeArrayResidualNorm(actual, target) {
        if (!Array.isArray(actual) || !Array.isArray(target) || actual.length !== target.length) {
            return Number.POSITIVE_INFINITY;
        }
        let sum = 0;
        for (let i = 0; i < target.length; i++) {
            const delta = (actual[i] || 0) - (target[i] || 0);
            sum += delta * delta;
        }
        return Math.sqrt(sum);
    }

    /**
     * 计算矩阵的伪逆（Moore-Penrose伪逆）
     * @param {number[][]} matrix - 输入矩阵
     * @returns {number[][]} 伪逆矩阵
     */
    pseudoInverse(matrix) {
        const rows = matrix.length;
        const cols = matrix[0].length;
        
        // 转置矩阵
        const transpose = [];
        for (let j = 0; j < cols; j++) {
            transpose[j] = [];
            for (let i = 0; i < rows; i++) {
                transpose[j][i] = matrix[i][j];
            }
        }
        
        // 计算 J * J^T
        const jjt = [];
        for (let i = 0; i < rows; i++) {
            jjt[i] = [];
            for (let j = 0; j < rows; j++) {
                let sum = 0;
                for (let k = 0; k < cols; k++) {
                    sum += matrix[i][k] * transpose[k][j];
                }
                jjt[i][j] = sum;
            }
        }
        
        // 添加正则化项（阻尼）
        const lambda = 0.01;
        for (let i = 0; i < rows; i++) {
            jjt[i][i] += lambda;
        }
        
        // 计算 (J * J^T)^(-1)
        // 使用通用矩阵求逆（支持任意大小的方阵）
        const jjtInv = this.inverseMatrix(jjt);
        
        // 计算伪逆: J^+ = J^T * (J * J^T)^(-1)
        const pseudoInv = [];
        for (let i = 0; i < cols; i++) {
            pseudoInv[i] = [];
            for (let j = 0; j < rows; j++) {
                let sum = 0;
                for (let k = 0; k < rows; k++) {
                    sum += transpose[i][k] * jjtInv[k][j];
                }
                pseudoInv[i][j] = sum;
            }
        }
        
        return pseudoInv;
    }
    
    /**
     * 计算通用矩阵的逆（使用高斯-约当消元法）
     * @param {number[][]} matrix - 方阵
     * @returns {number[][]} 逆矩阵
     */
    inverseMatrix(matrix) {
        const n = matrix.length;
        if (matrix[0].length !== n) {
            throw new Error('矩阵必须是方阵');
        }
        
        // 特殊情况：3x3矩阵使用快速公式
        if (n === 3) {
            return this.inverse3x3(matrix);
        }
        
        // 创建增广矩阵 [A | I]
        const augmented = [];
        for (let i = 0; i < n; i++) {
            augmented[i] = [];
            for (let j = 0; j < n; j++) {
                augmented[i][j] = matrix[i][j];
            }
            for (let j = n; j < 2 * n; j++) {
                augmented[i][j] = (j - n === i) ? 1 : 0;
            }
        }
        
        // 高斯-约当消元法
        for (let i = 0; i < n; i++) {
            // 找主元
            let maxRow = i;
            for (let k = i + 1; k < n; k++) {
                if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
                    maxRow = k;
                }
            }
            
            // 交换行
            [augmented[i], augmented[maxRow]] = [augmented[maxRow], augmented[i]];
            
            // 检查主元是否为0
            if (Math.abs(augmented[i][i]) < 1e-10) {
                console.warn('矩阵接近奇异，无法求逆');
                // 返回单位矩阵作为fallback
                const identity = [];
                for (let r = 0; r < n; r++) {
                    identity[r] = [];
                    for (let c = 0; c < n; c++) {
                        identity[r][c] = (r === c) ? 1 : 0;
                    }
                }
                return identity;
            }
            
            // 归一化当前行
            const pivot = augmented[i][i];
            for (let j = 0; j < 2 * n; j++) {
                augmented[i][j] /= pivot;
            }
            
            // 消元
            for (let k = 0; k < n; k++) {
                if (k !== i) {
                    const factor = augmented[k][i];
                    for (let j = 0; j < 2 * n; j++) {
                        augmented[k][j] -= factor * augmented[i][j];
                    }
                }
            }
        }
        
        // 提取逆矩阵
        const inverse = [];
        for (let i = 0; i < n; i++) {
            inverse[i] = [];
            for (let j = 0; j < n; j++) {
                inverse[i][j] = augmented[i][j + n];
            }
        }
        
        return inverse;
    }
    
    /**
     * 计算3x3矩阵的逆（快速公式）
     * @param {number[][]} matrix - 3x3矩阵
     * @returns {number[][]} 逆矩阵
     */
    inverse3x3(matrix) {
        const a = matrix[0][0], b = matrix[0][1], c = matrix[0][2];
        const d = matrix[1][0], e = matrix[1][1], f = matrix[1][2];
        const g = matrix[2][0], h = matrix[2][1], i = matrix[2][2];
        
        const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        
        if (Math.abs(det) < 1e-10) {
            console.warn('矩阵行列式接近0，无法求逆');
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]; // 返回单位矩阵
        }
        
        const invDet = 1 / det;
        
        return [
            [(e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet],
            [(f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet],
            [(d * h - e * g) * invDet, (b * g - a * h) * invDet, (a * e - b * d) * invDet]
        ];
    }

    /**
     * 添加一个环境箱体
     * @param {Object} options
     * @param {string} options.id - 唯一标识
     * @param {number} options.width - X 方向尺寸（米）
     * @param {number} options.depth - Y 方向尺寸（米）
     * @param {number} options.height - Z 方向尺寸（米）
     * @param {number} [options.color=0xf97316] - 箱子颜色
     * @param {number} [options.opacity=0.9] - 透明度
     * @param {{x:number,y:number,z:number}} [options.position] - 箱子中心位置（米）
     * @returns {THREE.Mesh|null}
     */
    addEnvironmentBox({
        id,
        width,
        depth,
        height,
        color = 0xf97316,
        opacity = 0.9,
        position = {}
    } = {}) {
        if (!id || !Number.isFinite(width) || !Number.isFinite(depth) || !Number.isFinite(height)) {
            console.warn('添加环境箱体失败：参数不完整');
            return null;
        }
        if (!this.environmentGroup) {
            console.warn('环境组尚未创建，无法添加环境物体');
            return null;
        }

        // 如果已存在同名对象，先移除
        if (this.environmentObjects.has(id)) {
            this.removeEnvironmentObject(id);
        }

        const geometry = new THREE.BoxGeometry(width, depth, height);
        const material = new THREE.MeshStandardMaterial({
            color,
            transparent: true,
            opacity,
            roughness: 0.35,
            metalness: 0.05
        });

        const boxMesh = new THREE.Mesh(geometry, material);
        boxMesh.castShadow = true;
        boxMesh.receiveShadow = true;
        boxMesh.name = `env-box-${id}`;
        boxMesh.userData = {
            environmentId: id,
            environmentType: 'box',
            dimensions: { width, depth, height },
            attachedTo: null,
            offset: {
                position: { x: 0, y: 0, z: 0 },
                rotationDeg: { x: 0, y: 0, z: 0 }
            }
        };

        const centerZ = Number.isFinite(position?.z) ? position.z : height / 2;
        boxMesh.position.set(position?.x ?? 0, position?.y ?? 0, centerZ);

        this.environmentGroup.add(boxMesh);
        this.environmentObjects.set(id, boxMesh);

        console.log(`✓ 已添加环境箱体 ${id}，尺寸: ${width.toFixed(3)} × ${depth.toFixed(3)} × ${height.toFixed(3)} (m)`);
        return boxMesh;
    }

    /**
     * 移除指定环境物体
     * @param {string} id - 环境物体的唯一标识
     * @returns {boolean} 是否成功移除
     */
    removeEnvironmentObject(id) {
        if (!id || !this.environmentObjects.has(id)) {
            return false;
        }
        const object = this.environmentObjects.get(id);
        if (object) {
            this.detachEnvironmentObject(id);
            if (this.environmentGroup) {
                this.environmentGroup.remove(object);
            }
            this.disposeObjectResources(object);
        }
        this.environmentObjects.delete(id);
        return true;
    }

    /**
     * 清空所有环境物体
     */
    clearEnvironmentObjects() {
        if (!this.environmentObjects.size) {
            return;
        }
        const ids = Array.from(this.environmentObjects.keys());
        ids.forEach((id) => {
            this.removeEnvironmentObject(id);
        });
        this.environmentObjects.clear();
    }

    // ── 场景预设管理 ─────────────────────────────────

    /**
     * 加载场景预设
     * @param {string} presetId - 预设 ID（如 'warehouse'）
     * @param {THREE.Group} presetGroup - 由环境构建器生成的 Three.js Group
     * @returns {boolean}
     */
    loadScenePreset(presetId, presetGroup) {
        if (!this.scene) {
            console.warn('场景未初始化，无法加载场景预设');
            return false;
        }

        // 清除已有预设
        this.clearScenePreset();

        // 添加新预设
        this.scenePresetGroup = presetGroup;
        this.currentScenePreset = presetId;
        this.scene.add(this.scenePresetGroup);

        console.log(`✓ 已加载场景预设: ${presetId}`);
        return true;
    }

    /**
     * 清除当前场景预设
     * @returns {boolean}
     */
    clearScenePreset() {
        if (!this.scenePresetGroup) {
            return false;
        }

        // 递归释放所有子对象资源
        this.scenePresetGroup.traverse((child) => {
            if (child.isMesh) {
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach((m) => m.dispose());
                    } else {
                        child.material.dispose();
                    }
                }
            }
        });

        this.scene.remove(this.scenePresetGroup);
        this.scenePresetGroup = null;
        this.currentScenePreset = null;

        console.log('✓ 已清除场景预设');
        return true;
    }

    /**
     * 获取当前场景预设 ID
     * @returns {string|null}
     */
    getCurrentScenePreset() {
        return this.currentScenePreset;
    }

    /**
     * 获取主要末端执行器标记
     * @returns {THREE.Object3D|null}
     */
    getPrimaryEndEffectorAnchor() {
        if (this.endEffector) {
            return this.endEffector;
        }
        // 人形模式：优先返回绝对位姿标记或左臂末端
        if (this.humanoidMode) {
            if (this.humanoidAbsoluteMarker) return this.humanoidAbsoluteMarker;
            if (this.humanoidLeftArmEndEffector) return this.humanoidLeftArmEndEffector;
            if (this.humanoidRightArmEndEffector) return this.humanoidRightArmEndEffector;
        }
        if (this.dualArmMode) {
            const activeMode = typeof window !== 'undefined' ? window.currentDualArmControlMode : null;
            if (activeMode === 'robot2' && this.robot2EndEffector) {
                return this.robot2EndEffector;
            }
            if (this.robot1EndEffector) {
                return this.robot1EndEffector;
            }
            if (this.robot2EndEffector) {
                return this.robot2EndEffector;
            }
        } else {
            if (this.robot1EndEffector) {
                return this.robot1EndEffector;
            }
            if (this.robot2EndEffector) {
                return this.robot2EndEffector;
            }
        }
        return null;
    }

    /**
     * 判断是否存在末端执行器锚点
     * @returns {boolean}
     */
    hasEndEffectorAnchor() {
        return !!(this.endEffector || this.robot1EndEffector || this.robot2EndEffector);
    }

    /**
     * 判断是否存在双臂绝对姿态标记
     * @returns {boolean}
     */
    hasAbsolutePoseMarker() {
        return !!(this.absolutePoseMarker || (this.humanoidMode && this.humanoidAbsoluteMarker));
    }

    /**
     * 将环境物体附着到指定锚点
     * @param {string} id - 环境物体 ID
     * @param {{target?: 'end-effector'|'dual-absolute'}} options
     * @returns {boolean}
     */
    attachEnvironmentObject(id, options = {}) {
        const target = options.target || 'end-effector';
        if (!this.environmentObjects.has(id)) {
            throw new Error(`未找到环境物体 ${id}`);
        }
        const object = this.environmentObjects.get(id);
        let anchor = null;

        if (target === 'dual-absolute') {
            if (this.humanoidMode && this.humanoidAbsoluteMarker) {
                anchor = this.humanoidAbsoluteMarker;
            } else if (this.dualArmMode && this.absolutePoseMarker) {
                anchor = this.absolutePoseMarker;
            } else {
                throw new Error('当前没有可用的双臂绝对姿态小球');
            }
        } else if (target === 'end-effector') {
            anchor = this.getPrimaryEndEffectorAnchor();
            if (!anchor) {
                throw new Error('当前没有可用的末端执行器小球');
            }
        } else {
            throw new Error(`未知的附着目标: ${target}`);
        }

        const savedScale = this._tempScale.set(object.scale.x, object.scale.y, object.scale.z);
        anchor.add(object);
        object.position.set(0, 0, 0);
        object.quaternion.identity();
        object.scale.copy(savedScale);
        anchor.updateMatrixWorld(true);
        object.updateMatrixWorld(true);

        if (!object.userData) {
            object.userData = {};
        }
        object.userData.attachedTo = target;
        if (!object.userData.offset) {
            object.userData.offset = {
                position: { x: 0, y: 0, z: 0 },
                rotationDeg: { x: 0, y: 0, z: 0 }
            };
        }
        this.applyEnvironmentObjectOffset(id);
        return true;
    }

    /**
     * 解除环境物体的附着
     * @param {string} id - 环境物体 ID
     * @returns {boolean}
     */
    detachEnvironmentObject(id) {
        if (!this.environmentObjects.has(id) || !this.environmentGroup) {
            return false;
        }
        const object = this.environmentObjects.get(id);
        object.getWorldPosition(this._tempVec3);
        object.getWorldQuaternion(this._tempQuat);
        object.getWorldScale(this._tempScale);

        this.environmentGroup.add(object);
        object.position.copy(this._tempVec3);
        object.quaternion.copy(this._tempQuat);
        object.scale.copy(this._tempScale);
        object.updateMatrixWorld(true);

        if (object.userData) {
            object.userData.attachedTo = null;
        }
        return true;
    }

    /**
     * 解除所有环境物体附着
     */
    detachAllEnvironmentObjects() {
        if (!this.environmentObjects.size) {
            return;
        }
        const ids = Array.from(this.environmentObjects.keys());
        ids.forEach((id) => {
            this.detachEnvironmentObject(id);
        });
    }

    /**
     * 根据存储的偏移应用局部变换
     * @param {string} id
     * @returns {boolean}
     */
    applyEnvironmentObjectOffset(id) {
        if (!this.environmentObjects.has(id)) {
            return false;
        }
        const object = this.environmentObjects.get(id);
        const offset = object.userData?.offset;
        if (!offset) {
            return false;
        }

        const pos = offset.position || { x: 0, y: 0, z: 0 };
        object.position.set(pos.x || 0, pos.y || 0, pos.z || 0);

        const rotDeg = offset.rotationDeg || { x: 0, y: 0, z: 0 };
        this._tempEuler.set(
            THREE.MathUtils.degToRad(rotDeg.x || 0),
            THREE.MathUtils.degToRad(rotDeg.y || 0),
            THREE.MathUtils.degToRad(rotDeg.z || 0),
            'XYZ'
        );
        object.quaternion.setFromEuler(this._tempEuler);
        object.updateMatrixWorld(true);
        return true;
    }

    /**
     * 设置环境物体的局部偏移
     * @param {string} id
     * @param {{position?:{x:number,y:number,z:number},rotation?:{x:number,y:number,z:number}}} offset
     * @returns {{position:{x:number,y:number,z:number},rotation:{x:number,y:number,z:number}}}
     */
    setEnvironmentObjectOffset(id, offset = {}) {
        if (!this.environmentObjects.has(id)) {
            throw new Error(`未找到环境物体 ${id}`);
        }
        const object = this.environmentObjects.get(id);
        if (!object.userData) {
            object.userData = {};
        }
        if (!object.userData.offset) {
            object.userData.offset = {
                position: { x: 0, y: 0, z: 0 },
                rotationDeg: { x: 0, y: 0, z: 0 }
            };
        }

        const current = object.userData.offset;

        if (offset.position) {
            const { x, y, z } = offset.position;
            current.position = {
                x: Number.isFinite(x) ? x : current.position.x,
                y: Number.isFinite(y) ? y : current.position.y,
                z: Number.isFinite(z) ? z : current.position.z
            };
        }

        if (offset.rotation) {
            const { x, y, z } = offset.rotation;
            current.rotationDeg = {
                x: Number.isFinite(x) ? x : current.rotationDeg.x,
                y: Number.isFinite(y) ? y : current.rotationDeg.y,
                z: Number.isFinite(z) ? z : current.rotationDeg.z
            };
        }

        if (object.userData.attachedTo) {
            this.applyEnvironmentObjectOffset(id);
        }

        return this.getEnvironmentObjectOffset(id);
    }

    /**
     * 获取环境物体的偏移
     * @param {string} id
     * @returns {{position:{x:number,y:number,z:number},rotation:{x:number,y:number,z:number}}|null}
     */
    getEnvironmentObjectOffset(id) {
        if (!this.environmentObjects.has(id)) {
            return null;
        }
        const object = this.environmentObjects.get(id);
        const offset = object.userData?.offset;
        if (!offset) {
            return {
                position: { x: 0, y: 0, z: 0 },
                rotation: { x: 0, y: 0, z: 0 }
            };
        }
        return {
            position: { ...offset.position },
            rotation: { ...offset.rotationDeg }
        };
    }

    /**
     * 获取环境物体世界坐标
     * @param {string} id
     * @returns {THREE.Vector3|null}
     */
    getEnvironmentObjectWorldPosition(id) {
        if (!this.environmentObjects.has(id)) {
            return null;
        }
        const object = this.environmentObjects.get(id);
        const position = new THREE.Vector3();
        object.getWorldPosition(position);
        return position;
    }

    /**
     * 获取环境物体当前附着目标
     * @param {string} id
     * @returns {string|null}
     */
    getEnvironmentObjectAttachment(id) {
        if (!this.environmentObjects.has(id)) {
            return null;
        }
        const object = this.environmentObjects.get(id);
        return object.userData?.attachedTo || null;
    }

    /**
     * 释放 Three.js 对象资源
     * @param {THREE.Object3D} object
     * @private
     */
    disposeObjectResources(object) {
        if (!object) {
            return;
        }
        if (object.geometry && typeof object.geometry.dispose === 'function') {
            object.geometry.dispose();
        }
        if (object.material) {
            if (Array.isArray(object.material)) {
                object.material.forEach((mat) => {
                    if (mat && typeof mat.dispose === 'function') {
                        mat.dispose();
                    }
                });
            } else if (typeof object.material.dispose === 'function') {
                object.material.dispose();
            }
        }
        if (object.children && object.children.length > 0) {
            for (let i = object.children.length - 1; i >= 0; i -= 1) {
                this.disposeObjectResources(object.children[i]);
            }
        }
    }
}

export function getHumanoidJogTranslationAxisGuard(internalOptions = null) {
    return internalOptions?.translationAxisGuard ?? null;
}
