import test from 'node:test';
import assert from 'node:assert/strict';

import { RobotVisualizer } from '../visualizer.js';
import { DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2 } from './humanoid-default-q-profiles.js';
import { buildHumanoidJogMJDiagnostics } from './humanoid-jog-mj-diagnostics.js';
import {
    computeAngularJacobian,
    computeCombinedCCTSM,
    computeTwistJacobian
} from './cctsm.js';
import { buildSurfaceAttachedFrame, buildSurfaceAttachedPoseTarget, evaluateSurfacePatch } from './surface-slide-primitive.js';

test('RobotVisualizer sets manifoldNullspaceStepSize default to 0.1', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.manifoldNullspaceStepSize, 0.1);
});

test('RobotVisualizer sets manifoldWVFIButtonStepSize default to 0.1', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.manifoldWVFIButtonStepSize, 0.1);
});

test('RobotVisualizer sets manifold reference restoration defaults', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.manifoldReferenceStiffness, 1.0);
    assert.equal(visualizer.vfiConfig.manifoldReferenceDamping, 0.35);
});

test('RobotVisualizer sets hierarchical VFI blend default to 1', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.manifoldHierarchicalVfiBlend, 1);
});

test('RobotVisualizer sets joint-limit manipulability activation threshold default to 0.15 rad', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.jointLimitEta, 5);
    assert.equal(visualizer.vfiConfig.jointLimitManipulabilityActivationThreshold, 0.15);
    assert.equal(visualizer.vfiConfig.manifoldActiveLimitRetreatHoldFraction, 0.5);
    assert.equal(visualizer.vfiConfig.manifoldActiveLimitRetreatHoldDistance, 1e-3);
    assert.equal(visualizer.vfiConfig.manifoldActiveLimitRetreatTargetFraction, 0.2);
});

test('RobotVisualizer can enable temporary dual-arm test lock limits for ljoint2 and rjoint2', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidJointNames = ['waist0', 'ljoint1', 'ljoint2', 'rjoint1', 'rjoint2'];
    visualizer.humanoidURDF = {
        joints: {
            waist0: { limit: { lower: -1, upper: 1 } },
            ljoint1: { limit: { lower: -1, upper: 1 } },
            ljoint2: { limit: { lower: -1, upper: 1 } },
            rjoint1: { limit: { lower: -1, upper: 1 } },
            rjoint2: { limit: { lower: -1, upper: 1 } }
        }
    };

    visualizer.setHumanoidTestDualArmLockEnabled(true);
    const limits = visualizer._getHumanoidJointLimits();

    assert.deepEqual(limits[2], { lower: -0.1, upper: 0.2 });
    assert.deepEqual(limits[4], { lower: -0.2, upper: 0.1 });
});

test('RobotVisualizer scales priority candidates to satisfy the waist x corridor', () => {
    const visualizer = new RobotVisualizer(null);
    const corridor = {
        enabled: true,
        min: Number.NEGATIVE_INFINITY,
        max: 0.05,
        tolerance: 0,
        evaluateX: (qActive) => qActive[0],
        vfiRows: [[1, 0]],
        rhs: [0.001]
    };

    const result = visualizer._scaleHumanoidPriorityStepForWaistXCorridor({
        corridor,
        qActiveCurrent: [0.049, 0],
        dqActive: [0.01, 0]
    });

    assert.equal(result.active, true);
    assert.ok(result.scale < 1);
    assert.ok(result.stateAfter.value <= 0.0500001);
    assert.ok(result.dqApplied[0] <= 0.0010001);
});

test('RobotVisualizer priority absolute full-body jog respects the waist x corridor when waist motion is active', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('priority');
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 1;
    visualizer.humanoidLeftArmLength = 1;
    visualizer.humanoidRightArmLength = 1;
    visualizer.humanoidJointNames = ['waist0', 'ljoint1', 'rjoint1'];
    visualizer.humanoidURDF = {
        joints: {
            waist0: { limit: { lower: -2, upper: 2 } },
            ljoint1: { limit: { lower: -2, upper: 2 } },
            rjoint1: { limit: { lower: -2, upper: 2 } }
        }
    };
    visualizer.setHumanoidJoints = () => {};
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._publishCurrentPriorityCCTSMDiagnostics = () => {};
    visualizer._notifyHumanoidJogUnreachable = () => {};
    visualizer._buildHumanoidManifoldWaistXCorridor = () => ({
        enabled: true,
        min: Number.NEGATIVE_INFINITY,
        max: 0.05,
        tolerance: 0,
        evaluateX: (qActive) => qActive[0],
        vfiRows: [[1, 0, 0]],
        rhs: [0.001]
    });
    visualizer._getHumanoidAbsolutePose = (q) => [1, q[0], 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidRelativePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidAbsolutePoseJacobian = () => {
        const J = Array.from({ length: 8 }, () => [0, 0, 0]);
        J[1] = [1, 0, 0];
        return J;
    };
    visualizer._getHumanoidRelativePoseJacobian = () => Array.from({ length: 8 }, () => [0, 0, 0]);

    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: [0.049, 0, 0],
        DQModule: { DQWrapper: {} },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    try {
        const ok = visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
            [1, 0.2, 0, 0, 0, 0, 0, 0],
            [0.049, 0, 0],
            true,
            true,
            false,
            null,
            null
        );
        assert.equal(ok, false);
        assert.ok(globalThis.window.humanoidQ[0] <= 0.0500001);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer sets manifoldJogSecondaryMetric default to none', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.manifoldJogSecondaryMetric, 'none');
});

test('RobotVisualizer sets mj_restore body-z defaults for upright absolute control', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyTolerance, 5e-5);
    assert.equal(visualizer.vfiConfig.mjRestoreHardTolerance, 1e-8);
    assert.equal(visualizer.vfiConfig.mjRestorePracticalCommandTolPose, 1e-6);
    assert.equal(visualizer.vfiConfig.mjRestorePracticalInvariantTol, 1e-6);
    assert.equal(visualizer.vfiConfig.mjRestoreMetricButtonCommandTolerance, 1e-3);
    assert.equal(visualizer.vfiConfig.mjRestoreMetricButtonInvariantTolerance, 1e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreActiveLimitRetreatCommandTolerance, 1e-2);
    assert.equal(visualizer.vfiConfig.mjRestoreActiveLimitRetreatInvariantTolerance, 2e-3);
    assert.equal(visualizer.vfiConfig.mjRestoreRelativeWeight, 1.0);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyVerticalWeight, 4.0);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyReferenceStiffness, 0.6);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyReferenceDamping, 0.45);
});

test('RobotVisualizer sets plain new qpJ pose convergence tolerances to practical defaults', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.hardCommandTolPose, 1e-6);
    assert.equal(visualizer.vfiConfig.practicalCommandTolPose, 1e-4);
    assert.equal(visualizer.vfiConfig.practicalViolationTol, 1e-7);
    assert.equal(visualizer.vfiConfig.qpFeasibleAcceptanceTol, 1e-6);
});

test('RobotVisualizer sets lightweight mj_restore_pre_w profile defaults', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.mjRestorePreWMetricIterations, 3);
    assert.equal(visualizer.vfiConfig.mjRestorePreWRestoreIterations, 1);
});

test('RobotVisualizer sets mj_restore arm-only fallback profile defaults', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.mjRestoreMaxStep, 0.08);
    assert.equal(visualizer.vfiConfig.mjRestoreSurfaceSlideMaxIter, 160);
    assert.equal(visualizer.vfiConfig.mjRestoreSurfaceSlideMaxStepScale, 1.5);
    assert.equal(visualizer.vfiConfig.mjRestoreSurfaceSlideDamping, 5e-5);
    assert.equal(visualizer.vfiConfig.mjRestoreSurfaceSlideReferenceDamping, 0.2);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingMaxIter, 160);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingMaxStepScale, 1.0);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDamping, 1e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingReferenceDamping, 0.15);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingMaxDampingRetries, 12);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDampingGrowthFactor, 4);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingMaxTrustRegionRetries, 12);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingRhoAcceptanceThreshold, 0.02);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingRhoShrinkThreshold, 0.02);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingBestEffortStepNormCap, 0.02);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingBestEffortCommandNormCap, 0.005);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingBestEffortInvariantNormCap, 0.002);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingBodyTolerance, 1e-6);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingHardCommandTolPose, 5e-6);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingPracticalCommandTolPose, 5e-6);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingHardInvariantTol, 1e-6);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingPracticalInvariantTol, 1e-6);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingPolishActivationResidual, 1);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingPolishMaxIter, 240);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingPolishDamping, 1e-8);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointMaxIter, 80);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointDamping, 1e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointReferenceDamping, 0.15);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointMaxDampingRetries, 10);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointDampingGrowthFactor, 4);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointMaxTrustRegionRetries, 10);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointRhoAcceptanceThreshold, 0.02);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointRhoShrinkThreshold, 0.02);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointPolishActivationResidual, 5e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointPolishMaxIter, 64);
    assert.equal(visualizer.vfiConfig.mjRestoreCooperativeSurfacePolishingDQSetpointPolishDamping, 1e-7);
    assert.equal(visualizer.vfiConfig.mjRestoreNullspaceStepSize, 0.01);
    assert.equal(visualizer.vfiConfig.mjRestoreMetricProjectionStepSize, 0.01);
    assert.equal(visualizer.vfiConfig.mjRestoreMetricProjectionPasses, 4);
    assert.equal(visualizer.vfiConfig.mjRestoreWaistMotionRegularization, 6);
    assert.equal(visualizer.vfiConfig.mjRestoreArmOnlyFallbackMaxIter, 12);
    assert.equal(visualizer.vfiConfig.mjRestoreArmOnlyFallbackReferenceDamping, 0.2);
    assert.equal(visualizer.vfiConfig.mjRestoreArmOnlyFallbackPolishActivationResidual, 5e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreArmOnlyFallbackPolishDamping, 1e-7);
});

test('RobotVisualizer exposes optimizeHumanoidAbsoluteManipulability for w_abs buttons', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer.optimizeHumanoidAbsoluteManipulability, 'function');
});

test('RobotVisualizer exposes optimizeHumanoidAbsoluteVFIManipulability for w_abs_vfi buttons', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer.optimizeHumanoidAbsoluteVFIManipulability, 'function');
});

test('RobotVisualizer exposes optimizeHumanoidAbsoluteArmManipulability for w_abs_arm buttons', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer.optimizeHumanoidAbsoluteArmManipulability, 'function');
});

test('RobotVisualizer exposes optimizeHumanoidHierarchicalAbsoluteManipulability for hierarchical w_abs buttons', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer.optimizeHumanoidHierarchicalAbsoluteManipulability, 'function');
});

test('RobotVisualizer exposes optimizeHumanoidHierarchicalAbsoluteVFIManipulability for hierarchical w_abs_vfi buttons', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulability, 'function');
});

test('RobotVisualizer exposes optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityIterative for iterative hierarchical w_abs_vfi buttons', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityIterative, 'function');
});

test('RobotVisualizer metric-only MJ optimization routes through the mj_restore wrapper', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: [0, 0]
    };
    const originalDocument = globalThis.document;
    globalThis.document = {
        getElementById: () => null
    };

    let called = null;
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidRelativePose = () => new Array(8).fill(0);
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
        called = args;
        return true;
    };

    try {
        const result = visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulability('increase');
        assert.equal(result, true);
        assert.ok(Array.isArray(called));
        assert.equal(called[7]?.jogMetricOverride, 'w_abs_hier_vfi');
        assert.equal(called[7]?.metricDirection, 'increase');
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer local metric-only absolute w_abs_hier_vfi request skips the full mj_restore solve when the current pose already satisfies the protected target', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = new Array(20).fill(0);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const Jabs = Array.from({ length: 8 }, () => new Array(q0.length).fill(0));
    const Jrel = Array.from({ length: 8 }, () => new Array(q0.length).fill(0));
    let solveAbsoluteTargetMJRestoreCalled = false;

    visualizer.humanoidModel = {
        solveAbsoluteTargetMJRestore: () => {
            solveAbsoluteTargetMJRestoreCalled = true;
            return {};
        }
    };
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => false;
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = () => ({
        w_abs_hier_vfi: 0.009,
        w_abs_arm_vfi: 0.009,
        w_abs_vfi: 0.009,
        w_abs_hier: 0.011,
        w_abs_arm: 0.011,
        w_abs: 0.011,
        w_cmd: 0.003,
        w: 0.007,
        w_vfi: 0.005
    });
    let firstMetricAttempt = null;
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => ({
        ...(firstMetricAttempt ??= structuredClone({
            metric: options.metric,
            preferWVFIFallbackOnClip: options.preferWVFIFallbackOnClip,
            directionFallbackMetric: options.directionFallbackMetric,
            preferActiveLimitRetreat: options.preferActiveLimitRetreat
        })),
        state: 'skipped(projection_only_no_progress)',
        before: 0.009,
        after: 0.009,
        applied: false,
        nullspaceResult: null,
        refineResult: null,
        finalProblem: null
    });
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: { DQWrapper: {} },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    try {
        const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
            absCurrent.slice(),
            q0.slice(),
            true,
            false,
            false,
            relCurrent.slice(),
            {
                solverMode: 'mj_restore',
                preferWorker: false,
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.equal(solveAbsoluteTargetMJRestoreCalled, false);
        assert.deepEqual(firstMetricAttempt, {
            metric: 'w_abs_arm_vfi',
            preferWVFIFallbackOnClip: false,
            directionFallbackMetric: '',
            preferActiveLimitRetreat: false
        });
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer metric-only MJ optimization routes relative-mode w+ through the relative-task mj_restore wrapper', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: [0, 0]
    };
    const originalDocument = globalThis.document;
    globalThis.document = {
        getElementById: (id) => {
            if (id === 'dual-arm-control-mode-select') return { value: 'relative' };
            return null;
        }
    };

    let called = null;
    visualizer._getHumanoidAbsolutePose = () => new Array(8).fill(0);
    visualizer._getHumanoidRelativePose = () => new Array(8).fill(1);
    visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore = (...args) => {
        called = args;
        return true;
    };

    try {
        const result = visualizer.optimizeHumanoidHierarchicalAbsoluteManipulability('increase');
        assert.equal(result, true);
        assert.ok(Array.isArray(called));
        assert.deepEqual(called[0], new Array(8).fill(1));
        assert.equal(called[4]?.jogMetricOverride, 'w_abs_hier');
        assert.equal(called[4]?.metricDirection, 'increase');
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer metric-only MJ optimization routes distance-mode w+ through the distance-task mj_restore wrapper', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: [0, 0]
    };
    const originalDocument = globalThis.document;
    globalThis.document = {
        getElementById: (id) => {
            if (id === 'dual-arm-control-mode-select') return { value: 'distance' };
            return null;
        }
    };

    let called = null;
    visualizer._getHumanoidAbsolutePose = () => new Array(8).fill(0);
    visualizer._getHumanoidRelativePose = () => new Array(8).fill(1);
    visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore = (...args) => {
        called = args;
        return true;
    };

    try {
        const result = visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulability('increase');
        assert.equal(result, true);
        assert.ok(Array.isArray(called));
        assert.equal(called[0], 0);
        assert.equal(called[4]?.jogMetricOverride, 'w_abs_hier_vfi');
        assert.equal(called[4]?.metricDirection, 'increase');
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer metric-only MJ optimization routes robot1-mode w+ through the dedicated TR single-arm helper', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    const originalDocument = globalThis.document;
    globalThis.document = {
        getElementById: (id) => {
            if (id === 'dual-arm-control-mode-select') return { value: 'robot1' };
            return null;
        }
    };

    let capturedArgs = null;
    visualizer._getHumanoidAbsolutePose = () => new Array(8).fill(0);
    visualizer._getHumanoidRelativePose = () => new Array(8).fill(0);
    visualizer._getHumanoidPose1 = () => new Array(8).fill(1);
    visualizer._updateHumanoidSingleArmPoseTargetMJRestore = async (...args) => {
        capturedArgs = args;
        return true;
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};

    try {
        const result = await visualizer.optimizeHumanoidHierarchicalAbsoluteManipulability('increase');
        assert.equal(result, true);
        assert.ok(capturedArgs);
        assert.deepEqual(capturedArgs[0], new Array(8).fill(1));
        assert.equal(capturedArgs[2], 1);
        assert.equal(capturedArgs[5]?.solverMode, 'mj_restore_tr');
        assert.equal(capturedArgs[5]?.jogMetricOverride, 'w_abs_hier');
        assert.equal(capturedArgs[5]?.metricDirection, 'increase');
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer plain new qpJ absolute-target step does not inject w secondary optimization or wvfi guard', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    visualizer.humanoidJointNames = Array.from({ length: 20 }, (_, index) => `joint${index}`);
    visualizer.humanoidURDF = {
        joints: Object.fromEntries(Array.from({ length: 20 }, (_, index) => [`joint${index}`, { limit: { lower: -2, upper: 2 } }]))
    };
    visualizer.setHumanoidJogSolverMode('vfi');

    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        DQModule: { DQWrapper: {}, Kinematics: null },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    const originalDocument = globalThis.document;
    globalThis.document = {
        getElementById: () => ({ checked: true, value: 'absolute' })
    };

    let capturedOptions = null;
    visualizer._getHumanoidAbsolutePose = () => new Array(8).fill(0);
    visualizer._getHumanoidRelativePose = () => new Array(8).fill(0);
    visualizer._getHumanoidAbsolutePoseJacobian = () => new Array(8).fill(0).map(() => new Array(14).fill(0));
    visualizer._getHumanoidRelativePoseJacobian = () => new Array(8).fill(0).map(() => new Array(14).fill(0));
    visualizer._getWaistXVFIRows = () => null;
    visualizer._computeProtectedManipulabilityGradient = () => ['unexpected'];
    visualizer._buildWVFILowerBoundGuard = () => ({ apply: () => ({ accepted: false }) });
    visualizer._solveVFICommandStep = (options) => {
        capturedOptions = options;
        return {
            dq: new Array(14).fill(0),
            stats: {
                qpConverged: true,
                stepNorm: 0,
                maxViolationAfter: 0,
                invariantRows: 8,
                commandRows: 8,
                constraintRows: 0
            }
        };
    };
    visualizer._computeModeCCTSM = () => ({ w: 0, w_vfi: 0 });
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer.setHumanoidJoints = () => {};

    try {
        const result = visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstVFI(
            [0, 1, 0, 0, 0, 0, 0, 0],
            new Array(20).fill(0),
            true,
            false,
            false,
            null,
            null
        );
        assert.equal(result, true);
        assert.ok(capturedOptions);
        assert.equal(capturedOptions.nullManipGrad, null);
        assert.equal(capturedOptions.wvfiGuard, null);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer iterative hierarchical w_abs_hier_vfi optimization routes through the local mj_restore manifold wrapper with a 15/35 plan', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: [0, 0]
    };
    const originalDocument = globalThis.document;
    globalThis.document = {
        getElementById: () => ({ value: 'absolute', checked: true })
    };

    let called = null;
    visualizer._getHumanoidAbsolutePose = () => new Array(8).fill(0);
    visualizer._getHumanoidRelativePose = () => new Array(8).fill(0);
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        called = args;
        return true;
    };

    try {
        const result = visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityIterative('increase');
        assert.equal(result, true);
        assert.ok(Array.isArray(called));
        assert.equal(called[6]?.solverMode, 'mj_restore_tr');
        assert.equal(called[6]?.preferWorker, false);
        assert.equal(called[6]?.jogMetricOverride, 'w_abs_hier_vfi');
        assert.equal(called[6]?.metricDirection, 'increase');
        assert.deepEqual(called[6]?.iterativeMetricProjectionPlan, {
            metricIterations: 15,
            restoreIterations: 35
        });
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer manifold metric evaluator prefers current-task hierarchical fields over absolute-task fields', () => {
    const visualizer = new RobotVisualizer(null);

    assert.equal(
        visualizer._evaluateHumanoidJogManifoldMetricValue('w_abs_hier', {
            w_abs_hier: 0.31,
            w_task_hier: 0.18
        }),
        0.18
    );

    assert.equal(
        visualizer._evaluateHumanoidJogManifoldMetricValue('w_abs_hier_vfi', {
            w_abs_hier_vfi: 0.27,
            w_task_hier: 0.2,
            w_task_hier_vfi: 0.14
        }),
        0.14
    );
});

test('RobotVisualizer prewarms the mj_restore worker when the solver mode switches to mj_restore', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMJWorkerModelSpec = { waistChain: [1] };

    let ensuredSpec = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
    visualizer._getHumanoidHierarchicalVfiBlend = () => 1;
    visualizer._getHumanoidJogMJWorkerClient = () => ({
        ensureModel: async (modelSpec) => {
            ensuredSpec = modelSpec;
            return true;
        }
    });

    visualizer.setHumanoidJogSolverMode('mj_restore');
    await new Promise((resolve) => setTimeout(resolve, 0));

    assert.deepEqual(ensuredSpec, { waistChain: [1] });
});

test('RobotVisualizer retains mj_restore_pre_w as the humanoid jog solver mode', () => {
    const visualizer = new RobotVisualizer(null);

    visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');

    assert.equal(visualizer.humanoidJogSolverMode, 'mj_restore_pre_w');
});

test('RobotVisualizer retains mj_restore_tr as the humanoid jog solver mode', () => {
    const visualizer = new RobotVisualizer(null);

    visualizer.setHumanoidJogSolverMode('mj_restore_tr');

    assert.equal(visualizer.humanoidJogSolverMode, 'mj_restore_tr');
});

test('RobotVisualizer absolute jog forwards mj_restore_tr into the MJ restore helper without pre-w routing', () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0.02, 0, 0, 0];
    let manifoldOptions = null;

    visualizer.setHumanoidJogSolverMode('mj_restore_tr');
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
        manifoldOptions = args[7];
        return true;
    };

    const ok = visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
        absTarget.slice(),
        q0.slice(),
        true,
        false,
        false,
        null,
        null
    );

    assert.equal(ok, true);
    assert.deepEqual(manifoldOptions, { solverMode: 'mj_restore_tr' });
});

test('RobotVisualizer mj_restore_pre_w coalesces overlapping absolute jog requests and keeps only the latest pending request', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const calls = [];
    const pendingResolvers = [];

    visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestorePreW = (...args) => {
        calls.push(args[0][4]);
        return new Promise((resolve) => {
            pendingResolvers.push(resolve);
        });
    };

    const targetA = [1, 0, 0, 0, 0.01, 0, 0, 0];
    const targetB = [1, 0, 0, 0, 0.02, 0, 0, 0];
    const targetC = [1, 0, 0, 0, 0.03, 0, 0, 0];

    assert.equal(
        visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(targetA.slice(), q0.slice(), true, false, false, null, null),
        true
    );
    assert.equal(
        visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(targetB.slice(), q0.slice(), true, false, false, null, null),
        true
    );
    assert.equal(
        visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(targetC.slice(), q0.slice(), true, false, false, null, null),
        true
    );

    await new Promise((resolve) => setTimeout(resolve, 0));
    assert.deepEqual(calls, [0.01]);
    assert.equal(pendingResolvers.length, 1);

    pendingResolvers.shift()(true);
    await new Promise((resolve) => setTimeout(resolve, 0));

    assert.deepEqual(calls, [0.01, 0.03]);
    assert.equal(pendingResolvers.length, 1);

    pendingResolvers.shift()(true);
    await new Promise((resolve) => setTimeout(resolve, 0));

    assert.deepEqual(calls, [0.01, 0.03]);
    assert.equal(visualizer._humanoidMJPreWLoopActive, false);
});

test('RobotVisualizer absolute mj_restore_pre_w resolves the dedicated pre-w metric, runs pre-w first, then restores from q_pre', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const qPre = [0.4, -0.1, 0.2];
    const absTarget = [1, 0, 0, 0, 0.02, 0, 0, 0];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const callOrder = [];
    const originalDocument = globalThis.document;
    const textEl = { textContent: '' };
    const tsEl = { textContent: '' };
    const panelEl = { classList: { toggle: () => {} } };

    globalThis.document = {
        getElementById: (id) => {
            if (id === 'humanoid-vfi-diagnostics-text') return textEl;
            if (id === 'humanoid-vfi-diagnostics-timestamp') return tsEl;
            if (id === 'humanoid-vfi-diagnostics-panel') return panelEl;
            return null;
        }
    };

    try {
        visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
        visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
        visualizer._getHumanoidRelativePose = () => relCurrent.slice();
        visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_vfi';
        visualizer._runHumanoidMJRestorePreWStage = async (options) => {
            callOrder.push({ stage: 'pre', options });
            return {
                state: 'active',
                acceptedCandidate: true,
                qApplied: qPre.slice()
            };
        };
        visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
            callOrder.push({ stage: 'restore', args });
            return true;
        };

        const result = await visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
            absTarget.slice(),
            q0.slice(),
            true,
            false,
            false,
            null,
            null
        );

        assert.equal(result, true);
        assert.deepEqual(callOrder.map((entry) => entry.stage), ['pre', 'restore']);
        assert.equal(callOrder[0].options.controlMode, 'absolute');
        assert.equal(callOrder[0].options.includeWaist, false);
        assert.equal(callOrder[0].options.keepBodyZVertical, false);
        assert.equal(callOrder[0].options.jogMetric, 'w_abs_hier');
        assert.equal(callOrder[0].options.metricDirection, 'increase');
        assert.deepEqual(callOrder[0].options.q0, q0);
        assert.deepEqual(callOrder[0].options.dqAbsoluteProtectedTargetArray, absCurrent);
        assert.deepEqual(callOrder[0].options.fixedRelativeProtectedTargetArray, relCurrent);
        assert.deepEqual(callOrder[1].args[0], absTarget);
        assert.deepEqual(callOrder[1].args[1], qPre);
        assert.deepEqual(callOrder[1].args[5], relCurrent);
        assert.equal(callOrder[1].args[7]?.solverMode, 'mj_restore_pre_w');
        assert.equal(callOrder[1].args[7]?.preWState, 'applied');
        assert.equal(callOrder[1].args[7]?.preWMetric, 'w_abs_hier');
        assert.equal(callOrder[1].args[7]?.preWStart, 'q_pre');
    } finally {
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer decoupled mj_restore_pre_w runs pre-w first with waist joints included and restores from q_pre while protecting current poses', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const qPre = [0.4, -0.1, 0.2];
    const absTarget = [1, 0, 0, 0, 0.02, 0, 0, 0];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const callOrder = [];

    visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
    visualizer._runHumanoidMJRestorePreWStage = async (options) => {
        callOrder.push({ stage: 'pre', options });
        return {
            state: 'active',
            acceptedCandidate: true,
            qApplied: qPre.slice()
        };
    };
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
        callOrder.push({ stage: 'restore', args });
        return true;
    };

    const result = await visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
        absTarget.slice(),
        q0.slice(),
        true,
        true,
        false,
        null,
        null
    );

    assert.equal(result, true);
    assert.deepEqual(callOrder.map((entry) => entry.stage), ['pre', 'restore']);
    assert.equal(callOrder[0].options.controlMode, 'decoupled');
    assert.equal(callOrder[0].options.includeWaist, true);
    assert.equal(callOrder[0].options.keepBodyZVertical, false);
    assert.deepEqual(callOrder[0].options.dqAbsoluteProtectedTargetArray, absCurrent);
    assert.deepEqual(callOrder[0].options.fixedRelativeProtectedTargetArray, relCurrent);
    assert.deepEqual(callOrder[1].args[1], qPre);
    assert.equal(callOrder[1].args[3], true);
    assert.equal(callOrder[1].args[4], false);
    assert.deepEqual(callOrder[1].args[5], relCurrent);
});

test('RobotVisualizer decoupled-upright mj_restore_pre_w runs pre-w first with waist joints, body-z protection, and restores from q_pre', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const qPre = [0.4, -0.1, 0.2];
    const absTarget = [1, 0, 0, 0, 0.02, 0, 0, 0];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const callOrder = [];

    visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
    visualizer._runHumanoidMJRestorePreWStage = async (options) => {
        callOrder.push({ stage: 'pre', options });
        return {
            state: 'active',
            acceptedCandidate: true,
            qApplied: qPre.slice()
        };
    };
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
        callOrder.push({ stage: 'restore', args });
        return true;
    };

    const result = await visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
        absTarget.slice(),
        q0.slice(),
        true,
        true,
        true,
        null,
        null
    );

    assert.equal(result, true);
    assert.deepEqual(callOrder.map((entry) => entry.stage), ['pre', 'restore']);
    assert.equal(callOrder[0].options.controlMode, 'decoupled-upright');
    assert.equal(callOrder[0].options.includeWaist, true);
    assert.equal(callOrder[0].options.keepBodyZVertical, true);
    assert.deepEqual(callOrder[0].options.dqAbsoluteProtectedTargetArray, absCurrent);
    assert.deepEqual(callOrder[0].options.fixedRelativeProtectedTargetArray, relCurrent);
    assert.deepEqual(callOrder[1].args[1], qPre);
    assert.equal(callOrder[1].args[3], true);
    assert.equal(callOrder[1].args[4], true);
    assert.deepEqual(callOrder[1].args[5], relCurrent);
});

test('RobotVisualizer absolute surface-slide forwards waist and body-z flags into cooperative absolute restoration', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const surfaceState = {
        surface: {
            position: (u, v) => [u, v, 0],
            du: () => [1, 0, 0],
            dv: () => [0, 1, 0]
        },
        u: 0.5,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidSurfaceSlideState = () => surfaceState;
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._extractPoseTranslationAndRotation = () => ({
        position: [0, 0, 0],
        quaternion: [1, 0, 0, 0]
    });
    visualizer._buildHumanoidSurfaceSlideDQTargetArray = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst = (...args) => {
        capturedArgs = args;
        return true;
    };

    try {
        const ok = visualizer._moveHumanoidSurfaceAlongParameter('x', 'positive', 'surface-slide-absolute');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[3], true);
        assert.equal(capturedArgs[4], true);
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer creates a DQ-motion ruled demo surface for humanoid surface-slide modes', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._extractPoseTranslationAndRotation = () => ({
        position: [0.2, -0.1, 0.3],
        quaternion: [1, 0, 0, 0]
    });

    const surface = visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose([1, 0, 0, 0, 0, 0, 0, 0]);
    const state = evaluateSurfacePatch(surface, 0.5, 0.5);

    assert.equal(surface.kind, 'dq_motion_ruled_surface');
    assert.equal(surface.name, 'humanoid-dq-ruled-surface');
    assert.deepEqual(state.position, [0.2, -0.1, 0.255]);
});

test('RobotVisualizer DQ-motion ruled demo surface keeps one-click orientation change moderate near the center', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._extractPoseTranslationAndRotation = () => ({
        position: [0, 0, 0],
        quaternion: [1, 0, 0, 0]
    });

    const surface = visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose([1, 0, 0, 0, 0, 0, 0, 0]);
    const pose0 = buildSurfaceAttachedPoseTarget({ surface, u: 0.5, v: 0.5, normalOffset: 0.06 });
    const pose1 = buildSurfaceAttachedPoseTarget({ surface, u: 0.52, v: 0.5, normalOffset: 0.06 });
    const dot = Math.abs(pose0.quaternion.reduce((sum, value, index) => sum + (value * pose1.quaternion[index]), 0));
    const angle = 2 * Math.acos(Math.min(1, Math.max(-1, dot)));

    assert.ok(angle <= ((1.5 * Math.PI) / 180), `expected <= 1.5 deg, got ${(angle * 180) / Math.PI}`);
});

test('RobotVisualizer surface-slide forwards a Plucker-tube invariant into single-arm restoration', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const surfaceState = {
        surface: {
            position: (u, v) => [u, v, 0],
            du: () => [1, 0, 0],
            dv: () => [0, 1, 0],
            lineAt: () => ({
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06, 0, 0],
                closestPoint: [0, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
            })
        },
        u: 0.5,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedInternalOptions = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidSurfaceSlideState = () => surfaceState;
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._buildHumanoidSurfaceSlideDQTargetArray = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({ solverMode: 'mj_restore_tr' });
    visualizer._updateHumanoidSingleArmPoseTargetMJRestore = (_dqTargetArray, _q, _armIndex, _includeWaist, _keepBodyZVertical, internalOptions) => {
        capturedInternalOptions = internalOptions;
        return true;
    };

    try {
        const ok = visualizer._moveHumanoidSurfaceAlongParameter('x', 'positive', 'surface-slide');
        assert.equal(ok, true);
        assert.ok(capturedInternalOptions?.surfaceTubeInvariant);
        assert.equal(capturedInternalOptions.surfaceTubeInvariant.kind, 'plucker_tube');
        assert.equal(capturedInternalOptions.surfaceTubeInvariant.line.kind, 'plucker_line');
        assert.equal(capturedInternalOptions.surfaceTubeInvariant.radius, 0.06);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer surface-slide-absolute forwards a Plucker-tube invariant into absolute restoration', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const surfaceState = {
        surface: {
            position: (u, v) => [u, v, 0],
            du: () => [1, 0, 0],
            dv: () => [0, 1, 0],
            lineAt: () => ({
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06, 0, 0],
                closestPoint: [0, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
            }),
            tubeAt: (_u, radius) => ({
                kind: 'plucker_tube',
                line: {
                    kind: 'plucker_line',
                    direction: [0, 1, 0],
                    moment: [0.06, 0, 0],
                    closestPoint: [0, 0, -0.06],
                    referenceCoordinate: 0,
                    pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
                },
                radius,
                radiusSquared: radius * radius
            })
        },
        u: 0.5,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidSurfaceSlideState = () => surfaceState;
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._extractPoseTranslationAndRotation = () => ({
        position: [0, 0, 0],
        quaternion: [1, 0, 0, 0]
    });
    visualizer._buildHumanoidSurfaceSlideDQTargetArray = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst = (...args) => {
        capturedArgs = args;
        return true;
    };

    try {
        const ok = visualizer._moveHumanoidSurfaceAlongParameter('x', 'positive', 'surface-slide-absolute');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[3], true);
        assert.equal(capturedArgs[4], true);
        assert.equal(capturedArgs[7]?.surfaceTubeInvariant?.kind, 'plucker_tube');
        assert.equal(capturedArgs[7]?.surfaceTubeInvariant?.line?.kind, 'plucker_line');
        assert.equal(capturedArgs[7]?.surfaceTubeInvariant?.radius, 0.06);
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer dual independent surface-slide forwards left/right targets and Plucker-tube invariants into bimanual restoration', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const dualState = {
        modeKey: 'dual-independent-surface-slide',
        surface: sharedSurface,
        left: {
            u: 0.25,
            v: 0.5,
            normalOffset: 0.06,
            normalOffsetBounds: [0, 0.16]
        },
        right: {
            u: 0.75,
            v: 0.5,
            normalOffset: 0.06,
            normalOffsetBounds: [0, 0.16]
        },
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidDualIndependentSurfaceSlideState = () => dualState;
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._buildHumanoidSurfaceSlideDQTargetArray = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer.updateHumanoidJointsFromDualIndependentSurfaceTargets = (...args) => {
        capturedArgs = args;
        return true;
    };

    try {
        const ok = visualizer._moveHumanoidDualIndependentSurfaceAlongParameter('x', 'positive', 'left');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[3], true);
        assert.equal(capturedArgs[4], true);
        assert.equal(capturedArgs[5], true);
        assert.equal(capturedArgs[6]?.leftTubeInvariant?.kind, 'plucker_tube');
        assert.equal(capturedArgs[6]?.rightTubeInvariant?.kind, 'plucker_tube');
        assert.equal(capturedArgs[6]?.leftTubeInvariant?.line?.kind, 'plucker_line');
        assert.equal(capturedArgs[6]?.rightTubeInvariant?.line?.kind, 'plucker_line');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer dual independent surface-slide initializes from current left/right surface attachment states', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidDualIndependentSurfaceSlideControlMode = () => 'dual-independent-surface-slide';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = () => planeSurface;
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.2, 0.4, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [0, 0, 0, 1, 0.8, 0.6, 0.06, 0];

    try {
        const state = visualizer._ensureHumanoidDualIndependentSurfaceSlideState();
        assert.ok(state);
        assert.ok(Math.abs(state.left.u - 0.2) < 0.08);
        assert.ok(Math.abs(state.left.v - 0.4) < 0.08);
        assert.ok(Math.abs(state.left.normalOffset - 0.06) < 0.02);
        assert.ok(Math.abs(state.right.u - 0.8) < 0.08);
        assert.ok(Math.abs(state.right.v - 0.6) < 0.08);
        assert.ok(Math.abs(state.right.normalOffset - 0.06) < 0.02);

        const leftTarget = visualizer._buildHumanoidDualIndependentSurfacePoseTarget(state, 'left');
        const rightTarget = visualizer._buildHumanoidDualIndependentSurfacePoseTarget(state, 'right');
        assert.deepEqual(leftTarget.position.map((value) => Number(value.toFixed(6))), [0.2, 0.4, 0.06]);
        assert.deepEqual(rightTarget.position.map((value) => Number(value.toFixed(6))), [0.8, 0.6, 0.06]);
        assert.deepEqual(leftTarget.quaternion.map((value) => Number(value.toFixed(6))), [1, 0, 0, 0]);
        assert.deepEqual(rightTarget.quaternion.map((value) => Number(value.toFixed(6))), [0, 0, 0, 1]);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing initializes shared strip coordinates from current left/right surface attachment states', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.32, 0.5, 0.06, 0];
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = () => planeSurface;
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.62, 0.06, 0];

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.ok(Math.abs(state.u - 0.32) < 0.08);
        assert.ok(Math.abs(state.vCenter - 0.5) < 0.08);
        assert.ok(Math.abs(state.stripSpacing - 0.24) < 0.08);
        assert.ok(Math.abs(state.normalOffset - 0.05) < 0.02);

        const leftTarget = visualizer._buildHumanoidCooperativeRuledSurfacePolishingPoseTargets(state).left;
        const rightTarget = visualizer._buildHumanoidCooperativeRuledSurfacePolishingPoseTargets(state).right;
        assert.deepEqual(leftTarget.position.map((value) => Number(value.toFixed(6))), [0.32, 0.38, 0.05]);
        assert.deepEqual(rightTarget.position.map((value) => Number(value.toFixed(6))), [0.32, 0.62, 0.05]);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing uses a fixed object pose when one is already set', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    let capturedSurfacePlacementPose = null;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.11, -0.22, -0.33],
            quaternion: [1, 0, 0, 0]
        }
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.32, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.62, 0.06, 0];
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = (poseLike) => {
        capturedSurfacePlacementPose = poseLike;
        return planeSurface;
    };

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.deepEqual(capturedSurfacePlacementPose.position, [0.11, -0.22, -0.33]);
        assert.deepEqual(capturedSurfacePlacementPose.quaternion, [1, 0, 0, 0]);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing forwards fixed object-pose normal flipping into demo surface creation', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    let capturedSurfacePlacementPose = null;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.11, -0.22, -0.33],
            quaternion: [1, 0, 0, 0],
            flipNormal: true
        }
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.32, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.62, 0.06, 0];
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = (poseLike) => {
        capturedSurfacePlacementPose = poseLike;
        return planeSurface;
    };

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(capturedSurfacePlacementPose.flipNormal, true);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing capture applies a requested normal flip exactly once', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0, 0, 0],
            quaternion: [1, 0, 0, 0],
            flipNormal: false,
            normalFlipApplied: false
        }
    };

    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.32, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.62, 0.06, 0];
    visualizer._getHumanoidCooperativeRuledSurfaceAnchorPose = () => ({
        position: [0.11, -0.22, -0.33],
        quaternion: [1, 0, 0, 0]
    });

    try {
        const capturedPose = visualizer.captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands({
            flipNormal: true
        });
        assert.equal(capturedPose.position.length, 3);
        assert.ok(capturedPose.position.every(Number.isFinite));
        assert.deepEqual(capturedPose.quaternion, [0, 0, 1, 0]);
        assert.equal(capturedPose.flipNormal, true);
        assert.equal(capturedPose.normalFlipApplied, true);
        assert.deepEqual(globalThis.window.humanoidCooperativeRuledSurfaceObjectPose, capturedPose);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing initializes captured hand-facing strip targets with a 5cm surface offset', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.11, -0.22, -0.33],
            quaternion: [1, 0, 0, 0],
            flipNormal: false
        }
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.32, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.14, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.62, 0.12, 0];
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = () => planeSurface;

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(Number(state.normalOffset.toFixed(6)), 0.05);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing capture with flipNormal keeps the captured strip close to the current hands', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;

    const rotateByQuaternion = (vector, quaternionLike) => {
        const [w, x, y, z] = quaternionLike;
        const vx = vector[0];
        const vy = vector[1];
        const vz = vector[2];
        const ix = (w * vx) + (y * vz) - (z * vy);
        const iy = (w * vy) + (z * vx) - (x * vz);
        const iz = (w * vz) + (x * vy) - (y * vx);
        const iw = -((x * vx) + (y * vy) + (z * vz));
        return [
            (ix * w) + (iw * -x) + (iy * -z) - (iz * -y),
            (iy * w) + (iw * -y) + (iz * -x) - (ix * -z),
            (iz * w) + (iw * -z) + (ix * -y) - (iy * -x)
        ];
    };
    const add3 = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
    const scale3 = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
    const subtract3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    const norm3 = (a) => Math.hypot(a[0], a[1], a[2]);

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    const leftPose = [1, 0, 0, 0, 0.0, -0.10, 0.05, 0];
    const rightPose = [1, 0, 0, 0, 0.0, 0.10, 0.05, 0];
    const absolutePose = [1, 0, 0, 0, 0.0, 0.0, 0.05, 0];

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidPose1 = () => leftPose;
    visualizer._getHumanoidPose2 = () => rightPose;
    visualizer._getHumanoidAbsolutePose = () => absolutePose;
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = (poseLike) => {
        const pose = visualizer._normalizeHumanoidCooperativeRuledSurfaceObjectPose(poseLike);
        const tangentU = rotateByQuaternion([1, 0, 0], pose.quaternion);
        const tangentV = rotateByQuaternion([0, 1, 0], pose.quaternion);
        const origin = pose.position.slice();
        return {
            position: (u, v) => add3(origin, add3(scale3(tangentU, u - 0.5), scale3(tangentV, v - 0.5))),
            du: () => tangentU.slice(),
            dv: () => tangentV.slice()
        };
    };

    try {
        visualizer.captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands({ flipNormal: true });
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        const targets = visualizer._buildHumanoidCooperativeRuledSurfacePolishingPoseTargets(state);
        assert.ok(targets?.left?.position && targets?.right?.position);

        const leftError = norm3(subtract3(targets.left.position, [leftPose[4], leftPose[5], leftPose[6]]));
        const rightError = norm3(subtract3(targets.right.position, [rightPose[4], rightPose[5], rightPose[6]]));

        assert.ok(leftError < 1e-3, `expected flipped capture to keep left target near the current hand, got error=${leftError}`);
        assert.ok(rightError < 1e-3, `expected flipped capture to keep right target near the current hand, got error=${rightError}`);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing refines its shared strip state to reduce the current hand-position error', () => {
    const visualizer = new RobotVisualizer(null);
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const leftPose = [1, 0, 0, 0, 0.42, 0.40, 0.05, 0];
    const rightPose = [1, 0, 0, 0, 0.42, 0.60, 0.05, 0];
    let extractCall = 0;
    visualizer._extractPoseTranslationAndRotation = () => {
        extractCall += 1;
        return extractCall % 2 === 1
            ? { position: [0.42, 0.40, 0.05], quaternion: [1, 0, 0, 0] }
            : { position: [0.42, 0.60, 0.05], quaternion: [1, 0, 0, 0] };
    };
    const initialState = visualizer._normalizeHumanoidCooperativeRuledSurfacePolishingState({
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: planeSurface,
        u: 0.32,
        vCenter: 0.48,
        stripSpacing: 0.12,
        normalOffset: 0.05,
        normalOffsetSign: 1,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftVSign: 1,
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    });

    const initialCost = visualizer._computeHumanoidCooperativeRuledSurfacePolishingPositionCost(initialState, leftPose, rightPose);
    const refined = visualizer._refineHumanoidCooperativeRuledSurfacePolishingStateToCurrentHands(initialState, leftPose, rightPose);
    const refinedCost = visualizer._computeHumanoidCooperativeRuledSurfacePolishingPositionCost(refined, leftPose, rightPose);

    assert.ok(refinedCost < initialCost, `expected refinement to reduce position cost, got initial=${initialCost} refined=${refinedCost}`);
    assert.ok(Math.abs(refined.u - 0.42) < 1e-3, `expected refined shared u near 0.42, got ${refined.u}`);
    assert.ok(Math.abs(refined.normalOffset - 0.05) < 0.01, `expected refined normal offset to stay near the realized hand offset, got ${refined.normalOffset}`);
});

test('RobotVisualizer cooperative ruled-surface polishing preserves the initial left/right v-side ordering when building arm targets', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.62, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = () => planeSurface;

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(state.leftVSign, 1);
        const targets = visualizer._buildHumanoidCooperativeRuledSurfacePolishingPoseTargets(state);
        assert.ok(targets.left.position[1] > targets.right.position[1], `expected left target to stay on the higher-v side, got left=${JSON.stringify(targets.left.position)} right=${JSON.stringify(targets.right.position)}`);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing falls back to analytic when a removed shell source is requested', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceSource: 'strip-ruled-shell'
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.68, 0.62, 0.06, 0];

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(state.surface.kind, 'dq_motion_ruled_surface');
        assert.equal(visualizer._getHumanoidCooperativeRuledSurfaceSource(), 'analytic');
        assert.equal(typeof state.surface.lineAt, 'function');
        assert.equal(typeof state.surface.tubeAt, 'function');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing can build an analytic cylindrical demo surface when that source is active', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceSource: 'cylindrical-patch'
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.68, 0.62, 0.06, 0];
    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(state.surface.kind, 'analytic_cylindrical_ruled_surface');
        assert.equal(state.surface.metadata.source, 'cylindrical-patch');
        assert.equal(state.surface.metadata.parameterization, 'analytic-cylinder');
        const center = state.surface.position(0.5, 0.5);
        const edge = state.surface.position(0, 0.5);
        const relativeEdge = edge.map((value, index) => value - center[index]);
        const handSpan = [0.68 - 0.32, 0.62 - 0.38, 0];
        const handSpanNorm = Math.hypot(...handSpan);
        const handSpanUnit = handSpan.map((value) => value / handSpanNorm);
        const relativeEdgeAlongSpan = relativeEdge[0] * handSpanUnit[0]
            + relativeEdge[1] * handSpanUnit[1]
            + relativeEdge[2] * handSpanUnit[2];
        assert.ok(Math.abs(relativeEdgeAlongSpan) < 1e-9, `expected cylindrical cross-section edge to stay orthogonal to the shared wiping span, got relative edge=${JSON.stringify(relativeEdge)}`);
        assert.ok(Math.hypot(...relativeEdge) > 0.045, `expected remapped cylindrical patch to preserve a visibly curved cylindrical depth after rotation, got relative edge=${JSON.stringify(relativeEdge)}`);
        const span = state.surface.position(0.5, 1).map((value, index) => value - state.surface.position(0.5, 0)[index]);
        const spanNorm = Math.hypot(...span);
        const spanUnit = span.map((value) => value / spanNorm);
        const spanAlignment = Math.abs(spanUnit[0] * handSpanUnit[0]
            + spanUnit[1] * handSpanUnit[1]
            + spanUnit[2] * handSpanUnit[2]);
        assert.ok(spanAlignment > 0.97, `expected cylindrical strip rulings to align strongly with the shared wiping span after the 90-degree rotation, got span=${JSON.stringify(span)}`);
        assert.equal(typeof state.surface.lineAt, 'function');
        assert.equal(typeof state.surface.tubeAt, 'function');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing anchors its demo surface below the bimanual work area instead of at the torso-centered pose height', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    let capturedAnchorPose = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.08, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.68, 0.62, 0.06, 0];
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = (poseLike) => {
        capturedAnchorPose = poseLike;
        return planeSurface;
    };

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.ok(capturedAnchorPose);
        assert.equal(Number(capturedAnchorPose.position[0].toFixed(6)), 0.5);
        assert.equal(Number(capturedAnchorPose.position[1].toFixed(6)), 0.5);
        assert.ok(
            capturedAnchorPose.position[2] < 0.06 - 0.09,
            `expected polishing surface anchor to sit below the hands, got z=${capturedAnchorPose.position[2]}`
        );
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing anchor aligns the surface v-axis with the left-right hand span', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const leftPose = [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    const rightPose = [1, 0, 0, 0, 0.68, 0.62, 0.06, 0];
    const absolutePose = [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    globalThis.window = {};

    try {
        const anchor = visualizer._getHumanoidCooperativeRuledSurfaceAnchorPose(leftPose, rightPose, absolutePose);
        const [w, x, y, z] = anchor.quaternion;
        const rotateByQuaternion = (vector) => {
            const [vx, vy, vz] = vector;
            const ix = (w * vx) + (y * vz) - (z * vy);
            const iy = (w * vy) + (z * vx) - (x * vz);
            const iz = (w * vz) + (x * vy) - (y * vx);
            const iw = (-x * vx) - (y * vy) - (z * vz);
            return [
                (ix * w) + (iw * -x) + (iy * -z) - (iz * -y),
                (iy * w) + (iw * -y) + (iz * -x) - (ix * -z),
                (iz * w) + (iw * -z) + (ix * -y) - (iy * -x)
            ];
        };
        const span = [0.36, 0.24, 0];
        const spanNorm = Math.hypot(...span);
        const spanUnit = span.map((value) => value / spanNorm);
        const vAxis = rotateByQuaternion([0, 1, 0]);
        const alignment = (vAxis[0] * spanUnit[0]) + (vAxis[1] * spanUnit[1]) + (vAxis[2] * spanUnit[2]);

        assert.ok(alignment > 0.999, `expected v-axis to align with hand span, got dot=${alignment}`);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing anchor aligns the surface normal with the blended hand x-axis projected off the span direction', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const leftPose = [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    const rightPose = [1, 0, 0, 0, 0.68, 0.62, 0.06, 0];
    const absolutePose = [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    globalThis.window = {};

    try {
        const anchor = visualizer._getHumanoidCooperativeRuledSurfaceAnchorPose(leftPose, rightPose, absolutePose);
        const [w, x, y, z] = anchor.quaternion;
        const rotateByQuaternion = (vector) => {
            const [vx, vy, vz] = vector;
            const ix = (w * vx) + (y * vz) - (z * vy);
            const iy = (w * vy) + (z * vx) - (x * vz);
            const iz = (w * vz) + (x * vy) - (y * vx);
            const iw = (-x * vx) - (y * vy) - (z * vz);
            return [
                (ix * w) + (iw * -x) + (iy * -z) - (iz * -y),
                (iy * w) + (iw * -y) + (iz * -x) - (ix * -z),
                (iz * w) + (iw * -z) + (ix * -y) - (iy * -x)
            ];
        };
        const zAxis = rotateByQuaternion([0, 0, 1]);
        const span = [0.36, 0.24, 0];
        const spanNorm = Math.hypot(...span);
        const spanUnit = span.map((value) => value / spanNorm);
        const projectedHandX = [
            1 - (spanUnit[0] * spanUnit[0]),
            -(spanUnit[0] * spanUnit[1]),
            0
        ];
        const projectedNorm = Math.hypot(...projectedHandX);
        const projectedUnit = projectedHandX.map((value) => value / projectedNorm);
        const alignment = (zAxis[0] * projectedUnit[0]) + (zAxis[1] * projectedUnit[1]) + (zAxis[2] * projectedUnit[2]);

        assert.ok(alignment > 0.999, `expected surface normal to follow the projected hand x-axis, got z-axis=${JSON.stringify(zAxis)} and dot=${alignment}`);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing anchor prefers the blended hand x-axis for the surface normal over the absolute pose heading', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const handQuaternion = [Math.SQRT1_2, 0, -Math.SQRT1_2, 0];
    const leftPose = [handQuaternion[0], handQuaternion[1], handQuaternion[2], handQuaternion[3], 0.5, -0.1, 0.06, 0];
    const rightPose = [handQuaternion[0], handQuaternion[1], handQuaternion[2], handQuaternion[3], 0.5, 0.1, 0.06, 0];
    const absolutePose = [1, 0, 0, 0, 0.5, 0, 0.06, 0];
    globalThis.window = {};

    try {
        const anchor = visualizer._getHumanoidCooperativeRuledSurfaceAnchorPose(leftPose, rightPose, absolutePose);
        const [w, x, y, z] = anchor.quaternion;
        const rotateByQuaternion = (vector) => {
            const [vx, vy, vz] = vector;
            const ix = (w * vx) + (y * vz) - (z * vy);
            const iy = (w * vy) + (z * vx) - (x * vz);
            const iz = (w * vz) + (x * vy) - (y * vx);
            const iw = (-x * vx) - (y * vy) - (z * vz);
            return [
                (ix * w) + (iw * -x) + (iy * -z) - (iz * -y),
                (iy * w) + (iw * -y) + (iz * -x) - (ix * -z),
                (iz * w) + (iw * -z) + (ix * -y) - (iy * -x)
            ];
        };
        const zAxis = rotateByQuaternion([0, 0, 1]);
        const normalAlignment = zAxis[2];

        assert.ok(
            normalAlignment > 0.999,
            `expected anchor normal to follow the blended hand x-axis instead of the absolute pose, got ${JSON.stringify(zAxis)}`
        );
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing uses a larger demo surface footprint', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    let capturedMetadata = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.42, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.58, 0.62, 0.06, 0];
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = (poseLike, options = {}) => {
        capturedMetadata = options;
        return {
            position: (u, v) => [u, v, 0],
            du: () => [1, 0, 0],
            dv: () => [0, 1, 0],
            lineAt: (u) => ({
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            }),
            tubeAt: (u, radius) => ({
                kind: 'plucker_tube',
                line: {
                    kind: 'plucker_line',
                    direction: [0, 1, 0],
                    moment: [0, 0, u],
                    closestPoint: [u, 0, 0],
                    referenceCoordinate: 0,
                    pointAtCoordinate: (coordinate) => [u, coordinate, 0]
                },
                radius,
                radiusSquared: radius * radius
            }),
            metadata: {
                spanU: options.spanU,
                spanV: options.spanV
            }
        };
    };

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.ok(capturedMetadata);
        const handSpan = Math.hypot(0.58 - 0.42, 0.62 - 0.38, 0);
        assert.equal(capturedMetadata.spanU, 0.3);
        assert.ok(Math.abs(capturedMetadata.spanV - Math.min(0.6, Math.max(0.26, handSpan + 0.14))) < 1e-9);
        assert.ok(capturedMetadata.spanV > capturedMetadata.spanU);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing biases its demo surface downward with a downward-opening parabolic lift', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    let capturedMetadata = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.5, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.42, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.58, 0.62, 0.06, 0];
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = (_poseLike, options = {}) => {
        capturedMetadata = options;
        return {
            position: (u, v) => [u, v, 0],
            du: () => [1, 0, 0],
            dv: () => [0, 1, 0],
            lineAt: (u) => ({
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            }),
            tubeAt: (u, radius) => ({
                kind: 'plucker_tube',
                line: {
                    kind: 'plucker_line',
                    direction: [0, 1, 0],
                    moment: [0, 0, u],
                    closestPoint: [u, 0, 0],
                    referenceCoordinate: 0,
                    pointAtCoordinate: (coordinate) => [u, coordinate, 0]
                },
                radius,
                radiusSquared: radius * radius
            })
        };
    };

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        assert.ok(capturedMetadata);
        assert.equal(capturedMetadata.centerOffset, -0.05);
        assert.equal(capturedMetadata.liftAmplitude, -0.045);
        assert.equal(capturedMetadata.twistAngle, 0);
        assert.equal(capturedMetadata.flipU, true);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing keeps its initial contact targets on the hand-facing side of the curved surface', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0.85, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, -0.22, 0.25, 0.88, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.22, -0.25, 0.88, 0];

    try {
        const state = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        assert.ok(state);
        const leftArmState = visualizer._buildHumanoidCooperativeRuledSurfacePolishingArmState(state, 'left');
        const rightArmState = visualizer._buildHumanoidCooperativeRuledSurfacePolishingArmState(state, 'right');
        const targetPoses = visualizer._buildHumanoidCooperativeRuledSurfacePolishingPoseTargets(state);
        const leftFrame = buildSurfaceAttachedFrame(evaluateSurfacePatch(state.surface, leftArmState.u, leftArmState.v));
        const rightFrame = buildSurfaceAttachedFrame(evaluateSurfacePatch(state.surface, rightArmState.u, rightArmState.v));

        assert.ok(leftArmState.normalOffset > 0.01, `expected left contact offset to stay on the hand-facing side, got ${leftArmState.normalOffset}`);
        assert.ok(rightArmState.normalOffset > 0.01, `expected right contact offset to stay on the hand-facing side, got ${rightArmState.normalOffset}`);
        assert.ok(targetPoses.left.position[2] > leftFrame.position[2], 'expected left target to sit above the curved surface');
        assert.ok(targetPoses.right.position[2] > rightFrame.position[2], 'expected right target to sit above the curved surface');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer projects a pose on the opposite side of a surface with a negative normalOffsetSign', () => {
    const visualizer = new RobotVisualizer(null);
    const surface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    visualizer._extractPoseTranslationAndRotation = () => ({
        position: [0.25, 0.5, -0.08],
        quaternion: [1, 0, 0, 0]
    });

    const projected = visualizer._projectHumanoidPoseToSurfaceState(
        surface,
        [1, 0, 0, 0, 0.25, 0.5, -0.08, 0],
        {
            initialGuess: { u: 0.25, v: 0.5 },
            bounds: { u: [0, 1], v: [0, 1] },
            normalOffsetBounds: [0, 0.16]
        }
    );

    assert.ok(projected);
    assert.equal(projected.normalOffsetSign, -1);
    assert.equal(Number(projected.normalOffset.toFixed(6)), 0.08);
});

test('RobotVisualizer left ruled-surface polishing initializes from the current left-hand surface attachment state', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidLeftRuledSurfacePolishingControlMode = () => 'left-ruled-surface-polishing';
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = () => planeSurface;

    try {
        const state = visualizer._ensureHumanoidLeftRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(state.modeKey, 'left-ruled-surface-polishing');
        assert.equal(state.surface, planeSurface);
        assert.ok(Math.abs(state.u - 0.32) < 0.08);
        assert.ok(Math.abs(state.v - 0.38) < 0.08);
        assert.ok(Math.abs(state.normalOffset - 0.06) < 0.02);
        assert.deepEqual(state.bounds, { u: [0, 1], v: [0, 1] });
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer left ruled-surface polishing reuses the cooperative cylindrical sample and object pose when available', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    let capturedPose = null;
    let capturedOptions = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceSource: 'cylindrical-patch',
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.11, -0.22, -0.33],
            quaternion: [1, 0, 0, 0]
        }
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidLeftRuledSurfacePolishingControlMode = () => 'left-ruled-surface-polishing';
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = (poseLike, options) => {
        capturedPose = poseLike;
        capturedOptions = options;
        return planeSurface;
    };
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = () => {
        throw new Error('left ruled-surface polishing should not use the legacy surface-slide surface source');
    };

    try {
        const state = visualizer._ensureHumanoidLeftRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(state.surface, planeSurface);
        assert.deepEqual(capturedPose.position, [0.11, -0.22, -0.33]);
        assert.deepEqual(capturedPose.quaternion, [1, 0, 0, 0]);
        assert.equal(capturedOptions.surfaceSource, 'cylindrical-patch');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer left ruled-surface polishing falls back to the cooperative anchor pose and shared hand span when no object pose is set', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const leftPose = [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    const rightPose = [1, 0, 0, 0, 0.68, 0.62, 0.06, 0];
    const absolutePose = [1, 0, 0, 0, 0.50, 0.50, 0.06, 0];
    const anchorPose = {
        position: [0.5, 0.5, -0.06],
        quaternion: [1, 0, 0, 0]
    };
    let capturedPose = null;
    let capturedOptions = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceSource: 'cylindrical-patch'
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidLeftRuledSurfacePolishingControlMode = () => 'left-ruled-surface-polishing';
    visualizer._getHumanoidPose1 = () => leftPose;
    visualizer._getHumanoidPose2 = () => rightPose;
    visualizer._getHumanoidAbsolutePose = () => absolutePose;
    visualizer._getHumanoidCooperativeRuledSurfaceAnchorPose = () => anchorPose;
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = (poseLike, options) => {
        capturedPose = poseLike;
        capturedOptions = options;
        return planeSurface;
    };
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = () => {
        throw new Error('left ruled-surface polishing should still route fallback placement through the cooperative surface builder');
    };

    try {
        const state = visualizer._ensureHumanoidLeftRuledSurfacePolishingState();
        assert.ok(state);
        assert.equal(state.surface, planeSurface);
        assert.deepEqual(capturedPose.position.map((value) => Number(value.toFixed(6))), [0.5, 0.5, -0.15]);
        assert.deepEqual(capturedPose.quaternion, [1, 0, 0, 0]);
        assert.ok(Math.abs(capturedOptions.spanV - 0.572666) < 1e-6);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer left ruled-surface polishing builds an explicit single-strip differential target for [u, v, n]', () => {
    const visualizer = new RobotVisualizer(null);
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const currentState = {
        modeKey: 'left-ruled-surface-polishing',
        surface: planeSurface,
        u: 0.35,
        v: 0.5,
        normalOffset: 0.05,
        normalOffsetSign: 1,
        normalOffsetBounds: [0, 0.16],
        quaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    const nextState = {
        ...currentState,
        u: 0.36,
        v: 0.52,
        normalOffset: 0.06
    };

    const explicitCommand = visualizer._buildHumanoidLeftRuledSurfacePolishingExplicitCommand(
        currentState,
        nextState
    );

    assert.deepEqual(
        explicitCommand.parameterDelta.map((value) => Number(value.toFixed(6))),
        [0.01, 0.02, 0.01]
    );
    assert.equal(explicitCommand.surfaceJacobian.length, 8);
    assert.equal(explicitCommand.surfaceJacobian[0].length, 3);
    assert.deepEqual(
        explicitCommand.commandTargetArray.map((value) => Number(value.toFixed(6))),
        [0, 0, 0, 0, 0, 0.005, 0.01, 0.005]
    );
});

test('RobotVisualizer cooperative ruled-surface polishing builds an explicit chain-rule differential target for shared-strip parameters', () => {
    const visualizer = new RobotVisualizer(null);
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const currentState = visualizer._normalizeHumanoidCooperativeRuledSurfacePolishingState({
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.50,
        stripSpacing: 0.20,
        normalOffset: 0.05,
        normalOffsetSign: 1,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftVSign: 1,
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    });
    const nextState = visualizer._normalizeHumanoidCooperativeRuledSurfacePolishingState({
        ...visualizer._cloneHumanoidCooperativeRuledSurfacePolishingState(currentState),
        u: 0.36,
        vCenter: 0.52,
        stripSpacing: 0.24,
        normalOffset: 0.06
    });

    const explicitCommand = visualizer._buildHumanoidCooperativeRuledSurfacePolishingExplicitCommand(
        currentState,
        nextState
    );

    assert.deepEqual(
        explicitCommand.parameterDelta.map((value) => Number(value.toFixed(6))),
        [0.01, 0.02, 0.04, 0.01]
    );
    assert.equal(explicitCommand.cooperativeJacobian.length, 16);
    assert.equal(explicitCommand.cooperativeJacobian[0].length, 4);
    assert.deepEqual(
        explicitCommand.commandTargetArray.map((value) => Number(value.toFixed(6))),
        [
            0, 0, 0, 0, 0, 0.005, 0.02, 0.005,
            0, 0, 0, 0, 0, 0.005, 0, 0.005
        ]
    );
});

test('RobotVisualizer cooperative ruled-surface polishing forwards explicit shared-strip differentials without tube invariants into bimanual restoration', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async (...args) => {
        capturedArgs = args.map((value, index) => (index <= 1 && value && typeof value === 'object')
            ? JSON.parse(JSON.stringify(value))
            : value);
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: false,
            applied: true,
            reason: 'best_effort',
            message: 'Restore (LS) 保留最后可行小步。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.37);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(Number(capturedArgs[0]?.u?.toFixed(3)), 0.35);
        assert.equal(Number(capturedArgs[1]?.u?.toFixed(3)), 0.37);
        assert.equal(capturedArgs[3], true);
        assert.equal(capturedArgs[4], true);
        assert.equal(capturedArgs[5], true);
        assert.equal(capturedArgs[6]?.controlModeOverride, 'cooperative-ruled-surface-polishing');
        assert.equal(capturedArgs[6]?.statusModeOverride, 'humanoid_cooperative_ruled_surface_polishing');
        assert.equal(capturedArgs[6]?.bestEffortAppliedCommandNormCap, 0.005);
        assert.equal(capturedArgs[6]?.bestEffortAppliedInvariantNormCap, 0.002);
        assert.equal(capturedArgs[6]?.requireHardConvergenceToApply, undefined);
        assert.equal(capturedArgs[6]?.leftTubeInvariant, undefined);
        assert.equal(capturedArgs[6]?.rightTubeInvariant, undefined);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing can route into the parallel DQ set-point mode for comparison', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing-dq-setpoint',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing-dq-setpoint';
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDQSetpointMJRestore = async (...args) => {
        capturedArgs = args;
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing_dq_setpoint',
            converged: false,
            applied: true,
            reason: 'best_effort',
            message: '恢复优化(TR) 保留最后可行小步。'
        };
        return true;
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        throw new Error('legacy differential path should not be used for dq set-point mode');
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.37);
        assert.equal(Number(capturedArgs[1]?.u?.toFixed(3)), 0.37);
        assert.equal(capturedArgs[6]?.controlModeOverride, 'cooperative-ruled-surface-polishing-dq-setpoint');
        assert.equal(capturedArgs[6]?.statusModeOverride, 'humanoid_cooperative_ruled_surface_polishing_dq_setpoint');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing DQ set-point mode forwards dedicated WASM targets into the custom restore path', async () => {
    const visualizer = new RobotVisualizer(null);
    const leftTarget = new Array(8).fill(0.1);
    const rightTarget = new Array(8).fill(-0.1);
    let observedOptions = null;

    visualizer._buildHumanoidCooperativeRuledSurfacePolishingDQSetpointTargets = () => ({
        leftDQTargetArray: leftTarget.slice(),
        rightDQTargetArray: rightTarget.slice(),
        poseTargets: [{}, {}]
    });
    visualizer._getHumanoidJointLimits = () => new Array(20).fill(null).map(() => ({ lower: -1, upper: 1 }));
    visualizer._humanoidSingleArmPlusWaistCoordinatedIndices = () => [0, 1, 2];
    visualizer._updateHumanoidCustomPoseTargetMJRestore = async (options) => {
        observedOptions = options;
        return true;
    };

    await visualizer._updateHumanoidCooperativeRuledSurfacePolishingDQSetpointMJRestore(
        { modeKey: 'cooperative-ruled-surface-polishing-dq-setpoint' },
        { modeKey: 'cooperative-ruled-surface-polishing-dq-setpoint' },
        new Array(20).fill(0),
        true,
        true,
        true,
        { solverMode: 'mj_restore_tr' }
    );

    assert.ok(observedOptions);
    assert.equal(observedOptions.internalOptions.wasmSolveVariant, 'cooperative_dq_setpoint');
    assert.deepEqual(observedOptions.internalOptions.wasmLeftTarget, leftTarget);
    assert.deepEqual(observedOptions.internalOptions.wasmRightTarget, rightTarget);
});

test('RobotVisualizer cooperative ruled-surface polishing arm-only DQ set-point mode disables waist and body-z in the custom restore path', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only';
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDQSetpointMJRestore = async (...args) => {
        capturedArgs = args;
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[4], false);
        assert.equal(capturedArgs[5], false);
        assert.equal(capturedArgs[6]?.controlModeOverride, 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing waist-maintained DQ set-point mode keeps waist and body-z in the custom restore path', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing-dq-setpoint',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing-dq-setpoint';
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDQSetpointMJRestore = async (...args) => {
        capturedArgs = args;
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[4], true);
        assert.equal(capturedArgs[5], true);
        assert.equal(capturedArgs[6]?.controlModeOverride, 'cooperative-ruled-surface-polishing-dq-setpoint');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer DQ set-point error model uses the right constrained Jacobian operator from the reference formulation', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const identityDQ = [1, 0, 0, 0, 0, 0, 0, 0];
    const targetDQ = [
        Math.SQRT1_2,
        0,
        0,
        Math.SQRT1_2,
        0,
        0,
        0,
        0
    ];
    const poseJacobian = [
        [1, 2],
        [3, 4],
        [5, 6],
        [7, 8],
        [9, 10],
        [11, 12],
        [13, 14],
        [15, 16]
    ];

    const multiplyQuaternion = (left, right) => {
        const [lw, lx, ly, lz] = left;
        const [rw, rx, ry, rz] = right;
        return [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw
        ];
    };

    const addQuaternion = (left, right) => left.map((value, index) => value + right[index]);

    const conjugateDQ = (dq) => [
        dq[0], -dq[1], -dq[2], -dq[3],
        dq[4], -dq[5], -dq[6], -dq[7]
    ];

    const multiplyDQ = (left, right) => {
        const leftPrimal = left.slice(0, 4);
        const leftDual = left.slice(4, 8);
        const rightPrimal = right.slice(0, 4);
        const rightDual = right.slice(4, 8);
        const primal = multiplyQuaternion(leftPrimal, rightPrimal);
        const dual = addQuaternion(
            multiplyQuaternion(leftPrimal, rightDual),
            multiplyQuaternion(leftDual, rightPrimal)
        );
        return primal.concat(dual);
    };

    const buildRightMatrix = (rightDQ) => {
        const matrix = Array.from({ length: 8 }, () => new Array(8).fill(0));
        for (let column = 0; column < 8; column += 1) {
            const basis = new Array(8).fill(0);
            basis[column] = 1;
            const resultColumn = multiplyDQ(basis, rightDQ);
            for (let row = 0; row < 8; row += 1) {
                matrix[row][column] = resultColumn[row];
            }
        }
        return matrix;
    };

    const multiplyMatrix = (left, right) => {
        const out = Array.from({ length: left.length }, () => new Array(right[0].length).fill(0));
        for (let row = 0; row < left.length; row += 1) {
            for (let column = 0; column < right[0].length; column += 1) {
                let sum = 0;
                for (let k = 0; k < right.length; k += 1) {
                    sum += left[row][k] * right[k][column];
                }
                out[row][column] = sum;
            }
        }
        return out;
    };

    globalThis.window = {
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                conjugate: conjugateDQ,
                multiply: multiplyDQ,
                toArray: (value) => value
            }
        }
    };

    try {
        const model = visualizer._buildHumanoidDQSetpointErrorModel(identityDQ, targetDQ, poseJacobian);
        assert.ok(model);
        const d8 = [
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, -1, 0, 0, 0, 0, 0, 0],
            [0, 0, -1, 0, 0, 0, 0, 0],
            [0, 0, 0, -1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, -1, 0, 0],
            [0, 0, 0, 0, 0, 0, -1, 0],
            [0, 0, 0, 0, 0, 0, 0, -1]
        ];
        const expectedRight = multiplyMatrix(buildRightMatrix(targetDQ), multiplyMatrix(d8, poseJacobian))
            .map((row) => row.map((value) => -value));

        assert.deepEqual(model.error.map((value) => (Object.is(value, -0) ? 0 : value)), [
            1 - Math.SQRT1_2, 0, 0, -Math.SQRT1_2, 0, 0, 0, 0
        ]);
        assert.deepEqual(model.jacobian, expectedRight);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer DQ set-point error model Jacobian matches finite differences for a simple one-DoF rotation path', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const targetDQ = [
        Math.cos(Math.PI / 8), 0, 0, Math.sin(Math.PI / 8),
        0, 0, 0, 0
    ];
    const buildPose = (q) => [
        Math.cos(q / 2), 0, 0, Math.sin(q / 2),
        0, 0, 0, 0
    ];
    const buildJacobian = (q) => [
        [-0.5 * Math.sin(q / 2)],
        [0],
        [0],
        [0.5 * Math.cos(q / 2)],
        [0],
        [0],
        [0],
        [0]
    ];
    const q0 = 0.2;
    const eps = 1e-7;

    const multiplyQuaternion = (left, right) => {
        const [lw, lx, ly, lz] = left;
        const [rw, rx, ry, rz] = right;
        return [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw
        ];
    };

    const addQuaternion = (left, right) => left.map((value, index) => value + right[index]);

    const conjugateDQ = (dq) => [
        dq[0], -dq[1], -dq[2], -dq[3],
        dq[4], -dq[5], -dq[6], -dq[7]
    ];

    const multiplyDQ = (left, right) => {
        const leftPrimal = left.slice(0, 4);
        const leftDual = left.slice(4, 8);
        const rightPrimal = right.slice(0, 4);
        const rightDual = right.slice(4, 8);
        const primal = multiplyQuaternion(leftPrimal, rightPrimal);
        const dual = addQuaternion(
            multiplyQuaternion(leftPrimal, rightDual),
            multiplyQuaternion(leftDual, rightPrimal)
        );
        return primal.concat(dual);
    };

    globalThis.window = {
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                conjugate: conjugateDQ,
                multiply: multiplyDQ,
                toArray: (value) => value
            }
        }
    };

    try {
        const model = visualizer._buildHumanoidDQSetpointErrorModel(buildPose(q0), targetDQ, buildJacobian(q0));
        assert.ok(model);
        const errorPlus = visualizer._computeHumanoidDQSetpointError(buildPose(q0 + eps), targetDQ);
        const errorMinus = visualizer._computeHumanoidDQSetpointError(buildPose(q0 - eps), targetDQ);
        const fd = errorPlus.map((value, index) => (value - errorMinus[index]) / (2 * eps));
        const jacColumn = model.jacobian.map((row) => row[0]);
        for (let index = 0; index < fd.length; index += 1) {
            assert.ok(Math.abs(fd[index] - jacColumn[index]) < 1e-6, `row ${index}: expected ${fd[index]}, got ${jacColumn[index]}`);
        }
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer cooperative ruled-surface polishing relies on DQ restoration outcome without injecting a cartesian post-apply guard', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const currentState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    const nextState = {
        ...currentState,
        u: 0.36
    };
    let capturedOptions = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidModel = {};
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidSingleArmPoseJacobian = () => Array.from({ length: 8 }, (_, rowIndex) =>
        new Array(20).fill(0).map((_, colIndex) => (rowIndex === 4 && colIndex === 0 ? 1 : 0))
    );
    visualizer._getHumanoidPose1Jacobian = visualizer._getHumanoidSingleArmPoseJacobian;
    visualizer._getHumanoidPose2Jacobian = visualizer._getHumanoidSingleArmPoseJacobian;
    visualizer._getHumanoidJointLimits = () => new Array(20).fill(null).map(() => ({ lower: -Math.PI, upper: Math.PI }));
    visualizer._getHumanoidBodyVerticalState = () => ({ residual: [], jacobian: [] });
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._notifyHumanoidJogUnreachable = () => {};
    visualizer._applyHumanoidJointSolution = () => true;
    visualizer._updateHumanoidCustomPoseTargetMJRestore = async (options) => {
        capturedOptions = options;
        return true;
    };

    try {
        const ok = await visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore(
            currentState,
            nextState,
            new Array(20).fill(0),
            true,
            true,
            true,
            {}
        );
        assert.equal(ok, true);
        assert.equal(capturedOptions?.internalOptions?.postApplyAcceptanceEvaluator, undefined);
        assert.equal(capturedOptions?.internalOptions?.disallowBestEffortApply, undefined);
        assert.equal(capturedOptions?.bodyEqualityTolerance, 1e-6);
        assert.equal(capturedOptions?.extraInvariantTargetArray, null);
        assert.equal(capturedOptions?.extraInvariantEvaluator, null);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing cartesian acceptance uses a strict default max position error', () => {
    const visualizer = new RobotVisualizer(null);

    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._computeHumanoidCooperativeRuledSurfacePolishingPositionResidual = () => [0.02, 0, 0, 0, 0, 0];

    const acceptance = visualizer._evaluateHumanoidCooperativeRuledSurfacePolishingCartesianAcceptance(
        { surface: {} },
        new Array(20).fill(0)
    );

    assert.equal(acceptance?.accepted, false);
    assert.equal(acceptance?.reason, 'command_mismatch');
    assert.equal(Number(acceptance?.maxPositionError?.toFixed(3)), 0.02);
});

test('RobotVisualizer left ruled-surface polishing forwards explicit differential commands and fixed-arm invariants into single-arm restoration', async () => {
    const visualizer = new RobotVisualizer(null);
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const currentState = {
        modeKey: 'left-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetSign: 1,
        normalOffsetBounds: [0, 0.16],
        quaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    const nextState = {
        ...currentState,
        u: 0.36
    };
    let capturedConfig = null;
    const surfaceTubeInvariant = {
        ...sharedSurface.tubeAt(0.35, 0.06),
        weight: 0.42
    };

    visualizer.humanoidWaistLength = 3;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.35, 0.5, 0.06, 0];
    visualizer._getHumanoidJointLimits = () => null;
    visualizer._updateHumanoidCustomPoseTargetMJRestore = async (config) => {
        capturedConfig = config;
        return true;
    };

    const ok = await visualizer._updateHumanoidLeftRuledSurfacePolishingDifferentialMJRestore(
        currentState,
        nextState,
        new Array(17).fill(0),
        true,
        {
            solverMode: 'mj_restore_tr',
            surfaceTubeInvariant
        }
    );

    assert.equal(ok, true);
    assert.ok(capturedConfig);
    assert.equal(capturedConfig.modeTag, 'left-ruled-surface-polishing-target');
    assert.equal(capturedConfig.statusMode, 'humanoid_left_ruled_surface_polishing');
    assert.equal(capturedConfig.armStart, 3);
    assert.equal(capturedConfig.activeCount, 7);
    assert.deepEqual(
        capturedConfig.dqTargetArray.map((value) => Number(value.toFixed(6))),
        [0, 0, 0, 0, 0, 0.005, 0, 0]
    );
    assert.deepEqual(capturedConfig.extraInvariantTargetArray, [0]);
    assert.equal(typeof capturedConfig.extraInvariantEvaluator, 'function');
    assert.equal(capturedConfig.invariantWeight, 0.42);
    assert.equal(capturedConfig.internalOptions?.disallowBestEffortApply, true);

    visualizer._getHumanoidSingleArmTranslationJacobian = () => [
        [1, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0]
    ];
    const originalWindow = globalThis.window;
    globalThis.window = {};
    try {
        assert.doesNotThrow(() => {
            const invariantState = capturedConfig.extraInvariantEvaluator({
                qFull: new Array(17).fill(0),
                pose: [0, 0, 0, 0, 0, 0.005, 0, 0]
            });
            assert.equal(Array.isArray(invariantState?.current), true);
            assert.equal(Array.isArray(invariantState?.jacobian), true);
            assert.equal(invariantState.current.length, 1);
            assert.equal(invariantState.jacobian.length, 1);
            assert.equal(invariantState.jacobian[0].length, 7);
        });
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer left ruled-surface polishing forwards explicit differential commands and surface-tube protection into parameter motion', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const surfaceState = {
        modeKey: 'left-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        quaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let capturedArgs = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidLeftRuledSurfacePolishingState = () => surfaceState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._updateHumanoidLeftRuledSurfacePolishingDifferentialMJRestore = async (...args) => {
        capturedArgs = args.map((value, index) => (index <= 1 && value && typeof value === 'object')
            ? JSON.parse(JSON.stringify(value))
            : value);
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_left_ruled_surface_polishing',
            converged: false,
            applied: true,
            reason: 'best_effort',
            message: 'Restore (LS) 保留最后可行小步。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidLeftRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(Number(surfaceState.u.toFixed(3)), 0.37);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(Number(capturedArgs[0]?.u?.toFixed(3)), 0.35);
        assert.equal(Number(capturedArgs[1]?.u?.toFixed(3)), 0.37);
        assert.equal(capturedArgs[3], true);
        assert.equal(capturedArgs[4]?.controlModeOverride, 'left-ruled-surface-polishing');
        assert.equal(capturedArgs[4]?.statusModeOverride, 'humanoid_left_ruled_surface_polishing');
        assert.equal(capturedArgs[4]?.surfaceTubeInvariant?.kind, 'plucker_tube');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer left ruled-surface polishing keeps the previous state when the solve is not applied', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const surfaceState = {
        modeKey: 'left-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        quaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidLeftRuledSurfacePolishingState = () => surfaceState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._updateHumanoidLeftRuledSurfacePolishingDifferentialMJRestore = async () => {
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_left_ruled_surface_polishing',
            converged: false,
            applied: false,
            reason: 'command_mismatch',
            message: '无法到达该位置（目标未达成）。'
        };
        return false;
    };

    try {
        const ok = await visualizer.moveHumanoidLeftRuledSurfacePolishingAlongParameter('x', 'negative');
        assert.equal(ok, false);
        assert.equal(Number(surfaceState.u.toFixed(3)), 0.35);
        assert.equal(Number(surfaceState.v.toFixed(3)), 0.5);
        assert.equal(Number(surfaceState.normalOffset.toFixed(3)), 0.06);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer left ruled-surface polishing retries smaller substeps when the full step is rejected', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const surfaceState = {
        modeKey: 'left-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        v: 0.5,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        quaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    const attemptedStates = [];

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidLeftRuledSurfacePolishingState = () => surfaceState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._updateHumanoidLeftRuledSurfacePolishingDifferentialMJRestore = async (_currentState, nextState) => {
        attemptedStates.push({
            u: nextState.u,
            v: nextState.v,
            normalOffset: nextState.normalOffset
        });
        const applied = attemptedStates.length >= 2;
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_left_ruled_surface_polishing',
            converged: applied,
            applied,
            reason: applied ? 'solved' : 'command_mismatch',
            message: applied ? '恢复成功。' : '无法到达该位置（目标未达成）。'
        };
        return applied;
    };

    try {
        const ok = await visualizer.moveHumanoidLeftRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(attemptedStates.length, 2);
        assert.equal(Number(attemptedStates[0].u.toFixed(3)), 0.37);
        assert.equal(Number(attemptedStates[1].u.toFixed(3)), 0.36);
        assert.equal(Number(surfaceState.u.toFixed(3)), 0.36);
        assert.equal(Number(surfaceState.v.toFixed(3)), 0.5);
        assert.equal(Number(surfaceState.normalOffset.toFixed(3)), 0.06);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing keeps the previous strip state when the solve is not applied', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: false,
            applied: false,
            reason: 'command_mismatch',
            message: '无法到达该位置（目标未达成）。'
        };
        return false;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'negative');
        assert.equal(ok, false);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.35);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.5);
        assert.equal(Number(cooperativeState.stripSpacing.toFixed(3)), 0.24);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.06);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing keeps the previous strip state when the restore call returns truthy but marks the step as unapplied', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: false,
            applied: false,
            reason: 'command_mismatch',
            message: '无法到达该位置（目标未达成）。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, false);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.35);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.5);
        assert.equal(Number(cooperativeState.stripSpacing.toFixed(3)), 0.24);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.06);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing does not retry smaller shared-strip substeps when the full step is rejected', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    const attemptedStates = [];
    let unreachableCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._notifyHumanoidJogUnreachable = () => {
        unreachableCount += 1;
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async (_currentState, nextState) => {
        attemptedStates.push({
            u: nextState.u,
            vCenter: nextState.vCenter,
            stripSpacing: nextState.stripSpacing,
            normalOffset: nextState.normalOffset
        });
        const applied = attemptedStates.length >= 2;
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: applied,
            applied,
            reason: applied ? 'solved' : 'command_mismatch',
            message: applied ? '恢复成功。' : '无法到达该位置（目标未达成）。'
        };
        return applied;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, false);
        assert.equal(attemptedStates.length, 1);
        assert.equal(Number(attemptedStates[0].u.toFixed(3)), 0.37);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.35);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.5);
        assert.equal(Number(cooperativeState.stripSpacing.toFixed(3)), 0.24);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.06);
        assert.equal(unreachableCount, 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing scales high-speed u parameter steps directly from teach speed percent', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [0.3 * u, 0.6 * (v - 0.5), 0],
        du: () => [0.3, 0, 0],
        dv: () => [0, 0.6, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    const attemptedStates = [];

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer.teachSpeedPercent = 100;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer.humanoidCooperativeRuledSurfacePolishingState = cooperativeState;
    visualizer._refineHumanoidCooperativeRuledSurfacePolishingStateToCurrentHands = (state) => state;
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async (_currentState, nextState) => {
        attemptedStates.push({ u: nextState.u, vCenter: nextState.vCenter });
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: true,
            applied: true,
            reason: 'solved',
            message: '恢复成功。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(attemptedStates.length, 1);
        assert.equal(Number(attemptedStates[0].u.toFixed(6)), Number((0.35 + 0.02).toFixed(6)));
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing re-fits an existing shared strip state to the current hands before building the next target', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let refineCallCount = 0;
    const attemptedStates = [];

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer.humanoidCooperativeRuledSurfacePolishingState = cooperativeState;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._refineHumanoidCooperativeRuledSurfacePolishingStateToCurrentHands = (state) => {
        refineCallCount += 1;
        return {
            ...state,
            u: 0.45,
            vCenter: 0.52
        };
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async (_currentState, nextState) => {
        attemptedStates.push({
            u: nextState.u,
            vCenter: nextState.vCenter,
            stripSpacing: nextState.stripSpacing,
            normalOffset: nextState.normalOffset
        });
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: true,
            applied: true,
            reason: 'solved',
            message: '恢复成功。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(refineCallCount, 2);
        assert.equal(Number(attemptedStates[0].u.toFixed(3)), 0.47);
        assert.equal(Number(attemptedStates[0].vCenter.toFixed(3)), 0.52);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing re-fits the shared strip state to the realized hand poses after an applied step', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let refineCallCount = 0;
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.12, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.18, 0, 0, 0];
    visualizer._refineHumanoidCooperativeRuledSurfacePolishingStateToCurrentHands = (state, leftPose, rightPose) => {
        refineCallCount += 1;
        assert.ok(Array.isArray(leftPose));
        assert.ok(Array.isArray(rightPose));
        return {
            ...state,
            u: 0.357,
            vCenter: 0.503,
            normalOffset: 0.041
        };
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: true,
            applied: true,
            reason: 'converged',
            message: '恢复成功。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(refineCallCount, 1);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.357);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.503);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.041);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing re-fits shared quaternion offsets to the realized hand poses', () => {
    const visualizer = new RobotVisualizer(null);
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };

    const projectedOffsets = [
        { quaternionOffset: [0, 0, 0, 1] },
        { quaternionOffset: [0, 0, 1, 0] }
    ];
    let projectCallCount = 0;
    visualizer._projectHumanoidPoseToSurfaceState = () => projectedOffsets[projectCallCount++] ?? null;

    const refined = visualizer._refineHumanoidCooperativeRuledSurfacePolishingStateToCurrentHands(
        cooperativeState,
        [1, 0, 0, 0, 0.12, 0, 0, 0],
        [1, 0, 0, 0, 0.18, 0, 0, 0]
    );

    assert.deepEqual(refined.leftQuaternionOffset, [0, 0, 0, 1]);
    assert.deepEqual(refined.rightQuaternionOffset, [0, 0, 1, 0]);
});

test('RobotVisualizer cooperative ruled-surface polishing re-fits the shared strip state to the realized hand poses after a rejected step', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let refineCallCount = 0;
    let unreachableCount = 0;
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._notifyHumanoidJogUnreachable = () => {
        unreachableCount += 1;
    };
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.11, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.19, 0, 0, 0];
    visualizer._refineHumanoidCooperativeRuledSurfacePolishingStateToCurrentHands = (state, leftPose, rightPose) => {
        refineCallCount += 1;
        assert.ok(Array.isArray(leftPose));
        assert.ok(Array.isArray(rightPose));
        return {
            ...state,
            u: 0.341,
            vCenter: 0.497,
            normalOffset: 0.038
        };
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        visualizer.lastIKSolveStatus = {
            solver: 'mj_restore_tr',
            mode: 'humanoid_cooperative_ruled_surface_polishing',
            converged: false,
            applied: false,
            reason: 'command_mismatch',
            message: '无法到达该位置（目标未达成）。'
        };
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, false);
        assert.equal(refineCallCount, 1);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.341);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.497);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.038);
        assert.equal(unreachableCount, 1);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing clamps and stops before a v-edge move would shrink the strip spacing', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.35,
        vCenter: 0.88,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let solveCallCount = 0;
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        solveCallCount += 1;
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('y', 'positive');
        assert.equal(ok, true);
        assert.equal(solveCallCount, 0);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.35);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.88);
        assert.equal(Number(cooperativeState.stripSpacing.toFixed(3)), 0.24);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.06);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing clamps and stops before a u-edge move would hit the arc bound', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const sharedSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0.06 + u, 0, 0],
            closestPoint: [u, 0, -0.06],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0.06 + u, 0, 0],
                closestPoint: [u, 0, -0.06],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, -0.06]
            },
            radius,
            radiusSquared: radius * radius
        })
    };
    const cooperativeState = {
        modeKey: 'cooperative-ruled-surface-polishing',
        surface: sharedSurface,
        u: 0.99,
        vCenter: 0.5,
        stripSpacing: 0.24,
        normalOffset: 0.06,
        normalOffsetBounds: [0, 0.16],
        stripSpacingBounds: [0.08, 0.6],
        leftQuaternionOffset: [1, 0, 0, 0],
        rightQuaternionOffset: [1, 0, 0, 0],
        bounds: { u: [0, 1], v: [0, 1] }
    };
    let solveCallCount = 0;
    let visualizationRefreshCount = 0;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState = () => cooperativeState;
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore_tr';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {
        visualizationRefreshCount += 1;
    };
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = async () => {
        solveCallCount += 1;
        return true;
    };

    try {
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('x', 'positive');
        assert.equal(ok, true);
        assert.equal(solveCallCount, 0);
        assert.equal(Number(cooperativeState.u.toFixed(3)), 0.99);
        assert.equal(Number(cooperativeState.vCenter.toFixed(3)), 0.5);
        assert.equal(Number(cooperativeState.stripSpacing.toFixed(3)), 0.24);
        assert.equal(Number(cooperativeState.normalOffset.toFixed(3)), 0.06);
        assert.ok(visualizationRefreshCount >= 1);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer cooperative ruled-surface polishing keeps n+ moving outward on the hand-facing side when the fixed object pose flips the surface normal', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceObjectPose: {
            position: [0.11, -0.22, -0.33],
            quaternion: [1, 0, 0, 0],
            flipNormal: true
        }
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidCooperativeRuledSurfacePolishingControlMode = () => 'cooperative-ruled-surface-polishing';
    visualizer._isHumanoidMJRestoreLikeSolverEnabled = () => true;
    visualizer._getHumanoidJogSolverMode = () => 'mj_restore';
    visualizer._buildHumanoidMJRestoreInternalOptions = () => ({});
    visualizer._updateHumanoidCooperativeRuledSurfacePolishingDifferentialMJRestore = () => true;
    visualizer._updateHumanoidSurfaceSlideVisualization = () => {};
    visualizer.lastIKSolveStatus = { applied: true };
    visualizer._buildHumanoidCooperativeRuledSurfacePolishingPoseTargets = (state) => ({
        left: {
            position: [state.u, state.vCenter, state.normalOffset],
            quaternion: [1, 0, 0, 0]
        },
        right: {
            position: [state.u, state.vCenter, state.normalOffset],
            quaternion: [1, 0, 0, 0]
        }
    });
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0.32, 0.5, 0.06, 0];
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.32, 0.38, 0.06, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0.32, 0.62, 0.06, 0];
    visualizer._createHumanoidCooperativeRuledSurfaceDemoSurfaceFromPose = () => ({
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    });

    try {
        const initialState = visualizer._ensureHumanoidCooperativeRuledSurfacePolishingState();
        const before = initialState.normalOffset;
        const ok = await visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter('z', 'positive');
        assert.equal(ok, true);
        assert.ok(visualizer.humanoidCooperativeRuledSurfacePolishingState.normalOffset > before);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer prepares a recommended initial posture for cooperative ruled-surface polishing', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    let appliedQ = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    visualizer.humanoidMode = true;
    visualizer.humanoidJointNames = [
        'ankle_joint', 'knee_joint', 'hip_pitch_joint', 'hip_yaw_joint',
        'ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7',
        'rjoint1', 'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7',
        'head1', 'head2'
    ];
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        return true;
    };

    try {
        const ok = visualizer.prepareHumanoidCooperativeRuledSurfacePolishingMode();
        assert.equal(ok, true);
        assert.ok(Array.isArray(appliedQ));
        const expected = DEFAULT_HUMANOID_INITIAL_Q_PROFILE_2.slice();
        expected[5] += 0.06;
        expected[6] -= 0.08;
        expected[7] -= 0.08;
        expected[8] += 0.06;
        expected[9] -= 0.08;
        expected[12] -= 0.06;
        expected[13] += 0.08;
        expected[14] += 0.08;
        expected[15] -= 0.06;
        expected[16] += 0.08;
        assert.deepEqual(
            appliedQ.slice(0, 20).map((value) => Number(value.toFixed(5))),
            expected.slice(0, 20).map((value) => Number(value.toFixed(5)))
        );
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer prepares a cylindrical-source-specific initial posture for cooperative ruled-surface polishing', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    let appliedQ = null;

    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        humanoidCooperativeRuledSurfaceSource: 'cylindrical-patch',
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    visualizer.humanoidMode = true;
    visualizer.humanoidJointNames = [
        'ankle_joint', 'knee_joint', 'hip_pitch_joint', 'hip_yaw_joint',
        'ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7',
        'rjoint1', 'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7',
        'head1', 'head2'
    ];
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        return true;
    };

    try {
        const ok = visualizer.prepareHumanoidCooperativeRuledSurfacePolishingMode();
        assert.equal(ok, true);
        assert.ok(Array.isArray(appliedQ));
        const expected = [
            -0.23762, 0.79408, -0.56331, 0.00002,
            -0.65502, 0.44354, 1.08083, -1.61959, -0.67143, 0.95951, 0.05528,
            0.65594, -0.44414, -1.0819, 1.6197, 0.67346, -0.95977, -0.05432,
            0, 0
        ];
        assert.deepEqual(
            appliedQ.slice(0, 20).map((value) => Number(value.toFixed(5))),
            expected.map((value) => Number(value.toFixed(5)))
        );
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer single-arm surface-slide initializes from current surface attachment state', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const planeSurface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0],
        lineAt: (u) => ({
            kind: 'plucker_line',
            direction: [0, 1, 0],
            moment: [0, 0, u],
            closestPoint: [u, 0, 0],
            referenceCoordinate: 0,
            pointAtCoordinate: (coordinate) => [u, coordinate, 0]
        }),
        tubeAt: (u, radius) => ({
            kind: 'plucker_tube',
            line: {
                kind: 'plucker_line',
                direction: [0, 1, 0],
                moment: [0, 0, u],
                closestPoint: [u, 0, 0],
                referenceCoordinate: 0,
                pointAtCoordinate: (coordinate) => [u, coordinate, 0]
            },
            radius,
            radiusSquared: radius * radius
        })
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };

    visualizer.humanoidMode = true;
    visualizer._getHumanoidSurfaceSlideControlMode = () => 'surface-slide';
    visualizer._createHumanoidSurfaceSlideDemoSurfaceFromPose = () => planeSurface;
    visualizer._getHumanoidPose1 = () => [0, 0, 0, 1, 0.2, 0.4, 0.06, 0];

    try {
        const state = visualizer._ensureHumanoidSurfaceSlideState('surface-slide');
        assert.ok(state);
        assert.ok(Math.abs(state.u - 0.2) < 0.08);
        assert.ok(Math.abs(state.v - 0.4) < 0.08);
        assert.ok(Math.abs(state.normalOffset - 0.06) < 0.02);
        assert.deepEqual(state.quaternionOffset.map((value) => Number(value.toFixed(6))), [0, 0, 0, 1]);

        const target = visualizer._buildHumanoidSurfaceSlidePoseTarget(state);
        assert.deepEqual(target.position.map((value) => Number(value.toFixed(6))), [0.2, 0.4, 0.06]);
        assert.deepEqual(target.quaternion.map((value) => Number(value.toFixed(6))), [0, 0, 0, 1]);
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer bimanual surface-slide MJ helper rejects the removed dual-independent surface mode adapter', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const currentQ = new Array(20).fill(0);
    let capturedOptions = null;

    globalThis.window = {
        humanoidQ: currentQ.slice(),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    visualizer.humanoidMode = true;
    visualizer.humanoidWaistLength = 2;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    visualizer.humanoidJointNames = Array.from({ length: 16 }, (_, index) => `joint${index}`);
    visualizer.humanoidURDF = {
        joints: Object.fromEntries(
            Array.from({ length: 16 }, (_, index) => [`joint${index}`, { limit: { lower: -2, upper: 2 } }])
        )
    };
    visualizer.humanoidModel = {
        fkmEffector: () => [1, 0, 0, 0, 0, 0, 0, 0],
        poseJacobianEffector: () => Array.from({ length: 8 }, () => new Array(16).fill(0))
    };
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0.1, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, -0.1, 0, 0, 0];
    visualizer._getHumanoidPose1Jacobian = () => Array.from({ length: 8 }, () => new Array(16).fill(0));
    visualizer._getHumanoidPose2Jacobian = () => Array.from({ length: 8 }, () => new Array(16).fill(0));
    visualizer._extractPoseTranslationAndRotation = (pose) => ({
        position: [pose[4] || 0, pose[5] || 0, pose[6] || 0],
        quaternion: pose.slice(0, 4)
    });
    visualizer._updateHumanoidCustomPoseTargetMJRestore = (options) => {
        capturedOptions = options;
        return true;
    };

    try {
        const result = await visualizer._updateHumanoidJointsFromDualIndependentSurfaceTargetsMJRestore(
            [1, 0, 0, 0, 0.2, 0, 0, 0],
            [1, 0, 0, 0, -0.2, 0, 0, 0],
            currentQ.slice(),
            true,
            true,
            true,
            {
                leftTubeInvariant: {
                    kind: 'plucker_tube',
                    line: {
                        kind: 'plucker_line',
                        direction: [0, 1, 0],
                        moment: [0.06, 0, 0],
                        closestPoint: [0, 0, -0.06],
                        referenceCoordinate: 0,
                        pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
                    },
                    radius: 0.06,
                    radiusSquared: 0.0036
                },
                rightTubeInvariant: {
                    kind: 'plucker_tube',
                    line: {
                        kind: 'plucker_line',
                        direction: [0, 1, 0],
                        moment: [0.08, 0, 0],
                        closestPoint: [0.2, 0, -0.06],
                        referenceCoordinate: 0,
                        pointAtCoordinate: (coordinate) => [0.2, coordinate, -0.06]
                    },
                    radius: 0.05,
                    radiusSquared: 0.0025
                },
                solverMode: 'mj_restore_tr'
            }
        );
        assert.equal(result, false);
        assert.equal(capturedOptions, null);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'mode_adapter_failed');
    } finally {
        globalThis.window = originalWindow;
    }
});

test('RobotVisualizer custom MJ restore keeps a best-effort pose candidate when it makes strong command progress', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let publishedDiagnostics = null;
    let unreachableMessage = null;
    let appliedQ = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 1,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._notifyHumanoidJogUnreachable = (message) => {
        unreachableMessage = message;
    };
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        return true;
    };

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.189, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false
        });

        assert.equal(result, true);
        assert.ok(Array.isArray(appliedQ));
        assert.ok(appliedQ[0] > 0.1, `expected best-effort q to be applied, got ${appliedQ[0]}`);
        assert.equal(unreachableMessage, null);
        assert.notEqual(visualizer.lastIKSolveStatus?.reason, 'hard_tol_not_reached');
        assert.ok((publishedDiagnostics?.stepNorm ?? 0) > 0.1);
        assert.match(publishedDiagnostics?.message ?? '', /保留|已执行|完成/);
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer custom MJ restore reports TR-specific best-effort messaging when mj_restore_tr keeps the last feasible step', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let publishedDiagnostics = null;
    let appliedQ = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 1,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._notifyHumanoidJogUnreachable = () => {};
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        return true;
    };

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.189, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore_tr',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false
        });

        assert.equal(result, true);
        assert.ok(Array.isArray(appliedQ));
        assert.ok(appliedQ[0] > 0.1, `expected best-effort q to be applied, got ${appliedQ[0]}`);
        assert.equal(visualizer.lastIKSolveStatus?.solver, 'mj_restore_tr');
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'best_effort');
        assert.match(visualizer.lastIKSolveStatus?.message ?? '', /Recovery Optimization \(TR\)/);
        assert.match(publishedDiagnostics?.message ?? '', /Recovery Optimization \(TR\)/);
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer custom MJ restore can disable best-effort application and report the target as unreachable instead', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let publishedDiagnostics = null;
    let unreachableMessage = null;
    let appliedQ = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 1,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._notifyHumanoidJogUnreachable = (message) => {
        unreachableMessage = message;
    };
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        return true;
    };

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.189, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false,
            internalOptions: {
                controlModeOverride: 'left-ruled-surface-polishing',
                disallowBestEffortApply: true
            }
        });

        assert.equal(result, true);
        assert.equal(appliedQ, null);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'command_mismatch');
        assert.equal(visualizer.lastIKSolveStatus?.applied, false);
        assert.match(publishedDiagnostics?.message ?? '', /目标未达成/);
        assert.equal(publishedDiagnostics?.stepNorm ?? 0, 0);
        assert.equal(unreachableMessage, '无法到达该位置');
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer custom MJ restore caps oversized best-effort cooperative polishing steps before applying them', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let publishedDiagnostics = null;
    let appliedQ = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 1,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._notifyHumanoidJogUnreachable = () => {};
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        return true;
    };

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.189, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false,
            internalOptions: {
                controlModeOverride: 'cooperative-ruled-surface-polishing',
                bestEffortAppliedStepNormCap: 0.05
            }
        });

        assert.equal(result, true);
        assert.ok(Array.isArray(appliedQ));
        assert.ok((publishedDiagnostics?.stepNorm ?? Infinity) <= 0.05 + 1e-9, `expected capped best-effort step, got ${publishedDiagnostics?.stepNorm}`);
        assert.ok((appliedQ?.[0] ?? Infinity) <= 0.05 + 1e-9, `expected applied q to be capped, got ${appliedQ?.[0]}`);
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer custom MJ restore rejects oversized best-effort cooperative polishing command residuals instead of applying them', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let publishedDiagnostics = null;
    let appliedQ = null;
    let unreachableMessage = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 120,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._notifyHumanoidJogUnreachable = (message) => {
        unreachableMessage = message;
    };
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        return true;
    };

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.641, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false,
            internalOptions: {
                controlModeOverride: 'cooperative-ruled-surface-polishing',
                bestEffortAppliedStepNormCap: 0.12,
                bestEffortAppliedCommandNormCap: 0.08,
                bestEffortAppliedInvariantNormCap: 0.01
            }
        });

        assert.equal(result, true);
        assert.equal(appliedQ, null);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'command_mismatch');
        assert.match(publishedDiagnostics?.message ?? '', /目标未达成/);
        assert.equal(publishedDiagnostics?.stepNorm ?? 0, 0);
        assert.equal(unreachableMessage, '无法到达该位置');
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer custom MJ restore can reject an otherwise applicable result through a post-apply acceptance guard', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let publishedDiagnostics = null;
    let appliedQ = null;
    let unreachableMessage = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 1,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._notifyHumanoidJogUnreachable = (message) => {
        unreachableMessage = message;
    };
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = (qSolved) => {
        appliedQ = qSolved.slice();
        for (let i = 0; i < window.humanoidQ.length && i < qSolved.length; i++) {
            window.humanoidQ[i] = qSolved[i];
        }
        return true;
    };

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.189, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore_tr',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false,
            internalOptions: {
                controlModeOverride: 'cooperative-ruled-surface-polishing',
                postApplyAcceptanceEvaluator: () => ({
                    accepted: false,
                    reason: 'command_mismatch',
                    message: '无法到达该位置（目标未达成）。',
                    commandNorm: 0.2
                })
            }
        });

        assert.equal(result, true);
        assert.equal(appliedQ, null);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'command_mismatch');
        assert.equal(visualizer.lastIKSolveStatus?.applied, false);
        assert.equal(publishedDiagnostics?.commandError, 0.2);
        assert.equal(publishedDiagnostics?.stepNorm ?? 0, 0);
        assert.equal(unreachableMessage, '无法到达该位置');
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer custom MJ restore can suppress unreachable notifications during internal retry chains', async () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let unreachableMessage = null;

    globalThis.window = {
        humanoidQ: [0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.vfiConfig = {
        mjRestoreMaxIter: 1,
        mjRestoreMaxStep: 0.16,
        hardCommandTolPose: 1e-6,
        practicalCommandTolPose: 1e-4,
        hardInvariantTol: 5e-5,
        practicalInvariantTol: 2.5e-4,
        qpMismatchTol: 1e-3,
        qpMismatchRequiredReduction: 0.8
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._notifyHumanoidJogUnreachable = (message) => {
        unreachableMessage = message;
    };
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._applyHumanoidJointSolution = () => true;

    const poseGetter = (q) => [1, 0, 0, 0, q[0], 0, 0, 0];
    const jacobianGetter = () => [[0], [0], [0], [0], [1], [0], [0], [0]];

    try {
        const result = await visualizer._updateHumanoidCustomPoseTargetMJRestore({
            dqTargetArray: [1, 0, 0, 0, 0.189, 0, 0, 0],
            currentQ: [0],
            solverMode: 'mj_restore_tr',
            statusMode: 'test',
            modeTag: 'test-target',
            includeWaist: false,
            armStart: 0,
            activeCount: 1,
            hardLimits: [{ lower: -1, upper: 1 }],
            poseGetter,
            jacobianGetter,
            invariantWeight: 0,
            keepBodyZVertical: false,
            internalOptions: {
                controlModeOverride: 'cooperative-ruled-surface-polishing',
                suppressUnreachableNotify: true,
                postApplyAcceptanceEvaluator: () => ({
                    accepted: false,
                    reason: 'command_mismatch',
                    message: '无法到达该位置（目标未达成）。',
                    commandNorm: 0.2
                })
            }
        });

        assert.equal(result, true);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'command_mismatch');
        assert.equal(unreachableMessage, null);
    } finally {
        globalThis.window = originalWindow;
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer mj_restore_pre_w falls back to q0 for stage 2 when pre-w is skipped, rejected, or failed', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0.02, 0, 0, 0];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const fallbackStates = [
        'skipped(no_progress)',
        'rejected(hard_constraints)',
        'failed(thrown)'
    ];
    const originalDocument = globalThis.document;
    const textEl = { textContent: '' };
    const tsEl = { textContent: '' };
    const panelEl = { classList: { toggle: () => {} } };

    globalThis.document = {
        getElementById: (id) => {
            if (id === 'humanoid-vfi-diagnostics-text') return textEl;
            if (id === 'humanoid-vfi-diagnostics-timestamp') return tsEl;
            if (id === 'humanoid-vfi-diagnostics-panel') return panelEl;
            return null;
        }
    };

    try {
        visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
        visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
        visualizer._getHumanoidRelativePose = () => relCurrent.slice();
        visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';

        for (const state of fallbackStates) {
            let restoreArgs = null;
            visualizer._runHumanoidMJRestorePreWStage = async () => ({
                state,
                acceptedCandidate: false,
                qApplied: [9, 9, 9]
            });
            visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
                restoreArgs = args;
                return true;
            };

            const result = await visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
                absTarget.slice(),
                q0.slice(),
                true,
                false,
                false,
                null,
                null
            );

            assert.equal(result, true, state);
            await new Promise((resolve) => setTimeout(resolve, 0));
            assert.deepEqual(restoreArgs[1], q0, state);
        }
    } finally {
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer candidate-only decoupled pre-w disables arm-only fallback and stays candidate-only', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0, 0, 0, 0];
    const relTarget = [0, 1, 0, 0, 0, 0, 0, 0];
    let called = null;
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        called = args;
        return Promise.resolve({
            state: 'skipped(no_progress)',
            acceptedCandidate: false,
            qApplied: q0.slice()
        });
    };

    const result = await visualizer._runHumanoidMJRestorePreWStage({
        controlMode: 'decoupled',
        q0: q0.slice(),
        dqAbsoluteProtectedTargetArray: absTarget.slice(),
        fixedRelativeProtectedTargetArray: relTarget.slice(),
        enableJointLimitAvoidance: true,
        includeWaist: true,
        keepBodyZVertical: false,
        jogMetric: 'w_abs_hier_vfi',
        metricDirection: 'increase'
    });

    assert.equal(result.acceptedCandidate, false);
    assert.ok(Array.isArray(called));
    assert.equal(called[3], true);
    assert.equal(called[6]?.candidateOnly, true);
    assert.equal(called[6]?.allowArmOnlyFallback, false);
    assert.equal(called[6]?.preferWorker, true);
    assert.deepEqual(called[6]?.iterativeMetricProjectionPlan, {
        metricIterations: 3,
        restoreIterations: 1
    });
});

test('RobotVisualizer candidate-only pre-w stage forwards mj_restore_pre_w solver identity into the manifold pass', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0, 0, 0, 0];
    const relTarget = [0, 1, 0, 0, 0, 0, 0, 0];
    let called = null;
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        called = args;
        return Promise.resolve({
            state: 'skipped(no_progress)',
            acceptedCandidate: false,
            qApplied: q0.slice()
        });
    };

    await visualizer._runHumanoidMJRestorePreWStage({
        controlMode: 'absolute',
        q0: q0.slice(),
        dqAbsoluteProtectedTargetArray: absTarget.slice(),
        fixedRelativeProtectedTargetArray: relTarget.slice(),
        enableJointLimitAvoidance: true,
        includeWaist: false,
        keepBodyZVertical: false,
        jogMetric: 'w_abs_hier',
        metricDirection: 'increase'
    });

    assert.ok(Array.isArray(called));
    assert.equal(called[6]?.solverMode, 'mj_restore_pre_w');
});

test('RobotVisualizer plain mj_restore still ignores the selected w metric for jog commands', async () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0.02, 0, 0, 0];
    const capturedCalls = [];

    visualizer.setHumanoidJogSolverMode('mj_restore');
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
        capturedCalls.push(args);
        return true;
    };

    const result = await visualizer.updateHumanoidJointsFromAbsolutePoseTargetRelativeFirst(
        absTarget.slice(),
        q0.slice(),
        true,
        false,
        false,
        null,
        null
    );

    assert.equal(result, true);
    assert.equal(capturedCalls.length, 1);
    assert.equal(capturedCalls[0][7], undefined);
});

test('RobotVisualizer plain mj_restore absolute jogs force jogMetricOverride none on the local path', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const absTarget = [1, 0, 0, 0, 0.01, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];

    let manifoldArgs = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => false;
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
    visualizer._signedPoseError = () => [1];
    visualizer._computePoseErrorNorm = () => 0.1;
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        manifoldArgs = args;
        return Promise.resolve(true);
    };

    const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
        absTarget.slice(),
        currentQ.slice(),
        true,
        false,
        false,
        relCurrent.slice(),
        null,
        null
    );

    assert.equal(result, true);
    assert.ok(Array.isArray(manifoldArgs));
    assert.equal(manifoldArgs[6]?.solverMode, 'mj_restore');
    assert.equal(manifoldArgs[6]?.jogMetricOverride, 'none');
    assert.equal(manifoldArgs[6]?.metricDirection, undefined);
});

test('RobotVisualizer absolute mj_restore_tr local path does not throw when building the restore request', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0.01, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;

    globalThis.window = {
        humanoidQ: currentQ.slice(),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {},
        DQModule: { DQWrapper: {} }
    };
    globalThis.document = {
        getElementById: () => null
    };

    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 0;
    visualizer.humanoidLeftArmLength = 3;
    visualizer.humanoidRightArmLength = 0;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => false;
    visualizer._getHumanoidAbsolutePose = (q) => [1, 0, 0, 0, q[0] ?? 0, 0, 0, 0];
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]];
    visualizer._getHumanoidRelativePoseJacobian = () => Array.from({ length: 8 }, () => [0, 0, 0]);
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = () => ({
        w_abs_hier_vfi: 1,
        w_abs_arm_vfi: 1,
        w_abs_vfi: 1,
        w_abs_hier: 1,
        w_abs_arm: 1,
        w_abs: 1,
        w: 1,
        w_vfi: 1
    });
    visualizer._computeProtectedManipulabilityGradient = () => [0, 0, 0];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._notifyHumanoidJogUnreachable = () => {};
    visualizer._applyHumanoidJointSolution = () => true;
    visualizer._getHumanoidJointLimits = () => [{ lower: -1, upper: 1 }, { lower: -1, upper: 1 }, { lower: -1, upper: 1 }];
    visualizer._sliceHumanoidActiveJoints = (_q, start, count) => currentQ.slice(start, start + count);
    visualizer._writeHumanoidActiveJoints = (_qInitial, qActive) => qActive.slice();
    visualizer._buildHumanoidManifoldWaistXCorridor = () => ({ enabled: false, violation: 0 });
    visualizer._signedPoseError = () => [1];
    visualizer._computePoseErrorNorm = () => 0.1;

    try {
        const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
            absTarget.slice(),
            currentQ.slice(),
            true,
            false,
            false,
            relCurrent.slice(),
            { solverMode: 'mj_restore_tr', preferWorker: false, jogMetricOverride: 'none' }
        );

        assert.equal(typeof result, 'boolean');
        assert.equal(visualizer.lastIKSolveStatus?.solver, 'mj_restore_tr');
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer mj_restore_pre_w absolute stage 2 preserves solver identity and pre-w metadata on the real local manifold path', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const absTarget = [1, 0, 0, 0, 0.01, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];

    let manifoldArgs = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => false;
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
    visualizer._signedPoseError = () => [1];
    visualizer._computePoseErrorNorm = () => 0.1;
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        manifoldArgs = args;
        return Promise.resolve(true);
    };

    const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
        absTarget.slice(),
        currentQ.slice(),
        true,
        false,
        false,
        relCurrent.slice(),
        null,
        {
            solverMode: 'mj_restore_pre_w',
            preWState: 'applied',
            preWMetric: 'w_abs_hier_vfi',
            preWStart: 'q_pre'
        }
    );

    assert.equal(result, true);
    assert.ok(Array.isArray(manifoldArgs));
    assert.equal(manifoldArgs[6]?.solverMode, 'mj_restore_pre_w');
    assert.equal(manifoldArgs[6]?.preWState, 'applied');
    assert.equal(manifoldArgs[6]?.preWMetric, 'w_abs_hier_vfi');
    assert.equal(manifoldArgs[6]?.preWStart, 'q_pre');
});

test('RobotVisualizer manifold executor keeps mj_restore_pre_w identity on early failure paths', async () => {
    const visualizer = new RobotVisualizer(null);

    const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold(
        [1, 0, 0, 0, 0, 0, 0, 0],
        [],
        true,
        false,
        false,
        null,
        {
            solverMode: 'mj_restore_pre_w'
        }
    );

    assert.equal(result, false);
    assert.equal(visualizer.lastIKSolveStatus?.solver, 'mj_restore_pre_w');
    assert.equal(visualizer.lastIKSolveStatus?.reason, 'invalid_state');
});

test('RobotVisualizer plain mj_restore distance jogs do not consult the selected w metric', () => {
    const visualizer = new RobotVisualizer(null);
    const q0 = [0.1, -0.2, 0.3];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0, 0, 0]
            },
            Kinematics: {
                distanceJacobian: () => [[0, 0, 0]]
            }
        }
    };

    let selectedMetricReads = 0;
    visualizer.humanoidModel = {};
    visualizer.setHumanoidJogSolverMode('mj_restore');
    visualizer.humanoidWaistLength = 1;
    visualizer.humanoidLeftArmLength = 1;
    visualizer.humanoidRightArmLength = 1;
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => {
        selectedMetricReads += 1;
        return 'w_abs_hier_vfi';
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidPose1 = () => relCurrent.slice();
    visualizer._getHumanoidPose2 = () => relCurrent.slice();
    visualizer._getHumanoidJointLimits = () => null;
    visualizer._sliceHumanoidActiveJoints = () => [0.1, -0.2, 0.3];
    visualizer._writeHumanoidActiveJoints = (_qInitial, qActive) => qActive.slice();
    visualizer._buildHumanoidManifoldWaistXCorridor = () => ({ enabled: false, violation: 0 });
    visualizer._buildManipulabilityVFIConstraints = () => null;
    visualizer._computeModeCCTSM = () => ({ w: 1, w_vfi: 1 });
    visualizer._computeProtectedManipulabilityGradient = () => [0, 0, 0];
    visualizer._extractHumanoidWVFIConstraintDiagnostics = () => ({});
    visualizer._extractHumanoidWaistXCorridorDiagnostics = () => ({});
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._notifyHumanoidJogUnreachable = () => {};
    visualizer._applyHumanoidJointSolution = () => true;
    visualizer._getHumanoidAbsolutePoseJacobian = () => Array.from({ length: 8 }, () => [0, 0, 0]);
    visualizer._getHumanoidRelativePoseJacobian = () => Array.from({ length: 8 }, () => [0, 0, 0]);
    visualizer._getHumanoidSingleArmPoseJacobian = () => Array.from({ length: 8 }, () => [0, 0, 0]);

    try {
        visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(0.01, q0.slice(), true, false);
        assert.equal(selectedMetricReads, 0);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer _runHumanoidMJRestorePreWStage skips unsupported control modes safely', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const dqAbsoluteTargetArray = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrentArray = [0, 1, 0, 0, 0, 0, 0, 0];
    const unsupportedControlModes = ['unknown'];

    let manifoldCalls = 0;
    visualizer._getHumanoidAbsolutePose = () => dqAbsoluteTargetArray.slice();
    visualizer._getHumanoidRelativePose = () => relCurrentArray.slice();
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = () => {
        manifoldCalls += 1;
        return true;
    };

    for (const controlMode of unsupportedControlModes) {
        manifoldCalls = 0;
        const result = await visualizer._runHumanoidMJRestorePreWStage({
            controlMode,
            q0: currentQ.slice(),
            dqAbsoluteProtectedTargetArray: dqAbsoluteTargetArray.slice(),
            fixedRelativeProtectedTargetArray: relCurrentArray.slice(),
            enableJointLimitAvoidance: true,
            includeWaist: controlMode.includes('fullbody'),
            keepBodyZVertical: false,
            jogMetric: 'w_abs_hier',
            metricDirection: 'increase'
        });

        assert.deepEqual(result, {
            state: 'skipped(unsupported_control_mode)',
            qPreEligible: false
        }, controlMode);
        assert.equal(manifoldCalls, 0, controlMode);
    }
});

test('RobotVisualizer mj_restore relative variants route through the dedicated MJ relative helper', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const relTarget = [0, 1, 0, 0, 0, 0, 0, 0];
    const variants = [
        { includeWaist: false, label: 'arm-only' },
        { includeWaist: true, label: 'fullbody' }
    ];

    let relativeMJCalls = 0;
    visualizer.setHumanoidJogSolverMode('mj_restore');
    visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore = (...args) => {
        relativeMJCalls += 1;
        assert.equal(args[3], variants[relativeMJCalls - 1].includeWaist);
        assert.equal(args[4], null);
        return true;
    };
    visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstVFI = () => {
        throw new Error('relative new qpJ fallback should not be used');
    };

    for (const variant of variants) {
        const result = await visualizer.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(
            relTarget.slice(),
            currentQ.slice(),
            true,
            variant.includeWaist,
            null
        );
        assert.equal(result, true, variant.label);
    }

    assert.equal(relativeMJCalls, variants.length);
});

test('RobotVisualizer mj_restore_pre_w relative variants run pre-w first, then use the MJ relative helper', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const qPre = [0.4, -0.1, 0.2];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const relTarget = [0, 1, 0, 0, 0, 0, 0, 0];
    const variants = [
        { includeWaist: false, label: 'arm-only' },
        { includeWaist: true, label: 'fullbody' }
    ];

    let preWCalls = 0;
    let relativeMJCalls = 0;
    const originalDocument = globalThis.document;
    const textEl = { textContent: '' };
    const tsEl = { textContent: '' };
    const panelEl = { classList: { toggle: () => {} } };
    globalThis.document = {
        getElementById: (id) => {
            if (id === 'humanoid-vfi-diagnostics-text') return textEl;
            if (id === 'humanoid-vfi-diagnostics-timestamp') return tsEl;
            if (id === 'humanoid-vfi-diagnostics-panel') return panelEl;
            return null;
        }
    };
    try {
        visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
        visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
        visualizer._getHumanoidRelativePose = () => relCurrent.slice();
        visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
        visualizer._runHumanoidMJRestorePreWStage = async (options) => {
            preWCalls += 1;
            assert.equal(options.controlMode, variants[preWCalls - 1].includeWaist ? 'relative-fullbody' : 'relative');
            assert.equal(options.jogMetric, 'w_abs_hier_vfi');
            assert.equal(options.metricDirection, 'increase');
            return { state: 'active', acceptedCandidate: true, qApplied: qPre.slice() };
        };
        visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore = (...args) => {
            relativeMJCalls += 1;
            assert.equal(args[3], variants[relativeMJCalls - 1].includeWaist);
            assert.equal(args[4]?.solverMode, 'mj_restore_pre_w');
            assert.equal(args[4]?.preWState, 'applied');
            assert.equal(args[4]?.preWMetric, 'w_abs_hier_vfi');
            assert.equal(args[4]?.preWStart, 'q_pre');
            assert.deepEqual(args[1], qPre);
            visualizer.lastHumanoidJogDiagnostics = {
                solverMode: 'mj_restore_pre_w',
                renderedLines: ['pre_w: applied']
            };
            return true;
        };
        visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstVFI = () => {
            throw new Error('relative new qpJ fallback should not be used');
        };

        for (const variant of variants) {
            const result = await visualizer.updateHumanoidJointsFromRelativePoseTargetAbsoluteFirst(
                relTarget.slice(),
                currentQ.slice(),
                true,
                variant.includeWaist,
                null
            );
            assert.equal(result, true, variant.label);
            assert.equal(visualizer.lastHumanoidJogDiagnostics.solverMode, 'mj_restore_pre_w', variant.label);
            assert.ok(
                visualizer.lastHumanoidJogDiagnostics.renderedLines.some((line) => line.includes('pre_w: applied')),
                variant.label
            );
        }

        assert.equal(preWCalls, variants.length);
        assert.equal(relativeMJCalls, variants.length);
    } finally {
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer mj_restore_pre_w distance variants run pre-w first, then use the MJ distance helper', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const qPre = [0.4, -0.1, 0.2];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const variants = [
        { includeWaist: false, label: 'arm-only' },
        { includeWaist: true, label: 'fullbody' }
    ];
    const originalDocument = globalThis.document;
    const textEl = { textContent: '' };
    const tsEl = { textContent: '' };
    const panelEl = { classList: { toggle: () => {} } };

    let preWCalls = 0;
    let distanceRestoreCalls = 0;
    globalThis.document = {
        getElementById: (id) => {
            if (id === 'humanoid-vfi-diagnostics-text') return textEl;
            if (id === 'humanoid-vfi-diagnostics-timestamp') return tsEl;
            if (id === 'humanoid-vfi-diagnostics-panel') return panelEl;
            return null;
        }
    };

    try {
        visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
        visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
        visualizer._getHumanoidRelativePose = () => relCurrent.slice();
        visualizer._runHumanoidMJRestorePreWStage = async (options) => {
            preWCalls += 1;
            assert.equal(options.controlMode, variants[preWCalls - 1].includeWaist ? 'distance-fullbody' : 'distance');
            assert.equal(options.jogMetric, 'w_abs_hier_vfi');
            assert.equal(options.metricDirection, 'increase');
            return { state: 'active', acceptedCandidate: true, qApplied: qPre.slice() };
        };
        visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
        visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore = (...args) => {
            distanceRestoreCalls += 1;
            assert.deepEqual(args[1], qPre);
            assert.equal(args[3], variants[distanceRestoreCalls - 1].includeWaist);
            assert.equal(args[4]?.solverMode, 'mj_restore_pre_w');
            assert.equal(args[4]?.preWState, 'applied');
            assert.equal(args[4]?.preWMetric, 'w_abs_hier_vfi');
            assert.equal(args[4]?.preWStart, 'q_pre');
            return true;
        };

        for (const variant of variants) {
            const result = await visualizer.updateHumanoidJointsFromRelativeDistanceDelta(
                0.01,
                currentQ.slice(),
                true,
                variant.includeWaist
            );
            assert.equal(result, true, variant.label);
        }

        assert.equal(preWCalls, variants.length);
        assert.equal(distanceRestoreCalls, variants.length);
    } finally {
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer relative-target mj_restore applies task-aware metric refinement when jogMetricOverride is requested', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 2) % 7)] = 0.08 * (rowIndex + 1);
        return row;
    });
    const metricCalls = [];
    let firstMetricAttempt = null;
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: { DQWrapper: {} },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.equal(args[3], 'relative');
        return {
            taskMetricKind: 'relative_pose',
            w_task_hier: 0.2,
            w_task_hier_vfi: 0.17,
            w_vfi: 0.17,
            w: 0.2
        };
    };
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => {
        metricCalls.push(options.metric);
        if (!firstMetricAttempt) firstMetricAttempt = options;
        const currentProblem = {
            ...options.baseProblem,
            q: options.baseQActive.slice(),
            metadata: {
                ...(options.baseProblem?.metadata || {}),
                cctsm: {
                    taskMetricKind: 'relative_pose',
                    w_task_arm: 0.21,
                    w_task_hier: options.metric === 'w_abs_hier' ? 0.22 : 0.2,
                    w_task_arm_vfi: 0.18,
                    w_task_hier_vfi: 0.17,
                    w_vfi: 0.17,
                    w: 0.2
                }
            }
        };
        return {
            state: 'active',
            before: 0.2,
            after: 0.22,
            applied: true,
            finalProblem: currentProblem,
            refineResult: {
                converged: true,
                reason: 'projected_candidate',
                qApplied: options.baseQActive.slice(),
                iterations: 0,
                acceptedIterations: 1,
                currentProblem
            },
            nullspaceResult: { reason: 'accepted' }
        };
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore(
            relCurrent.slice(),
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                jogMetricOverride: 'w_abs_hier',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.ok(firstMetricAttempt);
        assert.deepEqual(metricCalls, ['w_abs_arm', 'w_abs_hier']);
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.metricMode, 'relative');
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.cctsm?.taskMetricKind, 'relative_pose');
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.Jcmd?.length, 6);
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.protectedRows?.length, 6);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer relative-target mj_restore hierarchical VFI refinement uses task-arm primary before task-hier secondary', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 2) % 7)] = 0.08 * (rowIndex + 1);
        return row;
    });
    const metricCalls = [];
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: { DQWrapper: {} },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.equal(args[3], 'relative');
        return {
            taskMetricKind: 'relative_pose',
            w_task_arm_vfi: 0.14,
            w_task_hier_vfi: 0.17,
            w_task_arm: 0.16,
            w_task_hier: 0.2,
            w_vfi: 0.17,
            w: 0.2
        };
    };
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => {
        metricCalls.push(options.metric);
        const currentProblem = {
            ...options.baseProblem,
            q: options.baseQActive.slice(),
            metadata: {
                ...(options.baseProblem?.metadata || {}),
                cctsm: {
                    taskMetricKind: 'relative_pose',
                    w_task_arm_vfi: options.metric === 'w_abs_arm_vfi' ? 0.16 : 0.16,
                    w_task_hier_vfi: options.metric === 'w_abs_hier_vfi' ? 0.19 : 0.17,
                    w_task_arm: 0.16,
                    w_task_hier: 0.2,
                    w_vfi: 0.18,
                    w: 0.2
                }
            },
            equalities: {
                blocks: {
                    relative: { residual: [1e-6] },
                    absolute: { residual: [1e-6] }
                }
            },
            inequalities: {
                wvfiFloor: { violation: 0 },
                waistXCorridor: { violation: 0 }
            }
        };
        return {
            state: 'active',
            before: 0.17,
            after: options.metric === 'w_abs_arm_vfi' ? 0.18 : 0.19,
            applied: true,
            finalProblem: currentProblem,
            refineResult: {
                converged: true,
                reason: 'projected_candidate',
                qApplied: options.baseQActive.slice(),
                iterations: 0,
                acceptedIterations: 1,
                currentProblem
            },
            nullspaceResult: { reason: 'accepted' }
        };
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore(
            relCurrent.slice(),
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.deepEqual(metricCalls, ['w_abs_arm_vfi', 'w_abs_hier_vfi']);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer relative-target metric-only hierarchical VFI refinement can use the worker-side secondary metric fast path', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {
        solveAbsoluteTargetMJRestore: () => ({})
    };
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    visualizer.humanoidMJWorkerModelSpec = { waistChain: [] };
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 2) % 7)] = 0.08 * (rowIndex + 1);
        return row;
    });
    let observedSecondaryMetricRequest = null;
    let localMetricRefineCalls = 0;
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: { DQWrapper: {} },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
    visualizer._getHumanoidJogMJWorkerClient = () => ({
        solve: async ({ secondaryMetricRequest }) => {
            observedSecondaryMetricRequest = secondaryMetricRequest;
            return {
                converged: true,
                reason: 'converged',
                iterations: 0,
                acceptedIterations: 0,
                lastAcceptedScale: 1,
                commandResidualNorm: 0,
                relativeResidualNorm: 0,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                qApplied: q0.slice(4, 18),
                secondaryMetric: 'w_abs_hier_vfi',
                secondaryState: 'active',
                secondaryBefore: 1.6,
                secondaryAfter: 1.8,
                secondaryApplied: true
            };
        }
    });
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.equal(args[3], 'relative');
        return {
            taskMetricKind: 'relative_pose',
            w_task_arm_vfi: 1.7,
            w_task_hier_vfi: 1.8,
            w_task_arm: 1.75,
            w_task_hier: 1.9,
            w_vfi: 1.8,
            w: 1.9
        };
    };
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async () => {
        localMetricRefineCalls += 1;
        throw new Error('local refine should not run when worker metric attempt is accepted');
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore(
            relCurrent.slice(),
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                preferWorker: true,
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.deepEqual(observedSecondaryMetricRequest, {
            metric: 'w_abs_hier_vfi',
            metricMode: 'relative',
            stepSize: 0.01,
            restoreMaxIterations: 24,
            waistActiveCount: 0,
            hierarchicalVfiBlend: 1
        });
        assert.equal(localMetricRefineCalls, 0);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer distance-target mj_restore applies task-aware metric refinement when jogMetricOverride is requested', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0.2, 0, 0.3];
    const pose1 = [1, 0, 0, 0, 0, 0.1, 0, 0];
    const pose2 = [1, 0, 0, 0, 0, -0.1, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 1) % 7)] = 0.07 * (rowIndex + 1);
        return row;
    });
    const Jpose1 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.05 * (rowIndex + 1);
        return row;
    });
    const Jpose2 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[11 + (rowIndex % 7)] = 0.04 * (rowIndex + 1);
        return row;
    });
    const metricCalls = [];
    let firstMetricAttempt = null;
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0.2, 0, 0.3]
            },
            Kinematics: {
                distanceJacobian: () => [new Array(q0.length).fill(0).map((_, index) => (index === 4 ? 0.12 : 0))]
            }
        },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidPose1 = () => pose1.slice();
    visualizer._getHumanoidPose2 = () => pose2.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._getHumanoidSingleArmPoseJacobian = (q, armIndex) => (armIndex === 1 ? Jpose1 : Jpose2).map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.ok(args[3] === 'relative' || args[3] === 'distance');
        return {
            taskMetricKind: 'relative_pose',
            w_task_hier: 0.12,
            w_task_hier_vfi: 0.09,
            w_vfi: 0.09,
            w: 0.12
        };
    };
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => {
        metricCalls.push(options.metric);
        if (!firstMetricAttempt) firstMetricAttempt = options;
        const currentProblem = {
            ...options.baseProblem,
            q: options.baseQActive.slice(),
            metadata: {
                ...(options.baseProblem?.metadata || {}),
                cctsm: {
                    taskMetricKind: 'relative_pose',
                    w_task_arm: 0.13,
                    w_task_hier: 0.14,
                    w_task_arm_vfi: 0.1,
                    w_task_hier_vfi: options.metric === 'w_abs_hier_vfi' ? 0.14 : 0.12,
                    w_vfi: 0.14,
                    w: 0.14
                }
            }
        };
        return {
            state: 'active',
            before: 0.12,
            after: 0.14,
            applied: true,
            finalProblem: currentProblem,
            refineResult: {
                converged: true,
                reason: 'projected_candidate',
                qApplied: options.baseQActive.slice(),
                iterations: 0,
                acceptedIterations: 1,
                currentProblem
            },
            nullspaceResult: { reason: 'accepted' }
        };
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
            0,
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.ok(firstMetricAttempt);
        assert.deepEqual(metricCalls, ['w_abs_arm_vfi', 'w_abs_hier_vfi']);
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.metricMode, 'relative');
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.cctsm?.taskMetricKind, 'relative_pose');
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.Jcmd?.length, 6);
        assert.equal(firstMetricAttempt.baseProblem?.metadata?.protectedRows?.length, 6);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer distance-target mj_restore hierarchical VFI refinement uses task-arm primary before task-hier secondary', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0.2, 0, 0.3];
    const pose1 = [1, 0, 0, 0, 0, 0.1, 0, 0];
    const pose2 = [1, 0, 0, 0, 0, -0.1, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 1) % 7)] = 0.07 * (rowIndex + 1);
        return row;
    });
    const Jpose1 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.05 * (rowIndex + 1);
        return row;
    });
    const Jpose2 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[11 + (rowIndex % 7)] = 0.04 * (rowIndex + 1);
        return row;
    });
    const metricCalls = [];
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0.2, 0, 0.3]
            },
            Kinematics: {
                distanceJacobian: () => [new Array(q0.length).fill(0).map((_, index) => (index === 4 ? 0.12 : 0))]
            }
        },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidPose1 = () => pose1.slice();
    visualizer._getHumanoidPose2 = () => pose2.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._getHumanoidSingleArmPoseJacobian = (q, armIndex) => (armIndex === 1 ? Jpose1 : Jpose2).map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.ok(args[3] === 'relative' || args[3] === 'distance');
        return {
            taskMetricKind: 'relative_pose',
            w_task_arm_vfi: 0.09,
            w_task_hier_vfi: 0.12,
            w_task_arm: 0.1,
            w_task_hier: 0.14,
            w_vfi: 0.12,
            w: 0.14
        };
    };
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => {
        metricCalls.push(options.metric);
        const currentProblem = {
            ...options.baseProblem,
            q: options.baseQActive.slice(),
            metadata: {
                ...(options.baseProblem?.metadata || {}),
                cctsm: {
                    taskMetricKind: 'relative_pose',
                    w_task_arm_vfi: 0.1,
                    w_task_hier_vfi: options.metric === 'w_abs_hier_vfi' ? 0.14 : 0.12,
                    w_task_arm: 0.1,
                    w_task_hier: 0.14,
                    w_vfi: 0.14,
                    w: 0.14
                }
            },
            equalities: {
                blocks: {
                    command: { residual: [1e-6] },
                    absolute: { residual: [1e-6] },
                    relative: { residual: [] }
                }
            },
            inequalities: {
                wvfiFloor: { violation: 0 },
                waistXCorridor: { violation: 0 }
            }
        };
        return {
            state: 'active',
            before: 0.12,
            after: options.metric === 'w_abs_arm_vfi' ? 0.13 : 0.14,
            applied: true,
            finalProblem: currentProblem,
            refineResult: {
                converged: true,
                reason: 'projected_candidate',
                qApplied: options.baseQActive.slice(),
                iterations: 0,
                acceptedIterations: 1,
                currentProblem
            },
            nullspaceResult: { reason: 'accepted' }
        };
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
            0,
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.deepEqual(metricCalls, ['w_abs_arm_vfi', 'w_abs_hier_vfi']);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer distance-target metric-only hierarchical VFI refinement can use the worker-side secondary metric fast path', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {
        solveAbsoluteTargetMJRestore: () => ({})
    };
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    visualizer.humanoidMJWorkerModelSpec = { waistChain: [] };
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0.2, 0, 0.3];
    const pose1 = [1, 0, 0, 0, 0, 0.1, 0, 0];
    const pose2 = [1, 0, 0, 0, 0, -0.1, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 2) % 7)] = 0.08 * (rowIndex + 1);
        return row;
    });
    let observedSecondaryMetricRequest = null;
    let localMetricRefineCalls = 0;
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0.2, 0, 0.3]
            },
            Kinematics: {
                distanceJacobian: () => [new Array(q0.length).fill(0).map((_, index) => (index === 4 ? 0.12 : 0))]
            }
        },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
    visualizer._getHumanoidJogMJWorkerClient = () => ({
        solve: async ({ secondaryMetricRequest }) => {
            observedSecondaryMetricRequest = secondaryMetricRequest;
            return {
                converged: true,
                reason: 'converged',
                iterations: 0,
                acceptedIterations: 0,
                lastAcceptedScale: 1,
                commandResidualNorm: 0,
                relativeResidualNorm: 0,
                bodyResidualNorm: 0,
                inequalityViolation: 0,
                qApplied: q0.slice(4, 18),
                secondaryMetric: 'w_abs_hier_vfi',
                secondaryState: 'active',
                secondaryBefore: 1.7,
                secondaryAfter: 1.9,
                secondaryApplied: true
            };
        }
    });
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidPose1 = () => pose1.slice();
    visualizer._getHumanoidPose2 = () => pose2.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.ok(args[3] === 'relative' || args[3] === 'distance');
        return {
            taskMetricKind: 'relative_pose',
            w_task_arm_vfi: 1.8,
            w_task_hier_vfi: 1.9,
            w_task_arm: 1.85,
            w_task_hier: 2.0,
            w_vfi: 1.9,
            w: 2.0
        };
    };
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async () => {
        localMetricRefineCalls += 1;
        throw new Error('local refine should not run when worker metric attempt is accepted');
    };
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
            0,
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                preferWorker: true,
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.deepEqual(observedSecondaryMetricRequest, {
            metric: 'w_abs_hier_vfi',
            metricMode: 'relative',
            stepSize: 0.01,
            restoreMaxIterations: 24,
            waistActiveCount: 0,
            hierarchicalVfiBlend: 1
        });
        assert.equal(localMetricRefineCalls, 0);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer relative-target mj_restore metric-only accepts applied task-aware refinement using metric-button tolerances', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 2) % 7)] = 0.08 * (rowIndex + 1);
        return row;
    });
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: { DQWrapper: {} },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = () => ({
        taskMetricKind: 'relative_pose',
        w_task_hier: 2.009,
        w_task_hier_vfi: 2.021,
        w_vfi: 0.59,
        w: 0.62
    });
    let publishedDiagnostics = null;
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._applyHumanoidJointSolution = () => {};
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => {
        const currentProblem = {
            ...options.baseProblem,
            q: options.baseQActive.slice(),
            metadata: {
                ...(options.baseProblem?.metadata || {}),
                cctsm: {
                    taskMetricKind: 'relative_pose',
                    w_task_hier: 2.021,
                    w_task_hier_vfi: 2.021,
                    w_vfi: 0.59,
                    w: 0.62
                }
            },
            equalities: {
                blocks: {
                    relative: { residual: [1.31e-6] },
                    absolute: { residual: [3.77e-5] }
                }
            },
            inequalities: {
                wvfiFloor: { violation: 0 },
                waistXCorridor: { violation: 0 }
            }
        };
        return {
            state: 'active',
            before: 2.009,
            after: 2.021,
            applied: true,
            finalProblem: currentProblem,
            refineResult: {
                converged: true,
                reason: 'projected_integration',
                qApplied: options.baseQActive.slice(),
                iterations: 0,
                acceptedIterations: 1,
                currentProblem
            },
            nullspaceResult: { reason: 'accepted' }
        };
    };

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativePoseTargetAbsoluteFirstMJRestore(
            relCurrent.slice(),
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.equal(visualizer.lastIKSolveStatus?.converged, true);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'converged');
        assert.equal(Number.isFinite(publishedDiagnostics?.timingSolveMs), true);
        assert.equal(Number.isFinite(publishedDiagnostics?.timingTotalMs), true);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer distance-target mj_restore metric-only accepts applied task-aware refinement using metric-button tolerances', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0.2, 0, 0.3];
    const pose1 = [1, 0, 0, 0, 0, 0.1, 0, 0];
    const pose2 = [1, 0, 0, 0, 0, -0.1, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 1) % 7)] = 0.07 * (rowIndex + 1);
        return row;
    });
    const Jpose1 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.05 * (rowIndex + 1);
        return row;
    });
    const Jpose2 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[11 + (rowIndex % 7)] = 0.04 * (rowIndex + 1);
        return row;
    });
    globalThis.window = {
        humanoidQ: q0.slice(),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0.2, 0, 0.3]
            },
            Kinematics: {
                distanceJacobian: () => [new Array(q0.length).fill(0).map((_, index) => (index === 4 ? 0.12 : 0))]
            }
        },
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };
    globalThis.document = {
        getElementById: () => null
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidPose1 = () => pose1.slice();
    visualizer._getHumanoidPose2 = () => pose2.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._getHumanoidSingleArmPoseJacobian = (q, armIndex) => (armIndex === 1 ? Jpose1 : Jpose2).map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = () => ({
        taskMetricKind: 'relative_pose',
        w_task_hier: 0.31,
        w_task_hier_vfi: 0.34,
        w_vfi: 0.34,
        w: 0.31
    });
    let publishedDiagnostics = null;
    visualizer._publishHumanoidJogDiagnostics = (payload) => {
        publishedDiagnostics = payload;
    };
    visualizer._applyHumanoidJointSolution = () => {};
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async (options) => {
        const currentProblem = {
            ...options.baseProblem,
            q: options.baseQActive.slice(),
            metadata: {
                ...(options.baseProblem?.metadata || {}),
                cctsm: {
                    taskMetricKind: 'relative_pose',
                    w_task_hier: 0.34,
                    w_task_hier_vfi: 0.34,
                    w_vfi: 0.34,
                    w: 0.31
                }
            },
            equalities: {
                blocks: {
                    command: { residual: [1.31e-6] },
                    absolute: { residual: [3.77e-5] },
                    relative: { residual: [] }
                }
            },
            inequalities: {
                wvfiFloor: { violation: 0 },
                waistXCorridor: { violation: 0 }
            }
        };
        return {
            state: 'active',
            before: 0.31,
            after: 0.34,
            applied: true,
            finalProblem: currentProblem,
            refineResult: {
                converged: true,
                reason: 'projected_integration',
                qApplied: options.baseQActive.slice(),
                iterations: 0,
                acceptedIterations: 1,
                currentProblem
            },
            nullspaceResult: { reason: 'accepted' }
        };
    };

    try {
        const result = await visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
            0,
            q0.slice(),
            true,
            false,
            {
                solverMode: 'mj_restore',
                jogMetricOverride: 'w_abs_hier_vfi',
                metricDirection: 'increase'
            }
        );
        assert.equal(result, true);
        assert.equal(visualizer.lastIKSolveStatus?.converged, true);
        assert.equal(visualizer.lastIKSolveStatus?.reason, 'converged');
        assert.equal(Number.isFinite(publishedDiagnostics?.timingSolveMs), true);
        assert.equal(Number.isFinite(publishedDiagnostics?.timingTotalMs), true);
    } finally {
        globalThis.document = originalDocument;
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer _runHumanoidMJRestorePreWStage builds relative-task pre-w problems with relative metric semantics', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [0, 1, 0, 0, 0, 0, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 2) % 7)] = 0.08 * (rowIndex + 1);
        return row;
    });
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.equal(args[3], 'relative');
        return {
            taskMetricKind: 'relative_pose',
            w_task_hier: 0.2,
            w_task_hier_vfi: 0.17,
            w_vfi: 0.17
        };
    };
    let helperCalled = false;
    visualizer._runHumanoidMJRestoreMetricOnlyPreWAttempt = async (options) => {
        helperCalled = true;
        const context = {
            qInitialFull: q0.slice(),
            qInitialActive: visualizer._sliceHumanoidActiveJoints(q0, 4, 14),
            armStart: 4,
            activeCount: 14,
            hardLimits: null,
            lowerBounds: new Array(14).fill(-Math.PI),
            upperBounds: new Array(14).fill(Math.PI),
            stepBounds: new Array(14).fill(0.05),
            includeWaist: false,
            keepBodyZVertical: false,
            restoreProfile: {
                referenceStiffness: 1,
                referenceDamping: 0.35
            }
        };
        const problem = options.buildProblem(context.qInitialActive.slice(), context);
        assert.equal(problem?.metadata?.metricMode, 'relative');
        assert.equal(problem?.metadata?.cctsm?.taskMetricKind, 'relative_pose');
        assert.equal(problem?.metadata?.Jcmd?.length, 6);
        assert.equal(problem?.metadata?.protectedRows?.length, 6);
        return {
            state: 'active',
            applied: true,
            qApplied: q0.slice(),
            qPreEligible: true
        };
    };

    await visualizer._runHumanoidMJRestorePreWStage({
        controlMode: 'relative',
        q0,
        dqAbsoluteProtectedTargetArray: absCurrent.slice(),
        fixedRelativeProtectedTargetArray: relCurrent.slice(),
        enableJointLimitAvoidance: true,
        includeWaist: false,
        keepBodyZVertical: false,
        jogMetric: 'w_abs_hier_vfi',
        metricDirection: 'increase'
    });
    assert.equal(helperCalled, true);
});

test('RobotVisualizer _runHumanoidMJRestorePreWStage builds distance-task pre-w problems with relative-pose metric semantics', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;
    const originalWindow = globalThis.window;
    const q0 = new Array(20).fill(0).map((_, index) => index * 0.01);
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0.2, 0, 0.3];
    const pose1 = [1, 0, 0, 0, 0, 0.1, 0, 0];
    const pose2 = [1, 0, 0, 0, 0, -0.1, 0, 0];
    const Jabs = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.1 * (rowIndex + 1);
        return row;
    });
    const Jrel = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + ((rowIndex + 1) % 7)] = 0.07 * (rowIndex + 1);
        return row;
    });
    const Jpose1 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[4 + (rowIndex % 7)] = 0.05 * (rowIndex + 1);
        return row;
    });
    const Jpose2 = Array.from({ length: 8 }, (_, rowIndex) => {
        const row = new Array(q0.length).fill(0);
        row[11 + (rowIndex % 7)] = 0.04 * (rowIndex + 1);
        return row;
    });
    globalThis.window = {
        ...(originalWindow || {}),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: (value) => value
            },
            Kinematics: {
                distanceJacobian: () => [new Array(q0.length).fill(0).map((_, index) => (index === 4 ? 0.12 : 0))]
            }
        }
    };
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._getHumanoidPose1 = () => pose1.slice();
    visualizer._getHumanoidPose2 = () => pose2.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => Jabs.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => Jrel.map((row) => row.slice());
    visualizer._getHumanoidSingleArmPoseJacobian = (q, armIndex) => (armIndex === 1 ? Jpose1 : Jpose2).map((row) => row.slice());
    visualizer._buildManipulabilityVFIConstraints = () => [];
    visualizer._computeModeCCTSM = (...args) => {
        assert.equal(args[3], 'relative');
        return {
            taskMetricKind: 'relative_pose',
            w_task_hier: 0.12,
            w_task_hier_vfi: 0.09,
            w_vfi: 0.09
        };
    };
    let helperCalled = false;
    visualizer._runHumanoidMJRestoreMetricOnlyPreWAttempt = async (options) => {
        helperCalled = true;
        const context = {
            qInitialFull: q0.slice(),
            qInitialActive: visualizer._sliceHumanoidActiveJoints(q0, 4, 14),
            armStart: 4,
            activeCount: 14,
            hardLimits: null,
            lowerBounds: new Array(14).fill(-Math.PI),
            upperBounds: new Array(14).fill(Math.PI),
            stepBounds: new Array(14).fill(0.05),
            includeWaist: false,
            keepBodyZVertical: false,
            restoreProfile: {
                referenceStiffness: 1,
                referenceDamping: 0.35
            }
        };
        const problem = options.buildProblem(context.qInitialActive.slice(), context);
        assert.equal(problem?.metadata?.metricMode, 'relative');
        assert.equal(problem?.metadata?.cctsm?.taskMetricKind, 'relative_pose');
        assert.equal(problem?.metadata?.Jcmd?.length, 6);
        assert.equal(problem?.metadata?.protectedRows?.length, 6);
        return {
            state: 'active',
            applied: true,
            qApplied: q0.slice(),
            qPreEligible: true
        };
    };

    try {
        await visualizer._runHumanoidMJRestorePreWStage({
            controlMode: 'distance',
            q0,
            dqAbsoluteProtectedTargetArray: absCurrent.slice(),
            fixedRelativeProtectedTargetArray: relCurrent.slice(),
            enableJointLimitAvoidance: true,
            includeWaist: false,
            keepBodyZVertical: false,
            jogMetric: 'w_abs_hier',
            metricDirection: 'increase'
        });
        assert.equal(helperCalled, true);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer _computeModeCCTSM provides task-specific manipulability aliases for robot1 and waist-body-link-xz', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const q = new Array(20).fill(0);
    const armJacobian = Array.from({ length: 8 }, () => new Array(q.length).fill(0));
    armJacobian[1][4] = 0.5;
    armJacobian[2][5] = 0.5;
    armJacobian[3][6] = 0.5;
    armJacobian[5][7] = 0.5;
    armJacobian[6][8] = 0.5;
    armJacobian[7][9] = 0.5;
    const waistJacobian = Array.from({ length: 8 }, () => new Array(q.length).fill(0));
    waistJacobian[5][0] = 0.5;
    waistJacobian[7][1] = 0.5;
    globalThis.window = {
        ...(originalWindow || {}),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0.1, 0, 0.2]
            }
        }
    };
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidSingleArmPoseJacobian = () => armJacobian.map((row) => row.slice());
    visualizer.humanoidModel = {
        fkmEffector: () => [1, 0, 0, 0, 0, 0, 0, 0],
        poseJacobianEffector: () => waistJacobian.map((row) => row.slice())
    };

    try {
        const robot1Result = visualizer._computeModeCCTSM(q.slice(), 4, 7, 'robot1', [], { force: true });
        assert.equal(robot1Result?.taskMetricKind, 'left_arm_pose');
        assert.ok(Number.isFinite(robot1Result?.w_task_hier));

        const waistResult = visualizer._computeModeCCTSM(q.slice(), 0, 4, 'waist-body-link-xz', [], { force: true });
        assert.equal(waistResult?.taskMetricKind, 'body_link_xz');
        assert.ok(Number.isFinite(waistResult?.w_task_hier));
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer _computeModeCCTSM uses task-only command Jacobians for relative and distance task metrics', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const q = new Array(20).fill(0);
    const absPose = [1, 0, 0, 0, 0, 0, 0, 0];
    const relPose = [1, 0, 0, 0, 0, 0.2, 0, 0.3];
    const JA8 = Array.from({ length: 8 }, () => new Array(q.length).fill(0));
    const JR8 = Array.from({ length: 8 }, () => new Array(q.length).fill(0));
    JA8[1][4] = 0.5;
    JA8[2][5] = 0.5;
    JA8[3][6] = 0.5;
    JA8[5][7] = 0.5;
    JA8[6][8] = 0.5;
    JA8[7][9] = 0.5;
    JR8[1][4] = 0.5;
    JR8[2][5] = 0.5;
    JR8[3][6] = 0.5;
    JR8[5][7] = 0.5;
    JR8[6][8] = 0.5;
    JR8[7][9] = 0.5;
    const activeStart = 4;
    const activeCount = 14;
    globalThis.window = {
        ...(originalWindow || {}),
        DQModule: {
            DQWrapper: {
                createFromArray: (value) => value,
                getTranslation: () => [0, 0.2, 0, 0.3]
            },
            Kinematics: {
                distanceJacobian: () => {
                    const row = new Array(q.length).fill(0);
                    row[4] = 0.25;
                    row[5] = 0.1;
                    return [row];
                }
            }
        }
    };
    visualizer._getHumanoidAbsolutePose = () => absPose.slice();
    visualizer._getHumanoidRelativePose = () => relPose.slice();
    visualizer._getHumanoidAbsolutePoseJacobian = () => JA8.map((row) => row.slice());
    visualizer._getHumanoidRelativePoseJacobian = () => JR8.map((row) => row.slice());
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0.1, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, -0.1, 0, 0];
    visualizer._getHumanoidSingleArmPoseJacobian = () => JA8.map((row) => row.slice());

    try {
        const JAActive = JA8.map((row) => row.slice(activeStart, activeStart + activeCount));
        const JRActive = JR8.map((row) => row.slice(activeStart, activeStart + activeCount));
        const relativeTaskOnly = computeCombinedCCTSM(computeTwistJacobian(JRActive, relPose), []).w;
        const relativeResult = visualizer._computeModeCCTSM(q.slice(), activeStart, activeCount, 'relative', [], { force: true });
        assert.equal(relativeResult?.taskMetricKind, 'relative_pose');
        assert.ok(Number.isFinite(relativeResult?.w_task_hier));
        assert.ok(Math.abs(relativeResult.w_task_hier - relativeTaskOnly) < 1e-9);
        assert.ok(Math.abs(relativeResult.w_task_hier - relativeResult.w) > 1e-6);

        const distanceResult = visualizer._computeModeCCTSM(q.slice(), activeStart, activeCount, 'distance', [], { force: true });
        assert.equal(distanceResult?.taskMetricKind, 'relative_pose');
        assert.ok(Number.isFinite(distanceResult?.w_task_hier));
        assert.ok(Math.abs(distanceResult.w_task_hier - relativeTaskOnly) < 1e-9);
        assert.ok(Math.abs(distanceResult.w_task_hier - distanceResult.w) > 1e-6);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer plain mj_restore routes robot1 and robot2 jogs through the dedicated MJ single-arm helper', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('mj_restore');
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    let mjCalls = 0;
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._updateHumanoidSingleArmPoseTargetMJRestore = (...args) => {
        mjCalls += 1;
        assert.equal(args[mjCalls === 1 ? 2 : 2], mjCalls === 1 ? 1 : 2);
        return true;
    };
    visualizer.updateHumanoidSingleArmPoseTarget = () => {
        throw new Error('legacy single-arm backend should not be used');
    };
    const DQ = {
        createFromArray: (value) => value,
        getRotation: () => [1, 0, 0, 0],
        translation: (x, y, z) => ({ x, y, z }),
        multiply: (a, b) => ({ a, b }),
        toArray: () => [1, 0, 0, 0, 0, 0, 0, 0]
    };
    globalThis.window.DQModule = { DQWrapper: DQ };

    try {
        visualizer.moveHumanoidLeftArmAlongAxis('x', 'positive');
        visualizer.moveHumanoidRightArmAlongAxis('x', 'positive');
        assert.equal(mjCalls, 2);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer plain mj_restore routes robot1-waist and robot2-waist jogs through the dedicated MJ single-arm-plus-waist helper', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('mj_restore');
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    let mjCalls = 0;
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._updateHumanoidSingleArmPlusWaistPoseTargetMJRestore = (...args) => {
        mjCalls += 1;
        assert.equal(args[2], mjCalls === 1 ? 1 : 2);
        return true;
    };
    const DQ = {
        createFromArray: (value) => value,
        getRotation: () => [1, 0, 0, 0],
        translation: (x, y, z) => ({ x, y, z }),
        multiply: (a, b) => ({ a, b }),
        toArray: () => [1, 0, 0, 0, 0, 0, 0, 0]
    };
    globalThis.window.DQModule = { DQWrapper: DQ };

    try {
        visualizer.moveHumanoidLeftArmWithWaistAlongAxis('x', 'positive');
        visualizer.moveHumanoidRightArmWithWaistAlongAxis('x', 'positive');
        assert.equal(mjCalls, 2);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer zero-space single-arm-plus-waist control keeps the opposite arm pose fixed via coordinated compensation', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('priority');
    visualizer.humanoidModel = {};
    visualizer.humanoidWaistLength = 1;
    visualizer.humanoidLeftArmLength = 1;
    visualizer.humanoidRightArmLength = 1;
    visualizer.humanoidJointNames = ['waist0', 'ljoint1', 'rjoint1'];
    visualizer.humanoidURDF = {
        joints: {
            waist0: { limit: { lower: -2, upper: 2 } },
            ljoint1: { limit: { lower: -2, upper: 2 } },
            rjoint1: { limit: { lower: -2, upper: 2 } }
        }
    };

    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: [0, 0, 0],
        updateGlobalVariables: () => {},
        updateRobotPose: () => {},
        DQModule: { DQWrapper: {} }
    };

    const makePose = (trackedValue) => [1, trackedValue, 0, 0, 0, 0, 0, 0];
    const makeJacobian = (waistCoeff, leftCoeff, rightCoeff) => {
        const J = Array.from({ length: 8 }, () => [0, 0, 0]);
        J[1] = [waistCoeff, leftCoeff, rightCoeff];
        return J;
    };

    visualizer._getHumanoidPose1 = (q) => makePose(q[0]);
    visualizer._getHumanoidPose2 = (q) => makePose(q[0] + q[2]);
    visualizer._getHumanoidPose1Jacobian = () => makeJacobian(1, 0, 0);
    visualizer._getHumanoidPose2Jacobian = () => makeJacobian(1, 0, 1);
    visualizer.setHumanoidJoints = () => {};

    try {
        const targetPose1 = makePose(1);
        const ok = visualizer.updateHumanoidSingleArmPlusWaistPoseTarget(targetPose1, [0, 0, 0], 1);
        assert.equal(ok, true);
        const qSolved = globalThis.window.humanoidQ.slice();
        const pose1Solved = visualizer._getHumanoidPose1(qSolved);
        const pose2Solved = visualizer._getHumanoidPose2(qSolved);

        assert.ok(Math.abs((pose1Solved[1] || 0) - 1) < 1e-2);
        assert.ok(Math.abs(pose2Solved[1] || 0) < 1e-2);
        assert.ok(Math.abs(qSolved[0]) > 1e-3, 'waist joint should participate');
        assert.ok(Math.abs(qSolved[2]) > 1e-3, 'opposite arm should compensate to keep its pose fixed');
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});


test('RobotVisualizer plain mj_restore routes waist-body-link-xz jogs through the dedicated MJ waist helper', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('mj_restore');
    const originalWindow = globalThis.window;
    globalThis.window = {
        humanoidQ: new Array(20).fill(0),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    visualizer.humanoidModel = {
        fkmEffector: () => [1, 0, 0, 0, 0, 0, 0, 0]
    };
    visualizer._updateHumanoidWaistPoseTargetMJRestore = () => true;
    visualizer.updateHumanoidWaistPoseTarget = () => {
        throw new Error('legacy waist backend should not be used');
    };
    let called = 0;
    visualizer._updateHumanoidWaistPoseTargetMJRestore = (...args) => {
        called += 1;
        return true;
    };
    const DQ = {
        createFromArray: (value) => value,
        getRotation: () => [1, 0, 0, 0],
        translation: (x, y, z) => ({ x, y, z }),
        multiply: (a, b) => ({ a, b }),
        toArray: () => [1, 0, 0, 0, 0, 0, 0, 0]
    };
    globalThis.window.DQModule = { DQWrapper: DQ };

    try {
        visualizer.moveHumanoidWaistBodyLinkAlongAxis('x', 'positive');
        assert.equal(called, 1);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer mj_restore_pre_w routes robot1 and robot2 jogs through a local special-mode pre-w stage before stage 2', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier_vfi';
    const originalWindow = globalThis.window;
    const currentQ = new Array(20).fill(0).map((_, index) => index * 0.01);
    globalThis.window = {
        humanoidQ: currentQ.slice(),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    const preWOffsets = {
        1: 0.11,
        2: -0.07
    };
    const stage2Calls = [];
    let preWCalls = 0;
    visualizer._getHumanoidPose1 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidPose2 = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._runHumanoidMJRestoreSpecialModePreWStage = async (options) => {
        preWCalls += 1;
        assert.equal(options.preWMetric, 'w_abs_hier_vfi');
        assert.equal(options.mode, preWCalls === 1 ? 'robot1' : 'robot2');
        const offset = preWOffsets[preWCalls];
        return {
            state: 'active',
            applied: true,
            qApplied: currentQ.map((value, index) => value + offset + index * 0.001),
            qPreEligible: true
        };
    };
    visualizer._updateHumanoidCustomPoseTargetMJRestore = (options) => {
        stage2Calls.push(options);
        assert.equal(options.solverMode, 'mj_restore_pre_w');
        assert.equal(options.internalOptions?.preWState, 'applied');
        assert.equal(options.internalOptions?.preWMetric, 'w_abs_hier_vfi');
        assert.equal(options.internalOptions?.preWStart, 'q_pre');
        const expectedOffset = preWOffsets[stage2Calls.length];
        assert.ok(Array.isArray(options.currentQ));
        assert.equal(options.currentQ[0], currentQ[0] + expectedOffset);
        return true;
    };

    try {
        await visualizer._updateHumanoidSingleArmPoseTargetMJRestore([1, 0, 0, 0, 0, 0, 0, 0], currentQ.slice(), 1, false, false, { solverMode: 'mj_restore_pre_w' });
        await visualizer._updateHumanoidSingleArmPoseTargetMJRestore([1, 0, 0, 0, 0, 0, 0, 0], currentQ.slice(), 2, false, false, { solverMode: 'mj_restore_pre_w' });
        assert.equal(preWCalls, 2);
        assert.equal(stage2Calls.length, 2);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer mj_restore_pre_w routes waist-body-link-xz jogs through a local special-mode pre-w stage before stage 2', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('mj_restore_pre_w');
    visualizer._getHumanoidManifoldJogSecondaryMetric = () => 'w_abs_hier';
    const originalWindow = globalThis.window;
    const currentQ = new Array(20).fill(0).map((_, index) => index * 0.01);
    globalThis.window = {
        humanoidQ: currentQ.slice(),
        updateGlobalVariables: () => {},
        updateRobotPose: () => {}
    };

    visualizer.humanoidModel = {
        fkmEffector: () => [1, 0, 0, 0, 0, 0, 0, 0],
        poseJacobianEffector: () => [new Array(currentQ.length).fill(0)]
    };
    let preWCalls = 0;
    let stage2Calls = 0;
    visualizer._runHumanoidMJRestoreSpecialModePreWStage = async (options) => {
        preWCalls += 1;
        assert.equal(options.preWMetric, 'w_abs_hier');
        assert.equal(options.includeWaist, true);
        assert.equal(options.mode, 'waist-body-link-xz');
        return {
            state: 'active',
            applied: true,
            qApplied: currentQ.map((value, index) => value + 0.02 + index * 0.001),
            qPreEligible: true
        };
    };
    visualizer._updateHumanoidCustomPoseTargetMJRestore = (options) => {
        stage2Calls += 1;
        assert.equal(options.solverMode, 'mj_restore_pre_w');
        assert.equal(options.internalOptions?.preWState, 'applied');
        assert.equal(options.internalOptions?.preWMetric, 'w_abs_hier');
        assert.equal(options.internalOptions?.preWStart, 'q_pre');
        assert.equal(options.currentQ[0], currentQ[0] + 0.02);
        return true;
    };
    const DQ = {
        createFromArray: (value) => value,
        getRotation: () => [1, 0, 0, 0],
        translation: (x, y, z) => ({ x, y, z }),
        multiply: (a, b) => ({ a, b }),
        toArray: () => [1, 0, 0, 0, 0, 0, 0, 0]
    };
    globalThis.window.DQModule = { DQWrapper: DQ };

    try {
        await visualizer.moveHumanoidWaistBodyLinkAlongAxis('x', 'positive');
        assert.equal(preWCalls, 1);
        assert.equal(stage2Calls, 1);
    } finally {
        if (originalWindow === undefined) {
            delete globalThis.window;
        } else {
            globalThis.window = originalWindow;
        }
    }
});

test('RobotVisualizer publishes rendered mj_restore_pre_w diagnostics lines for applied, skipped, and disabled states', () => {
    const visualizer = new RobotVisualizer(null);
    const originalDocument = globalThis.document;
    const textEl = { textContent: '' };
    const tsEl = { textContent: '' };
    const panelEl = { classList: { toggle: () => {} } };

    globalThis.document = {
        getElementById: (id) => {
            if (id === 'humanoid-vfi-diagnostics-text') return textEl;
            if (id === 'humanoid-vfi-diagnostics-timestamp') return tsEl;
            if (id === 'humanoid-vfi-diagnostics-panel') return panelEl;
            return null;
        }
    };

    try {
        visualizer._publishHumanoidJogDiagnostics(buildHumanoidJogMJDiagnostics({
            solverMode: 'mj_restore_pre_w',
            modeTag: 'absolute-target',
            restorationResult: {
                iterations: 1,
                acceptedIterations: 1
            },
            polishIterations: 1,
            preWState: 'applied',
            preWMetric: 'w_abs_hier',
            preWStart: 'q_pre'
        }));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('iters(total/restore/polish): 2 / 1 / 1    active_joints: n/a'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w: applied'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w_metric: w_abs_hier'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('task_pre_w: w_abs'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w_start: q_pre'));

        visualizer._publishHumanoidJogDiagnostics({
            ...buildHumanoidJogMJDiagnostics({
                solverMode: 'mj_restore_tr',
                modeTag: 'cooperative-ruled-surface-polishing-target',
                restorationResult: {
                    iterations: 2,
                    acceptedIterations: 1,
                    lastAcceptedRho: 0.82,
                    lastPredictedReduction: 0.6,
                    lastActualReduction: 0.49,
                    acceptedBy: 'filter',
                    lastRejectedRho: -0.15,
                    lastRejectedPredictedReduction: 0.08,
                    lastRejectedActualReduction: -0.01,
                    lastRejectedReason: 'poor_ratio'
                },
                polishIterations: 2
            }),
            iterations: 4
        });
        assert.ok(
            visualizer.lastHumanoidJogDiagnostics.renderedLines.includes(
                'iters(total/restore/polish): 4 / 2 / 2    active_joints: n/a'
            )
        );
        assert.ok(
            visualizer.lastHumanoidJogDiagnostics.renderedLines.some((line) => (
                typeof line === 'string' &&
                line.startsWith('restore(pred/ared/rho/by):') &&
                line.includes('filter')
            ))
        );
        assert.ok(
            visualizer.lastHumanoidJogDiagnostics.renderedLines.some((line) => (
                typeof line === 'string' &&
                line.startsWith('restore(rej_pred/rej_ared/rej_rho/rej):') &&
                line.includes('poor_ratio')
            ))
        );

        visualizer._publishHumanoidJogDiagnostics({
            ...buildHumanoidJogMJDiagnostics({
                solverMode: 'mj_restore_tr',
                modeTag: 'cooperative-ruled-surface-polishing-target',
                restorationResult: {
                    iterations: 2,
                    acceptedIterations: 1
                },
                polishIterations: 2
            }),
            iterations: 2
        });
        assert.ok(
            visualizer.lastHumanoidJogDiagnostics.renderedLines.includes(
                'iters(total/restore/polish): 4 / 2 / 2    active_joints: n/a'
            )
        );

        visualizer._publishHumanoidJogDiagnostics(buildHumanoidJogMJDiagnostics({
            solverMode: 'mj_restore_pre_w',
            modeTag: 'absolute-target',
            preWState: 'skipped(no_progress)',
            preWMetric: 'w_abs_hier',
            preWStart: 'q0'
        }));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w: skipped(no_progress)'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w_metric: w_abs_hier'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('task_pre_w: w_abs'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w_start: q0'));

        visualizer._publishHumanoidJogDiagnostics(buildHumanoidJogMJDiagnostics({
            solverMode: 'mj_restore_pre_w',
            modeTag: 'relative-target',
            jogMetric: 'w_abs_hier_vfi',
            jogMetricState: 'active',
            jogMetricBefore: 1.2,
            jogMetricAfter: 1.3,
            jogMetricApplied: true,
            preWState: 'disabled_for_mode',
            preWMetric: 'w_abs_hier_vfi',
            preWStart: 'q0'
        }));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('task_w: w_rel_vfi'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w: disabled_for_mode'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w_metric: w_abs_hier_vfi'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('task_pre_w: w_rel_vfi'));
        assert.ok(visualizer.lastHumanoidJogDiagnostics.renderedLines.includes('pre_w_start: q0'));

        visualizer._publishHumanoidJogDiagnostics(buildHumanoidJogMJDiagnostics({
            solverMode: 'mj_restore',
            modeTag: 'cooperative-ruled-surface-polishing-target',
            jogMetric: 'none',
            jogMetricState: 'skipped(waiting_for_restore)',
            jogMetricBefore: Number.NaN,
            jogMetricAfter: Number.NaN,
            jogMetricApplied: false
        }));
        assert.ok(
            visualizer.lastHumanoidJogDiagnostics.renderedLines.includes(
                'jog_metric: none  skipped(waiting_for_restore)  before=n/a  after=n/a  applied=n/a'
            )
        );
    } finally {
        globalThis.document = originalDocument;
    }
});

test('RobotVisualizer optimizeHumanoidManipulability routes the w button through mj_restore_tr even when another solver is selected', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let capturedArgs = null;

    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('priority');
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidRelativePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore = (...args) => {
        capturedArgs = args;
        return true;
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'dual-arm-control-mode-select') return { value: 'absolute' };
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    try {
        const ok = visualizer.optimizeHumanoidManipulability('increase');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[7]?.solverMode, 'mj_restore_tr');
        assert.equal(capturedArgs[7]?.jogMetricOverride, 'w_cmd');
        assert.equal(capturedArgs[7]?.metricDirection, 'increase');
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
    }
});

test('RobotVisualizer optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityIterative routes the iterative w button through mj_restore_tr regardless of the selected solver', () => {
    const visualizer = new RobotVisualizer(null);
    const originalWindow = globalThis.window;
    const originalDocument = globalThis.document;
    let capturedArgs = null;

    visualizer.humanoidMode = true;
    visualizer.setHumanoidJogSolverMode('vfi');
    visualizer._getHumanoidAbsolutePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._getHumanoidRelativePose = () => [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        capturedArgs = args;
        return true;
    };

    globalThis.window = {
        humanoidQ: new Array(20).fill(0)
    };
    globalThis.document = {
        getElementById(id) {
            if (id === 'dual-arm-control-mode-select') return { value: 'absolute' };
            if (id === 'joint-limit-avoidance-toggle') return { checked: true };
            return null;
        }
    };

    try {
        const ok = visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulabilityIterative('increase');
        assert.equal(ok, true);
        assert.ok(Array.isArray(capturedArgs));
        assert.equal(capturedArgs[6]?.solverMode, 'mj_restore_tr');
        assert.equal(capturedArgs[6]?.jogMetricOverride, 'w_abs_hier_vfi');
        assert.equal(capturedArgs[6]?.metricDirection, 'increase');
    } finally {
        if (originalWindow === undefined) delete globalThis.window;
        else globalThis.window = originalWindow;
        if (originalDocument === undefined) delete globalThis.document;
        else globalThis.document = originalDocument;
    }
});

test('RobotVisualizer worker queue preserves mj_restore_pre_w solver identity and pre-w metadata when draining', async () => {
    const visualizer = new RobotVisualizer(null);
    let manifoldArgs = null;
    let resolveCall = null;
    const called = new Promise((resolve) => {
        resolveCall = resolve;
    });
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        manifoldArgs = args;
        resolveCall();
        return Promise.resolve(true);
    };

    const enqueued = visualizer._enqueueHumanoidJogMJRestoreWorkerRequest({
        dqAbsoluteTargetArray: [1, 0, 0, 0, 0, 0, 0, 0],
        currentQ: [0.1, -0.2, 0.3],
        enableJointLimitAvoidance: true,
        includeWaist: false,
        keepBodyZVertical: false,
        fixedRelativeTargetArray: [0, 1, 0, 0, 0, 0, 0, 0],
        translationAxisGuard: null,
        solverMode: 'mj_restore_pre_w',
        jogMetricOverride: 'none',
        metricDirection: 'increase',
        preWState: 'applied',
        preWMetric: 'w_abs_hier',
        preWStart: 'q_pre'
    });

    assert.equal(enqueued, true);
    await called;
    assert.ok(Array.isArray(manifoldArgs));
    assert.equal(manifoldArgs[6]?.solverMode, 'mj_restore_pre_w');
    assert.equal(manifoldArgs[6]?.preWState, 'applied');
    assert.equal(manifoldArgs[6]?.preWMetric, 'w_abs_hier');
    assert.equal(manifoldArgs[6]?.preWStart, 'q_pre');
});

test('RobotVisualizer routes metric-only hierarchical w_abs_hier_vfi MJ requests through the worker queue like w_abs_hier', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0, 0, 0, 0];
    const relTarget = [0, 1, 0, 0, 0, 0, 0, 0];

    let enqueued = false;
    let manifoldArgs = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
    visualizer._getHumanoidAbsolutePose = () => absTarget.slice();
    visualizer._getHumanoidRelativePose = () => relTarget.slice();
    visualizer._enqueueHumanoidJogMJRestoreWorkerRequest = () => {
        enqueued = true;
        return true;
    };
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        manifoldArgs = args;
        return Promise.resolve(true);
    };

    const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
        absTarget.slice(),
        currentQ.slice(),
        false,
        false,
        false,
        relTarget.slice(),
        null,
        {
            jogMetricOverride: 'w_abs_hier_vfi',
            metricDirection: 'increase'
        }
    );

    assert.equal(result, true);
    assert.equal(enqueued, true);
    assert.equal(manifoldArgs, null);
});

test('RobotVisualizer iterative metric projection plan runs 15 metric passes followed by 35 restore passes', async () => {
    const visualizer = new RobotVisualizer(null);
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: 0
        },
        inequalities: {
            wvfiFloor: { violation: 0 },
            waistXCorridor: { violation: 0 }
        },
        metadata: {
            cctsm: { marker: qActive[0] }
        }
    });
    let metricPasses = 0;
    let restorePasses = 0;
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.marker ?? Number.NaN;
    visualizer._attemptHumanoidJogManifoldMetricRefinement = async ({ baseQActive }) => {
        metricPasses += 1;
        const qApplied = [(baseQActive[0] ?? 0) + 1];
        const finalProblem = evaluateProblem(qApplied);
        return {
            metric: 'w_abs_hier_vfi',
            state: 'active',
            before: qApplied[0] - 1,
            after: qApplied[0],
            applied: true,
            axisLabel: '',
            nullspaceResult: { accepted: true, stepNorm: 1 },
            refineResult: {
                converged: true,
                reason: 'projected_candidate',
                qApplied,
                iterations: 0,
                acceptedIterations: 1,
                lastAcceptedScale: 1,
                currentProblem: finalProblem
            },
            finalProblem,
            directionResult: { applied: true, reason: 'improved', before: qApplied[0] - 1, after: qApplied[0] }
        };
    };
    const restoreFn = async ({ qInitial }) => {
        restorePasses += 1;
        const qApplied = [(qInitial[0] ?? 0) + 1];
        const currentProblem = evaluateProblem(qApplied);
        return {
            converged: true,
            reason: 'converged',
            qApplied,
            iterations: 1,
            acceptedIterations: 1,
            lastAcceptedScale: 1,
            currentProblem
        };
    };

    const result = await visualizer._runHumanoidIterativeMetricProjectionPlan({
        baseProblem: evaluateProblem([0]),
        baseQActive: [0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 1,
        activeLimits: null,
        lowerBounds: [-10],
        upperBounds: [10],
        stepBounds: [10],
        wvfiMin: 0,
        restoreMaxIterations: 1,
        damping: 1e-4,
        minScale: 1e-6,
        reductionFactor: 0.5,
        equalityTolerance: 1e-12,
        inequalityTolerance: 1e-12,
        stepSize: 0.01,
        restoreFn,
        restoreConfig: {
            referenceStiffness: 1,
            referenceDamping: 0.35
        },
        metricIterations: 15,
        restoreIterations: 35
    });

    assert.equal(metricPasses, 15);
    assert.equal(restorePasses, 35);
    assert.equal(result.applied, true);
    assert.equal(result.after, 50);
    assert.equal(result.refineResult?.qApplied?.[0], 50);
});

test('RobotVisualizer routes metric-only w_abs_vfi MJ requests to the local solver instead of the worker queue', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const absTarget = [1, 0, 0, 0, 0, 0, 0, 0];
    const relTarget = [0, 1, 0, 0, 0, 0, 0, 0];

    let enqueued = false;
    let manifoldArgs = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
    visualizer._getHumanoidAbsolutePose = () => absTarget.slice();
    visualizer._getHumanoidRelativePose = () => relTarget.slice();
    visualizer._enqueueHumanoidJogMJRestoreWorkerRequest = () => {
        enqueued = true;
        return true;
    };
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        manifoldArgs = args;
        return Promise.resolve(true);
    };

    const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
        absTarget.slice(),
        currentQ.slice(),
        false,
        false,
        false,
        relTarget.slice(),
        null,
        {
            jogMetricOverride: 'w_abs_vfi',
            metricDirection: 'increase'
        }
    );

    assert.equal(result, true);
    assert.equal(enqueued, false);
    assert.ok(Array.isArray(manifoldArgs));
    assert.equal(manifoldArgs[6]?.solverMode, 'mj_restore');
    assert.equal(manifoldArgs[6]?.preferWorker, false);
    assert.equal(manifoldArgs[6]?.jogMetricOverride, 'w_abs_vfi');
    assert.equal(manifoldArgs[6]?.metricDirection, 'increase');
});

test('RobotVisualizer routes full-body upright mj_restore pose jogs with waist regularization to the local solver', async () => {
    const visualizer = new RobotVisualizer(null);
    const currentQ = [0.1, -0.2, 0.3];
    const absCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    const absTarget = [1, 0, 0, 0, -0.01, 0, 0, 0];
    const relCurrent = [1, 0, 0, 0, 0, 0, 0, 0];
    visualizer.humanoidWaistLength = 4;
    visualizer.humanoidLeftArmLength = 7;
    visualizer.humanoidRightArmLength = 7;

    let enqueued = false;
    let manifoldArgs = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
    visualizer._getHumanoidAbsolutePose = () => absCurrent.slice();
    visualizer._getHumanoidRelativePose = () => relCurrent.slice();
    visualizer._signedPoseError = () => [1];
    visualizer._computePoseErrorNorm = () => 0.1;
    visualizer._enqueueHumanoidJogMJRestoreWorkerRequest = () => {
        enqueued = true;
        return true;
    };
    visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstManifold = (...args) => {
        manifoldArgs = args;
        return Promise.resolve(true);
    };

    const result = await visualizer._updateHumanoidJointsFromAbsolutePoseTargetRelativeFirstMJRestore(
        absTarget.slice(),
        currentQ.slice(),
        false,
        true,
        true,
        relCurrent.slice(),
        null,
        {
            jogMetricOverride: 'none'
        }
    );

    assert.equal(result, true);
    assert.equal(enqueued, false);
    assert.ok(Array.isArray(manifoldArgs));
    assert.equal(manifoldArgs[6]?.solverMode, 'mj_restore');
    assert.equal(manifoldArgs[6]?.preferWorker, false);
});


test('RobotVisualizer exposes _computePoseErrorNorm for hierarchical manipulability helpers', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(typeof visualizer._computePoseErrorNorm, 'function');
    assert.equal(visualizer._computePoseErrorNorm([3, 4]), 5);
});

test('RobotVisualizer manifold jog metric refinement computes stepNorm without throwing', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 1];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs ?? Number.NaN;

    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: { value: 1, violation: 0 },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [[1, 0]],
            cctsm: { w_abs: qActive[1] }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits: null,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        stepSize: 0.1
    });

    assert.equal(result.applied, true);
    assert.ok(Number.isFinite(result.nullspaceResult?.stepNorm));
});

test('RobotVisualizer manifold metric refinement can fallback to wvfi gradient', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8,
                violation: 0,
                gradient: [0, 0.2]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [[1, 0]],
            cctsm: { w_abs: qActive[1] }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits: null,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0.5,
        restoreMaxIterations: 2,
        stepSize: 0.1,
        restoreFn: ({ qInitial }) => ({
            converged: true,
            acceptedIterations: 1,
            qApplied: [0, qInitial[1] + 0.2],
            currentProblem: evaluateProblem([0, qInitial[1] + 0.2])
        })
    });

    assert.equal(result.applied, true);
    assert.equal(result.nullspaceResult?.gradientSource, 'wvfi');
    assert.equal(result.state, 'active');
});

test('RobotVisualizer manifold metric refinement can accept a wvfi fallback step when the fallback metric improves', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (metric, cctsm) => cctsm?.[metric] ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[1],
                violation: 0,
                gradient: [0, 1]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_arm_vfi: 1 + qActive[1],
                w_abs_arm: 1 - qActive[1],
                w_abs_hier_vfi: 1 + qActive[1]
            }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_arm_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits: null,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        stepSize: 0.1,
        directionFallbackMetric: 'w_abs_hier_vfi',
        restoreFn: ({ qInitial }) => ({
            converged: true,
            acceptedIterations: 1,
            qApplied: qInitial.slice(),
            currentProblem: evaluateProblem(qInitial)
        })
    });

    assert.equal(result.applied, true);
    assert.equal(result.nullspaceResult?.gradientSource, 'wvfi');
    assert.equal(result.state, 'active');
});

test('RobotVisualizer manifold metric refinement can accept a safety fallback when w_vfi improves and the requested metric regresses only slightly', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (metric, cctsm) => cctsm?.[metric] ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[1],
                violation: 0,
                gradient: [0, 1]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_arm_vfi: 1 - 5e-7 * qActive[1],
                w_abs_arm: 1 - 5e-7 * qActive[1],
                w_abs_hier_vfi: 1 - 5e-7 * qActive[1],
                w_vfi: 1 + qActive[1]
            }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_arm_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits: null,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        stepSize: 0.1,
        directionFallbackMetric: 'w_abs_hier_vfi',
        restoreFn: ({ qInitial }) => ({
            converged: true,
            acceptedIterations: 1,
            qApplied: qInitial.slice(),
            currentProblem: evaluateProblem(qInitial)
        })
    });

    assert.equal(result.applied, true);
    assert.equal(result.nullspaceResult?.gradientSource, 'wvfi');
    assert.equal(result.state, 'active');
});

test('RobotVisualizer manifold metric refinement can bias a hierarchical VFI step away from an active joint limit', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [-1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_arm ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_arm: qActive[0]
            }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0.0015, 0]),
        baseQActive: [0.0015, 0],
        metric: 'w_abs_arm',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        stepSize: 0.1,
        preferActiveLimitRetreat: true,
        restoreFn: ({ qInitial }) => ({
            converged: true,
            acceptedIterations: 1,
            qApplied: qInitial.slice(),
            currentProblem: evaluateProblem(qInitial)
        })
    });

    assert.equal(result.applied, true);
    assert.ok((result.nullspaceResult?.step?.[0] ?? 0) > 0);
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) > 0.0015);
    assert.ok((result.nullspaceResult?.stepNorm ?? 0) > 0.99);
});

test('RobotVisualizer manifold metric refinement can accept an active-limit retreat when VFI distance improves', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [-1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_arm ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_arm: 1 - 1e-5 * qActive[0]
            }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0.0015, 0]),
        baseQActive: [0.0015, 0],
        metric: 'w_abs_arm',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        stepSize: 0.1,
        preferActiveLimitRetreat: true,
        restoreFn: ({ qInitial }) => ({
            converged: true,
            acceptedIterations: 1,
            qApplied: qInitial.slice(),
            currentProblem: evaluateProblem(qInitial)
        })
    });

    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.directionResult?.reason, 'active_limit_retreat');
});

test('RobotVisualizer manifold metric refinement can accept the retreat target directly when the projected candidate already preserves pose', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [-1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_arm ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            weightedResidual: [],
            weightedJacobian: [],
            residual: [],
            jacobian: [],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_arm: qActive[0]
            }
        }
    });

    let wrappedRestoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_arm',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        stepSize: 1e-4,
        preferActiveLimitRetreat: true,
        restoreFn: () => {
            wrappedRestoreCalled = true;
            throw new Error('wrapped_restore_should_not_run_for_active_limit_retreat');
        }
    });

    assert.equal(result.applied, true);
    assert.equal(wrappedRestoreCalled, false);
    assert.equal(result.refineResult?.reason, 'projected_candidate');
    assert.equal(result.refineResult?.iterations, 0);
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) >= 0.03 - 1e-12);
});

test('RobotVisualizer manifold metric refinement continues with restore when the retreat seed still violates pose constraints', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [-1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: Math.abs((qActive[0] ?? 0) + (qActive[1] ?? 0)),
            weightedResidual: [(qActive[0] ?? 0) + (qActive[1] ?? 0)],
            weightedJacobian: [[1, 1]],
            residual: [(qActive[0] ?? 0) + (qActive[1] ?? 0)],
            jacobian: [[1, 1]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [-1, -1],
            effectiveUpper: [1, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [-1, -1],
        upperBounds: [1, 1],
        stepBounds: [1, 1],
        wvfiMin: 0,
        restoreMaxIterations: 2,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 1e-4,
        preferActiveLimitRetreat: true
    });

    assert.notEqual(result.refineResult?.reason, 'retreat_seed');
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) >= 0.03 - 1e-12);
});

test('RobotVisualizer manifold metric refinement accepts a projected nullspace candidate without running restore when pose protection already holds', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: Math.abs(qActive[1] ?? 0),
            weightedResidual: [qActive[1] ?? 0],
            weightedJacobian: [[0, 1]],
            residual: [qActive[1] ?? 0],
            jacobian: [[0, 1]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [0, -1],
            effectiveUpper: [2, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [[0, 1]],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.1, 0.1],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.1,
        preferActiveLimitRetreat: true,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_when_projected_candidate_is_already_pose_preserving');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_candidate');
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) > 0.05);
});

test('RobotVisualizer manifold metric refinement backtracks along the projected nullspace direction before falling back to restore', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: Math.pow(qActive[0] ?? 0, 2),
            weightedResidual: [Math.pow(qActive[0] ?? 0, 2)],
            weightedJacobian: [[0, 0]],
            residual: [Math.pow(qActive[0] ?? 0, 2)],
            jacobian: [[0, 0]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [0, -1],
            effectiveUpper: [2, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.1, 0.1],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 3e-3,
        inequalityTolerance: 1e-6,
        stepSize: 0.1,
        preferActiveLimitRetreat: true,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_when_backtracked_projected_candidate_is_pose_preserving');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_candidate_backtracked');
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) >= 0.05 - 1e-12);
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) < 0.1);
});

test('RobotVisualizer manifold metric refinement can reproject a drifting nullspace candidate back onto the protected pose manifold without restore', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: Math.abs((qActive[0] ?? 0) + (qActive[1] ?? 0)),
            weightedResidual: [(qActive[0] ?? 0) + (qActive[1] ?? 0)],
            weightedJacobian: [[1, 1]],
            residual: [(qActive[0] ?? 0) + (qActive[1] ?? 0)],
            jacobian: [[1, 1]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [0, -1],
            effectiveUpper: [2, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [[1, 1]],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });
    const baseProblem = evaluateProblem([0, 0]);
    baseProblem.equalities.weightedJacobian = [[0, 0]];
    baseProblem.equalities.jacobian = [[0, 0]];

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem,
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.1, 0.1],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.1,
        preferActiveLimitRetreat: true,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_when_projected_correction_is_sufficient');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_correction');
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) > 0);
    assert.ok((result.refineResult?.qApplied?.[1] ?? 0) < 0);
    assert.ok(Math.abs((result.refineResult?.qApplied?.[0] ?? 0) + (result.refineResult?.qApplied?.[1] ?? 0)) <= 1e-6);
});

test('RobotVisualizer projection-only manifold metric refinement returns quickly without calling restore when no strict nullspace progress exists', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: 1,
            weightedResidual: [1],
            weightedJacobian: [[0, 0]],
            residual: [1],
            jacobian: [[0, 0]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [0, -1],
            effectiveUpper: [2, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.1, 0.1],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.002,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_for_projection_only_metric_jog');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, false);
    assert.equal(result.state, 'skipped(projection_only_no_progress)');
});

test('RobotVisualizer projection-only manifold metric refinement can accumulate multiple strict nullspace micro-steps in one button press', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [1, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: 0,
            weightedResidual: [0],
            weightedJacobian: [[0, 1]],
            residual: [0],
            jacobian: [[0, 1]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [0, -1],
            effectiveUpper: [2, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [[0, 1]],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.3, 0.1],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.05,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        projectionPasses: 3,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_for_projection_only_micro_integration');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_integration');
    assert.ok((result.refineResult?.acceptedIterations ?? 0) >= 3);
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) >= 0.15 - 1e-12);
    assert.ok(Math.abs(result.refineResult?.qApplied?.[1] ?? 0) <= 1e-12);
});

test('RobotVisualizer projection-only manifold metric refinement keeps active-limit retreat steps small enough to stay on a curved protected manifold', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -0.001, upper: 0.001 }
    ];
    const evaluateProblem = (qActive) => {
        const q0 = qActive[0] ?? 0;
        const q1 = qActive[1] ?? 0;
        const curvedResidual = q1 - (0.1 * q0 * q0);
        return {
            q: qActive.slice(),
            equalities: {
                activeResidualNorm: Math.abs(curvedResidual),
                weightedResidual: [curvedResidual],
                weightedJacobian: [[-0.2 * q0, 1]],
                residual: [curvedResidual],
                jacobian: [[-0.2 * q0, 1]],
                blocks: {}
            },
            inequalities: {
                wvfiFloor: {
                    value: 0.8 + q0,
                    violation: 0,
                    gradient: [1, 0]
                },
                waistXCorridor: { violation: 0 }
            },
            bounds: {
                effectiveLower: [0, -0.001],
                effectiveUpper: [2, 0.001]
            },
            metadata: {
                qFull: qActive.slice(),
                Jabs: [[1, 0], [0, 1]],
                protectedRows: [[-0.2 * q0, 1]],
                cctsm: {
                    w_abs_vfi: 1 + q0
                }
            }
        };
    };

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -0.001],
        upperBounds: [2, 0.001],
        stepBounds: [0.3, 0.001],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-4,
        inequalityTolerance: 1e-6,
        stepSize: 0.002,
        preferWVFIFallbackOnClip: true,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        projectionPasses: 16,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_for_projection_only_curved_manifold_case');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_integration');
    assert.ok((result.refineResult?.acceptedIterations ?? 0) >= 10);
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) >= 0.02);
    assert.ok(Math.abs((result.refineResult?.qApplied?.[1] ?? 0)) <= 0.001 + 1e-12);
});

test('RobotVisualizer projection-only manifold metric refinement does not blow up active-limit retreat micro-steps', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = () => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: 0,
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => ({
        q: qActive.slice(),
        equalities: {
            activeResidualNorm: 0,
            weightedResidual: [0],
            weightedJacobian: [[0, 1]],
            residual: [0],
            jacobian: [[0, 1]],
            blocks: {}
        },
        inequalities: {
            wvfiFloor: {
                value: 0.8 + qActive[0],
                violation: 0,
                gradient: [1, 0]
            },
            waistXCorridor: { violation: 0 }
        },
        bounds: {
            effectiveLower: [0, -1],
            effectiveUpper: [2, 1]
        },
        metadata: {
            qFull: qActive.slice(),
            Jabs: [[1, 0], [0, 1]],
            protectedRows: [[0, 1]],
            cctsm: {
                w_abs_vfi: 1 + qActive[0]
            }
        }
    });

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.3, 0.1],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.002,
        preferWVFIFallbackOnClip: true,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        projectionPasses: 1,
        restoreFn: () => {
            throw new Error('restore_should_not_run_for_projection_only_micro_step_bound_check');
        }
    });

    assert.ok((result.nullspaceResult?.stepNorm ?? Infinity) <= 0.0025);
});

test('RobotVisualizer projection-only manifold metric refinement can accept a tiny active-limit retreat even when the requested metric regresses slightly', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = (qFull) => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: Math.max(0, qFull[0] ?? 0),
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => {
        const q0 = qActive[0] ?? 0;
        const q1 = qActive[1] ?? 0;
        const restored = q0 >= 0.001 - 1e-12 && Math.abs(q1) <= 1e-12;
        return {
            q: qActive.slice(),
            equalities: {
                activeResidualNorm: restored ? 0 : 1,
                weightedResidual: [restored ? 0 : 1],
                weightedJacobian: [[0, 0]],
                residual: [restored ? 0 : 1],
                jacobian: [[0, 0]],
                blocks: {}
            },
            inequalities: {
                wvfiFloor: {
                    value: 1,
                    violation: 0,
                    gradient: [0, 1]
                },
                waistXCorridor: { violation: 0 }
            },
            bounds: {
                effectiveLower: [0, -1],
                effectiveUpper: [2, 1]
            },
            metadata: {
                qFull: qActive.slice(),
                Jabs: [[1, 0], [0, 1]],
                protectedRows: [[1, 0]],
                cctsm: {
                    w_abs_vfi: restored ? (1.0 - 1e-4 * q0) : 1.0
                }
            }
        };
    };

    let restoreCalled = false;
    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.3, 0.3],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.002,
        preferWVFIFallbackOnClip: true,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        projectionPasses: 1,
        restoreFn: () => {
            restoreCalled = true;
            return {
                converged: true,
                reason: 'converged',
                qApplied: [0.002, 0],
                iterations: 1,
                acceptedIterations: 1,
                lastAcceptedScale: 1
            };
        }
    });

    assert.equal(restoreCalled, true);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projection_retreat_restore');
    assert.ok(Math.abs((result.refineResult?.qApplied?.[0] ?? 0) - 0.002) <= 1e-12);
});

test('RobotVisualizer projection-only manifold metric refinement does not throw when multi-pass integration re-evaluates an active-limit retreat result', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = (qFull) => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: Math.max(0, qFull[0] ?? 0),
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    const evaluateProblem = (qActive) => {
        const q0 = qActive[0] ?? 0;
        const q1 = qActive[1] ?? 0;
        const restored = q0 >= 0.001 - 1e-12 && Math.abs(q1) <= 1e-12;
        return {
            q: qActive.slice(),
            equalities: {
                activeResidualNorm: restored ? 0 : 1,
                weightedResidual: [restored ? 0 : 1],
                weightedJacobian: [[0, 0]],
                residual: [restored ? 0 : 1],
                jacobian: [[0, 0]],
                blocks: {}
            },
            inequalities: {
                wvfiFloor: {
                    value: 1,
                    violation: 0,
                    gradient: [0, 1]
                },
                waistXCorridor: { violation: 0 }
            },
            bounds: {
                effectiveLower: [0, -1],
                effectiveUpper: [2, 1]
            },
            metadata: {
                qFull: qActive.slice(),
                Jabs: [[1, 0], [0, 1]],
                protectedRows: [[1, 0]],
                cctsm: {
                    w_abs_vfi: restored ? (1.0 - 1e-4 * q0) : 1.0
                }
            }
        };
    };

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.3, 0.3],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.002,
        preferWVFIFallbackOnClip: true,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        projectionPasses: 2,
        restoreFn: () => ({
            converged: true,
            reason: 'converged',
            qApplied: [0.002, 0],
            iterations: 1,
            acceptedIterations: 1,
            lastAcceptedScale: 1
        })
    });

    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_integration');
    assert.ok((result.refineResult?.acceptedIterations ?? 0) >= 2);
});

test('RobotVisualizer projection-only manifold metric refinement can preserve pose via projected correction without falling back to restore', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._computeHumanoidJogManifoldMetricGradient = () => [0, 0];
    visualizer._evaluateHumanoidJogManifoldMetricValue = (_metric, cctsm) => cctsm?.w_abs_vfi ?? Number.NaN;
    visualizer._isHumanoidMetricRestoreUsable = (restoreResult) => restoreResult?.converged === true && Array.isArray(restoreResult?.qApplied);
    visualizer._buildManipulabilityVFIConstraints = (qFull) => [{
        source: 'joint_limit',
        row: [1, 0],
        distance: Math.max(0, qFull[0] ?? 0),
        threshold: 0.15
    }];

    const activeLimits = [
        { lower: 0, upper: 2 },
        { lower: -1, upper: 1 }
    ];
    let restoreCalled = false;
    const evaluateProblem = (qActive) => {
        const q0 = qActive[0] ?? 0;
        const q1 = qActive[1] ?? 0;
        const residual = q1 - (0.25 * q0);
        const restored = Math.abs(residual) <= 1e-6;
        return {
            q: qActive.slice(),
            equalities: {
                activeResidualNorm: Math.abs(residual),
                weightedResidual: [residual],
                weightedJacobian: [[-0.25, 1]],
                residual: [residual],
                jacobian: [[-0.25, 1]],
                blocks: {}
            },
            inequalities: {
                wvfiFloor: {
                    value: 1,
                    violation: 0,
                    gradient: [0, 1]
                },
                waistXCorridor: { violation: 0 }
            },
            bounds: {
                effectiveLower: [0, -1],
                effectiveUpper: [2, 1]
            },
            metadata: {
                qFull: qActive.slice(),
                Jabs: [[1, 0], [0, 1]],
                protectedRows: [[-0.25, 1]],
                cctsm: {
                    w_abs_vfi: restored ? (1.0 - 1e-4 * q0) : 1.0
                }
            }
        };
    };

    const result = await visualizer._attemptHumanoidJogManifoldMetricRefinement({
        baseProblem: evaluateProblem([0, 0]),
        baseQActive: [0, 0],
        metric: 'w_abs_hier_vfi',
        direction: 'increase',
        evaluateProblem,
        armStart: 0,
        activeCount: 2,
        activeLimits,
        lowerBounds: [0, -1],
        upperBounds: [2, 1],
        stepBounds: [0.3, 0.3],
        wvfiMin: 0,
        restoreMaxIterations: 8,
        equalityTolerance: 1e-6,
        inequalityTolerance: 1e-6,
        stepSize: 0.002,
        preferWVFIFallbackOnClip: true,
        preferActiveLimitRetreat: true,
        projectionOnly: true,
        projectionPasses: 1,
        restoreFn: () => {
            restoreCalled = true;
            throw new Error('restore_should_not_run_when_retreat_seed_projection_can_preserve_pose');
        }
    });

    assert.equal(restoreCalled, false);
    assert.equal(result.applied, true);
    assert.equal(result.state, 'active');
    assert.equal(result.refineResult?.reason, 'projected_correction');
    assert.ok((result.refineResult?.qApplied?.[0] ?? 0) >= 0.0019);
    assert.ok(Math.abs((result.refineResult?.qApplied?.[1] ?? 0) - 0.25 * (result.refineResult?.qApplied?.[0] ?? 0)) <= 1e-6);
});

test('RobotVisualizer mj_restore distance finalization reuses manipActiveLimits without throwing', () => {
    const visualizer = new RobotVisualizer(null);
    visualizer._writeHumanoidActiveJoints = (_qInitialFull, qActive) => qActive.slice();
    visualizer._clampHumanoidActiveJoints = () => {};
    visualizer._buildManipulabilityVFIConstraints = (_qApplied, _armStart, _activeCount, activeLimits) => [{
        source: 'joint_limit',
        distance: 0.05,
        activeLimits
    }];
    visualizer._computeModeCCTSM = () => ({
        w: 0.1,
        w_vfi: 0.09,
        w_abs: 0.11,
        w_abs_arm: 0.08,
        w_abs_vfi: 0.07,
        w_cmd: 0.05
    });
    visualizer._computeHumanoidActiveJointDistance = () => 0.01;
    visualizer._publishHumanoidJogDiagnostics = () => {};
    visualizer._applyHumanoidJointSolution = () => {};

    const finalProblem = {
        equalities: {
            blocks: {
                command: { residual: [0] },
                absolute: { residual: [0] },
                relative: { residual: [0] }
            }
        },
        inequalities: {
            wvfiFloor: { violation: 0 },
            waistXCorridor: { violation: 0 }
        }
    };

    assert.doesNotThrow(() => {
        visualizer._finalizeHumanoidMJDistanceResult({
            qInitialFull: [0, 0],
            armStart: 0,
            activeCount: 2,
            includeWaist: false,
            enableJointLimitAvoidance: true,
            hardLimits: [{ lower: -1, upper: 1 }, { lower: -1, upper: 1 }],
            manipActiveLimits: [{ lower: -1, upper: 1 }, { lower: -1, upper: 1 }],
            evaluateProblem: () => finalProblem,
            effectiveResult: {
                converged: true,
                qApplied: [0.1, -0.1],
                currentProblem: finalProblem
            },
            restorationResult: {
                reason: 'converged',
                iterations: 1,
                timings: {}
            },
            mjPolishState: 'skipped',
            mjPolishResult: null,
            nullspaceResult: null,
            nullspaceRefineResult: null,
            resolvedJogMetric: {
                metric: 'none'
            },
            jogMetricState: 'skipped(none)',
            jogMetricBefore: Number.NaN,
            jogMetricAfter: Number.NaN,
            jogMetricApplied: false,
            jogMetricAxis: '',
            solverMode: 'mj_restore_pre_w',
            preWState: 'disabled_for_mode',
            preWMetric: 'w_abs_hier',
            preWStart: 'q0',
            referenceModel: {
                absolute: { stiffness: 1, damping: 0.25 },
                relative: { stiffness: 1, damping: 0.35 },
                bodyZ: { stiffness: 0, damping: 0 }
            },
            hardCmdTol: 1e-8,
            hardInvariantTol: 1e-8,
            inequalityTol: 1e-8,
            mjTimingStartMs: 0
        });
    });
    assert.equal(visualizer.lastIKSolveStatus?.solver, 'mj_restore_pre_w');
});

test('RobotVisualizer mj_restore_pre_w distance early failures preserve solver identity in lastIKSolveStatus', async () => {
    const visualizer = new RobotVisualizer(null);

    const result = await visualizer._updateHumanoidJointsFromRelativeDistanceDeltaMJRestore(
        0.01,
        [],
        true,
        false,
        {
            solverMode: 'mj_restore_pre_w',
            preWState: 'disabled_for_mode',
            preWMetric: 'w_abs_hier_vfi',
            preWStart: 'q0'
        }
    );

    assert.equal(result, false);
    assert.equal(visualizer.lastIKSolveStatus?.solver, 'mj_restore_pre_w');
    assert.equal(visualizer.lastIKSolveStatus?.reason, 'invalid_state');
});
