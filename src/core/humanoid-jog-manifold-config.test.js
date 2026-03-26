import test from 'node:test';
import assert from 'node:assert/strict';

import { RobotVisualizer } from '../visualizer.js';

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

test('RobotVisualizer sets manifoldJogSecondaryMetric default to none', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.manifoldJogSecondaryMetric, 'none');
});

test('RobotVisualizer sets mj_restore body-z defaults for upright absolute control', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyTolerance, 5e-5);
    assert.equal(visualizer.vfiConfig.mjRestoreHardTolerance, 1e-8);
    assert.equal(visualizer.vfiConfig.mjRestorePracticalCommandTolPose, 5e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreMetricButtonCommandTolerance, 1e-3);
    assert.equal(visualizer.vfiConfig.mjRestoreMetricButtonInvariantTolerance, 1e-4);
    assert.equal(visualizer.vfiConfig.mjRestoreActiveLimitRetreatCommandTolerance, 1e-2);
    assert.equal(visualizer.vfiConfig.mjRestoreActiveLimitRetreatInvariantTolerance, 2e-3);
    assert.equal(visualizer.vfiConfig.mjRestoreRelativeWeight, 1.0);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyVerticalWeight, 4.0);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyReferenceStiffness, 0.6);
    assert.equal(visualizer.vfiConfig.mjRestoreBodyReferenceDamping, 0.45);
});

test('RobotVisualizer sets mj_restore arm-only fallback profile defaults', () => {
    const visualizer = new RobotVisualizer(null);
    assert.equal(visualizer.vfiConfig.mjRestoreMaxStep, 0.05);
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
    visualizer._getHumanoidAbsolutePose = () => new Array(8).fill(0);
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
        assert.equal(called[6]?.solverMode, 'mj_restore');
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

test('RobotVisualizer prewarms the mj_restore worker when the solver mode switches to mj_restore', async () => {
    const visualizer = new RobotVisualizer(null);
    visualizer.humanoidMJWorkerModelSpec = { waistChain: [1] };

    let ensuredSpec = null;
    visualizer._canUseHumanoidJogMJRestoreWorker = () => true;
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

test('RobotVisualizer absolute mj_restore_pre_w resolves the dedicated pre-w metric, runs pre-w first, then restores from q_pre', async () => {
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
    assert.equal(callOrder[1].args[7], null);
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
        assert.deepEqual(restoreArgs[1], q0, state);
    }
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
});
