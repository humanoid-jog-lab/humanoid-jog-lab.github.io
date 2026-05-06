import test from 'node:test';
import assert from 'node:assert/strict';

import { restoreHumanoidJogMJ } from './humanoid-jog-mj-restoration.js';
import { restoreHumanoidJogMJTrustRegion } from './humanoid-jog-mj-restoration-tr.js';

function buildTrustRegionSimpleProblem(q) {
    return {
        q: q.slice(),
        equalities: {
            residual: [1 - q[0]],
            jacobian: [[1]],
            residualNorm: Math.abs(1 - q[0]),
            activeResidual: [1 - q[0]],
            activeJacobian: [[1]],
            activeResidualNorm: Math.abs(1 - q[0]),
            weightedResidual: [1 - q[0]],
            weightedJacobian: [[1]],
            blocks: {
                absolute: { residual: [1 - q[0]] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: [-10],
            effectiveUpper: [10]
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

function buildPartiallyInfeasibleProblem(q) {
    const residual = 1 - q[0];
    return {
        q: q.slice(),
        equalities: {
            residual: [residual],
            jacobian: [[1]],
            residualNorm: Math.abs(residual),
            activeResidual: [residual],
            activeJacobian: [[1]],
            activeResidualNorm: Math.abs(residual),
            weightedResidual: [residual],
            weightedJacobian: [[1]],
            blocks: {
                absolute: { residual: [residual] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: {
                enabled: true,
                violation: Math.max(0, 0.5 - q[0])
            },
            waistXCorridor: {
                enabled: false,
                violation: 0,
                vfiRows: [],
                rhs: []
            }
        },
        bounds: {
            effectiveLower: [-10],
            effectiveUpper: [0.1]
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

function buildJointLimitVFIStepLimitedProblem(q) {
    const residual = 1 - q[0];
    return {
        q: q.slice(),
        equalities: {
            residual: [residual],
            jacobian: [[1]],
            residualNorm: Math.abs(residual),
            activeResidual: [residual],
            activeJacobian: [[1]],
            activeResidualNorm: Math.abs(residual),
            weightedResidual: [residual],
            weightedJacobian: [[1]],
            blocks: {
                absolute: { residual: [residual] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            jointLimitVFI: {
                enabled: true,
                rows: [[1]],
                rhs: [0.1],
                count: 1
            },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: [-10],
            effectiveUpper: [10]
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

function buildSafetyPushProblem(q, {
    poseTarget = 0,
    safetyIndex = 1,
    safetyUpper = 1,
    safetyTarget = 0.01,
    width = 2
} = {}) {
    const residual = poseTarget - q[0];
    const safetyRow = new Array(width).fill(0);
    safetyRow[safetyIndex] = -1;
    const safetyDistance = safetyUpper - q[safetyIndex];
    const safetyResidual = Math.max(0, safetyTarget - Math.max(0, safetyDistance));
    return {
        q: q.slice(),
        equalities: {
            residual: [residual],
            jacobian: [Array.from({ length: width }, (_, index) => (index === 0 ? 1 : 0))],
            residualNorm: Math.abs(residual),
            activeResidual: [residual],
            activeJacobian: [Array.from({ length: width }, (_, index) => (index === 0 ? 1 : 0))],
            activeResidualNorm: Math.abs(residual),
            weightedResidual: [residual],
            weightedJacobian: [Array.from({ length: width }, (_, index) => (index === 0 ? 1 : 0))],
            blocks: {
                absolute: { residual: [residual] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            jointLimitVFI: {
                enabled: true,
                rows: [safetyRow.map((value) => -value)],
                rhs: [Math.max(0, safetyDistance)],
                count: 1
            },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: new Array(width).fill(-10),
            effectiveUpper: new Array(width).fill(10)
        },
        safetyObjective: {
            enabled: safetyResidual > 0,
            rows: [safetyRow],
            targets: [safetyResidual],
            residual: [safetyResidual],
            residualNorm: safetyResidual,
            count: safetyResidual > 0 ? 1 : 0
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

function buildConflictingSafetyBiasProblem(q) {
    const poseTarget = 0.997;
    const residual = poseTarget - q[0];
    const safetyDistance = 1 - q[0];
    const safetyResidual = Math.max(0, 0.01 - Math.max(0, safetyDistance));
    return {
        q: q.slice(),
        equalities: {
            residual: [residual],
            jacobian: [[1]],
            residualNorm: Math.abs(residual),
            activeResidual: [residual],
            activeJacobian: [[1]],
            activeResidualNorm: Math.abs(residual),
            weightedResidual: [residual],
            weightedJacobian: [[1]],
            blocks: {
                absolute: { residual: [residual] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: [-10],
            effectiveUpper: [10]
        },
        safetyObjective: {
            enabled: safetyResidual > 0,
            rows: [[-1]],
            targets: [safetyResidual],
            residual: [safetyResidual],
            residualNorm: safetyResidual,
            count: safetyResidual > 0 ? 1 : 0
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

function buildSecondaryBiasProblem(q, { conflictPrimary = false } = {}) {
    const primaryResidual = 1 - q[0];
    const secondaryRow = conflictPrimary ? [-1, 0] : [0, 1];
    const secondaryTarget = conflictPrimary ? 0.5 : 0.1;
    const secondaryMotion = secondaryRow.reduce((sum, value, index) => sum + value * (q[index] || 0), 0);
    const secondaryResidual = Math.max(0, secondaryTarget - secondaryMotion);
    return {
        q: q.slice(),
        equalities: {
            residual: [primaryResidual],
            jacobian: [[1, 0]],
            residualNorm: Math.abs(primaryResidual),
            activeResidual: [primaryResidual],
            activeJacobian: [[1, 0]],
            activeResidualNorm: Math.abs(primaryResidual),
            weightedResidual: [primaryResidual],
            weightedJacobian: [[1, 0]],
            blocks: {
                absolute: { residual: [primaryResidual] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            jointLimitVFI: { enabled: false, rows: [], rhs: [], count: 0 },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: [-10, -10],
            effectiveUpper: [10, 10]
        },
        secondaryObjective: {
            enabled: secondaryResidual > 0,
            name: 'w_vfi',
            rows: [secondaryRow],
            targets: [secondaryResidual],
            residual: [secondaryResidual],
            residualNorm: secondaryResidual,
            count: secondaryResidual > 0 ? 1 : 0
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

test('restoreHumanoidJogMJTrustRegion converges a simple restoration problem', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0],
        evaluateProblem: (q) => buildTrustRegionSimpleProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0] - 1) <= 1e-9);
    assert.ok(Number.isFinite(result.trustRadius));
});

test('restoreHumanoidJogMJTrustRegion can improve a low-priority secondary objective after primary convergence', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildSecondaryBiasProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-9,
        secondaryObjectiveEnabled: true,
        secondaryObjectiveWeight: 1,
        secondaryObjectiveDuringRestore: false,
        secondaryObjectivePostConvergenceEnabled: true,
        initialTrustRadius: 0.2
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0] - 1) <= 1e-9);
    assert.ok(result.qApplied[1] > 0.09);
    assert.equal(result.secondaryStatus, 'active');
    assert.ok(result.secondaryResidualAfter < result.secondaryResidualBefore);
});

test('restoreHumanoidJogMJTrustRegion keeps the primary task dominant over a conflicting secondary objective', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildSecondaryBiasProblem(q, { conflictPrimary: true }),
        maxIterations: 8,
        equalityTolerance: 1e-6,
        secondaryObjectiveEnabled: true,
        secondaryObjectiveWeight: 0.002,
        secondaryObjectiveDuringRestore: true,
        secondaryObjectivePostConvergenceEnabled: false,
        initialTrustRadius: 0.2
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0] - 1) <= 1e-6);
});

test('restoreHumanoidJogMJTrustRegion gates secondary bias until the primary task is nearly held', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildSecondaryBiasProblem(q),
        maxIterations: 1,
        equalityTolerance: 1e-9,
        secondaryObjectiveEnabled: true,
        secondaryObjectiveWeight: 1,
        secondaryObjectiveActivationResidual: 1e-5,
        secondaryObjectiveDuringRestore: true,
        secondaryObjectivePostConvergenceEnabled: false,
        initialTrustRadius: 0.2
    });

    assert.ok(result.qApplied[0] > 0.19);
    assert.ok(Math.abs(result.qApplied[1]) <= 1e-9);
});

test('restoreHumanoidJogMJTrustRegion requests lightweight secondary evaluations for trials', () => {
    const calls = [];
    const evaluateProblem = (q, options = {}) => {
        calls.push({
            role: options.evaluationRole || '',
            computeWVFISecondary: options.computeWVFISecondary,
            computeManipData: options.computeManipData
        });
        return buildSecondaryBiasProblem(q);
    };

    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0],
        evaluateProblem,
        maxIterations: 4,
        equalityTolerance: 1e-9,
        secondaryObjectiveEnabled: true,
        secondaryObjectiveWeight: 1,
        secondaryObjectiveDuringRestore: false,
        secondaryObjectivePostConvergenceEnabled: true,
        initialTrustRadius: 0.2
    });

    const trialCalls = calls.filter((call) => call.role === 'trial');
    assert.equal(result.converged, true);
    assert.ok(trialCalls.length > 0);
    assert.equal(trialCalls.every((call) => call.computeWVFISecondary === false), true);
    assert.equal(trialCalls.every((call) => call.computeManipData === false), true);
    assert.equal(calls.some((call) => call.role === 'current' && call.computeWVFISecondary === true), true);
});

test('restoreHumanoidJogMJTrustRegion accepts a best-effort constraint-improving step that restoreHumanoidJogMJ rejects', () => {
    const classic = restoreHumanoidJogMJ({
        qInitial: [0],
        evaluateProblem: (q) => buildPartiallyInfeasibleProblem(q),
        maxIterations: 4,
        equalityTolerance: 1e-12,
        inequalityTolerance: 1e-12
    });
    const trustRegion = restoreHumanoidJogMJTrustRegion({
        qInitial: [0],
        evaluateProblem: (q) => buildPartiallyInfeasibleProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-12,
        inequalityTolerance: 1e-12,
        initialTrustRadius: 0.2
    });

    assert.equal(classic.converged, false);
    assert.equal(classic.reason, 'line_search_failed');

    assert.equal(trustRegion.converged, false);
    assert.equal(trustRegion.reason, 'best_effort');
    assert.ok(trustRegion.qApplied[0] > classic.qApplied[0] + 1e-9);
    assert.ok(trustRegion.qApplied[0] <= 0.1 + 1e-9);
    assert.ok(trustRegion.bestConstraintViolation < 0.5);
    assert.ok(trustRegion.acceptedIterations >= 1);
    assert.ok(['filter', 'rho', 'best_effort'].includes(trustRegion.acceptedBy));
});

test('restoreHumanoidJogMJTrustRegion respects joint-limit VFI step rows during active-row repair', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0],
        evaluateProblem: (q) => buildJointLimitVFIStepLimitedProblem(q),
        maxIterations: 1,
        equalityTolerance: 1e-12,
        initialTrustRadius: 1
    });

    assert.equal(result.acceptedIterations, 1);
    assert.ok(result.qApplied[0] <= 0.1 + 1e-9);
    assert.ok(result.qApplied[0] > 0);
});

test('restoreHumanoidJogMJTrustRegion can bias the primary restore step away from a limit', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0.995],
        evaluateProblem: (q) => buildSafetyPushProblem(q, { poseTarget: 0.1 }),
        maxIterations: 1,
        equalityTolerance: 1e-9,
        safetyPushEnabled: true,
        safetyPushWeight: 1,
        safetyBiasDuringRestore: true,
        safetyPushPostConvergenceEnabled: false,
        initialTrustRadius: 0.2
    });

    assert.equal(result.acceptedIterations, 1);
    assert.ok(Math.abs(result.qApplied[0] - 0.1) <= 1e-6);
    assert.ok(result.qApplied[1] < 0.995 - 1e-6);
    assert.equal(result.safetyStatus, 'bias');
    assert.ok(result.safetyResidualAfter < result.safetyResidualBefore);
});

test('restoreHumanoidJogMJTrustRegion lets small safety-biased commands finish primary convergence', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0.995],
        evaluateProblem: (q) => buildConflictingSafetyBiasProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-6,
        safetyPushEnabled: true,
        safetyPushWeight: 0.002,
        safetyPushActivationResidual: 1e-5,
        safetyBiasDuringRestore: true,
        safetyPushPostConvergenceEnabled: false,
        initialTrustRadius: 0.2
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0] - 0.997) <= 1e-6);
});

test('restoreHumanoidJogMJTrustRegion stops instead of overrunning a synchronous solve budget', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0],
        evaluateProblem: (q) => buildTrustRegionSimpleProblem(q),
        maxIterations: 100,
        maxSolveMs: 0,
        nowMs: () => 0,
        equalityTolerance: 1e-12
    });

    assert.equal(result.converged, false);
    assert.equal(result.reason, 'time_budget_exceeded');
    assert.equal(result.timeBudgetExceeded, true);
    assert.equal(result.iterations, 1);
    assert.deepEqual(result.qApplied, [0]);
});

test('restoreHumanoidJogMJTrustRegion pushes away from a joint limit through nullspace after pose convergence', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0.995],
        evaluateProblem: (q) => buildSafetyPushProblem(q),
        maxIterations: 4,
        equalityTolerance: 1e-9,
        safetyPushEnabled: true,
        safetyPushWeight: 1,
        safetyPushActivationResidual: 1e-5,
        safetyPushPoseHoldTolerance: 1e-9,
        initialTrustRadius: 0.1
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(result.qApplied[1] < 0.995 - 1e-6);
    assert.ok(Math.abs(result.qApplied[0]) <= 1e-9);
    assert.equal(result.safetyStatus, 'active');
    assert.ok(result.safetyResidualAfter < result.safetyResidualBefore);
});

test('restoreHumanoidJogMJTrustRegion treats unavailable safety nullspace as converged instead of unreachable', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0.995],
        evaluateProblem: (q) => buildSafetyPushProblem(q, { width: 1, safetyIndex: 0, poseTarget: 0.995 }),
        maxIterations: 4,
        equalityTolerance: 1e-9,
        safetyPushEnabled: true,
        safetyPushWeight: 1,
        safetyPushActivationResidual: 1e-5,
        safetyPushPoseHoldTolerance: 1e-9,
        initialTrustRadius: 0.1
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'safety_limited');
    assert.ok(Math.abs(result.qApplied[0] - 0.995) <= 1e-12);
    assert.equal(result.safetyStatus, 'limited');
});

test('restoreHumanoidJogMJTrustRegion rejects safety steps that exceed the pose hold tolerance', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0.995],
        evaluateProblem: (q) => buildSafetyPushProblem(q, { width: 1, safetyIndex: 0, poseTarget: 0.995 }),
        maxIterations: 2,
        equalityTolerance: 1e-9,
        safetyPushEnabled: true,
        safetyPushWeight: 1,
        safetyPushActivationResidual: 1e-5,
        safetyPushPoseHoldTolerance: 1e-12,
        initialTrustRadius: 0.1
    });

    assert.equal(result.converged, true);
    assert.equal(result.safetyStatus, 'limited');
    assert.ok(Math.abs(result.qApplied[0] - 0.995) <= 1e-12);
});

test('restoreHumanoidJogMJTrustRegion does not let safety push preempt an unconverged primary task', () => {
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: [0, 0.995],
        evaluateProblem: (q) => buildSafetyPushProblem(q, { poseTarget: 0.1 }),
        maxIterations: 1,
        equalityTolerance: 1e-9,
        safetyPushEnabled: true,
        safetyPushWeight: 1,
        safetyPushActivationResidual: 1e-5,
        safetyPushPoseHoldTolerance: 1e-9,
        initialTrustRadius: 0.1
    });

    assert.ok(result.qApplied[0] > 0.05);
    assert.ok(Math.abs(result.qApplied[1] - 0.995) <= 1e-9);
    assert.equal(result.safetyStatus, 'skipped');
});
