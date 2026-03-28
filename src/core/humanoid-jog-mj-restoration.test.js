import test from 'node:test';
import assert from 'node:assert/strict';

import { restoreHumanoidJogMJ } from './humanoid-jog-mj-restoration.js';

function buildSimpleProblem(q) {
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
            dampingRows: [0.25]
        }
    };
}

test('restoreHumanoidJogMJ converges a simple reference restoration problem', () => {
    const result = restoreHumanoidJogMJ({
        qInitial: [0],
        evaluateProblem: (q) => buildSimpleProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-9
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'converged');
    assert.ok(Math.abs(result.qApplied[0] - 1) <= 1e-9);
});

test('restoreHumanoidJogMJ exposes reference motion diagnostics', () => {
    const result = restoreHumanoidJogMJ({
        qInitial: [0],
        evaluateProblem: (q) => buildSimpleProblem(q),
        maxIterations: 2,
        equalityTolerance: 1e-12
    });

    assert.ok(Array.isArray(result.lastReferenceResidual));
    assert.equal(result.lastReferenceResidual.length, 1);
    assert.ok(Array.isArray(result.lastConstraintMotion));
});

function buildCorridorConstrainedProblem(q) {
    const residual = [1 - (q[0] + q[1])];
    return {
        q: q.slice(),
        equalities: {
            residual,
            jacobian: [[1, 1]],
            residualNorm: Math.abs(residual[0]),
            activeResidual: residual.slice(),
            activeJacobian: [[1, 1]],
            activeResidualNorm: Math.abs(residual[0]),
            weightedResidual: residual.slice(),
            weightedJacobian: [[1, 1]],
            blocks: {
                absolute: { residual: residual.slice() },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            waistXCorridor: {
                enabled: true,
                violation: Math.max(0, q[0]),
                vfiRows: [[1, 0]],
                rhs: [0]
            }
        },
        bounds: {
            effectiveLower: [-10, -10],
            effectiveUpper: [10, 10]
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

test('restoreHumanoidJogMJ uses active waist corridor rows to choose an alternative feasible direction', () => {
    const result = restoreHumanoidJogMJ({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildCorridorConstrainedProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-9,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.qApplied[0]) <= 1e-9);
    assert.ok(Math.abs(result.qApplied[1] - 1) <= 1e-6);
});

function buildNonBindingCorridorProblem(q) {
    const residual = [1 - q[1]];
    return {
        q: q.slice(),
        equalities: {
            residual,
            jacobian: [[0, 1]],
            residualNorm: Math.abs(residual[0]),
            activeResidual: residual.slice(),
            activeJacobian: [[0, 1]],
            activeResidualNorm: Math.abs(residual[0]),
            weightedResidual: residual.slice(),
            weightedJacobian: [[0, 1]],
            blocks: {
                absolute: { residual: residual.slice() },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            waistXCorridor: {
                enabled: true,
                violation: 0,
                vfiRows: [[1, 0]],
                rhs: [0.1]
            }
        },
        bounds: {
            effectiveLower: [-10, -10],
            effectiveUpper: [10, 10]
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

test('restoreHumanoidJogMJ does not consume corridor budget when the unconstrained step is already feasible', () => {
    const result = restoreHumanoidJogMJ({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildNonBindingCorridorProblem(q),
        maxIterations: 8,
        equalityTolerance: 1e-9,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
    assert.ok(Math.abs(result.qApplied[0]) <= 1e-9);
    assert.ok(Math.abs(result.qApplied[1] - 1) <= 1e-6);
});

function buildAdaptiveDampingProblem(q) {
    const scalarResidual = 1 + q[0] + q[1];
    const weightedBaseResidual = [-0.4911361178387631, -0.4995658220317402];
    const weightedJacobian = [
        [1.5085912793723901, 2.3422546837186253],
        [0, 0.6025506439985938]
    ];
    const weightedResidual = weightedBaseResidual.map((value) => value * scalarResidual);
    return {
        q: q.slice(),
        equalities: {
            residual: [scalarResidual],
            jacobian: [[1, 1]],
            residualNorm: Math.abs(scalarResidual),
            activeResidual: [scalarResidual],
            activeJacobian: [[1, 1]],
            activeResidualNorm: Math.abs(scalarResidual),
            weightedResidual,
            weightedJacobian,
            blocks: {
                absolute: { residual: [scalarResidual] },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: [-10, -10],
            effectiveUpper: [10, 10]
        },
        referenceModel: {
            stiffnessRows: [1.0, 1.0],
            dampingRows: [0, 0]
        }
    };
}

test('restoreHumanoidJogMJ retries with stronger damping when the initial step direction is not acceptable', () => {
    const result = restoreHumanoidJogMJ({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildAdaptiveDampingProblem(q),
        maxIterations: 8,
        damping: 1e-6,
        equalityTolerance: 1e-9,
        inequalityTolerance: 1e-12
    });

    assert.ok(result.acceptedIterations >= 1);
    assert.notEqual(result.reason, 'line_search_failed');
    assert.ok(Number.isFinite(result.lastAcceptedDamping));
    assert.ok(result.lastAcceptedDamping >= 0.1);
});

function buildRedundantPreferenceProblem(q) {
    const residual = [1 - (q[0] + q[1])];
    return {
        q: q.slice(),
        equalities: {
            residual,
            jacobian: [[1, 1]],
            residualNorm: Math.abs(residual[0]),
            activeResidual: residual.slice(),
            activeJacobian: [[1, 1]],
            activeResidualNorm: Math.abs(residual[0]),
            weightedResidual: residual.slice(),
            weightedJacobian: [[1, 1]],
            blocks: {
                absolute: { residual: residual.slice() },
                relative: { residual: [] },
                bodyZ: null
            }
        },
        inequalities: {
            wvfiFloor: { enabled: false, violation: 0 },
            waistXCorridor: { enabled: false, violation: 0, vfiRows: [], rhs: [] }
        },
        bounds: {
            effectiveLower: [-10, -10],
            effectiveUpper: [10, 10]
        },
        referenceModel: {
            stiffnessRows: [1.0],
            dampingRows: [0]
        }
    };
}

test('restoreHumanoidJogMJ can prefer lower-regularized joints in redundant restoration problems', () => {
    const result = restoreHumanoidJogMJ({
        qInitial: [0, 0],
        evaluateProblem: (q) => buildRedundantPreferenceProblem(q),
        maxIterations: 1,
        damping: 1,
        regularizationWeights: [8, 1],
        equalityTolerance: 1e-12,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.acceptedIterations, 1);
    assert.ok(
        result.qApplied[1] > result.qApplied[0] + 0.2,
        `expected lower-regularized joint to move more, got q=${JSON.stringify(result.qApplied)}`
    );
});
