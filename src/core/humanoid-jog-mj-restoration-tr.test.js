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
