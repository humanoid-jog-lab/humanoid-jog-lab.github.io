import test from 'node:test';
import assert from 'node:assert/strict';

import {
    shouldAttemptHumanoidJogMJPolish,
    shouldApplyHumanoidJogMJPolish
} from './humanoid-jog-mj-polish.js';

test('shouldAttemptHumanoidJogMJPolish activates only for near-solution non-converged restoration states', () => {
    assert.equal(shouldAttemptHumanoidJogMJPolish({
        restorationResult: { converged: false, reason: 'max_iter', currentResidualNorm: 1e-5 },
        activationResidual: 1e-4
    }), true);

    assert.equal(shouldAttemptHumanoidJogMJPolish({
        restorationResult: { converged: true, reason: 'converged', currentResidualNorm: 1e-10 },
        activationResidual: 1e-4
    }), false);

    assert.equal(shouldAttemptHumanoidJogMJPolish({
        restorationResult: { converged: false, reason: 'max_iter', currentResidualNorm: 1e-2 },
        activationResidual: 1e-4
    }), false);
});

test('shouldAttemptHumanoidJogMJPolish activates for accepted best-effort states that are already near solution', () => {
    assert.equal(shouldAttemptHumanoidJogMJPolish({
        restorationResult: {
            converged: false,
            reason: 'best_effort',
            currentResidualNorm: 3.9e-4,
            acceptedIterations: 10
        },
        activationResidual: 1e-4
    }), true);
});

test('shouldApplyHumanoidJogMJPolish accepts converged or strictly improved polish results', () => {
    assert.equal(shouldApplyHumanoidJogMJPolish({
        restorationResult: { currentResidualNorm: 1e-5 },
        polishResult: { converged: true, currentResidualNorm: 1e-7 }
    }), true);

    assert.equal(shouldApplyHumanoidJogMJPolish({
        restorationResult: { currentResidualNorm: 1e-5 },
        polishResult: { converged: false, currentResidualNorm: 1e-6 }
    }), true);

    assert.equal(shouldApplyHumanoidJogMJPolish({
        restorationResult: { currentResidualNorm: 1e-5 },
        polishResult: { converged: false, currentResidualNorm: 1e-5 }
    }), false);
});
