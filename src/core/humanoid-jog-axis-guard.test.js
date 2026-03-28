import test from 'node:test';
import assert from 'node:assert/strict';

import { evaluateTranslationAxisGuard } from './humanoid-jog-axis-guard.js';

test('evaluateTranslationAxisGuard accepts motion that stays mostly on the commanded axis', () => {
    const result = evaluateTranslationAxisGuard({
        commandDelta: [0.01, 0, 0],
        actualDelta: [0.008, 0.001, 0.001],
        minProgressRatio: 0.05,
        maxOrthogonalRatio: 0.35
    });

    assert.equal(result.violated, false);
    assert.equal(result.reason, null);
});

test('evaluateTranslationAxisGuard rejects motion with excessive orthogonal drift', () => {
    const result = evaluateTranslationAxisGuard({
        commandDelta: [0.01, 0, 0],
        actualDelta: [0.004, 0.005, 0.004],
        minProgressRatio: 0.05,
        maxOrthogonalRatio: 0.35
    });

    assert.equal(result.violated, true);
    assert.equal(result.reason, 'orthogonal_drift');
});

test('evaluateTranslationAxisGuard rejects motion with almost no progress along the commanded axis', () => {
    const result = evaluateTranslationAxisGuard({
        commandDelta: [0.01, 0, 0],
        actualDelta: [0.0002, 0, 0],
        minProgressRatio: 0.05,
        maxOrthogonalRatio: 0.35
    });

    assert.equal(result.violated, true);
    assert.equal(result.reason, 'insufficient_progress');
});

test('evaluateTranslationAxisGuard can measure orthogonal drift against projected progress for stricter axis fidelity', () => {
    const result = evaluateTranslationAxisGuard({
        commandDelta: [0.01, 0, 0],
        actualDelta: [0.003, 0, 0.002],
        minProgressRatio: 0.05,
        maxOrthogonalRatio: 0.35,
        orthogonalRatioBasis: 'progress'
    });

    assert.equal(result.violated, true);
    assert.equal(result.reason, 'orthogonal_drift');
});
