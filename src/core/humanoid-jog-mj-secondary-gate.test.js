import test from 'node:test';
import assert from 'node:assert/strict';

import {
    buildHumanoidJogMJResidualBlock,
    computeHumanoidJogMJResidualStackNorm,
    shouldBuildHumanoidJogMJSecondaryObjective
} from './humanoid-jog-mj-secondary-gate.js';

test('buildHumanoidJogMJResidualBlock computes target-minus-current residuals', () => {
    assert.deepEqual(buildHumanoidJogMJResidualBlock([1, 2, 3], [0.5, 3]), [0.5, -1, 3]);
});

test('computeHumanoidJogMJResidualStackNorm stacks finite residual blocks', () => {
    const norm = computeHumanoidJogMJResidualStackNorm([
        [3, 4],
        [12],
        null,
        [Number.NaN, Number.POSITIVE_INFINITY]
    ]);

    assert.equal(norm, 13);
});

test('shouldBuildHumanoidJogMJSecondaryObjective stays disabled when secondary is not requested', () => {
    assert.equal(shouldBuildHumanoidJogMJSecondaryObjective({
        enabled: false,
        residualBlocks: [[0]],
        activationResidual: 1e-5
    }), false);

    assert.equal(shouldBuildHumanoidJogMJSecondaryObjective({
        enabled: true,
        disabled: true,
        residualBlocks: [[0]],
        activationResidual: 1e-5
    }), false);
});

test('shouldBuildHumanoidJogMJSecondaryObjective waits until primary residual is near-held', () => {
    assert.equal(shouldBuildHumanoidJogMJSecondaryObjective({
        enabled: true,
        residualBlocks: [[1e-3]],
        activationResidual: 1e-5
    }), false);

    assert.equal(shouldBuildHumanoidJogMJSecondaryObjective({
        enabled: true,
        residualBlocks: [[6e-6, 8e-6]],
        activationResidual: 1e-5
    }), true);
});
