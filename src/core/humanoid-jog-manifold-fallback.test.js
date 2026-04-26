import test from 'node:test';
import assert from 'node:assert/strict';

import {
    resolveHumanoidJogManifoldArmOnlyFallback,
    resolveHumanoidJogWaistXCorridorCommandHold
} from './humanoid-jog-manifold-fallback.js';

test('resolveHumanoidJogManifoldArmOnlyFallback retries with arms when waist mode fails without body_z', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: false
    });

    assert.deepEqual(result, {
        shouldFallback: true,
        keepBodyZVertical: false,
        reason: 'arm_only_retry'
    });
});

test('resolveHumanoidJogManifoldArmOnlyFallback retries with arms when body_z is already satisfied', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: true,
        currentBodyResidualNorm: 1e-8,
        initialBodyResidualNorm: 1e-3,
        bodyTolerance: 5e-5
    });

    assert.deepEqual(result, {
        shouldFallback: true,
        keepBodyZVertical: false,
        reason: 'body_already_satisfied'
    });
});

test('resolveHumanoidJogManifoldArmOnlyFallback keeps full-body mode when body_z still needs waist motion', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: true,
        currentBodyResidualNorm: 1e-3,
        initialBodyResidualNorm: 1e-3,
        bodyTolerance: 5e-5
    });

    assert.deepEqual(result, {
        shouldFallback: false,
        keepBodyZVertical: true,
        reason: 'body_requires_waist'
    });
});

test('resolveHumanoidJogManifoldArmOnlyFallback keeps full-body mode near the waist corridor boundary', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: false,
        waistXCorridorEnabled: true,
        waistXPosition: 0.0495,
        waistXMax: 0.05,
        waistXActivationThreshold: 0.01
    });

    assert.deepEqual(result, {
        shouldFallback: false,
        keepBodyZVertical: false,
        reason: 'waist_corridor_requires_waist'
    });
});

test('resolveHumanoidJogManifoldArmOnlyFallback allows arm-only retry when the command pushes farther into the upper waist corridor bound', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: true,
        currentBodyResidualNorm: 1e-3,
        initialBodyResidualNorm: 1e-3,
        bodyTolerance: 5e-5,
        waistXCorridorEnabled: true,
        waistXPosition: 0.0495,
        waistXMax: 0.05,
        waistXActivationThreshold: 0.01,
        commandDelta: [0.005, 0, 0]
    });

    assert.deepEqual(result, {
        shouldFallback: true,
        keepBodyZVertical: false,
        reason: 'waist_corridor_saturated_arm_only'
    });
});

test('resolveHumanoidJogManifoldArmOnlyFallback prefers the current body residual over the initial one', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: true,
        currentBodyResidualNorm: 5e-5,
        initialBodyResidualNorm: 1e-3,
        bodyTolerance: 5e-5
    });

    assert.deepEqual(result, {
        shouldFallback: true,
        keepBodyZVertical: false,
        reason: 'body_already_satisfied'
    });
});

test('resolveHumanoidJogManifoldArmOnlyFallback keeps full-body mode when the command would release the upper waist corridor bound', () => {
    const result = resolveHumanoidJogManifoldArmOnlyFallback({
        includeWaist: true,
        converged: false,
        keepBodyZVertical: false,
        waistXCorridorEnabled: true,
        waistXPosition: 0.0495,
        waistXMax: 0.05,
        waistXActivationThreshold: 0.01,
        commandDelta: [-0.005, 0, 0]
    });

    assert.deepEqual(result, {
        shouldFallback: false,
        keepBodyZVertical: false,
        reason: 'waist_corridor_requires_waist'
    });
});

test('resolveHumanoidJogWaistXCorridorCommandHold holds when x command pushes farther below the lower corridor', () => {
    const result = resolveHumanoidJogWaistXCorridorCommandHold({
        waistXCorridorEnabled: true,
        waistXPosition: -0.4718,
        waistXMin: -0.1,
        waistXMax: 0.08,
        edgeTolerance: 5e-4,
        commandDelta: [-0.005, 0, 0]
    });

    assert.deepEqual(result, {
        hold: true,
        bound: 'lower',
        reason: 'waist_corridor_lower_command_hold'
    });
});

test('resolveHumanoidJogWaistXCorridorCommandHold allows a command that releases the lower corridor', () => {
    const result = resolveHumanoidJogWaistXCorridorCommandHold({
        waistXCorridorEnabled: true,
        waistXPosition: -0.4718,
        waistXMin: -0.1,
        waistXMax: 0.08,
        edgeTolerance: 5e-4,
        commandDelta: [0.005, 0, 0]
    });

    assert.deepEqual(result, {
        hold: false,
        bound: '',
        reason: 'not_pushing_outward'
    });
});

test('resolveHumanoidJogWaistXCorridorCommandHold holds inside the lower edge deadband', () => {
    const result = resolveHumanoidJogWaistXCorridorCommandHold({
        waistXCorridorEnabled: true,
        waistXPosition: -0.0998,
        waistXMin: -0.1,
        waistXMax: 0.08,
        edgeTolerance: 5e-4,
        commandDelta: [-0.005, 0, 0]
    });

    assert.equal(result.hold, true);
    assert.equal(result.bound, 'lower');
});

test('resolveHumanoidJogWaistXCorridorCommandHold holds before an outward x command would cross the lower corridor', () => {
    const result = resolveHumanoidJogWaistXCorridorCommandHold({
        waistXCorridorEnabled: true,
        waistXPosition: -0.097,
        waistXMin: -0.1,
        waistXMax: 0.08,
        edgeTolerance: 5e-4,
        commandDelta: [-0.005, 0, 0]
    });

    assert.equal(result.hold, true);
    assert.equal(result.bound, 'lower');
});

test('resolveHumanoidJogWaistXCorridorCommandHold allows an outward x command that remains inside the lower corridor', () => {
    const result = resolveHumanoidJogWaistXCorridorCommandHold({
        waistXCorridorEnabled: true,
        waistXPosition: -0.09,
        waistXMin: -0.1,
        waistXMax: 0.08,
        edgeTolerance: 5e-4,
        commandDelta: [-0.005, 0, 0]
    });

    assert.equal(result.hold, false);
    assert.equal(result.reason, 'not_pushing_outward');
});
