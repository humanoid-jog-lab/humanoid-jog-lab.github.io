import test from 'node:test';
import assert from 'node:assert/strict';

import { evaluateHumanoidJogManifoldAcceptance } from './humanoid-jog-manifold-acceptance.js';

test('evaluateHumanoidJogManifoldAcceptance keeps relative strict while allowing separate body tolerance', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: true,
        absoluteResidual: [7.14e-10],
        relativeResidual: [3e-13],
        bodyResidual: [4e-8],
        inequalityViolation: 0,
        commandTolerance: 1e-12,
        relativeTolerance: 1e-12,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, false);
    assert.ok(result.absoluteNorm < 1e-9);
    assert.ok(result.relativeNorm < 1e-12);
    assert.ok(result.bodyNorm < 5e-5);
});

test('evaluateHumanoidJogManifoldAcceptance converges when absolute and relative meet 1e-12 and body meets its own tolerance', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: true,
        absoluteResidual: [8e-13],
        relativeResidual: [3e-13],
        bodyResidual: [4e-8],
        inequalityViolation: 0,
        commandTolerance: 1e-12,
        relativeTolerance: 1e-12,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
});

test('evaluateHumanoidJogManifoldAcceptance still rejects when body_z exceeds its own tolerance', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: true,
        absoluteResidual: [8e-13],
        relativeResidual: [3e-13],
        bodyResidual: [8e-5],
        inequalityViolation: 0,
        commandTolerance: 1e-12,
        relativeTolerance: 1e-12,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, false);
});

test('evaluateHumanoidJogManifoldAcceptance accepts a best-effort restore when hard constraints are already satisfied', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: false,
        restorationAccepted: true,
        absoluteResidual: [8e-13],
        relativeResidual: [3e-13],
        bodyResidual: [4e-8],
        inequalityViolation: 0,
        commandTolerance: 1e-12,
        relativeTolerance: 1e-12,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
});

test('evaluateHumanoidJogManifoldAcceptance can use practical active-limit retreat tolerances for metric-only refinements', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: false,
        restorationAccepted: true,
        absoluteResidual: [0.0041],
        relativeResidual: [7.91e-4],
        bodyResidual: [],
        inequalityViolation: 0,
        commandTolerance: 5e-3,
        relativeTolerance: 1e-3,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
});

test('evaluateHumanoidJogManifoldAcceptance can accept practical convergence for MJ restore full-body jogs', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: false,
        restorationAccepted: true,
        absoluteResidual: [1.64e-5],
        relativeResidual: [1.96e-6],
        bodyResidual: [2.23e-5],
        inequalityViolation: 0,
        commandTolerance: 1e-8,
        relativeTolerance: 1e-8,
        practicalCommandTolerance: 1e-4,
        practicalRelativeTolerance: 2.5e-4,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-12
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'practical_converged');
    assert.equal(result.hardSatisfied, false);
    assert.equal(result.practicalSatisfied, true);
});

test('evaluateHumanoidJogManifoldAcceptance accepts time-budget TR+w full-body residuals under invariant tolerance', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: false,
        restorationAccepted: true,
        absoluteResidual: [5.9e-8],
        relativeResidual: [1.36e-6],
        bodyResidual: [7.69e-7],
        inequalityViolation: 0,
        commandTolerance: 1e-8,
        relativeTolerance: 1e-8,
        practicalCommandTolerance: 1e-6,
        practicalRelativeTolerance: 5e-5,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-8
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'practical_converged');
    assert.equal(result.practicalSatisfied, true);
});

test('evaluateHumanoidJogManifoldAcceptance accepts practical TR+w command residuals above the MJ hard floor', () => {
    const result = evaluateHumanoidJogManifoldAcceptance({
        restorationConverged: false,
        restorationAccepted: true,
        absoluteResidual: [1.113e-6],
        relativeResidual: [1.08e-7],
        bodyResidual: [2.835e-6],
        inequalityViolation: 0,
        commandTolerance: 1e-8,
        relativeTolerance: 1e-8,
        practicalCommandTolerance: 1e-4,
        practicalRelativeTolerance: 5e-5,
        bodyTolerance: 5e-5,
        inequalityTolerance: 1e-8
    });

    assert.equal(result.converged, true);
    assert.equal(result.reason, 'practical_converged');
    assert.equal(result.practicalSatisfied, true);
});
