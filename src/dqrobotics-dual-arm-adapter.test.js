import test from 'node:test';
import assert from 'node:assert/strict';

import { HumanoidDualArmAdapter } from './dqrobotics.js';

function buildHumanoidStub(overrides = {}) {
    return {
        getDimConfigurationSpace: () => 20,
        fkmEffector: () => {
            throw new Error('fkmEffector should not be used in fail-fast tests');
        },
        ...overrides
    };
}

test('HumanoidDualArmAdapter fails fast when cooperative dual-arm WASM pose API is unavailable', () => {
    const adapter = new HumanoidDualArmAdapter(buildHumanoidStub());

    assert.throws(
        () => adapter.getAbsolutePose(new Array(20).fill(0)),
        /cooperative dual-arm WASM API unavailable/
    );
    assert.throws(
        () => adapter.getRelativePose(new Array(20).fill(0)),
        /cooperative dual-arm WASM API unavailable/
    );
});

test('HumanoidDualArmAdapter rejects wrong-size joint vectors before calling WASM', () => {
    let wasmCalled = false;
    const adapter = new HumanoidDualArmAdapter(buildHumanoidStub({
        absolutePoseDualArm: () => {
            wasmCalled = true;
            throw new Error('WASM should not be called');
        },
        relativePoseDualArm: () => new Array(8).fill(0),
        absolutePoseJacobianDualArm: () => [],
        relativePoseJacobianDualArm: () => []
    }));

    assert.throws(
        () => adapter.getAbsolutePose(new Array(18).fill(0)),
        /joint angle vector length mismatch/
    );
    assert.equal(wasmCalled, false);
});

test('HumanoidDualArmAdapter fails fast when cooperative dual-arm WASM jacobian API is unavailable', () => {
    const adapter = new HumanoidDualArmAdapter(buildHumanoidStub({
        absolutePoseDualArm: () => new Array(8).fill(0),
        relativePoseDualArm: () => new Array(8).fill(0)
    }));

    assert.throws(
        () => adapter.getAbsolutePoseJacobian(new Array(20).fill(0)),
        /cooperative dual-arm WASM API unavailable/
    );
    assert.throws(
        () => adapter.getRelativePoseJacobian(new Array(20).fill(0)),
        /cooperative dual-arm WASM API unavailable/
    );
});

test('HumanoidDualArmAdapter uses cooperative dual-arm WASM APIs when all required methods exist', () => {
    const absolutePose = [1, 0, 0, 0, 0, 1, 2, 3];
    const relativePose = [1, 0, 0, 0, 0, 0.1, 0.2, 0.3];
    const absoluteJacobian = [[1, 0], [0, 1]];
    const relativeJacobian = [[0.5, 0], [0, 0.25]];
    const adapter = new HumanoidDualArmAdapter(buildHumanoidStub({
        getDimConfigurationSpace: () => 2,
        absolutePoseDualArm: () => absolutePose,
        relativePoseDualArm: () => relativePose,
        absolutePoseJacobianDualArm: () => absoluteJacobian,
        relativePoseJacobianDualArm: () => relativeJacobian
    }));

    assert.equal(adapter.getAbsolutePose([0, 0]), absolutePose);
    assert.equal(adapter.getRelativePose([0, 0]), relativePose);
    assert.equal(adapter.getAbsolutePoseJacobian([0, 0]), absoluteJacobian);
    assert.equal(adapter.getRelativePoseJacobian([0, 0]), relativeJacobian);
});
