import test from 'node:test';
import assert from 'node:assert/strict';

import { computeWaistXVFIRows } from './waist-x-vfi.js';

const MockDQ = {
    createFromArray(values) {
        return values;
    },
    getTranslation(values) {
        return [0, values[1], values[2], values[3]];
    }
};

function assertNear(actual, expected, tolerance = 1e-9) {
    assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} not near ${expected}`);
}

test('computeWaistXVFIRows keeps corridor VFI rows active over the full range', () => {
    const result = computeWaistXVFIRows({
        fkmFn: (q) => [1, q[0], 0, 0, 0, 0, 0, 0],
        q: [0],
        waistStart: 0,
        waistCount: 1,
        DQ: MockDQ,
        xMax: 1,
        xMin: -1,
        activationThreshold: 0.1
    });

    assert.deepEqual(result.rows, [[1], [-1]]);
    assert.deepEqual(result.rhs, [1, 1]);
    assert.deepEqual(result.vfiConstraints, [
        { row: [-1], distance: 1, threshold: 0.1 },
        { row: [1], distance: 1, threshold: 0.1 }
    ]);
});

test('computeWaistXVFIRows uses signed distance so violated bounds push back inside', () => {
    const result = computeWaistXVFIRows({
        fkmFn: (q) => [1, q[0], 0, 0, 0, 0, 0, 0],
        q: [1.1],
        waistStart: 0,
        waistCount: 1,
        DQ: MockDQ,
        xMax: 1,
        xMin: -1,
        eta: 2,
        activationThreshold: 0.1
    });

    assertNear(result.rows[0][0], 1);
    assertNear(result.rows[1][0], -1);
    assertNear(result.rhs[0], -0.2);
    assertNear(result.rhs[1], 4.2);
    assertNear(result.vfiConstraints[0].row[0], -1);
    assertNear(result.vfiConstraints[0].distance, 0);
    assertNear(result.vfiConstraints[1].row[0], 1);
    assertNear(result.vfiConstraints[1].distance, 2.1);
    assert.equal(result.vfiConstraints[0].threshold, 0.1);
    assert.equal(result.vfiConstraints[1].threshold, 0.1);
});

test('computeWaistXVFIRows stops inward motion inside the corridor edge deadband', () => {
    const nearUpper = computeWaistXVFIRows({
        fkmFn: (q) => [1, q[0], 0, 0, 0, 0, 0, 0],
        q: [0.9998],
        waistStart: 0,
        waistCount: 1,
        DQ: MockDQ,
        xMax: 1,
        xMin: -1,
        eta: 2,
        stopTolerance: 5e-4
    });

    const outsideUpper = computeWaistXVFIRows({
        fkmFn: (q) => [1, q[0], 0, 0, 0, 0, 0, 0],
        q: [1.0002],
        waistStart: 0,
        waistCount: 1,
        DQ: MockDQ,
        xMax: 1,
        xMin: -1,
        eta: 2,
        stopTolerance: 5e-4
    });

    assertNear(nearUpper.rhs[0], 0);
    assertNear(outsideUpper.rhs[0], -4e-4);
});
