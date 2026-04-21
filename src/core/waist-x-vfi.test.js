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

test('computeWaistXVFIRows exposes corridor distances for CCTSM even when control rows are inactive', () => {
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

    assert.equal(result.rows.length, 0);
    assert.equal(result.rhs.length, 0);
    assert.deepEqual(result.vfiConstraints, [
        { row: [-1], distance: 1, threshold: 0.1 },
        { row: [1], distance: 1, threshold: 0.1 }
    ]);
});
