import test from 'node:test';
import assert from 'node:assert/strict';

import { buildHumanoidJogOfflinePlotSvg } from './humanoid-jog-offline-plot.js';

test('buildHumanoidJogOfflinePlotSvg renders an SVG from offline sampler data', () => {
    const svg = buildHumanoidJogOfflinePlotSvg({
        meta: {
            command: {
                mode: 'absolute-target',
                solverMode: 'mj_restore_tr_safe',
                axis: 'x',
                direction: 'positive'
            }
        },
        samples: [
            {
                index: 1,
                err: { cmd: 1e-4, inv: 2e-5 },
                step: 0.01,
                dmin: 0.2,
                waist_x: { p0x: -0.2, dPlus: 0.3, dMinus: 0.1 },
                timingMs: { solve: 1.2 },
                q: [0.1, 0.2]
            },
            {
                index: 2,
                err: { cmd: 5e-5, inv: 1e-5 },
                step: 0.008,
                dmin: 0.18,
                waist_x: { p0x: -0.19, dPlus: 0.29, dMinus: 0.11 },
                timingMs: { solve: 1.0 },
                q: [0.12, 0.19]
            }
        ]
    }, {
        title: 'Offline jog sample',
        jointIndices: [0, 1]
    });

    assert.match(svg, /^<svg[^>]+>/);
    assert.match(svg, /Offline jog sample/);
    assert.match(svg, /err_cmd/);
    assert.match(svg, /waist_x/);
    assert.match(svg, /q\[0\]/);
    assert.doesNotMatch(svg, /Playwright|chromium|canvas/);
});
