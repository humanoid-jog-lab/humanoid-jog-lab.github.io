import test from 'node:test';
import assert from 'node:assert/strict';

import { createAnalyticCylindricalRuledSurface } from './surface-slide-primitive.js';
import { buildPrintableSurfaceShellObj } from './surface-print-export.js';

test('buildPrintableSurfaceShellObj exports a watertight printable shell from the current surface patch', () => {
    const surface = createAnalyticCylindricalRuledSurface({
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        radius: 0.12,
        spanV: 0.54,
        angleSpan: 0.3 / 0.12,
        sourceName: 'cylindrical-patch'
    });

    const exported = buildPrintableSurfaceShellObj(surface, {
        thicknessMetres: 0.008,
        segmentsU: 24,
        segmentsV: 10,
        objectName: 'cooperative_surface_current'
    });

    assert.equal(exported.watertight, true);
    assert.equal(exported.nonManifoldEdgeCount, 0);
    assert.match(exported.objText, /^# Printable ruled-surface shell/m);
    assert.match(exported.objText, /^o cooperative_surface_current/m);
    assert.ok(exported.vertexCount > 0);
    assert.ok(exported.triangleCount > 0);
    assert.ok(exported.metadata.axialSpan > 500, `expected axial span in mm, got ${exported.metadata.axialSpan}`);
    assert.ok(exported.metadata.arcLength > 250, `expected arc length in mm, got ${exported.metadata.arcLength}`);
    assert.ok(exported.metadata.thickness >= 8, `expected shell thickness in mm, got ${exported.metadata.thickness}`);
});
