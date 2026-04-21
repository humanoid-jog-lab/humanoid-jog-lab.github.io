import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

import {
    evaluateSurfacePatch,
    buildSurfaceAttachedFrame,
    buildSurfaceAttachedPoseTarget,
    buildSurfaceAttachedPoseJacobian,
    buildPluckerTube,
    blendUnitQuaternions,
    interpolatePosesScLERP,
    createAnalyticCylindricalRuledSurface,
    createDQMotionRuledSurface,
    computePointToPluckerLineSquaredDistance,
    computePointToPluckerLineDistanceJacobian,
    computePointToPluckerTubeSignedSquaredDistance,
    computePointToPluckerTubeSignedSquaredDistanceJacobian,
    samplePluckerTubeGuideSegments,
    parseStructuredObjRuledSurfaceGrid,
    resampleStructuredObjRuledSurfaceGrid,
    createStructuredObjRuledSurfaceFromGrid,
    createCylindricalStructuredObjPointTransform,
    advanceSurfaceParameters,
    advanceSurfaceNormalOffset
} from './surface-slide-primitive.js';

const ROOT = resolve(import.meta.dirname, '../..');

function assertApprox(actual, expected, tolerance = 1e-12) {
    assert.ok(Math.abs(actual - expected) <= tolerance, `expected ${expected}, got ${actual}`);
}

function assertApproxVector(actual, expected, tolerance = 1e-12) {
    assert.equal(actual.length, expected.length);
    for (let i = 0; i < actual.length; i++) {
        assertApprox(actual[i], expected[i], tolerance);
    }
}

function normalizeTestVector(values, label = 'vector') {
    const norm = Math.hypot(...values);
    assert.ok(norm > 1e-12, `expected non-degenerate ${label}`);
    return values.map((value) => value / norm);
}

test('evaluateSurfacePatch evaluates an analytic parameter surface and preserves parameters', () => {
    const surface = {
        position: (u, v) => [u, v, u * u + 0.5 * v],
        du: (u, _v) => [1, 0, 2 * u],
        dv: (_u, _v) => [0, 1, 0.5]
    };

    const state = evaluateSurfacePatch(surface, 0.25, -0.4);

    assert.equal(state.u, 0.25);
    assert.equal(state.v, -0.4);
    assertApproxVector(state.position, [0.25, -0.4, -0.1375]);
    assertApproxVector(state.du, [1, 0, 0.5]);
    assertApproxVector(state.dv, [0, 1, 0.5]);
});

test('buildSurfaceAttachedFrame orthonormalizes tangents and produces a right-handed frame', () => {
    const frame = buildSurfaceAttachedFrame({
        position: [0, 0, 0],
        du: [2, 0, 0],
        dv: [1, 3, 0]
    });

    assertApproxVector(frame.tangentU, [1, 0, 0]);
    assertApproxVector(frame.tangentV, [0, 1, 0]);
    assertApproxVector(frame.normal, [0, 0, 1]);
    assert.deepEqual(frame.rotationMatrix, [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ]);
    assertApproxVector(frame.quaternion, [1, 0, 0, 0]);
});

test('buildSurfaceAttachedFrame rejects degenerate parameter directions', () => {
    assert.throws(
        () => buildSurfaceAttachedFrame({
            position: [0, 0, 0],
            du: [0, 0, 0],
            dv: [1, 0, 0]
        }),
        /degenerate/i
    );

    assert.throws(
        () => buildSurfaceAttachedFrame({
            position: [0, 0, 0],
            du: [1, 0, 0],
            dv: [2, 0, 0]
        }),
        /degenerate/i
    );
});

test('buildSurfaceAttachedPoseTarget offsets along the surface normal and exposes pose metadata', () => {
    const surface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };

    const target = buildSurfaceAttachedPoseTarget({
        surface,
        u: 0.1,
        v: -0.2,
        normalOffset: 0.05
    });

    assertApproxVector(target.surface.position, [0.1, -0.2, 0]);
    assertApproxVector(target.surface.normal, [0, 0, 1]);
    assertApproxVector(target.position, [0.1, -0.2, 0.05]);
    assertApproxVector(target.quaternion, [1, 0, 0, 0]);
    assert.equal(target.kind, 'surface_attached_pose');
});

test('buildSurfaceAttachedPoseTarget can preserve an explicit orientation instead of the surface frame', () => {
    const surface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };

    const target = buildSurfaceAttachedPoseTarget({
        surface,
        u: 0.15,
        v: 0.25,
        normalOffset: 0.04,
        quaternionOverride: [0, 0, 1, 0]
    });

    assertApproxVector(target.position, [0.15, 0.25, 0.04]);
    assertApproxVector(target.surface.normal, [0, 0, 1]);
    assertApproxVector(target.quaternion, [0, 0, 1, 0]);
});

test('buildSurfaceAttachedPoseTarget follows the surface frame normal directly', () => {
    const surface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, -1, 0]
    };

    const target = buildSurfaceAttachedPoseTarget({
        surface,
        u: 0.2,
        v: -0.1,
        normalOffset: 0.05
    });

    assertApproxVector(target.surface.position, [0.2, -0.1, 0]);
    assertApproxVector(target.surface.normal, [0, 0, -1]);
    assertApproxVector(target.position, [0.2, -0.1, -0.05]);
    assert.equal(target.surface.normalOffset, 0.05);
});

test('buildSurfaceAttachedPoseTarget can offset along the opposite contact side via an explicit normalOffsetSign', () => {
    const surface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };

    const target = buildSurfaceAttachedPoseTarget({
        surface,
        u: 0.2,
        v: -0.1,
        normalOffset: 0.05,
        normalOffsetSign: -1
    });

    assertApproxVector(target.surface.position, [0.2, -0.1, 0]);
    assertApproxVector(target.surface.normal, [0, 0, 1]);
    assertApproxVector(target.position, [0.2, -0.1, -0.05]);
    assert.equal(target.surface.normalOffset, 0.05);
    assert.equal(target.surface.normalOffsetSign, -1);
});

test('buildSurfaceAttachedPoseJacobian returns explicit planar DQ columns for [u, v, n]', () => {
    const surface = {
        position: (u, v) => [u, v, 0],
        du: () => [1, 0, 0],
        dv: () => [0, 1, 0]
    };

    const model = buildSurfaceAttachedPoseJacobian({
        surface,
        u: 0.2,
        v: -0.1,
        normalOffset: 0.05,
        normalOffsetSign: 1,
        bounds: {
            u: [-1, 1],
            v: [-1, 1]
        }
    });

    assert.equal(model.pose.kind, 'surface_attached_pose');
    assert.equal(model.jacobian.length, 8);
    assert.equal(model.jacobian[0].length, 3);
    assertApproxVector(model.dqPose, [1, 0, 0, 0, 0, 0.1, -0.05, 0.025], 1e-9);
    assertApproxVector(model.jacobian.map((row) => row[0]), [0, 0, 0, 0, 0, 0.5, 0, 0], 1e-6);
    assertApproxVector(model.jacobian.map((row) => row[1]), [0, 0, 0, 0, 0, 0, 0.5, 0], 1e-6);
    assertApproxVector(model.jacobian.map((row) => row[2]), [0, 0, 0, 0, 0, 0, 0, 0.5], 1e-6);
});

test('createDQMotionRuledSurface produces a regular DQ-motion-generated ruled patch with analytic tangents', () => {
    const surface = createDQMotionRuledSurface({
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        centerOffset: -0.06,
        spanU: 0.2,
        spanV: 0.1,
        liftAmplitude: 0.03,
        twistAngle: Math.PI / 2
    });

    assert.equal(surface.kind, 'dq_motion_ruled_surface');
    assert.equal(surface.name, 'humanoid-dq-ruled-surface');

    const centerState = evaluateSurfacePatch(surface, 0.5, 0.5);
    const centerFrame = buildSurfaceAttachedFrame(centerState);
    assertApproxVector(centerState.position, [0, 0, -0.0525], 1e-12);
    assertApproxVector(centerState.du, [0.2, 0, 0.03], 1e-12);
    assertApproxVector(centerState.dv, [0, 0.1, 0], 1e-12);
    assertApproxVector(centerFrame.tangentU, [0.9889363528682975, 0, 0.14834045293024462], 1e-12);
    assertApproxVector(centerFrame.tangentV, [0, 1, 0], 1e-12);
    assertApproxVector(centerFrame.normal, [-0.14834045293024462, 0, 0.9889363528682975], 1e-12);
    assertApproxVector(centerFrame.quaternion, [0.9972302524663744, 0, -0.07437622984429393, 0], 1e-12);

    const edgeState = evaluateSurfacePatch(surface, 1, 1);
    const s2 = Math.SQRT1_2;
    assertApproxVector(edgeState.dv, [-0.1 * s2, 0.1 * s2, 0], 1e-12);
});

test('createDQMotionRuledSurface exposes an explicit Plucker line family consistent with the ruled surface', () => {
    const surface = createDQMotionRuledSurface({
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        centerOffset: -0.06,
        spanU: 0.2,
        spanV: 0.1,
        liftAmplitude: 0.03,
        twistAngle: Math.PI / 2
    });

    const line = surface.lineAt(0.5);
    assert.equal(line.kind, 'plucker_line');
    assertApproxVector(line.direction, [0, 1, 0], 1e-12);
    assertApproxVector(line.moment, [0.0525, 0, 0], 1e-12);
    assertApproxVector(line.closestPoint, [0, 0, -0.0525], 1e-12);
    assertApprox(line.referenceCoordinate, 0, 1e-12);

    const pMid = line.pointAtCoordinate(line.referenceCoordinate);
    assertApproxVector(pMid, surface.position(0.5, 0.5), 1e-12);

    const pEdge = line.pointAtCoordinate(line.referenceCoordinate + 0.5 * 0.1);
    assertApproxVector(pEdge, surface.position(0.5, 1), 1e-12);
});

test('createDQMotionRuledSurface can mirror the u-parameterization to flip the surface normal while preserving the v-direction geometry', () => {
    const baseOptions = {
        basePosition: [0, 0, 0],
        baseQuaternion: [0, 0.9356050226785754, 0.3530484974314752, 0],
        centerOffset: -0.05,
        spanU: 0.30,
        spanV: 0.60,
        liftAmplitude: -0.045,
        twistAngle: 0
    };
    const regularSurface = createDQMotionRuledSurface(baseOptions);
    const flippedSurface = createDQMotionRuledSurface({
        ...baseOptions,
        flipU: true
    });

    const regularState = evaluateSurfacePatch(regularSurface, 0.53, 0.0);
    const flippedState = evaluateSurfacePatch(flippedSurface, 0.53, 0.0);
    const regularFrame = buildSurfaceAttachedFrame(regularState);
    const flippedFrame = buildSurfaceAttachedFrame(flippedState);

    assert.ok(regularFrame.normal[2] < -0.9, `expected baseline surface normal to point downward, got ${regularFrame.normal}`);
    assert.ok(flippedFrame.normal[2] > 0.9, `expected mirrored-u surface normal to point upward, got ${flippedFrame.normal}`);
    assertApproxVector(flippedState.dv, regularState.dv, 1e-12);
});

test('createAnalyticCylindricalRuledSurface produces a regular cylindrical ruled patch with analytic tangents', () => {
    const surface = createAnalyticCylindricalRuledSurface({
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        radius: 0.18,
        spanV: 0.5,
        angleSpan: Math.PI / 2,
        centerOffset: -0.03,
        sourceName: 'cylindrical-patch'
    });

    assert.equal(surface.kind, 'analytic_cylindrical_ruled_surface');
    assert.equal(surface.metadata.source, 'cylindrical-patch');
    assert.equal(surface.metadata.parameterization, 'analytic-cylinder');

    const centerState = evaluateSurfacePatch(surface, 0.5, 0.5);
    const centerFrame = buildSurfaceAttachedFrame(centerState);
    assertApproxVector(centerState.position, [0, 0, -0.03], 1e-9);
    assert.ok(centerFrame.normal[2] > 0.999, `expected center normal to point upward, got ${JSON.stringify(centerFrame.normal)}`);

    const line = surface.lineAt(0.5);
    const state0 = evaluateSurfacePatch(surface, 0.5, 0);
    const state1 = evaluateSurfacePatch(surface, 0.5, 1);
    assertApproxVector(line.pointAtCoordinate(line.referenceCoordinate), state0.position, 1e-9);
    assertApproxVector(
        normalizeTestVector(state1.position.map((value, index) => value - state0.position[index]), 'analytic cylindrical span'),
        line.direction,
        1e-9
    );
    assertApproxVector(state0.position, [0, -0.25, -0.03], 1e-9);
    assertApproxVector(state1.position, [0, 0.25, -0.03], 1e-9);
});

test('parseStructuredObjRuledSurfaceGrid and createStructuredObjRuledSurfaceFromGrid adapt the cylindrical patch OBJ into a curved strip with straight rulings', () => {
    const objText = readFileSync(
        resolve(ROOT, 'public/meshes/ruled_surface_examples/cylindrical_patch.obj'),
        'utf8'
    );

    const grid = parseStructuredObjRuledSurfaceGrid(objText, {
        sourceName: 'cylindrical-patch'
    });
    const surface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0]
    });

    assert.equal(grid.rows, 7);
    assert.equal(grid.cols, 13);
    assert.equal(surface.metadata.source, 'cylindrical-patch');

    const line = surface.lineAt(0.5);
    const state0 = evaluateSurfacePatch(surface, 0.5, 0);
    const state1 = evaluateSurfacePatch(surface, 0.5, 1);
    assertApproxVector(line.pointAtCoordinate(line.referenceCoordinate), state0.position, 1e-9);
    const lineDirectionNorm = Math.hypot(line.direction[0], line.direction[1], line.direction[2]);
    assertApprox(lineDirectionNorm, 1, 1e-12);
    assertApproxVector(
        normalizeTestVector(state1.position.map((value, index) => value - state0.position[index]), 'cylindrical strip span'),
        line.direction,
        1e-9
    );

    const midState = evaluateSurfacePatch(surface, 0.5, 0.5);
    assertApprox(midState.position[2], 0, 1e-9);
    assert.ok(Math.abs(midState.position[0]) < 1e-9, `expected cylindrical patch midpoint x≈0, got ${midState.position[0]}`);
    assert.ok(Math.abs(midState.position[1]) < 1e-9, `expected cylindrical patch midpoint y≈0, got ${midState.position[1]}`);
});

test('resampleStructuredObjRuledSurfaceGrid can densify a structured cylindrical patch while preserving its key geometry', () => {
    const objText = readFileSync(
        resolve(ROOT, 'public/meshes/ruled_surface_examples/cylindrical_patch.obj'),
        'utf8'
    );

    const grid = parseStructuredObjRuledSurfaceGrid(objText, {
        sourceName: 'cylindrical-patch'
    });
    const resampled = resampleStructuredObjRuledSurfaceGrid(grid, {
        uSamples: 49,
        vSamples: 9
    });
    const originalSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0]
    });
    const resampledSurface = createStructuredObjRuledSurfaceFromGrid(resampled, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0]
    });

    assert.equal(resampled.rows, 9);
    assert.equal(resampled.cols, 49);
    assert.equal(resampled.points.length, 9);
    assert.equal(resampled.points[0].length, 49);
    assertApproxVector(resampled.points[0][0], grid.points[0][0], 1e-12);
    assertApproxVector(resampled.points[resampled.rows - 1][resampled.cols - 1], grid.points[grid.rows - 1][grid.cols - 1], 1e-12);
    assertApproxVector(resampledSurface.position(0.5, 0.5), originalSurface.position(0.5, 0.5), 1e-9);
    assertApproxVector(resampledSurface.position(0, 0.5), originalSurface.position(0, 0.5), 1e-9);
    assertApproxVector(resampledSurface.position(1, 0.5), originalSurface.position(1, 0.5), 1e-9);
});

test('createStructuredObjRuledSurfaceFromGrid can smooth the u-direction line family with cubic Hermite interpolation', () => {
    const grid = {
        kind: 'structured_obj_ruled_grid',
        source: 'smooth-u-test',
        rows: 2,
        cols: 5,
        uValues: [0, 0.25, 0.5, 0.75, 1],
        vValues: [0, 1],
        points: [
            [[-1.0, 0, 0], [-0.5, 0, 0.2], [0, 0, 0], [0.5, 0, -0.2], [1.0, 0, 0]],
            [[-1.0, 1, 0], [-0.5, 1, 0.2], [0, 1, 0], [0.5, 1, -0.2], [1.0, 1, 0]]
        ]
    };
    const linearSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        uInterpolation: 'linear'
    });
    const smoothSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        uInterpolation: 'cubic-hermite'
    });

    assert.equal(smoothSurface.metadata.uInterpolation, 'cubic-hermite');
    assert.equal(linearSurface.metadata.uInterpolation, 'linear');
    assertApproxVector(smoothSurface.position(0, 0.5), linearSurface.position(0, 0.5), 1e-12);
    assertApproxVector(smoothSurface.position(0.5, 0.5), linearSurface.position(0.5, 0.5), 1e-12);
    assertApproxVector(smoothSurface.position(1, 0.5), linearSurface.position(1, 0.5), 1e-12);

    const linearLeftDerivative = linearSurface.du(0.249, 0.5);
    const linearRightDerivative = linearSurface.du(0.251, 0.5);
    const smoothLeftDerivative = smoothSurface.du(0.249, 0.5);
    const smoothRightDerivative = smoothSurface.du(0.251, 0.5);
    const linearJump = Math.abs(linearLeftDerivative[2] - linearRightDerivative[2]);
    const smoothJump = Math.abs(smoothLeftDerivative[2] - smoothRightDerivative[2]);
    assert.ok(smoothJump < linearJump, `expected cubic Hermite interpolation to reduce derivative discontinuity, got linearJump=${linearJump} smoothJump=${smoothJump}`);
});

test('createStructuredObjRuledSurfaceFromGrid can uniformly shrink a mesh-backed ruled surface around its local center', () => {
    const grid = {
        kind: 'structured_obj_ruled_grid',
        source: 'scaled-test',
        rows: 2,
        cols: 2,
        uValues: [0, 1],
        vValues: [0, 1],
        points: [
            [[-0.2, -0.1, 0], [0.2, -0.1, 0]],
            [[-0.2, 0.1, 0], [0.2, 0.1, 0]]
        ]
    };
    const fullSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        meshScale: 1
    });
    const scaledSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        meshScale: 0.5
    });

    const fullStart = fullSurface.position(0.5, 0);
    const fullEnd = fullSurface.position(0.5, 1);
    const scaledStart = scaledSurface.position(0.5, 0);
    const scaledEnd = scaledSurface.position(0.5, 1);

    assertApproxVector(fullSurface.position(0.5, 0.5), [0, 0, 0], 1e-12);
    assertApproxVector(scaledSurface.position(0.5, 0.5), [0, 0, 0], 1e-12);
    assertApproxVector(scaledStart, fullStart.map((value) => value * 0.5), 1e-12);
    assertApproxVector(scaledEnd, fullEnd.map((value) => value * 0.5), 1e-12);
    assertApprox(scaledSurface.metadata.meshScale, 0.5, 1e-12);
});

test('createCylindricalStructuredObjPointTransform can make the cylinder smaller while increasing its visible curvature', () => {
    const objText = readFileSync(
        resolve(ROOT, 'public/meshes/ruled_surface_examples/cylindrical_patch.obj'),
        'utf8'
    );

    const grid = parseStructuredObjRuledSurfaceGrid(objText, {
        sourceName: 'cylindrical-patch'
    });
    const originalSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0]
    });
    const transformedSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        gridPointTransform: createCylindricalStructuredObjPointTransform(grid, {
            radiusScale: 0.72,
            angleScale: 1.43,
            axialScale: 1
        })
    });

    const originalEdge = originalSurface.position(0, 0.5);
    const transformedEdge = transformedSurface.position(0, 0.5);
    const originalCenter = originalSurface.position(0.5, 0.5);
    const transformedCenter = transformedSurface.position(0.5, 0.5);

    assertApproxVector(originalCenter, [0, 0, 0], 1e-9);
    assertApproxVector(transformedCenter, [0, 0, 0], 1e-9);
    assert.ok(Math.abs(transformedEdge[0]) < Math.abs(originalEdge[0]), 'expected smaller cylinder radius to reduce lateral extent');
    assert.ok(transformedEdge[1] > originalEdge[1], 'expected larger angular span to increase the visible curvature depth');
});

test('createStructuredObjRuledSurfaceFromGrid can rotate a mesh-backed ruled surface by a mesh-local quaternion before world placement', () => {
    const grid = {
        kind: 'structured_obj_ruled_grid',
        source: 'rotation-test',
        rows: 2,
        cols: 2,
        uValues: [0, 1],
        vValues: [0, 1],
        points: [
            [[-0.1, 0, -0.2], [0.1, 0, -0.2]],
            [[-0.1, 0, 0.2], [0.1, 0, 0.2]]
        ]
    };
    const originalSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0]
    });
    const rotatedSurface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        meshQuaternion: [Math.SQRT1_2, Math.SQRT1_2, 0, 0]
    });

    const originalSpan = originalSurface.position(0.5, 1).map((value, index) => value - originalSurface.position(0.5, 0)[index]);
    const rotatedSpan = rotatedSurface.position(0.5, 1).map((value, index) => value - rotatedSurface.position(0.5, 0)[index]);

    assertApproxVector(normalizeTestVector(originalSpan, 'original span'), [0, 0, 1], 1e-9);
    assertApproxVector(normalizeTestVector(rotatedSpan, 'rotated span'), [0, -1, 0], 1e-9);
});

test('computePointToPluckerLineDistanceJacobian matches finite differences for a fixed line and linear point map', () => {
    const line = {
        direction: [0, 1, 0],
        moment: [0.06, 0, 0]
    };
    const q = [0.1, -0.2];
    const pointOffset = [0.02, 0.01, -0.02];
    const pointJacobian = [
        [1.0, -0.5],
        [0.25, 0.75],
        [0.5, 1.25]
    ];
    const pointAt = (jointValues) => [
        pointOffset[0] + (pointJacobian[0][0] * jointValues[0]) + (pointJacobian[0][1] * jointValues[1]),
        pointOffset[1] + (pointJacobian[1][0] * jointValues[0]) + (pointJacobian[1][1] * jointValues[1]),
        pointOffset[2] + (pointJacobian[2][0] * jointValues[0]) + (pointJacobian[2][1] * jointValues[1])
    ];

    const state = computePointToPluckerLineSquaredDistance(pointAt(q), line);
    const analytic = computePointToPluckerLineDistanceJacobian(pointAt(q), pointJacobian, line);
    const eps = 1e-7;
    const numeric = [];
    for (let col = 0; col < q.length; col++) {
        const qp = q.slice();
        const qm = q.slice();
        qp[col] += eps;
        qm[col] -= eps;
        const dp = computePointToPluckerLineSquaredDistance(pointAt(qp), line).distanceSquared;
        const dm = computePointToPluckerLineSquaredDistance(pointAt(qm), line).distanceSquared;
        numeric.push((dp - dm) / (2 * eps));
    }

    assertApprox(state.distanceSquared, analytic.distanceSquared, 1e-12);
    assertApproxVector(analytic.jacobianRow, numeric, 1e-8);
});

test('buildPluckerTube and signed squared-distance Jacobian match the underlying line geometry', () => {
    const tube = buildPluckerTube({
        kind: 'plucker_line',
        direction: [0, 1, 0],
        moment: [0.06, 0, 0],
        closestPoint: [0, 0, -0.06],
        referenceCoordinate: 0,
        pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
    }, 0.06);
    const q = [0.1, -0.2];
    const pointOffset = [0.02, 0.01, -0.02];
    const pointJacobian = [
        [1.0, -0.5],
        [0.25, 0.75],
        [0.5, 1.25]
    ];
    const pointAt = (jointValues) => [
        pointOffset[0] + (pointJacobian[0][0] * jointValues[0]) + (pointJacobian[0][1] * jointValues[1]),
        pointOffset[1] + (pointJacobian[1][0] * jointValues[0]) + (pointJacobian[1][1] * jointValues[1]),
        pointOffset[2] + (pointJacobian[2][0] * jointValues[0]) + (pointJacobian[2][1] * jointValues[1])
    ];

    assert.equal(tube.kind, 'plucker_tube');
    assertApprox(tube.radius, 0.06, 1e-12);
    assertApprox(tube.radiusSquared, 0.0036, 1e-12);

    const lineState = computePointToPluckerLineSquaredDistance(pointAt(q), tube.line);
    const tubeState = computePointToPluckerTubeSignedSquaredDistance(pointAt(q), tube);
    const analytic = computePointToPluckerTubeSignedSquaredDistanceJacobian(pointAt(q), pointJacobian, tube);
    const eps = 1e-7;
    const numeric = [];
    for (let col = 0; col < q.length; col++) {
        const qp = q.slice();
        const qm = q.slice();
        qp[col] += eps;
        qm[col] -= eps;
        const dp = computePointToPluckerTubeSignedSquaredDistance(pointAt(qp), tube).signedSquaredDistance;
        const dm = computePointToPluckerTubeSignedSquaredDistance(pointAt(qm), tube).signedSquaredDistance;
        numeric.push((dp - dm) / (2 * eps));
    }

    assertApprox(tubeState.signedSquaredDistance, lineState.distanceSquared - tube.radiusSquared, 1e-12);
    assertApprox(analytic.signedSquaredDistance, tubeState.signedSquaredDistance, 1e-12);
    assertApproxVector(analytic.jacobianRow, numeric, 1e-8);
});

test('samplePluckerTubeGuideSegments generates ring and rail segments around a Plucker tube', () => {
    const tube = buildPluckerTube({
        kind: 'plucker_line',
        direction: [0, 1, 0],
        moment: [0.06, 0, 0],
        closestPoint: [0, 0, -0.06],
        referenceCoordinate: 0,
        pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
    }, 0.06);

    const sampled = samplePluckerTubeGuideSegments(tube, {
        coordinateStart: -0.03,
        coordinateEnd: 0.03,
        ringCount: 3,
        radialSegments: 8,
        railAngles: [0, Math.PI / 2],
        frameHint: [0, 0, 1]
    });

    assert.equal(sampled.segmentCount, 28);
    assert.equal(sampled.segments.length, 28);
    assertApproxVector(sampled.basis.radial, [0, 0, 1], 1e-12);
    assertApproxVector(sampled.basis.binormal, [1, 0, 0], 1e-12);

    const firstSegment = sampled.segments[0];
    const ringCenter = [0, -0.03, -0.06];
    const firstRadius = Math.sqrt(
        ((firstSegment.start[0] - ringCenter[0]) ** 2) +
        ((firstSegment.start[1] - ringCenter[1]) ** 2) +
        ((firstSegment.start[2] - ringCenter[2]) ** 2)
    );
    assertApprox(firstRadius, 0.06, 1e-12);
});

test('samplePluckerTubeGuideSegments can generate an open half-tube guide instead of a full closed ring', () => {
    const tube = buildPluckerTube({
        kind: 'plucker_line',
        direction: [0, 1, 0],
        moment: [0.06, 0, 0],
        closestPoint: [0, 0, -0.06],
        referenceCoordinate: 0,
        pointAtCoordinate: (coordinate) => [0, coordinate, -0.06]
    }, 0.06);

    const sampled = samplePluckerTubeGuideSegments(tube, {
        coordinateStart: -0.03,
        coordinateEnd: 0.03,
        ringCount: 3,
        radialSegments: 5,
        railAngles: [-Math.PI / 2, 0, Math.PI / 2],
        frameHint: [0, 0, 1],
        ringAngleStart: -Math.PI / 2,
        ringAngleEnd: Math.PI / 2
    });

    assert.equal(sampled.segmentCount, 18);
    assert.equal(sampled.segments.length, 18);
    assertApproxVector(sampled.basis.radial, [0, 0, 1], 1e-12);
    assertApproxVector(sampled.basis.binormal, [1, 0, 0], 1e-12);

    const firstRingStart = sampled.segments[0].start;
    const lastRingEnd = sampled.segments[3].end;
    assertApproxVector(firstRingStart, [-0.06, -0.03, -0.06], 1e-12);
    assertApproxVector(lastRingEnd, [0.06, -0.03, -0.06], 1e-12);
});

test('blendUnitQuaternions follows the shortest-arc interpolation between unit quaternions', () => {
    const identity = [1, 0, 0, 0];
    const halfTurnX = [0, 1, 0, 0];

    assertApproxVector(blendUnitQuaternions(identity, halfTurnX, 0), identity);
    assertApproxVector(blendUnitQuaternions(identity, halfTurnX, 1), halfTurnX);

    const halfway = blendUnitQuaternions(identity, halfTurnX, 0.5);
    assertApprox(halfway[0], Math.SQRT1_2, 1e-12);
    assertApprox(halfway[1], Math.SQRT1_2, 1e-12);
    assertApprox(halfway[2], 0, 1e-12);
    assertApprox(halfway[3], 0, 1e-12);
});

test('interpolatePosesScLERP follows a quarter-turn screw path about an offset z-axis', () => {
    const pose0 = {
        position: [0, 0, 0],
        quaternion: [1, 0, 0, 0]
    };
    const pose1 = {
        position: [1, -1, 0],
        quaternion: [Math.SQRT1_2, 0, 0, Math.SQRT1_2]
    };

    const halfway = interpolatePosesScLERP(pose0, pose1, 0.5);
    const expectedPosition = [1 - Math.SQRT1_2, -Math.SQRT1_2, 0];

    assertApproxVector(halfway.position, expectedPosition, 1e-9);
    assertApproxVector(
        halfway.quaternion,
        [Math.cos(Math.PI / 8), 0, 0, Math.sin(Math.PI / 8)],
        1e-9
    );
});

test('createStructuredObjRuledSurfaceFromGrid can migrate a cylindrical line family with ScLERP interpolation', () => {
    const objText = readFileSync(
        resolve(ROOT, 'public/meshes/ruled_surface_examples/cylindrical_patch.obj'),
        'utf8'
    );

    const grid = parseStructuredObjRuledSurfaceGrid(objText, {
        sourceName: 'cylindrical-patch'
    });
    const surface = createStructuredObjRuledSurfaceFromGrid(grid, {
        basePosition: [0, 0, 0],
        baseQuaternion: [1, 0, 0, 0],
        uInterpolation: 'sclerp'
    });

    assert.equal(surface.metadata.uInterpolation, 'sclerp');

    const line = surface.lineAt(0.5);
    const state0 = evaluateSurfacePatch(surface, 0.5, 0);
    const state1 = evaluateSurfacePatch(surface, 0.5, 1);
    assertApproxVector(line.pointAtCoordinate(line.referenceCoordinate), state0.position, 1e-9);
    assertApproxVector(
        normalizeTestVector(state1.position.map((value, index) => value - state0.position[index]), 'sclerp cylindrical span'),
        line.direction,
        1e-9
    );
});

test('advanceSurfaceParameters steps and clamps surface coordinates inside bounds', () => {
    const state1 = advanceSurfaceParameters(
        {
            u: 0.95,
            v: 0.1
        },
        {
            parameter: 'u',
            direction: 'positive',
            step: 0.1,
            bounds: {
                u: [0, 1],
                v: [-0.5, 0.5]
            }
        }
    );

    assertApproxVector([state1.u, state1.v], [1, 0.1]);

    const state2 = advanceSurfaceParameters(
        state1,
        {
            parameter: 'v',
            direction: 'negative',
            step: 0.8,
            bounds: {
                u: [0, 1],
                v: [-0.5, 0.5]
            }
        }
    );

    assertApproxVector([state2.u, state2.v], [1, -0.5]);
});

test('advanceSurfaceNormalOffset steps and clamps the attached normal distance', () => {
    const next1 = advanceSurfaceNormalOffset(
        { normalOffset: 0.06 },
        {
            direction: 'positive',
            step: 0.03,
            bounds: [0, 0.1]
        }
    );

    assertApprox(next1, 0.09);

    const next2 = advanceSurfaceNormalOffset(
        { normalOffset: next1 },
        {
            direction: 'positive',
            step: 0.03,
            bounds: [0, 0.1]
        }
    );

    assertApprox(next2, 0.1);

    const next3 = advanceSurfaceNormalOffset(
        { normalOffset: next2 },
        {
            direction: 'negative',
            step: 0.2,
            bounds: [0, 0.1]
        }
    );

    assertApprox(next3, 0);
});
