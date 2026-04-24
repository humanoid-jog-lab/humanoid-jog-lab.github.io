function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function asVector3(value, label) {
    if (!Array.isArray(value) || value.length !== 3) {
        throw new Error(`${label} must be a length-3 array`);
    }
    const out = value.map((entry) => asFinite(entry, Number.NaN));
    if (!out.every(Number.isFinite)) {
        throw new Error(`${label} must contain finite numbers`);
    }
    return out;
}

function dot(a, b) {
    return asFinite(a[0]) * asFinite(b[0]) + asFinite(a[1]) * asFinite(b[1]) + asFinite(a[2]) * asFinite(b[2]);
}

function cross(a, b) {
    return [
        asFinite(a[1]) * asFinite(b[2]) - asFinite(a[2]) * asFinite(b[1]),
        asFinite(a[2]) * asFinite(b[0]) - asFinite(a[0]) * asFinite(b[2]),
        asFinite(a[0]) * asFinite(b[1]) - asFinite(a[1]) * asFinite(b[0])
    ];
}

function norm(a) {
    return Math.sqrt(dot(a, a));
}

function scale(a, s) {
    return [a[0] * s, a[1] * s, a[2] * s];
}

function add(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function lerp(a, b, t) {
    return a + ((b - a) * t);
}

function lerpVector(a, b, t) {
    return [
        lerp(a[0], b[0], t),
        lerp(a[1], b[1], t),
        lerp(a[2], b[2], t)
    ];
}

function subtract(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function normalize(a, label) {
    const n = norm(a);
    if (!Number.isFinite(n) || n <= 1e-12) {
        throw new Error(`degenerate ${label}`);
    }
    return scale(a, 1 / n);
}

function normalizeAny(values, label) {
    const src = Array.isArray(values) ? values.map((value) => asFinite(value, Number.NaN)) : [];
    if (!src.every(Number.isFinite) || src.length === 0) {
        throw new Error(`degenerate ${label}`);
    }
    let sum = 0;
    for (const value of src) sum += value * value;
    const n = Math.sqrt(sum);
    if (!Number.isFinite(n) || n <= 1e-12) {
        throw new Error(`degenerate ${label}`);
    }
    return src.map((value) => value / n);
}

function orthogonalize(vector, reference) {
    return subtract(vector, scale(reference, dot(vector, reference)));
}

function buildPluckerLine(point, direction) {
    const anchor = asVector3(point, 'plucker line point');
    const unitDirection = normalize(direction, 'plucker line direction');
    const moment = cross(anchor, unitDirection);
    const closestPoint = cross(unitDirection, moment);
    const referenceCoordinate = dot(anchor, unitDirection);
    return {
        kind: 'plucker_line',
        point: anchor.slice(),
        direction: unitDirection.slice(),
        moment: moment.slice(),
        closestPoint: closestPoint.slice(),
        referenceCoordinate,
        pointAtCoordinate(coordinate) {
            return add(closestPoint, scale(unitDirection, asFinite(coordinate, 0)));
        }
    };
}

function buildPluckerLineDerivative(point, pointDerivative, line) {
    const anchor = asVector3(point, 'plucker line derivative point');
    const anchorDerivative = asVector3(pointDerivative, 'plucker line derivative point velocity');
    const direction = asVector3(line?.direction, 'plucker line derivative direction');
    const directionDerivative = asVector3(line?.directionDerivative, 'plucker line derivative direction');
    const moment = asVector3(line?.moment, 'plucker line derivative moment');
    const momentDerivative = add(
        cross(anchorDerivative, direction),
        cross(anchor, directionDerivative)
    );
    const closestPointDerivative = add(
        cross(directionDerivative, moment),
        cross(direction, momentDerivative)
    );
    const referenceCoordinateDerivative = dot(anchorDerivative, direction) + dot(anchor, directionDerivative);
    return {
        directionDerivative: directionDerivative.slice(),
        momentDerivative,
        closestPointDerivative,
        referenceCoordinateDerivative
    };
}

export function computePointToPluckerLineSquaredDistance(point, line) {
    const position = asVector3(point, 'plucker distance point');
    const direction = asVector3(line?.direction, 'plucker distance direction');
    const moment = asVector3(line?.moment, 'plucker distance moment');
    const separation = subtract(cross(position, direction), moment);
    const distanceSquared = dot(separation, separation);
    return {
        point: position.slice(),
        direction: direction.slice(),
        moment: moment.slice(),
        separation,
        distanceSquared
    };
}

export function computePointToPluckerLineDistanceJacobian(point, pointJacobian, line) {
    const distanceState = computePointToPluckerLineSquaredDistance(point, line);
    const rows = Array.isArray(pointJacobian) ? pointJacobian : [];
    if (rows.length < 3 || !Array.isArray(rows[0]) || !Array.isArray(rows[1]) || !Array.isArray(rows[2])) {
        throw new Error('pointJacobian must be a 3xN array');
    }
    const width = Math.min(rows[0].length, rows[1].length, rows[2].length);
    const gradientPoint = scale(cross(distanceState.direction, distanceState.separation), 2);
    const jacobianRow = new Array(width).fill(0);
    for (let col = 0; col < width; col++) {
        jacobianRow[col] =
            (gradientPoint[0] * asFinite(rows[0][col])) +
            (gradientPoint[1] * asFinite(rows[1][col])) +
            (gradientPoint[2] * asFinite(rows[2][col]));
    }
    return {
        ...distanceState,
        gradientPoint,
        jacobianRow
    };
}

export function buildPluckerTube(line, radius) {
    if (!line || line.kind !== 'plucker_line') {
        throw new Error('plucker tube expects a plucker_line');
    }
    const resolvedRadius = Math.max(0, asFinite(radius, 0));
    return {
        kind: 'plucker_tube',
        line,
        radius: resolvedRadius,
        radiusSquared: resolvedRadius * resolvedRadius
    };
}

export function computePointToPluckerTubeSignedSquaredDistance(point, tube) {
    if (!tube || tube.kind !== 'plucker_tube') {
        throw new Error('point-to-tube distance expects a plucker_tube');
    }
    const lineState = computePointToPluckerLineSquaredDistance(point, tube.line);
    return {
        ...lineState,
        radius: tube.radius,
        radiusSquared: tube.radiusSquared,
        signedSquaredDistance: lineState.distanceSquared - tube.radiusSquared
    };
}

export function computePointToPluckerTubeSignedSquaredDistanceJacobian(point, pointJacobian, tube) {
    if (!tube || tube.kind !== 'plucker_tube') {
        throw new Error('point-to-tube Jacobian expects a plucker_tube');
    }
    const lineState = computePointToPluckerLineDistanceJacobian(point, pointJacobian, tube.line);
    return {
        ...lineState,
        radius: tube.radius,
        radiusSquared: tube.radiusSquared,
        signedSquaredDistance: lineState.distanceSquared - tube.radiusSquared
    };
}

function resolvePerpendicularBasis(direction, frameHint = null) {
    const lineDirection = normalize(direction, 'plucker tube basis direction');
    const hintCandidate = Array.isArray(frameHint) && frameHint.length === 3
        ? asVector3(frameHint, 'plucker tube basis hint')
        : [0, 0, 1];
    let radial = orthogonalize(hintCandidate, lineDirection);
    if (norm(radial) <= 1e-10) {
        const fallbackCandidates = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ];
        for (const candidate of fallbackCandidates) {
            radial = orthogonalize(candidate, lineDirection);
            if (norm(radial) > 1e-10) break;
        }
    }
    const unitRadial = normalize(radial, 'plucker tube radial basis');
    const unitBinormal = normalize(cross(lineDirection, unitRadial), 'plucker tube binormal basis');
    return {
        radial: unitRadial,
        binormal: unitBinormal
    };
}

export function samplePluckerTubeGuideSegments(tube, {
    coordinateStart = null,
    coordinateEnd = null,
    ringCount = 5,
    radialSegments = 24,
    railAngles = [0, Math.PI / 2, Math.PI, 1.5 * Math.PI],
    frameHint = null,
    ringAngleStart = 0,
    ringAngleEnd = 2 * Math.PI
} = {}) {
    if (!tube || tube.kind !== 'plucker_tube') {
        throw new Error('tube guide sampling expects a plucker_tube');
    }
    const line = tube.line;
    const resolvedRingCount = Math.max(2, Math.floor(asFinite(ringCount, 5)));
    const resolvedRadialSegments = Math.max(3, Math.floor(asFinite(radialSegments, 24)));
    const lineDirection = asVector3(line?.direction, 'plucker tube guide direction');
    const startCoordinate = coordinateStart === null
        ? asFinite(line?.referenceCoordinate, 0) - Math.max(0.04, tube.radius)
        : asFinite(coordinateStart, 0);
    const endCoordinate = coordinateEnd === null
        ? asFinite(line?.referenceCoordinate, 0) + Math.max(0.04, tube.radius)
        : asFinite(coordinateEnd, 0);
    const { radial, binormal } = resolvePerpendicularBasis(lineDirection, frameHint);
    const resolvedRingAngleStart = asFinite(ringAngleStart, 0);
    const resolvedRingAngleEnd = asFinite(ringAngleEnd, 2 * Math.PI);
    const ringAngleSpan = resolvedRingAngleEnd - resolvedRingAngleStart;
    const isClosedRing = Math.abs(Math.abs(ringAngleSpan) - (2 * Math.PI)) <= 1e-6;
    const ringAngles = [];
    if (isClosedRing) {
        for (let i = 0; i < resolvedRadialSegments; i++) {
            ringAngles.push((2 * Math.PI * i) / resolvedRadialSegments);
        }
    } else {
        for (let i = 0; i < resolvedRadialSegments; i++) {
            const alpha = resolvedRadialSegments === 1 ? 0 : (i / (resolvedRadialSegments - 1));
            ringAngles.push(resolvedRingAngleStart + (ringAngleSpan * alpha));
        }
    }
    const railAngleList = Array.isArray(railAngles) && railAngles.length > 0
        ? railAngles.map((angle) => asFinite(angle, 0))
        : [0, Math.PI / 2, Math.PI, 1.5 * Math.PI];
    const ringCoordinates = [];
    const ringPoints = [];
    const segments = [];

    for (let ringIndex = 0; ringIndex < resolvedRingCount; ringIndex++) {
        const alpha = resolvedRingCount === 1 ? 0 : (ringIndex / (resolvedRingCount - 1));
        const coordinate = startCoordinate + ((endCoordinate - startCoordinate) * alpha);
        ringCoordinates.push(coordinate);
        const center = typeof line?.pointAtCoordinate === 'function'
            ? asVector3(line.pointAtCoordinate(coordinate), 'plucker tube guide center')
            : add(asVector3(line?.closestPoint, 'plucker tube guide closest point'), scale(lineDirection, coordinate));
        const ring = ringAngles.map((angle) => add(
            center,
            add(
                scale(radial, tube.radius * Math.cos(angle)),
                scale(binormal, tube.radius * Math.sin(angle))
            )
        ));
        ringPoints.push(ring);
        const ringSegmentCount = isClosedRing ? ring.length : Math.max(0, ring.length - 1);
        for (let i = 0; i < ringSegmentCount; i++) {
            segments.push({
                start: ring[i].slice(),
                end: ring[(i + 1) % ring.length].slice()
            });
        }
    }

    for (const angle of railAngleList) {
        const unit = add(scale(radial, Math.cos(angle)), scale(binormal, Math.sin(angle)));
        for (let ringIndex = 0; ringIndex < ringCoordinates.length - 1; ringIndex++) {
            const start = typeof line?.pointAtCoordinate === 'function'
                ? asVector3(line.pointAtCoordinate(ringCoordinates[ringIndex]), 'plucker tube guide rail start')
                : add(asVector3(line?.closestPoint, 'plucker tube guide closest point'), scale(lineDirection, ringCoordinates[ringIndex]));
            const end = typeof line?.pointAtCoordinate === 'function'
                ? asVector3(line.pointAtCoordinate(ringCoordinates[ringIndex + 1]), 'plucker tube guide rail end')
                : add(asVector3(line?.closestPoint, 'plucker tube guide closest point'), scale(lineDirection, ringCoordinates[ringIndex + 1]));
            segments.push({
                start: add(start, scale(unit, tube.radius)),
                end: add(end, scale(unit, tube.radius))
            });
        }
    }

    return {
        basis: {
            radial,
            binormal
        },
        ringCoordinates,
        ringPoints,
        segments,
        segmentCount: segments.length
    };
}

function quaternionFromRotationMatrix(R) {
    const m00 = R[0][0]; const m01 = R[0][1]; const m02 = R[0][2];
    const m10 = R[1][0]; const m11 = R[1][1]; const m12 = R[1][2];
    const m20 = R[2][0]; const m21 = R[2][1]; const m22 = R[2][2];
    const trace = m00 + m11 + m22;

    let w;
    let x;
    let y;
    let z;

    if (trace > 0) {
        const s = Math.sqrt(trace + 1.0) * 2;
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const s = Math.sqrt(1.0 + m00 - m11 - m22) * 2;
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const s = Math.sqrt(1.0 + m11 - m00 - m22) * 2;
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        const s = Math.sqrt(1.0 + m22 - m00 - m11) * 2;
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }

    return normalizeAny([w, x, y, z], 'quaternion');
}

function multiplyQuaternions(a, b) {
    const qa = Array.isArray(a) ? a.map((value) => asFinite(value, Number.NaN)) : [];
    const qb = Array.isArray(b) ? b.map((value) => asFinite(value, Number.NaN)) : [];
    if (qa.length !== 4 || qb.length !== 4 || !qa.every(Number.isFinite) || !qb.every(Number.isFinite)) {
        throw new Error('quaternion product expects two finite length-4 arrays');
    }
    const [aw, ax, ay, az] = qa;
    const [bw, bx, by, bz] = qb;
    return [
        (aw * bw) - (ax * bx) - (ay * by) - (az * bz),
        (aw * bx) + (ax * bw) + (ay * bz) - (az * by),
        (aw * by) - (ax * bz) + (ay * bw) + (az * bx),
        (aw * bz) + (ax * by) - (ay * bx) + (az * bw)
    ];
}

function conjugateQuaternion(q) {
    const [w, x, y, z] = normalizeAny(q, 'quaternion conjugate');
    return [w, -x, -y, -z];
}

function axisAngleQuaternion(axis, angle) {
    const unitAxis = normalize(axis, 'axis-angle axis');
    const half = 0.5 * asFinite(angle, 0);
    const s = Math.sin(half);
    return normalizeAny([
        Math.cos(half),
        unitAxis[0] * s,
        unitAxis[1] * s,
        unitAxis[2] * s
    ], 'axis-angle quaternion');
}

function rotateVectorByQuaternion(vector, quaternion) {
    const q = normalizeAny(quaternion, 'rotation quaternion');
    const pure = [0, ...asVector3(vector, 'rotation vector')];
    const rotated = multiplyQuaternions(multiplyQuaternions(q, pure), conjugateQuaternion(q));
    return [rotated[1], rotated[2], rotated[3]];
}

function clamp(value, lower, upper) {
    return Math.min(Math.max(value, lower), upper);
}

function buildSkewMatrix(vector) {
    const [x, y, z] = asVector3(vector, 'skew matrix vector');
    return [
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ];
}

function identityMatrix3() {
    return [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ];
}

function scaleMatrix3(matrix, scalar) {
    return matrix.map((row) => row.map((value) => value * scalar));
}

function addMatrix3(a, b) {
    return a.map((row, rowIndex) => row.map((value, colIndex) => value + b[rowIndex][colIndex]));
}

function multiplyMatrix3(a, b) {
    const result = identityMatrix3().map(() => [0, 0, 0]);
    for (let row = 0; row < 3; row++) {
        for (let col = 0; col < 3; col++) {
            result[row][col] =
                (a[row][0] * b[0][col]) +
                (a[row][1] * b[1][col]) +
                (a[row][2] * b[2][col]);
        }
    }
    return result;
}

function multiplyMatrix3Vector(matrix, vector) {
    const v = asVector3(vector, 'matrix-vector input');
    return [
        (matrix[0][0] * v[0]) + (matrix[0][1] * v[1]) + (matrix[0][2] * v[2]),
        (matrix[1][0] * v[0]) + (matrix[1][1] * v[1]) + (matrix[1][2] * v[2]),
        (matrix[2][0] * v[0]) + (matrix[2][1] * v[1]) + (matrix[2][2] * v[2])
    ];
}

function invertMatrix3(matrix) {
    const [
        [a, b, c],
        [d, e, f],
        [g, h, i]
    ] = matrix;
    const A = (e * i) - (f * h);
    const B = -((d * i) - (f * g));
    const C = (d * h) - (e * g);
    const D = -((b * i) - (c * h));
    const E = (a * i) - (c * g);
    const F = -((a * h) - (b * g));
    const G = (b * f) - (c * e);
    const H = -((a * f) - (c * d));
    const I = (a * e) - (b * d);
    const determinant = (a * A) + (b * B) + (c * C);
    if (!Number.isFinite(determinant) || Math.abs(determinant) <= 1e-12) {
        throw new Error('degenerate 3x3 matrix');
    }
    const invDeterminant = 1 / determinant;
    return [
        [A * invDeterminant, D * invDeterminant, G * invDeterminant],
        [B * invDeterminant, E * invDeterminant, H * invDeterminant],
        [C * invDeterminant, F * invDeterminant, I * invDeterminant]
    ];
}

export function interpolatePosesScLERP(fromPose, toPose, alpha = 1) {
    const t = clamp(asFinite(alpha, 1), 0, 1);
    const fromQuaternion = normalizeAny(fromPose?.quaternion, 'ScLERP source quaternion');
    const fromPosition = asVector3(fromPose?.position, 'ScLERP source position');
    let toQuaternion = normalizeAny(toPose?.quaternion, 'ScLERP target quaternion');
    const toPosition = asVector3(toPose?.position, 'ScLERP target position');

    if (dot(fromQuaternion, toQuaternion) < 0) {
        toQuaternion = toQuaternion.map((value) => -value);
    }
    if (t <= 0) {
        return {
            position: fromPosition.slice(),
            quaternion: fromQuaternion.slice()
        };
    }
    if (t >= 1) {
        return {
            position: toPosition.slice(),
            quaternion: toQuaternion.slice()
        };
    }

    const relativeQuaternionRaw = multiplyQuaternions(conjugateQuaternion(fromQuaternion), toQuaternion);
    const relativeQuaternion = relativeQuaternionRaw[0] < 0
        ? relativeQuaternionRaw.map((value) => -value)
        : relativeQuaternionRaw;
    const relativeTranslation = rotateVectorByQuaternion(
        subtract(toPosition, fromPosition),
        conjugateQuaternion(fromQuaternion)
    );
    const relativeVector = [relativeQuaternion[1], relativeQuaternion[2], relativeQuaternion[3]];
    const sinHalfTheta = Math.hypot(...relativeVector);

    if (sinHalfTheta <= 1e-10) {
        return {
            quaternion: fromQuaternion.slice(),
            position: add(fromPosition, scale(subtract(toPosition, fromPosition), t))
        };
    }

    const axis = relativeVector.map((value) => value / sinHalfTheta);
    const theta = 2 * Math.atan2(sinHalfTheta, relativeQuaternion[0]);
    const omega = scale(axis, theta);
    const thetaNorm = Math.hypot(...omega);
    if (!Number.isFinite(thetaNorm) || thetaNorm <= 1e-10) {
        return {
            quaternion: fromQuaternion.slice(),
            position: add(fromPosition, scale(subtract(toPosition, fromPosition), t))
        };
    }

    const omegaHat = buildSkewMatrix(omega);
    const omegaHatSquared = multiplyMatrix3(omegaHat, omegaHat);
    const thetaSquared = thetaNorm * thetaNorm;
    const thetaCubed = thetaSquared * thetaNorm;
    const V = addMatrix3(
        identityMatrix3(),
        addMatrix3(
            scaleMatrix3(omegaHat, (1 - Math.cos(thetaNorm)) / thetaSquared),
            scaleMatrix3(omegaHatSquared, (thetaNorm - Math.sin(thetaNorm)) / thetaCubed)
        )
    );
    const twistTranslation = multiplyMatrix3Vector(invertMatrix3(V), relativeTranslation);

    const scaledTheta = t * thetaNorm;
    const deltaQuaternion = axisAngleQuaternion(axis, scaledTheta);
    const Vt = addMatrix3(
        scaleMatrix3(identityMatrix3(), t),
        addMatrix3(
            scaleMatrix3(omegaHat, (1 - Math.cos(scaledTheta)) / thetaSquared),
            scaleMatrix3(omegaHatSquared, (scaledTheta - Math.sin(scaledTheta)) / thetaCubed)
        )
    );
    const deltaTranslation = multiplyMatrix3Vector(Vt, twistTranslation);

    return {
        quaternion: normalizeAny(
            multiplyQuaternions(fromQuaternion, deltaQuaternion),
            'ScLERP interpolated quaternion'
        ),
        position: add(fromPosition, rotateVectorByQuaternion(deltaTranslation, fromQuaternion))
    };
}

function findPiecewiseSegment(parameterValues, value) {
    const params = Array.isArray(parameterValues) ? parameterValues : [];
    if (params.length < 2) {
        throw new Error('piecewise parameter values must contain at least two entries');
    }
    const clampedValue = clamp(asFinite(value, 0), params[0], params[params.length - 1]);
    if (clampedValue >= params[params.length - 1]) {
        return {
            index: params.length - 2,
            localAlpha: 1,
            start: params[params.length - 2],
            end: params[params.length - 1]
        };
    }
    for (let index = 0; index < params.length - 1; index++) {
        const start = params[index];
        const end = params[index + 1];
        if (clampedValue >= start && clampedValue <= end) {
            const span = end - start;
            const localAlpha = span <= 1e-12 ? 0 : ((clampedValue - start) / span);
            return { index, localAlpha, start, end };
        }
    }
    return {
        index: 0,
        localAlpha: 0,
        start: params[0],
        end: params[1]
    };
}

function samplePiecewiseLinearCurve(points, parameterValues, value) {
    const segment = findPiecewiseSegment(parameterValues, value);
    const p0 = asVector3(points?.[segment.index], 'piecewise curve point');
    const p1 = asVector3(points?.[segment.index + 1], 'piecewise curve point');
    return lerpVector(p0, p1, segment.localAlpha);
}

function samplePiecewiseLinearCurveDerivative(points, parameterValues, value) {
    const segment = findPiecewiseSegment(parameterValues, value);
    const p0 = asVector3(points?.[segment.index], 'piecewise curve derivative point');
    const p1 = asVector3(points?.[segment.index + 1], 'piecewise curve derivative point');
    const span = segment.end - segment.start;
    if (!Number.isFinite(span) || Math.abs(span) <= 1e-12) {
        throw new Error('degenerate piecewise parameter span');
    }
    return scale(subtract(p1, p0), 1 / span);
}

function estimatePiecewiseCurveTangents(points, parameterValues) {
    const params = Array.isArray(parameterValues) ? parameterValues : [];
    const curvePoints = Array.isArray(points) ? points : [];
    if (params.length !== curvePoints.length || params.length < 2) {
        throw new Error('piecewise curve tangents require matching point and parameter arrays');
    }
    return curvePoints.map((point, index) => {
        const current = asVector3(point, 'piecewise tangent point');
        if (index === 0) {
            const next = asVector3(curvePoints[1], 'piecewise tangent next point');
            const span = params[1] - params[0];
            if (!Number.isFinite(span) || Math.abs(span) <= 1e-12) throw new Error('degenerate piecewise tangent span');
            return scale(subtract(next, current), 1 / span);
        }
        if (index === curvePoints.length - 1) {
            const previous = asVector3(curvePoints[index - 1], 'piecewise tangent previous point');
            const span = params[index] - params[index - 1];
            if (!Number.isFinite(span) || Math.abs(span) <= 1e-12) throw new Error('degenerate piecewise tangent span');
            return scale(subtract(current, previous), 1 / span);
        }
        const previous = asVector3(curvePoints[index - 1], 'piecewise tangent previous point');
        const next = asVector3(curvePoints[index + 1], 'piecewise tangent next point');
        const span = params[index + 1] - params[index - 1];
        if (!Number.isFinite(span) || Math.abs(span) <= 1e-12) throw new Error('degenerate piecewise tangent span');
        return scale(subtract(next, previous), 1 / span);
    });
}

function samplePiecewiseCubicHermiteCurve(points, parameterValues, tangents, value) {
    const segment = findPiecewiseSegment(parameterValues, value);
    const p0 = asVector3(points?.[segment.index], 'piecewise cubic point');
    const p1 = asVector3(points?.[segment.index + 1], 'piecewise cubic point');
    const m0 = asVector3(tangents?.[segment.index], 'piecewise cubic tangent');
    const m1 = asVector3(tangents?.[segment.index + 1], 'piecewise cubic tangent');
    const span = segment.end - segment.start;
    if (!Number.isFinite(span) || Math.abs(span) <= 1e-12) {
        throw new Error('degenerate piecewise cubic span');
    }
    const t = segment.localAlpha;
    const t2 = t * t;
    const t3 = t2 * t;
    const h00 = (2 * t3) - (3 * t2) + 1;
    const h10 = t3 - (2 * t2) + t;
    const h01 = (-2 * t3) + (3 * t2);
    const h11 = t3 - t2;
    return add(
        add(scale(p0, h00), scale(m0, h10 * span)),
        add(scale(p1, h01), scale(m1, h11 * span))
    );
}

function samplePiecewiseCubicHermiteCurveDerivative(points, parameterValues, tangents, value) {
    const segment = findPiecewiseSegment(parameterValues, value);
    const p0 = asVector3(points?.[segment.index], 'piecewise cubic derivative point');
    const p1 = asVector3(points?.[segment.index + 1], 'piecewise cubic derivative point');
    const m0 = asVector3(tangents?.[segment.index], 'piecewise cubic derivative tangent');
    const m1 = asVector3(tangents?.[segment.index + 1], 'piecewise cubic derivative tangent');
    const span = segment.end - segment.start;
    if (!Number.isFinite(span) || Math.abs(span) <= 1e-12) {
        throw new Error('degenerate piecewise cubic derivative span');
    }
    const t = segment.localAlpha;
    const t2 = t * t;
    const dh00 = (6 * t2) - (6 * t);
    const dh10 = (3 * t2) - (4 * t) + 1;
    const dh01 = (-6 * t2) + (6 * t);
    const dh11 = (3 * t2) - (2 * t);
    return scale(
        add(
            add(scale(p0, dh00), scale(m0, dh10 * span)),
            add(scale(p1, dh01), scale(m1, dh11 * span))
        ),
        1 / span
    );
}

function buildRuledLinePoseKeyframes(startPoints, endPoints, parameterValues) {
    const params = Array.isArray(parameterValues) ? parameterValues.slice() : [];
    if (params.length < 2 || params.length !== startPoints.length || params.length !== endPoints.length) {
        throw new Error('ruled line pose keyframes require matching sampled start/end curves');
    }
    const midpoints = startPoints.map((start, index) => lerpVector(start, endPoints[index], 0.5));
    const midpointTangents = estimatePiecewiseCurveTangents(midpoints, params);
    const keyframes = [];
    let previousQuaternion = null;
    let previousNormal = null;

    for (let index = 0; index < params.length; index++) {
        const start = asVector3(startPoints[index], 'ruled line pose keyframe start');
        const end = asVector3(endPoints[index], 'ruled line pose keyframe end');
        const midpoint = midpoints[index];
        const span = subtract(end, start);
        const length = norm(span);
        if (!Number.isFinite(length) || length <= 1e-10) {
            throw new Error('degenerate ruled line span');
        }
        const tangentV = normalize(span, 'ruled line tangent v');
        const midpointTangent = asVector3(midpointTangents[index], 'ruled line midpoint tangent');
        let tangentU = orthogonalize(midpointTangent, tangentV);
        if (norm(tangentU) <= 1e-10) {
            const fallbackCandidates = [
                keyframes[keyframes.length - 1]?.tangentU,
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ];
            for (const candidate of fallbackCandidates) {
                if (!candidate) continue;
                tangentU = orthogonalize(candidate, tangentV);
                if (norm(tangentU) > 1e-10) break;
            }
        }
        tangentU = normalize(tangentU, 'ruled line tangent u');
        let normal = normalize(cross(tangentU, tangentV), 'ruled line normal');
        if (previousNormal && dot(normal, previousNormal) < 0) {
            tangentU = scale(tangentU, -1);
            normal = scale(normal, -1);
        }
        let quaternion = quaternionFromRotationMatrix([
            [tangentU[0], tangentV[0], normal[0]],
            [tangentU[1], tangentV[1], normal[1]],
            [tangentU[2], tangentV[2], normal[2]]
        ]);
        if (previousQuaternion && dot(previousQuaternion, quaternion) < 0) {
            quaternion = quaternion.map((value) => -value);
        }
        previousQuaternion = quaternion;
        previousNormal = normal;
        keyframes.push({
            parameter: params[index],
            position: midpoint.slice(),
            quaternion: quaternion.slice(),
            tangentU: tangentU.slice(),
            direction: tangentV.slice(),
            normal: normal.slice(),
            halfLength: 0.5 * length
        });
    }

    return keyframes;
}

function sampleScLERPInterpolatedRuledLineState(keyframes, parameterValues, value) {
    const segment = findPiecewiseSegment(parameterValues, value);
    const startKeyframe = keyframes[segment.index];
    const endKeyframe = keyframes[segment.index + 1];
    const pose = interpolatePosesScLERP(startKeyframe, endKeyframe, segment.localAlpha);
    const halfLength = lerp(startKeyframe.halfLength, endKeyframe.halfLength, segment.localAlpha);
    const direction = normalize(
        rotateVectorByQuaternion([0, 1, 0], pose.quaternion),
        'ScLERP ruled line direction'
    );
    const tangentU = normalize(
        rotateVectorByQuaternion([1, 0, 0], pose.quaternion),
        'ScLERP ruled line tangent u'
    );
    const normal = normalize(
        rotateVectorByQuaternion([0, 0, 1], pose.quaternion),
        'ScLERP ruled line normal'
    );
    const start = add(pose.position, scale(direction, -halfLength));
    const end = add(pose.position, scale(direction, halfLength));
    return {
        position: pose.position,
        quaternion: pose.quaternion,
        tangentU,
        direction,
        normal,
        halfLength,
        length: 2 * halfLength,
        start,
        end
    };
}

function rotateAndTranslatePoint(point, quaternion, translation) {
    return add(rotateVectorByQuaternion(point, quaternion), translation);
}

function buildStructuredObjGridRow(points, rowIndex, cols, label) {
    const row = [];
    for (let colIndex = 0; colIndex < cols; colIndex++) {
        const point = points?.[rowIndex]?.[colIndex];
        row.push(asVector3(point, `${label} point`));
    }
    return row;
}

export function parseStructuredObjRuledSurfaceGrid(objText, {
    sourceName = 'structured-obj-ruled-surface'
} = {}) {
    if (typeof objText !== 'string' || objText.trim().length === 0) {
        throw new Error('structured OBJ text must be a non-empty string');
    }
    const vertices = [];
    const uvs = [];
    for (const rawLine of objText.split(/\r?\n/)) {
        const line = rawLine.trim();
        if (line.startsWith('v ')) {
            const [, xs, ys, zs] = line.split(/\s+/);
            vertices.push([
                asFinite(Number(xs), Number.NaN),
                asFinite(Number(ys), Number.NaN),
                asFinite(Number(zs), Number.NaN)
            ]);
        } else if (line.startsWith('vt ')) {
            const [, us, vs] = line.split(/\s+/);
            uvs.push([
                asFinite(Number(us), Number.NaN),
                asFinite(Number(vs), Number.NaN)
            ]);
        }
    }
    if (vertices.length === 0 || uvs.length === 0) {
        throw new Error('structured OBJ must contain both v and vt records');
    }
    if (vertices.length !== uvs.length) {
        throw new Error('structured OBJ demo expects one UV per vertex in matching order');
    }

    const uniqueU = Array.from(new Set(uvs.map(([u]) => u.toFixed(6)))).map(Number).sort((a, b) => a - b);
    const uniqueV = Array.from(new Set(uvs.map(([, v]) => v.toFixed(6)))).map(Number).sort((a, b) => a - b);
    if (uniqueU.length < 2 || uniqueV.length < 2) {
        throw new Error('structured OBJ must define at least a 2x2 UV grid');
    }

    const uIndexByKey = new Map(uniqueU.map((value, index) => [value.toFixed(6), index]));
    const vIndexByKey = new Map(uniqueV.map((value, index) => [value.toFixed(6), index]));
    const points = Array.from({ length: uniqueV.length }, () => Array.from({ length: uniqueU.length }, () => null));

    for (let index = 0; index < vertices.length; index++) {
        const vertex = asVector3(vertices[index], 'structured OBJ vertex');
        const [u, v] = uvs[index];
        const colIndex = uIndexByKey.get(u.toFixed(6));
        const rowIndex = vIndexByKey.get(v.toFixed(6));
        if (rowIndex == null || colIndex == null) {
            throw new Error('structured OBJ UV grid lookup failed');
        }
        points[rowIndex][colIndex] = vertex;
    }

    for (let rowIndex = 0; rowIndex < points.length; rowIndex++) {
        for (let colIndex = 0; colIndex < points[rowIndex].length; colIndex++) {
            if (!points[rowIndex][colIndex]) {
                throw new Error(`structured OBJ grid is incomplete at row ${rowIndex}, col ${colIndex}`);
            }
        }
    }

    return {
        kind: 'structured_obj_ruled_grid',
        source: sourceName,
        rows: uniqueV.length,
        cols: uniqueU.length,
        uValues: uniqueU,
        vValues: uniqueV,
        points
    };
}

function sampleStructuredObjGridPoint(grid, u, v) {
    if (!grid || grid.kind !== 'structured_obj_ruled_grid') {
        throw new Error('structured grid point sampling expects a structured_obj_ruled_grid');
    }
    const uSegment = findPiecewiseSegment(grid.uValues, u);
    const vSegment = findPiecewiseSegment(grid.vValues, v);
    const p00 = asVector3(grid.points?.[vSegment.index]?.[uSegment.index], 'structured ruled grid sample p00');
    const p10 = asVector3(grid.points?.[vSegment.index]?.[uSegment.index + 1], 'structured ruled grid sample p10');
    const p01 = asVector3(grid.points?.[vSegment.index + 1]?.[uSegment.index], 'structured ruled grid sample p01');
    const p11 = asVector3(grid.points?.[vSegment.index + 1]?.[uSegment.index + 1], 'structured ruled grid sample p11');
    const lower = lerpVector(p00, p10, uSegment.localAlpha);
    const upper = lerpVector(p01, p11, uSegment.localAlpha);
    return lerpVector(lower, upper, vSegment.localAlpha);
}

function buildEvenlySpacedParameterValues(start, end, sampleCount) {
    const count = Math.max(2, Math.floor(asFinite(sampleCount, 2)));
    if (count === 2) return [start, end];
    return Array.from({ length: count }, (_, index) => {
        const alpha = index / (count - 1);
        return lerp(start, end, alpha);
    });
}

export function resampleStructuredObjRuledSurfaceGrid(grid, {
    uSamples = null,
    vSamples = null,
    sourceName = null
} = {}) {
    if (!grid || grid.kind !== 'structured_obj_ruled_grid') {
        throw new Error('structured grid resampling expects a structured_obj_ruled_grid');
    }
    const originalUValues = Array.isArray(grid.uValues) ? grid.uValues.slice() : [];
    const originalVValues = Array.isArray(grid.vValues) ? grid.vValues.slice() : [];
    if (originalUValues.length < 2 || originalVValues.length < 2) {
        throw new Error('structured grid resampling requires at least a 2x2 parameter grid');
    }
    const resampledUValues = Number.isFinite(uSamples)
        ? buildEvenlySpacedParameterValues(originalUValues[0], originalUValues[originalUValues.length - 1], uSamples)
        : originalUValues.slice();
    const resampledVValues = Number.isFinite(vSamples)
        ? buildEvenlySpacedParameterValues(originalVValues[0], originalVValues[originalVValues.length - 1], vSamples)
        : originalVValues.slice();
    const points = resampledVValues.map((v) => resampledUValues.map((u) => sampleStructuredObjGridPoint(grid, u, v)));
    return {
        kind: 'structured_obj_ruled_grid',
        source: typeof sourceName === 'string' && sourceName.length > 0 ? sourceName : grid.source,
        rows: resampledVValues.length,
        cols: resampledUValues.length,
        uValues: resampledUValues,
        vValues: resampledVValues,
        points
    };
}

export function createStructuredObjRuledSurfaceFromGrid(grid, {
    basePosition = [0, 0, 0],
    baseQuaternion = [1, 0, 0, 0],
    meshScale = 1,
    gridPointTransform = null,
    meshQuaternion = [1, 0, 0, 0],
    uInterpolation = 'linear',
    name = null
} = {}) {
    if (!grid || grid.kind !== 'structured_obj_ruled_grid') {
        throw new Error('structured ruled surface expects a structured_obj_ruled_grid');
    }
    const baseTranslation = asVector3(basePosition, 'structured ruled surface base position');
    const baseRotation = normalizeAny(baseQuaternion, 'structured ruled surface base quaternion');
    const meshRotation = normalizeAny(meshQuaternion, 'structured ruled surface mesh quaternion');
    const resolvedMeshScale = Math.max(1e-9, asFinite(meshScale, 1));
    const pointTransform = typeof gridPointTransform === 'function'
        ? (point) => asVector3(gridPointTransform(point), 'structured ruled transformed point')
        : (point) => asVector3(point, 'structured ruled point');
    const resolvedUInterpolation = uInterpolation === 'cubic-hermite'
        ? 'cubic-hermite'
        : (uInterpolation === 'sclerp' ? 'sclerp' : 'linear');
    const uValues = Array.isArray(grid.uValues) ? grid.uValues.slice() : [];
    const vValues = Array.isArray(grid.vValues) ? grid.vValues.slice() : [];
    const startCurvePoints = buildStructuredObjGridRow(grid.points, 0, grid.cols, 'structured ruled start curve').map(pointTransform);
    const endCurvePoints = buildStructuredObjGridRow(grid.points, grid.rows - 1, grid.cols, 'structured ruled end curve').map(pointTransform);
    const startCurveTangents = resolvedUInterpolation === 'cubic-hermite'
        ? estimatePiecewiseCurveTangents(startCurvePoints, uValues)
        : null;
    const endCurveTangents = resolvedUInterpolation === 'cubic-hermite'
        ? estimatePiecewiseCurveTangents(endCurvePoints, uValues)
        : null;
    const localStartAt = (u) => resolvedUInterpolation === 'cubic-hermite'
        ? samplePiecewiseCubicHermiteCurve(startCurvePoints, uValues, startCurveTangents, u)
        : samplePiecewiseLinearCurve(startCurvePoints, uValues, u);
    const localEndAt = (u) => resolvedUInterpolation === 'cubic-hermite'
        ? samplePiecewiseCubicHermiteCurve(endCurvePoints, uValues, endCurveTangents, u)
        : samplePiecewiseLinearCurve(endCurvePoints, uValues, u);
    const localStartDerivativeAt = (u) => resolvedUInterpolation === 'cubic-hermite'
        ? samplePiecewiseCubicHermiteCurveDerivative(startCurvePoints, uValues, startCurveTangents, u)
        : samplePiecewiseLinearCurveDerivative(startCurvePoints, uValues, u);
    const localEndDerivativeAt = (u) => resolvedUInterpolation === 'cubic-hermite'
        ? samplePiecewiseCubicHermiteCurveDerivative(endCurvePoints, uValues, endCurveTangents, u)
        : samplePiecewiseLinearCurveDerivative(endCurvePoints, uValues, u);
    const localCenter = lerpVector(localStartAt(0.5), localEndAt(0.5), 0.5);
    const localToWorldPoint = (point) => rotateAndTranslatePoint(
        rotateVectorByQuaternion(scale(subtract(point, localCenter), resolvedMeshScale), meshRotation),
        baseRotation,
        baseTranslation
    );
    const localToWorldVector = (vector) => rotateVectorByQuaternion(
        rotateVectorByQuaternion(scale(vector, resolvedMeshScale), meshRotation),
        baseRotation
    );
    const worldStartCurvePoints = startCurvePoints.map(localToWorldPoint);
    const worldEndCurvePoints = endCurvePoints.map(localToWorldPoint);
    const scLerpKeyframes = resolvedUInterpolation === 'sclerp'
        ? buildRuledLinePoseKeyframes(worldStartCurvePoints, worldEndCurvePoints, uValues)
        : null;
    const minUStep = uValues.length > 1
        ? uValues.slice(1).reduce((minimum, current, index) => {
            const delta = current - uValues[index];
            return delta > 1e-12 ? Math.min(minimum, delta) : minimum;
        }, Number.POSITIVE_INFINITY)
        : 1;
    const resolvedDerivativeStep = Number.isFinite(minUStep) ? Math.max(1e-4, 0.25 * minUStep) : 1e-3;
    const sampleScLERPLineStateAt = (u) => sampleScLERPInterpolatedRuledLineState(
        scLerpKeyframes,
        uValues,
        clamp(asFinite(u, 0), uValues[0], uValues[uValues.length - 1])
    );
    const positionFromScLERPState = (state, v) => add(
        state.start,
        scale(state.direction, state.length * clamp(asFinite(v, 0), 0, 1))
    );

    return {
        kind: 'structured_obj_ruled_surface',
        name: name || `${grid.source}-surface`,
        metadata: {
            source: grid.source,
            rows: grid.rows,
            cols: grid.cols,
            meshScale: resolvedMeshScale,
            meshQuaternion: meshRotation.slice(),
            uInterpolation: resolvedUInterpolation
        },
        position(u, v) {
            if (resolvedUInterpolation === 'sclerp') {
                return positionFromScLERPState(sampleScLERPLineStateAt(u), v);
            }
            const uu = clamp(asFinite(u, 0), 0, 1);
            const vv = clamp(asFinite(v, 0), 0, 1);
            const localPoint = lerpVector(localStartAt(uu), localEndAt(uu), vv);
            return localToWorldPoint(localPoint);
        },
        du(u, v) {
            if (resolvedUInterpolation === 'sclerp') {
                const uu = clamp(asFinite(u, 0), 0, 1);
                const vv = clamp(asFinite(v, 0), 0, 1);
                const lowerU = Math.max(0, uu - resolvedDerivativeStep);
                const upperU = Math.min(1, uu + resolvedDerivativeStep);
                if (Math.abs(upperU - lowerU) <= 1e-10) {
                    return [0, 0, 0];
                }
                const lower = positionFromScLERPState(sampleScLERPLineStateAt(lowerU), vv);
                const upper = positionFromScLERPState(sampleScLERPLineStateAt(upperU), vv);
                return scale(subtract(upper, lower), 1 / (upperU - lowerU));
            }
            const uu = clamp(asFinite(u, 0), 0, 1);
            const vv = clamp(asFinite(v, 0), 0, 1);
            const localDerivative = lerpVector(localStartDerivativeAt(uu), localEndDerivativeAt(uu), vv);
            return localToWorldVector(localDerivative);
        },
        dv(u, _v) {
            if (resolvedUInterpolation === 'sclerp') {
                const lineState = sampleScLERPLineStateAt(u);
                return scale(lineState.direction, lineState.length);
            }
            const uu = clamp(asFinite(u, 0), 0, 1);
            return localToWorldVector(subtract(localEndAt(uu), localStartAt(uu)));
        },
        lineAt(u) {
            if (resolvedUInterpolation === 'sclerp') {
                const lineState = sampleScLERPLineStateAt(u);
                return buildPluckerLine(lineState.start, lineState.direction);
            }
            const uu = clamp(asFinite(u, 0), 0, 1);
            const start = localToWorldPoint(localStartAt(uu));
            const end = localToWorldPoint(localEndAt(uu));
            return buildPluckerLine(start, subtract(end, start));
        },
        tubeAt(u, radius) {
            return buildPluckerTube(this.lineAt(u), radius);
        }
    };
}

export function createAnalyticCylindricalRuledSurface({
    basePosition = [0, 0, 0],
    baseQuaternion = [1, 0, 0, 0],
    radius = 0.12,
    spanV = 0.3,
    angleSpan = Math.PI / 2,
    centerOffset = 0,
    flipU = false,
    sourceName = 'analytic-cylinder',
    name = null
} = {}) {
    const baseTranslation = asVector3(basePosition, 'analytic cylindrical ruled surface base position');
    const baseRotation = normalizeAny(baseQuaternion, 'analytic cylindrical ruled surface base quaternion');
    const resolvedRadius = Math.max(1e-6, asFinite(radius, 0.12));
    const resolvedSpanV = Math.max(1e-6, asFinite(spanV, 0.3));
    const resolvedAngleSpan = Math.max(1e-6, Math.abs(asFinite(angleSpan, Math.PI / 2)));
    const resolvedCenterOffset = asFinite(centerOffset, 0);
    const halfAngle = 0.5 * resolvedAngleSpan;
    const chordWidth = 2 * resolvedRadius * Math.sin(halfAngle);
    const sagitta = resolvedRadius * (1 - Math.cos(halfAngle));
    const localToWorldPoint = (point) => add(rotateVectorByQuaternion(point, baseRotation), baseTranslation);
    const localToWorldVector = (vector) => rotateVectorByQuaternion(vector, baseRotation);
    const sampleAngularState = (u) => {
        const uu = clamp(asFinite(u, 0), 0, 1);
        const mappedU = flipU ? 1 - uu : uu;
        const thetaMin = -0.5 * resolvedAngleSpan;
        const thetaMax = 0.5 * resolvedAngleSpan;
        const theta = lerp(thetaMin, thetaMax, mappedU);
        const dThetaDu = (flipU ? -1 : 1) * (thetaMax - thetaMin);
        return {
            u: uu,
            theta,
            dThetaDu
        };
    };
    const localCenterlinePoint = (u) => {
        const { theta } = sampleAngularState(u);
        return [
            resolvedRadius * Math.sin(theta),
            0,
            resolvedCenterOffset + (resolvedRadius * (Math.cos(theta) - 1))
        ];
    };
    const localCenterlineDerivative = (u) => {
        const { theta, dThetaDu } = sampleAngularState(u);
        return [
            resolvedRadius * Math.cos(theta) * dThetaDu,
            0,
            -resolvedRadius * Math.sin(theta) * dThetaDu
        ];
    };
    const localLineDirection = [0, 1, 0];
    const localLineSpan = [0, resolvedSpanV, 0];
    const localLineStart = (u) => add(localCenterlinePoint(u), [0, -0.5 * resolvedSpanV, 0]);

    return {
        kind: 'analytic_cylindrical_ruled_surface',
        name: name || `${sourceName}-surface`,
        metadata: {
            source: sourceName,
            parameterization: 'analytic-cylinder',
            radius: resolvedRadius,
            spanV: resolvedSpanV,
            angleSpan: resolvedAngleSpan,
            arcLength: resolvedRadius * resolvedAngleSpan,
            chordWidth,
            sagitta,
            localBBox: {
                x: chordWidth,
                y: resolvedSpanV,
                z: sagitta
            },
            centerOffset: resolvedCenterOffset,
            flipU: Boolean(flipU)
        },
        position(u, v) {
            const centerlinePoint = localCenterlinePoint(u);
            const vv = clamp(asFinite(v, 0), 0, 1);
            const localPoint = add(centerlinePoint, [0, resolvedSpanV * (vv - 0.5), 0]);
            return localToWorldPoint(localPoint);
        },
        du(u, _v) {
            return localToWorldVector(localCenterlineDerivative(u));
        },
        dv(_u, _v) {
            return localToWorldVector(localLineSpan);
        },
        lineAt(u) {
            return buildPluckerLine(
                localToWorldPoint(localLineStart(u)),
                localToWorldVector(localLineDirection)
            );
        },
        tubeAt(u, radiusValue) {
            return buildPluckerTube(this.lineAt(u), radiusValue);
        }
    };
}

export function createCylindricalStructuredObjPointTransform(grid, {
    radiusScale = 1,
    angleScale = 1,
    axialScale = 1
} = {}) {
    if (!grid || grid.kind !== 'structured_obj_ruled_grid') {
        throw new Error('cylindrical structured transform expects a structured_obj_ruled_grid');
    }
    const resolvedRadiusScale = Math.max(1e-9, asFinite(radiusScale, 1));
    const resolvedAngleScale = Math.max(1e-9, asFinite(angleScale, 1));
    const resolvedAxialScale = Math.max(1e-9, asFinite(axialScale, 1));
    const flatPoints = Array.isArray(grid.points) ? grid.points.flat() : [];
    let inferredRadius = Number.NaN;
    for (const point of flatPoints) {
        const [x, y] = asVector3(point, 'cylindrical structured point');
        if (Math.abs(y) <= 1e-12) continue;
        const radius = ((x * x) + (y * y)) / (2 * y);
        if (Number.isFinite(radius) && radius > 1e-9) {
            inferredRadius = radius;
            break;
        }
    }
    if (!Number.isFinite(inferredRadius) || inferredRadius <= 1e-9) {
        throw new Error('could not infer cylindrical patch radius from structured OBJ grid');
    }
    const targetRadius = inferredRadius * resolvedRadiusScale;
    return (point) => {
        const [x, y, z] = asVector3(point, 'cylindrical structured source point');
        const theta = Math.atan2(x, inferredRadius - y);
        const mappedTheta = theta * resolvedAngleScale;
        return [
            targetRadius * Math.sin(mappedTheta),
            targetRadius - (targetRadius * Math.cos(mappedTheta)),
            z * resolvedAxialScale
        ];
    };
}

export function blendUnitQuaternions(fromQuaternion, toQuaternion, alpha = 1) {
    const t = clamp(asFinite(alpha, 1), 0, 1);
    const from = normalizeAny(fromQuaternion, 'source quaternion');
    const toRaw = normalizeAny(toQuaternion, 'target quaternion');
    const signedTarget = dot(from, toRaw) < 0 ? toRaw.map((value) => -value) : toRaw;
    const blended = from.map((value, index) => ((1 - t) * value) + (t * signedTarget[index]));
    return normalizeAny(blended, 'blended quaternion');
}

export function evaluateSurfacePatch(surface, u, v) {
    if (!surface || typeof surface.position !== 'function' || typeof surface.du !== 'function' || typeof surface.dv !== 'function') {
        throw new Error('surface must provide position(u,v), du(u,v), and dv(u,v)');
    }

    return {
        u: asFinite(u),
        v: asFinite(v),
        position: asVector3(surface.position(u, v), 'surface position'),
        du: asVector3(surface.du(u, v), 'surface du'),
        dv: asVector3(surface.dv(u, v), 'surface dv')
    };
}

export function buildSurfaceAttachedFrame(surfaceState) {
    const position = asVector3(surfaceState?.position, 'surface position');
    const du = asVector3(surfaceState?.du, 'surface du');
    const dv = asVector3(surfaceState?.dv, 'surface dv');

    const tangentU = normalize(du, 'surface tangent u');
    const dvOrtho = orthogonalize(dv, tangentU);
    const tangentV = normalize(dvOrtho, 'surface tangent v');
    const normal = normalize(cross(tangentU, tangentV), 'surface normal');

    const rotationMatrix = [
        [tangentU[0], tangentV[0], normal[0]],
        [tangentU[1], tangentV[1], normal[1]],
        [tangentU[2], tangentV[2], normal[2]]
    ];

    return {
        position,
        tangentU,
        tangentV,
        normal,
        rotationMatrix,
        quaternion: quaternionFromRotationMatrix(rotationMatrix)
    };
}

export function buildSurfaceAttachedPoseTarget({
    surface,
    u = 0,
    v = 0,
    normalOffset = 0,
    normalOffsetSign = 1,
    quaternionOverride = null
} = {}) {
    const state = evaluateSurfacePatch(surface, u, v);
    const frame = buildSurfaceAttachedFrame(state);
    const resolvedNormalOffsetSign = Number.isFinite(normalOffsetSign)
        ? (Math.sign(normalOffsetSign) || 1)
        : 1;
    const quaternion = Array.isArray(quaternionOverride) && quaternionOverride.length === 4
        ? normalizeAny(quaternionOverride, 'quaternion override')
        : frame.quaternion.slice();
    return {
        kind: 'surface_attached_pose',
        u: state.u,
        v: state.v,
        position: add(frame.position, scale(frame.normal, asFinite(normalOffset) * resolvedNormalOffsetSign)),
        quaternion,
        rotationMatrix: frame.rotationMatrix.map((row) => row.slice()),
        surface: {
            position: frame.position.slice(),
            tangentU: frame.tangentU.slice(),
            tangentV: frame.tangentV.slice(),
            normal: frame.normal.slice(),
            normalOffset: asFinite(normalOffset),
            normalOffsetSign: resolvedNormalOffsetSign
        }
    };
}

function dotAny(a, b) {
    const left = Array.isArray(a) ? a : [];
    const right = Array.isArray(b) ? b : [];
    const width = Math.min(left.length, right.length);
    let sum = 0;
    for (let index = 0; index < width; index++) {
        sum += asFinite(left[index]) * asFinite(right[index]);
    }
    return sum;
}

function scaleAny(values, scalar) {
    return (Array.isArray(values) ? values : []).map((value) => asFinite(value) * scalar);
}

function addAny(a, b) {
    const left = Array.isArray(a) ? a : [];
    const right = Array.isArray(b) ? b : [];
    const width = Math.max(left.length, right.length);
    return Array.from({ length: width }, (_, index) => asFinite(left[index]) + asFinite(right[index]));
}

function subtractAny(a, b) {
    const left = Array.isArray(a) ? a : [];
    const right = Array.isArray(b) ? b : [];
    const width = Math.max(left.length, right.length);
    return Array.from({ length: width }, (_, index) => asFinite(left[index]) - asFinite(right[index]));
}

function alignQuaternionToReference(quaternion, referenceQuaternion) {
    const normalized = normalizeAny(quaternion, 'aligned quaternion');
    const reference = normalizeAny(referenceQuaternion, 'aligned quaternion reference');
    return dotAny(normalized, reference) < 0
        ? normalized.map((value) => -value)
        : normalized;
}

function buildPoseDualQuaternionArray(pose) {
    const quaternion = normalizeAny(pose?.quaternion, 'dual quaternion pose quaternion');
    const position = asVector3(pose?.position, 'dual quaternion pose position');
    const translationQuaternion = [0, position[0], position[1], position[2]];
    const dual = scaleAny(multiplyQuaternions(translationQuaternion, quaternion), 0.5);
    return [...quaternion, ...dual];
}

function resolveFiniteDifferenceStencil(value, bounds, step) {
    const resolvedStep = Math.max(1e-6, Math.abs(asFinite(step, 1e-4)));
    const lower = Number.isFinite(bounds?.[0]) ? bounds[0] : Number.NEGATIVE_INFINITY;
    const upper = Number.isFinite(bounds?.[1]) ? bounds[1] : Number.POSITIVE_INFINITY;
    if ((value - resolvedStep) >= lower && (value + resolvedStep) <= upper) {
        return {
            type: 'central',
            minus: value - resolvedStep,
            plus: value + resolvedStep,
            denominator: 2 * resolvedStep
        };
    }
    if ((value + resolvedStep) <= upper) {
        return {
            type: 'forward',
            base: value,
            plus: value + resolvedStep,
            denominator: resolvedStep
        };
    }
    if ((value - resolvedStep) >= lower) {
        return {
            type: 'backward',
            minus: value - resolvedStep,
            base: value,
            denominator: resolvedStep
        };
    }
    return {
        type: 'none',
        base: value,
        denominator: 1
    };
}

function differentiateAttachedPose(buildPose, value, bounds, step, referenceQuaternion) {
    const stencil = resolveFiniteDifferenceStencil(value, bounds, step);
    if (stencil.type === 'none') {
        return {
            positionDerivative: [0, 0, 0],
            quaternionDerivative: [0, 0, 0, 0]
        };
    }

    if (stencil.type === 'central') {
        const poseMinus = buildPose(stencil.minus);
        const posePlus = buildPose(stencil.plus);
        const alignedMinus = alignQuaternionToReference(poseMinus.quaternion, referenceQuaternion);
        const alignedPlus = alignQuaternionToReference(posePlus.quaternion, referenceQuaternion);
        return {
            positionDerivative: scale(subtract(posePlus.position, poseMinus.position), 1 / stencil.denominator),
            quaternionDerivative: scaleAny(subtractAny(alignedPlus, alignedMinus), 1 / stencil.denominator)
        };
    }

    if (stencil.type === 'forward') {
        const poseBase = buildPose(stencil.base);
        const posePlus = buildPose(stencil.plus);
        const alignedBase = alignQuaternionToReference(poseBase.quaternion, referenceQuaternion);
        const alignedPlus = alignQuaternionToReference(posePlus.quaternion, referenceQuaternion);
        return {
            positionDerivative: scale(subtract(posePlus.position, poseBase.position), 1 / stencil.denominator),
            quaternionDerivative: scaleAny(subtractAny(alignedPlus, alignedBase), 1 / stencil.denominator)
        };
    }

    const poseMinus = buildPose(stencil.minus);
    const poseBase = buildPose(stencil.base);
    const alignedMinus = alignQuaternionToReference(poseMinus.quaternion, referenceQuaternion);
    const alignedBase = alignQuaternionToReference(poseBase.quaternion, referenceQuaternion);
    return {
        positionDerivative: scale(subtract(poseBase.position, poseMinus.position), 1 / stencil.denominator),
        quaternionDerivative: scaleAny(subtractAny(alignedBase, alignedMinus), 1 / stencil.denominator)
    };
}

function poseTwistToDualQuaternionDerivative(pose, angularVelocity, translationalVelocity) {
    const quaternion = normalizeAny(pose?.quaternion, 'pose twist quaternion');
    const position = asVector3(pose?.position, 'pose twist position');
    const dqPose = buildPoseDualQuaternionArray({ position, quaternion });
    const dual = dqPose.slice(4, 8);
    const omegaQuaternion = [0, ...asVector3(angularVelocity, 'pose twist angular velocity')];
    const translationQuaternion = [0, ...asVector3(translationalVelocity, 'pose twist translational velocity')];
    const realDerivative = scaleAny(multiplyQuaternions(omegaQuaternion, quaternion), 0.5);
    const dualDerivative = scaleAny(
        addAny(
            multiplyQuaternions(translationQuaternion, quaternion),
            multiplyQuaternions(omegaQuaternion, dual)
        ),
        0.5
    );
    return [...realDerivative, ...dualDerivative];
}

export function buildSurfaceAttachedPoseJacobian({
    surface,
    u = 0,
    v = 0,
    normalOffset = 0,
    normalOffsetSign = 1,
    quaternionOffset = null,
    bounds = {
        u: [0, 1],
        v: [0, 1]
    },
    differentiationStep = 1e-4
} = {}) {
    const resolvedNormalOffsetSign = Number.isFinite(normalOffsetSign)
        ? (Math.sign(normalOffsetSign) || 1)
        : 1;
    const buildPoseAt = (nextU, nextV) => {
        const attachedPose = buildSurfaceAttachedPoseTarget({
            surface,
            u: nextU,
            v: nextV,
            normalOffset,
            normalOffsetSign: resolvedNormalOffsetSign
        });
        if (Array.isArray(quaternionOffset) && quaternionOffset.length === 4) {
            attachedPose.quaternion = normalizeAny(
                multiplyQuaternions(attachedPose.quaternion, quaternionOffset),
                'surface attached pose quaternion offset'
            );
        }
        return attachedPose;
    };

    const pose = buildPoseAt(asFinite(u, 0), asFinite(v, 0));
    const dqPose = buildPoseDualQuaternionArray(pose);
    const quaternion = normalizeAny(pose.quaternion, 'surface attached pose base quaternion');
    const uBounds = Array.isArray(bounds?.u) && bounds.u.length === 2 ? bounds.u : [0, 1];
    const vBounds = Array.isArray(bounds?.v) && bounds.v.length === 2 ? bounds.v : [0, 1];

    const duModel = differentiateAttachedPose(
        (sampleU) => buildPoseAt(sampleU, v),
        asFinite(u, 0),
        uBounds,
        differentiationStep,
        quaternion
    );
    const dvModel = differentiateAttachedPose(
        (sampleV) => buildPoseAt(u, sampleV),
        asFinite(v, 0),
        vBounds,
        differentiationStep,
        quaternion
    );

    const omegaUQuaternion = multiplyQuaternions(scaleAny(duModel.quaternionDerivative, 2), conjugateQuaternion(quaternion));
    const omegaVQuaternion = multiplyQuaternions(scaleAny(dvModel.quaternionDerivative, 2), conjugateQuaternion(quaternion));
    const omegaU = [omegaUQuaternion[1], omegaUQuaternion[2], omegaUQuaternion[3]];
    const omegaV = [omegaVQuaternion[1], omegaVQuaternion[2], omegaVQuaternion[3]];
    const vU = add(duModel.positionDerivative, cross(pose.position, omegaU));
    const vV = add(dvModel.positionDerivative, cross(pose.position, omegaV));
    const vN = scale(pose.surface.normal, resolvedNormalOffsetSign);

    const twistColumns = {
        u: [0, ...omegaU, 0, ...vU],
        v: [0, ...omegaV, 0, ...vV],
        n: [0, 0, 0, 0, 0, ...vN]
    };
    const columns = [
        poseTwistToDualQuaternionDerivative(pose, omegaU, vU),
        poseTwistToDualQuaternionDerivative(pose, omegaV, vV),
        poseTwistToDualQuaternionDerivative(pose, [0, 0, 0], vN)
    ];
    const jacobian = Array.from({ length: 8 }, (_, rowIndex) => columns.map((column) => column[rowIndex]));

    return {
        pose,
        dqPose,
        twistColumns,
        jacobian
    };
}

export function createDQMotionRuledSurface({
    basePosition = [0, 0, 0],
    baseQuaternion = [1, 0, 0, 0],
    centerOffset = -0.06,
    spanU = 0.22,
    spanV = 0.18,
    liftAmplitude = 0.035,
    twistAngle = Math.PI / 2,
    flipU = false
} = {}) {
    const centerPose = asVector3(basePosition, 'dq motion base position');
    const baseRotation = normalizeAny(baseQuaternion, 'dq motion base quaternion');
    const tangentU0 = rotateVectorByQuaternion([1, 0, 0], baseRotation);
    const tangentV0 = rotateVectorByQuaternion([0, 1, 0], baseRotation);
    const normal0 = normalize(rotateVectorByQuaternion([0, 0, 1], baseRotation), 'dq motion base normal');
    const center = add(centerPose, scale(normal0, asFinite(centerOffset, -0.06)));
    const resolvedSpanU = Math.abs(asFinite(spanU, 0.22));
    const resolvedSpanV = Math.abs(asFinite(spanV, 0.18));
    const resolvedLift = asFinite(liftAmplitude, 0.035);
    const resolvedTwist = asFinite(twistAngle, Math.PI / 2);
    const resolvedFlipU = Boolean(flipU);
    const mapU = (u) => resolvedFlipU ? 1 - asFinite(u, 0) : asFinite(u, 0);
    const parabolicLift = (u) => {
        const uu = clamp(asFinite(u, 0), 0, 1);
        return resolvedLift * uu * uu;
    };
    const parabolicLiftDerivative = (u) => {
        const uu = clamp(asFinite(u, 0), 0, 1);
        return 2 * resolvedLift * uu;
    };
    const centerlineLift = (u) => parabolicLift(u);
    const centerlineLiftDerivative = (u) => parabolicLiftDerivative(u);

    const centerlineAt = (u) => {
        const uu = asFinite(u, 0) - 0.5;
        return add(
            add(center, scale(tangentU0, resolvedSpanU * uu)),
            scale(normal0, centerlineLift(u))
        );
    };

    const centerlineDerivativeAt = (u) => {
        return add(
            scale(tangentU0, resolvedSpanU),
            scale(normal0, centerlineLiftDerivative(u))
        );
    };

    const twistQuaternionAt = (u) => axisAngleQuaternion(normal0, resolvedTwist * (asFinite(u, 0) - 0.5));
    const lineDirectionAt = (u) => rotateVectorByQuaternion(tangentV0, twistQuaternionAt(u));
    const lineDirectionDerivativeAt = (u) => scale(cross(normal0, lineDirectionAt(u)), resolvedTwist);
    const lineAtInternal = (u) => {
        const point = centerlineAt(u);
        const direction = lineDirectionAt(u);
        const line = buildPluckerLine(point, direction);
        line.directionDerivative = lineDirectionDerivativeAt(u);
        return line;
    };

    return {
        kind: 'dq_motion_ruled_surface',
        name: 'humanoid-dq-ruled-surface',
        metadata: {
            center: center.slice(),
            tangentU0: tangentU0.slice(),
            tangentV0: tangentV0.slice(),
            normal0: normal0.slice(),
            spanU: resolvedSpanU,
            spanV: resolvedSpanV,
            liftAmplitude: resolvedLift,
            twistAngle: resolvedTwist,
            flipU: resolvedFlipU
        },
        poseAt(u) {
            const mappedU = mapU(u);
            const twistQuaternion = twistQuaternionAt(mappedU);
            return {
                position: centerlineAt(mappedU),
                quaternion: normalizeAny(multiplyQuaternions(twistQuaternion, baseRotation), 'dq motion pose quaternion')
            };
        },
        lineAt(u) {
            return lineAtInternal(mapU(u));
        },
        tubeAt(u, radius) {
            return buildPluckerTube(lineAtInternal(mapU(u)), radius);
        },
        position(u, v) {
            const vv = asFinite(v, 0) - 0.5;
            const line = lineAtInternal(mapU(u));
            return line.pointAtCoordinate(line.referenceCoordinate + (resolvedSpanV * vv));
        },
        du(u, v) {
            const vv = asFinite(v, 0) - 0.5;
            const mappedU = mapU(u);
            const signU = resolvedFlipU ? -1 : 1;
            const line = lineAtInternal(mappedU);
            const lineDerivative = buildPluckerLineDerivative(centerlineAt(mappedU), centerlineDerivativeAt(mappedU), line);
            const coordinate = line.referenceCoordinate + (resolvedSpanV * vv);
            const derivative = add(
                lineDerivative.closestPointDerivative,
                add(
                    scale(line.direction, lineDerivative.referenceCoordinateDerivative),
                    scale(lineDerivative.directionDerivative, coordinate)
                )
            );
            return scale(derivative, signU);
        },
        dv(u, _v) {
            const line = lineAtInternal(mapU(u));
            return scale(line.direction, resolvedSpanV);
        }
    };
}

export function advanceSurfaceParameters(
    state = {},
    {
        parameter = 'u',
        direction = 'positive',
        step = 0.01,
        bounds = {
            u: [0, 1],
            v: [0, 1]
        }
    } = {}
) {
    const uBounds = Array.isArray(bounds?.u) && bounds.u.length === 2 ? bounds.u : [0, 1];
    const vBounds = Array.isArray(bounds?.v) && bounds.v.length === 2 ? bounds.v : [0, 1];
    const sign = direction === 'negative' ? -1 : 1;
    const delta = Math.abs(asFinite(step, 0.01)) * sign;
    const next = {
        u: asFinite(state?.u, 0),
        v: asFinite(state?.v, 0)
    };

    if (parameter === 'v') {
        next.v = clamp(next.v + delta, vBounds[0], vBounds[1]);
    } else {
        next.u = clamp(next.u + delta, uBounds[0], uBounds[1]);
    }

    return next;
}

export function advanceSurfaceNormalOffset(
    state = {},
    {
        direction = 'positive',
        step = 0.01,
        bounds = [0, 0.2]
    } = {}
) {
    const resolvedBounds = Array.isArray(bounds) && bounds.length === 2 ? bounds : [0, 0.2];
    const sign = direction === 'negative' ? -1 : 1;
    const delta = Math.abs(asFinite(step, 0.01)) * sign;
    const current = clamp(asFinite(state?.normalOffset, 0), resolvedBounds[0], resolvedBounds[1]);
    return clamp(current + delta, resolvedBounds[0], resolvedBounds[1]);
}

export default {
    evaluateSurfacePatch,
    buildSurfaceAttachedFrame,
    buildSurfaceAttachedPoseTarget,
    buildSurfaceAttachedPoseJacobian,
    createAnalyticCylindricalRuledSurface,
    createDQMotionRuledSurface,
    parseStructuredObjRuledSurfaceGrid,
    resampleStructuredObjRuledSurfaceGrid,
    createStructuredObjRuledSurfaceFromGrid,
    createCylindricalStructuredObjPointTransform,
    computePointToPluckerLineSquaredDistance,
    computePointToPluckerLineDistanceJacobian,
    buildPluckerTube,
    computePointToPluckerTubeSignedSquaredDistance,
    computePointToPluckerTubeSignedSquaredDistanceJacobian,
    samplePluckerTubeGuideSegments,
    blendUnitQuaternions,
    interpolatePosesScLERP,
    advanceSurfaceParameters,
    advanceSurfaceNormalOffset
};
