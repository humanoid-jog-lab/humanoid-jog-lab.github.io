function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function asVector(values) {
    if (!Array.isArray(values)) return [];
    return values.map((value) => asFinite(value));
}

function asRows(rows, width = 0) {
    if (!Array.isArray(rows)) return [];
    return rows.map((row) => {
        const src = Array.isArray(row) ? row : [];
        const out = src.map((value) => asFinite(value));
        while (width > 0 && out.length < width) out.push(0);
        return width > 0 ? out.slice(0, width) : out;
    });
}

function signedPoseError(target, current) {
    const tgt = asVector(target);
    const cur = asVector(current);
    const n = Math.min(tgt.length, cur.length);
    if (n === 0) return [];
    let dot = 0;
    for (let i = 0; i < n; i++) dot += tgt[i] * cur[i];
    const sign = dot < 0 ? -1 : 1;
    const err = new Array(n).fill(0);
    for (let i = 0; i < n; i++) err[i] = tgt[i] - sign * cur[i];
    return err;
}

function vectorNorm(values) {
    const src = Array.isArray(values) ? values : [];
    let sum = 0;
    for (const value of src) sum += asFinite(value) * asFinite(value);
    return Math.sqrt(sum);
}

function buildEqualityBlock(name, target, current, jacobianRows, width, offset) {
    const residual = signedPoseError(target, current);
    const jacobian = asRows(jacobianRows, width);
    return {
        name,
        offset,
        length: residual.length,
        target: asVector(target),
        current: asVector(current),
        residual,
        jacobian
    };
}

function scaledBlock(block, weight = 1) {
    const scale = Math.max(0, asFinite(weight, 1));
    return {
        residual: block.residual.map((value) => value * scale),
        jacobian: block.jacobian.map((row) => row.map((value) => value * scale))
    };
}

function buildBodyZBlock(bodyZConstraint, q, width, offset) {
    if (typeof bodyZConstraint !== 'function') return null;
    const state = bodyZConstraint(q) || {};
    const residual = asVector(state.residual ?? state.error ?? state.value);
    const jacobian = asRows(state.jacobian ?? state.Jrows, width);
    return {
        name: 'body_z',
        offset,
        length: residual.length,
        target: asVector(state.target),
        current: asVector(state.current),
        residual,
        jacobian
    };
}

function buildBounds(qCurrent, lowerBounds, upperBounds, stepBounds, width) {
    const qBase = asVector(qCurrent);
    const lower = new Array(width).fill(Number.NEGATIVE_INFINITY);
    const upper = new Array(width).fill(Number.POSITIVE_INFINITY);
    const step = new Array(width).fill(Number.POSITIVE_INFINITY);
    const effectiveLower = new Array(width).fill(Number.NEGATIVE_INFINITY);
    const effectiveUpper = new Array(width).fill(Number.POSITIVE_INFINITY);

    for (let i = 0; i < width; i++) {
        lower[i] = Number.isFinite(lowerBounds?.[i]) ? lowerBounds[i] : Number.NEGATIVE_INFINITY;
        upper[i] = Number.isFinite(upperBounds?.[i]) ? upperBounds[i] : Number.POSITIVE_INFINITY;
        step[i] = Number.isFinite(stepBounds?.[i]) ? Math.max(0, stepBounds[i]) : Number.POSITIVE_INFINITY;
        effectiveLower[i] = Math.max(lower[i], asFinite(qBase[i]) - step[i]);
        effectiveUpper[i] = Math.min(upper[i], asFinite(qBase[i]) + step[i]);
    }

    return {
        lower,
        upper,
        step,
        effectiveLower,
        effectiveUpper
    };
}

function buildWVFIConstraint(wvfiState, q, width, lowerBound) {
    const floor = Math.max(0, asFinite(lowerBound, 0));
    let value = Number.NaN;
    let gradient = new Array(width).fill(0);
    if (typeof wvfiState === 'function') {
        const state = wvfiState(q);
        if (typeof state === 'number') {
            value = state;
        } else if (state) {
            value = asFinite(state.value, Number.NaN);
            const srcGradient = asVector(state.gradient);
            if (srcGradient.length > 0) {
                gradient = new Array(width).fill(0);
                for (let i = 0; i < Math.min(width, srcGradient.length); i++) gradient[i] = srcGradient[i];
            }
        }
    }
    const slack = Number.isFinite(value) ? value - floor : Number.NaN;
    return {
        enabled: floor > 0,
        lowerBound: floor,
        value,
        slack,
        violation: Number.isFinite(slack) ? Math.max(0, -slack) : (floor > 0 ? Number.POSITIVE_INFINITY : 0),
        gradient
    };
}

function buildWaistXCorridorConstraint(waistXCorridor, q) {
    if (waistXCorridor?.enabled !== true || typeof waistXCorridor.evaluateX !== 'function') {
        return {
            enabled: false,
            min: Number.NEGATIVE_INFINITY,
            max: Number.POSITIVE_INFINITY,
            value: Number.NaN,
            violation: 0,
            vfiRows: [],
            rhs: [],
            vfiConstraints: []
        };
    }
    const width = Array.isArray(q) ? q.length : 0;
    const value = asFinite(waistXCorridor.evaluateX(q), Number.NaN);
    const min = Number.isFinite(waistXCorridor.min) ? waistXCorridor.min : Number.NEGATIVE_INFINITY;
    const max = Number.isFinite(waistXCorridor.max) ? waistXCorridor.max : Number.POSITIVE_INFINITY;
    const tolerance = Math.max(0, asFinite(waistXCorridor.tolerance, 0));
    const vfiRows = asRows(waistXCorridor.vfiRows);
    const rhs = asVector(waistXCorridor.rhs);
    const vfiConstraints = (Array.isArray(waistXCorridor.vfiConstraints) ? waistXCorridor.vfiConstraints : [])
        .map((constraint) => {
            const row = asRows([constraint?.row], width)[0] ?? [];
            return {
                ...constraint,
                row,
                distance: asFinite(constraint?.distance, Number.NaN),
                threshold: asFinite(constraint?.threshold, Number.NaN)
            };
        })
        .filter((constraint) =>
            Array.isArray(constraint.row) &&
            constraint.row.length === width &&
            Number.isFinite(constraint.distance)
        );
    const violationLow = Number.isFinite(value) ? Math.max(0, (min - tolerance) - value) : Number.POSITIVE_INFINITY;
    const violationHigh = Number.isFinite(value) ? Math.max(0, value - (max + tolerance)) : Number.POSITIVE_INFINITY;
    return {
        enabled: true,
        min,
        max,
        tolerance,
        value,
        vfiRows,
        rhs,
        violationLow,
        violationHigh,
        violation: Math.max(violationLow, violationHigh),
        vfiConstraints
    };
}

export function buildHumanoidJogManifoldProblem({
    q,
    qCurrent,
    absoluteTarget,
    relativeTarget,
    absolutePose,
    relativePose,
    absoluteJacobian,
    relativeJacobian,
    includeBodyZ = false,
    bodyZConstraint,
    bodyEqualityTolerance = 0,
    absoluteWeight = 1,
    relativeWeight = 1,
    bodyWeight = 1,
    wvfiState,
    wvfiLowerBound = 0,
    lowerBounds = [],
    upperBounds = [],
    stepBounds = [],
    waistXCorridor = null
} = {}) {
    const qEval = asVector(q);
    const qBase = asVector(qCurrent);
    const width = qEval.length;
    const absoluteCurrent = typeof absolutePose === 'function' ? asVector(absolutePose(qEval)) : [];
    const relativeCurrent = typeof relativePose === 'function' ? asVector(relativePose(qEval)) : [];
    const absoluteBlock = buildEqualityBlock(
        'absolute',
        absoluteTarget,
        absoluteCurrent,
        typeof absoluteJacobian === 'function' ? absoluteJacobian(qEval) : [],
        width,
        0
    );
    const relativeBlock = buildEqualityBlock(
        'relative',
        relativeTarget,
        relativeCurrent,
        typeof relativeJacobian === 'function' ? relativeJacobian(qEval) : [],
        width,
        absoluteBlock.length
    );
    const bodyZBlock = includeBodyZ
        ? buildBodyZBlock(bodyZConstraint, qEval, width, absoluteBlock.length + relativeBlock.length)
        : null;

    const blocks = [absoluteBlock, relativeBlock];
    if (bodyZBlock) blocks.push(bodyZBlock);

    const residual = [];
    const jacobian = [];
    const weightedResidual = [];
    const weightedJacobian = [];
    const activeResidual = [];
    const activeJacobian = [];
    const absoluteWeighted = scaledBlock(absoluteBlock, absoluteWeight);
    const relativeWeighted = scaledBlock(relativeBlock, relativeWeight);
    const bodyWeighted = bodyZBlock ? scaledBlock(bodyZBlock, bodyWeight) : null;
    for (const block of blocks) {
        residual.push(...block.residual);
        jacobian.push(...block.jacobian);
    }
    weightedResidual.push(...absoluteWeighted.residual, ...relativeWeighted.residual);
    weightedJacobian.push(...absoluteWeighted.jacobian, ...relativeWeighted.jacobian);
    activeResidual.push(...absoluteBlock.residual, ...relativeBlock.residual);
    activeJacobian.push(...absoluteBlock.jacobian, ...relativeBlock.jacobian);
    const bodyResidualNorm = bodyZBlock ? vectorNorm(bodyZBlock.residual) : 0;
    const bodyTol = Math.max(0, asFinite(bodyEqualityTolerance, 0));
    const bodyActive = bodyWeighted && bodyResidualNorm > bodyTol + 1e-18;
    if (bodyActive) {
        weightedResidual.push(...bodyWeighted.residual);
        weightedJacobian.push(...bodyWeighted.jacobian);
        activeResidual.push(...bodyZBlock.residual);
        activeJacobian.push(...bodyZBlock.jacobian);
    }

    return {
        q: qEval.slice(),
        qCurrent: qBase.slice(),
        qDelta: qEval.map((value, index) => value - asFinite(qBase[index])),
        equalities: {
            residual,
            jacobian,
            activeResidual,
            activeJacobian,
            activeResidualNorm: vectorNorm(activeResidual),
            weightedResidual,
            weightedJacobian,
            activeRowCount: weightedResidual.length,
            rowCount: residual.length,
            columnCount: width,
            residualNorm: vectorNorm(residual),
            blocks: {
                absolute: absoluteBlock,
                relative: relativeBlock,
                bodyZ: bodyZBlock
            }
        },
        inequalities: {
            wvfiFloor: buildWVFIConstraint(wvfiState, qEval, width, wvfiLowerBound),
            waistXCorridor: buildWaistXCorridorConstraint(waistXCorridor, qEval)
        },
        bounds: buildBounds(qBase, lowerBounds, upperBounds, stepBounds, width)
    };
}
