function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function vectorNorm(values = []) {
    return Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => {
        const numeric = Number(value) || 0;
        return sum + numeric * numeric;
    }, 0));
}

function normalizeSource(source) {
    return typeof source === 'string' ? source.trim().toLowerCase() : '';
}

function isJointSource(source) {
    return normalizeSource(source).startsWith('joint_');
}

function isWaistSource(source) {
    return normalizeSource(source).startsWith('waist_x_');
}

function activationDistanceForSource(source, jointActivationDistance, waistActivationDistance) {
    if (isJointSource(source)) return Math.max(0, asFinite(jointActivationDistance, 0.15));
    if (isWaistSource(source)) return Math.max(0, asFinite(waistActivationDistance, 0.08));
    return 0;
}

function normalizedGradient(gradient, width) {
    const row = new Array(width).fill(0);
    if (Array.isArray(gradient)) {
        for (let i = 0; i < Math.min(width, gradient.length); i += 1) {
            row[i] = asFinite(Number(gradient[i]), 0);
        }
    }
    const norm = vectorNorm(row);
    if (!(norm > 1e-12)) return null;
    return row.map((value) => value / norm);
}

export function buildHumanoidJogNullspaceSafetyRefineObjective({
    candidates = [],
    width = 0,
    jointActivationDistance = 0.15,
    waistActivationDistance = 0.08
} = {}) {
    const columnCount = Math.max(0, Math.floor(asFinite(width, 0)));
    const gradient = new Array(columnCount).fill(0);
    const rows = [];
    let marginBefore = Number.POSITIVE_INFINITY;
    let objectiveResidualSq = 0;

    if (columnCount <= 0 || !Array.isArray(candidates)) {
        return {
            active: false,
            reason: 'invalid_input',
            gradient,
            rows,
            rowCount: 0,
            marginBefore: Number.NaN,
            objectiveResidualNorm: 0
        };
    }

    for (const candidate of candidates) {
        const source = normalizeSource(candidate?.source);
        const margin = Number(candidate?.margin);
        if (Number.isFinite(margin)) marginBefore = Math.min(marginBefore, margin);

        const activationDistance = activationDistanceForSource(
            source,
            jointActivationDistance,
            waistActivationDistance
        );
        if (!(activationDistance > 0) || !Number.isFinite(margin) || margin >= activationDistance) {
            continue;
        }

        const row = normalizedGradient(candidate?.gradient, columnCount);
        if (!row) continue;

        const deficit = activationDistance - margin;
        objectiveResidualSq += deficit * deficit;
        for (let i = 0; i < columnCount; i += 1) {
            gradient[i] += row[i] * deficit;
        }
        rows.push({
            source,
            margin,
            activationDistance,
            deficit,
            gradient: row
        });
    }

    const gradientNorm = vectorNorm(gradient);
    return {
        active: rows.length > 0 && gradientNorm > 1e-12,
        reason: rows.length > 0 ? (gradientNorm > 1e-12 ? 'active' : 'zero_gradient') : 'inactive',
        gradient,
        gradientNorm,
        rows,
        rowCount: rows.length,
        marginBefore: Number.isFinite(marginBefore) ? marginBefore : Number.NaN,
        objectiveResidualNorm: Math.sqrt(objectiveResidualSq)
    };
}

export function evaluateHumanoidJogNullspaceSafetyRefineImprovement({
    marginBefore = Number.NaN,
    marginAfter = Number.NaN,
    objectiveBefore = Number.NaN,
    objectiveAfter = Number.NaN,
    tolerance = 1e-9
} = {}) {
    const tol = Math.max(0, asFinite(tolerance, 1e-9));
    if (Number.isFinite(marginBefore) && Number.isFinite(marginAfter) && marginAfter > marginBefore + tol) {
        return { improved: true, reason: 'margin_improved' };
    }
    if (Number.isFinite(objectiveBefore) && Number.isFinite(objectiveAfter) && objectiveAfter + tol < objectiveBefore) {
        return { improved: true, reason: 'objective_improved' };
    }
    return { improved: false, reason: 'not_improved' };
}
