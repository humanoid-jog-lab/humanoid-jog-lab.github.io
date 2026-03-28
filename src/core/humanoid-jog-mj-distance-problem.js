function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function cloneRows(rows) {
    return Array.isArray(rows)
        ? rows.map((row) => Array.isArray(row) ? row.slice() : [])
        : [];
}

function cloneArray(values) {
    return Array.isArray(values) ? values.slice() : [];
}

function cloneCorridorRows(corridor) {
    return cloneRows(corridor?.rows ?? corridor?.vfiRows);
}

function repeatValue(count, value) {
    return Array.from({ length: Math.max(0, count) }, () => value);
}

function vectorNorm(values) {
    return Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => {
        const numericValue = asFinite(value, 0);
        return sum + numericValue * numericValue;
    }, 0));
}

function mergeRows(...parts) {
    const rows = [];
    for (const part of parts) {
        if (!Array.isArray(part)) continue;
        rows.push(...part);
    }
    return rows;
}

export function buildHumanoidJogMJDistanceProblem(options = {}) {
    const q = cloneArray(options.q);
    const qCurrent = cloneArray(options.qCurrent);
    const width = q.length;

    const distanceResidual = [asFinite(options.distanceTarget) - asFinite(options.currentDistance)];
    const distanceJacobian = cloneRows(
        typeof options.distanceJacobian === 'function'
            ? options.distanceJacobian()
            : options.distanceJacobian
    );
    const absolutePose = cloneArray(
        typeof options.absolutePose === 'function'
            ? options.absolutePose()
            : options.absolutePose
    );
    const absoluteTarget = cloneArray(options.absoluteTarget);
    const absoluteResidual = absoluteTarget.map((value, index) => asFinite(value) - asFinite(absolutePose[index]));
    const absoluteJacobian = cloneRows(
        typeof options.absoluteJacobian === 'function'
            ? options.absoluteJacobian()
            : options.absoluteJacobian
    );
    const relativeRotation = cloneArray(
        typeof options.relativeRotation === 'function'
            ? options.relativeRotation()
            : options.relativeRotation
    );
    const relativeRotationTarget = cloneArray(options.relativeRotationTarget);
    const relativeResidual = relativeRotationTarget.map((value, index) => asFinite(value) - asFinite(relativeRotation[index]));
    const relativeJacobian = cloneRows(
        typeof options.relativeRotationJacobian === 'function'
            ? options.relativeRotationJacobian()
            : options.relativeRotationJacobian
    );
    const bodyConstraint = options.includeBodyZ && typeof options.bodyZConstraint === 'function'
        ? options.bodyZConstraint()
        : null;
    const bodyResidual = cloneArray(bodyConstraint?.residual);
    const bodyJacobian = cloneRows(bodyConstraint?.jacobian);

    const residual = [
        ...distanceResidual,
        ...absoluteResidual,
        ...relativeResidual,
        ...bodyResidual
    ];
    const jacobian = mergeRows(distanceJacobian, absoluteJacobian, relativeJacobian, bodyJacobian);

    const commandRows = distanceResidual.length;
    const absoluteRows = absoluteResidual.length;
    const relativeRows = relativeResidual.length;
    const bodyRows = bodyResidual.length;

    const absoluteModel = options.referenceModel?.absolute ?? {};
    const relativeModel = options.referenceModel?.relative ?? {};
    const bodyModel = options.referenceModel?.bodyZ ?? {};

    const lowerBounds = cloneArray(options.lowerBounds);
    const upperBounds = cloneArray(options.upperBounds);
    const stepBounds = cloneArray(options.stepBounds);
    const effectiveLower = qCurrent.map((value, index) => {
        const lower = Number.isFinite(lowerBounds[index]) ? lowerBounds[index] : Number.NEGATIVE_INFINITY;
        const step = Number.isFinite(stepBounds[index]) ? stepBounds[index] : Number.POSITIVE_INFINITY;
        return Math.max(lower, asFinite(value) - step);
    });
    const effectiveUpper = qCurrent.map((value, index) => {
        const upper = Number.isFinite(upperBounds[index]) ? upperBounds[index] : Number.POSITIVE_INFINITY;
        const step = Number.isFinite(stepBounds[index]) ? stepBounds[index] : Number.POSITIVE_INFINITY;
        return Math.min(upper, asFinite(value) + step);
    });

    const problem = {
        q,
        qCurrent,
        equalities: {
            residual: residual.slice(),
            jacobian: cloneRows(jacobian),
            residualNorm: vectorNorm(residual),
            activeResidual: residual.slice(),
            activeJacobian: cloneRows(jacobian),
            activeResidualNorm: vectorNorm(residual),
            weightedResidual: residual.slice(),
            weightedJacobian: cloneRows(jacobian),
            blocks: {
                command: { residual: distanceResidual.slice(), length: commandRows },
                absolute: { residual: absoluteResidual.slice(), length: absoluteRows },
                relative: { residual: relativeResidual.slice(), length: relativeRows },
                bodyZ: bodyRows > 0 ? { residual: bodyResidual.slice(), length: bodyRows } : null
            }
        },
        inequalities: {
            wvfiFloor: {
                enabled: Number.isFinite(options.wvfiLowerBound) && options.wvfiLowerBound > 0,
                violation: 0
            },
            waistXCorridor: {
                enabled: options?.waistXCorridor?.enabled === true,
                violation: 0,
                min: Number.isFinite(options?.waistXCorridor?.min) ? options.waistXCorridor.min : Number.NEGATIVE_INFINITY,
                max: Number.isFinite(options?.waistXCorridor?.max) ? options.waistXCorridor.max : Number.POSITIVE_INFINITY,
                tolerance: Math.max(0, asFinite(options?.waistXCorridor?.tolerance, 0)),
                vfiRows: cloneCorridorRows(options?.waistXCorridor),
                rhs: cloneArray(options?.waistXCorridor?.rhs)
            }
        },
        bounds: {
            lower: lowerBounds,
            upper: upperBounds,
            effectiveLower,
            effectiveUpper
        },
        referenceModel: {
            absolute: {
                offset: 0,
                length: commandRows + absoluteRows,
                stiffness: Math.max(0, asFinite(absoluteModel.stiffness, 1.0)),
                damping: Math.max(0, asFinite(absoluteModel.damping, 0.25))
            },
            relative: {
                offset: commandRows + absoluteRows,
                length: relativeRows,
                stiffness: Math.max(0, asFinite(relativeModel.stiffness, 1.0)),
                damping: Math.max(0, asFinite(relativeModel.damping, 0.35))
            },
            bodyZ: {
                offset: commandRows + absoluteRows + relativeRows,
                length: bodyRows,
                stiffness: Math.max(0, asFinite(bodyModel.stiffness, 0.6)),
                damping: Math.max(0, asFinite(bodyModel.damping, 0.45))
            }
        }
    };

    const absoluteBlockRows = commandRows + absoluteRows;
    problem.referenceModel.stiffnessRows = [
        ...repeatValue(absoluteBlockRows, problem.referenceModel.absolute.stiffness),
        ...repeatValue(relativeRows, problem.referenceModel.relative.stiffness),
        ...repeatValue(bodyRows, problem.referenceModel.bodyZ.stiffness)
    ];
    problem.referenceModel.dampingRows = [
        ...repeatValue(absoluteBlockRows, problem.referenceModel.absolute.damping),
        ...repeatValue(relativeRows, problem.referenceModel.relative.damping),
        ...repeatValue(bodyRows, problem.referenceModel.bodyZ.damping)
    ];

    return problem;
}

export default buildHumanoidJogMJDistanceProblem;
