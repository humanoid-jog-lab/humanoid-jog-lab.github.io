import { buildHumanoidJogManifoldProblem } from './humanoid-jog-manifold-problem.js';

function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function repeatValue(count, value) {
    return Array.from({ length: Math.max(0, count) }, () => value);
}

function buildJointLimitVFIRows({ q = [], lowerBounds = [], upperBounds = [], stepBounds = [], eta = 1 } = {}) {
    const qEval = Array.isArray(q) ? q : [];
    const width = qEval.length;
    const rows = [];
    const rhs = [];
    const gain = Math.max(0, asFinite(eta, 1));

    for (let index = 0; index < width; index++) {
        const qi = asFinite(qEval[index], 0);
        const lower = Number.isFinite(lowerBounds?.[index]) ? lowerBounds[index] : Number.NEGATIVE_INFINITY;
        const upper = Number.isFinite(upperBounds?.[index]) ? upperBounds[index] : Number.POSITIVE_INFINITY;
        const maxStep = Number.isFinite(stepBounds?.[index])
            ? Math.max(0, stepBounds[index])
            : Number.POSITIVE_INFINITY;

        if (Number.isFinite(upper)) {
            const upperSlack = Math.max(0, upper - qi);
            const upperStep = Math.min(maxStep, gain * upperSlack);
            const row = new Array(width).fill(0);
            row[index] = 1;
            rows.push(row);
            rhs.push(upperStep);
        }

        if (Number.isFinite(lower)) {
            const lowerSlack = Math.max(0, qi - lower);
            const lowerStep = Math.min(maxStep, gain * lowerSlack);
            const row = new Array(width).fill(0);
            row[index] = -1;
            rows.push(row);
            rhs.push(lowerStep);
        }
    }

    return {
        enabled: rows.length > 0,
        rows,
        rhs,
        count: rows.length
    };
}

function buildHumanoidJogMJSafetyObjective({
    problem,
    lowerBounds = [],
    upperBounds = [],
    jointLimitActivationThreshold = 0.15,
    safetyPushTargetFraction = 0.25,
    safetyPushMaxDistanceStep = 0.01
} = {}) {
    const qEval = Array.isArray(problem?.q) ? problem.q : [];
    const width = qEval.length;
    const rows = [];
    const targets = [];
    const maxDistanceStep = Math.max(0, asFinite(safetyPushMaxDistanceStep, 0.01));
    const targetFraction = Math.max(0, asFinite(safetyPushTargetFraction, 0.25));
    const jointTargetMargin = Math.max(0, asFinite(jointLimitActivationThreshold, 0.15)) * targetFraction;

    const addDistanceTarget = (row, distance, targetMargin) => {
        if (!Array.isArray(row) || row.length !== width || !Number.isFinite(distance)) return;
        const margin = Math.max(0, asFinite(targetMargin, 0));
        const residual = margin - Math.max(0, distance);
        if (residual <= 1e-12) return;
        rows.push(row.map((value) => asFinite(value, 0)));
        targets.push(Math.min(maxDistanceStep, residual));
    };

    if (jointTargetMargin > 0 && maxDistanceStep > 0) {
        for (let index = 0; index < width; index++) {
            const qi = asFinite(qEval[index], 0);
            const lower = Number.isFinite(lowerBounds?.[index]) ? lowerBounds[index] : Number.NEGATIVE_INFINITY;
            const upper = Number.isFinite(upperBounds?.[index]) ? upperBounds[index] : Number.POSITIVE_INFINITY;
            if (Number.isFinite(upper)) {
                const row = new Array(width).fill(0);
                row[index] = -1;
                addDistanceTarget(row, upper - qi, jointTargetMargin);
            }
            if (Number.isFinite(lower)) {
                const row = new Array(width).fill(0);
                row[index] = 1;
                addDistanceTarget(row, qi - lower, jointTargetMargin);
            }
        }
    }

    const corridor = problem?.inequalities?.waistXCorridor;
    if (corridor?.enabled === true && width > 0 && maxDistanceStep > 0) {
        const value = Number.isFinite(corridor.value) ? corridor.value : Number.NaN;
        const constraints = Array.isArray(corridor.vfiConstraints) ? corridor.vfiConstraints : [];
        if (constraints.length > 0) {
            for (const constraint of constraints) {
                const targetMargin = Math.max(0, asFinite(constraint?.threshold, 0)) * targetFraction;
                addDistanceTarget(constraint?.row, constraint?.distance, targetMargin);
            }
        } else if (Number.isFinite(value)) {
            const vfiRows = Array.isArray(corridor.vfiRows) ? corridor.vfiRows : [];
            let rowIndex = 0;
            if (Number.isFinite(corridor.max) && Array.isArray(vfiRows[rowIndex])) {
                const row = vfiRows[rowIndex].map((entry) => -asFinite(entry, 0));
                const targetMargin = Math.max(0, asFinite(corridor.threshold, asFinite(corridor.activationThreshold, 0))) * targetFraction;
                addDistanceTarget(row, corridor.max - value, targetMargin);
                rowIndex += 1;
            }
            if (Number.isFinite(corridor.min) && Array.isArray(vfiRows[rowIndex])) {
                const row = vfiRows[rowIndex].map((entry) => -asFinite(entry, 0));
                const targetMargin = Math.max(0, asFinite(corridor.threshold, asFinite(corridor.activationThreshold, 0))) * targetFraction;
                addDistanceTarget(row, value - corridor.min, targetMargin);
            }
        }
    }

    return {
        enabled: rows.length > 0,
        rows,
        targets,
        residual: targets.slice(),
        residualNorm: Math.sqrt(targets.reduce((sum, value) => sum + value * value, 0)),
        count: rows.length
    };
}

export function buildHumanoidJogMJProblem(options = {}) {
    const problem = buildHumanoidJogManifoldProblem(options);
    const absoluteRows = problem?.equalities?.blocks?.absolute?.length ?? 0;
    const relativeRows = problem?.equalities?.blocks?.relative?.length ?? 0;
    const bodyRows = problem?.equalities?.blocks?.bodyZ?.length ?? 0;

    const absoluteModel = options?.referenceModel?.absolute ?? {};
    const relativeModel = options?.referenceModel?.relative ?? {};
    const bodyModel = options?.referenceModel?.bodyZ ?? {};

    problem.referenceModel = {
        absolute: {
            offset: 0,
            length: absoluteRows,
            stiffness: Math.max(0, asFinite(absoluteModel.stiffness, 1.0)),
            damping: Math.max(0, asFinite(absoluteModel.damping, 0.25))
        },
        relative: {
            offset: absoluteRows,
            length: relativeRows,
            stiffness: Math.max(0, asFinite(relativeModel.stiffness, 1.0)),
            damping: Math.max(0, asFinite(relativeModel.damping, 0.35))
        },
        bodyZ: {
            offset: absoluteRows + relativeRows,
            length: bodyRows,
            stiffness: Math.max(0, asFinite(bodyModel.stiffness, 0.6)),
            damping: Math.max(0, asFinite(bodyModel.damping, 0.45))
        }
    };

    problem.referenceModel.stiffnessRows = [
        ...repeatValue(absoluteRows, problem.referenceModel.absolute.stiffness),
        ...repeatValue(relativeRows, problem.referenceModel.relative.stiffness),
        ...repeatValue(bodyRows, problem.referenceModel.bodyZ.stiffness)
    ];
    problem.referenceModel.dampingRows = [
        ...repeatValue(absoluteRows, problem.referenceModel.absolute.damping),
        ...repeatValue(relativeRows, problem.referenceModel.relative.damping),
        ...repeatValue(bodyRows, problem.referenceModel.bodyZ.damping)
    ];
    problem.inequalities.jointLimitVFI = options?.enableJointLimitVFI === false
        ? {
            enabled: false,
            rows: [],
            rhs: [],
            count: 0
        }
        : buildJointLimitVFIRows({
            q: problem?.q,
            lowerBounds: options?.lowerBounds,
            upperBounds: options?.upperBounds,
            stepBounds: options?.stepBounds,
            eta: options?.jointLimitEta
        });
    problem.safetyObjective = options?.mjRestoreSafetyPushEnabled === false
        ? {
            enabled: false,
            rows: [],
            targets: [],
            residual: [],
            residualNorm: 0,
            count: 0
        }
        : buildHumanoidJogMJSafetyObjective({
            problem,
            lowerBounds: options?.lowerBounds,
            upperBounds: options?.upperBounds,
            jointLimitActivationThreshold: options?.jointLimitActivationThreshold,
            safetyPushTargetFraction: options?.safetyPushTargetFraction,
            safetyPushMaxDistanceStep: options?.safetyPushMaxDistanceStep
        });

    return problem;
}
