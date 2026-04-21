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

    return problem;
}
