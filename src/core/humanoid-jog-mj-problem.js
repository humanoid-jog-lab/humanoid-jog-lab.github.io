import { buildHumanoidJogManifoldProblem } from './humanoid-jog-manifold-problem.js';

function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function repeatValue(count, value) {
    return Array.from({ length: Math.max(0, count) }, () => value);
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

    return problem;
}

