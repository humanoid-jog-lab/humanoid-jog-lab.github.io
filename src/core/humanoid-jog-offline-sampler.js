import { performance } from 'node:perf_hooks';

import { buildHumanoidJogMJProblem } from './humanoid-jog-mj-problem.js';
import { restoreHumanoidJogMJTrustRegion } from './humanoid-jog-mj-restoration-tr.js';

const DEFAULT_Q_LENGTH = 20;
const DEFAULT_WAIST_COUNT = 4;
const DEFAULT_ACTIVE_ARM_COUNT = 14;
const DEFAULT_ACTIVE_FULL_BODY_COUNT = 18;
const DEFAULT_JOINT_LIMIT = Math.PI;

function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function vectorNorm(values = []) {
    return Math.sqrt((Array.isArray(values) ? values : []).reduce((sum, value) => {
        const finite = asFinite(value, 0);
        return sum + finite * finite;
    }, 0));
}

function cloneNumberArray(values = [], fallbackLength = 0) {
    const source = Array.isArray(values) ? values : [];
    const length = Math.max(source.length, fallbackLength);
    return Array.from({ length }, (_, index) => asFinite(source[index], 0));
}

function directionSign(direction = 'positive') {
    const normalized = String(direction || '').trim().toLowerCase();
    return normalized === 'negative' || normalized === '-' || normalized.endsWith('-') ? -1 : 1;
}

function resolveActiveSlice({ q = [], includeWaist = false } = {}) {
    const activeStart = includeWaist ? 0 : Math.min(DEFAULT_WAIST_COUNT, q.length);
    const defaultCount = includeWaist ? DEFAULT_ACTIVE_FULL_BODY_COUNT : DEFAULT_ACTIVE_ARM_COUNT;
    const activeCount = Math.max(0, Math.min(defaultCount, q.length - activeStart));
    return { activeStart, activeCount };
}

function resolveCommandIndex({ axis = 'x', activeCount = 0, includeWaist = false, commandIndex } = {}) {
    if (Number.isInteger(commandIndex) && commandIndex >= 0 && commandIndex < activeCount) {
        return commandIndex;
    }
    const normalized = String(axis || 'x').trim().toLowerCase();
    const waistAxisMap = { x: 0, y: 1, z: 2 };
    const armAxisMap = { x: 0, y: 1, z: 2 };
    const mapped = includeWaist
        ? waistAxisMap[normalized]
        : armAxisMap[normalized];
    return Math.min(Math.max(0, mapped ?? 0), Math.max(0, activeCount - 1));
}

function buildBounds(activeCount, {
    lowerBounds,
    upperBounds,
    jointLimit = DEFAULT_JOINT_LIMIT
} = {}) {
    const lower = new Array(activeCount).fill(-Math.abs(asFinite(jointLimit, DEFAULT_JOINT_LIMIT)));
    const upper = new Array(activeCount).fill(Math.abs(asFinite(jointLimit, DEFAULT_JOINT_LIMIT)));
    for (let index = 0; index < activeCount; index += 1) {
        if (Number.isFinite(lowerBounds?.[index])) lower[index] = lowerBounds[index];
        if (Number.isFinite(upperBounds?.[index])) upper[index] = upperBounds[index];
    }
    return { lower, upper };
}

function computeDmin(qActive = [], lowerBounds = [], upperBounds = []) {
    let best = Number.POSITIVE_INFINITY;
    for (let index = 0; index < qActive.length; index += 1) {
        const q = asFinite(qActive[index], 0);
        const lower = Number.isFinite(lowerBounds[index]) ? q - lowerBounds[index] : Number.POSITIVE_INFINITY;
        const upper = Number.isFinite(upperBounds[index]) ? upperBounds[index] - q : Number.POSITIVE_INFINITY;
        best = Math.min(best, lower, upper);
    }
    return Number.isFinite(best) ? Math.max(0, best) : Number.NaN;
}

function buildWaistXCorridor(activeCount, {
    includeWaist = false,
    min = -0.5,
    max = 0.1,
    eta = 1,
    activationThreshold = 0.08,
    tolerance = 0
} = {}) {
    if (!includeWaist || activeCount <= 0) {
        return { enabled: false };
    }
    const upperRow = new Array(activeCount).fill(0);
    const lowerRow = new Array(activeCount).fill(0);
    upperRow[0] = 1;
    lowerRow[0] = -1;
    return {
        enabled: true,
        min,
        max,
        tolerance,
        evaluateX: (qActive) => asFinite(qActive?.[0], 0),
        vfiRows: [upperRow, lowerRow],
        rhs: [Number.POSITIVE_INFINITY, Number.POSITIVE_INFINITY],
        vfiConstraints: [
            {
                row: upperRow.map((value) => -value),
                distance: Number.NaN,
                threshold: activationThreshold
            },
            {
                row: lowerRow.map((value) => -value),
                distance: Number.NaN,
                threshold: activationThreshold
            }
        ],
        eta
    };
}

function withWaistDistances(corridor, qActive) {
    if (corridor?.enabled !== true) return corridor;
    const value = asFinite(qActive?.[0], 0);
    const dPlus = Number.isFinite(corridor.max) ? corridor.max - value : Number.NaN;
    const dMinus = Number.isFinite(corridor.min) ? value - corridor.min : Number.NaN;
    return {
        ...corridor,
        rhs: [
            Number.isFinite(dPlus) ? Math.max(0, asFinite(corridor.eta, 1) * dPlus) : Number.POSITIVE_INFINITY,
            Number.isFinite(dMinus) ? Math.max(0, asFinite(corridor.eta, 1) * dMinus) : Number.POSITIVE_INFINITY
        ],
        vfiConstraints: [
            { ...corridor.vfiConstraints[0], distance: dPlus },
            { ...corridor.vfiConstraints[1], distance: dMinus }
        ]
    };
}

function waistDiagnostics(corridor, qActive) {
    if (corridor?.enabled !== true) {
        return {
            enabled: false,
            p0x: Number.NaN,
            dPlus: Number.NaN,
            dMinus: Number.NaN,
            violation: 0,
            limit: Number.NaN
        };
    }
    const p0x = asFinite(qActive?.[0], 0);
    const dPlus = Number.isFinite(corridor.max) ? corridor.max - p0x : Number.NaN;
    const dMinus = Number.isFinite(corridor.min) ? p0x - corridor.min : Number.NaN;
    return {
        enabled: true,
        p0x,
        dPlus,
        dMinus,
        violation: Math.max(
            0,
            Number.isFinite(dPlus) ? -dPlus : 0,
            Number.isFinite(dMinus) ? -dMinus : 0
        ),
        limit: corridor.max
    };
}

function blockResidualNorm(problem, blockName) {
    return vectorNorm(problem?.equalities?.blocks?.[blockName]?.residual);
}

function buildSurrogateProblem({
    qActive,
    qInitialActive,
    targetValue,
    relativeReference,
    commandIndex,
    invariantIndex,
    lowerBounds,
    upperBounds,
    stepBounds,
    includeWaist,
    waistXCorridor,
    options
}) {
    const width = qActive.length;
    const commandRow = new Array(width).fill(0);
    commandRow[commandIndex] = 1;
    const invariantRow = new Array(width).fill(0);
    invariantRow[invariantIndex] = 1;
    const activeWaistCorridor = withWaistDistances(waistXCorridor, qActive);

    return buildHumanoidJogMJProblem({
        solverMode: options.solverMode,
        q: qActive,
        qCurrent: qInitialActive,
        absoluteTarget: [1, targetValue],
        relativeTarget: [1, relativeReference],
        absolutePose: () => [1, asFinite(qActive[commandIndex], 0)],
        relativePose: () => [1, asFinite(qActive[invariantIndex], 0)],
        absoluteJacobian: () => [
            new Array(width).fill(0),
            commandRow
        ],
        relativeJacobian: () => [
            new Array(width).fill(0),
            invariantRow
        ],
        lowerBounds,
        upperBounds,
        stepBounds,
        enableJointLimitVFI: options.enableJointLimitVFI !== false,
        jointLimitEta: options.jointLimitEta,
        jointLimitEdgeStopDistance: options.jointLimitEdgeStopDistance,
        jointLimitActivationThreshold: options.jointLimitActivationThreshold,
        mjRestoreSafetyPushEnabled: options.safetyPushEnabled,
        safetyPushTargetFraction: options.safetyPushTargetFraction,
        safetyPushMaxDistanceStep: options.safetyPushMaxDistanceStep,
        safetyPushIncludeWaistCorridor: options.safetyPushIncludeWaistCorridor,
        safetyPushWaistCorridorTargetDistance: options.safetyPushWaistCorridorTargetDistance,
        safetyPushWaistCorridorMaxDistanceStep: options.safetyPushWaistCorridorMaxDistanceStep,
        safetyPushWaistCorridorWeightScale: options.safetyPushWaistCorridorWeightScale,
        waistXCorridor: includeWaist ? activeWaistCorridor : null,
        referenceModel: {
            absolute: { stiffness: 1, damping: 0.2 },
            relative: { stiffness: 1, damping: 0.2 }
        }
    });
}

function defaultStepSolver({
    qActive,
    commandDelta,
    commandIndex,
    invariantIndex,
    lowerBounds,
    upperBounds,
    stepBounds,
    includeWaist,
    waistXCorridor,
    options
}) {
    const qInitialActive = qActive.slice();
    const targetValue = asFinite(qActive[commandIndex], 0) + commandDelta;
    const relativeReference = asFinite(qActive[invariantIndex], 0);
    const evaluateProblem = (candidate) => buildSurrogateProblem({
        qActive: candidate,
        qInitialActive,
        targetValue,
        relativeReference,
        commandIndex,
        invariantIndex,
        lowerBounds,
        upperBounds,
        stepBounds,
        includeWaist,
        waistXCorridor,
        options
    });
    const startMs = performance.now();
    const result = restoreHumanoidJogMJTrustRegion({
        qInitial: qInitialActive,
        evaluateProblem,
        maxIterations: options.maxIterations,
        damping: options.damping,
        maxDampingRetries: options.maxDampingRetries,
        dampingGrowthFactor: options.dampingGrowthFactor,
        initialTrustRadius: options.initialTrustRadius,
        maxTrustRadius: options.maxTrustRadius,
        equalityTolerance: options.equalityTolerance,
        inequalityTolerance: options.inequalityTolerance,
        safetyPushEnabled: options.safetyPushEnabled,
        safetyPushWeight: options.safetyPushWeight,
        safetyPushActivationResidual: options.safetyPushActivationResidual,
        safetyPushPoseHoldTolerance: options.safetyPushPoseHoldTolerance,
        safetyBiasDuringRestore: options.safetyBiasDuringRestore,
        safetyPushPostConvergenceEnabled: options.safetyPushPostConvergenceEnabled
    });
    const solveMs = performance.now() - startMs;
    const qApplied = Array.isArray(result.qApplied) ? result.qApplied.slice() : qInitialActive.slice();
    const finalProblem = result.currentProblem || evaluateProblem(qApplied);
    return {
        qApplied,
        result,
        finalProblem,
        timingMs: { solve: solveMs, total: solveMs }
    };
}

export function runHumanoidJogOfflineSampler(options = {}) {
    const initialQ = cloneNumberArray(options.initialQ, DEFAULT_Q_LENGTH);
    const steps = Math.max(1, Math.floor(asFinite(options.steps, 10)));
    const speedPercent = Math.max(0, asFinite(options.speedPercent, 50));
    const includeWaist = options.includeWaist !== false;
    const solverMode = String(options.solverMode || 'mj_restore_tr_safe');
    const direction = String(options.direction || 'positive');
    const axis = String(options.axis || 'x');
    const sign = directionSign(direction);
    const baseStep = Math.max(0, asFinite(options.baseStep, 0.02));
    const commandDelta = sign * baseStep * (speedPercent / 100);
    const { activeStart, activeCount } = resolveActiveSlice({ q: initialQ, includeWaist });
    const commandIndex = resolveCommandIndex({
        axis,
        activeCount,
        includeWaist,
        commandIndex: options.commandIndex
    });
    const invariantIndex = Math.min(
        Math.max(0, Number.isInteger(options.invariantIndex) ? options.invariantIndex : commandIndex + 1),
        Math.max(0, activeCount - 1)
    );
    const { lower, upper } = buildBounds(activeCount, options);
    const stepLimit = Math.max(Math.abs(commandDelta) * 2, asFinite(options.stepLimit, 0.04));
    const stepBounds = Array.from({ length: activeCount }, () => stepLimit);
    const waistXCorridor = buildWaistXCorridor(activeCount, {
        includeWaist,
        min: Number.isFinite(options.waistXMin) ? options.waistXMin : -0.5,
        max: Number.isFinite(options.waistXMax) ? options.waistXMax : 0.1,
        eta: Number.isFinite(options.waistXEta) ? options.waistXEta : 1,
        activationThreshold: Number.isFinite(options.waistXActivationThreshold) ? options.waistXActivationThreshold : 0.08,
        tolerance: Number.isFinite(options.waistXTolerance) ? options.waistXTolerance : 0
    });
    const solverOptions = {
        solverMode,
        enableJointLimitVFI: options.enableJointLimitVFI !== false,
        jointLimitEta: Number.isFinite(options.jointLimitEta) ? options.jointLimitEta : 5,
        jointLimitEdgeStopDistance: Number.isFinite(options.jointLimitEdgeStopDistance) ? options.jointLimitEdgeStopDistance : 0,
        jointLimitActivationThreshold: Number.isFinite(options.jointLimitActivationThreshold) ? options.jointLimitActivationThreshold : 0.15,
        safetyPushEnabled: options.safetyPushEnabled ?? solverMode.includes('safe'),
        safetyPushWeight: Number.isFinite(options.safetyPushWeight) ? options.safetyPushWeight : 0.002,
        safetyPushActivationResidual: Number.isFinite(options.safetyPushActivationResidual) ? options.safetyPushActivationResidual : 1e-5,
        safetyPushPoseHoldTolerance: Number.isFinite(options.safetyPushPoseHoldTolerance) ? options.safetyPushPoseHoldTolerance : 2e-6,
        safetyPushTargetFraction: Number.isFinite(options.safetyPushTargetFraction) ? options.safetyPushTargetFraction : 0.25,
        safetyPushMaxDistanceStep: Number.isFinite(options.safetyPushMaxDistanceStep) ? options.safetyPushMaxDistanceStep : 0.01,
        safetyPushIncludeWaistCorridor: options.safetyPushIncludeWaistCorridor !== false,
        safetyPushWaistCorridorTargetDistance: Number.isFinite(options.safetyPushWaistCorridorTargetDistance) ? options.safetyPushWaistCorridorTargetDistance : 0.025,
        safetyPushWaistCorridorMaxDistanceStep: Number.isFinite(options.safetyPushWaistCorridorMaxDistanceStep) ? options.safetyPushWaistCorridorMaxDistanceStep : 0.001,
        safetyPushWaistCorridorWeightScale: Number.isFinite(options.safetyPushWaistCorridorWeightScale) ? options.safetyPushWaistCorridorWeightScale : 0.25,
        maxIterations: Number.isInteger(options.maxIterations) ? options.maxIterations : 80,
        damping: Number.isFinite(options.damping) ? options.damping : 1e-6,
        maxDampingRetries: Number.isInteger(options.maxDampingRetries) ? options.maxDampingRetries : 6,
        dampingGrowthFactor: Number.isFinite(options.dampingGrowthFactor) ? options.dampingGrowthFactor : 10,
        initialTrustRadius: Number.isFinite(options.initialTrustRadius) ? options.initialTrustRadius : 0.05,
        maxTrustRadius: Number.isFinite(options.maxTrustRadius) ? options.maxTrustRadius : 0.2,
        equalityTolerance: Number.isFinite(options.equalityTolerance) ? options.equalityTolerance : 1e-6,
        inequalityTolerance: Number.isFinite(options.inequalityTolerance) ? options.inequalityTolerance : 1e-9,
        safetyBiasDuringRestore: options.safetyBiasDuringRestore ?? solverMode.includes('safe'),
        safetyPushPostConvergenceEnabled: options.safetyPushPostConvergenceEnabled ?? !solverMode.includes('safe')
    };

    const stepSolver = typeof options.stepSolver === 'function' ? options.stepSolver : defaultStepSolver;
    const samples = [];
    let qFull = initialQ.slice();
    const runStartMs = performance.now();
    for (let stepIndex = 1; stepIndex <= steps; stepIndex += 1) {
        const qActive = qFull.slice(activeStart, activeStart + activeCount);
        const solved = stepSolver({
            qActive,
            qFull: qFull.slice(),
            commandDelta,
            commandIndex,
            invariantIndex,
            lowerBounds: lower,
            upperBounds: upper,
            stepBounds,
            includeWaist,
            waistXCorridor,
            options: solverOptions
        });
        const qAppliedActive = Array.isArray(solved?.qApplied) ? solved.qApplied.slice() : qActive.slice();
        const qNext = qFull.slice();
        for (let i = 0; i < activeCount; i += 1) {
            qNext[activeStart + i] = asFinite(qAppliedActive[i], qActive[i]);
        }
        const finalProblem = solved?.finalProblem;
        const result = solved?.result || {};
        const activeStep = qAppliedActive.map((value, index) => asFinite(value, 0) - asFinite(qActive[index], 0));
        const waistState = waistDiagnostics(withWaistDistances(waistXCorridor, qAppliedActive), qAppliedActive);
        samples.push({
            index: stepIndex,
            status: result.converged === true ? 'ok' : 'failed',
            reason: String(result.reason || 'unknown'),
            q: qNext,
            err: {
                cmd: blockResidualNorm(finalProblem, 'absolute'),
                inv: Math.max(
                    blockResidualNorm(finalProblem, 'relative'),
                    blockResidualNorm(finalProblem, 'bodyZ'),
                    asFinite(result.bestConstraintViolation, 0)
                )
            },
            step: vectorNorm(activeStep),
            iters: {
                total: Math.max(0, asFinite(result.iterations, 0)),
                restore: Math.max(0, asFinite(result.iterations, 0)),
                polish: null
            },
            dmin: computeDmin(qAppliedActive, lower, upper),
            waist_x: waistState,
            safety: {
                tr_safety: String(result.safetyStatus || 'skipped'),
                before: asFinite(result.safetyResidualBefore, Number.NaN),
                after: asFinite(result.safetyResidualAfter, Number.NaN),
                accepted: Math.max(0, asFinite(result.safetyAcceptedIterations, 0))
            },
            timingMs: {
                solve: asFinite(solved?.timingMs?.solve, 0),
                total: asFinite(solved?.timingMs?.total, asFinite(solved?.timingMs?.solve, 0))
            }
        });
        qFull = qNext;
        if (options.stopOnFailure === true && result.converged !== true) break;
    }
    const totalMs = performance.now() - runStartMs;
    return {
        meta: {
            version: 1,
            source: 'offline-core',
            usesBrowser: false,
            generatedAt: options.generatedAt || new Date().toISOString(),
            command: {
                mode: String(options.mode || 'absolute-target'),
                solverMode,
                axis,
                direction,
                steps,
                speedPercent,
                commandDelta,
                includeWaist
            },
            activeSlice: { activeStart, activeCount },
            parameters: {
                ...solverOptions,
                waistXMin: waistXCorridor.min,
                waistXMax: waistXCorridor.max
            }
        },
        initialQ,
        samples,
        summary: {
            sampleCount: samples.length,
            okCount: samples.filter((sample) => sample.status === 'ok').length,
            totalMs,
            meanSolveMs: samples.reduce((sum, sample) => sum + asFinite(sample.timingMs.solve, 0), 0) / Math.max(1, samples.length),
            finalQ: samples.length > 0 ? samples[samples.length - 1].q.slice() : initialQ.slice()
        }
    };
}

function csvCell(value) {
    if (value == null) return '';
    if (typeof value === 'number') return Number.isFinite(value) ? String(value) : '';
    const text = String(value);
    return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

export function serializeHumanoidJogOfflineSamplesToCsv(result = {}) {
    const rows = Array.isArray(result.samples) ? result.samples : [];
    const header = [
        'index',
        'status',
        'reason',
        'err_cmd',
        'err_inv',
        'step',
        'iters_total',
        'dmin',
        'waist_x',
        'waist_d_plus',
        'waist_d_minus',
        'tr_safety',
        'timing_solve_ms'
    ];
    const lines = [header.join(',')];
    for (const sample of rows) {
        lines.push([
            sample.index,
            sample.status,
            sample.reason,
            sample.err?.cmd,
            sample.err?.inv,
            sample.step,
            sample.iters?.total,
            sample.dmin,
            sample.waist_x?.p0x,
            sample.waist_x?.dPlus,
            sample.waist_x?.dMinus,
            sample.safety?.tr_safety,
            sample.timingMs?.solve
        ].map(csvCell).join(','));
    }
    return `${lines.join('\n')}\n`;
}
