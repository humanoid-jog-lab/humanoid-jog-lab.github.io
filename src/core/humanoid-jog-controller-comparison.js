const CONTROLLERS = [
    { solverMode: 'vfi', key: 'qp', label: 'new qpJ' },
    { solverMode: 'priority', key: 'nullspace', label: '零空间投影' },
    { solverMode: 'mj_restore', key: 'ours', label: 'Ours' },
    { solverMode: 'mj_restore_tr', key: 'restore_tr', label: '恢复优化(TR)' }
];

const TASKS = [
    { mode: 'absolute', key: 'absolute', label: 'absolute', controlMode: 'absolute', includeWaist: false, keepBodyZVertical: false },
    { mode: 'decoupled-upright', key: 'decoupled_upright', label: 'decoupled-upright', controlMode: 'decoupled-upright', includeWaist: true, keepBodyZVertical: true }
];

const POSTURES = [
    { profile: 'nominal', key: 'nominal', label: 'nominal' },
    { profile: 'near-limit', key: 'near_limit', label: 'near-limit' }
];

const EXPLICIT_TERMINAL_FAILURE_REASONS = new Set([
    'fk_fail',
    'jacobian_fail',
    'body_task_fail',
    'solver_fail',
    'command_mismatch'
]);

const QP_COMPARISON_COMMAND_TOLERANCE = 2e-3;
const DEFAULT_TARGET_X_METERS = 0.30;
const ARM_ONLY_ABSOLUTE_TICK_TARGET_METERS = 0.005;
const TARGET_REACH_TOLERANCE = 5e-4;

function asFiniteNumber(value) {
    return Number.isFinite(Number(value)) ? Number(value) : Number.NaN;
}

function meanOf(items = [], selector = (value) => value) {
    const values = [];
    for (const item of items) {
        const value = asFiniteNumber(selector(item));
        if (Number.isFinite(value)) values.push(value);
    }
    if (values.length === 0) return Number.NaN;
    return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function medianOf(items = [], selector = (value) => value) {
    const values = [];
    for (const item of items) {
        const value = asFiniteNumber(selector(item));
        if (Number.isFinite(value)) values.push(value);
    }
    if (values.length === 0) return Number.NaN;
    values.sort((left, right) => left - right);
    const middle = Math.floor(values.length / 2);
    if (values.length % 2 === 1) return values[middle];
    return (values[middle - 1] + values[middle]) / 2;
}

function formatTickValue(value) {
    if (!Number.isFinite(value)) return 'n/a';
    return Math.abs(value - Math.round(value)) <= 1e-9
        ? String(Math.round(value))
        : value.toFixed(1);
}

function vectorNorm(values = []) {
    if (!Array.isArray(values) || values.length === 0) return Number.NaN;
    let sumSquares = 0;
    let hasFiniteValue = false;
    for (const value of values) {
        const numericValue = asFiniteNumber(value);
        if (!Number.isFinite(numericValue)) continue;
        sumSquares += numericValue * numericValue;
        hasFiniteValue = true;
    }
    return hasFiniteValue ? Math.sqrt(sumSquares) : Number.NaN;
}

function buildControllerSummaryCell(results = []) {
    if (!Array.isArray(results) || results.length === 0) return 'n/a';
    const completionRate = results.filter((result) => result?.completed === true).length / results.length;
    const completionTicks = medianOf(
        results.filter((result) => result?.completed === true),
        (result) => result?.completionTicks
    );
    const meanSolveMs = meanOf(results, (result) => result?.meanSolveMs);
    const completionText = Number.isFinite(completionRate) ? `${(completionRate * 100).toFixed(1)}%` : 'n/a';
    const ticksText = formatTickValue(completionTicks);
    const solveText = Number.isFinite(meanSolveMs) ? `${meanSolveMs.toFixed(1)} ms` : 'n/a';
    return `${completionText} (${ticksText}, ${solveText})`;
}

export function buildHumanoidControllerComparisonTasks() {
    const tasks = [];
    for (const task of TASKS) {
        for (const posture of POSTURES) {
            for (const controller of CONTROLLERS) {
                tasks.push({
                    ...task,
                    ...posture,
                    ...controller,
                    scenarioLabel: `${task.label} | ${posture.label} | ${controller.label}`
                });
            }
        }
    }
    return tasks;
}

export function resolveHumanoidControllerComparisonControllerOptions({
    controllerKey = '',
    mode = '',
    includeWaist = false,
    maxTicks = 60,
    commandTolerance = 5e-4,
    maxStallTicks = 3
} = {}) {
    const normalizedMaxTicks = Number.isFinite(Number(maxTicks))
        ? Math.max(1, Math.floor(Number(maxTicks)))
        : 60;
    const normalizedCommandTolerance = Number.isFinite(Number(commandTolerance))
        ? Math.max(0, Number(commandTolerance))
        : 5e-4;
    const normalizedMaxStallTicks = Number.isFinite(Number(maxStallTicks))
        ? Math.max(1, Math.floor(Number(maxStallTicks)))
        : 3;
    const scenarioMaxTicks = mode === 'absolute' && includeWaist !== true
        ? Math.min(normalizedMaxTicks, 15)
        : normalizedMaxTicks;
    const targetXMeters = mode === 'absolute' && includeWaist !== true
        ? Number((scenarioMaxTicks * ARM_ONLY_ABSOLUTE_TICK_TARGET_METERS).toFixed(3))
        : DEFAULT_TARGET_X_METERS;
    const defaultOptions = {
        maxTicks: scenarioMaxTicks,
        targetXMeters,
        commandTolerance: normalizedCommandTolerance,
        maxStallTicks: Math.min(normalizedMaxStallTicks, scenarioMaxTicks),
        jointLimitAvoidanceEnabled: true,
        qpUseInvariantEquality: true
    };

    if (controllerKey === 'qp') {
        const isArmOnlyAbsolute = mode === 'absolute' && includeWaist !== true;
        return {
            ...defaultOptions,
            commandTolerance: Math.max(normalizedCommandTolerance, QP_COMPARISON_COMMAND_TOLERANCE),
            maxStallTicks: scenarioMaxTicks,
            jointLimitAvoidanceEnabled: isArmOnlyAbsolute ? false : true,
            qpUseInvariantEquality: isArmOnlyAbsolute ? false : true
        };
    }

    if (controllerKey === 'nullspace') {
        return {
            ...defaultOptions,
            jointLimitAvoidanceEnabled: false
        };
    }

    return defaultOptions;
}

export function evaluateHumanoidControllerComparisonStep(options = {}) {
    const stepProgress = Number(options.stepProgress);
    const minEffectiveProgress = Number.isFinite(Number(options.minEffectiveProgress))
        ? Math.max(1e-8, Number(options.minEffectiveProgress))
        : 1e-5;
    const stallTicks = Number.isFinite(Number(options.stallTicks))
        ? Math.max(0, Math.floor(Number(options.stallTicks)))
        : 0;
    const maxStallTicks = Number.isFinite(Number(options.maxStallTicks))
        ? Math.max(1, Math.floor(Number(options.maxStallTicks)))
        : 3;
    const status = options.status && typeof options.status === 'object' ? options.status : {};
    const reason = typeof status.reason === 'string' && status.reason.length > 0
        ? status.reason
        : 'unknown';
    const progressed = Number.isFinite(stepProgress) && stepProgress >= minEffectiveProgress;
    const nextStallTicks = progressed ? 0 : stallTicks + 1;
    const explicitFailure = EXPLICIT_TERMINAL_FAILURE_REASONS.has(reason);
    const ok = !explicitFailure;
    const shouldStop = explicitFailure || nextStallTicks >= maxStallTicks;

    return {
        reason,
        progressed,
        ok,
        shouldStop,
        nextStallTicks
    };
}

export function evaluateHumanoidControllerComparisonCompletion(options = {}) {
    const xProgress = asFiniteNumber(options.xProgress);
    const targetXMeters = Number.isFinite(Number(options.targetXMeters))
        ? Math.max(1e-8, Number(options.targetXMeters))
        : 0.30;
    const commandError = asFiniteNumber(options.commandError);
    const commandTolerance = Number.isFinite(Number(options.commandTolerance))
        ? Math.max(0, Number(options.commandTolerance))
        : 5e-4;
    const bodyError = asFiniteNumber(options.bodyError);
    const bodyTolerance = Number.isFinite(Number(options.bodyTolerance))
        ? Math.max(0, Number(options.bodyTolerance))
        : 5e-5;
    const waistXViolation = asFiniteNumber(options.waistXViolation);

    const reachedTarget = Number.isFinite(xProgress) && xProgress + TARGET_REACH_TOLERANCE >= targetXMeters;
    const commandConverged = Number.isFinite(commandError) && commandError <= commandTolerance;
    const bodyConverged = !Number.isFinite(bodyError) || bodyError <= bodyTolerance;
    const waistFeasible = !Number.isFinite(waistXViolation) || waistXViolation <= 1e-12;
    const completed = reachedTarget && commandConverged && bodyConverged && waistFeasible;
    const progressRatio = Number.isFinite(xProgress) ? Math.max(0, Math.min(1, xProgress / targetXMeters)) : Number.NaN;

    return {
        reachedTarget,
        commandConverged,
        bodyConverged,
        waistFeasible,
        completed,
        progressRatio
    };
}

export function evaluateHumanoidControllerComparisonRelativePoseChange(options = {}) {
    const initialRelativePose = Array.isArray(options.initialRelativePose) ? options.initialRelativePose : null;
    const currentRelativePose = Array.isArray(options.currentRelativePose) ? options.currentRelativePose : null;
    const signedPoseError = typeof options.signedPoseError === 'function' ? options.signedPoseError : null;
    const poseErrorNorm = typeof options.poseErrorNorm === 'function' ? options.poseErrorNorm : vectorNorm;
    if (!initialRelativePose || !currentRelativePose || !signedPoseError) return Number.NaN;
    const error = signedPoseError(initialRelativePose, currentRelativePose);
    const norm = poseErrorNorm(error);
    return asFiniteNumber(norm);
}

export function summarizeHumanoidControllerComparisonRows(results = []) {
    const buckets = new Map();
    for (const result of Array.isArray(results) ? results : []) {
        const mode = typeof result?.mode === 'string' ? result.mode : 'unknown';
        const posture = typeof result?.profile === 'string' ? result.profile : 'unknown';
        const controllerKey = typeof result?.controllerKey === 'string' ? result.controllerKey : 'unknown';
        const bucketKey = `${mode}::${posture}`;
        if (!buckets.has(bucketKey)) {
            buckets.set(bucketKey, {
                mode,
                start_posture: posture,
                _controllerRuns: new Map()
            });
        }
        const row = buckets.get(bucketKey);
        if (!row._controllerRuns.has(controllerKey)) {
            row._controllerRuns.set(controllerKey, []);
        }
        row._controllerRuns.get(controllerKey).push(result);
    }
    return Array.from(buckets.values()).map((bucket) => {
        const row = {
            mode: bucket.mode,
            start_posture: bucket.start_posture
        };
        for (const controller of CONTROLLERS) {
            row[controller.key] = buildControllerSummaryCell(bucket._controllerRuns.get(controller.key) || []);
        }
        return row;
    });
}

export function getHumanoidControllerComparisonColumns() {
    return [
        { key: 'mode', label: 'Mode' },
        { key: 'start_posture', label: 'Start posture' },
        { key: 'qp', label: 'new qpJ' },
        { key: 'nullspace', label: 'Null-space' },
        { key: 'ours', label: 'Ours' },
        { key: 'restore_tr', label: '恢复优化(TR)' }
    ];
}
