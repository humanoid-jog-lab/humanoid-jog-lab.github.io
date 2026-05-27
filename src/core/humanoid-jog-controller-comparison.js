import { t } from '../i18n.js';

const CONTROLLERS = [
    { solverMode: 'proxqp', key: 'proxqp', label: 'ProxQP' },
    { solverMode: 'proxqp_w', key: 'proxqp_w', label: 'ProxQP+w' },
    { solverMode: 'proxqp_w_vfi', key: 'proxqp_w_vfi', label: 'ProxQP+w_vfi' }
];

const TASKS = [
    { mode: 'absolute', key: 'absolute', label: 'absolute', controlMode: 'absolute', includeWaist: false, keepBodyZVertical: false },
    { mode: 'decoupled-upright', key: 'decoupled_upright', label: 'decoupled-upright', controlMode: 'decoupled-upright', includeWaist: true, keepBodyZVertical: true },
    { mode: 'waist-distance', key: 'waist_distance', label: 'waist-distance', controlMode: 'distance-fullbody', includeWaist: true, keepBodyZVertical: false, useWaistCorridor: true, jogDirection: 'negative', progressKind: 'relative-distance' }
];

const POSTURES = [
    { profile: 'nominal', key: 'nominal', label: 'nominal' }
];

const EXPLICIT_TERMINAL_FAILURE_REASONS = new Set([
    'fk_fail',
    'jacobian_fail',
    'body_task_fail',
    'solver_fail',
    'command_mismatch'
]);

const DEFAULT_TARGET_X_METERS = 0.30;
const DEFAULT_TEACH_SPEED_PERCENT = 50;
const DEFAULT_MAX_TICKS_AT_FULL_SPEED = 60;
const ARM_ONLY_ABSOLUTE_TARGET_METERS = 0.075;
const ARM_ONLY_ABSOLUTE_TICK_TARGET_METERS = 0.005;
const WAIST_DISTANCE_TARGET_METERS = 0.075;
const TARGET_REACH_TOLERANCE = 5e-4;
const DEFAULT_COMPARISON_SEEDS = Object.freeze([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
export const DEFAULT_TASK_VALID_DEVIATION_TOLERANCE = 1e-6;

function asFiniteNumber(value) {
    if (value == null || value === '') return Number.NaN;
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

function percentileOf(items = [], selector = (value) => value, percentile = 0.95) {
    const values = [];
    for (const item of items) {
        const value = asFiniteNumber(selector(item));
        if (Number.isFinite(value)) values.push(value);
    }
    if (values.length === 0) return Number.NaN;
    values.sort((left, right) => left - right);
    const clamped = Math.max(0, Math.min(1, Number(percentile)));
    const index = Math.min(values.length - 1, Math.ceil(clamped * values.length) - 1);
    return values[Math.max(0, index)];
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

function controllerLabelFor(key = '', fallback = '') {
    const controller = CONTROLLERS.find((item) => item.key === key || item.solverMode === key);
    if (typeof fallback === 'string' && fallback.trim().length > 0) return fallback;
    return controller ? t(controller.label) : key || 'unknown';
}

function finiteMax(values = []) {
    const finiteValues = values.map(asFiniteNumber).filter(Number.isFinite);
    return finiteValues.length > 0 ? Math.max(...finiteValues) : Number.NaN;
}

function finiteMin(values = []) {
    const finiteValues = values.map(asFiniteNumber).filter(Number.isFinite);
    return finiteValues.length > 0 ? Math.min(...finiteValues) : Number.NaN;
}

function cleanNumber(value) {
    const numeric = asFiniteNumber(value);
    return Number.isFinite(numeric) ? Number(numeric.toPrecision(12)) : Number.NaN;
}

function sanitizeTrialKey(value = '') {
    return String(value || 'unknown').trim().replace(/[^A-Za-z0-9]+/g, '_').replace(/^_+|_+$/g, '') || 'unknown';
}

function finiteValues(items = [], selector = (value) => value) {
    const values = [];
    for (const item of items) {
        const value = asFiniteNumber(selector(item));
        if (Number.isFinite(value)) values.push(value);
    }
    return values;
}

function sampleStdOfValues(values = []) {
    const finite = values.filter(Number.isFinite);
    if (finite.length === 0) return Number.NaN;
    if (finite.length === 1) return 0;
    const mean = finite.reduce((sum, value) => sum + value, 0) / finite.length;
    const variance = finite.reduce((sum, value) => sum + (value - mean) ** 2, 0) / (finite.length - 1);
    return Math.sqrt(Math.max(0, variance));
}

function formatFixedMeanStd(values = [], digits = 3) {
    const finite = values.filter(Number.isFinite);
    if (finite.length === 0) return 'n/a';
    const mean = finite.reduce((sum, value) => sum + value, 0) / finite.length;
    const std = sampleStdOfValues(finite);
    return `${mean.toFixed(digits)} +/- ${std.toFixed(digits)}`;
}

function formatSciMeanStd(values = [], digits = 3) {
    const finite = values.filter(Number.isFinite);
    if (finite.length === 0) return 'n/a';
    const mean = finite.reduce((sum, value) => sum + value, 0) / finite.length;
    const std = sampleStdOfValues(finite);
    return `${mean.toExponential(digits)} +/- ${std.toExponential(digits)}`;
}

function terminalReasonSummary(results = []) {
    const counts = new Map();
    for (const result of results) {
        const reason = typeof result?.terminalReason === 'string' && result.terminalReason.trim().length > 0
            ? result.terminalReason.trim()
            : 'unknown';
        counts.set(reason, (counts.get(reason) || 0) + 1);
    }
    return Array.from(counts.entries()).map(([reason, count]) => `${reason}:${count}`).join('; ');
}

function resultFinalStep(result = {}) {
    const steps = Array.isArray(result?.steps) ? result.steps : [];
    return steps[steps.length - 1] || {};
}

function resultProgress(result = {}) {
    const rawProgress = Number.isFinite(asFiniteNumber(result?.finalDistanceProgress))
        ? asFiniteNumber(result.finalDistanceProgress)
        : asFiniteNumber(result?.finalXProgress);
    const targetProgress = asFiniteNumber(result?.targetXMeters);
    const fixedTickRun = result?.fixedTicks === true
        || String(result?.terminalReason || '').startsWith('fixed_ticks');
    return fixedTickRun !== true
        && result?.completed === true
        && Number.isFinite(rawProgress)
        && Number.isFinite(targetProgress)
        ? Math.min(rawProgress, targetProgress)
        : rawProgress;
}

function resultFinalValue(result = {}, resultKey = '', stepKey = '') {
    const explicit = asFiniteNumber(result?.[resultKey]);
    if (Number.isFinite(explicit)) return explicit;
    return asFiniteNumber(resultFinalStep(result)?.[stepKey]);
}

function resultMinStepValue(result = {}, stepKey = '') {
    return finiteMin((Array.isArray(result?.steps) ? result.steps : []).map((step) => step?.[stepKey]));
}

export function resolveHumanoidControllerComparisonSpeedPercent(value = DEFAULT_TEACH_SPEED_PERCENT) {
    const numeric = asFiniteNumber(value);
    if (!Number.isFinite(numeric)) return DEFAULT_TEACH_SPEED_PERCENT;
    return Math.max(1, Math.min(100, Math.round(numeric)));
}

export function resolveHumanoidControllerComparisonSeeds(value = '', options = {}) {
    const fallbackSeeds = Array.isArray(options.fallbackSeeds) && options.fallbackSeeds.length > 0
        ? options.fallbackSeeds
        : DEFAULT_COMPARISON_SEEDS;
    const normalize = (items) => {
        const seeds = [];
        const seen = new Set();
        for (const item of Array.isArray(items) ? items : []) {
            const seed = Number(item);
            if (!Number.isFinite(seed)) continue;
            const normalized = Math.floor(seed);
            if (seen.has(normalized)) continue;
            seen.add(normalized);
            seeds.push(normalized);
        }
        return seeds;
    };

    if (Array.isArray(value)) {
        const parsed = normalize(value);
        return parsed.length > 0 ? parsed : fallbackSeeds.slice();
    }

    const text = String(value ?? '').trim();
    if (!text) return fallbackSeeds.slice();

    try {
        const parsed = JSON.parse(text);
        if (Array.isArray(parsed)) {
            const seeds = normalize(parsed);
            if (seeds.length > 0) return seeds;
        }
    } catch (_) {}

    const seeds = normalize(text.split(',').map((part) => part.trim()));
    return seeds.length > 0 ? seeds : fallbackSeeds.slice();
}

export function resolveHumanoidControllerComparisonDefaultMaxTicks(speedPercent = DEFAULT_TEACH_SPEED_PERCENT) {
    const speedScale = resolveHumanoidControllerComparisonSpeedPercent(speedPercent) / 100;
    return Math.max(1, Math.ceil(DEFAULT_MAX_TICKS_AT_FULL_SPEED / Math.max(speedScale, 1e-6)));
}

export function resolveHumanoidControllerComparisonTaskValidTolerance(value = DEFAULT_TASK_VALID_DEVIATION_TOLERANCE) {
    const numeric = asFiniteNumber(value);
    if (!Number.isFinite(numeric)) return DEFAULT_TASK_VALID_DEVIATION_TOLERANCE;
    return Math.max(0, numeric);
}

function stepTaskDeviation(step = {}) {
    return asFiniteNumber(step.commandError ?? step.err?.cmd);
}

function formatTaskValidTicks(taskValues = [], tolerance = DEFAULT_TASK_VALID_DEVIATION_TOLERANCE) {
    if (!Array.isArray(taskValues) || taskValues.length === 0) return 'n/a';
    const validCount = taskValues.filter((value) => Number.isFinite(value) && value <= tolerance).length;
    return `${validCount} / ${taskValues.length}`;
}

function stepInvariantDeviation(step = {}, mode = '') {
    if (mode === 'decoupled-upright') {
        const relativeOnly = asFiniteNumber(step.relativePoseChange);
        if (Number.isFinite(relativeOnly)) return relativeOnly;
    }
    const invariant = asFiniteNumber(step.invariantError ?? step.err?.inv ?? step.relativePoseChange);
    return invariant;
}

function stepJointClearance(step = {}) {
    return asFiniteNumber(step.jointClearanceMin ?? step.dmin);
}

function stepWaistClearance(step = {}) {
    const explicit = asFiniteNumber(step.waistClearanceMin);
    if (Number.isFinite(explicit)) return explicit;
    const waist = step.waist_x || step.waistX || {};
    return finiteMin([waist.dPlus, waist.dMinus]);
}

function stepViolation(step = {}) {
    return finiteMax([
        step.maxViolation,
        step.waistXViolation,
        step.waist_x?.violation,
        step.constraintViolation
    ]);
}

function stepSolveMs(step = {}) {
    return asFiniteNumber(step.tickSolveMs ?? step.runtimeMs ?? step.timingMs?.solve);
}

function stepTaskScale(step = {}) {
    return asFiniteNumber(step.taskScale ?? step.predictiveTaskScale ?? step.snsTaskScale);
}

function stepTRSafety(step = {}) {
    const value = step.trSafetyStatus ?? step.safety?.tr_safety;
    return typeof value === 'string' && value.length > 0 ? value : '';
}

function formatMsPair(meanValue, p95Value) {
    const meanText = Number.isFinite(meanValue) ? meanValue.toFixed(1) : 'n/a';
    const p95Text = Number.isFinite(p95Value) ? p95Value.toFixed(1) : 'n/a';
    return `${meanText} / ${p95Text}`;
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
                    scenarioLabel: `${t(task.label)} | ${t(posture.label)} | ${t(controller.label)}`
                });
            }
        }
    }
    return tasks;
}

export function expandHumanoidControllerComparisonTasksForSeeds(tasks = [], seeds = DEFAULT_COMPARISON_SEEDS) {
    const safeTasks = Array.isArray(tasks) ? tasks : [];
    const safeSeeds = resolveHumanoidControllerComparisonSeeds(seeds);
    const expanded = [];
    for (const seed of safeSeeds) {
        for (const task of safeTasks) {
            const modeKey = sanitizeTrialKey(task?.mode);
            const controllerKey = sanitizeTrialKey(task?.key || task?.solverMode || 'controller');
            expanded.push({
                ...task,
                profile: 'near-limit',
                profileKey: 'near-limit',
                seed,
                nearLimitSeed: seed,
                trialId: `${modeKey}-${controllerKey}-seed${seed}`,
                scenarioLabel: `${t(task?.label || task?.mode || 'unknown')} | ${t('near-limit')} | ${t(task?.label || task?.solverMode || 'controller')} | seed ${seed}`
            });
        }
    }
    return expanded;
}

export function resolveHumanoidControllerComparisonControllerOptions({
    controllerKey = '',
    mode = '',
    includeWaist = false,
    maxTicks = 60,
    teachSpeedPercent = DEFAULT_TEACH_SPEED_PERCENT,
    targetXMeters = DEFAULT_TARGET_X_METERS,
    commandTolerance = 5e-4,
    maxStallTicks = 3,
    fixedTicks = false
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
    const speedScale = resolveHumanoidControllerComparisonSpeedPercent(teachSpeedPercent) / 100;
    const armOnlyAbsoluteTicks = Math.ceil(
        ARM_ONLY_ABSOLUTE_TARGET_METERS / Math.max(ARM_ONLY_ABSOLUTE_TICK_TARGET_METERS * speedScale, 1e-9)
    );
    const waistDistanceTicks = Math.ceil(
        WAIST_DISTANCE_TARGET_METERS / Math.max(ARM_ONLY_ABSOLUTE_TICK_TARGET_METERS * speedScale, 1e-9)
    );
    const scenarioMaxTicks = fixedTicks === true
        ? normalizedMaxTicks
        : (mode === 'absolute' && includeWaist !== true
            ? Math.min(normalizedMaxTicks, armOnlyAbsoluteTicks)
            : (mode === 'waist-distance'
                ? Math.min(normalizedMaxTicks, waistDistanceTicks)
                : normalizedMaxTicks));
    const requestedTargetXMeters = Number.isFinite(Number(targetXMeters))
        ? Math.max(0.01, Number(targetXMeters))
        : DEFAULT_TARGET_X_METERS;
    const scenarioTargetXMeters = mode === 'absolute' && includeWaist !== true
        ? ARM_ONLY_ABSOLUTE_TARGET_METERS
        : (mode === 'waist-distance'
            ? WAIST_DISTANCE_TARGET_METERS
            : requestedTargetXMeters);
    const defaultOptions = {
        maxTicks: scenarioMaxTicks,
        targetXMeters: scenarioTargetXMeters,
        commandTolerance: normalizedCommandTolerance,
        maxStallTicks: Math.min(normalizedMaxStallTicks, scenarioMaxTicks),
        jointLimitAvoidanceEnabled: true,
        qpUseInvariantEquality: true
    };

    if (controllerKey === 'proxqp' && mode === 'absolute' && includeWaist !== true) {
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

export function summarizeHumanoidControllerComparisonRows(results = [], options = {}) {
    const taskValidTolerance = resolveHumanoidControllerComparisonTaskValidTolerance(options.taskValidTolerance);
    return (Array.isArray(results) ? results : []).map((result) => {
        const steps = Array.isArray(result?.steps) ? result.steps : [];
        const mode = typeof result?.mode === 'string' ? result.mode : 'unknown';
        const taskValues = steps.map(stepTaskDeviation);
        const invariantValues = steps.map((step) => stepInvariantDeviation(step, mode));
        const jointClearances = steps.map(stepJointClearance);
        const waistClearances = steps.map(stepWaistClearance);
        const solveValues = steps.map(stepSolveMs);
        const finalStep = steps[steps.length - 1] || {};
        const controllerKey = typeof result?.controllerKey === 'string'
            ? result.controllerKey
            : (typeof result?.solverMode === 'string' ? result.solverMode : 'unknown');
        const taskScaleValues = steps.map(stepTaskScale).filter(Number.isFinite);
        const trSafetyValues = steps.map(stepTRSafety).filter(Boolean);
        const meanSolveMs = Number.isFinite(asFiniteNumber(result?.meanSolveMs))
            ? asFiniteNumber(result.meanSolveMs)
            : meanOf(solveValues);
        const p95SolveMs = Number.isFinite(asFiniteNumber(result?.p95SolveMs))
            ? asFiniteNumber(result.p95SolveMs)
            : percentileOf(solveValues);
        const rawProgress = Number.isFinite(asFiniteNumber(result?.finalDistanceProgress))
            ? asFiniteNumber(result.finalDistanceProgress)
            : asFiniteNumber(result?.finalXProgress);
        const targetProgress = asFiniteNumber(result?.targetXMeters);
        const fixedTickRun = result?.fixedTicks === true
            || String(result?.terminalReason || '').startsWith('fixed_ticks');
        const reportedProgress = fixedTickRun !== true
            && result?.completed === true
            && Number.isFinite(rawProgress)
            && Number.isFinite(targetProgress)
            ? Math.min(rawProgress, targetProgress)
            : rawProgress;

        return {
            scenario: mode,
            start_posture: typeof result?.profile === 'string' ? result.profile : 'unknown',
            controller: controllerLabelFor(controllerKey, result?.controllerLabel),
            success_ticks: result?.completed === true
                ? `yes / ${formatTickValue(result?.completionTicks)}`
                : 'no / n/a',
            task_valid_ticks: formatTaskValidTicks(taskValues, taskValidTolerance),
            progress_m: cleanNumber(reportedProgress),
            avg_task_deviation: cleanNumber(meanOf(taskValues)),
            avg_invariant_deviation: cleanNumber(meanOf(invariantValues)),
            avg_joint_clearance_rad: cleanNumber(meanOf(jointClearances)),
            min_joint_clearance_rad: cleanNumber(finiteMin(jointClearances)),
            avg_waist_clearance_m: cleanNumber(meanOf(waistClearances)),
            min_waist_clearance_m: cleanNumber(finiteMin(waistClearances)),
            max_violation: cleanNumber(finiteMax([
                result?.maxViolation,
                ...steps.map(stepViolation)
            ])),
            mean_solve_ms: cleanNumber(meanSolveMs),
            p95_solve_ms: cleanNumber(p95SolveMs),
            mean_p95_solve_ms: formatMsPair(meanSolveMs, p95SolveMs),
            final_task_deviation: cleanNumber(Number.isFinite(asFiniteNumber(result?.finalCommandError))
                ? asFiniteNumber(result.finalCommandError)
                : stepTaskDeviation(finalStep)),
            final_invariant_deviation: cleanNumber(mode === 'decoupled-upright'
                ? (Number.isFinite(asFiniteNumber(result?.finalRelativePoseChange))
                    ? asFiniteNumber(result.finalRelativePoseChange)
                    : stepInvariantDeviation(finalStep, mode))
                : (Number.isFinite(asFiniteNumber(result?.finalInvariantError))
                    ? asFiniteNumber(result.finalInvariantError)
                    : stepInvariantDeviation(finalStep, mode))),
            terminal_reason: typeof result?.terminalReason === 'string' ? result.terminalReason : 'unknown',
            task_scale: taskScaleValues.length > 0 ? cleanNumber(meanOf(taskScaleValues)) : Number.NaN,
            tr_safety: typeof result?.trSafetyStatus === 'string' && result.trSafetyStatus.length > 0
                ? result.trSafetyStatus
                : (trSafetyValues[trSafetyValues.length - 1] || 'n/a')
        };
    });
}

export function summarizeHumanoidControllerComparisonAggregateRows(results = []) {
    const safeResults = Array.isArray(results) ? results : [];
    const grouped = new Map();
    for (const result of safeResults) {
        const mode = typeof result?.mode === 'string' ? result.mode : 'unknown';
        const controllerKey = typeof result?.controllerKey === 'string'
            ? result.controllerKey
            : (typeof result?.solverMode === 'string' ? result.solverMode : 'unknown');
        const groupKey = `${mode}||${controllerKey}`;
        if (!grouped.has(groupKey)) {
            grouped.set(groupKey, {
                mode,
                controllerKey,
                controller: controllerLabelFor(controllerKey, result?.controllerLabel),
                results: []
            });
        }
        grouped.get(groupKey).results.push(result);
    }

    return Array.from(grouped.values()).map((group) => {
        const items = group.results;
        const n = items.length;
        const successCount = items.filter((result) => result?.completed === true).length;
        const meanSolveValues = finiteValues(items, (result) => result?.meanSolveMs);
        const p95SolveValues = finiteValues(items, (result) => result?.p95SolveMs);
        const finalWaistValues = finiteValues(items, (result) => resultFinalValue(result, 'finalWaistClearanceMin', 'waistClearanceMin'));
        const minWaistValues = finiteValues(items, (result) => resultMinStepValue(result, 'waistClearanceMin'));

        return {
            scenario: group.mode,
            controller: group.controller,
            n,
            success_rate: n > 0 ? successCount / n : Number.NaN,
            progress_m: formatFixedMeanStd(finiteValues(items, resultProgress), 3),
            final_joint_clearance_rad: formatFixedMeanStd(
                finiteValues(items, (result) => resultFinalValue(result, 'finalJointClearanceMin', 'jointClearanceMin')),
                4
            ),
            min_joint_clearance_rad: formatFixedMeanStd(
                finiteValues(items, (result) => resultMinStepValue(result, 'jointClearanceMin')),
                4
            ),
            final_waist_clearance_m: formatFixedMeanStd(finalWaistValues, 4),
            min_waist_clearance_m: formatFixedMeanStd(minWaistValues, 4),
            final_w: formatSciMeanStd(
                finiteValues(items, (result) => resultFinalValue(result, 'finalCCTSMW', 'cctsmW')),
                3
            ),
            final_w_vfi: formatSciMeanStd(
                finiteValues(items, (result) => resultFinalValue(result, 'finalCCTSMWvfi', 'cctsmWvfi')),
                3
            ),
            final_task_deviation: formatSciMeanStd(
                finiteValues(items, (result) => resultFinalValue(result, 'finalCommandError', 'commandError')),
                3
            ),
            final_invariant_deviation: formatSciMeanStd(
                finiteValues(items, (result) => {
                    if (result?.mode === 'decoupled-upright') {
                        const relative = asFiniteNumber(result?.finalRelativePoseChange);
                        if (Number.isFinite(relative)) return relative;
                    }
                    return resultFinalValue(result, 'finalInvariantError', 'invariantError');
                }),
                3
            ),
            mean_p95_solve_ms: `${formatFixedMeanStd(meanSolveValues, 1)} / ${formatFixedMeanStd(p95SolveValues, 1)}`,
            terminal_reasons: terminalReasonSummary(items)
        };
    });
}

export function getHumanoidControllerComparisonColumns() {
    return [
        { key: 'scenario', label: 'Scenario' },
        { key: 'controller', label: 'Controller' },
        { key: 'success_ticks', label: 'Success / ticks' },
        { key: 'task_valid_ticks', label: 'Task-valid ticks' },
        { key: 'progress_m', label: 'Progress (m)', digits: 3 },
        { key: 'avg_task_deviation', label: 'Avg. task deviation', format: 'sci', digits: 2 },
        { key: 'avg_invariant_deviation', label: 'Avg. invariant deviation', format: 'sci', digits: 2 },
        { key: 'avg_joint_clearance_rad', label: 'Avg. joint-limit clearance (rad)', digits: 3 },
        { key: 'avg_waist_clearance_m', label: 'Avg. waist-corridor clearance (m)', digits: 3 },
        { key: 'mean_p95_solve_ms', label: 'Mean / P95 solve time (ms)' }
    ];
}

export function getHumanoidControllerComparisonAggregateColumns() {
    return [
        { key: 'scenario', label: 'Scenario' },
        { key: 'controller', label: 'Controller' },
        { key: 'n', label: 'N' },
        { key: 'success_rate', label: 'Success rate', format: 'percent', digits: 1 },
        { key: 'progress_m', label: 'Progress (m)' },
        { key: 'final_joint_clearance_rad', label: 'Final joint clr. (rad)' },
        { key: 'min_joint_clearance_rad', label: 'Min joint clr. (rad)' },
        { key: 'final_waist_clearance_m', label: 'Final waist clr. (m)' },
        { key: 'min_waist_clearance_m', label: 'Min waist clr. (m)' },
        { key: 'final_w', label: 'Final w' },
        { key: 'final_w_vfi', label: 'Final w_vfi' },
        { key: 'final_task_deviation', label: 'Final task dev.' },
        { key: 'final_invariant_deviation', label: 'Final inv. dev.' },
        { key: 'mean_p95_solve_ms', label: 'Mean / P95 solve time (ms)' },
        { key: 'terminal_reasons', label: 'Terminal reasons' }
    ];
}
