#!/usr/bin/env node

import fs from 'node:fs/promises';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

import { chromium } from 'playwright';

import {
    computePearsonCorrelation,
    formatBenchmarkMarkdownTable,
    summarizePairedMethodDelta,
    summarizeSustainedCommandEpisodes
} from '../src/core/humanoid-jog-benchmark-metrics.js';

const REPO_ROOT = '/home/echoz/dq_vizlab';
const DEFAULT_BASE_URL = process.env.BASE_URL || 'http://127.0.0.1:4173';
const DEFAULT_EPISODES_HOLD = Number.parseInt(process.env.BENCHMARK_EPISODES_HOLD || '8', 10);
const DEFAULT_EPISODES_TRANSITION = Number.parseInt(process.env.BENCHMARK_EPISODES_TRANSITION || '6', 10);

const METHOD_PRESETS = {
    tpIk: { id: 'tp-ik', label: 'TP-IK', solver: 'priority', branching: false, predictive: false, nextMode: 'none' },
    reactiveQp: { id: 'reactive-qp', label: 'Reactive-QP', solver: 'vfi', branching: false, predictive: false, nextMode: 'none' },
    reactiveQpMb: { id: 'reactive-qp-mb', label: 'Reactive-QP+MB', solver: 'vfi', branching: true, predictive: false, nextMode: 'none' },
    predictiveQp: { id: 'predictive-qp', label: 'Predictive-QP', solver: 'vfi', branching: true, predictive: true, nextMode: 'none' },
    predictiveQpFta: { id: 'predictive-qp-fta', label: 'Predictive-QP+FTA', solver: 'vfi', branching: true, predictive: true, nextMode: 'relative-distance' }
};

const HOLD_METHODS = [
    METHOD_PRESETS.tpIk,
    METHOD_PRESETS.reactiveQp,
    METHOD_PRESETS.reactiveQpMb,
    METHOD_PRESETS.predictiveQp,
    METHOD_PRESETS.predictiveQpFta
];

const TRANSITION_METHODS = [
    METHOD_PRESETS.tpIk,
    METHOD_PRESETS.reactiveQp,
    METHOD_PRESETS.reactiveQpMb,
    METHOD_PRESETS.predictiveQp,
    METHOD_PRESETS.predictiveQpFta
];

const TRANSITION_CORE_METHOD_IDS = new Set([
    METHOD_PRESETS.reactiveQpMb.id,
    METHOD_PRESETS.predictiveQp.id,
    METHOD_PRESETS.predictiveQpFta.id
]);

async function waitForHttp(url, timeoutMs = 120000) {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
        try {
            const response = await fetch(url);
            if (response.ok) return;
        } catch (_) {}
        await delay(500);
    }
    throw new Error(`Timed out waiting for ${url}`);
}

async function startServerIfNeeded() {
    if (process.env.BASE_URL) {
        await waitForHttp(DEFAULT_BASE_URL);
        return null;
    }

    const child = spawn(
        'npm',
        ['run', 'dev', '--', '--host', '127.0.0.1', '--port', '4173', '--strictPort'],
        {
            cwd: REPO_ROOT,
            stdio: ['ignore', 'pipe', 'pipe'],
            detached: true
        }
    );

    child.stdout.on('data', (chunk) => {
        process.stdout.write(chunk);
    });
    child.stderr.on('data', (chunk) => {
        process.stderr.write(chunk);
    });

    await waitForHttp(DEFAULT_BASE_URL);
    return child;
}

async function stopServer(child) {
    if (!child) return;
    const signalGroup = (signal) => {
        try {
            process.kill(-child.pid, signal);
            return true;
        } catch (_) {
            return false;
        }
    };
    const exitPromise = new Promise((resolve) => {
        child.once('exit', resolve);
        child.once('close', resolve);
    });
    if (!signalGroup('SIGTERM')) {
        child.kill('SIGTERM');
    }
    const exited = await Promise.race([
        exitPromise.then(() => true),
        delay(1500).then(() => false)
    ]);
    if (!exited) {
        if (!signalGroup('SIGKILL')) {
            child.kill('SIGKILL');
        }
        await Promise.race([
            exitPromise,
            delay(1500)
        ]);
    }
}

async function ensureHumanoidPageReady(page) {
    await page.goto(DEFAULT_BASE_URL, { waitUntil: 'domcontentloaded' });
    await page.waitForFunction(
        () => Boolean(
            window.visualizer &&
            window.humanoidMode === true &&
            Array.isArray(window.humanoidQ) &&
            window.humanoidQ.length > 0 &&
            window.DQModule?.DQWrapper
        ),
        { timeout: 180000 }
    );
    await page.evaluate(() => {
        window.visualizer.setTeachMultiplicationOrder?.('current-first');
        const toggle = document.getElementById('teach-multiply-order-toggle');
        if (toggle) toggle.checked = false;
    });
}

async function runScenario(page, scenarioName, method, episodes) {
    return page.evaluate(
        async ({ scenarioName: innerScenarioName, method: innerMethod, episodes: innerEpisodes }) => {
            const viz = window.visualizer;
            const DQ = window.DQModule?.DQWrapper;

            const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

            const clearBranchMemory = () => {
                if (viz?._humanoidTaskEquivalentBranchMemory?.clear) {
                    viz._humanoidTaskEquivalentBranchMemory.clear();
                }
            };

            const dispatchSelectValue = (id, value) => {
                const element = document.getElementById(id);
                if (!element) return;
                element.value = value;
                element.dispatchEvent(new Event('change', { bubbles: true }));
            };

            const setMethod = (methodConfig) => {
                viz.setHumanoidJogSolverMode?.(methodConfig.solver);
                viz.setHumanoidVFIStepBackend?.('qp');
                viz.setHumanoidVFITaskEquivalentBranching?.(methodConfig.branching);
                viz.setHumanoidPredictiveJogEnabled?.(methodConfig.predictive);
                viz.setHumanoidPredictiveNextMode?.(methodConfig.nextMode || 'none');
                if (viz?.vfiConfig) {
                    // Benchmark all predictive variants under the same rollout budget and tie policy.
                    viz.vfiConfig.predictiveTimeBudgetMs = 80;
                    viz.vfiConfig.predictiveReadinessTimeBudgetMs = 80;
                    viz.vfiConfig.predictiveReserveTieTolerance = 1e-6;
                    viz.vfiConfig.predictiveReadinessRolloutSteps = 4;
                    viz.vfiConfig.predictiveReadinessFirstStepWeight = 3;
                    viz.vfiConfig.taskEquivalentBranchScales = [0, 0.5, 1.0, 1.5];
                    viz.vfiConfig.taskEquivalentBranchSeedStep = 0.045;
                }
                dispatchSelectValue('humanoid-jog-solver-select', methodConfig.solver);
                dispatchSelectValue('humanoid-vfi-step-backend-select', 'qp');
                const predictiveToggle = document.getElementById('humanoid-predictive-jog-toggle');
                if (predictiveToggle) predictiveToggle.checked = Boolean(methodConfig.predictive);
                dispatchSelectValue('humanoid-predictive-next-mode-select', methodConfig.nextMode || 'none');
                clearBranchMemory();
            };

            const getJointLimits = () => viz._getHumanoidJointLimits?.() || [];

            const getLimitBounds = (limit) => {
                if (!limit || typeof limit !== 'object') {
                    return [Number.NaN, Number.NaN];
                }
                return [Number(limit.lower), Number(limit.upper)];
            };

            const getMinJointMarginRatio = () => {
                const limits = getJointLimits();
                let minRatio = Number.POSITIVE_INFINITY;
                for (let index = 0; index < Math.min(limits.length, window.humanoidQ.length); index += 1) {
                    const [lower, upper] = getLimitBounds(limits[index]);
                    const q = window.humanoidQ[index];
                    if (!Number.isFinite(lower) || !Number.isFinite(upper) || !Number.isFinite(q) || upper <= lower) {
                        continue;
                    }
                    const range = upper - lower;
                    const ratio = Math.min((q - lower) / range, (upper - q) / range);
                    minRatio = Math.min(minRatio, ratio);
                }
                return Number.isFinite(minRatio) ? minRatio : Number.NaN;
            };

            const dqTranslation = (dqArray) => {
                if (!Array.isArray(dqArray) || !DQ) return [Number.NaN, Number.NaN, Number.NaN];
                const dq = DQ.createFromArray(dqArray);
                const translation = DQ.getTranslation(dq);
                return [
                    Number(translation?.[1]) || 0,
                    Number(translation?.[2]) || 0,
                    Number(translation?.[3]) || 0
                ];
            };

            const getAbsoluteTranslation = () => dqTranslation(viz._getHumanoidAbsolutePose(window.humanoidQ));

            const getRelativeDistance = () => {
                const dqArray = viz._getHumanoidRelativePose(window.humanoidQ);
                if (!Array.isArray(dqArray) || !DQ) return Number.NaN;
                const dq = DQ.createFromArray(dqArray);
                const translation = DQ.getTranslation(dq);
                return Math.hypot(
                    Number(translation?.[1]) || 0,
                    Number(translation?.[2]) || 0,
                    Number(translation?.[3]) || 0
                );
            };

            const getDistanceReadiness = (includeWaist = false, diagnostics = null) => {
                const selectorReadiness = Number.isFinite(diagnostics?.readiness)
                    ? diagnostics.readiness
                    : Number.NaN;
                const rolloutReadiness = viz._computeHumanoidPredictiveDistanceReadiness?.(window.humanoidQ, includeWaist);
                return {
                    selectorReadiness,
                    rolloutReadiness: Number.isFinite(rolloutReadiness) ? rolloutReadiness : Number.NaN,
                    effectiveReadiness: Number.isFinite(selectorReadiness)
                        ? selectorReadiness
                        : (Number.isFinite(rolloutReadiness) ? rolloutReadiness : Number.NaN)
                };
            };

            const createRng = (seed) => {
                let state = (seed >>> 0) || 1;
                return () => {
                    state = (1664525 * state + 1013904223) >>> 0;
                    return state / 0x100000000;
                };
            };

            const resetHumanoidState = (seed, profile = 'near-limit') => {
                const initial = Array.isArray(window.customRobotConfig?.initial_q)
                    ? window.customRobotConfig.initial_q.slice()
                    : window.humanoidQ.slice();
                const limits = getJointLimits();
                const rng = createRng(seed + 1);
                const waistCount = viz.humanoidWaistLength || 0;
                const leftCount = viz.humanoidLeftArmLength || 0;
                const rightCount = viz.humanoidRightArmLength || 0;
                const armEnd = waistCount + leftCount + rightCount;
                const startIndex = profile === 'future-focused' ? 0 : waistCount;
                for (let index = startIndex; index < armEnd; index += 1) {
                    const [lower, upper] = getLimitBounds(limits[index]);
                    if (!Number.isFinite(lower) || !Number.isFinite(upper) || upper <= lower) continue;
                    const range = upper - lower;
                    if (profile === 'near-limit') {
                        if (rng() < 0.45) {
                            const band = 0.05 + 0.10 * rng();
                            initial[index] = rng() < 0.5
                                ? lower + range * band
                                : upper - range * band;
                        }
                    } else if (profile === 'transition') {
                        const span = 0.22 * (rng() - 0.5);
                        initial[index] = Math.min(upper, Math.max(lower, initial[index] + range * span));
                    } else if (profile === 'future-focused') {
                        const centerBias = rng() < 0.5 ? 0.18 : 0.82;
                        const noise = 0.06 * (rng() - 0.5);
                        initial[index] = Math.min(
                            upper,
                            Math.max(lower, lower + range * (centerBias + noise))
                        );
                        if (index < waistCount && rng() < 0.5) {
                            initial[index] = Math.min(
                                upper,
                                Math.max(lower, initial[index] + range * 0.08 * (rng() - 0.5))
                            );
                        }
                    }
                }
                for (let index = 0; index < window.humanoidQ.length && index < initial.length; index += 1) {
                    window.humanoidQ[index] = initial[index];
                }
                viz.setHumanoidJoints(window.humanoidQ);
                window.updateGlobalVariables?.();
                window.updateRobotPose?.();
                clearBranchMemory();
            };

            const readStepRecord = (runtimeMs) => {
                const diagnostics = viz.lastHumanoidJogDiagnostics || {};
                const status = viz.lastIKSolveStatus || {};
                return {
                    ok: status.converged !== false,
                    runtimeMs,
                    diagnosticLabel: diagnostics.diagnosticLabel || (status.converged === false ? 'step-fail' : 'step-ok'),
                    feasibleHorizon: Number.isFinite(diagnostics.feasibleHorizon) ? diagnostics.feasibleHorizon : Number.NaN,
                    minReserve: Number.isFinite(diagnostics.minReserve) ? diagnostics.minReserve : Number.NaN,
                    branchSwitched: diagnostics.branchSwitched === true,
                    minJointMarginRatio: getMinJointMarginRatio()
                };
            };

            const setBenchmarkBranchDiversity = (expanded = false) => {
                if (!viz?.vfiConfig) return;
                viz.vfiConfig.taskEquivalentBranchScales = expanded
                    ? [0, 1, 2, 3]
                    : [0, 0.5, 1.0, 1.5];
                viz.vfiConfig.taskEquivalentBranchSeedStep = expanded ? 0.08 : 0.045;
            };

            const runAbsoluteHoldEpisode = async (seed, methodConfig) => {
                resetHumanoidState(seed, 'near-limit');
                setMethod(methodConfig);
                setBenchmarkBranchDiversity(false);
                dispatchSelectValue('dual-arm-control-mode-select', 'absolute');
                await sleep(10);

                const start = getAbsoluteTranslation();
                const steps = [];
                for (let tick = 0; tick < 12; tick += 1) {
                    const t0 = performance.now();
                    viz.moveDualArmAbsolutePoseAlongAxis('x', 'positive');
                    const runtimeMs = performance.now() - t0;
                    const record = readStepRecord(runtimeMs);
                    steps.push(record);
                    if (!record.ok) break;
                }
                const end = getAbsoluteTranslation();
                const sustainableTicks = steps.findIndex((step) => !step.ok);
                const lastStep = steps[steps.length - 1] || {};
                return {
                    sustainableTicks: sustainableTicks === -1 ? steps.length : sustainableTicks,
                    targetTicks: 12,
                    meanRuntimeMs: steps.reduce((sum, step) => sum + step.runtimeMs, 0) / Math.max(steps.length, 1),
                    minJointMarginRatio: steps.reduce(
                        (minValue, step) => Number.isFinite(step.minJointMarginRatio) ? Math.min(minValue, step.minJointMarginRatio) : minValue,
                        Number.POSITIVE_INFINITY
                    ),
                    absoluteProgress: end[0] - start[0],
                    avgFeasibleHorizon: steps.reduce((sum, step) => sum + (Number.isFinite(step.feasibleHorizon) ? step.feasibleHorizon : 0), 0)
                        / Math.max(steps.filter((step) => Number.isFinite(step.feasibleHorizon)).length, 1),
                    avgReserve: steps.reduce((sum, step) => sum + (Number.isFinite(step.minReserve) ? step.minReserve : 0), 0)
                        / Math.max(steps.filter((step) => Number.isFinite(step.minReserve)).length, 1),
                    branchSwitchCount: steps.filter((step) => step.branchSwitched).length,
                    terminalLabel: lastStep.diagnosticLabel || 'unknown'
                };
            };

            const runTransitionEpisode = async (seed, methodConfig) => {
                resetHumanoidState(seed, 'transition');
                setMethod(methodConfig);
                setBenchmarkBranchDiversity(false);
                dispatchSelectValue('dual-arm-control-mode-select', 'absolute');
                await sleep(10);

                const task1Start = getAbsoluteTranslation();
                const task1Steps = [];
                for (let tick = 0; tick < 8; tick += 1) {
                    const t0 = performance.now();
                    viz.moveDualArmAbsolutePoseAlongAxis('x', 'positive');
                    const runtimeMs = performance.now() - t0;
                    const record = readStepRecord(runtimeMs);
                    task1Steps.push(record);
                    if (!record.ok) break;
                }
                const task1End = getAbsoluteTranslation();
                const task1Readiness = getDistanceReadiness(false, viz.lastHumanoidJogDiagnostics || {});

                dispatchSelectValue('dual-arm-control-mode-select', 'distance');
                await sleep(10);
                const distanceBefore = getRelativeDistance();
                const task2Steps = [];
                let distanceAfterFirst = Number.NaN;
                for (let tick = 0; tick < 8; tick += 1) {
                    const t0 = performance.now();
                    viz.moveDualArmRelativeDistance('negative', false);
                    const runtimeMs = performance.now() - t0;
                    const record = readStepRecord(runtimeMs);
                    task2Steps.push(record);
                    const distanceNow = getRelativeDistance();
                    if (tick === 0) distanceAfterFirst = distanceNow;
                    if (!record.ok) break;
                }
                const distanceAfter = getRelativeDistance();
                const lastTask1 = task1Steps[task1Steps.length - 1] || {};
                const lastTask2 = task2Steps[task2Steps.length - 1] || {};
                const task1FailureIndex = task1Steps.findIndex((step) => !step.ok);
                const task2FailureIndex = task2Steps.findIndex((step) => !step.ok);

                return {
                    task1Ticks: task1FailureIndex === -1 ? task1Steps.length : task1FailureIndex,
                    task1TargetTicks: 8,
                    task1MeanRuntimeMs: task1Steps.reduce((sum, step) => sum + step.runtimeMs, 0) / Math.max(task1Steps.length, 1),
                    task1Progress: task1End[0] - task1Start[0],
                    task1AvgFeasibleHorizon: task1Steps.reduce((sum, step) => sum + (Number.isFinite(step.feasibleHorizon) ? step.feasibleHorizon : 0), 0)
                        / Math.max(task1Steps.filter((step) => Number.isFinite(step.feasibleHorizon)).length, 1),
                    task1AvgReserve: task1Steps.reduce((sum, step) => sum + (Number.isFinite(step.minReserve) ? step.minReserve : 0), 0)
                        / Math.max(task1Steps.filter((step) => Number.isFinite(step.minReserve)).length, 1),
                    task1SelectorReadiness: task1Readiness.selectorReadiness,
                    task1RolloutReadiness: task1Readiness.rolloutReadiness,
                    task1TerminalReadiness: task1Readiness.effectiveReadiness,
                    task1TerminalLabel: lastTask1.diagnosticLabel || 'unknown',
                    task2Ticks: task2FailureIndex === -1 ? task2Steps.length : task2FailureIndex,
                    task2TargetTicks: 8,
                    task2MeanRuntimeMs: task2Steps.reduce((sum, step) => sum + step.runtimeMs, 0) / Math.max(task2Steps.length, 1),
                    task2FirstStepClosing: Number.isFinite(distanceBefore) && Number.isFinite(distanceAfterFirst)
                        ? distanceBefore - distanceAfterFirst
                        : Number.NaN,
                    task2DistanceReduction: Number.isFinite(distanceBefore) && Number.isFinite(distanceAfter)
                        ? distanceBefore - distanceAfter
                        : Number.NaN,
                    task2TerminalLabel: lastTask2.diagnosticLabel || 'unknown'
                };
            };

            const runFullbodyFutureTransitionEpisode = async (seed, methodConfig) => {
                resetHumanoidState(seed, 'future-focused');
                setMethod(methodConfig);
                setBenchmarkBranchDiversity(false);
                dispatchSelectValue('dual-arm-control-mode-select', 'decoupled');
                await sleep(10);

                const commandSequence = [
                    { axis: 'x', direction: 'positive' },
                    { axis: 'x', direction: 'positive' },
                    { axis: 'x', direction: 'positive' },
                    { axis: 'y', direction: seed % 2 === 0 ? 'positive' : 'negative' },
                    { axis: 'y', direction: seed % 2 === 0 ? 'positive' : 'negative' },
                    { axis: 'z', direction: 'positive' },
                    { axis: 'x', direction: 'positive' },
                    { axis: 'x', direction: 'positive' }
                ];

                const task1Start = getAbsoluteTranslation();
                const task1Steps = [];
                for (const command of commandSequence) {
                    const t0 = performance.now();
                    viz.moveHumanoidAbsolutePoseAlongAxisDecoupled(command.axis, command.direction);
                    const runtimeMs = performance.now() - t0;
                    const record = readStepRecord(runtimeMs);
                    task1Steps.push(record);
                    if (!record.ok) break;
                }
                const task1End = getAbsoluteTranslation();
                const task1Readiness = getDistanceReadiness(true, viz.lastHumanoidJogDiagnostics || {});

                dispatchSelectValue('dual-arm-control-mode-select', 'distance-fullbody');
                await sleep(10);
                const distanceBefore = getRelativeDistance();
                const task2Steps = [];
                let distanceAfterFirst = Number.NaN;
                for (let tick = 0; tick < 8; tick += 1) {
                    const t0 = performance.now();
                    viz.moveDualArmRelativeDistance('negative', true);
                    const runtimeMs = performance.now() - t0;
                    const record = readStepRecord(runtimeMs);
                    task2Steps.push(record);
                    const distanceNow = getRelativeDistance();
                    if (tick === 0) distanceAfterFirst = distanceNow;
                    if (!record.ok) break;
                }
                const distanceAfter = getRelativeDistance();
                const lastTask1 = task1Steps[task1Steps.length - 1] || {};
                const lastTask2 = task2Steps[task2Steps.length - 1] || {};
                const task1FailureIndex = task1Steps.findIndex((step) => !step.ok);
                const task2FailureIndex = task2Steps.findIndex((step) => !step.ok);

                return {
                    task1Ticks: task1FailureIndex === -1 ? task1Steps.length : task1FailureIndex,
                    task1TargetTicks: commandSequence.length,
                    task1MeanRuntimeMs: task1Steps.reduce((sum, step) => sum + step.runtimeMs, 0) / Math.max(task1Steps.length, 1),
                    task1Progress: task1End[0] - task1Start[0],
                    task1AvgFeasibleHorizon: task1Steps.reduce((sum, step) => sum + (Number.isFinite(step.feasibleHorizon) ? step.feasibleHorizon : 0), 0)
                        / Math.max(task1Steps.filter((step) => Number.isFinite(step.feasibleHorizon)).length, 1),
                    task1AvgReserve: task1Steps.reduce((sum, step) => sum + (Number.isFinite(step.minReserve) ? step.minReserve : 0), 0)
                        / Math.max(task1Steps.filter((step) => Number.isFinite(step.minReserve)).length, 1),
                    task1SelectorReadiness: task1Readiness.selectorReadiness,
                    task1RolloutReadiness: task1Readiness.rolloutReadiness,
                    task1TerminalReadiness: task1Readiness.effectiveReadiness,
                    task1TerminalLabel: lastTask1.diagnosticLabel || 'unknown',
                    task2Ticks: task2FailureIndex === -1 ? task2Steps.length : task2FailureIndex,
                    task2TargetTicks: 8,
                    task2MeanRuntimeMs: task2Steps.reduce((sum, step) => sum + step.runtimeMs, 0) / Math.max(task2Steps.length, 1),
                    task2FirstStepClosing: Number.isFinite(distanceBefore) && Number.isFinite(distanceAfterFirst)
                        ? distanceBefore - distanceAfterFirst
                        : Number.NaN,
                    task2DistanceReduction: Number.isFinite(distanceBefore) && Number.isFinite(distanceAfter)
                        ? distanceBefore - distanceAfter
                        : Number.NaN,
                    task2TerminalLabel: lastTask2.diagnosticLabel || 'unknown'
                };
            };

            const runSingleStepFutureTransitionEpisode = async (seed, methodConfig) => {
                resetHumanoidState(seed, 'future-focused');
                setMethod(methodConfig);
                setBenchmarkBranchDiversity(true);
                dispatchSelectValue('dual-arm-control-mode-select', 'decoupled');
                await sleep(10);

                const task1Start = getAbsoluteTranslation();
                const t0 = performance.now();
                viz.moveHumanoidAbsolutePoseAlongAxisDecoupled('x', 'positive');
                const task1RuntimeMs = performance.now() - t0;
                const task1Record = readStepRecord(task1RuntimeMs);
                const task1Diagnostics = viz.lastHumanoidJogDiagnostics || {};
                const task1End = getAbsoluteTranslation();
                const task1Readiness = getDistanceReadiness(true, task1Diagnostics);

                dispatchSelectValue('dual-arm-control-mode-select', 'distance-fullbody');
                await sleep(10);
                const distanceBefore = getRelativeDistance();
                const task2Steps = [];
                let distanceAfterFirst = Number.NaN;
                for (let tick = 0; tick < 6; tick += 1) {
                    const task2Start = performance.now();
                    viz.moveDualArmRelativeDistance('negative', true);
                    const runtimeMs = performance.now() - task2Start;
                    const record = readStepRecord(runtimeMs);
                    task2Steps.push(record);
                    const distanceNow = getRelativeDistance();
                    if (tick === 0) distanceAfterFirst = distanceNow;
                    if (!record.ok) break;
                }
                const distanceAfter = getRelativeDistance();
                const lastTask2 = task2Steps[task2Steps.length - 1] || {};
                const task2FailureIndex = task2Steps.findIndex((step) => !step.ok);

                return {
                    task1Ticks: task1Record.ok ? 1 : 0,
                    task1TargetTicks: 1,
                    task1MeanRuntimeMs: task1RuntimeMs,
                    task1Progress: task1End[0] - task1Start[0],
                    task1AvgFeasibleHorizon: Number.isFinite(task1Record.feasibleHorizon) ? task1Record.feasibleHorizon : Number.NaN,
                    task1AvgReserve: Number.isFinite(task1Record.minReserve) ? task1Record.minReserve : Number.NaN,
                    task1SelectorReadiness: task1Readiness.selectorReadiness,
                    task1RolloutReadiness: task1Readiness.rolloutReadiness,
                    task1TerminalReadiness: task1Readiness.effectiveReadiness,
                    task1SelectedBranch: typeof task1Diagnostics.branchSelectedLabel === 'string'
                        ? task1Diagnostics.branchSelectedLabel
                        : 'none',
                    task1TerminalLabel: task1Record.diagnosticLabel || 'unknown',
                    task2Ticks: task2FailureIndex === -1 ? task2Steps.length : task2FailureIndex,
                    task2TargetTicks: 6,
                    task2MeanRuntimeMs: task2Steps.reduce((sum, step) => sum + step.runtimeMs, 0) / Math.max(task2Steps.length, 1),
                    task2FirstStepClosing: Number.isFinite(distanceBefore) && Number.isFinite(distanceAfterFirst)
                        ? distanceBefore - distanceAfterFirst
                        : Number.NaN,
                    task2DistanceReduction: Number.isFinite(distanceBefore) && Number.isFinite(distanceAfter)
                        ? distanceBefore - distanceAfter
                        : Number.NaN,
                    task2TerminalLabel: lastTask2.diagnosticLabel || 'unknown'
                };
            };

            const episodes = [];
            for (let seed = 0; seed < innerEpisodes; seed += 1) {
                if (innerScenarioName === 'absolute-hold-near-limit') {
                    episodes.push(await runAbsoluteHoldEpisode(seed, innerMethod));
                } else if (innerScenarioName === 'fullbody-future-transition') {
                    episodes.push(await runFullbodyFutureTransitionEpisode(seed, innerMethod));
                } else if (innerScenarioName === 'single-step-future-transition') {
                    episodes.push(await runSingleStepFutureTransitionEpisode(seed, innerMethod));
                } else {
                    episodes.push(await runTransitionEpisode(seed, innerMethod));
                }
            }
            return episodes;
        },
        { scenarioName, method, episodes }
    );
}

function summarizeHoldScenario(method, episodes) {
    const summary = summarizeSustainedCommandEpisodes(episodes, {
        ticksKey: 'sustainableTicks',
        targetKey: 'targetTicks',
        labelKey: 'terminalLabel',
        numericKeys: [
            'meanRuntimeMs',
            'minJointMarginRatio',
            'absoluteProgress',
            'avgFeasibleHorizon',
            'avgReserve',
            'branchSwitchCount'
        ]
    });

    return {
        methodId: method.id,
        method: method.label,
        episodes: summary.episodeCount,
        successRate: summary.successRate,
        meanTicks: summary.meanTicks,
        meanRuntimeMs: summary.numericMeans.meanRuntimeMs,
        meanMargin: summary.numericMeans.minJointMarginRatio,
        meanProgress: summary.numericMeans.absoluteProgress,
        meanHorizon: summary.numericMeans.avgFeasibleHorizon,
        meanReserve: summary.numericMeans.avgReserve,
        meanSwitches: summary.numericMeans.branchSwitchCount,
        labels: summary.labelCounts
    };
}

function summarizeTransitionScenario(method, episodes) {
    const task1 = summarizeSustainedCommandEpisodes(episodes, {
        ticksKey: 'task1Ticks',
        targetKey: 'task1TargetTicks',
        labelKey: 'task1TerminalLabel',
        numericKeys: ['task1MeanRuntimeMs', 'task1Progress', 'task1AvgFeasibleHorizon', 'task1AvgReserve', 'task1TerminalReadiness']
    });
    const task2 = summarizeSustainedCommandEpisodes(episodes, {
        ticksKey: 'task2Ticks',
        targetKey: 'task2TargetTicks',
        labelKey: 'task2TerminalLabel',
        numericKeys: ['task2MeanRuntimeMs', 'task2FirstStepClosing', 'task2DistanceReduction']
    });

    return {
        methodId: method.id,
        method: method.label,
        episodes: task1.episodeCount,
        task1SuccessRate: task1.successRate,
        task1Progress: task1.numericMeans.task1Progress,
        task1RuntimeMs: task1.numericMeans.task1MeanRuntimeMs,
        task1TerminalReadiness: task1.numericMeans.task1TerminalReadiness,
        task2SuccessRate: task2.successRate,
        task2FirstClosing: task2.numericMeans.task2FirstStepClosing,
        task2DistanceReduction: task2.numericMeans.task2DistanceReduction,
        task2RuntimeMs: task2.numericMeans.task2MeanRuntimeMs,
        task1Labels: task1.labelCounts,
        task2Labels: task2.labelCounts
    };
}

function computeTransitionCorrelations(methodRuns = [], { requireTask1Success = false } = {}) {
    const samples = methodRuns
        .filter(({ method }) => TRANSITION_CORE_METHOD_IDS.has(method.id))
        .flatMap(({ episodes }) => episodes)
        .filter((episode) => {
            if (!requireTask1Success) return true;
            const task1Ticks = Number(episode?.task1Ticks);
            const task1TargetTicks = Number(episode?.task1TargetTicks);
            return Number.isFinite(task1Ticks) && Number.isFinite(task1TargetTicks) && task1Ticks >= task1TargetTicks;
        });

    return {
        sampleCount: samples.length,
        readinessVsFirstClose: computePearsonCorrelation(
            samples,
            (episode) => episode.task1TerminalReadiness,
            (episode) => episode.task2FirstStepClosing
        ),
        readinessVsTotalClose: computePearsonCorrelation(
            samples,
            (episode) => episode.task1TerminalReadiness,
            (episode) => episode.task2DistanceReduction
        )
    };
}

function formatCorrelationLine(label, value, sampleCount) {
    if (!Number.isFinite(value)) {
        return `${label}: n/a (samples=${sampleCount})`;
    }
    return `${label}: ${value.toFixed(4)} (samples=${sampleCount})`;
}

async function main() {
    const server = await startServerIfNeeded();
    let browser;
    try {
        browser = await chromium.launch({
            headless: process.env.PLAYWRIGHT_HEADLESS !== '0',
            args: ['--use-angle=swiftshader', '--ignore-gpu-blocklist']
        });
        const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
        await ensureHumanoidPageReady(page);

        const holdRuns = [];
        for (const method of HOLD_METHODS) {
            const episodes = await runScenario(page, 'absolute-hold-near-limit', method, DEFAULT_EPISODES_HOLD);
            holdRuns.push({ method, episodes });
        }
        const holdRows = holdRuns.map(({ method, episodes }) => summarizeHoldScenario(method, episodes));

        const transitionRuns = [];
        for (const method of TRANSITION_METHODS) {
            const episodes = await runScenario(page, 'absolute-to-distance-transition', method, DEFAULT_EPISODES_TRANSITION);
            transitionRuns.push({ method, episodes });
        }
        const transitionRows = transitionRuns.map(({ method, episodes }) => summarizeTransitionScenario(method, episodes));

        const futureFocusedRuns = [];
        for (const method of TRANSITION_METHODS) {
            const episodes = await runScenario(page, 'fullbody-future-transition', method, DEFAULT_EPISODES_TRANSITION);
            futureFocusedRuns.push({ method, episodes });
        }
        const futureFocusedRows = futureFocusedRuns.map(({ method, episodes }) => summarizeTransitionScenario(method, episodes));
        const pregraspRuns = [];
        for (const method of TRANSITION_METHODS) {
            const episodes = await runScenario(page, 'single-step-future-transition', method, DEFAULT_EPISODES_TRANSITION);
            pregraspRuns.push({ method, episodes });
        }
        const pregraspRows = pregraspRuns.map(({ method, episodes }) => summarizeTransitionScenario(method, episodes));
        const transitionCoreRows = transitionRows.filter((row) => TRANSITION_CORE_METHOD_IDS.has(row.methodId));
        const futureFocusedCoreRows = futureFocusedRows.filter((row) => TRANSITION_CORE_METHOD_IDS.has(row.methodId));
        const pregraspCoreRows = pregraspRows.filter((row) => TRANSITION_CORE_METHOD_IDS.has(row.methodId));
        const transitionCorrelations = computeTransitionCorrelations(transitionRuns);
        const transitionSuccessfulCorrelations = computeTransitionCorrelations(transitionRuns, { requireTask1Success: true });
        const futureFocusedCorrelations = computeTransitionCorrelations(futureFocusedRuns);
        const futureFocusedSuccessfulCorrelations = computeTransitionCorrelations(futureFocusedRuns, { requireTask1Success: true });
        const pregraspCorrelations = computeTransitionCorrelations(pregraspRuns);
        const pregraspSuccessfulCorrelations = computeTransitionCorrelations(pregraspRuns, { requireTask1Success: true });
        const predictivePregraspEpisodes = pregraspRuns.find(({ method }) => method.id === METHOD_PRESETS.predictiveQp.id)?.episodes || [];
        const predictiveFtaPregraspEpisodes = pregraspRuns.find(({ method }) => method.id === METHOD_PRESETS.predictiveQpFta.id)?.episodes || [];
        const pregraspPairedDelta = summarizePairedMethodDelta(
            predictivePregraspEpisodes,
            predictiveFtaPregraspEpisodes,
            {
                branchKey: 'task1SelectedBranch',
                numericKeys: [
                    'task1SelectorReadiness',
                    'task1RolloutReadiness',
                    'task1TerminalReadiness',
                    'task2FirstStepClosing',
                    'task2DistanceReduction'
                ]
            }
        );

        const holdTable = formatBenchmarkMarkdownTable(holdRows, {
            columns: [
                { key: 'method', label: 'Method' },
                { key: 'episodes', label: 'Episodes', digits: 0 },
                { key: 'successRate', label: 'Hold success', format: 'percent' },
                { key: 'meanTicks', label: 'Mean ticks', digits: 2 },
                { key: 'meanRuntimeMs', label: 'Runtime ms', digits: 2 },
                { key: 'meanMargin', label: 'Min margin', digits: 3 },
                { key: 'meanProgress', label: 'X progress', digits: 4 },
                { key: 'meanHorizon', label: 'Mean horizon', digits: 2 },
                { key: 'meanReserve', label: 'Mean reserve', digits: 4 }
            ]
        });

        const transitionTable = formatBenchmarkMarkdownTable(transitionCoreRows, {
            columns: [
                { key: 'method', label: 'Method' },
                { key: 'episodes', label: 'Episodes', digits: 0 },
                { key: 'task1SuccessRate', label: 'Task1 success', format: 'percent' },
                { key: 'task1Progress', label: 'Task1 X progress', digits: 4 },
                { key: 'task1TerminalReadiness', label: 'Task1 readiness', digits: 4 },
                { key: 'task2SuccessRate', label: 'Task2 success', format: 'percent' },
                { key: 'task2FirstClosing', label: 'Task2 first close', digits: 4 },
                { key: 'task2DistanceReduction', label: 'Task2 total close', digits: 4 },
                { key: 'task2RuntimeMs', label: 'Task2 ms', digits: 2 }
            ]
        });

        const futureFocusedTable = formatBenchmarkMarkdownTable(futureFocusedCoreRows, {
            columns: [
                { key: 'method', label: 'Method' },
                { key: 'episodes', label: 'Episodes', digits: 0 },
                { key: 'task1SuccessRate', label: 'Task1 success', format: 'percent' },
                { key: 'task1Progress', label: 'Task1 X progress', digits: 4 },
                { key: 'task1TerminalReadiness', label: 'Task1 readiness', digits: 4 },
                { key: 'task2SuccessRate', label: 'Task2 success', format: 'percent' },
                { key: 'task2FirstClosing', label: 'Task2 first close', digits: 4 },
                { key: 'task2DistanceReduction', label: 'Task2 total close', digits: 4 },
                { key: 'task2RuntimeMs', label: 'Task2 ms', digits: 2 }
            ]
        });
        const pregraspTable = formatBenchmarkMarkdownTable(pregraspCoreRows, {
            columns: [
                { key: 'method', label: 'Method' },
                { key: 'episodes', label: 'Episodes', digits: 0 },
                { key: 'task1SuccessRate', label: 'Task1 success', format: 'percent' },
                { key: 'task1Progress', label: 'Task1 X progress', digits: 4 },
                { key: 'task1TerminalReadiness', label: 'Task1 readiness', digits: 4 },
                { key: 'task2SuccessRate', label: 'Task2 success', format: 'percent' },
                { key: 'task2FirstClosing', label: 'Task2 first close', digits: 4 },
                { key: 'task2DistanceReduction', label: 'Task2 total close', digits: 4 },
                { key: 'task2RuntimeMs', label: 'Task2 ms', digits: 2 }
            ]
        });

        const results = {
            baseUrl: DEFAULT_BASE_URL,
            generatedAt: new Date().toISOString(),
            holdScenario: holdRows,
            transitionScenario: transitionRows,
            futureFocusedTransitionScenario: futureFocusedRows,
            pregraspTransitionScenario: pregraspRows,
            transitionCorrelation: transitionCorrelations,
            transitionSuccessfulCorrelation: transitionSuccessfulCorrelations,
            futureFocusedTransitionCorrelation: futureFocusedCorrelations,
            futureFocusedSuccessfulTransitionCorrelation: futureFocusedSuccessfulCorrelations,
            pregraspTransitionCorrelation: pregraspCorrelations,
            pregraspSuccessfulTransitionCorrelation: pregraspSuccessfulCorrelations,
            pregraspPairedDelta
        };

        const outputPath = path.join(REPO_ROOT, 'log', 'humanoid-jog-benchmark.json');
        await fs.writeFile(outputPath, JSON.stringify(results, null, 2));

        console.log('\n[absolute-hold-near-limit]');
        console.log(holdTable);
        for (const row of holdRows) {
            console.log(`labels ${row.method}: ${JSON.stringify(row.labels)}`);
        }

        console.log('\n[absolute-to-distance-transition]');
        console.log(transitionTable);
        console.log(formatCorrelationLine('readiness vs task2 first close', transitionCorrelations.readinessVsFirstClose, transitionCorrelations.sampleCount));
        console.log(formatCorrelationLine('readiness vs task2 total close', transitionCorrelations.readinessVsTotalClose, transitionCorrelations.sampleCount));
        console.log(formatCorrelationLine('success-only readiness vs task2 first close', transitionSuccessfulCorrelations.readinessVsFirstClose, transitionSuccessfulCorrelations.sampleCount));
        console.log(formatCorrelationLine('success-only readiness vs task2 total close', transitionSuccessfulCorrelations.readinessVsTotalClose, transitionSuccessfulCorrelations.sampleCount));
        for (const row of transitionRows) {
            console.log(`task1 labels ${row.method}: ${JSON.stringify(row.task1Labels)}`);
            console.log(`task2 labels ${row.method}: ${JSON.stringify(row.task2Labels)}`);
        }

        console.log('\n[fullbody-future-transition]');
        console.log(futureFocusedTable);
        console.log(formatCorrelationLine('readiness vs task2 first close', futureFocusedCorrelations.readinessVsFirstClose, futureFocusedCorrelations.sampleCount));
        console.log(formatCorrelationLine('readiness vs task2 total close', futureFocusedCorrelations.readinessVsTotalClose, futureFocusedCorrelations.sampleCount));
        console.log(formatCorrelationLine('success-only readiness vs task2 first close', futureFocusedSuccessfulCorrelations.readinessVsFirstClose, futureFocusedSuccessfulCorrelations.sampleCount));
        console.log(formatCorrelationLine('success-only readiness vs task2 total close', futureFocusedSuccessfulCorrelations.readinessVsTotalClose, futureFocusedSuccessfulCorrelations.sampleCount));
        for (const row of futureFocusedRows) {
            console.log(`task1 labels ${row.method}: ${JSON.stringify(row.task1Labels)}`);
            console.log(`task2 labels ${row.method}: ${JSON.stringify(row.task2Labels)}`);
        }

        console.log('\n[single-step-future-transition]');
        console.log(pregraspTable);
        console.log(formatCorrelationLine('readiness vs task2 first close', pregraspCorrelations.readinessVsFirstClose, pregraspCorrelations.sampleCount));
        console.log(formatCorrelationLine('readiness vs task2 total close', pregraspCorrelations.readinessVsTotalClose, pregraspCorrelations.sampleCount));
        console.log(formatCorrelationLine('success-only readiness vs task2 first close', pregraspSuccessfulCorrelations.readinessVsFirstClose, pregraspSuccessfulCorrelations.sampleCount));
        console.log(formatCorrelationLine('success-only readiness vs task2 total close', pregraspSuccessfulCorrelations.readinessVsTotalClose, pregraspSuccessfulCorrelations.sampleCount));
        console.log(`paired divergence rate (Predictive-QP -> Predictive-QP+FTA): ${Number.isFinite(pregraspPairedDelta.branchDivergenceRate) ? (pregraspPairedDelta.branchDivergenceRate * 100).toFixed(1) + '%' : 'n/a'} (pairs=${pregraspPairedDelta.pairCount})`);
        console.log(`paired mean selector-readiness gain: ${Number.isFinite(pregraspPairedDelta.meanDeltas.task1SelectorReadiness) ? pregraspPairedDelta.meanDeltas.task1SelectorReadiness.toExponential(3) : 'n/a'}`);
        console.log(`paired mean rollout-readiness gain: ${Number.isFinite(pregraspPairedDelta.meanDeltas.task1RolloutReadiness) ? pregraspPairedDelta.meanDeltas.task1RolloutReadiness.toExponential(3) : 'n/a'}`);
        console.log(`paired mean effective-readiness gain: ${Number.isFinite(pregraspPairedDelta.meanDeltas.task1TerminalReadiness) ? pregraspPairedDelta.meanDeltas.task1TerminalReadiness.toExponential(3) : 'n/a'}`);
        console.log(`paired mean first-close gain: ${Number.isFinite(pregraspPairedDelta.meanDeltas.task2FirstStepClosing) ? pregraspPairedDelta.meanDeltas.task2FirstStepClosing.toExponential(3) : 'n/a'}`);
        console.log(`paired mean total-close gain: ${Number.isFinite(pregraspPairedDelta.meanDeltas.task2DistanceReduction) ? pregraspPairedDelta.meanDeltas.task2DistanceReduction.toExponential(3) : 'n/a'}`);
        for (const row of pregraspRows) {
            console.log(`task1 labels ${row.method}: ${JSON.stringify(row.task1Labels)}`);
            console.log(`task2 labels ${row.method}: ${JSON.stringify(row.task2Labels)}`);
        }

        console.log(`\nSaved JSON to ${outputPath}`);
    } finally {
        await browser?.close();
        await stopServer(server);
    }
}

main().catch((error) => {
    console.error(error);
    process.exitCode = 1;
});
