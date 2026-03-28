import test from 'node:test';
import assert from 'node:assert/strict';

import {
    computePearsonCorrelation,
    countByLabel,
    formatBenchmarkMarkdownTable,
    meanOf,
    summarizePairedMethodDelta,
    summarizeSustainedCommandEpisodes
} from './humanoid-jog-benchmark-metrics.js';

test('meanOf ignores non-finite values', () => {
    const value = meanOf(
        [
            { score: 1 },
            { score: Number.NaN },
            { score: 5 },
            { score: Number.POSITIVE_INFINITY }
        ],
        (item) => item.score
    );
    assert.equal(value, 3);
});

test('countByLabel groups missing labels into unknown', () => {
    const counts = countByLabel(
        [
            { label: 'reachable' },
            { label: 'reachable' },
            { label: 'unreachable' },
            {}
        ],
        (item) => item.label
    );
    assert.deepEqual(counts, {
        reachable: 2,
        unreachable: 1,
        unknown: 1
    });
});

test('summarizeSustainedCommandEpisodes computes rates and numeric means', () => {
    const summary = summarizeSustainedCommandEpisodes(
        [
            {
                sustainableTicks: 8,
                targetTicks: 8,
                meanRuntimeMs: 12,
                minJointMarginRatio: 0.11,
                absoluteProgress: 0.05,
                terminalLabel: 'reachable'
            },
            {
                sustainableTicks: 5,
                targetTicks: 8,
                meanRuntimeMs: 14,
                minJointMarginRatio: 0.08,
                absoluteProgress: 0.03,
                terminalLabel: 'margin-limited'
            }
        ],
        {
            ticksKey: 'sustainableTicks',
            targetKey: 'targetTicks',
            labelKey: 'terminalLabel',
            numericKeys: ['meanRuntimeMs', 'minJointMarginRatio', 'absoluteProgress']
        }
    );

    assert.equal(summary.episodeCount, 2);
    assert.equal(summary.successRate, 0.5);
    assert.equal(summary.meanTicks, 6.5);
    assert.equal(summary.numericMeans.meanRuntimeMs, 13);
    assert.equal(summary.numericMeans.minJointMarginRatio, 0.095);
    assert.equal(summary.numericMeans.absoluteProgress, 0.04);
    assert.deepEqual(summary.labelCounts, {
        reachable: 1,
        'margin-limited': 1
    });
});

test('formatBenchmarkMarkdownTable renders a markdown table', () => {
    const table = formatBenchmarkMarkdownTable(
        [
            {
                method: 'Predictive',
                successRate: 0.75,
                meanTicks: 9.5,
                meanRuntimeMs: 13.21
            }
        ],
        {
            columns: [
                { key: 'method', label: 'Method' },
                { key: 'successRate', label: 'Success', format: 'percent' },
                { key: 'meanTicks', label: 'Ticks', digits: 1 },
                { key: 'meanRuntimeMs', label: 'Runtime', digits: 2 }
            ]
        }
    );

    assert.match(table, /\| Method \| Success \| Ticks \| Runtime \|/);
    assert.match(table, /\| Predictive \| 75\.0% \| 9\.5 \| 13\.21 \|/);
});

test('computePearsonCorrelation returns a positive correlation for aligned trends', () => {
    const correlation = computePearsonCorrelation(
        [
            { readiness: 0.1, close: 0.01 },
            { readiness: 0.2, close: 0.02 },
            { readiness: 0.3, close: 0.03 }
        ],
        (item) => item.readiness,
        (item) => item.close
    );
    assert.ok(correlation > 0.99);
});

test('summarizePairedMethodDelta reports branch divergence and numeric gains', () => {
    const summary = summarizePairedMethodDelta(
        [
            { task1SelectedBranch: 'real', task2FirstStepClosing: 0.01, task2DistanceReduction: 0.03 },
            { task1SelectedBranch: 'real', task2FirstStepClosing: 0.02, task2DistanceReduction: 0.04 }
        ],
        [
            { task1SelectedBranch: 'phantom', task2FirstStepClosing: 0.011, task2DistanceReduction: 0.031 },
            { task1SelectedBranch: 'real', task2FirstStepClosing: 0.019, task2DistanceReduction: 0.042 }
        ],
        {
            branchKey: 'task1SelectedBranch',
            numericKeys: ['task2FirstStepClosing', 'task2DistanceReduction']
        }
    );

    assert.equal(summary.pairCount, 2);
    assert.equal(summary.branchDivergenceRate, 0.5);
    assert.ok(summary.meanDeltas.task2FirstStepClosing < 0.001 && summary.meanDeltas.task2FirstStepClosing > -0.001);
    assert.ok(Math.abs(summary.meanDeltas.task2DistanceReduction - 0.0015) < 1e-12);
});
