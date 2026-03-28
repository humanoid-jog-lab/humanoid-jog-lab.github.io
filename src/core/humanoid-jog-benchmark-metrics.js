function asFiniteNumber(value) {
    return Number.isFinite(value) ? value : Number.NaN;
}

export function meanOf(items = [], selector = (value) => value) {
    const values = [];
    for (const item of items) {
        const value = asFiniteNumber(selector(item));
        if (Number.isFinite(value)) values.push(value);
    }
    if (values.length === 0) return Number.NaN;
    return values.reduce((sum, value) => sum + value, 0) / values.length;
}

export function countByLabel(items = [], selector = (value) => value) {
    const counts = {};
    for (const item of items) {
        const rawLabel = selector(item);
        const label = (typeof rawLabel === 'string' && rawLabel.trim().length > 0)
            ? rawLabel.trim()
            : 'unknown';
        counts[label] = (counts[label] || 0) + 1;
    }
    return counts;
}

export function summarizeSustainedCommandEpisodes(
    episodes = [],
    {
        ticksKey = 'sustainableTicks',
        targetKey = 'targetTicks',
        labelKey = 'terminalLabel',
        numericKeys = []
    } = {}
) {
    const safeEpisodes = Array.isArray(episodes) ? episodes : [];
    const successCount = safeEpisodes.filter((episode) => {
        const ticks = asFiniteNumber(episode?.[ticksKey]);
        const target = asFiniteNumber(episode?.[targetKey]);
        return Number.isFinite(ticks) && Number.isFinite(target) && ticks >= target;
    }).length;

    const numericMeans = {};
    for (const key of numericKeys) {
        numericMeans[key] = meanOf(safeEpisodes, (episode) => episode?.[key]);
    }

    return {
        episodeCount: safeEpisodes.length,
        successCount,
        successRate: safeEpisodes.length > 0 ? successCount / safeEpisodes.length : Number.NaN,
        meanTicks: meanOf(safeEpisodes, (episode) => episode?.[ticksKey]),
        labelCounts: countByLabel(safeEpisodes, (episode) => episode?.[labelKey]),
        numericMeans
    };
}

export function computePearsonCorrelation(items = [], xSelector = (value) => value?.x, ySelector = (value) => value?.y) {
    const pairs = [];
    for (const item of items) {
        const x = asFiniteNumber(xSelector(item));
        const y = asFiniteNumber(ySelector(item));
        if (Number.isFinite(x) && Number.isFinite(y)) {
            pairs.push([x, y]);
        }
    }
    if (pairs.length < 2) return Number.NaN;

    const xMean = pairs.reduce((sum, [x]) => sum + x, 0) / pairs.length;
    const yMean = pairs.reduce((sum, [, y]) => sum + y, 0) / pairs.length;

    let numerator = 0;
    let xVariance = 0;
    let yVariance = 0;
    for (const [x, y] of pairs) {
        const dx = x - xMean;
        const dy = y - yMean;
        numerator += dx * dy;
        xVariance += dx * dx;
        yVariance += dy * dy;
    }

    if (xVariance <= 1e-12 || yVariance <= 1e-12) return Number.NaN;
    return numerator / Math.sqrt(xVariance * yVariance);
}

export function summarizePairedMethodDelta(
    baselineEpisodes = [],
    candidateEpisodes = [],
    {
        branchKey = 'task1SelectedBranch',
        numericKeys = []
    } = {}
) {
    const pairCount = Math.min(
        Array.isArray(baselineEpisodes) ? baselineEpisodes.length : 0,
        Array.isArray(candidateEpisodes) ? candidateEpisodes.length : 0
    );
    const pairs = [];
    for (let index = 0; index < pairCount; index += 1) {
        pairs.push({
            baseline: baselineEpisodes[index] || {},
            candidate: candidateEpisodes[index] || {}
        });
    }

    const divergenceCount = pairs.filter(({ baseline, candidate }) => {
        const left = baseline?.[branchKey];
        const right = candidate?.[branchKey];
        return typeof left === 'string' && typeof right === 'string' && left !== right;
    }).length;

    const meanDeltas = {};
    for (const key of numericKeys) {
        meanDeltas[key] = meanOf(pairs, ({ baseline, candidate }) => {
            const left = asFiniteNumber(baseline?.[key]);
            const right = asFiniteNumber(candidate?.[key]);
            if (!Number.isFinite(left) || !Number.isFinite(right)) return Number.NaN;
            return right - left;
        });
    }

    return {
        pairCount,
        branchDivergenceRate: pairCount > 0 ? divergenceCount / pairCount : Number.NaN,
        meanDeltas
    };
}

function formatValue(value, column = {}) {
    if (!Number.isFinite(value)) return 'n/a';
    if (column.format === 'percent') {
        const digits = Number.isInteger(column.digits) ? column.digits : 1;
        return `${(value * 100).toFixed(digits)}%`;
    }
    const digits = Number.isInteger(column.digits) ? column.digits : 3;
    return value.toFixed(digits);
}

export function formatBenchmarkMarkdownTable(
    rows = [],
    { columns = [] } = {}
) {
    const safeRows = Array.isArray(rows) ? rows : [];
    const safeColumns = Array.isArray(columns) ? columns : [];
    if (safeColumns.length === 0) return '';

    const header = `| ${safeColumns.map((column) => column.label).join(' | ')} |`;
    const separator = `| ${safeColumns.map(() => '---').join(' | ')} |`;
    const body = safeRows.map((row) => {
        const cells = safeColumns.map((column) => {
            const value = row?.[column.key];
            if (typeof value === 'string') return value;
            return formatValue(value, column);
        });
        return `| ${cells.join(' | ')} |`;
    });
    return [header, separator, ...body].join('\n');
}
