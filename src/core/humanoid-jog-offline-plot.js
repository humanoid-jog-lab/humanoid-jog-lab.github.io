function asFinite(value, fallback = Number.NaN) {
    return Number.isFinite(value) ? value : fallback;
}

function escapeXml(value) {
    return String(value ?? '')
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;');
}

function collectSeries(result = {}, jointIndices = []) {
    const samples = Array.isArray(result.samples) ? result.samples : [];
    const baseSeries = [
        {
            key: 'err_cmd',
            label: 'err_cmd',
            color: '#2563eb',
            values: samples.map((sample) => [sample.index, asFinite(sample.err?.cmd)])
        },
        {
            key: 'err_inv',
            label: 'err_inv',
            color: '#dc2626',
            values: samples.map((sample) => [sample.index, asFinite(sample.err?.inv)])
        },
        {
            key: 'dmin',
            label: 'dmin',
            color: '#059669',
            values: samples.map((sample) => [sample.index, asFinite(sample.dmin)])
        },
        {
            key: 'waist_x',
            label: 'waist_x',
            color: '#d97706',
            values: samples.map((sample) => [sample.index, asFinite(sample.waist_x?.p0x)])
        },
        {
            key: 'solve_ms',
            label: 'solve_ms',
            color: '#7c3aed',
            values: samples.map((sample) => [sample.index, asFinite(sample.timingMs?.solve)])
        }
    ];
    const jointSeries = jointIndices.map((jointIndex, seriesIndex) => ({
        key: `q_${jointIndex}`,
        label: `q[${jointIndex}]`,
        color: ['#0891b2', '#be123c', '#4d7c0f', '#9333ea'][seriesIndex % 4],
        values: samples.map((sample) => [sample.index, asFinite(sample.q?.[jointIndex])])
    }));
    return [...baseSeries, ...jointSeries].filter((series) =>
        series.values.some(([, value]) => Number.isFinite(value))
    );
}

function valueExtent(seriesList) {
    const values = [];
    for (const series of seriesList) {
        for (const [, value] of series.values) {
            if (Number.isFinite(value)) values.push(value);
        }
    }
    if (values.length === 0) return [0, 1];
    let min = Math.min(...values);
    let max = Math.max(...values);
    if (Math.abs(max - min) < 1e-12) {
        min -= 1;
        max += 1;
    }
    const pad = (max - min) * 0.08;
    return [min - pad, max + pad];
}

function buildPath(series, {
    xMin,
    xMax,
    yMin,
    yMax,
    left,
    top,
    width,
    height
}) {
    const points = [];
    for (const [xRaw, yRaw] of series.values) {
        if (!Number.isFinite(xRaw) || !Number.isFinite(yRaw)) continue;
        const xRatio = xMax === xMin ? 0 : (xRaw - xMin) / (xMax - xMin);
        const yRatio = yMax === yMin ? 0.5 : (yRaw - yMin) / (yMax - yMin);
        const x = left + xRatio * width;
        const y = top + (1 - yRatio) * height;
        points.push(`${points.length === 0 ? 'M' : 'L'}${x.toFixed(2)},${y.toFixed(2)}`);
    }
    return points.join(' ');
}

export function buildHumanoidJogOfflinePlotSvg(result = {}, {
    title,
    width = 1100,
    height = 720,
    jointIndices = [0, 5, 12]
} = {}) {
    const samples = Array.isArray(result.samples) ? result.samples : [];
    const series = collectSeries(result, jointIndices);
    const plotLeft = 78;
    const plotTop = 82;
    const plotWidth = Math.max(120, width - 130);
    const plotHeight = Math.max(120, height - 190);
    const xValues = samples.map((sample) => asFinite(sample.index)).filter(Number.isFinite);
    const xMin = xValues.length > 0 ? Math.min(...xValues) : 0;
    const xMax = xValues.length > 0 ? Math.max(...xValues) : 1;
    const [yMin, yMax] = valueExtent(series);
    const command = result.meta?.command || {};
    const plotTitle = title || `${command.mode || 'offline'} ${command.solverMode || ''} ${command.axis || ''}${command.direction || ''}`.trim();
    const paths = series.map((item) => {
        const d = buildPath(item, {
            xMin,
            xMax,
            yMin,
            yMax,
            left: plotLeft,
            top: plotTop,
            width: plotWidth,
            height: plotHeight
        });
        return `<path d="${d}" fill="none" stroke="${item.color}" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/>`;
    }).join('\n');
    const legends = series.map((item, index) => {
        const x = plotLeft + (index % 4) * 180;
        const y = height - 82 + Math.floor(index / 4) * 24;
        return [
            `<line x1="${x}" y1="${y}" x2="${x + 28}" y2="${y}" stroke="${item.color}" stroke-width="3"/>`,
            `<text x="${x + 36}" y="${y + 5}" font-size="14" fill="#334155">${escapeXml(item.label)}</text>`
        ].join('\n');
    }).join('\n');

    return `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">
<rect width="100%" height="100%" fill="#f8fafc"/>
<text x="${plotLeft}" y="42" font-size="24" font-weight="700" fill="#0f172a">${escapeXml(plotTitle)}</text>
<text x="${plotLeft}" y="66" font-size="14" fill="#64748b">offline-core samples=${samples.length}</text>
<rect x="${plotLeft}" y="${plotTop}" width="${plotWidth}" height="${plotHeight}" fill="#ffffff" stroke="#cbd5e1"/>
<line x1="${plotLeft}" y1="${plotTop + plotHeight}" x2="${plotLeft + plotWidth}" y2="${plotTop + plotHeight}" stroke="#475569"/>
<line x1="${plotLeft}" y1="${plotTop}" x2="${plotLeft}" y2="${plotTop + plotHeight}" stroke="#475569"/>
<text x="${plotLeft}" y="${plotTop + plotHeight + 36}" font-size="14" fill="#475569">sample index</text>
<text x="${plotLeft - 58}" y="${plotTop + 14}" font-size="13" fill="#475569">${yMax.toExponential(2)}</text>
<text x="${plotLeft - 58}" y="${plotTop + plotHeight}" font-size="13" fill="#475569">${yMin.toExponential(2)}</text>
${paths}
${legends}
</svg>`;
}
