import { buildSurfaceAttachedFrame, evaluateSurfacePatch } from './surface-slide-primitive.js';

function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function clamp01(value) {
    return Math.min(Math.max(asFinite(value, 0), 0), 1);
}

function sampleRange(boundsLike, fallback = [0, 1]) {
    if (Array.isArray(boundsLike) && boundsLike.length === 2) {
        const lo = asFinite(boundsLike[0], fallback[0]);
        const hi = asFinite(boundsLike[1], fallback[1]);
        if (hi > lo) return [lo, hi];
    }
    return fallback.slice();
}

function formatObjVertex(vertex) {
    return `v ${vertex[0].toFixed(6)} ${vertex[1].toFixed(6)} ${vertex[2].toFixed(6)}`;
}

function formatObjTriangle(a, b, c) {
    return `f ${a} ${b} ${c}`;
}

function addQuadTriangles(faceLines, a, b, c, d, reversed = false) {
    if (reversed) {
        faceLines.push(formatObjTriangle(a, d, c));
        faceLines.push(formatObjTriangle(a, c, b));
        return;
    }
    faceLines.push(formatObjTriangle(a, b, c));
    faceLines.push(formatObjTriangle(a, c, d));
}

function estimatePolylineLength(points) {
    let total = 0;
    for (let index = 1; index < points.length; index += 1) {
        const prev = points[index - 1];
        const curr = points[index];
        total += Math.hypot(
            (curr[0] || 0) - (prev[0] || 0),
            (curr[1] || 0) - (prev[1] || 0),
            (curr[2] || 0) - (prev[2] || 0)
        );
    }
    return total;
}

export function buildPrintableSurfaceShellObj(surface, {
    uBounds = [0, 1],
    vBounds = [0, 1],
    thicknessMetres = 0.008,
    segmentsU = 96,
    segmentsV = 32,
    units = 'millimetres',
    objectName = 'printable_surface_shell'
} = {}) {
    if (!surface) {
        throw new Error('buildPrintableSurfaceShellObj requires a surface');
    }

    const resolvedUBounds = sampleRange(uBounds);
    const resolvedVBounds = sampleRange(vBounds);
    const resolvedSegmentsU = Math.max(2, Math.floor(asFinite(segmentsU, 96)));
    const resolvedSegmentsV = Math.max(2, Math.floor(asFinite(segmentsV, 32)));
    const resolvedThicknessMetres = Math.max(1e-5, asFinite(thicknessMetres, 0.008));
    const unitScale = units === 'millimetres' ? 1000 : 1;
    const shellThickness = resolvedThicknessMetres * unitScale;

    const vertices = [];
    const faces = [];
    const edgeCounts = new Map();
    const innerGrid = [];
    const outerGrid = [];

    const addVertex = (vertex) => {
        vertices.push(vertex);
        return vertices.length;
    };

    const countTriangleEdges = (a, b, c) => {
        const edges = [[a, b], [b, c], [c, a]];
        for (const [u, v] of edges) {
            const key = u < v ? `${u}-${v}` : `${v}-${u}`;
            edgeCounts.set(key, (edgeCounts.get(key) || 0) + 1);
        }
    };

    const addFaceLine = (line) => {
        faces.push(line);
        const [, sa, sb, sc] = line.split(/\s+/);
        countTriangleEdges(Number(sa), Number(sb), Number(sc));
    };

    const pushQuad = (a, b, c, d, reversed = false) => {
        const quadFaces = [];
        addQuadTriangles(quadFaces, a, b, c, d, reversed);
        quadFaces.forEach(addFaceLine);
    };

    const samplePointRow = [];
    const sampleSpanRow = [];

    for (let j = 0; j <= resolvedSegmentsV; j += 1) {
        const v = resolvedVBounds[0] + ((resolvedVBounds[1] - resolvedVBounds[0]) * (j / resolvedSegmentsV));
        const innerRow = [];
        const outerRow = [];
        for (let i = 0; i <= resolvedSegmentsU; i += 1) {
            const u = resolvedUBounds[0] + ((resolvedUBounds[1] - resolvedUBounds[0]) * (i / resolvedSegmentsU));
            const patch = evaluateSurfacePatch(surface, clamp01(u), clamp01(v));
            const frame = buildSurfaceAttachedFrame(patch);
            const pointMm = frame.position.map((value) => asFinite(value, 0) * unitScale);
            const normalMm = frame.normal.map((value) => asFinite(value, 0));
            const offsetPointMm = [
                pointMm[0] + (normalMm[0] * shellThickness),
                pointMm[1] + (normalMm[1] * shellThickness),
                pointMm[2] + (normalMm[2] * shellThickness)
            ];
            innerRow.push(addVertex(pointMm));
            outerRow.push(addVertex(offsetPointMm));
            if (j === Math.floor(resolvedSegmentsV / 2)) samplePointRow.push(pointMm);
            if (i === Math.floor(resolvedSegmentsU / 2)) sampleSpanRow.push(pointMm);
        }
        innerGrid.push(innerRow);
        outerGrid.push(outerRow);
    }

    for (let j = 0; j < resolvedSegmentsV; j += 1) {
        for (let i = 0; i < resolvedSegmentsU; i += 1) {
            const i00 = innerGrid[j][i];
            const i10 = innerGrid[j][i + 1];
            const i01 = innerGrid[j + 1][i];
            const i11 = innerGrid[j + 1][i + 1];
            const o00 = outerGrid[j][i];
            const o10 = outerGrid[j][i + 1];
            const o01 = outerGrid[j + 1][i];
            const o11 = outerGrid[j + 1][i + 1];

            pushQuad(i00, i10, i11, i01, true);
            pushQuad(o00, o01, o11, o10, false);
        }
    }

    for (let j = 0; j < resolvedSegmentsV; j += 1) {
        pushQuad(outerGrid[j][0], outerGrid[j + 1][0], innerGrid[j + 1][0], innerGrid[j][0], false);
        pushQuad(innerGrid[j][resolvedSegmentsU], innerGrid[j + 1][resolvedSegmentsU], outerGrid[j + 1][resolvedSegmentsU], outerGrid[j][resolvedSegmentsU], false);
    }

    for (let i = 0; i < resolvedSegmentsU; i += 1) {
        pushQuad(innerGrid[0][i], outerGrid[0][i], outerGrid[0][i + 1], innerGrid[0][i + 1], false);
        pushQuad(outerGrid[resolvedSegmentsV][i], innerGrid[resolvedSegmentsV][i], innerGrid[resolvedSegmentsV][i + 1], outerGrid[resolvedSegmentsV][i + 1], false);
    }

    const nonManifoldEdges = [...edgeCounts.entries()].filter(([, count]) => count !== 2);
    const chordWidth = Math.hypot(
        samplePointRow[0]?.[0] - samplePointRow[samplePointRow.length - 1]?.[0] || 0,
        samplePointRow[0]?.[1] - samplePointRow[samplePointRow.length - 1]?.[1] || 0,
        samplePointRow[0]?.[2] - samplePointRow[samplePointRow.length - 1]?.[2] || 0
    );
    const axialSpan = Math.hypot(
        sampleSpanRow[0]?.[0] - sampleSpanRow[sampleSpanRow.length - 1]?.[0] || 0,
        sampleSpanRow[0]?.[1] - sampleSpanRow[sampleSpanRow.length - 1]?.[1] || 0,
        sampleSpanRow[0]?.[2] - sampleSpanRow[sampleSpanRow.length - 1]?.[2] || 0
    );
    const arcLength = estimatePolylineLength(samplePointRow);

    const lines = [
        '# Printable ruled-surface shell',
        `# Units: ${units}`,
        `o ${objectName}`,
        ...vertices.map(formatObjVertex),
        ...faces
    ];

    return {
        objText: `${lines.join('\n')}\n`,
        filename: `${objectName}.obj`,
        vertexCount: vertices.length,
        triangleCount: faces.length,
        watertight: nonManifoldEdges.length === 0,
        nonManifoldEdgeCount: nonManifoldEdges.length,
        metadata: {
            units,
            thickness: shellThickness,
            uBounds: resolvedUBounds,
            vBounds: resolvedVBounds,
            segmentsU: resolvedSegmentsU,
            segmentsV: resolvedSegmentsV,
            chordWidth,
            axialSpan,
            arcLength,
        }
    };
}
