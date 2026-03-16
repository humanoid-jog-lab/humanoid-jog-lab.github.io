function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

export function extractBodyZAxisLineDirectionVec4(rotMat) {
    return [
        0,
        asFinite(rotMat?.[0]?.[2], 0),
        asFinite(rotMat?.[1]?.[2], 0),
        asFinite(rotMat?.[2]?.[2], 1)
    ];
}

export function computeLineDirectionMismatchSquared(currentVec4, referenceVec4 = [0, 0, 0, 1]) {
    const diffVec4 = new Array(4).fill(0);
    let errorScalar = 0;
    for (let i = 0; i < 4; i++) {
        const diff = asFinite(referenceVec4?.[i], 0) - asFinite(currentVec4?.[i], 0);
        diffVec4[i] = diff;
        errorScalar += diff * diff;
    }
    return { diffVec4, errorScalar };
}

export function buildBodyVerticalLineTask({
    rotMat,
    lineJacobianFull,
    armStart = 0,
    activeCount = 0,
    referenceVec4 = [0, 0, 0, 1]
}) {
    if (!Array.isArray(lineJacobianFull) || activeCount <= 0) return null;
    const currentVec4 = extractBodyZAxisLineDirectionVec4(rotMat);
    const { diffVec4, errorScalar } = computeLineDirectionMismatchSquared(currentVec4, referenceVec4);

    const directionRows = [];
    for (let rowIdx = 0; rowIdx < 4; rowIdx++) {
        const row = Array.isArray(lineJacobianFull[rowIdx]) ? lineJacobianFull[rowIdx] : null;
        if (!row) return null;
        directionRows.push(row.slice(armStart, armStart + activeCount));
    }

    const jError = new Array(activeCount).fill(0);
    for (let j = 0; j < activeCount; j++) {
        let sum = 0;
        for (let rowIdx = 0; rowIdx < 4; rowIdx++) {
            sum += diffVec4[rowIdx] * asFinite(directionRows[rowIdx]?.[j], 0);
        }
        jError[j] = -2 * sum;
    }

    return {
        currentVec4,
        referenceVec4: referenceVec4.slice(0, 4),
        diffVec4,
        mismatchSquared: errorScalar,
        error: [-errorScalar],
        errorNorm: Math.sqrt(errorScalar),
        Jrows: [jError],
        directionRows
    };
}
