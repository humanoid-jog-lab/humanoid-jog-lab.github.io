function finiteOrZero(value) {
    return Number.isFinite(value) ? value : 0;
}

export function buildHumanoidJogMJResidualBlock(target = [], current = []) {
    const count = Math.max(
        Array.isArray(target) ? target.length : 0,
        Array.isArray(current) ? current.length : 0
    );
    const out = new Array(count);
    for (let i = 0; i < count; i++) {
        out[i] = finiteOrZero(Number(target?.[i])) - finiteOrZero(Number(current?.[i]));
    }
    return out;
}

export function computeHumanoidJogMJResidualStackNorm(blocks = []) {
    let sumSq = 0;
    for (const block of Array.isArray(blocks) ? blocks : []) {
        if (!Array.isArray(block)) continue;
        for (const value of block) {
            const finite = finiteOrZero(Number(value));
            sumSq += finite * finite;
        }
    }
    return Math.sqrt(sumSq);
}

export function shouldBuildHumanoidJogMJSecondaryObjective({
    enabled = false,
    disabled = false,
    residualBlocks = [],
    activationResidual = 1e-5
} = {}) {
    if (enabled !== true || disabled === true) return false;
    const threshold = Math.max(0, Number.isFinite(activationResidual) ? activationResidual : 1e-5);
    return computeHumanoidJogMJResidualStackNorm(residualBlocks) <= threshold;
}
