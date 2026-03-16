export function minimizeLBFGS(f, grad, x0, options = {}) {
    const maxIter = options.maxIter ?? 100;
    const gradTol = options.gradTol ?? options.tol ?? 1e-6;
    const costTol = options.costTol ?? 0;
    const stepTol = options.stepTol ?? 1e-12;
    const m = options.m ?? 10;
    const maxLineSearch = options.maxLineSearch ?? 25;
    const c1 = 1e-4;

    const n = x0.length;
    const x = [...x0];
    const g = new Array(n).fill(0);
    const q = new Array(n).fill(0);
    const r = new Array(n).fill(0);
    const d = new Array(n).fill(0);

    const sList = [];
    const yList = [];
    const rhoList = [];
    const alphaList = new Array(m).fill(0);

    const vectorNorm = (vec) => Math.sqrt(vec.reduce((sum, value) => sum + value * value, 0));

    grad(x, g);
    let cost = f(x);
    let gradNorm = vectorNorm(g);
    if (cost <= costTol || gradNorm <= gradTol) {
        return { x, converged: true, cost, gradNorm, iterations: 0, reason: cost <= costTol ? 'cost' : 'gradient' };
    }

    for (let iter = 0; iter < maxIter; iter++) {
        for (let i = 0; i < n; i++) q[i] = g[i];

        const historySize = sList.length;
        for (let i = historySize - 1; i >= 0; i--) {
            let sq = 0;
            for (let j = 0; j < n; j++) sq += sList[i][j] * q[j];
            alphaList[i] = rhoList[i] * sq;
            for (let j = 0; j < n; j++) q[j] -= alphaList[i] * yList[i][j];
        }

        let h0 = 1;
        if (historySize > 0) {
            let ys = 0;
            let yy = 0;
            const sLast = sList[historySize - 1];
            const yLast = yList[historySize - 1];
            for (let j = 0; j < n; j++) {
                ys += yLast[j] * sLast[j];
                yy += yLast[j] * yLast[j];
            }
            h0 = ys / (yy + 1e-14);
        }

        for (let j = 0; j < n; j++) r[j] = h0 * q[j];
        for (let i = 0; i < historySize; i++) {
            let yr = 0;
            for (let j = 0; j < n; j++) yr += yList[i][j] * r[j];
            const beta = rhoList[i] * yr;
            for (let j = 0; j < n; j++) r[j] += sList[i][j] * (alphaList[i] - beta);
        }
        for (let j = 0; j < n; j++) d[j] = -r[j];

        let dg = 0;
        for (let i = 0; i < n; i++) dg += g[i] * d[i];
        if (!Number.isFinite(dg) || dg >= -1e-16) {
            for (let i = 0; i < n; i++) d[i] = -g[i];
            dg = -gradNorm * gradNorm;
        }

        let alpha = 1;
        let accepted = false;
        let costNew = Number.POSITIVE_INFINITY;
        const xNew = new Array(n);
        for (let ls = 0; ls < maxLineSearch; ls++) {
            for (let i = 0; i < n; i++) xNew[i] = x[i] + alpha * d[i];
            costNew = f(xNew);
            if (Number.isFinite(costNew) && costNew <= cost + c1 * alpha * dg) {
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }

        if (!accepted) {
            return { x, converged: cost <= costTol, cost, gradNorm, iterations: iter, reason: 'line_search_failed' };
        }

        const gOld = [...g];
        for (let i = 0; i < n; i++) x[i] = xNew[i];
        cost = costNew;
        grad(x, g);
        gradNorm = vectorNorm(g);

        const s = new Array(n);
        const y = new Array(n);
        let ys = 0;
        let stepNormSq = 0;
        for (let i = 0; i < n; i++) {
            s[i] = alpha * d[i];
            y[i] = g[i] - gOld[i];
            ys += y[i] * s[i];
            stepNormSq += s[i] * s[i];
        }
        const stepNorm = Math.sqrt(stepNormSq);

        if (ys > 1e-14) {
            sList.push(s);
            yList.push(y);
            rhoList.push(1 / ys);
            if (sList.length > m) {
                sList.shift();
                yList.shift();
                rhoList.shift();
            }
        }

        if (cost <= costTol) {
            return { x, converged: true, cost, gradNorm, iterations: iter + 1, reason: 'cost' };
        }
        if (gradNorm <= gradTol) {
            return { x, converged: true, cost, gradNorm, iterations: iter + 1, reason: 'gradient' };
        }
        if (stepNorm <= stepTol) {
            return {
                x,
                converged: cost <= Math.max(costTol, 1e-12),
                cost,
                gradNorm,
                iterations: iter + 1,
                reason: 'step'
            };
        }
    }

    return { x, converged: cost <= costTol, cost, gradNorm, iterations: maxIter, reason: 'max_iter' };
}
