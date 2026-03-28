let dqModulePromise = null;
let currentBaseUrl = '/';
let currentModelKey = '';
let currentModel = null;
let currentDQModule = null;

function nowMs() {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
        return performance.now();
    }
    return Date.now();
}

function normalizeBaseUrl(baseUrl) {
    const raw = typeof baseUrl === 'string' && baseUrl.length > 0 ? baseUrl : '/';
    return raw.endsWith('/') ? raw : `${raw}/`;
}

function stableModelKey(modelSpec) {
    return JSON.stringify(modelSpec ?? null);
}

async function ensureDQModule(baseUrl = '/') {
    const normalizedBaseUrl = normalizeBaseUrl(baseUrl);
    if (!dqModulePromise || currentBaseUrl !== normalizedBaseUrl) {
        currentBaseUrl = normalizedBaseUrl;
        dqModulePromise = (async () => {
            self.importScripts(`${currentBaseUrl}wasm/dqrobotics.js`);
            if (typeof self.createDQRoboticsModule !== 'function') {
                throw new Error('dqrobotics_wasm_factory_unavailable');
            }
            const module = await self.createDQRoboticsModule();
            currentDQModule = module;
            return module;
        })();
    }
    return dqModulePromise;
}

async function ensureModel({ baseUrl = '/', modelSpec = null } = {}) {
    const modelKey = stableModelKey(modelSpec);
    if (!modelSpec || modelKey === 'null') {
        throw new Error('mj_restore_worker_model_spec_missing');
    }
    if (currentModel && currentModelKey === modelKey) {
        return currentModel;
    }

    const dqModule = await ensureDQModule(baseUrl);
    if (!dqModule || typeof dqModule.HumanoidURDF !== 'function') {
        throw new Error('mj_restore_worker_humanoid_urdf_unavailable');
    }

    currentModel = new dqModule.HumanoidURDF(
        Array.isArray(modelSpec.baseDQ) ? modelSpec.baseDQ : [1, 0, 0, 0, 0, 0, 0, 0],
        Array.isArray(modelSpec.waistChain) ? modelSpec.waistChain : [],
        Array.isArray(modelSpec.leftArmChain) ? modelSpec.leftArmChain : [],
        Array.isArray(modelSpec.rightArmChain) ? modelSpec.rightArmChain : [],
        Array.isArray(modelSpec.headChain) ? modelSpec.headChain : [],
        Array.isArray(modelSpec.waistEffector) ? modelSpec.waistEffector : [1, 0, 0, 0, 0, 0, 0, 0],
        Array.isArray(modelSpec.leftArmEffector) ? modelSpec.leftArmEffector : [1, 0, 0, 0, 0, 0, 0, 0],
        Array.isArray(modelSpec.rightArmEffector) ? modelSpec.rightArmEffector : [1, 0, 0, 0, 0, 0, 0, 0],
        Array.isArray(modelSpec.headEffector) ? modelSpec.headEffector : [1, 0, 0, 0, 0, 0, 0, 0]
    );
    currentModelKey = modelKey;
    return currentModel;
}

function asFinite(value, fallback = 0) {
    return Number.isFinite(value) ? value : fallback;
}

function cloneArray(values) {
    return Array.isArray(values) ? values.slice() : [];
}

function dot(a, b) {
    const count = Math.min(Array.isArray(a) ? a.length : 0, Array.isArray(b) ? b.length : 0);
    let sum = 0;
    for (let i = 0; i < count; i++) sum += asFinite(a[i]) * asFinite(b[i]);
    return sum;
}

function vectorNorm(values) {
    return Math.sqrt(dot(values, values));
}

function transpose(A) {
    const rows = Array.isArray(A) ? A.length : 0;
    const cols = rows > 0 && Array.isArray(A[0]) ? A[0].length : 0;
    return Array.from({ length: cols }, (_, col) =>
        Array.from({ length: rows }, (_, row) => asFinite(A[row]?.[col]))
    );
}

function matMul(A, B) {
    const rows = Array.isArray(A) ? A.length : 0;
    const inner = Array.isArray(B) ? B.length : 0;
    const cols = inner > 0 && Array.isArray(B[0]) ? B[0].length : 0;
    const out = Array.from({ length: rows }, () => new Array(cols).fill(0));
    for (let i = 0; i < rows; i++) {
        for (let k = 0; k < inner; k++) {
            const aik = asFinite(A[i]?.[k]);
            if (aik === 0) continue;
            for (let j = 0; j < cols; j++) {
                out[i][j] += aik * asFinite(B[k]?.[j]);
            }
        }
    }
    return out;
}

function det(A) {
    const n = Array.isArray(A) ? A.length : 0;
    if (n === 0) return 0;
    const M = A.map((row) => (Array.isArray(row) ? row.slice() : new Array(n).fill(0)));
    let sign = 1;
    for (let i = 0; i < n; i++) {
        let pivotRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(asFinite(M[k]?.[i])) > Math.abs(asFinite(M[pivotRow]?.[i]))) {
                pivotRow = k;
            }
        }
        if (pivotRow !== i) {
            [M[i], M[pivotRow]] = [M[pivotRow], M[i]];
            sign = -sign;
        }
        if (Math.abs(asFinite(M[i]?.[i])) < 1e-14) return 0;
        for (let k = i + 1; k < n; k++) {
            const factor = asFinite(M[k]?.[i]) / asFinite(M[i]?.[i]);
            for (let j = i; j < n; j++) {
                M[k][j] = asFinite(M[k]?.[j]) - factor * asFinite(M[i]?.[j]);
            }
        }
    }
    let value = sign;
    for (let i = 0; i < n; i++) value *= asFinite(M[i]?.[i]);
    return Number.isFinite(value) ? value : 0;
}

function hamiltonLeft4(q) {
    const [q0, q1, q2, q3] = (Array.isArray(q) ? q : []).map((value) => asFinite(value));
    return [
        [q0, -q1, -q2, -q3],
        [q1, q0, -q3, q2],
        [q2, q3, q0, -q1],
        [q3, -q2, q1, q0]
    ];
}

function hamiltonLeft8(x) {
    const r = (Array.isArray(x) ? x : []).slice(0, 4);
    const d = (Array.isArray(x) ? x : []).slice(4, 8);
    const Hr = hamiltonLeft4(r);
    const Hd = hamiltonLeft4(d);
    const zero = [0, 0, 0, 0];
    return [
        [...Hr[0], ...zero],
        [...Hr[1], ...zero],
        [...Hr[2], ...zero],
        [...Hr[3], ...zero],
        [...Hd[0], ...Hr[0]],
        [...Hd[1], ...Hr[1]],
        [...Hd[2], ...Hr[2]],
        [...Hd[3], ...Hr[3]]
    ];
}

function dqConjugate(x) {
    return [
        asFinite(x?.[0]),
        -asFinite(x?.[1]),
        -asFinite(x?.[2]),
        -asFinite(x?.[3]),
        -asFinite(x?.[4]),
        asFinite(x?.[5]),
        asFinite(x?.[6]),
        asFinite(x?.[7])
    ];
}

function computeTwistJacobian(J8, x8) {
    if (!Array.isArray(J8) || J8.length < 8 || !Array.isArray(x8) || x8.length < 8) return null;
    const width = Array.isArray(J8[0]) ? J8[0].length : 0;
    if (width <= 0) return null;
    const H = hamiltonLeft8(dqConjugate(x8));
    const rows = [1, 2, 3, 5, 6, 7];
    return rows.map((rowIndex) => {
        const out = new Array(width).fill(0);
        for (let col = 0; col < width; col++) {
            let sum = 0;
            for (let k = 0; k < 8; k++) {
                sum += asFinite(H[rowIndex]?.[k]) * asFinite(J8[k]?.[col]);
            }
            out[col] = 2 * sum;
        }
        return out;
    });
}

function signedPoseError(target, current) {
    const tgt = Array.isArray(target) ? target : [];
    const cur = Array.isArray(current) ? current : [];
    const count = Math.min(tgt.length, cur.length);
    let alignment = 0;
    for (let i = 0; i < count; i++) alignment += asFinite(tgt[i]) * asFinite(cur[i]);
    const sign = alignment < 0 ? -1 : 1;
    const error = new Array(count).fill(0);
    for (let i = 0; i < count; i++) error[i] = asFinite(tgt[i]) - sign * asFinite(cur[i]);
    return error;
}

function sliceActiveColumns(rows, activeStart, activeCount) {
    return (Array.isArray(rows) ? rows : []).map((row) =>
        Array.isArray(row) ? row.slice(activeStart, activeStart + activeCount) : new Array(activeCount).fill(0)
    );
}

function buildFullConfiguration(request, qActive) {
    const qFull = cloneArray(request?.qCurrent);
    const activeStart = Math.max(0, Math.floor(asFinite(request?.activeStart, 0)));
    const activeCount = Math.max(0, Math.floor(asFinite(request?.activeCount, 0)));
    for (let i = 0; i < activeCount; i++) {
        qFull[activeStart + i] = asFinite(qActive?.[i], qFull[activeStart + i]);
    }
    return qFull;
}

function buildManipulabilityVFIConstraints({
    qActive,
    lowerBounds,
    upperBounds,
    activationThreshold = 0.05
} = {}) {
    const constraints = [];
    const width = Array.isArray(qActive) ? qActive.length : 0;
    const threshold = Math.max(0, asFinite(activationThreshold, 0.05));
    if (!(threshold > 0)) return constraints;
    for (let i = 0; i < width; i++) {
        const qi = asFinite(qActive[i]);
        const dUp = Number.isFinite(upperBounds?.[i]) ? Math.max(0, upperBounds[i] - qi) : Infinity;
        const dLo = Number.isFinite(lowerBounds?.[i]) ? Math.max(0, qi - lowerBounds[i]) : Infinity;
        const dMin = Math.min(dUp, dLo);
        if (!Number.isFinite(dMin) || dMin >= threshold) continue;
        const row = new Array(width).fill(0);
        row[i] = 1;
        constraints.push({ row, distance: dMin, threshold, source: 'joint_limit' });
    }
    return constraints;
}

function computeVFIAttenuationMatrix(constraints, width) {
    const W = Array.from({ length: width }, (_, row) =>
        Array.from({ length: width }, (_, col) => (row === col ? 1 : 0))
    );
    for (const constraint of Array.isArray(constraints) ? constraints : []) {
        if (!Array.isArray(constraint?.row) || !Number.isFinite(constraint?.distance) || !Number.isFinite(constraint?.threshold)) continue;
        if (!(constraint.threshold > 0)) continue;
        const omega = constraint.distance / constraint.threshold;
        if (!(omega >= 0 && omega < 1)) continue;
        const baseScale = 1 - omega;
        const scale = baseScale;
        const norm2 = constraint.row.reduce((sum, value) => sum + asFinite(value) * asFinite(value), 0);
        if (!(norm2 > 1e-24)) continue;
        for (let i = 0; i < width; i++) {
            for (let j = 0; j < width; j++) {
                W[i][j] -= scale * asFinite(constraint.row[i]) * asFinite(constraint.row[j]) / norm2;
            }
        }
    }
    return W;
}

function computeCombinedCCTSM(JCommand, JInvariant) {
    const JFull = [...(Array.isArray(JCommand) ? JCommand : []), ...(Array.isArray(JInvariant) ? JInvariant : [])];
    if (JFull.length === 0) return null;
    const Gram = matMul(JFull, transpose(JFull));
    return {
        w: Math.sqrt(Math.max(0, asFinite(det(Gram), 0))),
        Gram,
        J_full: JFull
    };
}

function computeCombinedVFICCTSM(JFull, attenuation) {
    if (!Array.isArray(JFull) || !Array.isArray(attenuation)) return null;
    const Gvfi = matMul(matMul(JFull, attenuation), transpose(JFull));
    return {
        w_vfi: Math.sqrt(Math.max(0, asFinite(det(Gvfi), 0))),
        G_vfi: Gvfi
    };
}

function blendHierarchicalVFIValue(baseline, vfiValue, hierarchicalBlend = 0.5) {
    if (!Number.isFinite(baseline) && !Number.isFinite(vfiValue)) return Number.NaN;
    if (!Number.isFinite(baseline)) return vfiValue;
    if (!Number.isFinite(vfiValue)) return baseline;
    const blended = (1 - hierarchicalBlend) * baseline + hierarchicalBlend * vfiValue;
    return Math.min(baseline, blended);
}

function solveLinearSystem(A, b) {
    const n = Array.isArray(A) ? A.length : 0;
    if (n === 0) return [];
    const M = A.map((row, index) => {
        const out = new Array(n + 1).fill(0);
        for (let j = 0; j < n; j++) out[j] = asFinite(row?.[j]);
        out[n] = asFinite(b?.[index]);
        return out;
    });
    for (let col = 0; col < n; col++) {
        let pivotRow = col;
        let pivotMag = Math.abs(asFinite(M[col]?.[col]));
        for (let row = col + 1; row < n; row++) {
            const mag = Math.abs(asFinite(M[row]?.[col]));
            if (mag > pivotMag) {
                pivotMag = mag;
                pivotRow = row;
            }
        }
        if (pivotMag < 1e-12) return null;
        if (pivotRow !== col) [M[col], M[pivotRow]] = [M[pivotRow], M[col]];
        const pivot = asFinite(M[col][col]);
        for (let j = col; j <= n; j++) M[col][j] /= pivot;
        for (let row = 0; row < n; row++) {
            if (row === col) continue;
            const factor = asFinite(M[row][col]);
            if (Math.abs(factor) < 1e-18) continue;
            for (let j = col; j <= n; j++) M[row][j] -= factor * M[col][j];
        }
    }
    return M.map((row) => asFinite(row[n]));
}

function matVec(A, x) {
    return (Array.isArray(A) ? A : []).map((row) => dot(row, x));
}

function buildProjector(equalityJacobian, width, damping) {
    const rows = (Array.isArray(equalityJacobian) ? equalityJacobian : [])
        .filter((row) => Array.isArray(row) && row.length === width);
    if (rows.length === 0) {
        return Array.from({ length: width }, (_, row) =>
            Array.from({ length: width }, (_, col) => (row === col ? 1 : 0))
        );
    }
    const gram = rows.map((rowI, i) => rows.map((rowJ, j) => {
        const value = dot(rowI, rowJ);
        return i === j ? value + damping : value;
    }));
    const projector = Array.from({ length: width }, (_, row) =>
        Array.from({ length: width }, (_, col) => (row === col ? 1 : 0))
    );
    for (let col = 0; col < width; col++) {
        const rhs = rows.map((row) => asFinite(row?.[col]));
        const alpha = solveLinearSystem(gram, rhs);
        if (!Array.isArray(alpha) || alpha.length !== rows.length) continue;
        for (let row = 0; row < width; row++) {
            let correction = 0;
            for (let i = 0; i < rows.length; i++) {
                correction += asFinite(rows[i]?.[row]) * asFinite(alpha[i]);
            }
            projector[row][col] -= correction;
        }
    }
    return projector;
}

function clampToBounds(qCurrent, step, lowerBounds, upperBounds, stepBounds) {
    const width = Math.min(
        Array.isArray(qCurrent) ? qCurrent.length : 0,
        Array.isArray(step) ? step.length : 0
    );
    const qNext = new Array(width).fill(0);
    const dqClamped = new Array(width).fill(0);
    let clipped = false;
    for (let i = 0; i < width; i++) {
        const q0 = asFinite(qCurrent[i]);
        const lower = Number.isFinite(lowerBounds?.[i]) ? lowerBounds[i] : Number.NEGATIVE_INFINITY;
        const upper = Number.isFinite(upperBounds?.[i]) ? upperBounds[i] : Number.POSITIVE_INFINITY;
        const stepLimit = Number.isFinite(stepBounds?.[i]) ? Math.max(0, stepBounds[i]) : Number.POSITIVE_INFINITY;
        const qMin = Math.max(lower, q0 - stepLimit);
        const qMax = Math.min(upper, q0 + stepLimit);
        const qCandidate = q0 + asFinite(step[i]);
        const qClamped = Math.min(qMax, Math.max(qMin, qCandidate));
        qNext[i] = qClamped;
        dqClamped[i] = qClamped - q0;
        if (Math.abs(qClamped - qCandidate) > 1e-12) clipped = true;
    }
    return { qNext, step: dqClamped, clipped };
}

function projectSecondaryGradient(gradient, protectRows = [], damping = 1e-6) {
    const grad = Array.isArray(gradient) ? gradient.map((value) => asFinite(value)) : [];
    const width = grad.length;
    if (width === 0) return [];
    const rows = (Array.isArray(protectRows) ? protectRows : [])
        .filter((row) => Array.isArray(row) && row.length === width);
    if (rows.length === 0) return grad;
    const rhs = matVec(rows, grad);
    const gramBase = rows.map((rowI) => rows.map((rowJ) => dot(rowI, rowJ)));
    let alpha = solveLinearSystem(gramBase, rhs);
    if (!Array.isArray(alpha) || alpha.length !== rows.length) {
        const gramDamped = gramBase.map((row, i) => row.map((value, j) => (
            i === j ? value + damping : value
        )));
        alpha = solveLinearSystem(gramDamped, rhs);
    }
    if (!Array.isArray(alpha) || alpha.length !== rows.length) {
        return new Array(width).fill(0);
    }
    const projected = grad.slice();
    for (let i = 0; i < rows.length; i++) {
        const scale = asFinite(alpha[i]);
        for (let j = 0; j < width; j++) {
            projected[j] -= scale * asFinite(rows[i]?.[j]);
        }
    }
    const norm = vectorNorm(projected);
    if (!(norm > 1e-10)) return new Array(width).fill(0);
    return projected.map((value) => value / norm);
}

function computeNullspaceStep({
    qCurrent,
    equalityJacobian,
    objectiveGradient,
    wvfiGradient,
    stepSize,
    lowerBounds,
    upperBounds,
    stepBounds,
    evaluateWVFI,
    wvfiMin = 0,
    preferWVFIFallbackOnClip = false
} = {}) {
    const width = Array.isArray(qCurrent) ? qCurrent.length : 0;
    const zeroStep = new Array(width).fill(0);
    if (width === 0) {
        return { accepted: false, reason: 'invalid_input', step: zeroStep, qNext: cloneArray(qCurrent) };
    }
    const projector = buildProjector(equalityJacobian, width, 1e-12);
    const objectiveStep = Array.isArray(objectiveGradient)
        ? objectiveGradient.slice(0, width).map((value) => asFinite(value))
        : zeroStep.slice();
    const wvfiStep = Array.isArray(wvfiGradient)
        ? wvfiGradient.slice(0, width).map((value) => asFinite(value))
        : zeroStep.slice();
    const objectiveProjected = matVec(projector, objectiveStep);
    const objectiveProjectedNorm = vectorNorm(objectiveProjected);
    const wvfiProjected = matVec(projector, wvfiStep);
    const wvfiProjectedNorm = vectorNorm(wvfiProjected);

    let selected = null;
    let selectedSource = 'objective';
    if (objectiveProjectedNorm > 1e-10) {
        selected = objectiveProjected;
    } else if (wvfiProjectedNorm > 1e-10) {
        selected = wvfiProjected;
        selectedSource = 'wvfi';
    }

    if (!Array.isArray(selected) || selected.length === 0) {
        return { accepted: false, reason: 'no_nullspace', step: zeroStep, qNext: cloneArray(qCurrent) };
    }
    const buildCandidate = (projectedGradient, projectedNorm, source) => {
        if (!Array.isArray(projectedGradient) || projectedGradient.length === 0 || projectedNorm <= 1e-10) {
            return null;
        }
        const scale = asFinite(stepSize, 1) / projectedNorm;
        const rawStep = projectedGradient.map((value) => value * scale);
        const clamped = clampToBounds(qCurrent, rawStep, lowerBounds, upperBounds, stepBounds);
        if (typeof evaluateWVFI === 'function') {
            const wvfi = evaluateWVFI(clamped.qNext, clamped.step);
            if (Number.isFinite(wvfiMin) && wvfiMin > 0 && (!Number.isFinite(wvfi) || wvfi + 1e-12 < wvfiMin)) {
                return {
                    accepted: false,
                    reason: 'wvfi_lower_bound',
                    gradientSource: source
                };
            }
        }
        return {
            accepted: true,
            reason: clamped.clipped ? 'accepted_clipped' : 'accepted',
            gradientSource: source,
            step: clamped.step,
            qNext: clamped.qNext,
            clipped: clamped.clipped
        };
    };

    const objectiveCandidate = selectedSource === 'objective'
        ? buildCandidate(objectiveProjected, objectiveProjectedNorm, 'objective')
        : null;
    const wvfiCandidate = buildCandidate(wvfiProjected, wvfiProjectedNorm, 'wvfi');

    let selectedCandidate = null;
    if (objectiveCandidate?.accepted === true) {
        selectedCandidate = objectiveCandidate;
        if (preferWVFIFallbackOnClip === true &&
            objectiveCandidate.clipped === true &&
            wvfiCandidate?.accepted === true) {
            selectedCandidate = wvfiCandidate;
        }
    } else if (wvfiCandidate?.accepted === true) {
        selectedCandidate = wvfiCandidate;
    }

    if (!selectedCandidate) {
        const rejected = objectiveCandidate || wvfiCandidate;
        return {
            accepted: false,
            reason: rejected?.reason ?? 'no_nullspace',
            step: zeroStep,
            qNext: cloneArray(qCurrent),
            gradientSource: rejected?.gradientSource ?? selectedSource
        };
    }

    return {
        accepted: true,
        reason: selectedCandidate.reason,
        gradientSource: selectedCandidate.gradientSource,
        step: selectedCandidate.step,
        qNext: selectedCandidate.qNext
    };
}

function evaluateObjectiveDirection(before, after, direction = 'increase', tolerance = 1e-12) {
    const delta = Number.isFinite(before) && Number.isFinite(after) ? after - before : Number.NaN;
    const wantsDecrease = direction === 'decrease';
    const applied = Number.isFinite(delta) && (wantsDecrease ? (delta < -tolerance) : (delta > tolerance));
    return {
        applied,
        reason: applied ? 'improved' : 'not_improved'
    };
}

function evaluateNonDegradation(before, after, direction = 'increase', tolerance = 1e-12) {
    const delta = Number.isFinite(before) && Number.isFinite(after) ? after - before : Number.NaN;
    const wantsDecrease = direction === 'decrease';
    const applied = Number.isFinite(delta) && (wantsDecrease ? (delta <= tolerance) : (delta >= -tolerance));
    return {
        applied,
        reason: applied ? 'protected' : 'degraded'
    };
}

function computeRotationMatrixFromPose(pose) {
    const w = asFinite(pose?.[0], 1);
    const x = asFinite(pose?.[1]);
    const y = asFinite(pose?.[2]);
    const z = asFinite(pose?.[3]);
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
    ];
}

function buildBodyConstraint(model, dqModule, request, qFull) {
    if (request?.keepBodyZVertical !== true) return null;
    if (!model || !dqModule?.Kinematics?.lineJacobian || typeof model.fkmEffector !== 'function' || typeof model.poseJacobianEffector !== 'function') {
        return null;
    }
    const waistPose = model.fkmEffector(qFull, 0);
    const waistPoseJacobianFull = model.poseJacobianEffector(qFull, 0);
    const lineDirection = [0, 0, 0, 1, 0, 0, 0, 0];
    const lineJacobianFull = dqModule.Kinematics.lineJacobian(waistPoseJacobianFull, waistPose, lineDirection);
    const lineJacobianActive = sliceActiveColumns(lineJacobianFull, request.activeStart, request.activeCount);
    const rotation = computeRotationMatrixFromPose(waistPose);
    const diff0 = 0;
    const diff1 = -rotation[0][2];
    const diff2 = -rotation[1][2];
    const diff3 = 1 - rotation[2][2];
    const mismatchSquared = diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
    const residual = [-mismatchSquared];
    const jacobianRow = new Array(request.activeCount).fill(0);
    for (let j = 0; j < request.activeCount; j++) {
        jacobianRow[j] = -2 * (
            diff0 * asFinite(lineJacobianActive?.[0]?.[j]) +
            diff1 * asFinite(lineJacobianActive?.[1]?.[j]) +
            diff2 * asFinite(lineJacobianActive?.[2]?.[j]) +
            diff3 * asFinite(lineJacobianActive?.[3]?.[j])
        );
    }
    return {
        residual,
        jacobian: [jacobianRow]
    };
}

function buildSecondaryProblem(model, dqModule, request, secondaryMetricRequest, qActive) {
    const qFull = buildFullConfiguration(request, qActive);
    const absolutePose = model.absolutePoseDualArm(qFull);
    const relativePose = model.relativePoseDualArm(qFull);
    const absoluteJacobianFull = model.absolutePoseJacobianDualArm(qFull);
    const relativeJacobianFull = model.relativePoseJacobianDualArm(qFull);
    const absoluteJacobian = sliceActiveColumns(absoluteJacobianFull, request.activeStart, request.activeCount);
    const relativeJacobian = sliceActiveColumns(relativeJacobianFull, request.activeStart, request.activeCount);
    const bodyConstraint = buildBodyConstraint(model, dqModule, request, qFull);

    const absoluteError = signedPoseError(request.absoluteTarget, absolutePose);
    const relativeError = signedPoseError(request.relativeTarget, relativePose);
    const equalityJacobian = [
        ...absoluteJacobian.map((row) => row.slice()),
        ...relativeJacobian.map((row) => row.slice()),
        ...(Array.isArray(bodyConstraint?.jacobian) ? bodyConstraint.jacobian.map((row) => row.slice()) : [])
    ];

    const absoluteTwist = computeTwistJacobian(absoluteJacobian, absolutePose);
    const relativeTwist = computeTwistJacobian(relativeJacobian, relativePose);
    const width = request.activeCount;
    const attenuation = computeVFIAttenuationMatrix(
        buildManipulabilityVFIConstraints({
            qActive,
            lowerBounds: request.lowerBounds,
            upperBounds: request.upperBounds,
            activationThreshold: request.jointLimitActivationThreshold
        }),
        width
    );
    const combined = computeCombinedCCTSM(absoluteTwist, relativeTwist);
    const absoluteOnly = computeCombinedCCTSM(absoluteTwist, []);
    const relativeOnly = computeCombinedCCTSM(relativeTwist, []);
    const waistActiveCount = Math.max(0, Math.min(
        Number.isFinite(secondaryMetricRequest?.waistActiveCount)
            ? Math.floor(secondaryMetricRequest.waistActiveCount)
            : 0,
        width
    ));
    const armOnlyColumns = width - waistActiveCount;
    const absoluteArmOnly = armOnlyColumns > 0
        ? computeCombinedCCTSM(
            absoluteTwist.map((row) => row.slice(waistActiveCount, waistActiveCount + armOnlyColumns)),
            []
        )
        : absoluteOnly;
    const attenuationArm = armOnlyColumns > 0
        ? attenuation.slice(waistActiveCount, waistActiveCount + armOnlyColumns)
            .map((row) => Array.isArray(row) ? row.slice(waistActiveCount, waistActiveCount + armOnlyColumns) : new Array(armOnlyColumns).fill(0))
        : attenuation;
    const combinedVfi = computeCombinedVFICCTSM(combined?.J_full ?? [], attenuation);
    const absoluteVfi = computeCombinedVFICCTSM(absoluteTwist ?? [], attenuation);
    const relativeVfi = computeCombinedVFICCTSM(relativeTwist ?? [], attenuation);
    const absoluteArmVfi = armOnlyColumns > 0
        ? computeCombinedVFICCTSM(
            absoluteTwist.map((row) => row.slice(waistActiveCount, waistActiveCount + armOnlyColumns)),
            attenuationArm
        )
        : absoluteVfi;
    const hierarchicalBlend = Number.isFinite(secondaryMetricRequest?.hierarchicalVfiBlend)
        ? Math.max(0, Math.min(1, secondaryMetricRequest.hierarchicalVfiBlend))
        : 0.5;
    const absoluteVfiValue = absoluteVfi?.w_vfi;
    const absoluteOnlyValue = absoluteOnly?.w;
    const absoluteHierValue = Number.isFinite(absoluteOnlyValue)
        ? absoluteOnlyValue
        : (Number.isFinite(absoluteVfiValue) ? absoluteVfiValue : Number.NaN);
    const absoluteHierVfiValue = blendHierarchicalVFIValue(absoluteHierValue, absoluteVfiValue, hierarchicalBlend);
    const relativeOnlyValue = relativeOnly?.w;
    const relativeVfiValue = relativeVfi?.w_vfi;
    const relativeHierValue = Number.isFinite(relativeOnlyValue)
        ? relativeOnlyValue
        : (Number.isFinite(relativeVfiValue) ? relativeVfiValue : Number.NaN);
    const relativeHierVfiValue = blendHierarchicalVFIValue(relativeHierValue, relativeVfiValue, hierarchicalBlend);
    const normalizedMetricMode = typeof secondaryMetricRequest?.metricMode === 'string' && secondaryMetricRequest.metricMode.trim().length > 0
        ? secondaryMetricRequest.metricMode.trim()
        : 'absolute';
    const commandRows = normalizedMetricMode === 'relative'
        ? relativeTwist.map((row) => row.slice())
        : absoluteJacobian.map((row) => row.slice());
    const protectedRows = normalizedMetricMode === 'relative'
        ? absoluteTwist.map((row) => row.slice())
        : [
            ...relativeJacobian.map((row) => row.slice()),
            ...(Array.isArray(bodyConstraint?.jacobian) ? bodyConstraint.jacobian.map((row) => row.slice()) : [])
        ];
    let taskMetricKind = 'absolute_pose';
    let wTask = absoluteOnlyValue;
    let wTaskVfi = absoluteVfiValue;
    let wTaskArm = absoluteArmOnly?.w ?? Number.NaN;
    let wTaskArmVfi = absoluteArmVfi?.w_vfi ?? Number.NaN;
    let wTaskHier = absoluteHierValue;
    let wTaskHierVfi = absoluteHierVfiValue;
    if (normalizedMetricMode === 'relative') {
        taskMetricKind = 'relative_pose';
        wTask = relativeOnlyValue;
        wTaskVfi = relativeVfiValue;
        wTaskArm = relativeOnlyValue;
        wTaskArmVfi = relativeVfiValue;
        wTaskHier = relativeHierValue;
        wTaskHierVfi = relativeHierVfiValue;
    }

    return {
        qActive: cloneArray(qActive),
        qFull,
        equalities: {
            jacobian: equalityJacobian,
            residualNorm: vectorNorm([
                ...absoluteError,
                ...relativeError,
                ...(Array.isArray(bodyConstraint?.residual) ? bodyConstraint.residual : [])
            ])
        },
        metadata: {
            Jabs: absoluteJacobian,
            Jcmd: commandRows,
            protectedRows,
            cctsm: {
                w_vfi: combinedVfi?.w_vfi ?? Number.NaN,
                w_abs: absoluteOnly?.w ?? Number.NaN,
                w_abs_hier: absoluteHierValue,
                w_abs_hier_vfi: absoluteHierVfiValue,
                w_abs_arm: absoluteArmOnly?.w ?? Number.NaN,
                w_abs_arm_vfi: absoluteArmVfi?.w_vfi ?? Number.NaN,
                w_abs_vfi: absoluteVfi?.w_vfi ?? Number.NaN,
                w_task: wTask,
                w_task_vfi: wTaskVfi,
                w_task_arm: wTaskArm,
                w_task_arm_vfi: wTaskArmVfi,
                w_task_hier: wTaskHier,
                w_task_hier_vfi: wTaskHierVfi,
                taskMetricKind,
                hierarchicalVfiBlend: hierarchicalBlend
            }
        }
    };
}

function evaluateSecondaryMetricValue(metric, cctsm, metricMode = 'absolute') {
    const normalizedMetricMode = typeof metricMode === 'string' && metricMode.trim().length > 0
        ? metricMode.trim()
        : 'absolute';
    const useTaskMetric = normalizedMetricMode !== 'absolute';
    if (useTaskMetric) {
        if (metric === 'w_abs_hier' || metric === 'w_abs_hier_vfi') {
            const hierarchical = Number.isFinite(cctsm?.w_task_hier) ? cctsm.w_task_hier : Number.NaN;
            if (metric === 'w_abs_hier') {
                return hierarchical;
            }
            const hierarchicalVfi = Number.isFinite(cctsm?.w_task_hier_vfi) ? cctsm.w_task_hier_vfi : Number.NaN;
            if (Number.isFinite(hierarchicalVfi)) return hierarchicalVfi;
            return hierarchical;
        }
        if (metric === 'w_abs_arm') return Number.isFinite(cctsm?.w_task_arm) ? cctsm.w_task_arm : Number.NaN;
        if (metric === 'w_abs_arm_vfi') return Number.isFinite(cctsm?.w_task_arm_vfi) ? cctsm.w_task_arm_vfi : Number.NaN;
        if (metric === 'w_abs_vfi') return Number.isFinite(cctsm?.w_task_vfi) ? cctsm.w_task_vfi : Number.NaN;
        if (metric === 'w_abs') return Number.isFinite(cctsm?.w_task) ? cctsm.w_task : Number.NaN;
    }
    if (metric === 'w_abs_hier' || metric === 'w_abs_hier_vfi') {
        const wAbs = Number.isFinite(cctsm?.w_abs) ? cctsm.w_abs : Number.NaN;
        if (!Number.isFinite(wAbs)) return Number.NaN;

        if (metric === 'w_abs_hier') {
            return wAbs;
        }

        const explicitHierarchicalVfi = Number.isFinite(cctsm?.w_abs_hier_vfi)
            ? cctsm.w_abs_hier_vfi
            : Number.NaN;
        if (Number.isFinite(explicitHierarchicalVfi)) {
            return explicitHierarchicalVfi;
        }

        const hierarchicalBlend = Number.isFinite(cctsm?.hierarchicalVfiBlend)
            ? Math.max(0, Math.min(1, cctsm.hierarchicalVfiBlend))
            : 0.5;
        const wAbsVfi = Number.isFinite(cctsm?.w_abs_vfi)
            ? cctsm.w_abs_vfi
            : Number.NaN;
        if (!Number.isFinite(wAbsVfi)) return wAbs;
        return hierarchicalBlend * wAbsVfi + (1 - hierarchicalBlend) * wAbs;
    }
    if (metric === 'w_abs_arm') return Number.isFinite(cctsm?.w_abs_arm) ? cctsm.w_abs_arm : Number.NaN;
    if (metric === 'w_abs_arm_vfi') return Number.isFinite(cctsm?.w_abs_arm_vfi) ? cctsm.w_abs_arm_vfi : Number.NaN;
    if (metric === 'w_abs_vfi') return Number.isFinite(cctsm?.w_abs_vfi) ? cctsm.w_abs_vfi : Number.NaN;
    if (metric === 'w_abs') return Number.isFinite(cctsm?.w_abs) ? cctsm.w_abs : Number.NaN;
    return Number.isFinite(cctsm?.w_vfi) ? cctsm.w_vfi : Number.NaN;
}

function computeProtectedMetricGradient(model, dqModule, request, secondaryMetricRequest, baseProblem, metric) {
    const qActive = cloneArray(baseProblem?.qActive);
    if (qActive.length === 0) return [];
    const protectRows = [
        ...(Array.isArray(baseProblem?.metadata?.Jcmd)
            ? baseProblem.metadata.Jcmd.map((row) => row.slice())
            : (Array.isArray(baseProblem?.metadata?.Jabs) ? baseProblem.metadata.Jabs.map((row) => row.slice()) : [])),
        ...(Array.isArray(baseProblem?.metadata?.protectedRows) ? baseProblem.metadata.protectedRows.map((row) => row.slice()) : [])
    ];
    const eps = 1e-5;
    const rawGradient = new Array(qActive.length).fill(0);
    for (let i = 0; i < qActive.length; i++) {
        const qp = qActive.slice();
        const qm = qActive.slice();
        qp[i] += eps;
        qm[i] -= eps;
        const plusProblem = buildSecondaryProblem(model, dqModule, request, secondaryMetricRequest, qp);
        const minusProblem = buildSecondaryProblem(model, dqModule, request, secondaryMetricRequest, qm);
        const plusValue = evaluateSecondaryMetricValue(metric, plusProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode);
        const minusValue = evaluateSecondaryMetricValue(metric, minusProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode);
        if (Number.isFinite(plusValue) && Number.isFinite(minusValue)) {
            rawGradient[i] = (plusValue - minusValue) / (2 * eps);
        }
    }
    return projectSecondaryGradient(rawGradient, protectRows, 1e-6);
}

function runRestoreFromActive(model, request, qActive, restoreMaxIterations) {
    const qFull = buildFullConfiguration(request, qActive);
    const restoreRequest = {
        ...request,
        qCurrent: qFull,
        maxIterations: Number.isFinite(restoreMaxIterations)
            ? Math.max(1, Math.floor(restoreMaxIterations))
            : request.maxIterations
    };
    return model.solveAbsoluteTargetMJRestore(restoreRequest);
}

function attemptSecondaryMetricRefinement(model, dqModule, request, secondaryMetricRequest, baseProblem, metric) {
    const before = evaluateSecondaryMetricValue(metric, baseProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode);
    const gradient = computeProtectedMetricGradient(model, dqModule, request, secondaryMetricRequest, baseProblem, metric);
    const wvfiGradient = computeProtectedMetricGradient(model, dqModule, request, secondaryMetricRequest, baseProblem, 'w_vfi');
    const hasObjectiveGradient = Array.isArray(gradient) && gradient.length > 0 && vectorNorm(gradient) > 1e-10;
    const hasWVFIGradient = Array.isArray(wvfiGradient) && wvfiGradient.length > 0 && vectorNorm(wvfiGradient) > 1e-10;
    if (!hasObjectiveGradient && !hasWVFIGradient) {
        return {
            state: 'skipped(no_gradient)',
            before,
            after: before,
            applied: false,
            finalProblem: baseProblem,
            refineResult: null,
            nullspaceResult: null
        };
    }
    const overallMetric = secondaryMetricRequest?.metric;
    const nullspaceResult = computeNullspaceStep({
        qCurrent: baseProblem.qActive,
        equalityJacobian: baseProblem?.equalities?.jacobian ?? [],
        objectiveGradient: gradient,
        wvfiGradient,
        stepSize: secondaryMetricRequest?.stepSize,
        lowerBounds: request.lowerBounds,
        upperBounds: request.upperBounds,
        stepBounds: request.stepBounds,
        evaluateWVFI: (qNextActive) => {
            const problem = buildSecondaryProblem(model, dqModule, request, secondaryMetricRequest, qNextActive);
            return problem?.metadata?.cctsm?.w_vfi ?? Number.NaN;
        },
        wvfiMin: Number.isFinite(request?.wvfiLowerBound) ? Math.max(0, request.wvfiLowerBound) : 0,
        preferWVFIFallbackOnClip:
            metric === 'w_abs_vfi' ||
            (overallMetric === 'w_abs_hier_vfi' && metric === 'w_abs_arm_vfi')
    });
    if (nullspaceResult.accepted !== true) {
        return {
            state: `skipped(${nullspaceResult.reason ?? 'no_nullspace'})`,
            before,
            after: before,
            applied: false,
            finalProblem: baseProblem,
            refineResult: null,
            nullspaceResult
        };
    }
    const refineResult = runRestoreFromActive(
        model,
        request,
        nullspaceResult.qNext,
        secondaryMetricRequest?.restoreMaxIterations
    );
    const finalProblem = buildSecondaryProblem(
        model,
        dqModule,
        request,
        secondaryMetricRequest,
        Array.isArray(refineResult?.qApplied) ? refineResult.qApplied : baseProblem.qActive
    );
    const after = evaluateSecondaryMetricValue(metric, finalProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode);
    if (refineResult?.converged !== true) {
        return {
            state: 'rejected(hard_constraints)',
            before,
            after,
            applied: false,
            finalProblem,
            refineResult,
            nullspaceResult
        };
    }
    let direction = evaluateObjectiveDirection(before, after, 'increase', 1e-12);
    if (direction.applied !== true &&
        nullspaceResult?.gradientSource === 'wvfi' &&
        overallMetric === 'w_abs_hier_vfi' &&
        metric === 'w_abs_arm_vfi') {
        const fallbackDirection = evaluateObjectiveDirection(
            evaluateSecondaryMetricValue('w_abs_hier_vfi', baseProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode),
            evaluateSecondaryMetricValue('w_abs_hier_vfi', finalProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode),
            'increase',
            1e-12
        );
        if (fallbackDirection.applied === true) {
            direction = {
                applied: true,
                reason: 'fallback_metric(w_abs_hier_vfi)'
            };
        } else {
            const safetyDirection = evaluateObjectiveDirection(
                evaluateSecondaryMetricValue('w_vfi', baseProblem?.metadata?.cctsm),
                evaluateSecondaryMetricValue('w_vfi', finalProblem?.metadata?.cctsm),
                'increase',
                1e-12
            );
            const requestedNonDegraded =
                Number.isFinite(before) &&
                Number.isFinite(after) &&
                after + 1e-6 >= before;
            if (safetyDirection.applied === true && requestedNonDegraded) {
                direction = {
                    applied: true,
                    reason: 'fallback_metric(w_vfi)'
                };
            }
        }
    }
    return {
        state: direction.applied ? 'active' : 'rejected(direction_mismatch)',
        before,
        after,
        applied: direction.applied === true,
        finalProblem,
        refineResult,
        nullspaceResult
    };
}

function applySecondaryMetric(model, dqModule, request, secondaryMetricRequest, restoreResult) {
    const overallMetric = secondaryMetricRequest?.metric;
    if (!overallMetric) return null;
    if (!restoreResult || restoreResult.converged !== true || !Array.isArray(restoreResult.qApplied)) return null;

    const baseProblem = buildSecondaryProblem(
        model,
        dqModule,
        request,
        secondaryMetricRequest,
        restoreResult.qApplied
    );

    if (overallMetric === 'w_abs_arm' || overallMetric === 'w_abs_arm_vfi' || overallMetric === 'w_abs' || overallMetric === 'w_abs_vfi') {
        const directAttempt = attemptSecondaryMetricRefinement(
            model,
            dqModule,
            request,
            secondaryMetricRequest,
            baseProblem,
            overallMetric
        );
        return {
            converged: directAttempt.refineResult?.converged === true
                ? true
                : restoreResult.converged === true,
            reason: typeof directAttempt.refineResult?.reason === 'string' && directAttempt.refineResult.reason.trim()
                ? directAttempt.refineResult.reason
                : restoreResult.reason,
            iterations: Number.isFinite(directAttempt.refineResult?.iterations)
                ? directAttempt.refineResult.iterations
                : restoreResult.iterations,
            acceptedIterations: Number.isFinite(directAttempt.refineResult?.acceptedIterations)
                ? directAttempt.refineResult.acceptedIterations
                : restoreResult.acceptedIterations,
            lastAcceptedScale: Number.isFinite(directAttempt.refineResult?.lastAcceptedScale)
                ? directAttempt.refineResult.lastAcceptedScale
                : restoreResult.lastAcceptedScale,
            commandResidualNorm: Number.isFinite(directAttempt.refineResult?.commandResidualNorm)
                ? directAttempt.refineResult.commandResidualNorm
                : restoreResult.commandResidualNorm,
            relativeResidualNorm: Number.isFinite(directAttempt.refineResult?.relativeResidualNorm)
                ? directAttempt.refineResult.relativeResidualNorm
                : restoreResult.relativeResidualNorm,
            bodyResidualNorm: Number.isFinite(directAttempt.refineResult?.bodyResidualNorm)
                ? directAttempt.refineResult.bodyResidualNorm
                : restoreResult.bodyResidualNorm,
            inequalityViolation: Number.isFinite(directAttempt.refineResult?.inequalityViolation)
                ? directAttempt.refineResult.inequalityViolation
                : restoreResult.inequalityViolation,
            referenceResidual: Array.isArray(directAttempt.refineResult?.referenceResidual)
                ? directAttempt.refineResult.referenceResidual.slice()
                : cloneArray(restoreResult.referenceResidual),
            constraintMotion: Array.isArray(directAttempt.refineResult?.constraintMotion)
                ? directAttempt.refineResult.constraintMotion.slice()
                : cloneArray(restoreResult.constraintMotion),
            qApplied: Array.isArray(directAttempt.refineResult?.qApplied)
                ? directAttempt.refineResult.qApplied.slice()
                : cloneArray(restoreResult.qApplied),
            secondaryMetric: overallMetric,
            secondaryState: directAttempt.state,
            secondaryBefore: directAttempt.before,
            secondaryAfter: directAttempt.after,
            secondaryApplied: directAttempt.applied === true
        };
    }

    if (overallMetric !== 'w_abs_hier_vfi' && overallMetric !== 'w_abs_hier') return null;
    const overallBefore = evaluateSecondaryMetricValue(overallMetric, baseProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode);
    const primaryMetric = overallMetric === 'w_abs_hier_vfi' ? 'w_abs_arm_vfi' : 'w_abs_arm';
    const primaryAttempt = attemptSecondaryMetricRefinement(
        model,
        dqModule,
        request,
        secondaryMetricRequest,
        baseProblem,
        primaryMetric
    );

    if (primaryAttempt.applied !== true || !primaryAttempt.refineResult) {
        return {
            secondaryMetric: overallMetric,
            secondaryState: primaryAttempt.state,
            secondaryBefore: overallBefore,
            secondaryAfter: overallBefore,
            secondaryApplied: false
        };
    }

    let finalResult = primaryAttempt.refineResult;
    let finalProblem = primaryAttempt.finalProblem;
    let state = primaryAttempt.state;
    const secondaryAttempt = attemptSecondaryMetricRefinement(
        model,
        dqModule,
        request,
        secondaryMetricRequest,
        primaryAttempt.finalProblem,
        overallMetric === 'w_abs_hier_vfi' ? 'w_abs_vfi' : 'w_abs'
    );
    const armGuard = evaluateNonDegradation(
        evaluateSecondaryMetricValue(primaryMetric, primaryAttempt.finalProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode),
        evaluateSecondaryMetricValue(primaryMetric, secondaryAttempt.finalProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode),
        'increase',
        1e-12
    );
    if (secondaryAttempt.applied === true && armGuard.applied === true && secondaryAttempt.refineResult) {
        finalResult = secondaryAttempt.refineResult;
        finalProblem = secondaryAttempt.finalProblem;
        state = 'active';
    } else {
        state = secondaryAttempt.applied !== true
            ? `active(primary_only; secondary=${secondaryAttempt.state})`
            : 'active(primary_only; secondary=arm_guard)';
    }

    return {
        converged: finalResult?.converged === true,
        reason: typeof finalResult?.reason === 'string' && finalResult.reason.trim()
            ? finalResult.reason
            : restoreResult.reason,
        iterations: Number.isFinite(finalResult?.iterations) ? finalResult.iterations : restoreResult.iterations,
        acceptedIterations: Number.isFinite(finalResult?.acceptedIterations)
            ? finalResult.acceptedIterations
            : restoreResult.acceptedIterations,
        lastAcceptedScale: Number.isFinite(finalResult?.lastAcceptedScale)
            ? finalResult.lastAcceptedScale
            : restoreResult.lastAcceptedScale,
        commandResidualNorm: Number.isFinite(finalResult?.commandResidualNorm)
            ? finalResult.commandResidualNorm
            : restoreResult.commandResidualNorm,
        relativeResidualNorm: Number.isFinite(finalResult?.relativeResidualNorm)
            ? finalResult.relativeResidualNorm
            : restoreResult.relativeResidualNorm,
        bodyResidualNorm: Number.isFinite(finalResult?.bodyResidualNorm)
            ? finalResult.bodyResidualNorm
            : restoreResult.bodyResidualNorm,
        inequalityViolation: Number.isFinite(finalResult?.inequalityViolation)
            ? finalResult.inequalityViolation
            : restoreResult.inequalityViolation,
        referenceResidual: Array.isArray(finalResult?.referenceResidual)
            ? finalResult.referenceResidual.slice()
            : cloneArray(restoreResult.referenceResidual),
        constraintMotion: Array.isArray(finalResult?.constraintMotion)
            ? finalResult.constraintMotion.slice()
            : cloneArray(restoreResult.constraintMotion),
        qApplied: Array.isArray(finalResult?.qApplied)
            ? finalResult.qApplied.slice()
            : cloneArray(restoreResult.qApplied),
        secondaryMetric: overallMetric,
        secondaryState: state,
        secondaryBefore: overallBefore,
        secondaryAfter: evaluateSecondaryMetricValue(overallMetric, finalProblem?.metadata?.cctsm, secondaryMetricRequest?.metricMode),
        secondaryApplied: true
    };
}

self.onmessage = async (event) => {
    const data = event?.data ?? {};
    const id = data.id;
    try {
        if (!id) {
            throw new Error('mj_restore_worker_message_id_missing');
        }
        if (data.type === 'init') {
            await ensureModel(data.payload ?? {});
            self.postMessage({ id, ok: true, payload: { initialized: true } });
            return;
        }
        if (data.type === 'solve') {
            if (!currentModel || typeof currentModel.solveAbsoluteTargetMJRestore !== 'function') {
                throw new Error('mj_restore_worker_model_uninitialized');
            }
            const payload = data.payload ?? {};
            const solveStartMs = nowMs();
            const baseResult = currentModel.solveAbsoluteTargetMJRestore(payload.request ?? {});
            const secondaryResult = applySecondaryMetric(
                currentModel,
                currentDQModule,
                payload.request ?? {},
                payload.secondaryMetricRequest ?? null,
                baseResult
            );
            const result = secondaryResult && typeof secondaryResult === 'object'
                ? { ...baseResult, ...secondaryResult }
                : baseResult;
            const solveMs = nowMs() - solveStartMs;
            const response = result && typeof result === 'object'
                ? { ...result, solveMs }
                : { solveMs };
            self.postMessage({ id, ok: true, payload: response });
            return;
        }
        throw new Error('mj_restore_worker_unknown_message');
    } catch (error) {
        self.postMessage({
            id,
            ok: false,
            error: error instanceof Error ? error.message : String(error)
        });
    }
};
