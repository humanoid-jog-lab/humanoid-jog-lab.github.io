/**
 * Waist x-direction corridor VFI constraint.
 *
 * Implements the point-to-plane corridor from Section III-B of the paper:
 *   "To bound the fore-aft waist translation, the waist-end point p_0 is
 *    constrained to remain within [x_min, x_max] along the base x axis."
 *
 * Reference equations: (649)–(671), (861)–(866).
 *
 * VFI form for the upper bound (x <= x_max):
 *   d_x^+ = x_max - p0_x  (must stay >= 0)
 *   ḋ_x^+ >= -eta_x * d_x^+
 *   −(e_x^T J_p0 u) >= −eta_x * d_x^+
 *   => row: e_x^T J_p0_active,  rhs: eta_x * d_x^+  (in Ax <= b form)
 */

function asFinite(v, fb = 0) {
    return Number.isFinite(v) ? v : fb;
}

/**
 * Compute the x-component of the waist-end position from a DQ pose array.
 *
 * DQ.getTranslation returns [0, px, py, pz], so px is at index 1.
 *
 * @param {number[]} poseArray  8-element DQ array
 * @param {function} DQ         DQWrapper module
 * @returns {number | null}
 */
function waistEndPx(poseArray, DQ) {
    if (!Array.isArray(poseArray) || !DQ) return null;
    try {
        const dqObj = DQ.createFromArray(poseArray);
        const t = DQ.getTranslation(dqObj); // [0, px, py, pz]
        return asFinite(t[1], null);
    } catch (_) {
        return null;
    }
}

/**
 * Numerically compute the x-component row of the waist-end position Jacobian.
 *
 * For each active waist joint j:
 *   J_pos_x[j] ≈ (p0_x(q+eps*e_j) - p0_x(q-eps*e_j)) / (2*eps)
 *
 * @param {function} fkmFn        FK function: fkmFn(q) -> 8-array DQ pose of waist end
 * @param {number[]} q            full joint vector
 * @param {number}   waistStart   index of first waist joint in q
 * @param {number}   waistCount   number of waist joints
 * @param {function} DQ           DQWrapper module
 * @param {number}   [eps]        finite-difference step
 * @returns {number[] | null}     length-waistCount row vector, or null on error
 */
function computeWaistPxJacobianRow(fkmFn, q, waistStart, waistCount, DQ, eps = 1e-7) {
    const row = new Array(waistCount).fill(0);
    for (let j = 0; j < waistCount; j++) {
        const globalJ = waistStart + j;
        const qp = q.slice(); qp[globalJ] += eps;
        const qm = q.slice(); qm[globalJ] -= eps;
        const pp = fkmFn(qp);
        const pm = fkmFn(qm);
        if (!pp || !pm) return null;
        const pxp = waistEndPx(pp, DQ);
        const pxm = waistEndPx(pm, DQ);
        if (!Number.isFinite(pxp) || !Number.isFinite(pxm)) return null;
        row[j] = (pxp - pxm) / (2 * eps);
    }
    return row;
}

/**
 * Compute the VFI inequality rows for the waist x-direction corridor.
 *
 * Produces at most two rows (upper bound, lower bound) for the QP:
 *   A_x * u_active <= b_x
 *
 * For upper bound  p0_x <= x_max:
 *   row =  J_pos_x,  rhs = eta * (x_max - p0_x)
 *
 * For lower bound  p0_x >= x_min  (optional):
 *   row = -J_pos_x,  rhs = eta * (p0_x - x_min)
 *
 * Control rows are always included for every finite corridor bound, matching
 * the standard VFI form. Far from the boundary the RHS is large and the row is
 * naturally inactive; if the state is outside the corridor the signed distance
 * makes the RHS negative and drives the waist back inside. The returned
 * `vfiConstraints` still use `activationThreshold` as the CCTSM attenuation
 * scale, not as a control-row gate.
 *
 * @param {object} opts
 * @param {function} opts.fkmFn       FK for waist end effector: (q) -> 8-array
 * @param {number[]} opts.q           full joint vector
 * @param {number}   opts.waistStart  first waist joint index in q
 * @param {number}   opts.waistCount  number of waist joints
 * @param {function} opts.DQ          DQWrapper module
 * @param {number}   [opts.xMax]      upper x limit in metres (default 0.05 = 5cm)
 * @param {number}   [opts.xMin]      lower x limit (default -Infinity, inactive)
 * @param {number}   [opts.eta]       VFI decay rate (default 1.0)
 * @param {number}   [opts.activationThreshold]  CCTSM attenuation scale
 * @returns {{
 *   rows: number[][],
 *   rhs: number[],
 *   vfiConstraints: Array<{row: number[], distance: number, threshold: number}>,
 *   p0x: number,
 *   d_upper: number,
 *   d_lower: number
 * } | null}
 */
export function computeWaistXVFIRows({
    fkmFn,
    q,
    waistStart,
    waistCount,
    DQ,
    xMax = 0.05,
    xMin = -Infinity,
    eta = 1.0,
    activationThreshold = 0.12
} = {}) {
    if (typeof fkmFn !== 'function' || !Array.isArray(q) || waistCount <= 0 || !DQ) return null;

    const poseArray = fkmFn(q);
    if (!poseArray) return null;

    const p0x = waistEndPx(poseArray, DQ);
    if (!Number.isFinite(p0x)) return null;

    const JpxRow = computeWaistPxJacobianRow(fkmFn, q, waistStart, waistCount, DQ);
    if (!JpxRow) return null;

    const rows = [];
    const rhs = [];
    const vfiConstraints = [];

    const d_upper = xMax - p0x;
    const d_lower = Number.isFinite(xMin) ? p0x - xMin : Infinity;

    // Upper bound: p0_x <= x_max  →  J_px * u <= eta * d_upper
    if (Number.isFinite(xMax)) {
        const safeD = Math.max(0, d_upper);
        vfiConstraints.push({ row: JpxRow.map((v) => -v), distance: safeD, threshold: activationThreshold });
        rows.push([...JpxRow]);
        rhs.push(eta * d_upper);
    }

    // Lower bound: p0_x >= x_min  →  -J_px * u <= eta * d_lower
    if (Number.isFinite(xMin)) {
        const safeD = Math.max(0, d_lower);
        vfiConstraints.push({ row: [...JpxRow], distance: safeD, threshold: activationThreshold });
        rows.push(JpxRow.map((v) => -v));
        rhs.push(eta * d_lower);
    }

    return { rows, rhs, vfiConstraints, p0x, d_upper, d_lower };
}
