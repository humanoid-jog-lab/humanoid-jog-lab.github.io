function finiteNumber(value, fallback = 0) {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : fallback;
}

function multiplyQuaternions(left, right) {
    const [lw, lx, ly, lz] = left;
    const [rw, rx, ry, rz] = right;
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw
    ];
}

export function multiplyDualQuaternionArrays(left, right) {
    if (!Array.isArray(left) || left.length !== 8 || !Array.isArray(right) || right.length !== 8) {
        return null;
    }
    const leftPrimal = left.slice(0, 4).map((value) => finiteNumber(value));
    const leftDual = left.slice(4, 8).map((value) => finiteNumber(value));
    const rightPrimal = right.slice(0, 4).map((value) => finiteNumber(value));
    const rightDual = right.slice(4, 8).map((value) => finiteNumber(value));
    const primal = multiplyQuaternions(leftPrimal, rightPrimal);
    const dualLeft = multiplyQuaternions(leftPrimal, rightDual);
    const dualRight = multiplyQuaternions(leftDual, rightPrimal);
    return primal.concat(dualLeft.map((value, index) => value + dualRight[index]));
}

export function buildHumanoidBasePoseDQArray(basePose = {}) {
    const x = finiteNumber(basePose.x);
    const y = finiteNumber(basePose.y);
    const yaw = finiteNumber(basePose.yaw);
    const halfYaw = 0.5 * yaw;
    const c = Math.cos(halfYaw);
    const s = Math.sin(halfYaw);
    return [
        c, 0, 0, s,
        0,
        0.5 * (x * c + y * s),
        0.5 * (-x * s + y * c),
        0
    ];
}

export function composeHumanoidWorldPoseDQArray(basePose = {}, localPoseArray = null) {
    if (!Array.isArray(localPoseArray) || localPoseArray.length !== 8) return null;
    return multiplyDualQuaternionArrays(
        buildHumanoidBasePoseDQArray(basePose),
        localPoseArray
    );
}
