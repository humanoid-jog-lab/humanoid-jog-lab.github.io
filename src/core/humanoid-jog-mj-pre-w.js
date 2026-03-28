const SUPPORTED_HUMANOID_JOG_MJ_PRE_W_CONTROL_MODES = new Set([
    'absolute',
    'decoupled',
    'decoupled-upright',
    'relative',
    'relative-fullbody',
    'distance',
    'distance-fullbody',
    'robot1',
    'robot2',
    'waist-body-link-xz'
]);

const SUPPORTED_HUMANOID_JOG_MJ_PRE_W_METRICS = new Set([
    'w_abs_hier',
    'w_abs_hier_vfi'
]);

function normalizeHumanoidJogMJPreWState(state) {
    return typeof state === 'string' ? state.trim() : '';
}

function isHumanoidJogMJPreWOutcomeWithAcceptedCandidate(metricResult) {
    return metricResult?.acceptedCandidate === true;
}

export function supportsHumanoidJogMJPreWControlMode(controlMode = '') {
    return SUPPORTED_HUMANOID_JOG_MJ_PRE_W_CONTROL_MODES.has(normalizeHumanoidJogMJPreWState(controlMode));
}

export function resolveHumanoidJogMJPreWMetric(metricRaw = '') {
    const normalizedMetric = normalizeHumanoidJogMJPreWState(metricRaw);
    return SUPPORTED_HUMANOID_JOG_MJ_PRE_W_METRICS.has(normalizedMetric) ? normalizedMetric : 'w_abs_hier';
}

export function getHumanoidJogMJPreWDirection() {
    return 'increase';
}

export function mapHumanoidJogMJPreWOutcome(metricResult = null) {
    if (metricResult === true) {
        return {
            state: 'skipped(none)',
            qPreEligible: false
        };
    }

    if (metricResult === false) {
        return {
            state: 'skipped(no_progress)',
            qPreEligible: false
        };
    }

    if (!metricResult || typeof metricResult !== 'object') {
        return {
            state: 'failed(invalid_metric_result)',
            qPreEligible: false
        };
    }

    if (metricResult.thrown === true || metricResult.error) {
        return {
            state: 'failed(thrown)',
            qPreEligible: false
        };
    }

    const state = normalizeHumanoidJogMJPreWState(metricResult.state);
    if (!state) {
        if (Array.isArray(metricResult.qApplied) || metricResult.converged === true) {
            return {
                state: isHumanoidJogMJPreWOutcomeWithAcceptedCandidate(metricResult)
                    ? 'applied'
                    : 'skipped(none)',
                qPreEligible: isHumanoidJogMJPreWOutcomeWithAcceptedCandidate(metricResult)
            };
        }
        return {
            state: 'failed(invalid_metric_result)',
            qPreEligible: false
        };
    }

    if (state === 'active') {
        return {
            state: 'applied',
            qPreEligible: true
        };
    }

    if (state.startsWith('active(primary_only;')) {
        if (isHumanoidJogMJPreWOutcomeWithAcceptedCandidate(metricResult)) {
            return {
                state: 'applied',
                qPreEligible: true
            };
        }

        return {
            state: 'skipped(projection_only_no_progress)',
            qPreEligible: false
        };
    }

    if (state.startsWith('skipped(')) {
        return {
            state,
            qPreEligible: false
        };
    }

    if (state.startsWith('rejected(')) {
        return {
            state,
            qPreEligible: false
        };
    }

    if (state.startsWith('failed(')) {
        return {
            state,
            qPreEligible: false
        };
    }

    return {
        state: 'failed(invalid_metric_result)',
        qPreEligible: false
    };
}
