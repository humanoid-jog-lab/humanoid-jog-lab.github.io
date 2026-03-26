const SUPPORTED_HUMANOID_JOG_MJ_PRE_W_CONTROL_MODES = new Set([
    'absolute',
    'decoupled',
    'decoupled-upright'
]);

const SUPPORTED_HUMANOID_JOG_MJ_PRE_W_METRICS = new Set([
    'w_abs_hier',
    'w_abs_hier_vfi'
]);

const SUPPORTED_HUMANOID_JOG_MJ_PRE_W_SKIPPED_STATES = new Set([
    'skipped(no_progress)',
    'skipped(no_gradient)',
    'skipped(projection_only_no_progress)'
]);

function normalizeHumanoidJogMJPreWState(state) {
    return typeof state === 'string' ? state.trim() : '';
}

function hasHumanoidJogMJPreWCandidatePayload(metricResult) {
    return (
        Array.isArray(metricResult?.qApplied) ||
        Array.isArray(metricResult?.qPre) ||
        Array.isArray(metricResult?.candidate) ||
        Array.isArray(metricResult?.candidateQ)
    );
}

function isHumanoidJogMJPreWOutcomeWithAcceptedCandidate(metricResult) {
    return metricResult?.acceptedCandidate === true || hasHumanoidJogMJPreWCandidatePayload(metricResult);
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

    if (SUPPORTED_HUMANOID_JOG_MJ_PRE_W_SKIPPED_STATES.has(state)) {
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
