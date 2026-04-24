/**
 * 事件处理模块
 * 
 * 包含主事件监听器设置、拖拽加载、预设位姿、标签切换、
 * DQ 演示、MDH 参数提取/对比/应用等功能。
 */

import { DEG_TO_RAD } from '../core/constants.js';
import yaml from 'js-yaml';
import { appState, callbacks } from '../core/state.js';
import { DQ, SerialManipulator, FrankaPanda, getDQModule } from '../dqrobotics.js';
import {
    DEFAULT_HUMANOID_INITIAL_Q_PROFILE_ID,
    normalizeHumanoidInitialQProfileId,
    resolveHumanoidInitialQ
} from '../core/humanoid-default-q-profiles.js';
import { extractMDHFromURDF, createMDHTestSuite, formatMDHForDQ } from '../urdf-to-mdh.js';
import { URDFPackageManager, loadURDFFromPackage } from '../urdf-loader.js';
import { getActiveJointConfig, displayStep, angleToDisplay } from '../utils/math.js';
import { setupGlobalDragAndDrop } from './drag-drop.js';
import { setupScenePresetListeners } from './environment-panel.js';
import { normalizeHumanoidJogSolverMode } from '../core/humanoid-jog-solver-presentation.js';
import { getHumanoidTaskMetricOptionLabels } from '../core/humanoid-jog-task-metric-labels.js';
import { onLanguageChange, t } from '../i18n.js';

const TEACH_MULTIPLICATION_ORDER_STORAGE_KEY = 'teach-multiply-order-delta-first';
const HUMANOID_W_METRIC_HOLD_DELAY_MS = 250;
const HUMANOID_W_METRIC_REPEAT_INTERVAL_MS = 100;

export function setupHumanoidDefaultPoseControls() {
    const profile1Btn = document.getElementById('humanoid-default-q-profile-1-btn');
    const profile2Btn = document.getElementById('humanoid-default-q-profile-2-btn');
    const HUMANOID_DEFAULT_Q_PROFILE_STORAGE_KEY = 'humanoid-default-q-profile';
    const buttonByProfile = {
        default1: profile1Btn,
        default2: profile2Btn
    };

    const updateButtonState = (activeProfileId) => {
        Object.entries(buttonByProfile).forEach(([profileId, button]) => {
            if (!button) return;
            const active = profileId === activeProfileId;
            button.dataset.active = active ? 'true' : 'false';
            if (button.classList?.toggle) button.classList.toggle('is-active', active);
            if (button.setAttribute) button.setAttribute('aria-pressed', active ? 'true' : 'false');
        });
    };

    const applyHumanoidDefaultQProfile = (profileIdRaw, options = {}) => {
        const { applyToCurrentHumanoid = true } = options;
        const profileId = normalizeHumanoidInitialQProfileId(profileIdRaw);
        appState.humanoidInitialQProfileId = profileId;
        if (typeof window !== 'undefined') {
            window.humanoidInitialQProfileId = profileId;
        }
        try {
            localStorage.setItem(HUMANOID_DEFAULT_Q_PROFILE_STORAGE_KEY, profileId);
        } catch (_) {}
        updateButtonState(profileId);

        if (applyToCurrentHumanoid && appState.humanoidMode && appState.humanoidJointNames.length > 0) {
            appState.humanoidQ = resolveHumanoidInitialQ(
                { initial_q_profile: profileId },
                appState.humanoidJointNames.length,
                profileId
            );
            callbacks.updateGlobalVariables?.();
            callbacks.updateRobotPose?.();
            appState.visualizer?.resetHumanoidSurfaceSlideReference?.();
        }

        return profileId;
    };

    const savedProfileId = (() => {
        try { return localStorage.getItem(HUMANOID_DEFAULT_Q_PROFILE_STORAGE_KEY); } catch (_) { return null; }
    })();
    applyHumanoidDefaultQProfile(
        savedProfileId ?? appState.humanoidInitialQProfileId ?? DEFAULT_HUMANOID_INITIAL_Q_PROFILE_ID,
        { applyToCurrentHumanoid: false }
    );

    profile1Btn?.addEventListener('click', () => {
        applyHumanoidDefaultQProfile('default1');
    });
    profile2Btn?.addEventListener('click', () => {
        applyHumanoidDefaultQProfile('default2');
    });

    if (typeof window !== 'undefined') {
        window.setHumanoidDefaultQProfile = applyHumanoidDefaultQProfile;
    }

    return {
        applyHumanoidDefaultQProfile
    };
}

export function setupTeachOrderControls() {
    const teachOrderToggle = document.getElementById('teach-multiply-order-toggle');
    const teachOrderStatus = document.getElementById('teach-multiply-order-status');
    const updateTeachOrderStatus = (isDeltaFirst) => {
        if (!teachOrderStatus) return;
        teachOrderStatus.textContent = isDeltaFirst
            ? t('Current Mode: local frame (current × Δ)')
            : t('Current Mode: world frame (Δ × current)');
    };
    const applyTeachOrder = (isDeltaFirst) => {
        if (appState.visualizer?.setTeachMultiplicationOrder) {
            const order = isDeltaFirst ? 'delta-first' : 'current-first';
            appState.visualizer.setTeachMultiplicationOrder(order);
        }
    };
    const persistTeachOrder = (isDeltaFirst) => {
        try {
            localStorage.setItem(TEACH_MULTIPLICATION_ORDER_STORAGE_KEY, isDeltaFirst ? '1' : '0');
        } catch (_) {}
    };
    const applyTeachOrderPreference = (isDeltaFirst) => {
        if (teachOrderToggle) {
            teachOrderToggle.checked = isDeltaFirst;
        }
        updateTeachOrderStatus(isDeltaFirst);
        applyTeachOrder(isDeltaFirst);
        persistTeachOrder(isDeltaFirst);
    };
    if (teachOrderToggle) {
        const saved = (() => {
            try { return localStorage.getItem(TEACH_MULTIPLICATION_ORDER_STORAGE_KEY); } catch (_) { return null; }
        })();
        const initialIsDeltaFirst = saved != null ? saved === '1' : Boolean(teachOrderToggle.checked);
        applyTeachOrderPreference(initialIsDeltaFirst);
        onLanguageChange(() => updateTeachOrderStatus(teachOrderToggle.checked));
        teachOrderToggle.addEventListener('change', (e) => {
            applyTeachOrderPreference(Boolean(e.target?.checked));
        });
    } else {
        updateTeachOrderStatus(false);
    }

    return {
        applyTeachOrderPreference
    };
}

export function setupHumanoidCooperativePolishingSurfaceControls() {
    const analyticBtn = document.getElementById('humanoid-cooperative-polishing-surface-source-analytic-btn');
    const cylinderBtn = document.getElementById('humanoid-cooperative-polishing-surface-source-cylinder-btn');
    const STORAGE_KEY = 'humanoid-cooperative-polishing-surface-source';
    const buttonBySource = {
        analytic: analyticBtn,
        'cylindrical-patch': cylinderBtn
    };

    const updateButtonState = (activeSource) => {
        Object.entries(buttonBySource).forEach(([source, button]) => {
            if (!button) return;
            const active = source === activeSource;
            button.dataset.active = active ? 'true' : 'false';
            if (button.classList?.toggle) button.classList.toggle('is-active', active);
            if (button.setAttribute) button.setAttribute('aria-pressed', active ? 'true' : 'false');
        });
    };

    const applyHumanoidCooperativePolishingSurfaceSource = async (sourceRaw) => {
        const source = sourceRaw === 'cylindrical-patch' ? 'cylindrical-patch' : 'analytic';
        appState.humanoidCooperativePolishingSurfaceSource = source;
        if (typeof window !== 'undefined') {
            window.humanoidCooperativeRuledSurfaceSource = source;
        }
        try {
            localStorage.setItem(STORAGE_KEY, source);
        } catch (_) {}
        updateButtonState(source);
        if (appState.visualizer?.setHumanoidCooperativeRuledSurfaceSurfaceSource) {
            await appState.visualizer.setHumanoidCooperativeRuledSurfaceSurfaceSource(source);
        }
        return source;
    };

    const savedSource = (() => {
        try { return localStorage.getItem(STORAGE_KEY); } catch (_) { return null; }
    })();
    void applyHumanoidCooperativePolishingSurfaceSource(
        savedSource ?? appState.humanoidCooperativePolishingSurfaceSource ?? 'analytic'
    );

    analyticBtn?.addEventListener('click', () => {
        void applyHumanoidCooperativePolishingSurfaceSource('analytic');
    });
    cylinderBtn?.addEventListener('click', () => {
        void applyHumanoidCooperativePolishingSurfaceSource('cylindrical-patch');
    });

    if (typeof window !== 'undefined') {
        window.setHumanoidCooperativePolishingSurfaceSource = applyHumanoidCooperativePolishingSurfaceSource;
    }

    return {
        applyHumanoidCooperativePolishingSurfaceSource
    };
}

export function setupHumanoidCooperativePolishingObjectPoseControls() {
    const xInput = document.getElementById('humanoid-cooperative-polishing-object-x-input');
    const yInput = document.getElementById('humanoid-cooperative-polishing-object-y-input');
    const zInput = document.getElementById('humanoid-cooperative-polishing-object-z-input');
    const yawInput = document.getElementById('humanoid-cooperative-polishing-object-yaw-input');
    const flipNormalToggle = document.getElementById('humanoid-cooperative-polishing-object-flip-normal-toggle');
    const applyBtn = document.getElementById('humanoid-cooperative-polishing-object-apply-btn');
    const captureBtn = document.getElementById('humanoid-cooperative-polishing-object-capture-btn');
    const exportBtn = document.getElementById('humanoid-cooperative-polishing-export-obj-btn');
    const STORAGE_KEY = 'humanoid-cooperative-polishing-object-pose';
    let currentPose = null;

    const asFiniteNumber = (value, fallback = 0) => {
        const parsed = Number(value);
        return Number.isFinite(parsed) ? parsed : fallback;
    };
    const buildYawQuaternion = (yawDegrees) => {
        const yawRadians = asFiniteNumber(yawDegrees, 0) * DEG_TO_RAD;
        const halfYaw = 0.5 * yawRadians;
        return [Math.cos(halfYaw), 0, 0, Math.sin(halfYaw)];
    };
    const extractYawDegrees = (quaternionLike) => {
        const w = asFiniteNumber(quaternionLike?.[0], 1);
        const x = asFiniteNumber(quaternionLike?.[1], 0);
        const y = asFiniteNumber(quaternionLike?.[2], 0);
        const z = asFiniteNumber(quaternionLike?.[3], 0);
        const sinyCosp = 2 * ((w * z) + (x * y));
        const cosyCosp = 1 - (2 * ((y * y) + (z * z)));
        return Math.atan2(sinyCosp, cosyCosp) / DEG_TO_RAD;
    };
    const multiplyQuaternions = (a, b) => {
        const [aw, ax, ay, az] = a;
        const [bw, bx, by, bz] = b;
        return [
            (aw * bw) - (ax * bx) - (ay * by) - (az * bz),
            (aw * bx) + (ax * bw) + (ay * bz) - (az * by),
            (aw * by) - (ax * bz) + (ay * bw) + (az * bx),
            (aw * bz) + (ax * by) - (ay * bx) + (az * bw)
        ];
    };
    const NORMAL_FLIP_QUATERNION = [0, 0, 1, 0];
    const applyNormalFlipQuaternion = (quaternionLike) => {
        const q = [
            asFiniteNumber(quaternionLike?.[0], 1),
            asFiniteNumber(quaternionLike?.[1], 0),
            asFiniteNumber(quaternionLike?.[2], 0),
            asFiniteNumber(quaternionLike?.[3], 0)
        ];
        return multiplyQuaternions(q, NORMAL_FLIP_QUATERNION);
    };
    const normalizePose = (poseLike) => {
        if (!poseLike?.position || !poseLike?.quaternion) return null;
        return {
            position: [
                asFiniteNumber(poseLike.position[0], 0),
                asFiniteNumber(poseLike.position[1], 0),
                asFiniteNumber(poseLike.position[2], 0)
            ],
            quaternion: [
                asFiniteNumber(poseLike.quaternion[0], 1),
                asFiniteNumber(poseLike.quaternion[1], 0),
                asFiniteNumber(poseLike.quaternion[2], 0),
                asFiniteNumber(poseLike.quaternion[3], 0)
            ],
            flipNormal: Boolean(poseLike.flipNormal),
            normalFlipApplied: Boolean(poseLike.normalFlipApplied)
        };
    };
    const syncInputsFromPose = (poseLike) => {
        const pose = normalizePose(poseLike);
        if (!pose) return null;
        currentPose = {
            position: pose.position.slice(),
            quaternion: pose.quaternion.slice(),
            flipNormal: Boolean(pose.flipNormal),
            normalFlipApplied: Boolean(pose.normalFlipApplied)
        };
        if (xInput) xInput.value = String(pose.position[0]);
        if (yInput) yInput.value = String(pose.position[1]);
        if (zInput) zInput.value = String(pose.position[2]);
        if (yawInput) yawInput.value = String(extractYawDegrees(pose.quaternion));
        if (flipNormalToggle) flipNormalToggle.checked = Boolean(pose.flipNormal);
        return pose;
    };
    const persistPose = (poseLike) => {
        const pose = normalizePose(poseLike);
        if (!pose) return null;
        appState.humanoidCooperativePolishingObjectPose = {
            position: pose.position.slice(),
            quaternion: pose.quaternion.slice(),
            flipNormal: Boolean(pose.flipNormal),
            normalFlipApplied: Boolean(pose.normalFlipApplied)
        };
        if (typeof window !== 'undefined') {
            window.humanoidCooperativeRuledSurfaceObjectPose = {
                position: pose.position.slice(),
                quaternion: pose.quaternion.slice(),
                flipNormal: Boolean(pose.flipNormal),
                normalFlipApplied: Boolean(pose.normalFlipApplied)
            };
        }
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify(pose));
        } catch (_) {}
        return pose;
    };
    const applyPose = (poseLike) => {
        const pose = persistPose(poseLike);
        if (!pose) return null;
        const appliedPose = appState.visualizer?.setHumanoidCooperativeRuledSurfaceObjectPose
            ? (appState.visualizer.setHumanoidCooperativeRuledSurfaceObjectPose(pose) || pose)
            : pose;
        syncInputsFromPose(appliedPose);
        return appliedPose;
    };

    const savedPose = (() => {
        try {
            const raw = localStorage.getItem(STORAGE_KEY);
            return raw ? normalizePose(JSON.parse(raw)) : null;
        } catch (_) {
            return null;
        }
    })() || normalizePose(
        (typeof window !== 'undefined' ? window.humanoidCooperativeRuledSurfaceObjectPose : null)
        || appState.humanoidCooperativePolishingObjectPose
    );
    if (savedPose) {
        persistPose(savedPose);
        syncInputsFromPose(savedPose);
    }

    applyBtn?.addEventListener('click', () => {
        const flipNormal = Boolean(flipNormalToggle?.checked);
        const baseQuaternion = buildYawQuaternion(yawInput?.value);
        applyPose({
            position: [
                asFiniteNumber(xInput?.value, 0),
                asFiniteNumber(yInput?.value, 0),
                asFiniteNumber(zInput?.value, 0)
            ],
            quaternion: flipNormal ? applyNormalFlipQuaternion(baseQuaternion) : baseQuaternion,
            flipNormal,
            normalFlipApplied: flipNormal
        });
    });

    captureBtn?.addEventListener('click', async () => {
        const flipNormal = Boolean(flipNormalToggle?.checked);
        const capturedPose = appState.visualizer?.captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands
            ? await Promise.resolve(appState.visualizer.captureHumanoidCooperativeRuledSurfaceObjectPoseFromCurrentHands({
                flipNormal
            }))
            : null;
        if (!capturedPose) return;
        applyPose(capturedPose);
    });

    exportBtn?.addEventListener('click', async () => {
        await Promise.resolve(
            appState.visualizer?.exportHumanoidCooperativeRuledSurfacePrintableObj?.({
                thicknessMetres: 0.008,
                segmentsU: 96,
                segmentsV: 32
            })
        );
    });

    flipNormalToggle?.addEventListener('change', (event) => {
        const nextFlipNormal = Boolean(event?.target?.checked);
        const nextQuaternion = Array.isArray(currentPose?.quaternion)
            ? applyNormalFlipQuaternion(currentPose.quaternion)
            : applyNormalFlipQuaternion(buildYawQuaternion(yawInput?.value));
        applyPose({
            position: [
                asFiniteNumber(xInput?.value, 0),
                asFiniteNumber(yInput?.value, 0),
                asFiniteNumber(zInput?.value, 0)
            ],
            quaternion: nextQuaternion,
            flipNormal: nextFlipNormal,
            normalFlipApplied: nextFlipNormal
        });
    });

    return {
        applyHumanoidCooperativePolishingObjectPose: applyPose
    };
}

export function setupHumanoidParameterControls() {
    const humanoidJointLimitEtaInput = document.getElementById('humanoid-joint-limit-eta-input');
    const humanoidJointLimitEtaStatus = document.getElementById('humanoid-joint-limit-eta-status');
    const humanoidRos2ArmVelInput = document.getElementById('humanoid-ros2-arm-vel-input');
    const humanoidRos2ArmAccInput = document.getElementById('humanoid-ros2-arm-acc-input');
    const humanoidRos2WaistVelInput = document.getElementById('humanoid-ros2-waist-vel-input');
    const humanoidRos2WaistAccInput = document.getElementById('humanoid-ros2-waist-acc-input');
    const humanoidRos2PublishDefaultsStatus = document.getElementById('humanoid-ros2-publish-defaults-status');
    const HUMANOID_JOINT_LIMIT_ETA_STORAGE_KEY = 'humanoid-joint-limit-eta';
    const HUMANOID_ROS2_ARM_VEL_STORAGE_KEY = 'humanoid-ros2-arm-vel';
    const HUMANOID_ROS2_ARM_ACC_STORAGE_KEY = 'humanoid-ros2-arm-acc';
    const HUMANOID_ROS2_WAIST_VEL_STORAGE_KEY = 'humanoid-ros2-waist-vel';
    const HUMANOID_ROS2_WAIST_ACC_STORAGE_KEY = 'humanoid-ros2-waist-acc';

    if (!humanoidJointLimitEtaInput) {
        return {
            applyHumanoidJointLimitEta: () => false
        };
    }

    const normalizeHumanoidJointLimitEta = (valueRaw) => {
        const parsed = Number(valueRaw);
        return Number.isFinite(parsed) && parsed > 0 ? parsed : null;
    };

    const updateHumanoidJointLimitEtaStatus = (message) => {
        if (!humanoidJointLimitEtaStatus) return;
        humanoidJointLimitEtaStatus.textContent = message;
    };
    const updateHumanoidRos2PublishDefaultsStatus = (message) => {
        if (!humanoidRos2PublishDefaultsStatus) return;
        humanoidRos2PublishDefaultsStatus.textContent = message;
    };

    const readCurrentHumanoidJointLimitEta = () => {
        const visualizerEta = appState.visualizer?.vfiConfig?.jointLimitEta;
        if (Number.isFinite(visualizerEta) && visualizerEta > 0) return visualizerEta;
        const windowEta = typeof window !== 'undefined' ? window.humanoidJointLimitEta : null;
        if (Number.isFinite(windowEta) && windowEta > 0) return windowEta;
        return 5;
    };

    let appliedHumanoidJointLimitEta = readCurrentHumanoidJointLimitEta();
    let appliedHumanoidRos2PublishDefaults = {
        armVel: Number.isFinite(appState.humanoidRos2PublishDefaults?.armVel) ? appState.humanoidRos2PublishDefaults.armVel : 0.5,
        armAcc: Number.isFinite(appState.humanoidRos2PublishDefaults?.armAcc) ? appState.humanoidRos2PublishDefaults.armAcc : 1.0,
        waistVel: Number.isFinite(appState.humanoidRos2PublishDefaults?.waistVel) ? appState.humanoidRos2PublishDefaults.waistVel : 0.5,
        waistAcc: Number.isFinite(appState.humanoidRos2PublishDefaults?.waistAcc) ? appState.humanoidRos2PublishDefaults.waistAcc : 1.0
    };

    const applyHumanoidJointLimitEta = (valueRaw) => {
        const value = normalizeHumanoidJointLimitEta(valueRaw);
        if (value == null) {
            humanoidJointLimitEtaInput.value = String(appliedHumanoidJointLimitEta);
            updateHumanoidJointLimitEtaStatus(`输入无效，已恢复为 η_θ=${appliedHumanoidJointLimitEta}`);
            return false;
        }

        appliedHumanoidJointLimitEta = value;
        humanoidJointLimitEtaInput.value = String(value);
        if (typeof window !== 'undefined') {
            window.humanoidJointLimitEta = value;
        }
        if (appState.visualizer?.setHumanoidJointLimitEta) {
            appState.visualizer.setHumanoidJointLimitEta(value);
        } else if (appState.visualizer?.vfiConfig) {
            appState.visualizer.vfiConfig.jointLimitEta = value;
        }
        try {
            localStorage.setItem(HUMANOID_JOINT_LIMIT_ETA_STORAGE_KEY, String(value));
        } catch (_) {}
        updateHumanoidJointLimitEtaStatus(`当前值已应用：η_θ=${value}`);
        return true;
    };
    const normalizeRos2DefaultNumber = (valueRaw) => {
        const parsed = Number(valueRaw);
        return Number.isFinite(parsed) && parsed >= 0 ? parsed : null;
    };
    const applyHumanoidRos2PublishDefaults = (defaultsRaw = {}) => {
        const next = {
            armVel: normalizeRos2DefaultNumber(defaultsRaw.armVel ?? appliedHumanoidRos2PublishDefaults.armVel),
            armAcc: normalizeRos2DefaultNumber(defaultsRaw.armAcc ?? appliedHumanoidRos2PublishDefaults.armAcc),
            waistVel: normalizeRos2DefaultNumber(defaultsRaw.waistVel ?? appliedHumanoidRos2PublishDefaults.waistVel),
            waistAcc: normalizeRos2DefaultNumber(defaultsRaw.waistAcc ?? appliedHumanoidRos2PublishDefaults.waistAcc)
        };
        if (Object.values(next).some((value) => value == null)) {
            if (humanoidRos2ArmVelInput) humanoidRos2ArmVelInput.value = String(appliedHumanoidRos2PublishDefaults.armVel);
            if (humanoidRos2ArmAccInput) humanoidRos2ArmAccInput.value = String(appliedHumanoidRos2PublishDefaults.armAcc);
            if (humanoidRos2WaistVelInput) humanoidRos2WaistVelInput.value = String(appliedHumanoidRos2PublishDefaults.waistVel);
            if (humanoidRos2WaistAccInput) humanoidRos2WaistAccInput.value = String(appliedHumanoidRos2PublishDefaults.waistAcc);
            updateHumanoidRos2PublishDefaultsStatus(
                `输入无效，已恢复为 双臂(${appliedHumanoidRos2PublishDefaults.armVel}/${appliedHumanoidRos2PublishDefaults.armAcc}) 腰部(${appliedHumanoidRos2PublishDefaults.waistVel}/${appliedHumanoidRos2PublishDefaults.waistAcc})`
            );
            return false;
        }

        appliedHumanoidRos2PublishDefaults = next;
        if (humanoidRos2ArmVelInput) humanoidRos2ArmVelInput.value = String(next.armVel);
        if (humanoidRos2ArmAccInput) humanoidRos2ArmAccInput.value = String(next.armAcc);
        if (humanoidRos2WaistVelInput) humanoidRos2WaistVelInput.value = String(next.waistVel);
        if (humanoidRos2WaistAccInput) humanoidRos2WaistAccInput.value = String(next.waistAcc);
        appState.humanoidRos2PublishDefaults = {
            armVel: next.armVel,
            armAcc: next.armAcc,
            waistVel: next.waistVel,
            waistAcc: next.waistAcc
        };
        if (typeof window !== 'undefined') {
            window.humanoidRos2PublishDefaults = { ...appState.humanoidRos2PublishDefaults };
        }
        try {
            localStorage.setItem(HUMANOID_ROS2_ARM_VEL_STORAGE_KEY, String(next.armVel));
            localStorage.setItem(HUMANOID_ROS2_ARM_ACC_STORAGE_KEY, String(next.armAcc));
            localStorage.setItem(HUMANOID_ROS2_WAIST_VEL_STORAGE_KEY, String(next.waistVel));
            localStorage.setItem(HUMANOID_ROS2_WAIST_ACC_STORAGE_KEY, String(next.waistAcc));
        } catch (_) {}
        updateHumanoidRos2PublishDefaultsStatus(
            `当前值已应用：双臂 vel/acc=${next.armVel}/${next.armAcc}，腰部 vel/acc=${next.waistVel}/${next.waistAcc}`
        );
        return true;
    };

    const savedHumanoidJointLimitEta = (() => {
        try { return localStorage.getItem(HUMANOID_JOINT_LIMIT_ETA_STORAGE_KEY); } catch (_) { return null; }
    })();
    applyHumanoidJointLimitEta(savedHumanoidJointLimitEta ?? appliedHumanoidJointLimitEta);
    applyHumanoidRos2PublishDefaults({
        armVel: (() => { try { return localStorage.getItem(HUMANOID_ROS2_ARM_VEL_STORAGE_KEY); } catch (_) { return null; } })() ?? appliedHumanoidRos2PublishDefaults.armVel,
        armAcc: (() => { try { return localStorage.getItem(HUMANOID_ROS2_ARM_ACC_STORAGE_KEY); } catch (_) { return null; } })() ?? appliedHumanoidRos2PublishDefaults.armAcc,
        waistVel: (() => { try { return localStorage.getItem(HUMANOID_ROS2_WAIST_VEL_STORAGE_KEY); } catch (_) { return null; } })() ?? appliedHumanoidRos2PublishDefaults.waistVel,
        waistAcc: (() => { try { return localStorage.getItem(HUMANOID_ROS2_WAIST_ACC_STORAGE_KEY); } catch (_) { return null; } })() ?? appliedHumanoidRos2PublishDefaults.waistAcc
    });

    humanoidJointLimitEtaInput.addEventListener('change', (e) => {
        applyHumanoidJointLimitEta(e.target?.value);
    });
    humanoidJointLimitEtaInput.addEventListener('blur', (e) => {
        applyHumanoidJointLimitEta(e.target?.value);
    });
    humanoidJointLimitEtaInput.addEventListener('keydown', (e) => {
        if (e.key !== 'Enter') return;
        e.preventDefault?.();
        applyHumanoidJointLimitEta(e.target?.value);
    });
    const bindRos2DefaultInput = (input, key) => {
        if (!input) return;
        const applyFromInputs = () => applyHumanoidRos2PublishDefaults({
            armVel: key === 'armVel' ? input.value : humanoidRos2ArmVelInput?.value,
            armAcc: key === 'armAcc' ? input.value : humanoidRos2ArmAccInput?.value,
            waistVel: key === 'waistVel' ? input.value : humanoidRos2WaistVelInput?.value,
            waistAcc: key === 'waistAcc' ? input.value : humanoidRos2WaistAccInput?.value
        });
        input.addEventListener('change', applyFromInputs);
        input.addEventListener('blur', applyFromInputs);
        input.addEventListener('keydown', (e) => {
            if (e.key !== 'Enter') return;
            e.preventDefault?.();
            applyFromInputs();
        });
    };
    bindRos2DefaultInput(humanoidRos2ArmVelInput, 'armVel');
    bindRos2DefaultInput(humanoidRos2ArmAccInput, 'armAcc');
    bindRos2DefaultInput(humanoidRos2WaistVelInput, 'waistVel');
    bindRos2DefaultInput(humanoidRos2WaistAccInput, 'waistAcc');

    if (typeof window !== 'undefined') {
        window.setHumanoidJointLimitEta = applyHumanoidJointLimitEta;
        window.setHumanoidRos2PublishDefaults = applyHumanoidRos2PublishDefaults;
    }

    return {
        applyHumanoidJointLimitEta,
        applyHumanoidRos2PublishDefaults
    };
}

function setupHumanoidJogSolverControls() {
    // 人形点动求解器切换（new qpJ / 零空间投影 / MuJoCo 风格恢复）
    const humanoidJogSolverSelect = document.getElementById('humanoid-jog-solver-select');
    const humanoidJogMetricSelect = document.getElementById('humanoid-manifold-jog-metric-select');
    const humanoidWMetricIncreaseBtn = document.getElementById('humanoid-w-metric-increase-btn');
    const humanoidWaistXCorridorToggle = document.getElementById('humanoid-waist-x-corridor-toggle');
    const humanoidWaistXCorridorMinInput = document.getElementById('humanoid-waist-x-corridor-min-input');
    const humanoidWaistXCorridorMaxInput = document.getElementById('humanoid-waist-x-corridor-max-input');
    const humanoidTestDualArmLockToggle = document.getElementById('humanoid-test-dual-arm-lock-toggle');
    const dualArmControlModeSelect = document.getElementById('dual-arm-control-mode-select');
    const HUMANOID_JOG_SOLVER_STORAGE_KEY = 'humanoid-jog-solver-mode';
    const HUMANOID_JOG_METRIC_STORAGE_KEY = 'humanoid-manifold-jog-metric';
    const HUMANOID_WAIST_X_CORRIDOR_STORAGE_KEY = 'humanoid-waist-x-corridor-enabled';
    const HUMANOID_WAIST_X_CORRIDOR_MIN_STORAGE_KEY = 'humanoid-waist-x-corridor-min';
    const HUMANOID_WAIST_X_CORRIDOR_MAX_STORAGE_KEY = 'humanoid-waist-x-corridor-max';
    const HUMANOID_TEST_DUAL_ARM_LOCK_STORAGE_KEY = 'humanoid-test-dual-arm-lock-enabled';
    const isHumanoidRuledSurfacePolishingMode = (mode) => (
        mode === 'left-ruled-surface-polishing'
        || mode === 'cooperative-ruled-surface-polishing'
        || mode === 'cooperative-ruled-surface-polishing-dq-setpoint'
        || mode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'
    );
    const isHumanoidMJRestoreLikeSolverMode = (mode) => (
        mode === 'mj_restore'
        || mode === 'mj_restore_pre_w'
        || mode === 'mj_restore_tr'
    );
    const normalizeHumanoidJogSolverModeForUI = (modeRaw) => {
        const mode = normalizeHumanoidJogSolverMode(modeRaw);
        return mode === 'mj_restore_pre_w' ? 'mj_restore_tr' : mode;
    };
    const normalizeHumanoidJogMetric = (metricRaw) => {
        return metricRaw === 'w_abs_hier_vfi' ? 'w_abs_hier_vfi' : 'w_abs_hier';
    };
    const applyHumanoidJogSolverMode = (modeRaw) => {
        const mode = normalizeHumanoidJogSolverModeForUI(modeRaw);
        if (humanoidJogSolverSelect) humanoidJogSolverSelect.value = mode;
        if (typeof window !== 'undefined') {
            window.humanoidJogSolverMode = mode;
        }
        if (appState.visualizer?.setHumanoidJogSolverMode) {
            appState.visualizer.setHumanoidJogSolverMode(mode);
        }
        try {
            localStorage.setItem(HUMANOID_JOG_SOLVER_STORAGE_KEY, mode);
        } catch (_) {}
    };
    const ensureHumanoidJogSolverSupportsControlMode = (controlModeRaw) => {
        if (!isHumanoidRuledSurfacePolishingMode(controlModeRaw)) return;
        const currentSolverMode = normalizeHumanoidJogSolverModeForUI(
            humanoidJogSolverSelect?.value
            ?? (typeof window !== 'undefined' ? window.humanoidJogSolverMode : null)
            ?? 'vfi'
        );
        if (!isHumanoidMJRestoreLikeSolverMode(currentSolverMode)) {
            applyHumanoidJogSolverMode('mj_restore');
        }
    };
    const applyHumanoidJogMetric = (metricRaw) => {
        const metric = normalizeHumanoidJogMetric(metricRaw);
        if (humanoidJogMetricSelect) humanoidJogMetricSelect.value = metric;
        if (typeof window !== 'undefined') {
            window.humanoidManifoldJogSecondaryMetric = metric;
        }
        if (appState.visualizer?.setHumanoidManifoldJogSecondaryMetric) {
            appState.visualizer.setHumanoidManifoldJogSecondaryMetric(metric);
        }
        try {
            localStorage.setItem(HUMANOID_JOG_METRIC_STORAGE_KEY, metric);
        } catch (_) {}
    };
    const updateHumanoidJogMetricOptionLabels = (controlModeRaw = null) => {
        if (!humanoidJogMetricSelect?.options) return;
        const controlMode = controlModeRaw
            ?? dualArmControlModeSelect?.value
            ?? (typeof window !== 'undefined' ? window.currentDualArmControlMode : null)
            ?? 'absolute';
        const labels = getHumanoidTaskMetricOptionLabels(controlMode);
        Array.from(humanoidJogMetricSelect.options).forEach((option) => {
            if (Object.prototype.hasOwnProperty.call(labels, option.value)) {
                option.textContent = labels[option.value];
            }
        });
    };
    const applyHumanoidTestDualArmLock = (enabledRaw) => {
        const enabled = Boolean(enabledRaw);
        if (humanoidTestDualArmLockToggle) humanoidTestDualArmLockToggle.checked = enabled;
        if (typeof window !== 'undefined') {
            window.humanoidTestDualArmLockEnabled = enabled;
        }
        if (appState.visualizer?.setHumanoidTestDualArmLockEnabled) {
            appState.visualizer.setHumanoidTestDualArmLockEnabled(enabled);
        }
        try {
            localStorage.setItem(HUMANOID_TEST_DUAL_ARM_LOCK_STORAGE_KEY, enabled ? '1' : '0');
        } catch (_) {}
    };
    const applyHumanoidWaistXCorridor = (enabledRaw) => {
        const enabled = Boolean(enabledRaw);
        if (humanoidWaistXCorridorToggle) humanoidWaistXCorridorToggle.checked = enabled;
        if (typeof window !== 'undefined') {
            window.humanoidWaistXCorridorEnabled = enabled;
        }
        if (appState.visualizer?.setHumanoidWaistXCorridorEnabled) {
            appState.visualizer.setHumanoidWaistXCorridorEnabled(enabled);
        }
        try {
            localStorage.setItem(HUMANOID_WAIST_X_CORRIDOR_STORAGE_KEY, enabled ? '1' : '0');
        } catch (_) {}
    };
    const normalizeHumanoidWaistXCorridorMax = (valueRaw) => {
        const parsed = Number(valueRaw);
        return Number.isFinite(parsed) ? parsed : 0.05;
    };
    const normalizeHumanoidWaistXCorridorMin = (valueRaw) => {
        const parsed = Number(valueRaw);
        return Number.isFinite(parsed) ? parsed : -0.1;
    };
    const applyHumanoidWaistXCorridorMin = (valueRaw) => {
        const value = normalizeHumanoidWaistXCorridorMin(valueRaw);
        if (humanoidWaistXCorridorMinInput) humanoidWaistXCorridorMinInput.value = String(value);
        if (typeof window !== 'undefined') {
            window.humanoidWaistXCorridorMin = value;
        }
        if (appState.visualizer?.setHumanoidWaistXCorridorMin) {
            appState.visualizer.setHumanoidWaistXCorridorMin(value);
        }
        try {
            localStorage.setItem(HUMANOID_WAIST_X_CORRIDOR_MIN_STORAGE_KEY, String(value));
        } catch (_) {}
    };
    const applyHumanoidWaistXCorridorMax = (valueRaw) => {
        const value = normalizeHumanoidWaistXCorridorMax(valueRaw);
        if (humanoidWaistXCorridorMaxInput) humanoidWaistXCorridorMaxInput.value = String(value);
        if (typeof window !== 'undefined') {
            window.humanoidWaistXCorridorMax = value;
        }
        if (appState.visualizer?.setHumanoidWaistXCorridorMax) {
            appState.visualizer.setHumanoidWaistXCorridorMax(value);
        }
        try {
            localStorage.setItem(HUMANOID_WAIST_X_CORRIDOR_MAX_STORAGE_KEY, String(value));
        } catch (_) {}
    };
    const triggerHumanoidWMetricOptimization = (direction) => {
        if (!appState.visualizer) return;
        const metric = normalizeHumanoidJogMetric(
            humanoidJogMetricSelect?.value
                ?? (typeof window !== 'undefined' ? window.humanoidManifoldJogSecondaryMetric : null)
        );
        if (metric === 'w_abs_hier_vfi') {
            appState.visualizer.optimizeHumanoidHierarchicalAbsoluteVFIManipulability?.(direction);
            return;
        }
        appState.visualizer.optimizeHumanoidHierarchicalAbsoluteManipulability?.(direction);
    };
    const bindRepeatingMetricButton = (button, direction) => {
        if (!button) return;
        let repeatDelayTimeoutId = null;
        let repeatIntervalId = null;
        let pointerTriggeredClick = false;

        const clearRepeatDelay = () => {
            if (repeatDelayTimeoutId === null) return;
            clearTimeout(repeatDelayTimeoutId);
            repeatDelayTimeoutId = null;
        };
        const clearRepeatInterval = () => {
            if (repeatIntervalId === null) return;
            clearInterval(repeatIntervalId);
            repeatIntervalId = null;
        };
        const stopRepeating = () => {
            clearRepeatDelay();
            clearRepeatInterval();
        };
        const fire = () => triggerHumanoidWMetricOptimization(direction);
        const startRepeating = () => {
            clearRepeatInterval();
            repeatIntervalId = setInterval(() => {
                fire();
            }, HUMANOID_W_METRIC_REPEAT_INTERVAL_MS);
        };
        const handlePointerDown = (event) => {
            event?.preventDefault?.();
            stopRepeating();
            pointerTriggeredClick = true;
            fire();
            repeatDelayTimeoutId = setTimeout(() => {
                repeatDelayTimeoutId = null;
                startRepeating();
            }, HUMANOID_W_METRIC_HOLD_DELAY_MS);
        };
        const handleClick = (event) => {
            if (pointerTriggeredClick) {
                pointerTriggeredClick = false;
                event?.preventDefault?.();
                return;
            }
            fire();
        };

        button.addEventListener('pointerdown', handlePointerDown);
        button.addEventListener('pointerup', stopRepeating);
        button.addEventListener('pointerleave', stopRepeating);
        button.addEventListener('pointercancel', stopRepeating);
        button.addEventListener('click', handleClick);
        if (typeof window !== 'undefined' && typeof window.addEventListener === 'function') {
            window.addEventListener('blur', stopRepeating);
        }
    };
    if (
        humanoidJogSolverSelect ||
        humanoidJogMetricSelect ||
        humanoidWaistXCorridorToggle ||
        humanoidTestDualArmLockToggle ||
        humanoidWMetricIncreaseBtn
    ) {
        const saved = (() => {
            try { return localStorage.getItem(HUMANOID_JOG_SOLVER_STORAGE_KEY); } catch (_) { return null; }
        })();
        const savedMetric = (() => {
            try { return localStorage.getItem(HUMANOID_JOG_METRIC_STORAGE_KEY); } catch (_) { return null; }
        })();
        const savedTestDualArmLock = (() => {
            try { return localStorage.getItem(HUMANOID_TEST_DUAL_ARM_LOCK_STORAGE_KEY); } catch (_) { return null; }
        })();
        const savedWaistXCorridor = (() => {
            try { return localStorage.getItem(HUMANOID_WAIST_X_CORRIDOR_STORAGE_KEY); } catch (_) { return null; }
        })();
        const savedWaistXCorridorMin = (() => {
            try { return localStorage.getItem(HUMANOID_WAIST_X_CORRIDOR_MIN_STORAGE_KEY); } catch (_) { return null; }
        })();
        const savedWaistXCorridorMax = (() => {
            try { return localStorage.getItem(HUMANOID_WAIST_X_CORRIDOR_MAX_STORAGE_KEY); } catch (_) { return null; }
        })();
        applyHumanoidJogSolverMode(saved ?? humanoidJogSolverSelect.value ?? 'vfi');
        applyHumanoidJogMetric(savedMetric ?? humanoidJogMetricSelect?.value ?? 'w_abs_hier');
        updateHumanoidJogMetricOptionLabels();
        ensureHumanoidJogSolverSupportsControlMode(
            dualArmControlModeSelect?.value
            ?? (typeof window !== 'undefined' ? window.currentDualArmControlMode : null)
        );
        applyHumanoidWaistXCorridorMin(
            savedWaistXCorridorMin
            ?? ((typeof window !== 'undefined' && Number.isFinite(window.humanoidWaistXCorridorMin))
                ? window.humanoidWaistXCorridorMin
                : -0.1)
        );
        applyHumanoidWaistXCorridorMax(
            savedWaistXCorridorMax
            ?? ((typeof window !== 'undefined' && Number.isFinite(window.humanoidWaistXCorridorMax))
                ? window.humanoidWaistXCorridorMax
                : 0.05)
        );
        applyHumanoidWaistXCorridor(
            savedWaistXCorridor != null
                ? savedWaistXCorridor === '1'
                : ((typeof window !== 'undefined' && typeof window.humanoidWaistXCorridorEnabled === 'boolean')
                    ? window.humanoidWaistXCorridorEnabled
                    : false)
        );
        applyHumanoidTestDualArmLock(
            savedTestDualArmLock != null
                ? savedTestDualArmLock === '1'
                : ((typeof window !== 'undefined' && typeof window.humanoidTestDualArmLockEnabled === 'boolean')
                    ? window.humanoidTestDualArmLockEnabled
                    : false)
        );
        humanoidJogSolverSelect.addEventListener('change', (e) => {
            applyHumanoidJogSolverMode(e.target?.value);
        });
        if (humanoidJogMetricSelect) {
            humanoidJogMetricSelect.addEventListener('change', (e) => {
                applyHumanoidJogMetric(e.target?.value);
            });
        }
        if (dualArmControlModeSelect) {
            dualArmControlModeSelect.addEventListener('change', (e) => {
                updateHumanoidJogMetricOptionLabels(e.target?.value);
                ensureHumanoidJogSolverSupportsControlMode(e.target?.value);
            });
        }
        if (humanoidWaistXCorridorToggle) {
            humanoidWaistXCorridorToggle.addEventListener('change', (e) => {
                applyHumanoidWaistXCorridor(Boolean(e.target?.checked));
            });
        }
        if (humanoidWaistXCorridorMinInput) {
            humanoidWaistXCorridorMinInput.addEventListener('change', (e) => {
                applyHumanoidWaistXCorridorMin(e.target?.value);
            });
        }
        if (humanoidWaistXCorridorMaxInput) {
            humanoidWaistXCorridorMaxInput.addEventListener('change', (e) => {
                applyHumanoidWaistXCorridorMax(e.target?.value);
            });
        }
        if (humanoidTestDualArmLockToggle) {
            humanoidTestDualArmLockToggle.addEventListener('change', (e) => {
                applyHumanoidTestDualArmLock(Boolean(e.target?.checked));
            });
        }
        bindRepeatingMetricButton(humanoidWMetricIncreaseBtn, 'increase');
        if (typeof window !== 'undefined') {
            window.setHumanoidJogSolverMode = applyHumanoidJogSolverMode;
            window.setHumanoidJogMetric = applyHumanoidJogMetric;
            window.setHumanoidWaistXCorridorEnabled = applyHumanoidWaistXCorridor;
            window.setHumanoidWaistXCorridorMin = applyHumanoidWaistXCorridorMin;
            window.setHumanoidWaistXCorridorMax = applyHumanoidWaistXCorridorMax;
            window.setHumanoidTestDualArmLockEnabled = applyHumanoidTestDualArmLock;
            window.refreshHumanoidJogMetricOptionLabels = updateHumanoidJogMetricOptionLabels;
        }
    }

    return {
        applyHumanoidJogSolverMode,
        applyHumanoidJogMetric,
        applyHumanoidWaistXCorridor,
        applyHumanoidWaistXCorridorMin,
        applyHumanoidWaistXCorridorMax,
        applyHumanoidTestDualArmLock
    };
}

// ═══════════════════════════════════════════
//  Main Event Listeners Setup
// ═══════════════════════════════════════════

function setupEventListeners() {
    // 预设位姿按钮
    document.querySelectorAll('.preset-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const preset = btn.dataset.preset;
            applyPreset(preset);
        });
    });

    // DQ 演示按钮（可选）
    const dqRotationDemoBtn = document.getElementById('dq-rotation-demo');
    if (dqRotationDemoBtn) {
        dqRotationDemoBtn.addEventListener('click', demoRotation);
    }
    const dqTranslationDemoBtn = document.getElementById('dq-translation-demo');
    if (dqTranslationDemoBtn) {
        dqTranslationDemoBtn.addEventListener('click', demoTranslation);
    }
    const dqPoseDemoBtn = document.getElementById('dq-pose-demo');
    if (dqPoseDemoBtn) {
        dqPoseDemoBtn.addEventListener('click', demoPose);
    }
    
    // URDF → MDH 按钮（可选）
    const extractMdhBtn = document.getElementById('extract-mdh-btn');
    if (extractMdhBtn) {
        extractMdhBtn.addEventListener('click', extractMDHFromCurrentURDF);
    }
    const compareMdhBtn = document.getElementById('compare-mdh-btn');
    if (compareMdhBtn) {
        compareMdhBtn.addEventListener('click', compareMDHParameters);
    }
    const useUrdfMdhBtn = document.getElementById('use-urdf-mdh-btn');
    if (useUrdfMdhBtn) {
        useUrdfMdhBtn.addEventListener('click', useURDFMDH);
    }
    
    // MDH坐标系显示开关
    const mdhFramesToggle = document.getElementById('show-mdh-frames-toggle');
    if (mdhFramesToggle) {
        mdhFramesToggle.addEventListener('change', (e) => {
            if (appState.visualizer) {
                appState.visualizer.toggleMDHFrames(e.target.checked);
            }
        });
    }
    
    // URDF坐标系显示开关
    const urdfFramesToggle = document.getElementById('show-urdf-frames-toggle');
    if (urdfFramesToggle) {
        urdfFramesToggle.addEventListener('change', (e) => {
            if (appState.visualizer) {
                appState.visualizer.toggleURDFFrames(e.target.checked);
            }
        });
    }
    
    const angleUnitToggle = document.getElementById('angle-unit-toggle');
    if (angleUnitToggle) {
        const updateAngleUnitToggleLabel = () => {
            angleUnitToggle.textContent = appState.angleDisplayMode === 'deg' ? t('Switch to Radians') : t('Switch to Degrees');
        };
        updateAngleUnitToggleLabel();
        onLanguageChange(updateAngleUnitToggleLabel);
        angleUnitToggle.addEventListener('click', () => {
            appState.angleDisplayMode = appState.angleDisplayMode === 'deg' ? 'rad' : 'deg';
            if (callbacks.refreshJointControlDisplay) {
                callbacks.refreshJointControlDisplay();
            }
            updateAngleUnitToggleLabel();
        });
    }

    const topDownViewBtn = document.getElementById('top-down-view-btn');
    if (topDownViewBtn) {
        topDownViewBtn.addEventListener('click', () => {
            topDownViewBtn.disabled = true;
            try {
                if (appState.visualizer?.setTopDownView) {
                    appState.visualizer.setTopDownView();
                }
                topDownViewBtn.textContent = t('Top View Set');
            } catch (error) {
                console.error('切换俯视视角失败:', error);
                topDownViewBtn.textContent = t('Switch Failed');
            } finally {
                setTimeout(() => {
                    topDownViewBtn.textContent = t('Top View');
                    topDownViewBtn.disabled = false;
                }, 1200);
            }
        });
    }

    const importAnglesBtn = document.getElementById('import-joint-angles-btn');
    if (importAnglesBtn) {
        const defaultText = importAnglesBtn.textContent.trim();
        importAnglesBtn.addEventListener('click', async () => {
            let inputText = '';
            importAnglesBtn.disabled = true;
            try {
                if (navigator?.clipboard?.readText) {
                    inputText = await navigator.clipboard.readText();
                    if (!inputText || !inputText.trim()) {
                        throw new Error(t('Clipboard is empty; cannot import joint angles'));
                    }
                } else {
                    const promptText = window.prompt(t('Paste joint angle array (supports direct paste from copied output):'));
                    if (promptText === null) {
                        importAnglesBtn.disabled = false;
                        return;
                    }
                    inputText = promptText;
                }

                if (callbacks.importJointAnglesFromText) {
                    callbacks.importJointAnglesFromText(inputText);
                }
                importAnglesBtn.textContent = t('Joint angle import complete');
            } catch (error) {
                console.error('导入关节角度失败:', error);
                importAnglesBtn.textContent = t('Joint angle import failed');
                window.alert(t('Failed to import joint angles: {{message}}', { message: error.message }));
            } finally {
                setTimeout(() => {
                    importAnglesBtn.textContent = t('Import from Clipboard');
                    importAnglesBtn.disabled = false;
                }, 1500);
            }
        });
    }

    const addEnvironmentBoxBtn = document.getElementById('add-environment-box-btn');
    if (addEnvironmentBoxBtn) {
        const defaultLabel = addEnvironmentBoxBtn.textContent.trim();
        addEnvironmentBoxBtn.addEventListener('click', () => {
            addEnvironmentBoxBtn.disabled = true;
            try {
                if (callbacks.addEnvironmentBoxToScene) {
                    callbacks.addEnvironmentBoxToScene();
                }
                addEnvironmentBoxBtn.textContent = t('Environment box added');
            } catch (error) {
                console.error('添加环境箱体失败:', error);
                addEnvironmentBoxBtn.textContent = t('Add failed');
                window.alert(t('Failed to add environment box: {{message}}', { message: error.message }));
            } finally {
                setTimeout(() => {
                    addEnvironmentBoxBtn.textContent = t('Add Box (41.5 × 30.5 × 14 cm)');
                    addEnvironmentBoxBtn.disabled = false;
                }, 1500);
            }
        });
    }

    // 场景预设下拉菜单
    setupScenePresetListeners();

    const copyAnglesBtn = document.getElementById('copy-joint-angles-btn');
    if (copyAnglesBtn) {
        const defaultCopyText = copyAnglesBtn.textContent.trim();
        copyAnglesBtn.addEventListener('click', async () => {
            copyAnglesBtn.disabled = true;
            try {
                if (callbacks.copyJointAnglesToClipboard) {
                    await callbacks.copyJointAnglesToClipboard();
                }
                copyAnglesBtn.textContent = t('Copied!');
            } catch (error) {
                console.error('复制关节角度失败:', error);
                copyAnglesBtn.textContent = t('Copy failed');
            } finally {
                setTimeout(() => {
                    copyAnglesBtn.textContent = t('Copy Joint Angles');
                    copyAnglesBtn.disabled = false;
                }, 1500);
            }
        });
    }

    const copyLimitsBtn = document.getElementById('copy-joint-limits-btn');
    if (copyLimitsBtn) {
        const defaultText = copyLimitsBtn.textContent.trim();
        copyLimitsBtn.addEventListener('click', async () => {
            copyLimitsBtn.disabled = true;
            try {
                if (callbacks.copyJointLimitsToClipboard) {
                    await callbacks.copyJointLimitsToClipboard();
                }
                copyLimitsBtn.textContent = t('Copied!');
            } catch (error) {
                console.error('复制关节限位失败:', error);
                copyLimitsBtn.textContent = t('Copy failed');
            } finally {
                setTimeout(() => {
                    copyLimitsBtn.textContent = t('Copy Joint Limits');
                    copyLimitsBtn.disabled = false;
                }, 1500);
            }
        });
    }
    
    setupTeachOrderControls();

    // 示教器速度百分比（影响每步增量）
    const teachSpeedSlider = document.getElementById('teach-speed-percent');
    const teachSpeedValue = document.getElementById('teach-speed-value');
    const TEACH_SPEED_STORAGE_KEY = 'teach-speed-percent';
    const clampInt = (v, min, max) => Math.max(min, Math.min(max, Number.parseInt(v, 10) || min));
    const applyTeachSpeedPercent = (percent) => {
        const p = clampInt(percent, 1, 100);
        if (teachSpeedSlider) teachSpeedSlider.value = String(p);
        if (teachSpeedValue) teachSpeedValue.textContent = String(p);
        try {
            localStorage.setItem(TEACH_SPEED_STORAGE_KEY, String(p));
        } catch (_) {}
        if (appState.visualizer?.setTeachSpeedPercent) {
            appState.visualizer.setTeachSpeedPercent(p);
        } else if (appState.visualizer) {
            appState.visualizer.teachSpeedPercent = p;
        }
    };
    if (teachSpeedSlider) {
        const saved = (() => {
            try { return localStorage.getItem(TEACH_SPEED_STORAGE_KEY); } catch (_) { return null; }
        })();
        applyTeachSpeedPercent(saved ?? teachSpeedSlider.value ?? 100);
        teachSpeedSlider.addEventListener('input', (e) => {
            applyTeachSpeedPercent(e.target?.value);
        });
    }

    setupHumanoidDefaultPoseControls();
    setupHumanoidParameterControls();
    setupHumanoidCooperativePolishingSurfaceControls();
    setupHumanoidCooperativePolishingObjectPoseControls();
    setupHumanoidJogSolverControls();
    
    // 示教器控制按钮（移除拖动末端执行器模式）
    const teachButtons = document.querySelectorAll('.teach-btn');
    teachButtons.forEach(button => {
        let intervalId = null;
        
        button.addEventListener('mousedown', (e) => {
            // 相对距离按钮单独处理（避免走 6 轴逻辑）
            if (button.classList.contains('teach-distance-btn')) {
                const direction = button.dataset.direction;
                if (!appState.visualizer || !direction) return;
                
                // 双臂或人形模式 + distance/distance-fullbody 模式下有效
                if (!appState.visualizer.dualArmMode && !appState.visualizer.humanoidMode) return;
                const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
                const controlMode = controlModeSelect ? controlModeSelect.value : 'absolute';
                if (controlMode !== 'distance' && controlMode !== 'distance-fullbody') return;
                if (typeof appState.visualizer.moveDualArmRelativeDistance !== 'function') return;
                
                const includeWaist = (controlMode === 'distance-fullbody');
                appState.visualizer.moveDualArmRelativeDistance(direction, includeWaist);
                intervalId = setInterval(() => {
                    appState.visualizer.moveDualArmRelativeDistance(direction, includeWaist);
                }, 20);
                return;
            }
            const axis = button.dataset.axis;
            const direction = button.dataset.direction;
            const type = button.dataset.type || 'translation'; // 'translation' 或 'rotation'
            
            if (!appState.visualizer || !axis || !direction) return;
            if (appState.visualizer.humanoidMode && typeof appState.visualizer.recordHumanoidJogAxisIntent === 'function') {
                appState.visualizer.recordHumanoidJogAxisIntent(axis);
            }
            
            // 检查是否为双臂或人形模式（人形下左臂=robot1，右臂=robot2，支持绝对/相对/距离控制）
            const isDualArm = appState.visualizer.dualArmMode || appState.visualizer.humanoidMode;
            
            if (isDualArm) {
                // 获取控制模式选择器
                const controlModeSelect = document.getElementById('dual-arm-control-mode-select');
                const controlMode = controlModeSelect ? controlModeSelect.value : 'absolute';
                // 人形模式下左臂/右臂单臂示教
                if (appState.visualizer.humanoidMode && controlMode === 'robot1-waist') {
                    if (type === 'rotation') {
                        appState.visualizer.rotateHumanoidLeftArmWithWaistAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateHumanoidLeftArmWithWaistAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveHumanoidLeftArmWithWaistAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidLeftArmWithWaistAlongAxis(axis, direction);
                        }, 20);
                    }
                    return;
                }
                if (appState.visualizer.humanoidMode && controlMode === 'robot2-waist') {
                    if (type === 'rotation') {
                        appState.visualizer.rotateHumanoidRightArmWithWaistAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateHumanoidRightArmWithWaistAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveHumanoidRightArmWithWaistAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidRightArmWithWaistAlongAxis(axis, direction);
                        }, 20);
                    }
                    return;
                }
                if (appState.visualizer.humanoidMode && controlMode === 'robot1') {
                    // 人形机器人左臂控制
                    if (type === 'rotation') {
                        appState.visualizer.rotateHumanoidLeftArmAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateHumanoidLeftArmAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveHumanoidLeftArmAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidLeftArmAlongAxis(axis, direction);
                        }, 20);
                    }
                    return;
                }
                if (appState.visualizer.humanoidMode && controlMode === 'robot2') {
                    // 人形机器人右臂控制
                    if (type === 'rotation') {
                        appState.visualizer.rotateHumanoidRightArmAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateHumanoidRightArmAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveHumanoidRightArmAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidRightArmAlongAxis(axis, direction);
                        }, 20);
                    }
                    return;
                }
                if (
                    appState.visualizer.humanoidMode
                    && (
                        controlMode === 'cooperative-ruled-surface-polishing'
                        || controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint'
                        || controlMode === 'cooperative-ruled-surface-polishing-dq-setpoint-arm-only'
                    )
                ) {
                    const parameterAxis = type === 'rotation'
                        ? (axis === 'rx' ? 'rx' : null)
                        : axis;
                    if (parameterAxis !== 'x' && parameterAxis !== 'y' && parameterAxis !== 'z' && parameterAxis !== 'rx') return;
                    let cooperativeMovePending = false;
                    const stopRepeating = () => {
                        if (intervalId) {
                            clearInterval(intervalId);
                            intervalId = null;
                        }
                    };
                    const settleCooperativeMove = (result) => {
                        cooperativeMovePending = false;
                        if (result === false) {
                            stopRepeating();
                        }
                    };
                    const triggerCooperativeMove = () => {
                        if (cooperativeMovePending) return;
                        cooperativeMovePending = true;
                        try {
                            const moveResult = appState.visualizer.moveHumanoidCooperativeRuledSurfacePolishingAlongParameter(parameterAxis, direction);
                            if (moveResult && typeof moveResult.then === 'function') {
                                void moveResult.then(settleCooperativeMove).catch(() => {
                                    settleCooperativeMove(false);
                                });
                                return;
                            }
                            settleCooperativeMove(moveResult);
                        } catch (error) {
                            settleCooperativeMove(false);
                            throw error;
                        }
                    };
                    intervalId = setInterval(() => {
                        triggerCooperativeMove();
                    }, 20);
                    triggerCooperativeMove();
                    return;
                }
                if (appState.visualizer.humanoidMode && controlMode === 'left-ruled-surface-polishing') {
                    const parameterAxis = type === 'rotation' ? null : axis;
                    if (parameterAxis !== 'x' && parameterAxis !== 'y' && parameterAxis !== 'z') return;
                    const moveAlongParameter = appState.visualizer?.moveHumanoidLeftRuledSurfacePolishingAlongParameter;
                    if (typeof moveAlongParameter !== 'function') return;
                    moveAlongParameter.call(appState.visualizer, parameterAxis, direction);
                    intervalId = setInterval(() => {
                        moveAlongParameter.call(appState.visualizer, parameterAxis, direction);
                    }, 20);
                    return;
                }
                if (controlMode === 'robot1') {
                    // 控制机器人1
                    if (type === 'rotation') {
                        appState.visualizer.rotateRobot1EndEffectorAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateRobot1EndEffectorAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveRobot1EndEffectorAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveRobot1EndEffectorAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'robot2') {
                    // 控制机器人2
                    if (type === 'rotation') {
                        appState.visualizer.rotateRobot2EndEffectorAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateRobot2EndEffectorAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveRobot2EndEffectorAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveRobot2EndEffectorAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'relative') {
                    // 控制相对位姿（保持绝对位姿不变）
                    if (type === 'rotation') {
                        appState.visualizer.rotateDualArmRelativePoseAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateDualArmRelativePoseAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveDualArmRelativePoseAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveDualArmRelativePoseAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'decoupled') {
                    // 全身绝对位姿：腰部+双臂联动，控制绝对位姿
                    if (appState.visualizer.humanoidMode) {
                        if (type === 'rotation') {
                            appState.visualizer.rotateHumanoidAbsolutePoseAroundAxisDecoupled(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.rotateHumanoidAbsolutePoseAroundAxisDecoupled(axis, direction);
                            }, 20);
                        } else {
                            appState.visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupled(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupled(axis, direction);
                            }, 20);
                        }
                    }
                } else if (controlMode === 'decoupled-upright') {
                    // 全身绝对位姿（腰部竖直）：腰部+双臂联动，控制绝对位姿并约束 body_link z 轴
                    if (appState.visualizer.humanoidMode) {
                        if (type === 'rotation') {
                            appState.visualizer.rotateHumanoidAbsolutePoseAroundAxisDecoupledUpright(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.rotateHumanoidAbsolutePoseAroundAxisDecoupledUpright(axis, direction);
                            }, 20);
                        } else {
                            appState.visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupledUpright(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.moveHumanoidAbsolutePoseAlongAxisDecoupledUpright(axis, direction);
                            }, 20);
                        }
                    }
                } else if (controlMode === 'waist-body-link-xz') {
                    // 腰部 body_link 点动：仅支持 X/Z 平移（仅人形模式）
                    if (appState.visualizer.humanoidMode) {
                        if (type === 'rotation') return;
                        if (axis !== 'x' && axis !== 'z') return;
                        appState.visualizer.moveHumanoidWaistBodyLinkAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveHumanoidWaistBodyLinkAlongAxis(axis, direction);
                        }, 20);
                    }
                } else if (controlMode === 'relative-fullbody') {
                    // 全身相对位姿：腰部+双臂联动，控制相对位姿
                    if (appState.visualizer.humanoidMode) {
                        if (type === 'rotation') {
                            appState.visualizer.rotateHumanoidRelativePoseAroundAxisFullbody(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.rotateHumanoidRelativePoseAroundAxisFullbody(axis, direction);
                            }, 20);
                        } else {
                            appState.visualizer.moveHumanoidRelativePoseAlongAxisFullbody(axis, direction);
                            intervalId = setInterval(() => {
                                appState.visualizer.moveHumanoidRelativePoseAlongAxisFullbody(axis, direction);
                            }, 20);
                        }
                    }
                } else if (controlMode === 'distance') {
                    // 距离模式：按键已单独处理，这里兜底不做事
                    return;
                } else {
                    // 默认：控制绝对位姿（保持相对位姿不变）
                    if (type === 'rotation') {
                        appState.visualizer.rotateDualArmAbsolutePoseAroundAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.rotateDualArmAbsolutePoseAroundAxis(axis, direction);
                        }, 20);
                    } else {
                        appState.visualizer.moveDualArmAbsolutePoseAlongAxis(axis, direction);
                        intervalId = setInterval(() => {
                            appState.visualizer.moveDualArmAbsolutePoseAlongAxis(axis, direction);
                        }, 20);
                    }
                }
            } else {
                // 单臂模式：控制末端执行器
                if (type === 'rotation') {
                    // 旋转控制
                    appState.visualizer.rotateEndEffectorAroundAxis(axis, direction);
                    intervalId = setInterval(() => {
                        appState.visualizer.rotateEndEffectorAroundAxis(axis, direction);
                    }, 20);
                } else {
                    // 平移控制
                    appState.visualizer.moveEndEffectorAlongAxis(axis, direction);
                    intervalId = setInterval(() => {
                        appState.visualizer.moveEndEffectorAlongAxis(axis, direction);
                    }, 20);
                }
            }
        });
        
        button.addEventListener('mouseup', () => {
            if (intervalId) {
                clearInterval(intervalId);
                intervalId = null;
            }
        });
        
        button.addEventListener('mouseleave', () => {
            if (intervalId) {
                clearInterval(intervalId);
                intervalId = null;
            }
        });
    });

    // 全局拖拽加载
    if (callbacks.renderEnvironmentObjectList) {
        callbacks.renderEnvironmentObjectList();
    }
    setupGlobalDragAndDrop();
    
    // 浮动面板与底部功能栏
    if (callbacks.setupFloatingPanels) {
        callbacks.setupFloatingPanels();
    }
}

// ═══════════════════════════════════════════
//  Presets & Tab Switching
// ═══════════════════════════════════════════

async function applyPreset(preset) {
    const numJoints = appState.jointAngles.length;
    
    // 获取当前配置和关节名称（用于限位检查）
    let config = null;
    let jointNames = [];
    let urdfRobot = null;
    
    // 尝试从当前加载的配置获取信息
    if (appState.customRobotConfig) {
        config = appState.customRobotConfig;
        jointNames = config.joint_chain?.joints || [];
        urdfRobot = appState.visualizer?.urdfRobot || null;
    } else if (appState.currentRobotType === 'franka') {
        // 对于内置Franka，尝试加载配置文件获取限位
        try {
            const baseUrl = import.meta.env.BASE_URL || '/';
            const response = await fetch(`${baseUrl}robots/franka/robot_config.yaml`);
            if (response.ok) {
                const text = await response.text();
                config = yaml.load(text);
                jointNames = config.joint_chain?.joints || [];
            }
        } catch (error) {
            // 使用硬编码的关节名称
            jointNames = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
                         'panda_joint5', 'panda_joint6', 'panda_joint7'];
        }
    }
    
    switch (preset) {
        case 'home':
            appState.jointAngles = new Array(numJoints).fill(0);
            break;
            
        case 'extended':
            if (appState.currentRobotType === 'franka') {
                appState.jointAngles = [0, -30, 0, -120, 0, 90, 45].map(deg => deg * Math.PI / 180);
            } else {
                appState.jointAngles = [0, -45, 90, 0, 45, 0, 0].map(deg => deg * Math.PI / 180);
            }
            break;
            
        case 'folded':
            if (appState.currentRobotType === 'franka') {
                appState.jointAngles = [0, 45, 0, -90, 0, 135, 0].map(deg => deg * Math.PI / 180);
            } else {
                appState.jointAngles = [0, 45, -90, 0, -45, 0, 0].map(deg => deg * Math.PI / 180);
            }
            break;
            
        case 'random':
            appState.jointAngles = appState.jointAngles.map(() => (Math.random() - 0.5) * Math.PI);
            break;
    }
    
    // 限制角度在限位范围内
    if (jointNames.length === appState.jointAngles.length && callbacks.clampJointAngles) {
        appState.jointAngles = callbacks.clampJointAngles(appState.jointAngles, jointNames, urdfRobot, config);
    }
    
    if (appState.dualArmMode) {
        if (callbacks.updateDualArmJointSlidersFromAngles) {
            callbacks.updateDualArmJointSlidersFromAngles();
        }
    } else {
        if (callbacks.updateJointSlidersFromAngles) {
            callbacks.updateJointSlidersFromAngles();
        }
    }
    
    if (callbacks.updateRobotPose) {
        callbacks.updateRobotPose();
    }
}

// ═══════════════════════════════════════════
//  DQ Demos
// ═══════════════════════════════════════════

/**
 * DQ 旋转演示
 */
function demoRotation() {
    console.log('=== DQ 旋转演示 ===');
    
    // 创建绕 Z 轴旋转 90 度的 DQ
    const rotation = DQ.rotation(Math.PI / 2, 0, 0, 1);
    console.log('旋转 DQ (90° 绕 Z 轴):', rotation.toArray());
    
    alert(t('See the console for the rotation DQ output'));
}

/**
 * DQ 平移演示
 */
function demoTranslation() {
    console.log('=== DQ 平移演示 ===');
    
    // 创建平移 (1, 2, 3) 的 DQ
    const translation = DQ.translation(1, 2, 3);
    console.log('平移 DQ (1, 2, 3):', translation.toArray());
    
    alert(t('See the console for the translation DQ output'));
}

/**
 * DQ 位姿演示
 */
function demoPose() {
    console.log('=== DQ 位姿演示 ===');
    
    // 创建组合位姿: 绕 Z 轴旋转 45° + 平移 (0.5, 0.5, 0.5)
    const pose = DQ.pose(0, 0, 1, Math.PI / 4, 0.5, 0.5, 0.5);
    console.log('位姿 DQ:', pose.toArray());
    
    const translation = pose.getTranslation();
    const rotation = pose.getRotation();
    console.log('平移部分:', translation);
    console.log('旋转部分:', rotation);
    
    alert(t('See the console for the pose DQ output'));
}

// ═══════════════════════════════════════════
//  MDH Parameter Extraction & Comparison
// ═══════════════════════════════════════════

/**
 * 从当前 URDF 提取 MDH 参数
 */
function extractMDHFromCurrentURDF() {
    if (!appState.visualizer || !appState.visualizer.urdfRobot) {
        alert(t('⚠ URDF model is not loaded\n\nPlease make sure the URDF model has loaded successfully.'));
        return;
    }
    
    console.log('═══════════════════════════════════════════');
    console.log('       从 URDF 提取 MDH 参数');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        const jointNames = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6',
            'panda_joint7'
        ];
        
        const result = extractMDHFromURDF(appState.visualizer.urdfRobot, jointNames);
        // 处理新的返回格式
        if (result && typeof result === 'object' && result.mdh) {
            appState.extractedMDH = result.mdh;
            
            // 设置MDH坐标系可视化
            if (result.allMDHFrames && appState.visualizer) {
                appState.visualizer.setMDHFrames(result.allMDHFrames);
                console.log('  ✓ 已设置MDH坐标系可视化');
            }
        } else {
            appState.extractedMDH = result; // 向后兼容
        }
        
        // 显示状态
        const statusDiv = document.getElementById('mdh-status');
        statusDiv.innerHTML = `
            <div style="color: #10b981; font-size: 0.875rem; margin-top: 0.5rem;">
                ✓ 已提取 ${appState.extractedMDH.length} 个关节的 MDH 参数<br>
                查看控制台了解详情
            </div>
        `;
        
        alert(t('✓ MDH parameters extracted successfully!\n\nPlease open the console (F12) to view the details.'));
        
    } catch (error) {
        console.error('提取 MDH 失败:', error);
        alert(t('❌ Extraction failed\n\n{{message}}', { message: error.message }));
    }
}

/**
 * 对比 MDH 参数
 */
function compareMDHParameters() {
    if (!appState.visualizer || !appState.visualizer.urdfRobot) {
        alert(t('⚠ URDF model is not loaded'));
        return;
    }
    
    console.log('═══════════════════════════════════════════');
    console.log('       MDH 参数对比测试');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        const result = createMDHTestSuite(appState.visualizer.urdfRobot);
        
        const statusDiv = document.getElementById('mdh-status');
        
        if (result.comparison.match) {
            statusDiv.innerHTML = `
                <div style="color: #10b981; font-size: 0.875rem; margin-top: 0.5rem;">
                    ✓ MDH 参数完全一致！<br>
                    URDF 和预定义参数匹配
                </div>
            `;
            alert(t('✓ Parameters match!\n\nThe MDH parameters extracted from the URDF exactly match the official reference.\n\nSee the console for details.'));
        } else {
            const diffCount = result.comparison.differences.length;
            statusDiv.innerHTML = `
                <div style="color: #f59e0b; font-size: 0.875rem; margin-top: 0.5rem;">
                    ⚠ 发现 ${diffCount} 处差异<br>
                    查看控制台了解详情
                </div>
            `;
            alert(t('⚠ Differences found\n\n{{count}} parameters differ.\n\nPossible reasons:\n- Different frame conventions\n- Different zero-position definitions\n- Different parameter sources\n\nSee the console for details.', { count: diffCount }));
        }
        
    } catch (error) {
        console.error('对比失败:', error);
        alert(t('❌ Comparison failed\n\n{{message}}', { message: error.message }));
    }
}

/**
 * 使用 URDF 提取的 MDH 创建机器人
 */
function useURDFMDH() {
    if (!appState.extractedMDH) {
        alert(t('⚠ Please extract the MDH parameters first.\n\nClick "Extract MDH from URDF".'));
        return;
    }
    
    try {
        console.log('使用 URDF 提取的 MDH 参数创建机器人...');
        
        // 格式化为 DQ Robotics 格式
        const formatted = formatMDHForDQ(appState.extractedMDH);
        
        // 转置为列格式（DQ 需要的格式）
        const mdhMatrix = [];
        for (let row = 0; row < 5; row++) {
            mdhMatrix.push(formatted[row]);
        }
        
        // 创建新的串联机械臂
        appState.urdfBasedRobot = new SerialManipulator(mdhMatrix);
        
        // 切换到使用新机器人
        appState.currentRobot = appState.urdfBasedRobot;
        if (callbacks.updateGlobalVariables) {
            callbacks.updateGlobalVariables();
        }
        
        const statusDiv = document.getElementById('mdh-status');
        statusDiv.innerHTML = `
            <div style="color: #3b82f6; font-size: 0.875rem; margin-top: 0.5rem;">
                ✓ 正在使用 URDF-MDH<br>
                运动学现在基于URDF参数
            </div>
        `;
        
        // 更新显示
        if (callbacks.updateRobotPose) {
            callbacks.updateRobotPose();
        }
        
        alert(t('✓ Switch successful!\n\nThe kinematics now use the MDH parameters extracted from the URDF.\n\nThe yellow marker should align exactly with the URDF end-effector.'));
        
    } catch (error) {
        console.error('切换失败:', error);
        alert(t('❌ Switch failed\n\n{{message}}', { message: error.message }));
    }
}

// ═══════════════════════════════════════════
//  Callback Registration
// ═══════════════════════════════════════════

/**
 * 注册事件处理回调到全局 callbacks 对象
 */
export function registerEventHandlersCallbacks() {
    callbacks.setupEventListeners = setupEventListeners;
}

// ═══════════════════════════════════════════
//  Exports
// ═══════════════════════════════════════════

export {
    setupEventListeners,
    setupHumanoidJogSolverControls,
    applyPreset,
    demoRotation,
    demoTranslation,
    demoPose,
    extractMDHFromCurrentURDF,
    compareMDHParameters,
    useURDFMDH,
};
