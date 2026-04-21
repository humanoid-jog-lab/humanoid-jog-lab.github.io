import { CM_TO_M, DEFAULT_ENVIRONMENT_BOX_CM, DEFAULT_ENVIRONMENT_BOX_COLOR, ENVIRONMENT_BOX_BASE_OFFSET_X } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { createWarehouseEnvironment, getWarehouseInfo } from '../environments/warehouse.js';
import { onLanguageChange, t } from '../i18n.js';

let environmentPanelLanguageBound = false;

function getEnvironmentObjectTitle(object) {
    if (object?.type === 'box' && Number.isFinite(object?.displayNameNumber)) {
        return t('Box #{{number}}', { number: object.displayNameNumber });
    }
    return object?.displayName || t('Environment Object');
}

/**
 * 渲染环境物体列表
 */
export function renderEnvironmentObjectList() {
    const list = document.getElementById('environment-object-list');
    if (!list) {
        return;
    }

    list.innerHTML = '';

    if (!appState.environmentObjects.length) {
        const empty = document.createElement('div');
        empty.className = 'env-empty-message';
        empty.textContent = t('No environment objects in the current scene');
        list.appendChild(empty);
        return;
    }

    appState.environmentObjects.forEach((object) => {
        // 同步可视化器中的最新附着及位置
        if (appState.visualizer?.getEnvironmentObjectAttachment) {
            const currentAttachment = appState.visualizer.getEnvironmentObjectAttachment(object.id);
            object.attachedTo = currentAttachment || null;
        }
        if (appState.visualizer?.getEnvironmentObjectWorldPosition) {
            const worldPos = appState.visualizer.getEnvironmentObjectWorldPosition(object.id);
            if (worldPos) {
                object.position = { x: worldPos.x, y: worldPos.y, z: worldPos.z };
            }
        }
        if (appState.visualizer?.getEnvironmentObjectOffset) {
            const offset = appState.visualizer.getEnvironmentObjectOffset(object.id);
            if (offset) {
                object.offset = offset;
            }
        }

        const item = document.createElement('div');
        item.className = 'env-object-item';

        const info = document.createElement('div');
        info.className = 'env-object-info';

        const title = document.createElement('div');
        title.className = 'env-object-title';
        title.textContent = getEnvironmentObjectTitle(object);

        const dimsCm = object.dimensionsCm || {};
        const lengthCm = Number(dimsCm.length ?? 0).toFixed(1);
        const widthCm = Number(dimsCm.width ?? 0).toFixed(1);
        const heightCm = Number(dimsCm.height ?? 0).toFixed(1);

        const dimensions = document.createElement('div');
        dimensions.className = 'env-object-meta';
        dimensions.textContent = t('Size: {{length}} × {{width}} × {{height}} cm', {
            length: lengthCm,
            width: widthCm,
            height: heightCm
        });

        const posInfo = document.createElement('div');
        posInfo.className = 'env-object-meta';
        const pos = object.position || { x: 0, y: 0, z: 0 };
        posInfo.textContent = t('Position (m): ({{x}}, {{y}}, {{z}})', {
            x: pos.x.toFixed(2),
            y: pos.y.toFixed(2),
            z: pos.z.toFixed(2)
        });

        const attachStatus = document.createElement('div');
        attachStatus.className = 'env-object-meta';
        attachStatus.textContent = t('Attachment: {{label}}', {
            label: getEnvironmentAttachmentLabel(object.attachedTo)
        });

        info.appendChild(title);
        info.appendChild(dimensions);
        info.appendChild(posInfo);
        info.appendChild(attachStatus);
        const offset = object.offset || {
            position: { x: 0, y: 0, z: 0 },
            rotation: { x: 0, y: 0, z: 0 }
        };
        const offsetPos = offset.position || { x: 0, y: 0, z: 0 };
        const offsetRot = offset.rotation || { x: 0, y: 0, z: 0 };
        const offsetInfo = document.createElement('div');
        offsetInfo.className = 'env-object-meta';
        offsetInfo.textContent = t(
            'Offset (cm): ({{x}}, {{y}}, {{z}}) | Rotation (deg): ({{roll}}, {{pitch}}, {{yaw}})',
            {
                x: (offsetPos.x / CM_TO_M).toFixed(1),
                y: (offsetPos.y / CM_TO_M).toFixed(1),
                z: (offsetPos.z / CM_TO_M).toFixed(1),
                roll: offsetRot.x.toFixed(1),
                pitch: offsetRot.y.toFixed(1),
                yaw: offsetRot.z.toFixed(1)
            }
        );
        info.appendChild(offsetInfo);

        const controls = document.createElement('div');
        controls.className = 'env-object-controls';

        const attachmentOptions = getEnvironmentAttachmentOptions();
        const attachSelect = document.createElement('select');
        attachSelect.className = 'env-attach-select';
        attachmentOptions.forEach((option) => {
            const opt = document.createElement('option');
            opt.value = option.value;
            opt.textContent = option.label;
            opt.disabled = option.disabled;
            attachSelect.appendChild(opt);
        });
        const currentValue = object.attachedTo || 'none';
        if (!attachmentOptions.some((opt) => opt.value === currentValue && !opt.disabled)) {
            attachSelect.value = 'none';
        } else {
            attachSelect.value = currentValue;
        }
        attachSelect.addEventListener('change', (e) => {
            updateEnvironmentAttachment(object.id, e.target.value);
        });
        controls.appendChild(attachSelect);

        const adjustBtn = document.createElement('button');
        adjustBtn.className = 'env-adjust-btn';
        adjustBtn.textContent = t('Adjust');
        if (!object.attachedTo) {
            adjustBtn.disabled = true;
            adjustBtn.title = t('Adjust offset after attaching');
        }
        adjustBtn.addEventListener('click', () => {
            promptEnvironmentObjectAdjustment(object.id);
        });
        controls.appendChild(adjustBtn);

        const removeBtn = document.createElement('button');
        removeBtn.className = 'env-remove-btn';
        removeBtn.textContent = t('Remove');
        removeBtn.addEventListener('click', () => {
            removeEnvironmentObjectEntry(object.id);
        });
        controls.appendChild(removeBtn);

        item.appendChild(info);
        item.appendChild(controls);
        list.appendChild(item);
    });
}

/**
 * 获取环境附着标签
 * @param {string} value
 * @returns {string}
 */
export function getEnvironmentAttachmentLabel(value) {
    switch (value) {
        case 'end-effector':
            return t('End-Effector Marker');
        case 'dual-absolute':
            return t('Dual-Arm Absolute-Pose Marker');
        default:
            return t('Not Attached');
    }
}

/**
 * 获取环境附着选项
 * @returns {Array}
 */
export function getEnvironmentAttachmentOptions() {
    const options = [
        { value: 'none', label: t('Not Attached'), disabled: false }
    ];
    const hasVisualizer = !!appState.visualizer;
    const hasEndEffector = hasVisualizer && typeof appState.visualizer.hasEndEffectorAnchor === 'function'
        ? appState.visualizer.hasEndEffectorAnchor()
        : false;
    options.push({
        value: 'end-effector',
        label: t('End-Effector Marker'),
        disabled: !hasEndEffector
    });
    const hasAbsolute = hasVisualizer && typeof appState.visualizer.hasAbsolutePoseMarker === 'function'
        ? appState.visualizer.hasAbsolutePoseMarker()
        : false;
    if (hasAbsolute) {
        options.push({
            value: 'dual-absolute',
            label: t('Dual-Arm Absolute-Pose Marker'),
            disabled: false
        });
    }
    return options;
}

/**
 * 弹出环境物体偏移调整对话框
 * @param {string} id
 */
export function promptEnvironmentObjectAdjustment(id) {
    const entry = appState.environmentObjects.find((item) => item.id === id);
    if (!entry) {
        return;
    }
    if (!entry.attachedTo) {
        window.alert(t('Please attach this object to a marker before adjusting it.'));
        return;
    }
    if (!appState.visualizer || typeof appState.visualizer.setEnvironmentObjectOffset !== 'function') {
        window.alert(t('The current visualizer does not support offset adjustment'));
        return;
    }

    const offset = appState.visualizer.getEnvironmentObjectOffset?.(id) || {
        position: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0 }
    };
    const defaultInput = [
        (offset.position.x / CM_TO_M).toFixed(2),
        (offset.position.y / CM_TO_M).toFixed(2),
        (offset.position.z / CM_TO_M).toFixed(2),
        (offset.rotation.x || 0).toFixed(1),
        (offset.rotation.y || 0).toFixed(1),
        (offset.rotation.z || 0).toFixed(1)
    ].join(', ');

    const input = window.prompt(
        t('Enter offset and rotation as: dx, dy, dz, roll, pitch, yaw\ndx/dy/dz are in centimeters, roll/pitch/yaw are in degrees.\nPositive directions follow the right-hand rule with XYZ rotation order.'),
        defaultInput
    );
    if (input === null) {
        return;
    }

    const parts = input.split(/[,\s]+/).filter(Boolean);
    if (parts.length !== 6) {
        window.alert(t('Please enter 6 numeric values, for example: 2, 0, 5, 0, 0, 90'));
        return;
    }

    const numbers = parts.map((value) => Number.parseFloat(value));
    if (numbers.some((num) => Number.isNaN(num))) {
        window.alert(t('The input contains non-numeric values. Please try again.'));
        return;
    }

    const [dxCm, dyCm, dzCm, rollDeg, pitchDeg, yawDeg] = numbers;

    try {
        const updatedOffset = appState.visualizer.setEnvironmentObjectOffset(id, {
            position: {
                x: dxCm * CM_TO_M,
                y: dyCm * CM_TO_M,
                z: dzCm * CM_TO_M
            },
            rotation: {
                x: rollDeg,
                y: pitchDeg,
                z: yawDeg
            }
        });

        entry.offset = {
            position: { ...updatedOffset.position },
            rotation: { ...updatedOffset.rotation }
        };

        if (typeof appState.visualizer.getEnvironmentObjectWorldPosition === 'function') {
            const worldPos = appState.visualizer.getEnvironmentObjectWorldPosition(id);
            if (worldPos) {
                entry.position = { x: worldPos.x, y: worldPos.y, z: worldPos.z };
            }
        }

        renderEnvironmentObjectList();
    } catch (error) {
        console.error('调整环境物体偏移失败:', error);
        window.alert(t('Adjustment failed: {{message}}', { message: error.message }));
    }
}

/**
 * 更新环境物体附着
 * @param {string} id
 * @param {string} targetValue
 */
export function updateEnvironmentAttachment(id, targetValue) {
    const entry = appState.environmentObjects.find((item) => item.id === id);
    if (!entry) {
        return;
    }

    const normalized = targetValue === 'none' ? null : targetValue;

    try {
        if (!appState.visualizer) {
            throw new Error(t('Visualizer is not initialized'));
        }

        if (!normalized) {
            if (typeof appState.visualizer.detachEnvironmentObject === 'function') {
                appState.visualizer.detachEnvironmentObject(id);
            }
            entry.attachedTo = null;
        } else if (typeof appState.visualizer.attachEnvironmentObject === 'function') {
            appState.visualizer.attachEnvironmentObject(id, { target: normalized });
            entry.attachedTo = normalized;
        } else {
            throw new Error(t('The visualizer does not implement environment attachment'));
        }

        if (typeof appState.visualizer.getEnvironmentObjectWorldPosition === 'function') {
            const worldPos = appState.visualizer.getEnvironmentObjectWorldPosition(id);
            if (worldPos) {
                entry.position = { x: worldPos.x, y: worldPos.y, z: worldPos.z };
            }
        }
        if (typeof appState.visualizer.getEnvironmentObjectOffset === 'function') {
            const offset = appState.visualizer.getEnvironmentObjectOffset(id);
            if (offset) {
                entry.offset = offset;
            }
        }
    } catch (error) {
        console.error('更新环境物体附着失败:', error);
        window.alert(t('Attachment update failed: {{message}}', { message: error.message }));
    }

    renderEnvironmentObjectList();
}

/**
 * 添加默认环境箱体到场景
 * @returns {Object} mesh
 */
export function addEnvironmentBoxToScene() {
    if (!appState.visualizer || typeof appState.visualizer.addEnvironmentBox !== 'function') {
        throw new Error(t('Unable to add environment objects because the visualizer is not initialized'));
    }

    appState.environmentObjectCounter += 1;

    const dimsCm = DEFAULT_ENVIRONMENT_BOX_CM;
    const dims = {
        width: Number(dimsCm.length || 0) * CM_TO_M, // X 方向
        depth: Number(dimsCm.width || 0) * CM_TO_M,  // Y 方向
        height: Number(dimsCm.height || 0) * CM_TO_M // Z 方向
    };

    const spacing = Math.max(dims.width, dims.depth) + 0.15;
    const position = {
        x: ENVIRONMENT_BOX_BASE_OFFSET_X + (appState.environmentObjectCounter - 1) * spacing,
        y: 0,
        z: dims.height / 2
    };

    const id = `env-box-${appState.environmentObjectCounter}`;
    const mesh = appState.visualizer.addEnvironmentBox({
        id,
        width: dims.width,
        depth: dims.depth,
        height: dims.height,
        color: DEFAULT_ENVIRONMENT_BOX_COLOR,
        opacity: 0.92,
        position
    });

    if (!mesh) {
        appState.environmentObjectCounter -= 1;
        throw new Error(t('Failed to add environment object because the visualizer returned no mesh'));
    }

    appState.environmentObjects.push({
        id,
        type: 'box',
        displayNameNumber: appState.environmentObjectCounter,
        dimensionsCm: { ...dimsCm },
        position,
        attachedTo: null,
        offset: {
            position: { x: 0, y: 0, z: 0 },
            rotation: { x: 0, y: 0, z: 0 }
        }
    });

    renderEnvironmentObjectList();
    return mesh;
}

/**
 * 移除环境物体
 * @param {string} id
 * @returns {boolean}
 */
export function removeEnvironmentObjectEntry(id) {
    const index = appState.environmentObjects.findIndex((item) => item.id === id);
    if (index === -1) {
        return false;
    }

    if (appState.visualizer && typeof appState.visualizer.removeEnvironmentObject === 'function') {
        appState.visualizer.removeEnvironmentObject(id);
    }

    appState.environmentObjects.splice(index, 1);
    renderEnvironmentObjectList();
    return true;
}

// ══════════════════════════════════════════════
//  场景预设管理
// ══════════════════════════════════════════════

/**
 * 可用的场景预设列表
 */
const SCENE_PRESETS = [
    { id: 'none', label: 'No Scene', description: 'Clear the current scene preset' },
    {
        id: 'warehouse',
        label: 'Warehouse Robot Environment',
        description: 'A typical warehouse scene with shelves, crates, pallets, and navigation markings',
        builder: createWarehouseEnvironment,
        info: getWarehouseInfo,
    },
];

/**
 * 加载指定场景预设
 * @param {string} presetId
 */
export function loadScenePreset(presetId) {
    if (!appState.visualizer) {
        console.warn('可视化器未初始化，无法加载场景预设');
        return;
    }

    // 清除当前预设
    if (presetId === 'none' || !presetId) {
        appState.visualizer.clearScenePreset();
        appState.currentScenePreset = null;
        updateScenePresetUI();
        return;
    }

    const preset = SCENE_PRESETS.find((p) => p.id === presetId);
    if (!preset || !preset.builder) {
        console.warn(`未知的场景预设: ${presetId}`);
        return;
    }

    try {
        const sceneGroup = preset.builder();
        appState.visualizer.loadScenePreset(presetId, sceneGroup);
        appState.currentScenePreset = presetId;
        updateScenePresetUI();
        console.log(`✓ 场景预设 "${preset.label}" 加载完成`);
    } catch (error) {
        console.error('加载场景预设失败:', error);
        window.alert(t('Failed to load scene preset: {{message}}', { message: error.message }));
    }
}

/**
 * 清除场景预设
 */
export function clearScenePreset() {
    loadScenePreset('none');
}

/**
 * 更新场景预设 UI 状态
 */
function updateScenePresetUI() {
    const select = document.getElementById('scene-preset-select');
    if (select) {
        select.value = appState.currentScenePreset || 'none';
    }

    const infoEl = document.getElementById('scene-preset-info');
    if (infoEl) {
        const preset = SCENE_PRESETS.find((p) => p.id === appState.currentScenePreset);
        if (preset && preset.info) {
            const info = preset.info();
            infoEl.innerHTML = `
                <div class="scene-preset-active">
                    <strong>${t(info.name)}</strong>
                    <span class="scene-preset-dims">${info.dimensions}</span>
                    <ul class="scene-preset-elements">
                        ${info.elements.map((e) => `<li>${t(e)}</li>`).join('')}
                    </ul>
                </div>
            `;
            infoEl.style.display = 'block';
        } else {
            infoEl.innerHTML = '';
            infoEl.style.display = 'none';
        }
    }
}

/**
 * 初始化场景预设下拉菜单事件
 */
export function setupScenePresetListeners() {
    const select = document.getElementById('scene-preset-select');
    if (select) {
        select.innerHTML = '';
        SCENE_PRESETS.forEach((preset) => {
            const opt = document.createElement('option');
            opt.value = preset.id;
            opt.textContent = t(preset.label);
            select.appendChild(opt);
        });
        select.value = appState.currentScenePreset || 'none';

        if (!select.dataset.environmentPanelBound) {
            select.addEventListener('change', (e) => {
                loadScenePreset(e.target.value);
            });
            select.dataset.environmentPanelBound = 'true';
        }
    }
}

/**
 * 注册环境面板回调到全局 callbacks
 */
export function registerEnvironmentPanelCallbacks() {
    callbacks.renderEnvironmentObjectList = renderEnvironmentObjectList;
    callbacks.addEnvironmentBoxToScene = addEnvironmentBoxToScene;
    callbacks.loadScenePreset = loadScenePreset;
    callbacks.clearScenePreset = clearScenePreset;

    if (!environmentPanelLanguageBound) {
        onLanguageChange(() => {
            setupScenePresetListeners();
            updateScenePresetUI();
            renderEnvironmentObjectList();
        });
        environmentPanelLanguageBound = true;
    }
}
