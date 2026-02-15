import { CM_TO_M, DEFAULT_ENVIRONMENT_BOX_CM, DEFAULT_ENVIRONMENT_BOX_COLOR, ENVIRONMENT_BOX_BASE_OFFSET_X } from '../core/constants.js';
import { appState, callbacks } from '../core/state.js';
import { createWarehouseEnvironment, getWarehouseInfo } from '../environments/warehouse.js';

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
        empty.textContent = '当前场景没有环境物体';
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
        title.textContent = object.displayName || '环境物体';

        const dimsCm = object.dimensionsCm || {};
        const lengthCm = Number(dimsCm.length ?? 0).toFixed(1);
        const widthCm = Number(dimsCm.width ?? 0).toFixed(1);
        const heightCm = Number(dimsCm.height ?? 0).toFixed(1);

        const dimensions = document.createElement('div');
        dimensions.className = 'env-object-meta';
        dimensions.textContent = `尺寸：${lengthCm} × ${widthCm} × ${heightCm} cm`;

        const posInfo = document.createElement('div');
        posInfo.className = 'env-object-meta';
        const pos = object.position || { x: 0, y: 0, z: 0 };
        posInfo.textContent = `位置 (m)：(${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`;

        const attachStatus = document.createElement('div');
        attachStatus.className = 'env-object-meta';
        attachStatus.textContent = `附着：${getEnvironmentAttachmentLabel(object.attachedTo)}`;

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
        offsetInfo.textContent = `偏移(cm)：(${(offsetPos.x / CM_TO_M).toFixed(1)}, ${(offsetPos.y / CM_TO_M).toFixed(1)}, ${(offsetPos.z / CM_TO_M).toFixed(1)}) ｜ 旋转(°)：(${offsetRot.x.toFixed(1)}, ${offsetRot.y.toFixed(1)}, ${offsetRot.z.toFixed(1)})`;
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
        adjustBtn.textContent = '调整';
        if (!object.attachedTo) {
            adjustBtn.disabled = true;
            adjustBtn.title = '附着后可调整偏移';
        }
        adjustBtn.addEventListener('click', () => {
            promptEnvironmentObjectAdjustment(object.id);
        });
        controls.appendChild(adjustBtn);

        const removeBtn = document.createElement('button');
        removeBtn.className = 'env-remove-btn';
        removeBtn.textContent = '移除';
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
            return '末端小球';
        case 'dual-absolute':
            return '双臂绝对姿态小球';
        default:
            return '未附着';
    }
}

/**
 * 获取环境附着选项
 * @returns {Array}
 */
export function getEnvironmentAttachmentOptions() {
    const options = [
        { value: 'none', label: '未附着', disabled: false }
    ];
    const hasVisualizer = !!appState.visualizer;
    const hasEndEffector = hasVisualizer && typeof appState.visualizer.hasEndEffectorAnchor === 'function'
        ? appState.visualizer.hasEndEffectorAnchor()
        : false;
    options.push({
        value: 'end-effector',
        label: '末端小球',
        disabled: !hasEndEffector
    });
    const hasAbsolute = hasVisualizer && typeof appState.visualizer.hasAbsolutePoseMarker === 'function'
        ? appState.visualizer.hasAbsolutePoseMarker()
        : false;
    if (hasAbsolute) {
        options.push({
            value: 'dual-absolute',
            label: '双臂绝对姿态小球',
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
        window.alert('请先将该物体附着到小球上，再进行调整。');
        return;
    }
    if (!appState.visualizer || typeof appState.visualizer.setEnvironmentObjectOffset !== 'function') {
        window.alert('当前可视化器不支持偏移调整');
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
        '输入偏移与旋转，格式：dx, dy, dz, roll, pitch, yaw\n' +
        'dx/dy/dz 单位为厘米，roll/pitch/yaw 单位为度。\n' +
        '正方向遵循右手定则，旋转顺序为 XYZ。',
        defaultInput
    );
    if (input === null) {
        return;
    }

    const parts = input.split(/[,\s]+/).filter(Boolean);
    if (parts.length !== 6) {
        window.alert('请输入 6 个数值，格式示例：2, 0, 5, 0, 0, 90');
        return;
    }

    const numbers = parts.map((value) => Number.parseFloat(value));
    if (numbers.some((num) => Number.isNaN(num))) {
        window.alert('输入包含非数值内容，请重新输入');
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
        window.alert(`调整失败：${error.message}`);
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
            throw new Error('可视化器尚未初始化');
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
            throw new Error('可视化器未实现环境附着功能');
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
        window.alert(`更新附着失败：${error.message}`);
    }

    renderEnvironmentObjectList();
}

/**
 * 添加默认环境箱体到场景
 * @returns {Object} mesh
 */
export function addEnvironmentBoxToScene() {
    if (!appState.visualizer || typeof appState.visualizer.addEnvironmentBox !== 'function') {
        throw new Error('可视化器尚未初始化，无法添加环境物体');
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
        throw new Error('添加环境物体失败，返回值为空');
    }

    appState.environmentObjects.push({
        id,
        type: 'box',
        displayName: `箱子 #${appState.environmentObjectCounter}`,
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
    { id: 'none', label: '无场景', description: '清空场景预设' },
    {
        id: 'warehouse',
        label: '仓储机器人环境',
        description: '包含货架、货箱、托盘和导航标线的典型仓库场景',
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
        window.alert(`加载场景预设失败：${error.message}`);
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
                    <strong>${info.name}</strong>
                    <span class="scene-preset-dims">${info.dimensions}</span>
                    <ul class="scene-preset-elements">
                        ${info.elements.map((e) => `<li>${e}</li>`).join('')}
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
        // 填充选项
        select.innerHTML = '';
        SCENE_PRESETS.forEach((preset) => {
            const opt = document.createElement('option');
            opt.value = preset.id;
            opt.textContent = preset.label;
            select.appendChild(opt);
        });
        select.value = appState.currentScenePreset || 'none';

        select.addEventListener('change', (e) => {
            loadScenePreset(e.target.value);
        });
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
}
