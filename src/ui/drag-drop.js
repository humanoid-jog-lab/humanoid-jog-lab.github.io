/**
 * 拖拽加载模块
 * 
 * 处理全局拖拽文件加载 URDF 的功能。
 */

import { appState, callbacks } from '../core/state.js';
import { URDFPackageManager, loadURDFFromPackage } from '../urdf-loader.js';

// ═══════════════════════════════════════════
//  Global Drag & Drop
// ═══════════════════════════════════════════

/**
 * 设置拖拽区域
 */
function setupGlobalDragAndDrop() {
    appState.dropOverlayElement = document.getElementById('drop-overlay');
    if (!appState.dropOverlayElement) {
        return;
    }

    let dragCounter = 0;

    const isFileDrag = (event) => {
        const types = event.dataTransfer?.types;
        if (!types) {
            return false;
        }
        if (typeof types.includes === 'function') {
            return types.includes('Files');
        }
        return Array.from(types).indexOf('Files') >= 0;
    };

    window.addEventListener('dragenter', (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        event.preventDefault();
        dragCounter += 1;
        updateDropOverlay('ready', '释放以加载 URDF 文件夹', '📁');
    });

    window.addEventListener('dragover', (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        event.preventDefault();
    });

    window.addEventListener('dragleave', (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        dragCounter = Math.max(0, dragCounter - 1);
        if (dragCounter === 0) {
            hideDropOverlay();
        }
    });

    window.addEventListener('drop', async (event) => {
        if (!isFileDrag(event)) {
            return;
        }
        event.preventDefault();
        dragCounter = 0;

        const transfer = event.dataTransfer;
        const items = transfer?.items?.length ? transfer.items : transfer?.files;
        if (!items || !items.length) {
            hideDropOverlay();
            return;
        }

        updateDropOverlay('loading', '正在加载 URDF 资源...', '⏳');
        await handleDrop(items);
    });
}

function updateDropOverlay(state, message, icon) {
    appState.dropOverlayElement = appState.dropOverlayElement || document.getElementById('drop-overlay');
    if (!appState.dropOverlayElement) {
        return;
    }

    if (appState.dropOverlayHideTimer) {
        clearTimeout(appState.dropOverlayHideTimer);
        appState.dropOverlayHideTimer = null;
    }

    const stateClasses = ['drop-ready', 'drop-loading', 'drop-success', 'drop-error'];
    const iconFallback = {
        ready: '📁',
        loading: '⏳',
        success: '✅',
        error: '⚠️'
    };
    const textFallback = {
        ready: '释放以加载 URDF 文件夹',
        loading: '正在加载 URDF 资源...',
        success: '加载完成',
        error: '加载失败'
    };

    appState.dropOverlayElement.classList.remove('hidden');
    appState.dropOverlayElement.classList.add('visible');
    stateClasses.forEach((cls) => appState.dropOverlayElement.classList.remove(cls));
    appState.dropOverlayElement.classList.add(`drop-${state}`);

    const iconElement = appState.dropOverlayElement.querySelector('.drop-overlay-icon');
    const textElement = appState.dropOverlayElement.querySelector('.drop-overlay-text');

    if (iconElement) {
        iconElement.textContent = icon || iconFallback[state] || '📁';
    }
    if (textElement) {
        textElement.textContent = message || textFallback[state] || '';
    }
}

function hideDropOverlay(delay = 0) {
    appState.dropOverlayElement = appState.dropOverlayElement || document.getElementById('drop-overlay');
    if (!appState.dropOverlayElement) {
        return;
    }

    if (appState.dropOverlayHideTimer) {
        clearTimeout(appState.dropOverlayHideTimer);
        appState.dropOverlayHideTimer = null;
    }

    appState.dropOverlayHideTimer = setTimeout(() => {
        appState.dropOverlayElement.classList.remove('visible', 'drop-ready', 'drop-loading', 'drop-success', 'drop-error');
        appState.dropOverlayElement.classList.add('hidden');
        appState.dropOverlayHideTimer = null;
    }, delay);
}

/**
 * 处理拖拽的文件
 */
async function handleDrop(items) {
    console.log('═══════════════════════════════════════════');
    console.log('       开始拖拽加载');
    console.log('═══════════════════════════════════════════\n');
    
    try {
        // 在加载新模型前，先清理所有旧模型
        if (appState.visualizer && typeof appState.visualizer.cleanupOldRobots === 'function') {
            console.log('0️⃣ 清理旧模型...');
            appState.visualizer.cleanupOldRobots();
            console.log('✓ 旧模型已清理');
        }
        
        console.log('1️⃣ 创建包管理器...');
        appState.packageManager = new URDFPackageManager();
        await appState.packageManager.addFiles(items);
        console.log(`✓ 已添加 ${appState.packageManager.files.size} 个文件`);
        
        console.log('\n2️⃣ 加载 URDF...');
        const robot = await loadURDFFromPackage(appState.packageManager);
        console.log('✓ URDF 对象创建成功');
        
        console.log('\n3️⃣ 读取配置...');
        appState.customRobotConfig = appState.packageManager.getConfig();
        if (!appState.customRobotConfig) {
            throw new Error('未找到 robot_config.yaml 配置文件');
        }
        console.log('✓ 配置:', appState.customRobotConfig.robot_name);
        const isHumanoid = appState.customRobotConfig?.humanoid === true;
        if (isHumanoid) {
            console.log('\n4️⃣ 人形模式：按 link 名提取腰/左臂/右臂/头...');
            if (callbacks.loadHumanoidSystem) {
                await callbacks.loadHumanoidSystem(robot, appState.customRobotConfig);
            }
            if (callbacks.switchToHumanoidUI) {
                callbacks.switchToHumanoidUI();
            }
            console.log('\n6️⃣ 同步界面状态...');
            updateDropOverlay('success', `✓ 已加载人形: ${appState.customRobotConfig.robot_name}`, '✅');
            hideDropOverlay(1400);
            if (callbacks.refreshDockState) {
                callbacks.refreshDockState();
            }
            console.log('\n✅ 全部完成！');
            console.log('═══════════════════════════════════════════\n');
            return;
        }
        console.log('  基座:', appState.customRobotConfig.joint_chain?.base_link);
        console.log('  末端:', appState.customRobotConfig.joint_chain?.end_link);
        console.log('  关节数:', appState.customRobotConfig.joint_chain?.joints?.length || 0);
        
        const isDualArm = appState.customRobotConfig?.dual_arm === true;
        appState.dualArmMode = isDualArm;
        
        if (isDualArm) {
            console.log('\n4️⃣ 双臂模式：加载两个机器人...');
            const jointControlsContainer = document.getElementById('joint-controls');
            if (jointControlsContainer) {
                jointControlsContainer.innerHTML = '';
            }
            
            if (callbacks.loadDualArmSystem) {
                await callbacks.loadDualArmSystem(robot, appState.customRobotConfig);
            }
            if (callbacks.switchToDualArmUI) {
                callbacks.switchToDualArmUI();
            }
            if (callbacks.createDualArmJointControls) {
                callbacks.createDualArmJointControls(appState.customRobotConfig);
            }
        } else {
            console.log('\n4️⃣ 加载到 3D 场景...');
            if (!appState.visualizer) {
                throw new Error('可视化器未初始化');
            }
            const loaded = await appState.visualizer.loadCustomURDF(robot, null, appState.customRobotConfig);
            if (!loaded) {
                throw new Error('可视化器加载失败');
            }
            console.log('✓ 3D 模型已添加到场景');
            if (callbacks.switchToSingleArmUI) {
                callbacks.switchToSingleArmUI();
            }
            
            console.log('\n5️⃣ 提取 MDH 参数...');
            if (appState.customRobotConfig.mdh_extraction?.auto_extract) {
                if (callbacks.extractMDHFromCustomRobot) {
                    await callbacks.extractMDHFromCustomRobot(robot, appState.customRobotConfig);
                }
                console.log('✓ MDH 提取并应用成功');
            } else {
                console.log('⊘ MDH 自动提取已禁用');
            }
        }
        
        console.log('\n6️⃣ 同步界面状态...');
        updateDropOverlay('success', `✓ 已加载: ${appState.customRobotConfig.robot_name}`, '✅');
        hideDropOverlay(1400);
        if (callbacks.refreshDockState) {
            callbacks.refreshDockState();
        }
        
        console.log('\n✅ 全部完成！');
        console.log('═══════════════════════════════════════════\n');
    } catch (error) {
        console.error('\n❌ 加载失败:', error);
        console.error('错误堆栈:', error.stack);
        console.log('═══════════════════════════════════════════\n');
        updateDropOverlay('error', '❌ 加载失败，请查看控制台了解详情', '⚠️');
        hideDropOverlay(2200);
        alert(`加载失败:\n\n${error.message}\n\n请打开控制台(F12)查看详细错误信息。`);
    }
}

// ═══════════════════════════════════════════
//  Exports
// ═══════════════════════════════════════════

export {
    setupGlobalDragAndDrop,
    updateDropOverlay,
    hideDropOverlay,
    handleDrop,
};
