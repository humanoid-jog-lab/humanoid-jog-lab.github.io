import { appState, callbacks } from '../core/state.js';

/**
 * 使面板可拖动
 * @param {HTMLElement} panel
 */
export function makePanelDraggable(panel) {
    const header = panel.querySelector('.panel-header');
    if (!header) return;
    
    let isDragging = false;
    let currentX;
    let currentY;
    let initialX;
    let initialY;
    let xOffset = 0;
    let yOffset = 0;
    
    // 获取当前面板的位置
    const rect = panel.getBoundingClientRect();
    xOffset = rect.left;
    yOffset = rect.top;
    
    header.addEventListener('mousedown', dragStart);
    document.addEventListener('mousemove', drag);
    document.addEventListener('mouseup', dragEnd);
    
    function dragStart(e) {
        // 如果点击的是按钮，不触发拖动
        if (e.target.classList.contains('panel-toggle')) {
            return;
        }
        
        initialX = e.clientX - xOffset;
        initialY = e.clientY - yOffset;
        
        isDragging = true;
        header.style.cursor = 'grabbing';
        bringPanelToFront(panel);
    }
    
    function drag(e) {
        if (isDragging) {
            e.preventDefault();
            
            currentX = e.clientX - initialX;
            currentY = e.clientY - initialY;
            
            xOffset = currentX;
            yOffset = currentY;
            
            // 限制在视口内
            const maxX = window.innerWidth - panel.offsetWidth;
            const maxY = window.innerHeight - panel.offsetHeight;
            
            xOffset = Math.max(0, Math.min(xOffset, maxX));
            yOffset = Math.max(0, Math.min(yOffset, maxY));
            
            setTranslate(xOffset, yOffset, panel);
        }
    }
    
    function dragEnd() {
        if (isDragging) {
            initialX = currentX;
            initialY = currentY;
            isDragging = false;
            header.style.cursor = 'grab';
        }
    }
    
    function setTranslate(xPos, yPos, el) {
        el.style.left = xPos + 'px';
        el.style.top = yPos + 'px';
        el.style.right = 'auto';
        el.style.bottom = 'auto';
    }
}

/**
 * 将面板置于最前
 * @param {HTMLElement} panel
 */
export function bringPanelToFront(panel) {
    if (!panel) {
        return;
    }
    appState.panelStackIndex += 1;
    panel.style.zIndex = appState.panelStackIndex;
}

/**
 * 初始化底部 Dock
 */
export function initializeDock() {
    appState.dockElement = document.getElementById('bottom-dock');
    if (!appState.dockElement) {
        return;
    }

    const dockItems = appState.dockElement.querySelectorAll('.dock-item');
    dockItems.forEach((item) => {
        const panelId = item.dataset.panel;
        if (!panelId) {
            return;
        }

        item.addEventListener('click', () => {
            togglePanelVisibility(panelId);
        });
    });
}

/**
 * 刷新 Dock 状态
 */
export function refreshDockState() {
    if (!appState.dockElement) {
        appState.dockElement = document.getElementById('bottom-dock');
        if (!appState.dockElement) {
            return;
        }
    }

    const dockItems = appState.dockElement.querySelectorAll('.dock-item');
    dockItems.forEach((item) => {
        const panelId = item.dataset.panel;
        const panel = document.getElementById(panelId);
        const isActive = !!panel && !panel.classList.contains('hidden');
        item.classList.toggle('active', isActive);
    });
}

/**
 * 更新单个 Dock 项状态
 * @param {string} panelId
 * @param {boolean} isActive
 */
export function updateDockItemState(panelId, isActive) {
    if (!appState.dockElement) {
        appState.dockElement = document.getElementById('bottom-dock');
        if (!appState.dockElement) {
            return;
        }
    }
    const item = appState.dockElement.querySelector(`.dock-item[data-panel="${panelId}"]`);
    if (!item) {
        return;
    }
    item.classList.toggle('active', isActive);
}

/**
 * 切换面板可见性
 * @param {string} panelId
 */
export function togglePanelVisibility(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) {
        return;
    }
    if (panel.classList.contains('hidden')) {
        showPanel(panelId);
    } else {
        hidePanel(panelId);
    }
}

/**
 * 显示面板
 * @param {string} panelId
 */
export function showPanel(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) {
        return;
    }
    panel.classList.remove('hidden');
    panel.classList.remove('collapsed');
    const toggle = panel.querySelector('.panel-toggle');
    if (toggle) {
        toggle.textContent = '−';
    }
    bringPanelToFront(panel);
    updateDockItemState(panelId, true);
}

/**
 * 隐藏面板
 * @param {string} panelId
 */
export function hidePanel(panelId) {
    const panel = document.getElementById(panelId);
    if (!panel) {
        return;
    }
    if (!panel.classList.contains('hidden')) {
        panel.classList.add('hidden');
    }
    const toggle = panel.querySelector('.panel-toggle');
    if (toggle) {
        toggle.textContent = '+';
    }
    updateDockItemState(panelId, false);
}

/**
 * 初始化浮动面板系统
 */
export function setupFloatingPanels() {
    const panels = document.querySelectorAll('.floating-panel');
    panels.forEach((panel) => {
        makePanelDraggable(panel);
        panel.addEventListener('mousedown', () => bringPanelToFront(panel));
    });

    document.querySelectorAll('.panel-toggle').forEach((toggle) => {
        const panel = toggle.closest('.floating-panel');
        if (!panel) {
            return;
        }
        toggle.addEventListener('click', (event) => {
            event.stopPropagation();
            hidePanel(panel.id);
        });
    });

    // ROS 2 通讯事件处理
    callbacks.setupROS2EventListeners?.();
    callbacks.setupInverseKinematicsControls?.();
    callbacks.setupDQCalculatorPanel?.();
    
    initializeDock();
    refreshDockState();
}

/**
 * 注册面板管理回调到全局 callbacks
 */
export function registerPanelManagerCallbacks() {
    callbacks.setupFloatingPanels = setupFloatingPanels;
    callbacks.refreshDockState = refreshDockState;
    callbacks.showPanel = showPanel;
    callbacks.hidePanel = hidePanel;
}
