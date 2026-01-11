/**
 * Electron 主进程
 * 启动前端 UI 和后端 ROS 2 服务
 */

import { app, BrowserWindow, Menu, ipcMain } from 'electron';
import path from 'path';
import { fileURLToPath } from 'url';
import { createROS2Server } from './ros2-backend.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

let mainWindow;
let ros2Server;

/**
 * 创建应用窗口
 */
function createWindow() {
    mainWindow = new BrowserWindow({
        width: 1400,
        height: 900,
        minWidth: 800,
        minHeight: 600,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
            enableRemoteModule: false
        },
        icon: path.join(__dirname, '../public/icon.png')
    });

    // 加载前端
    const isDev = process.env.VITE_DEV_SERVER_URL;
    
    if (isDev) {
        // 开发模式：从 Vite 开发服务器加载
        mainWindow.loadURL(process.env.VITE_DEV_SERVER_URL);
        mainWindow.webDevTools.openDevTools();
        console.log('Electron 以开发模式启动');
    } else {
        // 生产模式：从打包的文件加载
        const indexPath = path.join(__dirname, '../dist/index.html');
        mainWindow.loadFile(indexPath);
        console.log('Electron 以生产模式启动');
    }

    mainWindow.on('closed', () => {
        mainWindow = null;
    });
}

/**
 * Electron 应用启动
 */
app.on('ready', async () => {
    console.log('\nElectron 应用启动中...');
    
    // 1. 启动后端 ROS 2 服务
    console.log('正在启动 ROS 2 后端服务...');
    ros2Server = createROS2Server(8081);
    ros2Server.httpServer.listen(8081, () => {
        console.log('✓ ROS 2 后端已启动 (端口 8081)');
    });

    // 2. 创建窗口
    createWindow();

    // 3. 创建菜单
    createMenu();
});

/**
 * 所有窗口关闭时退出
 */
app.on('window-all-closed', () => {
    // 关闭后端服务
    if (ros2Server?.httpServer) {
        console.log('\n正在关闭 ROS 2 后端...');
        ros2Server.httpServer.close(() => {
            console.log('✓ ROS 2 后端已关闭');
            if (process.platform !== 'darwin') {
                app.quit();
            }
        });
    } else {
        if (process.platform !== 'darwin') {
            app.quit();
        }
    }
});

/**
 * 在 macOS 上，应用在被激活时继续运行
 */
app.on('activate', () => {
    if (mainWindow === null) {
        createWindow();
    }
});

/**
 * 创建应用菜单
 */
function createMenu() {
    const template = [
        {
            label: '文件',
            submenu: [
                {
                    label: '退出',
                    accelerator: 'CmdOrCtrl+Q',
                    click: () => {
                        app.quit();
                    }
                }
            ]
        },
        {
            label: '编辑',
            submenu: [
                { label: '撤销', role: 'undo' },
                { label: '重做', role: 'redo' },
                { type: 'separator' },
                { label: '剪切', role: 'cut' },
                { label: '复制', role: 'copy' },
                { label: '粘贴', role: 'paste' }
            ]
        },
        {
            label: '查看',
            submenu: [
                { label: '重新加载', role: 'reload' },
                { label: '强制重新加载', role: 'forceReload' },
                { label: '切换开发者工具', role: 'toggleDevTools' }
            ]
        },
        {
            label: '帮助',
            submenu: [
                {
                    label: '关于应用',
                    click: () => {
                        // 可以添加关于对话框
                        console.log('DQ Robotics Vizlab v1.0.0');
                    }
                }
            ]
        }
    ];

    // macOS 特殊处理
    if (process.platform === 'darwin') {
        template.unshift({
            label: app.name,
            submenu: [
                { role: 'about' },
                { type: 'separator' },
                { role: 'services' },
                { type: 'separator' },
                { role: 'hide' },
                { role: 'hideOthers' },
                { role: 'unhide' },
                { type: 'separator' },
                { role: 'quit' }
            ]
        });
    }

    const menu = Menu.buildFromTemplate(template);
    Menu.setApplicationMenu(menu);
}

/**
 * IPC 通讯处理（可选）
 * 前端可以通过 ipcRenderer 与主进程通讯
 */
ipcMain.handle('get-app-version', () => {
    return app.getVersion();
});

ipcMain.handle('get-app-path', () => {
    return app.getAppPath();
});

export { mainWindow, ros2Server };

