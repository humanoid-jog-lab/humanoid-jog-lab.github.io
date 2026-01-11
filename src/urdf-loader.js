/**
 * URDF 拖拽加载器
 * 
 * 支持拖拽整个 URDF 文件夹到浏览器
 * 参考: https://github.com/jurmy24/mechaverse
 */

import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import yaml from 'js-yaml';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

/**
 * URDF 包管理器
 */
export class URDFPackageManager {
    constructor() {
        this.files = new Map(); // 存储所有文件
        this.packages = {}; // 包路径映射
        this.config = null; // YAML 配置
    }
    
    /**
     * 从 FileList 或拖拽事件添加文件
     * @param {FileList|DataTransferItemList} items
     */
    async addFiles(items) {
        const files = [];
        
        // 处理拖拽的文件和文件夹
        if (items instanceof FileList) {
            for (const file of items) {
                files.push(file);
            }
        } else {
            // DataTransferItemList (拖拽)
            for (const item of items) {
                if (item.kind === 'file') {
                    const entry = item.webkitGetAsEntry();
                    if (entry) {
                        await this.traverseFileTree(entry, files);
                    }
                }
            }
        }
        
        // 存储文件
        for (const file of files) {
            this.files.set(file.webkitRelativePath || file.name, file);
        }
        
        console.log(`✓ 已加载 ${this.files.size} 个文件`);
        
        // 查找并解析 YAML 配置
        await this.findAndParseConfig();
        
        // 设置包路径
        this.setupPackagePaths();
    }

    /**
     * 从远程 URL 批量加载文件（用于内置 public/robots 资源的默认加载）
     *
     * @param {Array<{path: string, url: string}>} entries - path 用作包内相对路径（会作为 webkitRelativePath），url 用于 fetch
     */
    async addRemoteFiles(entries) {
        if (!Array.isArray(entries) || entries.length === 0) {
            throw new Error('addRemoteFiles: entries 不能为空');
        }

        const fetchedFiles = await Promise.all(entries.map(async ({ path, url }) => {
            if (!path || !url) {
                throw new Error(`addRemoteFiles: 无效条目: path=${path}, url=${url}`);
            }

            const response = await fetch(url);
            if (!response.ok) {
                throw new Error(`无法加载远程文件: ${url} (HTTP ${response.status})`);
            }

            const blob = await response.blob();
            const filename = (path.split('/').pop() || 'file');

            // 浏览器环境下使用 File；若 File 不存在则退化为 Blob（仍提供 text()/arrayBuffer()）
            let fileLike;
            if (typeof File !== 'undefined') {
                fileLike = new File([blob], filename, { type: blob.type || response.headers.get('content-type') || '' });
            } else {
                fileLike = blob;
                fileLike.name = filename;
            }

            // 关键：让后续逻辑把它当成“拖拽目录结构”里的文件
            try {
                Object.defineProperty(fileLike, 'webkitRelativePath', {
                    value: path,
                    writable: false
                });
            } catch (e) {
                // 某些环境可能不允许 defineProperty（极少见）；退化为普通属性
                fileLike.webkitRelativePath = path;
            }

            return fileLike;
        }));

        for (const file of fetchedFiles) {
            const key = file.webkitRelativePath || file.name;
            this.files.set(key, file);
        }

        console.log(`✓ 已从远程加载 ${this.files.size} 个文件`);

        // 查找并解析 YAML 配置
        await this.findAndParseConfig();

        // 设置包路径
        this.setupPackagePaths();
    }
    
    /**
     * 遍历文件树（处理文件夹）
     */
    async traverseFileTree(entry, files, path = '') {
        if (entry.isFile) {
            const file = await new Promise((resolve) => {
                entry.file(resolve);
            });
            // 设置相对路径
            Object.defineProperty(file, 'webkitRelativePath', {
                value: path + file.name,
                writable: false
            });
            files.push(file);
        } else if (entry.isDirectory) {
            const reader = entry.createReader();
            const entries = await new Promise((resolve) => {
                reader.readEntries(resolve);
            });
            
            for (const childEntry of entries) {
                await this.traverseFileTree(
                    childEntry, 
                    files, 
                    path + entry.name + '/'
                );
            }
        }
    }
    
    /**
     * 查找并解析 robot_config.yaml
     */
    async findAndParseConfig() {
        // 查找 YAML 配置文件
        const configFiles = Array.from(this.files.keys()).filter(name => 
            name.endsWith('.yaml') || name.endsWith('.yml')
        );
        
        if (configFiles.length === 0) {
            console.warn('⚠ 未找到 YAML 配置文件，将使用默认设置');
            return;
        }
        
        // 优先使用 robot_config.yaml
        let configFile = configFiles.find(name => name.includes('robot_config'));
        if (!configFile) {
            configFile = configFiles[0];
        }
        
        console.log(`📄 找到配置文件: ${configFile}`);
        
        const file = this.files.get(configFile);
        const text = await file.text();
        
        try {
            this.config = yaml.load(text);
            console.log('✓ YAML 配置解析成功:', this.config);
        } catch (error) {
            console.error('❌ YAML 解析失败:', error);
        }
    }
    
    /**
     * 设置包路径
     */
    setupPackagePaths() {
        // 自动检测包结构
        const paths = Array.from(this.files.keys());
        
        // 查找 package.xml 或 描述性文件夹
        const packageDirs = new Set();
        
        for (const path of paths) {
            const parts = path.split('/');
            if (parts.length > 1) {
                // 假设第一级目录是包名
                packageDirs.add(parts[0]);
            }
        }
        
        // 设置包映射
        for (const dir of packageDirs) {
            this.packages[dir] = '/uploaded/' + dir;
        }
        
        console.log('📦 检测到的包:', Object.keys(this.packages));
    }
    
    /**
     * 获取文件的 Blob URL
     * @param {string} path
     * @returns {string} Blob URL
     */
    getFileURL(path) {
        const file = this.files.get(path);
        if (!file) {
            console.warn(`文件未找到: ${path}`);
            return null;
        }
        return URL.createObjectURL(file);
    }
    
    /**
     * 查找 URDF 文件
     * @returns {string|null}
     */
    findURDFFile() {
        const urdfFiles = Array.from(this.files.keys()).filter(name => 
            name.endsWith('.urdf') || name.endsWith('.urdf.xacro')
        );
        
        if (urdfFiles.length === 0) {
            return null;
        }
        
        // 优先使用配置中指定的
        if (this.config && this.config.urdf_file) {
            const configUrdf = urdfFiles.find(name => name.includes(this.config.urdf_file));
            if (configUrdf) return configUrdf;
        }
        
        // 否则使用第一个找到的
        return urdfFiles[0];
    }
    
    /**
     * 获取配置信息
     */
    getConfig() {
        return this.config || {
            robot_name: 'Custom Robot',
            base_link: 'base_link',
            end_link: 'end_effector_link',
            joint_names: []
        };
    }
}

/**
 * 自定义 URDF 加载器（支持本地文件）
 */
export class CustomURDFLoader extends URDFLoader {
    constructor(packageManager) {
        super();
        this.packageManager = packageManager;
        this.packages = packageManager.packages;
    }
    
    /**
     * 重写 loadMeshCb 以支持本地文件
     */
    loadMeshCb(path, manager, onComplete) {
        console.log('🔍 加载 mesh:', path);
        
        // 尝试从包管理器获取文件
        let filePath = path;
        
        // 移除 package:// 前缀
        if (filePath.startsWith('package://')) {
            const original = filePath;
            filePath = filePath.replace(/^package:\/\/[^/]+\//, ''); // 移除 package://包名/
            console.log('  📦 移除包前缀:', original, '→', filePath);
        }
        
        // 查找匹配的文件
        console.log('  📂 在', this.packageManager.files.size, '个文件中查找:', filePath);
        const matchingFiles = Array.from(this.packageManager.files.keys()).filter(key => {
            const match = key.endsWith(filePath) || key.includes(filePath);
            if (match) {
                console.log('    ✓ 匹配:', key);
            }
            return match;
        });
        
        console.log('  📊 找到', matchingFiles.length, '个匹配文件');
        
        if (matchingFiles.length > 0) {
            const file = this.packageManager.files.get(matchingFiles[0]);
            const url = URL.createObjectURL(file);
            
            console.log('使用本地文件:', matchingFiles[0]);
            
            // 根据文件类型加载
            if (path.toLowerCase().endsWith('.dae') || path.toLowerCase().endsWith('.collada')) {
                const loader = new ColladaLoader(manager);
                loader.load(
                    url, 
                    (dae) => {
                        URL.revokeObjectURL(url);
                        onComplete(dae.scene);
                    },
                    undefined,
                    (error) => {
                        console.error('DAE 加载失败:', error);
                        URL.revokeObjectURL(url);
                    }
                );
            } else if (path.toLowerCase().endsWith('.stl')) {
                const loader = new STLLoader(manager);
                loader.load(
                    url,
                    (geometry) => {
                        const mesh = new THREE.Mesh(
                            geometry,
                            new THREE.MeshPhongMaterial({ 
                                color: 0xeeeeee,
                                shininess: 30
                            })
                        );
                        URL.revokeObjectURL(url);
                        onComplete(mesh);
                    },
                    undefined,
                    (error) => {
                        console.error('STL 加载失败:', error);
                        URL.revokeObjectURL(url);
                    }
                );
            } else if (path.toLowerCase().endsWith('.obj')) {
                const loader = new OBJLoader(manager);
                loader.load(
                    url,
                    (obj) => {
                        URL.revokeObjectURL(url);
                        onComplete(obj);
                    },
                    undefined,
                    (error) => {
                        console.error('OBJ 加载失败:', error);
                        URL.revokeObjectURL(url);
                    }
                );
            } else {
                console.warn('不支持的文件格式:', path);
                URL.revokeObjectURL(url);
            }
        } else {
            console.warn(`Mesh 文件未找到: ${path}`);
            // 创建占位符
            const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
            const material = new THREE.MeshPhongMaterial({ color: 0xff00ff });
            const mesh = new THREE.Mesh(geometry, material);
            onComplete(mesh);
        }
    }
}

/**
 * 从包管理器加载 URDF
 * @param {URDFPackageManager} packageManager
 * @returns {Promise<Object>} URDF 机器人对象
 */
export async function loadURDFFromPackage(packageManager) {
    const urdfPath = packageManager.findURDFFile();
    
    if (!urdfPath) {
        throw new Error('未找到 URDF 文件。请确保文件夹中包含 .urdf 文件。');
    }
    
    console.log(`📄 加载 URDF: ${urdfPath}`);
    console.log(`📦 可用文件: ${packageManager.files.size} 个`);
    
    const urdfFile = packageManager.files.get(urdfPath);
    const urdfText = await urdfFile.text();
    
    console.log(`📝 URDF 内容长度: ${urdfText.length} 字符`);
    
    // 创建 Blob URL
    const blob = new Blob([urdfText], { type: 'application/xml' });
    const url = URL.createObjectURL(blob);
    
    // 使用自定义加载器
    const loader = new CustomURDFLoader(packageManager);
    
    // 设置默认材质
    loader.defaultMaterial = new THREE.MeshPhongMaterial({
        color: 0xeeeeee,
        shininess: 60
    });
    
    // 关键：设置 packages 映射（让 URDFLoader 知道包的位置）
    // 将所有检测到的包映射到一个虚拟路径
    loader.packages = {};
    const pkgKeys = Object.keys(packageManager.packages);
    
    if (pkgKeys.length > 0) {
        pkgKeys.forEach(pkg => {
            // 使用一个特殊标记，这样我们可以在 loadMeshCb 中识别
            loader.packages[pkg] = `__LOCAL_PACKAGE__/${pkg}`;
            console.log(`📦 注册包: ${pkg} -> __LOCAL_PACKAGE__/${pkg}`);
        });
        
        // 添加常见的包名别名映射（URDF文件中可能使用不同的包名）
        // 例如：robot_arm_description -> ia_robot
        // 如果检测到ia_robot包，同时映射robot_arm_description到它
        if (pkgKeys.includes('ia_robot')) {
            loader.packages['robot_arm_description'] = `__LOCAL_PACKAGE__/ia_robot`;
            console.log(`📦 注册包别名: robot_arm_description -> __LOCAL_PACKAGE__/ia_robot`);
        }
    } else {
        // 如果没有检测到包，使用默认值
        loader.packages['default'] = '__LOCAL_PACKAGE__/default';
        console.log('📦 使用默认包映射');
    }
    
    // 关键：显式设置 loadMeshCb（URDFLoader 需要这样做）
    loader.loadMeshCb = (path, manager, onComplete) => {
        console.log('🔍 加载 mesh:', path);
        
        let filePath = path;
        
        // 移除我们的特殊标记
        if (filePath.includes('__LOCAL_PACKAGE__/')) {
            const original = filePath;
            // 提取包名和实际路径
            const match = filePath.match(/__LOCAL_PACKAGE__\/([^/]+)\/(.+)/);
            if (match) {
                const [, pkgName, relativePath] = match;
                // 如果包名是别名（如robot_arm_description），映射到实际包（如ia_robot）
                const actualPkg = (pkgName === 'robot_arm_description' && pkgKeys.includes('ia_robot')) ? 'ia_robot' : pkgName;
                filePath = actualPkg + '/' + relativePath;
                console.log('  📦 包映射:', pkgName, '→', actualPkg, ', 路径:', relativePath);
            } else {
                filePath = filePath.replace(/__LOCAL_PACKAGE__\/[^/]+\//, ''); // 移除标记和包名
            }
            console.log('  📦 移除包前缀:', original, '→', filePath);
        }
        // 如果还有 package:// 前缀（备用）- 处理包名别名
        else if (filePath.startsWith('package://')) {
            const original = filePath;
            const match = filePath.match(/^package:\/\/([^/]+)\/(.+)/);
            if (match) {
                const [, pkgName, relativePath] = match;
                // 如果包名是别名（如robot_arm_description），映射到实际包（如ia_robot）
                const actualPkg = (pkgName === 'robot_arm_description' && pkgKeys.includes('ia_robot')) ? 'ia_robot' : pkgName;
                filePath = actualPkg + '/' + relativePath;
                console.log('  📦 包别名映射:', pkgName, '→', actualPkg, ', 路径:', relativePath);
            } else {
                filePath = filePath.replace(/^package:\/\/[^/]+\//, '');
            }
            console.log('  📦 处理 package:// 前缀:', original, '→', filePath);
        }
        // 如果是 blob: URL（说明路径解析有问题）
        else if (filePath.startsWith('blob:')) {
            console.error('  ❌ 收到 blob: URL，包映射可能有问题:', filePath);
            // 尝试从 URL 中提取路径
            const match = filePath.match(/meshes\/.*$/);
            if (match) {
                filePath = match[0];
                console.log('  🔧 尝试提取路径:', filePath);
            }
        }
        
        // 查找匹配的文件
        console.log('  📂 在', packageManager.files.size, '个文件中查找:', filePath);
        const matchingFiles = Array.from(packageManager.files.keys()).filter(key => {
            const match = key.endsWith(filePath) || key.includes(filePath);
            if (match) {
                console.log('    ✓ 匹配:', key);
            }
            return match;
        });
        
        console.log('  📊 找到', matchingFiles.length, '个匹配文件');
        
        if (matchingFiles.length > 0) {
            const file = packageManager.files.get(matchingFiles[0]);
            const url = URL.createObjectURL(file);
            
            console.log('  ✓ 使用本地文件:', matchingFiles[0]);
            
            // 根据文件类型加载
            if (path.toLowerCase().endsWith('.dae') || path.toLowerCase().endsWith('.collada')) {
                const colladaLoader = new ColladaLoader(manager);
                colladaLoader.load(
                    url, 
                    (dae) => {
                        URL.revokeObjectURL(url);
                        onComplete(dae.scene);
                    },
                    undefined,
                    (error) => {
                        console.error('  ❌ DAE 加载失败:', error);
                        URL.revokeObjectURL(url);
                    }
                );
            } else if (path.toLowerCase().endsWith('.stl')) {
                const stlLoader = new STLLoader(manager);
                stlLoader.load(
                    url,
                    (geometry) => {
                        const mesh = new THREE.Mesh(
                            geometry,
                            new THREE.MeshPhongMaterial({ 
                                color: 0xeeeeee,
                                shininess: 30
                            })
                        );
                        URL.revokeObjectURL(url);
                        onComplete(mesh);
                    },
                    undefined,
                    (error) => {
                        console.error('  ❌ STL 加载失败:', error);
                        URL.revokeObjectURL(url);
                    }
                );
            } else if (path.toLowerCase().endsWith('.obj')) {
                const objLoader = new OBJLoader(manager);
                objLoader.load(
                    url,
                    (obj) => {
                        URL.revokeObjectURL(url);
                        onComplete(obj);
                    },
                    undefined,
                    (error) => {
                        console.error('  ❌ OBJ 加载失败:', error);
                        URL.revokeObjectURL(url);
                    }
                );
            } else {
                console.warn('  ⚠️ 不支持的文件格式:', path);
                URL.revokeObjectURL(url);
            }
        } else {
            console.warn(`  ❌ Mesh 文件未找到: ${path}`);
            console.log('  可用文件列表:');
            Array.from(packageManager.files.keys()).slice(0, 10).forEach(key => {
                console.log('    -', key);
            });
            // 创建占位符
            const geometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
            const material = new THREE.MeshPhongMaterial({ color: 0xff00ff });
            const mesh = new THREE.Mesh(geometry, material);
            onComplete(mesh);
        }
    };
    
    return new Promise((resolve, reject) => {
        loader.load(
            url,
            (robot) => {
                console.log('✓ URDF 结构加载成功');
                console.log('  - 关节数:', Object.keys(robot.joints).length);
                console.log('  - 连杆数:', Object.keys(robot.links).length);
                URL.revokeObjectURL(url);
                resolve(robot);
            },
            (progress) => {
                // 修复: progress 可能为 null 或 undefined
                if (progress && progress.lengthComputable) {
                    const percent = (progress.loaded / progress.total * 100).toFixed(0);
                    console.log(`  加载进度: ${percent}%`);
                }
            },
            (error) => {
                console.error('❌ URDF 加载失败:', error);
                console.error('错误详情:', error);
                URL.revokeObjectURL(url);
                reject(error);
            }
        );
    });
}

