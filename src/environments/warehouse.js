/**
 * 仓储机器人环境构建器
 *
 * 生成一个典型仓储场景，包含：
 *   - 地面（带导航标线）
 *   - 多排货架（shelving racks）
 *   - 货架上的货箱
 *   - 托盘
 *   - 柱子 / 墙壁轮廓
 *
 * 所有尺寸单位：米（m），坐标系：Z-up
 */

import * as THREE from 'three';

// ── 颜色常量 ────────────────────────────────────────────
const COLOR = {
    floor: 0x4a4a52,
    floorLine: 0xf0c040,
    shelf: 0x8b8b8b,
    shelfPost: 0x666666,
    crate: 0xd4893f,
    crateAlt: 0x3a8fd4,
    crateGreen: 0x4caf50,
    pallet: 0xc8a96e,
    wall: 0x5c5c6a,
    pillar: 0x6e6e7a,
    label: 0xffffff,
};

// ── 仓库参数 ────────────────────────────────────────────
const WAREHOUSE = {
    // 地面
    floorWidth: 12,       // X 方向
    floorDepth: 8,        // Y 方向
    floorThickness: 0.02,

    // 货架
    shelfRows: 3,         // 几排货架
    shelfCols: 4,         // 每排几组
    shelfWidth: 1.2,      // 单组货架 X 宽度
    shelfDepth: 0.5,      // 货架 Y 深度
    shelfHeight: 2.0,     // 货架总高度
    shelfLevels: 3,       // 层板数（含底层）
    shelfPostRadius: 0.025,
    shelfBoardThickness: 0.03,
    shelfRowSpacing: 2.5, // 排与排之间 Y 间距
    shelfColSpacing: 1.4, // 列与列之间 X 间距

    // 货箱
    crateMinSize: 0.15,
    crateMaxSize: 0.35,

    // 托盘
    palletWidth: 0.8,
    palletDepth: 0.6,
    palletHeight: 0.12,

    // 地面标线
    lineWidth: 0.08,

    // 墙壁
    wallHeight: 3.0,
    wallThickness: 0.12,
};

/**
 * 创建仓储环境场景
 * @returns {THREE.Group} 包含所有仓储元素的组
 */
export function createWarehouseEnvironment() {
    const group = new THREE.Group();
    group.name = 'WarehouseEnvironment';

    // 1. 地面
    const floor = createFloor();
    group.add(floor);

    // 2. 地面导航标线
    const floorLines = createFloorLines();
    group.add(floorLines);

    // 3. 货架
    const shelves = createShelves();
    group.add(shelves);

    // 4. 随机放一些货箱到货架上
    const crates = createCratesOnShelves(shelves);
    group.add(crates);

    // 5. 地面上的托盘
    const pallets = createPallets();
    group.add(pallets);

    // 6. 墙壁轮廓 / 柱子
    const walls = createWalls();
    group.add(walls);

    return group;
}

// ── 地面 ─────────────────────────────────────────────────

function createFloor() {
    const { floorWidth, floorDepth, floorThickness } = WAREHOUSE;
    const geo = new THREE.BoxGeometry(floorWidth, floorDepth, floorThickness);
    const mat = new THREE.MeshStandardMaterial({
        color: COLOR.floor,
        roughness: 0.9,
        metalness: 0.05,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(0, 0, -floorThickness / 2); // 顶面齐 z=0
    mesh.receiveShadow = true;
    mesh.name = 'warehouse-floor';
    return mesh;
}

// ── 地面标线 ─────────────────────────────────────────────

function createFloorLines() {
    const group = new THREE.Group();
    group.name = 'warehouse-floor-lines';

    const { floorWidth, floorDepth, lineWidth } = WAREHOUSE;
    const mat = new THREE.MeshBasicMaterial({ color: COLOR.floorLine });
    const Z = 0.002; // 略高于地面

    // 中央纵向主通道线（两条平行线）
    const aisleHalfGap = 0.6;
    for (const sign of [-1, 1]) {
        const geo = new THREE.PlaneGeometry(lineWidth, floorDepth * 0.85);
        const line = new THREE.Mesh(geo, mat);
        line.position.set(sign * aisleHalfGap, 0, Z);
        group.add(line);
    }

    // 横向通道线（分隔区域）
    const rowCount = 3;
    for (let i = 0; i < rowCount; i++) {
        const y = -floorDepth * 0.35 + i * (floorDepth * 0.7 / (rowCount - 1));
        const geo = new THREE.PlaneGeometry(floorWidth * 0.85, lineWidth);
        const line = new THREE.Mesh(geo, mat);
        line.position.set(0, y, Z);
        group.add(line);
    }

    // 虚线箭头指示（简化为短横线段）
    const dashMat = new THREE.MeshBasicMaterial({ color: COLOR.floorLine, transparent: true, opacity: 0.6 });
    for (let j = 0; j < 8; j++) {
        const geo = new THREE.PlaneGeometry(0.25, lineWidth * 0.6);
        const dash = new THREE.Mesh(geo, dashMat);
        dash.position.set(0, -floorDepth * 0.35 + j * 0.6, Z);
        group.add(dash);
    }

    return group;
}

// ── 货架 ─────────────────────────────────────────────────

function createShelves() {
    const group = new THREE.Group();
    group.name = 'warehouse-shelves';

    const {
        shelfRows, shelfCols,
        shelfWidth, shelfDepth, shelfHeight, shelfLevels,
        shelfPostRadius, shelfBoardThickness,
        shelfRowSpacing, shelfColSpacing,
    } = WAREHOUSE;

    // 货架排列在通道两侧
    const startX = -(shelfCols * shelfColSpacing) / 2 + shelfColSpacing / 2;
    const startY = 1.2; // 第一排 Y 偏移

    for (let row = 0; row < shelfRows; row++) {
        const ySign = row < 2 ? 1 : -1; // 前两排在 +Y 侧，第三排在 -Y 侧
        const rowIdx = row < 2 ? row : 0;
        const baseY = ySign * (startY + rowIdx * shelfRowSpacing);

        for (let col = 0; col < shelfCols; col++) {
            const x = startX + col * shelfColSpacing;
            const shelf = createSingleShelf(
                shelfWidth, shelfDepth, shelfHeight,
                shelfLevels, shelfPostRadius, shelfBoardThickness
            );
            shelf.position.set(x, baseY, 0);
            shelf.userData = { shelfRow: row, shelfCol: col };
            group.add(shelf);
        }
    }

    return group;
}

/**
 * 创建单组货架
 */
function createSingleShelf(width, depth, height, levels, postRadius, boardThickness) {
    const shelf = new THREE.Group();
    shelf.name = 'shelf-unit';

    const postMat = new THREE.MeshStandardMaterial({
        color: COLOR.shelfPost,
        roughness: 0.6,
        metalness: 0.3,
    });
    const boardMat = new THREE.MeshStandardMaterial({
        color: COLOR.shelf,
        roughness: 0.7,
        metalness: 0.1,
    });

    // 四根立柱
    const postGeo = new THREE.CylinderGeometry(postRadius, postRadius, height, 8);
    // CylinderGeometry 默认沿 Y 轴，需旋转到 Z 轴方向 (Z-up)
    const positions = [
        [-width / 2, -depth / 2],
        [width / 2, -depth / 2],
        [-width / 2, depth / 2],
        [width / 2, depth / 2],
    ];
    for (const [px, py] of positions) {
        const post = new THREE.Mesh(postGeo, postMat);
        post.rotation.x = Math.PI / 2; // Y-up → Z-up
        post.position.set(px, py, height / 2);
        post.castShadow = true;
        shelf.add(post);
    }

    // 层板
    const boardGeo = new THREE.BoxGeometry(width, depth, boardThickness);
    for (let l = 0; l < levels; l++) {
        const z = (l / (levels - 1)) * (height - boardThickness) + boardThickness / 2;
        const board = new THREE.Mesh(boardGeo, boardMat);
        board.position.set(0, 0, z);
        board.receiveShadow = true;
        board.castShadow = true;
        board.userData = { shelfLevel: l, shelfBoardZ: z + boardThickness / 2 };
        shelf.add(board);
    }

    return shelf;
}

// ── 货架上的货箱 ─────────────────────────────────────────

function createCratesOnShelves(shelvesGroup) {
    const group = new THREE.Group();
    group.name = 'warehouse-crates';

    const { crateMinSize, crateMaxSize, shelfBoardThickness } = WAREHOUSE;
    const colors = [COLOR.crate, COLOR.crateAlt, COLOR.crateGreen];

    // 确定性随机种子（可重复）
    let seed = 42;
    const rand = () => {
        seed = (seed * 16807 + 0) % 2147483647;
        return (seed - 1) / 2147483646;
    };

    shelvesGroup.children.forEach((shelfUnit) => {
        // 取层板
        shelfUnit.children.forEach((child) => {
            if (!child.userData?.shelfBoardZ) return;
            const boardZ = child.userData.shelfBoardZ;

            // 在每层板上随机放 0-3 个箱子
            const count = Math.floor(rand() * 3.5);
            for (let c = 0; c < count; c++) {
                const w = crateMinSize + rand() * (crateMaxSize - crateMinSize);
                const d = crateMinSize + rand() * (crateMaxSize - crateMinSize);
                const h = crateMinSize + rand() * (crateMaxSize - crateMinSize) * 0.8;
                const color = colors[Math.floor(rand() * colors.length)];

                const geo = new THREE.BoxGeometry(w, d, h);
                const mat = new THREE.MeshStandardMaterial({
                    color,
                    roughness: 0.7,
                    metalness: 0.05,
                    transparent: true,
                    opacity: 0.92,
                });
                const crate = new THREE.Mesh(geo, mat);
                crate.castShadow = true;

                // 在层板范围内随机偏移
                const halfW = (WAREHOUSE.shelfWidth - w) / 2 * 0.8;
                const halfD = (WAREHOUSE.shelfDepth - d) / 2 * 0.8;
                const offsetX = -halfW + rand() * halfW * 2;
                const offsetY = -halfD + rand() * halfD * 2;

                // 位置 = 货架世界位置 + 层板 z + 箱子半高
                crate.position.set(
                    shelfUnit.position.x + offsetX,
                    shelfUnit.position.y + offsetY,
                    boardZ + h / 2
                );
                group.add(crate);
            }
        });
    });

    return group;
}

// ── 地面托盘 ─────────────────────────────────────────────

function createPallets() {
    const group = new THREE.Group();
    group.name = 'warehouse-pallets';

    const { palletWidth, palletDepth, palletHeight } = WAREHOUSE;
    const palletMat = new THREE.MeshStandardMaterial({
        color: COLOR.pallet,
        roughness: 0.85,
        metalness: 0.02,
    });

    // 放几个托盘在通道旁边
    const positions = [
        { x: -3.5, y: -2.0, rot: 0 },
        { x: -3.5, y: -2.8, rot: 0 },
        { x: 3.0, y: -2.5, rot: Math.PI / 6 },
        { x: 4.0, y: 0.0, rot: 0 },
    ];

    positions.forEach(({ x, y, rot }) => {
        const pallet = createSinglePallet(palletWidth, palletDepth, palletHeight, palletMat);
        pallet.position.set(x, y, 0);
        pallet.rotation.z = rot;
        group.add(pallet);
    });

    return group;
}

function createSinglePallet(w, d, h, mat) {
    const pallet = new THREE.Group();
    pallet.name = 'pallet';

    // 底板
    const boardH = h * 0.25;
    const topGeo = new THREE.BoxGeometry(w, d, boardH);
    const top = new THREE.Mesh(topGeo, mat);
    top.position.z = h - boardH / 2;
    top.castShadow = true;
    top.receiveShadow = true;
    pallet.add(top);

    // 底部横木（3 根）
    const blockW = w * 0.15;
    const blockH = h * 0.65;
    const blockGeo = new THREE.BoxGeometry(blockW, d * 0.9, blockH);
    for (let i = -1; i <= 1; i++) {
        const block = new THREE.Mesh(blockGeo, mat);
        block.position.set(i * (w * 0.35), 0, blockH / 2);
        block.castShadow = true;
        pallet.add(block);
    }

    return pallet;
}

// ── 墙壁 / 柱子 ─────────────────────────────────────────

function createWalls() {
    const group = new THREE.Group();
    group.name = 'warehouse-walls';

    const { floorWidth, floorDepth, wallHeight, wallThickness } = WAREHOUSE;
    const wallMat = new THREE.MeshStandardMaterial({
        color: COLOR.wall,
        roughness: 0.85,
        metalness: 0.05,
        transparent: true,
        opacity: 0.35,
    });

    const halfW = floorWidth / 2;
    const halfD = floorDepth / 2;

    // 三面墙壁（留一面开口作为出入口）
    const wallDefs = [
        // 后墙 (+Y)
        { w: floorWidth, d: wallThickness, x: 0, y: halfD, rz: 0 },
        // 左墙 (-X)
        { w: floorDepth, d: wallThickness, x: -halfW, y: 0, rz: Math.PI / 2 },
        // 右墙 (+X)
        { w: floorDepth, d: wallThickness, x: halfW, y: 0, rz: Math.PI / 2 },
    ];

    wallDefs.forEach(({ w, d, x, y, rz }) => {
        const geo = new THREE.BoxGeometry(w, d, wallHeight);
        const wall = new THREE.Mesh(geo, wallMat);
        wall.position.set(x, y, wallHeight / 2);
        wall.rotation.z = rz;
        wall.receiveShadow = true;
        group.add(wall);
    });

    // 柱子
    const pillarMat = new THREE.MeshStandardMaterial({
        color: COLOR.pillar,
        roughness: 0.6,
        metalness: 0.2,
    });
    const pillarR = 0.1;
    const pillarGeo = new THREE.CylinderGeometry(pillarR, pillarR, wallHeight, 12);

    const pillarPositions = [
        [-halfW + 0.5, -halfD + 0.5],
        [halfW - 0.5, -halfD + 0.5],
        [-halfW + 0.5, halfD - 0.5],
        [halfW - 0.5, halfD - 0.5],
    ];
    pillarPositions.forEach(([px, py]) => {
        const pillar = new THREE.Mesh(pillarGeo, pillarMat);
        pillar.rotation.x = Math.PI / 2;
        pillar.position.set(px, py, wallHeight / 2);
        pillar.castShadow = true;
        group.add(pillar);
    });

    return group;
}

/**
 * 获取仓储环境的描述信息
 * @returns {Object}
 */
export function getWarehouseInfo() {
    return {
        id: 'warehouse',
        name: 'Warehouse Robot Environment',
        description: 'A typical warehouse scene with shelves, crates, pallets, and navigation markings',
        dimensions: `${WAREHOUSE.floorWidth}m × ${WAREHOUSE.floorDepth}m`,
        elements: [
            `${WAREHOUSE.shelfRows}×${WAREHOUSE.shelfCols} shelf array`,
            `${WAREHOUSE.shelfLevels} shelf levels`,
            'Randomly distributed crates',
            'Floor navigation markings',
            'Pallets',
            'Walls and pillars',
        ],
    };
}
