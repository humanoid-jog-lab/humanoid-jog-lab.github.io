export const LANGUAGE_STORAGE_KEY = 'ui-language';
export const DEFAULT_LANGUAGE = 'en';

const SUPPORTED_LANGUAGES = new Set(['en', 'zh']);

let currentLanguage = DEFAULT_LANGUAGE;

const listeners = new Set();

const zhMessages = {
    'DQ Robotics Visualizer': 'DQ Robotics 可视化',
    'Loading WASM module...': '正在加载 WASM 模块...',
    'Control Panel': '控制面板',
    'Language': '语言',
    'Loading failed': '加载失败',
    'Please make sure the WASM module has been built correctly.': '请确保已正确构建 WASM 模块',
    'Joint Controls': '关节控制',
    'Import from Clipboard': '从剪贴板导入',
    'Copy Joint Angles': '复制关节角度',
    'Copy Joint Limits': '复制关节限位',
    'Switch to Radians': '切换为弧度',
    'Switch to Degrees': '切换为角度',
    'Top View': '俯视视角',
    'Inverse Kinematics': '逆运动学求解',
    'Target Dual Quaternion (8 numbers, space/comma separated):': '目标双四元数（8个数，空格/逗号分隔）：',
    'Solve Inverse Kinematics': '求解逆运动学',
    'Copy Current Dual Quaternion': '复制当前双四元数',
    'Update current robot joints with Jacobian pseudoinverse': '使用雅可比伪逆更新当前机器人角度',
    'Frames & Control': '坐标系与控制',
    'Show URDF Frames': '显示URDF坐标系',
    'End-Effector Control': '末端执行器控制',
    'Control Mode:': '控制模式：',
    'Absolute Pose': '绝对位姿',
    'Whole-Body Absolute Pose': '全身绝对位姿',
    'Whole-Body Absolute Pose (Upright Waist)': '全身绝对位姿（腰部竖直）',
    'Waist body_link X/Z': '腰部 body_link X/Z',
    'Relative Pose': '相对位姿',
    'Whole-Body Relative Pose': '全身相对位姿',
    'Relative Distance': '相对距离',
    'Whole-Body Relative Distance': '全身相对距离',
    'Left Ruled-Surface Polishing': '左臂擦面',
    'Cooperative Surface Polishing': '双臂协同擦面',
    'Cooperative Surface Polishing (DQ Set-point, Waist Maintained)': '双臂协同擦面（DQ目标，腰部维持）',
    'Cooperative Surface Polishing (DQ Set-point, Arms Only)': '双臂协同擦面（DQ目标，无腰部）',
    'Left Arm': '左臂',
    'Left Arm + Waist': '左臂+腰部',
    'Right Arm': '右臂',
    'Right Arm + Waist': '右臂+腰部',
    'Absolute Pose Mode: control the absolute pose while keeping the relative pose unchanged': '绝对位姿模式：控制绝对位姿，保持相对位姿不变',
    'Joint Limit Avoidance': '关节限位避免',
    'Optimize in the null space to avoid joint limits': '在零空间中优化，避免关节到达极限',
    'Jog Solver:': '点动求解器：',
    'Null-space Projection': '零空间投影',
    'Restore (LS)': 'Restore (LS)',
    'Recovery Optimization (TR)': '恢复优化(TR)',
    'Recovery Optimization (TR+w)': '恢复优化(TR+w)',
    'Recovery Optimization (TR+w_vfi)': '恢复优化(TR+w_vfi)',
    'ProxQP+w': 'ProxQP+w',
    'ProxQP+w_vfi': 'ProxQP+w_vfi',
    'MuJoCo-style Recovery + w Pre-optimization': 'MuJoCo 风格恢复 + w预优化',
    'Applies to humanoid cooperative jogging mode and can be switched anytime.': '对人形协同点动模式生效，可随时切换。',
    'Speed Percentage:': '速度百分比：',
    'w Metric:': 'w 指标：',
    'Only standard, extended, and VFI manipulability are kept.': '仅保留普通、扩展与 VFI 可操作度。',
    'Waist X Constraint': '腰部 X 约束',
    'Constrain the waist X position to remain below the current corridor limit': '约束腰部 X 位置不超过当前走廊上界',
    'x_min (m):': 'x_min (m)：',
    'x_max (m):': 'x_max (m)：',
    'Constrain the waist X position to stay inside the adjustable corridor': '约束腰部 X 位置落在可调走廊区间内',
    'Dual-Arm Test Lock': '双臂测试锁',
    'Temporarily limit ljoint2∈[-0.1,0.2], rjoint2∈[-0.2,0.1]': '临时限制 ljoint2∈[-0.1,0.2], rjoint2∈[-0.2,0.1]',
    'Waist X corridor enabled (x∈[{{min}}, {{max}}]m)': '腰部 X 走廊已启用（x∈[{{min}}, {{max}}]m）',
    'Waist X corridor disabled.': '腰部 X 走廊已关闭。',
    'Waist X corridor lower bound updated (x_min={{min}}m)': '腰部 X 走廊下界已更新（x_min={{min}}m）',
    'Waist X corridor lower bound set to x_min={{min}}m.': '腰部 X 走廊下界已设置为 x_min={{min}}m。',
    'Waist X corridor upper bound updated (x_max={{max}}m)': '腰部 X 走廊上界已更新（x_max={{max}}m）',
    'Waist X corridor upper bound set to x_max={{max}}m.': '腰部 X 走廊上界已设置为 x_max={{max}}m。',
    'Waist X corridor VFI updated ({{detail}}).': '腰部 X 走廊 VFI 已更新（{{detail}}）。',
    'Dual-arm test lock enabled (ljoint2∈[-0.1,0.2], rjoint2∈[-0.2,0.1]).': '双臂测试锁已启用（ljoint2∈[-0.1,0.2], rjoint2∈[-0.2,0.1]）。',
    'Dual-arm test lock disabled.': '双臂测试锁已关闭。',
    'Joint-limit VFI η_θ updated to {{value}}.': '关节极限 VFI η_θ 已更新为 {{value}}。',
    'Safety Push Weight β updated to {{value}}.': '安全推离权重 β 已更新为 {{value}}。',
    'Invalid input; restored to η_θ={{value}}': '输入无效，已恢复为 η_θ={{value}}',
    'Value applied: η_θ={{value}}': '当前值已应用：η_θ={{value}}',
    'Invalid input; restored to β={{value}}': '输入无效，已恢复为 β={{value}}',
    'Value applied: β={{value}}': '当前值已应用：β={{value}}',
    'Invalid input; restored to joint={{joint}}rad, waist={{waist}}m': '输入无效，已恢复为 关节={{joint}}rad，腰部={{waist}}m',
    'Value applied: joint={{joint}}rad, waist={{waist}}m': '当前值已应用：关节={{joint}}rad，腰部={{waist}}m',
    'Invalid input; restored to η_x={{value}}': '输入无效，已恢复为 η_x={{value}}',
    'Value applied: η_x={{value}}': '当前值已应用：η_x={{value}}',
    'Invalid input; restored to arms({{armVel}}/{{armAcc}}) waist({{waistVel}}/{{waistAcc}})': '输入无效，已恢复为 双臂({{armVel}}/{{armAcc}}) 腰部({{waistVel}}/{{waistAcc}})',
    'Value applied: arm vel/acc={{armVel}}/{{armAcc}}, waist vel/acc={{waistVel}}/{{waistAcc}}': '当前值已应用：双臂 vel/acc={{armVel}}/{{armAcc}}，腰部 vel/acc={{waistVel}}/{{waistAcc}}',
    'Null-space Safety Refine thresholds updated: joint={{joint}}rad, waist={{waist}}m.': '零空间 Safety Refine 阈值已更新：关节={{joint}}rad，腰部={{waist}}m。',
    'Default Joint Angles:': '默认关节角：',
    'Default 1': '默认1',
    'Default 2': '默认2',
    'Switch the humanoid default startup pose and optionally apply it immediately to the current humanoid model': '切换 humanoid 全局默认启动姿态，并可立即应用到当前人形模型',
    'Cooperative Polishing Surface Sample:': '协同擦面示例曲面：',
    'Analytic Surface': '解析曲面',
    'Cylinder Mesh': '圆筒 Mesh',
    'Only affects polishing modes and switches between the analytic ruled surface and cylinder': '仅影响擦面模式，可在解析 ruled surface 与圆筒之间切换',
    'Object Pose': '物体位姿',
    'Flip Normal': '法向反转',
    'Apply Pose': '应用位姿',
    'Capture Current Hands': '捕捉当前双手',
    'Export Printable OBJ': '导出打印 OBJ',
    'Fix the polishing workpiece in world coordinates, or capture the current two-hand workspace once and freeze it': '将协同擦面工件固定在世界坐标中，或从当前双手工作区捕捉一次后冻结',
    'Real-time Constraint Diagnostics (including CCTSM §IV)': '实时约束诊断（含 CCTSM §IV）',
    'Waiting for jog commands...': '等待点动命令...',
    'Local Increment (Δ × Current)': '局部增量（Δ × 当前）',
    'Current Mode: local frame (current × Δ)': '当前模式：局部坐标（当前 × Δ）',
    'Current Mode: world frame (Δ × current)': '当前模式：世界坐标（Δ × 当前）',
    'Speed Percentage': '速度百分比',
    'Affects the incremental step size without changing key repeat frequency': '影响每步增量（不改变按键重复频率）',
    'X Axis': 'X轴',
    'Y Axis': 'Y轴',
    'Z Axis': 'Z轴',
    'Distance': '距离',
    'Robot Status': '机器人状态',
    'Battery Status': '电池状态',
    'Charge:': '电量:',
    'Voltage:': '电压:',
    'Current:': '电流:',
    'Status:': '状态:',
    'End-Effector Pose': '末端执行器位姿',
    'Dual-Arm Robot Pose': '双臂机器人位姿',
    'Jacobian Matrix': '雅可比矩阵',
    'DQ Representation': 'DQ 表示',
    'Position (m)': '位置 (m)',
    'Rotation Quaternion': '旋转四元数',
    'Euler Angles (deg)': '欧拉角 (deg)',
    'Primary': '实部',
    'Dual': '对偶部',
    'Absolute Pose Position (m)': '绝对位姿位置 (m)',
    'Absolute Pose Quaternion': '绝对位姿四元数',
    'Absolute Pose DQ': '绝对位姿 DQ',
    'Relative Pose Position (m)': '相对位姿位置 (m)',
    'Distance:': '距离:',
    'Relative Pose Quaternion': '相对位姿四元数',
    'Relative Pose DQ': '相对位姿 DQ',
    'Environment': '环境管理',
    'Scene Presets': '场景预设',
    'Choose a preset environment scene to provide a simulated workspace for the robot.': '选择预设环境场景，为机器人提供仿真工作空间。',
    'No Scene': '无场景',
    'Warehouse Robot Environment': '仓储机器人环境',
    'Environment Objects': '环境物体',
    'Manage additional scene objects; add boxes and attach them to the end-effector marker or the dual-arm absolute-pose marker.': '管理场景中的附加物体，可添加箱子并附着到末端小球或双臂绝对姿态小球。',
    'Add Box (41.5 × 30.5 × 14 cm)': '添加箱子（41.5 × 30.5 × 14 cm）',
    'No environment objects in the current scene': '当前场景没有环境物体',
    'DQ Calculator': 'DQ 计算器',
    'Dual Quaternion Input': '双四元数输入',
    'Robot': '机器人',
    'Load Current Pose': '加载当前姿态',
    'Load Current Quaternion': '加载当前四元数',
    'Load Current Position': '加载当前位置',
    'Copy Input': '复制输入',
    'Create DQ from Quaternion and Translation': '从四元数和平移创建DQ',
    'Quaternion (w, x, y, z):': '四元数 (w, x, y, z)：',
    'Position (x, y, z):': '位置 (x, y, z)：',
    'Compute Dual Quaternion': '计算对偶四元数',
    'Create a dual quaternion from quaternion and translation': '从四元数和平移创建对偶四元数',
    'Translation Operation': '平移操作',
    'Left Multiply': '左乘',
    'Right Multiply': '右乘',
    'Compute New Dual Quaternion': '计算新双四元数',
    'Result Output': '结果输出',
    'Results will appear here': '结果将在此显示',
    'Copy Result': '复制结果',
    'Use left/right multiplication with translation to compose the current dual quaternion': '使用左乘/右乘与平移组合当前双四元数',
    'Humanoid: left/right arm pose with relative/absolute/distance control': '人形：左臂/右臂位姿与相对/绝对/距离控制',
    'Left Arm Position (m)': '左臂位置 (m)',
    'Left Arm Quaternion': '左臂四元数',
    'Left Arm DQ': '左臂 DQ',
    'Right Arm Position (m)': '右臂位置 (m)',
    'Right Arm Quaternion': '右臂四元数',
    'Right Arm DQ': '右臂 DQ',
    'Left Arm Mode: control the left end-effector': '左臂模式：控制左臂末端',
    'Left Arm + Waist Mode: control the left end-effector with waist cooperation while keeping the right end-effector fixed': '左臂+腰部模式：控制左臂末端并允许腰部协同，保持右臂末端不变',
    'Right Arm Mode: control the right end-effector': '右臂模式：控制右臂末端',
    'Right Arm + Waist Mode: control the right end-effector with waist cooperation while keeping the left end-effector fixed': '右臂+腰部模式：控制右臂末端并允许腰部协同，保持左臂末端不变',
    'Left Ruled-Surface Polishing: control u, v, and n while keeping the right arm and waist fixed': '左臂擦面：控制 u、v 和 n，同时保持右臂与腰部固定',
    'Cooperative Surface Polishing: share forward u, center vc, and normal n, while Δv controls the polishing bandwidth': '双臂协同擦面：共享 u 前进、共享中心 vc、共享法向 n，并用 Δv 控制左右擦拭带宽',
    'Cooperative Surface Polishing (DQ Set-point, Waist Maintained): use the same shared strip parameters and true dual-quaternion pose targets while keeping waist participation and body-z maintenance': '双臂协同擦面（DQ目标，腰部维持）：使用共享擦拭带参数和真实双四元数位姿目标，同时保留腰部参与与 body-z 维持',
    'Cooperative Surface Polishing (DQ Set-point, Arms Only): use the same shared strip parameters and true dual-quaternion pose targets without waist participation': '双臂协同擦面（DQ目标，无腰部）：使用共享擦拭带参数和真实双四元数位姿目标，不让腰部参与',
    'Relative Pose Mode: control the relative pose while keeping the absolute pose unchanged': '相对位姿模式：控制相对位姿，保持绝对位姿不变',
    'Relative Distance Mode: control the distance between the two end-effectors while keeping absolute pose and relative orientation unchanged': '相对距离模式：控制两末端距离，保持绝对位姿与相对姿态不变',
    'Whole-Body Relative Distance: coordinate waist and both arms to control the end-effector distance while keeping absolute pose and relative orientation unchanged': '全身相对距离：腰部+双臂联动，控制两末端距离，保持绝对位姿与相对姿态不变',
    'Whole-Body Absolute Pose Mode: coordinate waist and both arms to control the absolute pose while keeping the relative pose unchanged': '全身绝对位姿：腰部+双臂联动，控制绝对位姿，保持相对位姿不变',
    'Whole-Body Absolute Pose Mode (Upright Waist): coordinate waist and both arms to control the absolute pose while keeping the body_link z-axis upright': '全身绝对位姿（腰部竖直）：腰部+双臂联动，控制绝对位姿，同时保持 body_link 的 z 轴竖直地面',
    'Waist body_link Jog Mode: control only the body_link X/Z translation': '腰部 body_link 点动：仅控制 body_link 的 X/Z 平移',
    'Whole-Body Relative Pose Mode: coordinate waist and both arms to control the relative pose while keeping the absolute pose unchanged': '全身相对位姿：腰部+双臂联动，控制相对位姿，保持绝对位姿不变',
    'Robot 1 Mode: control robot 1 end-effector': '机器人1模式：控制机器人1的末端执行器',
    'Robot 2 Mode: control robot 2 end-effector': '机器人2模式：控制机器人2的末端执行器',
    'Robot 1 Position (m)': '机器人1位置 (m)',
    'Robot 1 Quaternion': '机器人1四元数',
    'Robot 1 DQ': '机器人1 DQ',
    'Robot 2 Position (m)': '机器人2位置 (m)',
    'Robot 2 Quaternion': '机器人2四元数',
    'Robot 2 DQ': '机器人2 DQ',
    'Teleoperation Mode - controlled via VR devices': '遥操作模式 - 通过VR设备控制',
    'Unknown Mode': '未知模式',
    'Current Mode:': '当前模式：',
    'Humanoid mode: left arm / right arm / absolute pose / relative pose': '人形模式：左臂/右臂/绝对位姿/相对位姿',
    'Dual-arm mode: choose robot 1 / robot 2 / absolute pose / relative pose': '双臂模式：可选择机器人1/机器人2/绝对位姿/相对位姿',
    'Single-arm mode: only control robot 1': '单臂模式：只控制机器人1',
    'Single-arm mode: update current robot joints with Jacobian pseudoinverse': '单臂模式：使用雅可比伪逆更新当前机器人角度',
    'Humanoid absolute pose: solve the absolute pose while keeping the relative pose unchanged': '人形绝对位姿：求解绝对位姿，保持相对位姿不变',
    'Dual-arm absolute pose: solve the absolute pose while keeping the relative pose unchanged': '双臂绝对位姿：求解绝对位姿，保持相对位姿不变',
    'Humanoid relative pose: solve the relative pose while keeping the absolute pose unchanged': '人形相对位姿：求解相对位姿，保持绝对位姿不变',
    'Dual-arm relative pose: solve the relative pose while keeping the absolute pose unchanged': '双臂相对位姿：求解相对位姿，保持绝对位姿不变',
    'Humanoid single-arm solve: update only the left or right arm joints': '人形单臂求解：仅更新左臂或右臂角度',
    'Dual-arm single-arm solve: update only robot 1 or robot 2 joints': '双臂单臂求解：仅更新机器人1或机器人2角度',
    'Please enter 8 dual quaternion elements (comma or space separated)': '请输入 8 个双四元数元素（逗号或空格分隔）',
    'Visualizer is not initialized; inverse kinematics unavailable': '视觉器未初始化，无法求解逆运动学',
    'Humanoid joints are not initialized': '人形关节未初始化',
    'The current inverse-kinematics solve supports only robot 1 / robot 2': '当前逆运动学求解仅支持机器人1/机器人2',
    'Inverse kinematics failed: target accuracy not reached': '逆运动学求解失败：未达到目标精度',
    'Absolute Pose (keep relative pose unchanged)': '绝对位姿（保持相对不变）',
    'Relative Pose (keep absolute pose unchanged)': '相对位姿（保持绝对不变）',
    'Inverse kinematics succeeded; updated {{label}}': '逆运动学求解成功，已更新 {{label}}',
    'Inverse kinematics failed: {{message}}': '逆运动学失败：{{message}}',
    'Unable to get the current pose dual quaternion': '无法获取当前姿态的双四元数',
    'Copied dual quaternion: {{value}}': '已复制双四元数：{{value}}',
    'Copied the current pose dual quaternion': '已复制当前姿态的双四元数',
    'Failed to copy dual quaternion: {{message}}': '复制双四元数失败：{{message}}',
    'Copy failed: {{message}}': '复制失败：{{message}}',
    'Loaded current pose dual quaternion': '已加载当前姿态的双四元数',
    'Input dual quaternion': '输入双四元数',
    'Output dual quaternion': '输出双四元数',
    'Please enter 8 dual quaternion elements': '请输入 8 个双四元数元素',
    'DQ module is not loaded; cannot calculate': 'DQ 模块未加载，无法计算',
    'Calculation complete, result ready to copy': '计算完成，可复制结果',
    'Calculation failed: {{message}}': '计算失败：{{message}}',
    'Quaternion cannot be zero': '四元数不能为零',
    'Calculation complete, verification passed (translation matches)': '计算完成，验证通过（平移一致）',
    'Calculation complete, but translation verification failed (difference: {{difference}})': '计算完成，但平移验证失败（差异: {{difference}}）',
    'Calculation complete, but verification failed': '计算完成，但验证失败',
    'Nothing to copy for {{label}}': '没有可复制的{{label}}',
    'Copied {{label}}': '已复制{{label}}',
    'Top View Set': '已切换俯视',
    'Switch Failed': '切换失败',
    'Clipboard is empty; cannot import joint angles': '剪贴板为空，无法导入关节角度',
    'Paste joint angle array (supports direct paste from copied output):': '粘贴关节角度数组（支持直接粘贴复制的输出）：',
    'Joint angle import complete': '导入成功',
    'Joint angle import failed': '导入失败',
    'Failed to import joint angles: {{message}}': '导入关节角度失败：{{message}}',
    'Environment box added': '已添加箱子',
    'Add failed': '添加失败',
    'Failed to add environment box: {{message}}': '添加环境箱体失败：{{message}}',
    'Copied!': '已复制！',
    'Copy failed': '复制失败',
    'Current Mode: world frame (Δ × current)': '当前模式：世界坐标（Δ × 当前）',
    'See the console for the rotation DQ output': '查看控制台以查看旋转 DQ 输出',
    'See the console for the translation DQ output': '查看控制台以查看平移 DQ 输出',
    'See the console for the pose DQ output': '查看控制台以查看位姿 DQ 输出',
    '⚠ URDF model is not loaded\n\nPlease make sure the URDF model has loaded successfully.': '⚠ URDF 模型未加载\n\n请确保 URDF 模型已成功加载。',
    '✓ MDH parameters extracted successfully!\n\nPlease open the console (F12) to view the details.': '✓ MDH 参数提取成功！\n\n请打开控制台 (F12) 查看详细参数。',
    '❌ Extraction failed\n\n{{message}}': '❌ 提取失败\n\n{{message}}',
    '⚠ URDF model is not loaded': '⚠ URDF 模型未加载',
    '✓ Parameters match!\n\nThe MDH parameters extracted from the URDF exactly match the official reference.\n\nSee the console for details.': '✓ 参数一致！\n\nURDF 提取的 MDH 参数与官方参考参数完全一致。\n\n查看控制台了解详情。',
    '⚠ Differences found\n\n{{count}} parameters differ.\n\nPossible reasons:\n- Different frame conventions\n- Different zero-position definitions\n- Different parameter sources\n\nSee the console for details.': '⚠ 发现差异\n\n在 {{count}} 个参数上存在差异。\n\n这可能是由于：\n- 坐标系约定不同\n- 零位定义不同\n- 参数来源不同\n\n查看控制台了解详情。',
    '❌ Comparison failed\n\n{{message}}': '❌ 对比失败\n\n{{message}}',
    '⚠ Please extract the MDH parameters first.\n\nClick "Extract MDH from URDF".': '⚠ 请先提取 MDH 参数\n\n点击 "从URDF提取MDH" 按钮。',
    '✓ Switch successful!\n\nThe kinematics now use the MDH parameters extracted from the URDF.\n\nThe yellow marker should align exactly with the URDF end-effector.': '✓ 切换成功！\n\n现在使用从 URDF 提取的 MDH 参数进行运动学计算。\n\n黄色球应该与 URDF 末端完全对齐！',
    '❌ Switch failed\n\n{{message}}': '❌ 切换失败\n\n{{message}}',
    'ROS 2 Communication': 'ROS 2 通讯',
    'ROS 2 Communication & Control': 'ROS 2 通讯和控制',
    'Connection': '连接管理',
    'Connection Status': '连接状态',
    'Status': '状态',
    'Disconnected': '未连接',
    'Connect': '连接',
    'Disconnect': '断开',
    'Service Address': '服务地址设置',
    'WebSocket URL:': 'WebSocket 地址：',
    'Save': '保存',
    'Save URL': '保存地址',
    'Tip: reconnect after changing the address': '提示：修改地址后需要重新连接',
    'Operation Mode': '操作模式',
    'Manual Mode': '手动模式',
    'Teleoperation Mode': '遥操作模式',
    'Confirm': '确定',
    'Current Mode: Manual Mode - direct joint control available': '当前模式：手动模式 - 可直接控制机器人关节',
    'Real Joint State': '真机关节状态',
    'JointState Topic:': 'JointState 话题：',
    'Subscribe': '订阅',
    'Humanoid mode matches by joint name; wheel joints are subscribed but do not overwrite the UI pose.': '人形模式按 joint name 匹配；轮子关节只订阅不写入界面姿态',
    'Not subscribed to real joint state': '未订阅真机关节状态',
    'Joint Position Publishing (RobotArmServoMsg)': '关节位置发布 (RobotArmServoMsg)',
    'Joint Position Publishing (Robotarmjoint)': '关节位置发布 (Robotarmjoint)',
    'Topic Name:': '话题名称：',
    'Continuous Joint Position Publishing': '持续发布关节位置',
    'Switch is off; no messages published': '开关关闭，未发布消息',
    'Waist Position Publishing (RobotServoMsg)': '腰部位置发布 (RobotServoMsg)',
    'Waist Position Publishing (Robotlegjoint)': '腰部位置发布 (Robotlegjoint)',
    'Continuous Waist Joint Publishing': '持续发布腰部关节',
    'Message Log': '消息日志',
    'No messages yet': '暂无消息',
    'Info Panel': '信息面板',
    'Parameters': '参数面板',
    'Humanoid VFI': '人形 VFI',
    'Joint-Limit VFI η_θ': '关节极限 VFI η_θ',
    'Corridor VFI η_x': '走廊 VFI η_x',
    'Value applied': '当前值已应用',
    'Safety Push Weight β': '安全推离权重 β',
    'Refine Joint Activation Distance (rad)': 'Refine 关节激活距离(rad)',
    'Refine Waist Activation Distance (m)': 'Refine 腰部激活距离(m)',
    'ROS 2 Publish Defaults': 'ROS 2 发布默认值',
    'Arm vel': '双臂 vel',
    'Arm acc': '双臂 acc',
    'Waist vel': '腰部 vel',
    'Waist acc': '腰部 acc',
    'ROS 2': 'ROS 2 通讯',
    'Control Panel Dock': '控制面板',
    'Environment Dock': '环境管理',
    'DQ Calculator Dock': 'DQ 计算器',
    'ROS 2 Dock': 'ROS 2 通讯',
    'Drag a URDF folder anywhere on the page to load it': '拖拽 URDF 文件夹到页面任意位置以加载',
    'Connecting...': '连接中...',
    'Connected': '已连接',
    'Error': '错误',
    'Unknown': '未知',
    'Charging': '充电中',
    'Discharging': '放电中',
    'Not Charging': '未充电',
    'Full': '已充满',
    'Manual Mode - direct joint control available': '手动模式 - 可直接控制机器人关节',
    'Adjust': '调整',
    'Adjust offset after attaching': '附着后可调整偏移',
    'Adjustment failed: {{message}}': '调整失败：{{message}}',
    'Attachment update failed: {{message}}': '更新附着失败：{{message}}',
    'Attachment: {{label}}': '附着：{{label}}',
    'Box #{{number}}': '箱子 #{{number}}',
    'End-Effector Marker': '末端小球',
    'Dual-Arm Absolute-Pose Marker': '双臂绝对姿态小球',
    'Environment Object': '环境物体',
    'Size: {{length}} × {{width}} × {{height}} cm': '尺寸：{{length}} × {{width}} × {{height}} cm',
    'Position (m): ({{x}}, {{y}}, {{z}})': '位置 (m)：({{x}}, {{y}}, {{z}})',
    'Offset (cm): ({{x}}, {{y}}, {{z}}) | Rotation (deg): ({{roll}}, {{pitch}}, {{yaw}})': '偏移(cm)：({{x}}, {{y}}, {{z}}) ｜ 旋转(°)：({{roll}}, {{pitch}}, {{yaw}})',
    'Not Attached': '未附着',
    'Please attach this object to a marker before adjusting it.': '请先将该物体附着到小球上，再进行调整。',
    'The current visualizer does not support offset adjustment': '当前可视化器不支持偏移调整',
    'Enter offset and rotation as: dx, dy, dz, roll, pitch, yaw\ndx/dy/dz are in centimeters, roll/pitch/yaw are in degrees.\nPositive directions follow the right-hand rule with XYZ rotation order.': '输入偏移与旋转，格式：dx, dy, dz, roll, pitch, yaw\ndx/dy/dz 单位为厘米，roll/pitch/yaw 单位为度。\n正方向遵循右手定则，旋转顺序为 XYZ。',
    'Please enter 6 numeric values, for example: 2, 0, 5, 0, 0, 90': '请输入 6 个数值，格式示例：2, 0, 5, 0, 0, 90',
    'The input contains non-numeric values. Please try again.': '输入包含非数值内容，请重新输入',
    'Visualizer is not initialized': '可视化器尚未初始化',
    'The visualizer does not implement environment attachment': '可视化器未实现环境附着功能',
    'Unable to add environment objects because the visualizer is not initialized': '可视化器尚未初始化，无法添加环境物体',
    'Failed to add environment object because the visualizer returned no mesh': '添加环境物体失败，返回值为空',
    'Failed to load scene preset: {{message}}': '加载场景预设失败：{{message}}',
    'Remove': '移除',
    'Drop to load a URDF folder': '释放以加载 URDF 文件夹',
    'Loading URDF assets...': '正在加载 URDF 资源...',
    'Load complete': '加载完成',
    'Load failed': '加载失败',
    'Load failed. See the console for details.': '加载失败，请查看控制台了解详情',
    'Loaded humanoid: {{name}}': '已加载人形: {{name}}',
    'Loaded: {{name}}': '已加载: {{name}}',
    'Loading failed:\n\n{{message}}\n\nOpen the console (F12) for detailed error information.': '加载失败:\n\n{{message}}\n\n请打开控制台(F12)查看详细错误信息。',
    'robot_config.yaml was not found': '未找到 robot_config.yaml 配置文件',
    'The visualizer failed to load the URDF': '可视化器加载失败',
    'Joint {{number}}': '关节 {{number}}',
    'Robot {{number}}': '机器人{{number}}',
    'Robot 1': '机器人1',
    'Robot 2': '机器人2',
    '{{count}} Joints': '{{count}} 关节',
    '🦴 Waist': '🦴 腰部',
    '🦾 Left Arm': '🦾 左臂',
    '🦿 Right Arm': '🦿 右臂',
    '🗣️ Head': '🗣️ 头部',
    'Angle (deg)': '角度(°)',
    'Angle (rad)': '角度(rad)',
    'Mode': '模式',
    'Start posture': '起始姿态',
    'Null-space': '零空间',
    'Connection URL: {{url}}': '连接地址: {{url}}',
    'Please enter a valid URL': '请输入有效的地址',
    'The URL must start with ws:// or wss://': '地址必须以 ws:// 或 wss:// 开头',
    'Saved URL: {{url}}. Please reconnect.': '✓ 地址已保存: {{url}}，请重新连接',
    'ROS 2 is not connected': 'ROS 2 未连接',
    'Switching...': '切换中...',
    'Switched control mode: {{mode}}': '切换控制模式: {{mode}}',
    'Failed to switch mode: {{message}}': '切换模式失败: {{message}}',
    'humanoidQ is not ready, so humanoid arm joints cannot be published': 'humanoidQ 未就绪，无法发布人形手臂关节',
    'ROS 2 is not connected, so publishing is unavailable': 'ROS 2 未连接，无法发布',
    'Topic name is not set': '话题名称未设置',
    'Sent to {{topic}} (source: {{source}})': '已发送到 {{topic}}（来源: {{source}}）',
    'Send failed: {{topic}}': '发送失败: {{topic}}',
    'Publish failed: {{message}}': '发布失败: {{message}}',
    'Waist joints can only be published in humanoid mode': '仅人形模式可发布腰部关节',
    'Sent to {{topic}}': '已发送到 {{topic}}',
    'Real joint state is currently subscribed. Please unsubscribe before starting continuous joint publishing.': '当前已订阅真机关节状态，如需持续发布关节位置，请先取消真机关节订阅。',
    'Continuous joint publishing was blocked because real joint state is subscribed. Please unsubscribe first.': '开启持续发布关节位置被阻止：已订阅真机关节状态，请先取消订阅。',
    'Please set a topic name': '请设置话题名称',
    'Continuously publishing to: {{topic}} ({{hz}}Hz)': '正在持续发布到: {{topic}} ({{hz}}Hz)',
    'Started continuous joint publishing to: {{topic}}': '开始持续发布关节位置到: {{topic}}',
    'Publishing stopped': '已停止发布',
    'Continuous joint publishing stopped': '已停止关节位置发布',
    'Real joint state is currently subscribed. Please unsubscribe before starting continuous waist publishing.': '当前已订阅真机关节状态，如需持续发布腰部关节，请先取消真机关节订阅。',
    'Continuous waist publishing was blocked because real joint state is subscribed. Please unsubscribe first.': '开启持续发布腰部关节被阻止：已订阅真机关节状态，请先取消订阅。',
    'Started continuous waist publishing to: {{topic}}': '开始持续发布腰部关节到: {{topic}}',
    'Continuous waist publishing stopped': '已停止腰部关节发布',
    'Subscribed to {{topic}}': '已订阅 {{topic}}',
    'Please connect to ROS 2 first': '请先连接 ROS 2',
    'Continuous joint or waist publishing is active. Please stop publishing before subscribing to real joint state.': '当前已开启持续发布关节/腰部关节，请先关闭相应发布，再订阅真机关节状态。',
    'Subscribing to real joint state was blocked because continuous publishing is active. Please stop publishing first.': '订阅真机关节状态被阻止：存在持续发布关节/腰部关节，请先关闭相应发布。',
    'Subscribed to real joint state: {{topic}}': '已订阅真机关节: {{topic}}',
    'Subscription failed. Please check the topic.': '订阅失败，请检查话题',
    'Unsubscribe': '取消订阅',
    'Unsubscribed': '已取消订阅',
    'Unsubscribed from real joint state': '已取消真机关节订阅',
    'Continuous joint or waist publishing is active, so automatic real joint state subscription was skipped. Stop publishing first if you want to subscribe.': '检测到已开启持续发布关节/腰部关节，已跳过自动订阅真机关节状态。如需订阅，请先关闭持续发布。',
    'Received at {{time}}; matched {{count}} joints': '已接收 {{time}}，匹配 {{count}} 个关节',
    'Received at {{time}}, but no updatable humanoid joints were matched': '收到 {{time}}，但未匹配到可更新的人形关节',
    'Received at {{time}}': '已接收 {{time}}',
    'Status: {{status}}': '状态: {{status}}',
    'Connection lost; subscription stopped': '连接断开，已停止订阅',
    'Current control mode: {{mode}}': '当前控制模式: {{mode}}',
    'ROS 2 initialized successfully': 'ROS 2 初始化成功',
    'ROS 2 initialization failed': 'ROS 2 初始化失败',
    'Initialization exception: {{message}}': '初始化异常: {{message}}',
    'Unsubscribed (preparing to disconnect)': '已取消订阅（准备断开连接）',
    'Unsubscribed from real joint state before disconnecting: {{topic}}': '断开前已取消真机关节订阅: {{topic}}',
    '{{solver}} is enabled and waiting for jog input.': '{{solver}} 已启用，等待点动输入。',
    'Current solver: {{solver}}. Displaying the current CCTSM.': '当前为{{solver}}求解器，显示当前 CCTSM。',
    'Constraint manifold diagnostics updated.': '约束流形诊断已更新。',
    'Restore (LS) diagnostics updated.': 'Restore (LS) 诊断已更新。',
};

const staticSelectorBindings = [
    { selector: '#loading p', property: 'textContent', key: 'Loading WASM module...' },
    { selector: '#ik-dq-input', property: 'placeholder', key: "qw qx qy qz qw' qx' qy' qz'" },
    { selector: '#control-panel .panel-header h2', property: 'textContent', key: 'Control Panel' },
    { selector: '#language-select', property: 'title', key: 'Language' },
];

const reverseDictionary = new Map();

function registerReverseEntry(source, key) {
    const normalized = normalizeLookupValue(source);
    if (!normalized) return;
    reverseDictionary.set(normalized, key);
}

function normalizeLookupValue(value) {
    return String(value ?? '').trim();
}

Object.keys(zhMessages).forEach((key) => {
    registerReverseEntry(key, key);
    registerReverseEntry(zhMessages[key], key);
});

staticSelectorBindings.forEach(({ key }) => {
    registerReverseEntry(key, key);
    registerReverseEntry(zhMessages[key], key);
});

function sanitizeLanguage(language) {
    return SUPPORTED_LANGUAGES.has(language) ? language : DEFAULT_LANGUAGE;
}

function interpolate(template, params = {}) {
    return String(template).replace(/\{\{\s*([a-zA-Z0-9_]+)\s*\}\}/g, (_match, name) => {
        if (!Object.prototype.hasOwnProperty.call(params, name)) {
            return '';
        }
        return String(params[name]);
    });
}

function resolveDocument(documentOverride) {
    return documentOverride ?? (typeof document !== 'undefined' ? document : null);
}

function resolveStorage(storageOverride) {
    return storageOverride ?? (typeof localStorage !== 'undefined' ? localStorage : null);
}

function readStoredLanguage(storage) {
    try {
        return sanitizeLanguage(storage?.getItem?.(LANGUAGE_STORAGE_KEY));
    } catch (_error) {
        return DEFAULT_LANGUAGE;
    }
}

function translateTextNode(node) {
    const raw = String(node?.textContent ?? '');
    const trimmed = normalizeLookupValue(raw);
    if (!trimmed) return;
    const key = reverseDictionary.get(trimmed);
    if (!key) return;
    node.textContent = raw.replace(trimmed, t(key));
}

function translateKnownAttributes(doc) {
    const elements = doc?.querySelectorAll?.('[placeholder],[title]') ?? [];
    elements.forEach((element) => {
        ['placeholder', 'title'].forEach((attribute) => {
            if (typeof element?.getAttribute !== 'function' || typeof element?.setAttribute !== 'function') return;
            const currentValue = element.getAttribute(attribute);
            const key = reverseDictionary.get(normalizeLookupValue(currentValue));
            if (!key) return;
            element.setAttribute(attribute, t(key));
        });
    });
}

function applySelectorBindings(doc) {
    staticSelectorBindings.forEach(({ selector, property, key }) => {
        const elements = doc?.querySelectorAll?.(selector) ?? [];
        elements.forEach((element) => {
            if (!element) return;
            element[property] = t(key);
        });
    });
}

function translateDocumentText(doc) {
    if (!doc?.body || typeof doc.createTreeWalker !== 'function') {
        return;
    }
    const SHOW_TEXT = globalThis.NodeFilter?.SHOW_TEXT ?? 4;
    const walker = doc.createTreeWalker(doc.body, SHOW_TEXT);
    let current = walker.nextNode();
    while (current) {
        translateTextNode(current);
        current = walker.nextNode();
    }
    translateKnownAttributes(doc);
}

function syncDocumentMetadata(doc) {
    if (!doc) return;
    if (doc.documentElement) {
        doc.documentElement.lang = currentLanguage === 'zh' ? 'zh-CN' : 'en';
    }
    doc.title = t('DQ Robotics Visualizer');
    const select = doc.getElementById?.('language-select');
    if (select) {
        select.value = currentLanguage;
        select.title = t('Language');
    }
}

export function t(key, params = {}) {
    const template = currentLanguage === 'zh'
        ? (zhMessages[key] ?? key)
        : key;
    return interpolate(template, params);
}

export function getCurrentLanguage() {
    return currentLanguage;
}

export function applyTranslations({ document: documentOverride } = {}) {
    const doc = resolveDocument(documentOverride);
    if (!doc) return;
    applySelectorBindings(doc);
    translateDocumentText(doc);
    syncDocumentMetadata(doc);
}

export function setLanguage(language, { document: documentOverride, storage: storageOverride, persist = true } = {}) {
    const doc = resolveDocument(documentOverride);
    const storage = resolveStorage(storageOverride);
    currentLanguage = sanitizeLanguage(language);
    if (persist) {
        try {
            storage?.setItem?.(LANGUAGE_STORAGE_KEY, currentLanguage);
        } catch (_error) {}
    }
    applyTranslations({ document: doc });
    listeners.forEach((listener) => listener(currentLanguage));
    return currentLanguage;
}

export function onLanguageChange(listener) {
    if (typeof listener !== 'function') {
        return () => {};
    }
    listeners.add(listener);
    return () => listeners.delete(listener);
}

export function initializeI18n({ document: documentOverride, storage: storageOverride } = {}) {
    const doc = resolveDocument(documentOverride);
    const storage = resolveStorage(storageOverride);
    currentLanguage = readStoredLanguage(storage);

    const select = doc?.getElementById?.('language-select');
    if (select && !select.dataset?.i18nBound) {
        select.addEventListener('change', (event) => {
            setLanguage(event?.target?.value, { document: doc, storage });
        });
        if (select.dataset) {
            select.dataset.i18nBound = 'true';
        }
    }

    applyTranslations({ document: doc });
    return currentLanguage;
}
