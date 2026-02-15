/**
 * 全局常量定义
 */

export const RAD_TO_DEG = 180 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180;
export const CM_TO_M = 0.01;

export const JOINT_POSITION_PUBLISH_INTERVAL = 10; // 10ms，即100Hz发布频率
export const ROS2_MAX_LOG_ENTRIES = 50;
export const ROS2_BATTERY_STATE_TOPIC = '/battery_state';

export const DEFAULT_ENVIRONMENT_BOX_CM = {
    length: 41.5,
    width: 30.5,
    height: 14
};
export const DEFAULT_ENVIRONMENT_BOX_COLOR = 0xf97316;
export const ENVIRONMENT_BOX_BASE_OFFSET_X = 0.45; // 米
