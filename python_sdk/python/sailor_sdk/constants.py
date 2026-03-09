DEFAULT_JOINT_STATE_TOPIC = '/whole/joint_states'
DEFAULT_ARM_COMMAND_TOPIC = '/arm_axis_position_cmd'
DEFAULT_WAIST_COMMAND_TOPIC = '/axis_position_cmd'

HUMANOID_JOINT_NAMES = [
    'ankle_joint',
    'knee_joint',
    'hip_pitch_joint',
    'hip_yaw_joint',
    'ljoint1',
    'ljoint2',
    'ljoint3',
    'ljoint4',
    'ljoint5',
    'ljoint6',
    'ljoint7',
    'rjoint1',
    'rjoint2',
    'rjoint3',
    'rjoint4',
    'rjoint5',
    'rjoint6',
    'rjoint7',
    'head_yaw_joint',
    'head_pitch_joint',
]

ACTIVE_COMMAND_JOINT_NAMES = HUMANOID_JOINT_NAMES[:18]
WHEEL_JOINT_NAMES = {
    'left_wheel_joint',
    'right_wheel_joint',
}

WAIST_JOINT_NAMES = HUMANOID_JOINT_NAMES[:4]
LEFT_ARM_JOINT_NAMES = HUMANOID_JOINT_NAMES[4:11]
RIGHT_ARM_JOINT_NAMES = HUMANOID_JOINT_NAMES[11:18]
HEAD_JOINT_NAMES = HUMANOID_JOINT_NAMES[18:20]

DEFAULT_INITIAL_Q = [
    -0.3,
    0.9,
    -0.6,
    0.0,
    -0.5236,
    0.0,
    0.0,
    -1.0471,
    0.0,
    0.0,
    0.0,
    0.5236,
    0.0,
    0.0,
    1.0471,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
]

DEFAULT_SOLVER_BINARY = 'python_sdk/build/sailor_nullspace_solver'

DEFAULT_TOPPRA_MAX_VEL = 0.45
DEFAULT_TOPPRA_MAX_ACC = 0.90
