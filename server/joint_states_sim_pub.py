import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_STATE_TOPIC = '/whole/joint_states'
JOINT_ORDER = [
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
    'head_pitch_joint'
]

HUMANOID_Q = [
    -0.3,
    0.9,
    -0.6,
    0.0,
    -0.36757,
    0.2782,
    0.50861,
    -1.39601,
    -0.38305,
    0.43154,
    0.03948,
    0.36758,
    -0.27819,
    -0.50863,
    1.39601,
    0.38303,
    -0.43156,
    -0.03949,
    0.0,
    0.0
]

POSITION_BY_JOINT = dict(zip(JOINT_ORDER, HUMANOID_Q))
VELOCITY_BY_JOINT = {name: 0.0 for name in JOINT_ORDER}


def build_joint_state_payload():
    return {
        'name': list(JOINT_ORDER),
        'position': [POSITION_BY_JOINT[name] for name in JOINT_ORDER],
        'velocity': [VELOCITY_BY_JOINT[name] for name in JOINT_ORDER],
        'effort': [0.0] * len(JOINT_ORDER),
    }

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, JOINT_STATE_TOPIC, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        payload = build_joint_state_payload()
        msg.name = payload['name']
        msg.position = payload['position']
        msg.velocity = payload['velocity']
        msg.effort = payload['effort']
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states to {JOINT_STATE_TOPIC}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
