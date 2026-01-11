import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/arm/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = [
            'rjoint6',
            'rjoint4',
            'rjoint7',
            'ljoint2',
            'rjoint5',
            'rjoint3',
            'ljoint7',
            'rjoint2',
            'ljoint5',
            'rjoint1',
            'ljoint1',
            'ljoint4',
            'ljoint6',
            'ljoint3'
        ]
        msg.position = [
            0.5957238358203698,
            0.5702813263462978,
            0.03807388252431783,
            0.19988488719644226,
            0.0818762245533961,
            -0.18283133515611985,
            0.1737353084529542,
            -0.058770638935868626,
            -0.006279733850406844,
            0.8498013880388345,
            -0.6448112210327104,
            -0.7820545646488538,
            -0.355488063367592,
            0.14103035868623615
        ]
        msg.velocity = [
            0.000587227020362472,
            0.0,
            -0.00013182647395892227,
            0.0,
            0.0,
            0.0,
            -1.1984224905356572e-05,
            0.0,
            0.0003475425222553406,
            0.0,
            0.0,
            0.0,
            0.0,
            -0.0003954794218767669
        ]
        msg.effort = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint states')

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
