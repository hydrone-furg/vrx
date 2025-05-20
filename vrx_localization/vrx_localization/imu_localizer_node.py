import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster

class ImuLocalizerNode(Node):
    def __init__(self):
        super().__init__('imu_localizer_node')

        self.declare_parameter('imu_topic', '/wamv/sensors/imu/data')
        self.declare_parameter('odom_topic', '/odometry/imu')
        self.declare_parameter('base_frame_id', 'wamv/base_link')
        self.declare_parameter('odom_frame_id', 'wamv/odom')

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.base_frame_id_ = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.odom_frame_id_ = self.get_parameter('odom_frame_id').get_parameter_value().string_value

        self.imu_subscriber = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10)

        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0

        self.current_twist = Twist()
        self.get_logger().info(f"Ouvindo IMU em '{imu_topic}'.")
        self.get_logger().info(f"Publicando odometria em '{odom_topic}'.")

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now().to_msg()

        self.current_pose.orientation = msg.orientation
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame_id_
        odom_msg.child_frame_id = self.base_frame_id_
        odom_msg.pose.pose = self.current_pose
        '''
        ...
        ...
        falta desenvolver aqui...
        ...
        ...
        '''

def main(args=None):
    rclpy.init(args=args)
    node = ImuLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
