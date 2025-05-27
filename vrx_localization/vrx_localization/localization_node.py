# https://github.com/ros2/rclpy/issues/1077
# https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/singaboat_vrx/src/singaboat_scene_perception.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPoseStamped

# Topicos do VRX
GPS_TOPIC = '/wamv/sensors/gps/gps/fix'
IMU_TOPIC = '/wamv/sensors/imu/imu/data'

class LocalizationNode(Node):

    def __init__(self):
        super().__init__('wamv_localization_node')

        self.latest_gps: NavSatFix | None = None
        self.latest_imu: Imu | None = None

        # Subscriber
        self.gps_subscriber = self.create_subscription(NavSatFix, GPS_TOPIC, self.gps_callback, 10) # QoS profile depth
        self.imu_subscriber = self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, 10)

        # Publisher
        self.geopose_publisher = self.create_publisher(GeoPoseStamped, '/wamv/geopose', 10)

        t_period = 0.1 # 10 Hz
        self.timer = self.create_timer(t_period, self.timer_callback)

        self.get_logger().info('Nó de localização iniciado.')
        self.get_logger().info('Aguardando GPS e IMU...')

    def gps_callback(self, msg: NavSatFix):
        if self.latest_gps is None:
            self.get_logger().info('Mensagem de GPS recebido.')
        self.latest_gps = msg

    def imu_callback(self, msg: Imu):
        if self.latest_imu is None:
            self.get_logger().info('Mensagem de IMU recebido.')
        self.latest_imu = msg

    def timer_callback(self):
        if self.latest_gps is not None and self.latest_imu is not None:
            geo_pose_msg = GeoPoseStamped()

            # Header:
            geo_pose_msg.header.stamp = self.get_clock().now().to_msg()
            geo_pose_msg.header.frame_id = 'map' # world...

            # Pose:
            geo_pose_msg.pose.position.latitude = self.latest_gps.latitude
            geo_pose_msg.pose.position.longitude = self.latest_gps.longitude
            geo_pose_msg.pose.position.altitude = self.latest_gps.altitude

            # > orientation
            geo_pose_msg.pose.orientation = self.latest_imu.orientation

            # > publish
            self.geopose_publisher.publish(geo_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    try:
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
