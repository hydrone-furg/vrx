# https://github.com/cavadibrahimli1/gps_publisher_ros2humble/blob/main/src/ros_pospac_bridge.cpp
# https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/singaboat_vrx/src/singaboat_station_keeping.py
# https://stackoverflow.com/questions/54214698/quaternion-to-yaw-pitch-roll
# https://youtu.be/2DOmG9-tvVY?si=3uczoIvKm1ZS-leF
# https://www.askpython.com/python/examples/find-distance-between-two-geo-locations
# https://github.com/matthew-brett/transforms3d/blob/main/transforms3d/taitbryan.py
# https://www.movable-type.co.uk/scripts/latlong.html

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
import math

# TARGET_WAYPOINT = (-33.722772, 150.674138) # ideal (para referencia, sem usar loiter 'dinamico')
# TARGET_WAYPOINT = (-33.722768, 150.673990) # default (para referencia)

MAX_ABS_THRUST = 10000.0
MIN_ABS_THRUST = -10000.0

LOITER_DEADBAND = 1.5
KP_DISTANCE_LOITER = 150.0
MAX_FORWARD_THRUST_LOITER = 1200.0
KP_YAW_LOITER = 250.0
MAX_THRUST_DIFFERENTIAL_LOITER = 3500.0
ALIGNED_YAW_ERROR_THRESHOLD_RAD_LOITER = math.radians(15.0)

# Topicos do VRX
GPS_TOPIC = '/wamv/sensors/gps/gps/fix'
IMU_TOPIC = '/wamv/sensors/imu/imu/data'
LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left/thrust'
RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right/thrust'

class LoiterNode(Node):
    def __init__(self):
        super().__init__('gps_dynamic_loiter_node')

        self.control_timer_period = 0.1 # 10 Hz
        self.control_timer = self.create_timer(self.control_timer_period, self.control_loop)

        # Estado atual
        self.current_latitude = None
        self.current_longitude = None
        self.current_yaw_rad = None

        # Waypoint de destino
        self.target_lat = None
        self.target_lon = None
        self.loiter_center_set = False

        # Subscribers para o IMU e o GPS
        self.gps_subscriber = self.create_subscription(NavSatFix, GPS_TOPIC, self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, 10)

        # Publishers para os thrusters
        self.left_thrust_pub = self.create_publisher(Float64, LEFT_THRUSTER_TOPIC, 10)
        self.right_thrust_pub = self.create_publisher(Float64, RIGHT_THRUSTER_TOPIC, 10)
        
        self.get_logger().info("No de Loiter Dinamico Iniciado.")
        self.get_logger().info("Aguardando primeira leitura do GPS para definir o centro do loiter...")

    def gps_callback(self, msg: NavSatFix):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        if not self.loiter_center_set and self.current_latitude is not None and self.current_longitude is not None:
            self.target_lat = self.current_latitude
            self.target_lon = self.current_longitude
            self.loiter_center_set = True
            self.get_logger().info(f"Centro do Loiter definido em: Lat={self.target_lat:.6f}, Lon={self.target_lon:.6f}")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.current_yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        return yaw_rad

    def calculate_distance_and_bearing(self, current_lat: float, current_lon: float, target_lat: float, target_lon: float):
        R_earth = 6371000.0

        lat1_rad = math.radians(current_lat)
        lon1_rad = math.radians(current_lon)
        lat2_rad = math.radians(target_lat)
        lon2_rad = math.radians(target_lon)

        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad

        # Haversine
        a = math.sin(delta_lat / 2.0)**2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * \
            math.sin(delta_lon / 2.0)**2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        distance = R_earth * c

        # Bearing
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        bearing_rad = math.atan2(y, x)

        return distance, bearing_rad

    def normalize_angle_mpi_pi(self, angle_rad: float) -> float: # [-PI, PI]
        angle_rad = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

        return angle_rad

    def set_thruster_commands(self, left_thrust: float, right_thrust: float):
        left_thrust_clamped = max(MIN_ABS_THRUST, min(MAX_ABS_THRUST, left_thrust))
        right_thrust_clamped = max(MIN_ABS_THRUST, min(MAX_ABS_THRUST, right_thrust))

        left_msg = Float64()
        left_msg.data = float(left_thrust_clamped)
        self.left_thrust_pub.publish(left_msg)

        right_msg = Float64()
        right_msg.data = float(right_thrust_clamped)
        self.right_thrust_pub.publish(right_msg)

    def control_loop(self):
        if self.current_latitude is None or self.current_longitude is None or self.current_yaw_rad is None:
            self.set_thruster_commands(0.0, 0.0) # Thrusters sao desligados se nao houver dados

            return

        if not self.loiter_center_set:
            self.set_thruster_commands(0.0, 0.0) # Mantem os thrusters desligados ate o instante que o centro seja definido

            return

        distance_to_target, bearing_to_target_rad = self.calculate_distance_and_bearing(
            self.current_latitude, self.current_longitude,
            self.target_lat, self.target_lon
        )

        current_yaw_ned_rad = self.normalize_angle_mpi_pi((math.pi / 2.0) - self.current_yaw_rad) # Em NED
        error_yaw_rad = self.normalize_angle_mpi_pi(bearing_to_target_rad - current_yaw_ned_rad)

        # Log
        self.get_logger().info(
             f"Loiter: D={distance_to_target:.1f}m, AzTgt(NED)={math.degrees(bearing_to_target_rad):.1f}°, "
             f"YawCurr(NED)={math.degrees(current_yaw_ned_rad):.1f}°, ErrYaw={math.degrees(error_yaw_rad):.1f}°",
             throttle_duration_sec=0.5
        )

        # P Controller
        turn_control_effort = KP_YAW_LOITER * error_yaw_rad
        turn_control_effort = max(-MAX_THRUST_DIFFERENTIAL_LOITER, min(MAX_THRUST_DIFFERENTIAL_LOITER, turn_control_effort))
        forward_thrust_command = 0.0
        if distance_to_target > LOITER_DEADBAND:
            forward_thrust_command = KP_DISTANCE_LOITER * distance_to_target
            forward_thrust_command = min(forward_thrust_command, MAX_FORWARD_THRUST_LOITER)
            if abs(error_yaw_rad) > ALIGNED_YAW_ERROR_THRESHOLD_RAD_LOITER: # Correcao
                alignment_factor = max(0.0, (math.pi - abs(error_yaw_rad)) / math.pi)
                forward_thrust_command *= alignment_factor**2 

        left_command = forward_thrust_command + turn_control_effort
        right_command = forward_thrust_command - turn_control_effort

        self.set_thruster_commands(left_command, right_command)

        if distance_to_target <= LOITER_DEADBAND and self.loiter_center_set:
             self.get_logger().info(f"Mantendo posiçao. Dist: {distance_to_target:.2f}m", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    gps_loiter_node = LoiterNode()
    try:
        rclpy.spin(gps_loiter_node)
    except KeyboardInterrupt:
        gps_loiter_node.get_logger().info("Loiter interrompido por (Ctrl+C).")
    finally:
        gps_loiter_node.get_logger().info("Parando...")
        if hasattr(gps_loiter_node, 'set_thruster_commands'):
             gps_loiter_node.set_thruster_commands(0.0, 0.0)
        if rclpy.ok() and gps_loiter_node.executor is not None :
            try:
                gps_loiter_node.destroy_node()
            except Exception as e:
                if gps_loiter_node and hasattr(gps_loiter_node, 'get_logger'):
                    gps_loiter_node.get_logger().error(e)
                else:
                    e
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()