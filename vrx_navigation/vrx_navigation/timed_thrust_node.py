# https://docs.ros.org/en/dashing/Tutorials/Using-Parameters-In-A-Class-Python.html

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoseStamped

LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left/thrust'
RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right/thrust'

GEOPOSE_TOPIC = '/wamv/geopose' ##

THRUST_VALUE = 5000.0
MOVE_DURATION = 5.0

class TimedThrustNode(Node):

    def __init__(self):
        super().__init__('wamv_simple_navigation_node')

        self.stop_timer = None # Para guardar a referencia do timer
        self.geopose_received = False ##
        self.movement_started = False

        self.geopose_subscriber = self.create_subscription(GeoPoseStamped, GEOPOSE_TOPIC, self.geopose_callback, 10) ##
        self.left_thrust_publisher = self.create_publisher(Float64, LEFT_THRUSTER_TOPIC, 10)
        self.right_thrust_publisher = self.create_publisher(Float64, RIGHT_THRUSTER_TOPIC, 10)

        self.get_logger().info('TimedThrustNode iniciado.')
        self.get_logger().info('Aguardando Geopose...')
        time.sleep(1.0) 
        #self.start_movement()
    #''' 
    def geopose_callback(self, msg: GeoPoseStamped): ##
        if not self.geopose_received:
            self.geopose_received = True
            self.get_logger().info('Mensagem GeoPose recebida. Iniciando movimento.')
            self.start_movement() ##
    #'''
    def start_movement(self):
        if self.movement_started:
            return

        self.movement_started = True
        thrust_msg = Float64()
        thrust_msg.data = THRUST_VALUE
        self.get_logger().info(f'{THRUST_VALUE} por {MOVE_DURATION} segundos.')
        self.left_thrust_publisher.publish(thrust_msg)
        self.right_thrust_publisher.publish(thrust_msg)
        self.stop_timer = self.create_timer(MOVE_DURATION, self.stop_and_shutdown)

    def stop_and_shutdown(self):
        thrust_msg = Float64()
        thrust_msg.data = 0.0

        self.get_logger().info('Tempo esgotado. Parando os motores e desligando o nó.')
        self.left_thrust_publisher.publish(thrust_msg)
        self.right_thrust_publisher.publish(thrust_msg)
        time.sleep(1)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    timed_thrust_node = TimedThrustNode()
    try:
        rclpy.spin(timed_thrust_node)
    except KeyboardInterrupt:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()