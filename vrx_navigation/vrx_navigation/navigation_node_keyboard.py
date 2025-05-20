import rclpy
import time
import sys
import select
import tty
import termios
from rclpy.node import Node
from std_msgs.msg import Float64

LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left/thrust'
RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right/thrust'

THRUST_MIN = -10000.0
THRUST_MAX = 10000.0
THRUST_INCREMENT = 10

KEY_FORWARD = 'w'
KEY_BACKWARD = 's'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_STOP = 'p'
KEY_QUIT = 'q'

instructions = f"""
Control Your WAM-V!
---------------------------
Moving around:
      {KEY_FORWARD}
{KEY_LEFT}     {KEY_RIGHT}
      {KEY_BACKWARD}

{KEY_STOP} : stop all thrusters
{KEY_QUIT} : quit

CTRL-C para sair
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class WAMVThrusterController(Node):
    def __init__(self):
        super().__init__('wamv_thruster_controller')

        self.left_thrust_pub = self.create_publisher(Float64, LEFT_THRUSTER_TOPIC, 10)
        self.right_thrust_pub = self.create_publisher(Float64, RIGHT_THRUSTER_TOPIC, 10)
        
        self.current_left_thrust = 0.0
        self.current_right_thrust = 0.0

        self.get_logger().info(f'Controlador de propulsores WAM-V iniciado.')
        self.get_logger().info(f'Publicando para: {LEFT_THRUSTER_TOPIC}, {RIGHT_THRUSTER_TOPIC}')
        self.get_logger().info(instructions)


    def set_thrusters(self, left_target, right_target):
        self.current_left_thrust = max(THRUST_MIN, min(THRUST_MAX, left_target))
        self.current_right_thrust = max(THRUST_MIN, min(THRUST_MAX, right_target))

        left_msg = Float64()
        left_msg.data = self.current_left_thrust
        self.left_thrust_pub.publish(left_msg)

        right_msg = Float64()
        right_msg.data = self.current_right_thrust
        self.right_thrust_pub.publish(right_msg)

        self.get_logger().info(f'Impulso: Esquerdo={self.current_left_thrust:.2f}, Direito={self.current_right_thrust:.2f}')

    def stop_thrusters(self):
        self.set_thrusters(0.0, 0.0)
        self.get_logger().info('Propulsores parados.')

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    wamv_controller_node = WAMVThrusterController()

    try:
        while rclpy.ok():
            key = get_key(settings)

            left_target = wamv_controller_node.current_left_thrust
            right_target = wamv_controller_node.current_right_thrust

            if key == KEY_FORWARD:
                left_target += THRUST_INCREMENT
                right_target += THRUST_INCREMENT
            elif key == KEY_BACKWARD:
                left_target -= THRUST_INCREMENT
                right_target -= THRUST_INCREMENT
            elif key == KEY_LEFT:
                left_target -= THRUST_INCREMENT
                right_target += THRUST_INCREMENT
            elif key == KEY_RIGHT:
                left_target += THRUST_INCREMENT
                right_target -= THRUST_INCREMENT
            elif key == KEY_STOP:
                left_target = 0.0
                right_target = 0.0
            elif key == KEY_QUIT:
                break
            else:
                pass

            wamv_controller_node.set_thrusters(left_target, right_target)
            time.sleep(0.05) # 20Hz

    except Exception as e:
        wamv_controller_node.get_logger().error(f'Erro: {e}')
    finally:
        wamv_controller_node.get_logger().info('Parando propulsores antes de sair...')
        wamv_controller_node.stop_thrusters()
        time.sleep(0.5)
        wamv_controller_node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("Nó encerrado.")

if __name__ == '__main__':
    main()
