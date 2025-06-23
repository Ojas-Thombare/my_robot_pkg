import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()

    def timer_callback(self):
        self.publisher.publish(self.twist)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    print("Use WASD to move (W: forward, S: backward, A: left, D: right, Q: quit)")
    try:
        while True:
            key = get_key()
            if key == 'w':
                node.twist.linear.x = 0.5
            elif key == 's':
                node.twist.linear.x = -0.5
            elif key == 'a':
                node.twist.angular.z = 1.0
            elif key == 'd':
                node.twist.angular.z = -1.0
            elif key == 'q':
                break
            else:
                node.twist.linear.x = 0.0
                node.twist.angular.z = 0.0
    finally:
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher.publish(node.twist)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
