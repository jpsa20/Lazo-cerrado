import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import signal

class PuzzlebotPoseController(Node):
    def __init__(self):
        super().__init__('puzzlebot_pose_controller')

        # Estado actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goal
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Control
        self.Kp_d = 2.0
        self.Kp_theta = 2.0
        self.max_linear_speed = 0.35
        self.max_angular_speed = 0.7
        self.goal_threshold = 0.05  # m

        # Subscripciones
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_d_pub = self.create_publisher(Float32, 'error_d', 10)
        self.error_theta_pub = self.create_publisher(Float32, 'error_theta', 10)

        # Timer de control
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Nodo de Control de Posición mejorado iniciado.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def control_loop(self):
        # Cálculo de errores
        e_x = self.goal_x - self.x
        e_y = self.goal_y - self.y

        e_d = np.hypot(e_x, e_y)
        e_theta = np.arctan2(e_y, e_x) - self.theta
        e_theta = self.wrap_to_pi(e_theta)

        # Publicar errores
        self.error_d_pub.publish(Float32(data=e_d))
        self.error_theta_pub.publish(Float32(data=e_theta))

        # Mensaje de control
        cmd_msg = Twist()

        if e_d > self.goal_threshold:
            # Control proporcional
            V_cmd = self.Kp_d * e_d
            Omega_cmd = self.Kp_theta * e_theta

            # Saturaciones
            V_cmd = np.clip(V_cmd, 0.0, self.max_linear_speed)
            Omega_cmd = np.clip(Omega_cmd, -self.max_angular_speed, self.max_angular_speed)

            cmd_msg.linear.x = V_cmd
            cmd_msg.angular.z = Omega_cmd
        else:
            # Llegó: detenerse
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_pub.publish(cmd_msg)

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPoseController()
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
