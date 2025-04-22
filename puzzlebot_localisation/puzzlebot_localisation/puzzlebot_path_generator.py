import rclpy
import numpy as np
import yaml
import os
import signal
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from rclpy import qos

class PuzzlebotPathGenerator(Node):
    def __init__(self):
        super().__init__('puzzlebot_path_generator')

        # Cargar waypoints
        self.waypoints = self.load_waypoints()
        self.current_idx = 0

        # Umbral para considerar que llegó
        self.goal_threshold = 0.05  # [m]

        # Estado actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Control PID (últimos valores que pediste usar)
        self.Kp_d = 2.0
        self.Ki_d = 0.05
        self.Kd_d = 0.9

        self.Kp_theta = 2.0
        self.Ki_theta = 0.02
        self.Kd_theta = 1.09

        # Variables para PID
        self.e_d_integral = 0.0
        self.e_theta_integral = 0.0
        self.prev_e_d = 0.0
        self.prev_e_theta = 0.0

        # Saturaciones
        self.max_linear_speed = 0.35
        self.max_angular_speed = 0.7

        # Publicadores
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos.qos_profile_sensor_data)

        # Subscripciones
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)

        # Timer principal
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Nodo de Generador de Rutas + Control PID Iniciado.")

    def load_waypoints(self):
        """Carga waypoints desde archivo YAML"""
        try:
            package_share_directory = get_package_share_directory('puzzlebot_localisation')
            yaml_path = os.path.join(package_share_directory, 'config', 'waypoints.yaml')

            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self.get_logger().info(f"Waypoints cargados exitosamente: {data['waypoints']}")
            return data['waypoints']
        except Exception as e:
            self.get_logger().error(f"Error al cargar waypoints: {e}")
            return []

    def odom_callback(self, msg):
        """Actualiza estado del robot con la odometría"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def publish_goal(self):
        """Publica el waypoint actual"""
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info("Todos los waypoints completados.")
            return

        goal_x, goal_y = self.waypoints[self.current_idx]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'odom'

        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Publicando objetivo {self.current_idx+1}: ({goal_x}, {goal_y})")

    def control_loop(self):
        """Calcula y publica los comandos de control PID"""
        if self.current_idx >= len(self.waypoints):
            # No hay más objetivos
            cmd_msg = Twist()
            self.cmd_vel_pub.publish(cmd_msg)
            return

        goal_x, goal_y = self.waypoints[self.current_idx]

        # Cálculo de errores
        e_x = goal_x - self.x
        e_y = goal_y - self.y

        e_d = np.hypot(e_x, e_y)
        e_theta = np.arctan2(e_y, e_x) - self.theta
        e_theta = self.wrap_to_pi(e_theta)

        cmd_msg = Twist()

        if e_d > self.goal_threshold:
            # Control PID combinado
            self.e_d_integral += e_d * 0.05
            self.e_theta_integral += e_theta * 0.05

            de_d = (e_d - self.prev_e_d) / 0.05
            de_theta = (e_theta - self.prev_e_theta) / 0.05

            V_cmd = (self.Kp_d * e_d) + (self.Ki_d * self.e_d_integral) + (self.Kd_d * de_d)
            Omega_cmd = (self.Kp_theta * e_theta) + (self.Ki_theta * self.e_theta_integral) + (self.Kd_theta * de_theta)

            # Saturar velocidades
            V_cmd = np.clip(V_cmd, 0.0, self.max_linear_speed)
            Omega_cmd = np.clip(Omega_cmd, -self.max_angular_speed, self.max_angular_speed)

            cmd_msg.linear.x = V_cmd
            cmd_msg.angular.z = Omega_cmd

            self.prev_e_d = e_d
            self.prev_e_theta = e_theta

            self.cmd_vel_pub.publish(cmd_msg)
        else:
            # Llegó al waypoint
            self.get_logger().info(f"Llegado al Waypoint {self.current_idx + 1}.")
            self.current_idx += 1
            self.reset_pid_integrals()
            self.publish_goal()  # Publicar siguiente objetivo

    def reset_pid_integrals(self):
        """Resetea integrales del PID para estabilidad"""
        self.e_d_integral = 0.0
        self.e_theta_integral = 0.0
        self.prev_e_d = 0.0
        self.prev_e_theta = 0.0

    def wrap_to_pi(self, angle):
        """Normaliza ángulo entre [-pi, pi]"""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def stop_handler(self, signum, frame):
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPathGenerator()
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
