#!/usr/bin/env python3
import rclpy
import numpy as np
import yaml
import os
import signal
import transforms3d
from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from ament_index_python.packages import get_package_share_directory

class PuzzlebotPathGenerator(Node):
    def __init__(self):
        super().__init__('puzzlebot_path_generator')

        # Parámetros de corrección (porcentajes fijos)
        self.error_pct_linear  = 0.06   # 6% de error en distancia
        self.error_pct_angular = 0.077   # 7.7% de error en orientación

        # Carga de waypoints
        self.waypoints     = self.load_waypoints()
        self.current_idx   = 0
        self.goal_threshold = 0.05     # [m] umbral de llegada

        # Pose bruta y corregida
        self.raw_x = self.raw_y = self.raw_th = 0.0
        self.x = self.y = self.theta = 0.0

        # Parámetros PID distancia
        self.Kp_d, self.Ki_d, self.Kd_d = 2.0, 0.05, 0.9
        # Parámetros PID ángulo
        self.Kp_th, self.Ki_th, self.Kd_th = 2.0, 0.02, 1.09

        # Estados internos PID
        self.e_d_int   = 0.0
        self.e_th_int  = 0.0
        self.prev_e_d  = 0.0
        self.prev_e_th = 0.0

        # Límites de velocidad
        self.max_v = 0.35   # m/s
        self.max_w = 0.7    # rad/s

        # Publicadores
        self.goal_pub          = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cmd_pub           = self.create_publisher(Twist,      'cmd_vel',  qos.qos_profile_sensor_data)
        self.corrected_pub     = self.create_publisher(Odometry,   'corrected_odom', 10)

        # Subscripción a odometría bruta
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data)

        # Timer de control (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Publicar primer objetivo
        self.publish_goal()
        self.get_logger().info('PathGenerator+PID iniciado.')

    def load_waypoints(self):
        pkg_dir = get_package_share_directory('puzzlebot_localisation')
        path = os.path.join(pkg_dir, 'config', 'waypoints.yaml')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.get_logger().info(f"Waypoints cargados: {data['waypoints']}")
        return data['waypoints']

    def odom_callback(self, msg: Odometry):
        # Extraer pose bruta
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.raw_th = np.arctan2(siny, cosy)

        # Calcular pose corregida
        self.x = self.raw_x * (1.0 + self.error_pct_linear)
        self.y = self.raw_y * (1.0 + self.error_pct_linear)

        th = self.raw_th * (1.0 + self.error_pct_angular)
        self.theta = (th + np.pi) % (2.0*np.pi) - np.pi

        # Publicar odometría corregida
        corrected = Odometry()
        corrected.header = msg.header
        corrected.child_frame_id = msg.child_frame_id
        # Pose corregida
        corrected.pose.pose.position.x = self.x
        corrected.pose.pose.position.y = self.y
        corrected.pose.pose.position.z = 0.0
        q_corr = transforms3d.euler.euler2quat(0, 0, self.theta)
        corrected.pose.pose.orientation.x = q_corr[1]
        corrected.pose.pose.orientation.y = q_corr[2]
        corrected.pose.pose.orientation.z = q_corr[3]
        corrected.pose.pose.orientation.w = q_corr[0]
        # Conserva twist bruto (opcional)
        corrected.twist = msg.twist
        self.corrected_pub.publish(corrected)

    def publish_goal(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('Todos los waypoints completados.')
            return
        gx, gy = self.waypoints[self.current_idx]
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp     = self.get_clock().now().to_msg()
        goal.pose.position.x  = gx
        goal.pose.position.y  = gy
        goal.pose.position.z  = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Objetivo #{self.current_idx+1}: ({gx:.2f},{gy:.2f})')

    def control_loop(self):
        if self.current_idx >= len(self.waypoints):
            # Detener robot si no hay más waypoints
            self.cmd_pub.publish(Twist())
            return

        gx, gy = self.waypoints[self.current_idx]

        # Errores basados en pose corregida
        ex = gx - self.x
        ey = gy - self.y
        e_d = np.hypot(ex, ey)

        theta_d = np.arctan2(ey, ex)
        e_th = theta_d - self.theta
        e_th = (e_th + np.pi) % (2.0*np.pi) - np.pi

        cmd = Twist()
        dt = 0.05

        if e_d > self.goal_threshold:
            # PID distancia
            self.e_d_int += e_d * dt
            de_d = (e_d - self.prev_e_d) / dt
            v_cmd = self.Kp_d*e_d + self.Ki_d*self.e_d_int + self.Kd_d*de_d

            # PID angular
            self.e_th_int += e_th * dt
            de_th = (e_th - self.prev_e_th) / dt
            w_cmd = self.Kp_th*e_th + self.Ki_th*self.e_th_int + self.Kd_th*de_th

            # Guardar para derivadas
            self.prev_e_d  = e_d
            self.prev_e_th = e_th

            # Saturar
            cmd.linear.x  = float(np.clip(v_cmd, 0.0, self.max_v))
            cmd.angular.z = float(np.clip(w_cmd, -self.max_w, self.max_w))
            self.cmd_pub.publish(cmd)
        else:
            # Waypoint alcanzado
            self.get_logger().info(f'Waypoint #{self.current_idx+1} alcanzado.')
            self.current_idx += 1
            # Reset PID
            self.e_d_int = self.e_th_int = 0.0
            self.prev_e_d = self.prev_e_th = 0.0
            self.publish_goal()

    def stop_handler(self, signum, frame):
        self.get_logger().info('Shutting down PathGenerator...')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPathGenerator()
    signal.signal(signal.SIGINT, node.stop_handler)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
