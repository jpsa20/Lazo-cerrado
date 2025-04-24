#!/usr/bin/env python3
import rclpy
import numpy as np
import signal
from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # PID distancia
        self.Kp_d, self.Ki_d, self.Kd_d = 1.6, 0.001, 0.05
        # PID ángulo
        self.Kp_th, self.Ki_th, self.Kd_th = 1.0, 0.01, 0.05

        self.e_d_int = self.e_th_int = 0.0
        self.prev_e_d = self.prev_e_th = 0.0

        # Límites de velocidad (saturación y zona muerta)
        self.max_v = 0.29
        self.max_w = 0.5
        self.min_v = 0.05
        self.min_w = 0.05

        # Estado actual
        self.x = self.y = self.theta = 0.0
        self.gx = self.gy = None

        # Subscripciones
        self.sub_odom = self.create_subscription(
            Odometry, 'corrected_odom', self.odom_cb, qos.qos_profile_sensor_data)
        self.sub_goal = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_cb, qos.qos_profile_sensor_data)

        # Publicador de velocidad
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer PID
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Controller iniciado.')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = np.arctan2(siny, cosy)

    def goal_cb(self, msg: PoseStamped):
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        # Reset errores PID al llegar un nuevo objetivo
        self.e_d_int = 0.0
        self.e_th_int = 0.0
        self.prev_e_d = 0.0
        self.prev_e_th = 0.0

    def control_loop(self):
        if self.gx is None:
            # No hay más waypoints => detener robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            return

        # Cálculo de errores
        ex = self.gx - self.x
        ey = self.gy - self.y
        e_d = np.hypot(ex, ey)

        theta_d = np.arctan2(ey, ex)
        e_th = theta_d - self.theta
        e_th = (e_th + np.pi) % (2 * np.pi) - np.pi

        dt = 0.05

        # PID angular
        self.e_th_int += e_th * dt
        de_th = (e_th - self.prev_e_th) / dt
        w_cmd = self.Kp_th * e_th + self.Ki_th * self.e_th_int + self.Kd_th * de_th
        self.prev_e_th = e_th

        # Fase de giro: si error angular > 20°, no avanzar
        if abs(e_th) > np.deg2rad(15):
            v_cmd = 0.0
        else:
            # PID distancia
            self.e_d_int += e_d * dt
            self.e_d_int = np.clip(self.e_d_int, -0.5, 0.5)
            de_d = (e_d - self.prev_e_d) / dt
            v_cmd = self.Kp_d * e_d + self.Ki_d * self.e_d_int + self.Kd_d * de_d
            self.prev_e_d = e_d

        # Aplicar zona muerta
        if abs(v_cmd) < self.min_v:
            v_cmd = 0.0
        if abs(w_cmd) < self.min_w:
            w_cmd = 0.0

        # Saturación
        v_cmd = float(np.clip(v_cmd, 0.0, self.max_v))
        w_cmd = float(np.clip(w_cmd, -self.max_w, self.max_w))

        # Publicar velocidades
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.pub_cmd.publish(twist)

    def stop_handler(self, sig, frame):
        self.get_logger().info('Controller detenido.')
        raise SystemExit

def main():
    rclpy.init()
    node = Controller()
    signal.signal(signal.SIGINT, node.stop_handler)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
