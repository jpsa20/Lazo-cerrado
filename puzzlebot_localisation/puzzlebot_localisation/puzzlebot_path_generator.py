#!/usr/bin/env python3
import rclpy
import numpy as np
import yaml, os, signal, transforms3d
from rclpy.node import Node
from rclpy import qos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # % de corrección fijo
        self.error_pct_linear  = 0.06
        self.error_pct_angular = 0.05

        # Carga waypoints
        pkg = os.getenv('AMENT_PREFIX_PATH').split(':')[0]
        cfg = os.path.join(pkg, 'share', 'puzzlebot_localisation', 'config', 'waypoints.yaml')
        with open(cfg) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.current_idx = 0
        self.goal_threshold = 0.03  # m

        # Pose bruta y corregida
        self.raw_x = self.raw_y = self.raw_th = 0.0
        self.x = self.y = self.theta = 0.0

        # Publishers
        self.pub_corr = self.create_publisher(Odometry,   'corrected_odom', 10)
        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose',      10)

        # Subscriber a odom bruto
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_cb, qos.qos_profile_sensor_data)

        # Timer para detección de llegada a waypoint
        self.create_timer(0.05, self._check_goal)

        # Publica el primer goal
        self._publish_goal()
        self.get_logger().info('PathGenerator iniciado.')

    def odom_cb(self, msg: Odometry):
        # 1) Leer odometría bruta
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.raw_th = np.arctan2(siny, cosy)

        # 2) Corregirla
        self.x = self.raw_x * (1 + self.error_pct_linear)
        self.y = self.raw_y * (1 + self.error_pct_linear)
        th  = self.raw_th * (1 + self.error_pct_angular)
        self.theta = (th + np.pi)%(2*np.pi) - np.pi

        # 3) Publicar corrected_odom
        out = msg  # reescribimos sobre el mismo
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        qc = transforms3d.euler.euler2quat(0,0,self.theta)
        out.pose.pose.orientation.x = qc[1]
        out.pose.pose.orientation.y = qc[2]
        out.pose.pose.orientation.z = qc[3]
        out.pose.pose.orientation.w = qc[0]
        self.pub_corr.publish(out)

    def _publish_goal(self):
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('Todos los waypoints completados.')
            return
        gx, gy = self.waypoints[self.current_idx]
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.pose.position.x  = gx
        msg.pose.position.y  = gy
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.get_logger().info(f'Goal #{self.current_idx+1}: ({gx},{gy})')

    def _check_goal(self):
        # Comprueba si la pose corregida está dentro del umbral
        if self.current_idx >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.current_idx]
        if np.hypot(gx - self.x, gy - self.y) < self.goal_threshold:
            self.get_logger().info(f'Waypoint {self.current_idx+1} alcanzado.')
            self.current_idx += 1
            self._publish_goal()

    def stop_handler(self, sig, frame):
        self.get_logger().info('PathGenerator detenido.')
        raise SystemExit

def main():
    rclpy.init()
    node = PathGenerator()
    signal.signal(signal.SIGINT, node.stop_handler)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()