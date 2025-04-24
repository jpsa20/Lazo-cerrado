#!/usr/bin/env python3
import os
import signal
import yaml
import numpy as np
import transforms3d
import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # ----- Parámetros de corrección -----
        self.error_pct_linear  = 0.06
        self.error_pct_angular = 0.05

        # ----- Carga de waypoints y tiempos desde YAML -----
        pkg = os.getenv('AMENT_PREFIX_PATH').split(':')[0]
        cfg_file = os.path.join(pkg,
                                'share',
                                'puzzlebot_localisation',
                                'config',
                                'waypoints.yaml')
        with open(cfg_file, 'r') as f:
            cfg = yaml.safe_load(f)
        self.waypoints = cfg['waypoints']
        self.times     = cfg['times']

        # Índice actual de waypoint y umbral de llegada
        self.current_idx   = 0
        self.goal_threshold = 0.05  # m

        # Pose cruda y corregida
        self.raw_x = self.raw_y = self.raw_th = 0.0
        self.x = self.y = self.theta = 0.0

        # Publishers
        self.pub_corr = self.create_publisher(Odometry,   'corrected_odom', 10)
        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose',      10)
        self.pub_time = self.create_publisher(Float32,      'goal_time',      10)

        # Subscriber a odometría cruda
        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_cb,
            qos.qos_profile_sensor_data)

        # Timer para comprobar llegada al waypoint
        self.create_timer(0.05, self._check_goal)

        # Publica el primer objetivo
        self._publish_goal()
        self.get_logger().info('PathGenerator iniciado.')

    def odom_cb(self, msg: Odometry):
        """Callback de odometría: corrige y republica."""
        # 1) Lectura cruda
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.raw_th = np.arctan2(siny, cosy)

        # 2) Corrección
        self.x = self.raw_x * (1 + self.error_pct_linear)
        self.y = self.raw_y * (1 + self.error_pct_linear)
        th = self.raw_th * (1 + self.error_pct_angular)
        self.theta = (th + np.pi) % (2*np.pi) - np.pi

        # 3) Publicar corrected_odom
        out = msg
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        qc = transforms3d.euler.euler2quat(0, 0, self.theta)
        out.pose.pose.orientation.x = qc[1]
        out.pose.pose.orientation.y = qc[2]
        out.pose.pose.orientation.z = qc[3]
        out.pose.pose.orientation.w = qc[0]
        self.pub_corr.publish(out)

    def _publish_goal(self):
        """Publica el siguiente waypoint y su tiempo."""
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('Todos los waypoints completados.')
            return

        gx, gy = self.waypoints[self.current_idx]
        # Publicar PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.header.stamp     = self.get_clock().now().to_msg()
        pose_msg.pose.position.x  = gx
        pose_msg.pose.position.y  = gy
        pose_msg.pose.orientation.w = 1.0
        self.pub_goal.publish(pose_msg)

        # Publicar Float32 con tiempo deseado
        t_msg = Float32()
        t_msg.data = self.times[self.current_idx]
        self.pub_time.publish(t_msg)

        self.get_logger().info(
            f'Goal #{self.current_idx+1}: ({gx:.2f},{gy:.2f}), '
            f'time={t_msg.data:.2f}s')

    def _check_goal(self):
        """Comprueba si se alcanzó el waypoint para avanzar."""
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

if __name__ == '__main__':
    main()
