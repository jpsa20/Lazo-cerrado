import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # ----- Parámetros PID distancia y ángulo -----
        self.Kp_d, self.Ki_d, self.Kd_d = 25.1886, 117.42, 0.1656
        self.Kp_th, self.Ki_th, self.Kd_th = 28.8044, 153.6469, 0.1811

        # Estados integrales y previos (distancia y ángulo)
        self.e_d_int = self.e_th_int = 0.0
        self.prev_e_d = self.prev_e_th = 0.0

        # ----- Límites de velocidad -----
        self.max_v = 0.837   # m/s (lineal)
        self.max_w = 7.57    # rad/s (angular)
        self.min_v = 0.033   # m/s (zona muerta lineal)
        self.min_w = 0.506   # rad/s (zona muerta angular)

        # ----- Estado interno -----
        self.x = self.y = self.theta = 0.0
        self.gx = self.gy = None

        # Feed-forward de tiempo
        self.time_to_reach = None
        self.goal_start_time = None

        # ----- Subscripciones -----
        self.sub_odom = self.create_subscription(
            Odometry,
            'corrected_odom',
            self.odom_cb,
            qos.qos_profile_sensor_data)

        self.sub_goal = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_cb,
            qos.qos_profile_sensor_data)

        self.sub_time = self.create_subscription(
            Float32,
            'goal_time',
            self.time_cb,
            qos.qos_profile_sensor_data)

        # ----- Publicador de cmd_vel -----
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # Lazo de control cada 50 ms
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Controller iniciado.')

    def odom_cb(self, msg: Odometry):
        """Actualiza pose (x, y, theta) desde odometría corregida."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.theta = np.arctan2(siny, cosy)

    def goal_cb(self, msg: PoseStamped):
        """Actualiza meta espacial (gx, gy)."""
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y

    def time_cb(self, msg: Float32):
        """
        Recibe tiempo objetivo (s) para la meta actual.
        Registra instante de inicio para calcular tiempo_restante.
        """
        self.time_to_reach   = msg.data
        self.goal_start_time = self.get_clock().now()
        self.get_logger().info(
            f'Recibido tiempo objetivo: {self.time_to_reach:.2f}s')

    def control_loop(self):
        """Lazo PID + feed-forward de tiempo + saturación + zona muerta."""
        # Esperar meta y odometría inicial
        if self.gx is None:
            return

        # 1) Cálculo de errores
        ex = self.gx - self.x
        ey = self.gy - self.y
        e_d = np.hypot(ex, ey)           # error distancia
        theta_d = np.arctan2(ey, ex)
        e_th = theta_d - self.theta      # error orientación
        e_th = (e_th + np.pi) % (2*np.pi) - np.pi

        dt = 0.05  # intervalo fijo (50 ms)

        # 2) PID de orientación (w_cmd)
        self.e_th_int += e_th * dt
        de_th = (e_th - self.prev_e_th) / dt
        w_cmd = (self.Kp_th * e_th
               + self.Ki_th * self.e_th_int
               + self.Kd_th * de_th)
        self.prev_e_th = e_th
        # Saturación angular y zona muerta
        w_cmd = float(np.clip(w_cmd, -self.max_w, self.max_w))
        if abs(w_cmd) < self.min_w:
            w_cmd = 0.0

        # 3) Feed-forward lineal según tiempo
        if self.time_to_reach is not None and self.goal_start_time:
            # Tiempo transcurrido (s)
            tiempo_trans = (
                self.get_clock().now() - self.goal_start_time
            ).nanoseconds * 1e-9
            tiempo_rest = self.time_to_reach - tiempo_trans

            if tiempo_rest <= 0 or e_d == 0.0:
                v_cmd = 0.0
                if tiempo_rest < 0:
                    self.get_logger().warn(
                        f"¡Tiempo excedido! {tiempo_rest:.2f}s restantes.")
            else:
                # velocidad requerida para cubrir e_d en tiempo_rest
                v_req = e_d / tiempo_rest
                # comprobar límites
                if v_req > self.max_v:
                    self.get_logger().warn(
                        f"v_req={v_req:.3f} m/s > max_v={self.max_v:.3f}. Se saturará.")
                    v_req = self.max_v
                elif 0 < v_req < self.min_v:
                    self.get_logger().warn(
                        f"v_req={v_req:.3f} m/s < min_v={self.min_v:.3f}. Se detendrá.")
                    v_req = 0.0
                v_cmd = v_req
        else:
            # Si no hay tiempo asignado, recurre al PID de distancia
            self.e_d_int += e_d * dt
            de_d = (e_d - self.prev_e_d) / dt
            v_pid = (self.Kp_d * e_d
                   + self.Ki_d * self.e_d_int
                   + self.Kd_d * de_d)
            self.prev_e_d = e_d
            # Saturación y zona muerta
            v_cmd = float(np.clip(v_pid, 0.0, self.max_v))
            if 0.0 < v_cmd < self.min_v:
                v_cmd = 0.0

        # 4) Publicar comando
        twist = Twist()
        twist.linear.x  = v_cmd
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