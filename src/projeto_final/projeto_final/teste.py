from .robot import R2D2
import rclpy
import numpy
import math
from geometry_msgs.msg import Twist, Vector3
import tf_transformations

class RobotControl(R2D2):

    def navigation_start(self):
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.ir_para_tras = Twist(linear=Vector3(x=-0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.girar_direita = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.girar_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.curva_direita = Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.curva_esquerda = Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))

        # Inicializa a trajetória a ser seguida pelo robô
        self.path = path
        self.path_index = 0

    def navigation_update(self):
        if self.path_index >= len(self.path):
            self.pub_cmd_vel.publish(self.parar)
            return

        current_position = self.path[self.path_index]
        next_position = self.path[self.path_index + 1] if self.path_index + 1 < len(self.path) else current_position

        # Converte as coordenadas do mapa para coordenadas do ambiente
        target_x = next_position[1] * resolution  # ajuste necessário baseado na resolução do mapa
        target_y = next_position[0] * resolution

        roll, pitch, yaw = tf_transformations.euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])

        dx = target_x - self.pose.position.x
        dy = target_y - self.pose.position.y
        angle_to_target = math.atan2(dy, dx)

        if abs(angle_to_target - yaw) > 0.1:
            if angle_to_target > yaw:
                self.pub_cmd_vel.publish(self.girar_esquerda)
            else:
                self.pub_cmd_vel.publish(self.girar_direita)
        else:
            self.pub_cmd_vel.publish(self.ir_para_frente)
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.path_index += 1

def main(args=None):
    rclpy.init(args=args)
    r2d2_control = RobotControl()
    r2d2_control.navigation()
    r2d2_control.destroy_node()
    rclpy.try_shutdown()
