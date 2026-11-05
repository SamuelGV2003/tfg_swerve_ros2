#!/usr/bin/env python3
"""Nodo de cinemática inversa de la plataforma swerve de dos módulos.

Recibe la velocidad deseada del chasis en /cmd_vel y calcula, para cada
módulo, el ángulo de dirección y la velocidad angular de la rueda:

    v_ix = vx - omega * l_iy
    v_iy = vy + omega * l_ix
    phi_i = atan2(v_iy, v_ix)
    omega_rueda_i = sqrt(v_ix^2 + v_iy^2) / r

donde (l_ix, l_iy) es la posición del módulo i respecto al origen del
chasis. En la simulación se comanda directamente la junta del módulo y de
la rueda, por lo que no se aplican las relaciones de transmisión del
prototipo.

Además de la cinemática, el nodo hace dos cosas:
  - Mientras un módulo no esté cerca de su ángulo objetivo, su rueda se
    mantiene detenida. Así se evita que un módulo ya orientado empuje
    al robot mientras el otro todavía está girando.
  - Si no llega /cmd_vel durante cmd_vel_timeout segundos, el robot se
    detiene. Los controladores de ros2_control mantienen el último
    comando, así que sin esto el robot seguiría avanzando.

Suscripciones:
    /cmd_vel        geometry_msgs/Twist   velocidad del chasis (vx, vy, omega)
    /joint_states   sensor_msgs/JointState   ángulo real de cada módulo

Publicaciones (orden [derecho, izquierdo]):
    /steer_position_controller/commands   ángulos de dirección [rad]
    /wheel_velocity_controller/commands   velocidades de rueda [rad/s]

Parámetros:
    cmd_vel_timeout   tiempo sin /cmd_vel antes de detener el robot [s] (0.5)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SwerveKinematicsNode(Node):

    def __init__(self):
        super().__init__('swerve_kinematics_node')

        # Geometría del prototipo
        self.r = 0.0275   # radio de la rueda [m]
        self.w = 0.163    # distancia entre ejes de dirección [m]

        # Posición de cada módulo respecto al origen del chasis, ubicado en
        # el punto medio entre los dos ejes de dirección
        self.l1 = (0.0, -self.w / 2.0)  # módulo derecho
        self.l2 = (0.0, self.w / 2.0)   # módulo izquierdo

        # Error de orientación a partir del cual la rueda puede empujar
        # (0.17 rad, unos 10 grados)
        self.umbral_orientado = 0.17

        # Último comando de velocidad recibido
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.ultimo_cmd_vel = None       # instante del último /cmd_vel
        self.detenido_por_timeout = False

        # Último ángulo medido de cada módulo
        self.phi1_real = 0.0
        self.phi2_real = 0.0

        self.steer_pub = self.create_publisher(
            Float64MultiArray, '/steer_position_controller/commands', 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)

        # El cálculo corre a 50 Hz con su propio temporizador, de modo que
        # la condición de orientación se revisa siempre con el ángulo más
        # reciente y no solo cuando llega un /cmd_vel nuevo.
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            'Nodo de cinematica swerve listo, escuchando /cmd_vel')

    def cinematica_modulo(self, vx, vy, omega, l):
        """Calcula el ángulo y la velocidad de rueda de un módulo.

        Recibe la velocidad del chasis y la posición l = (lx, ly) del
        módulo. Devuelve (phi_i [rad], omega_rueda_i [rad/s]).
        """
        lx, ly = l
        vix = vx - omega * ly
        viy = vy + omega * lx

        v_i = math.hypot(vix, viy)
        phi_i = math.atan2(viy, vix)
        omega_rueda_i = v_i / self.r

        return phi_i, omega_rueda_i

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z
        self.ultimo_cmd_vel = self.get_clock().now()
        if self.detenido_por_timeout:
            self.detenido_por_timeout = False
            self.get_logger().info('/cmd_vel recibido de nuevo, retomando')

    def joint_states_callback(self, msg: JointState):
        # Las juntas se buscan por nombre porque el orden en el mensaje no
        # está garantizado.
        for nombre, posicion in zip(msg.name, msg.position):
            if nombre == 'der_steer_joint':
                self.phi1_real = posicion
            elif nombre == 'izq_steer_joint':
                self.phi2_real = posicion

    def control_loop(self):
        if self.ultimo_cmd_vel is not None and not self.detenido_por_timeout:
            espera = (self.get_clock().now() - self.ultimo_cmd_vel).nanoseconds * 1e-9
            if espera > self.cmd_vel_timeout:
                self.vx = self.vy = self.omega = 0.0
                self.detenido_por_timeout = True
                self.get_logger().warn(
                    'Sin /cmd_vel hace mas de %.2f s: robot detenido por seguridad'
                    % self.cmd_vel_timeout)

        phi1, w1 = self.cinematica_modulo(self.vx, self.vy, self.omega, self.l1)
        phi2, w2 = self.cinematica_modulo(self.vx, self.vy, self.omega, self.l2)

        # El ángulo se envía siempre, para que el módulo empiece a girar
        # cuanto antes.
        steer_msg = Float64MultiArray()
        steer_msg.data = [phi1, phi2]
        self.steer_pub.publish(steer_msg)

        # La rueda solo recibe su velocidad cuando el módulo ya está cerca
        # del ángulo objetivo; mientras tanto se envía cero.
        error1 = abs(self.angulo_normalizado(phi1 - self.phi1_real))
        error2 = abs(self.angulo_normalizado(phi2 - self.phi2_real))

        w1_final = w1 if error1 < self.umbral_orientado else 0.0
        w2_final = w2 if error2 < self.umbral_orientado else 0.0

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [w1_final, w2_final]
        self.wheel_pub.publish(wheel_msg)

    @staticmethod
    def angulo_normalizado(angulo):
        """Lleva un ángulo al intervalo [-pi, pi].

        Evita errores grandes falsos cuando el ángulo objetivo y el real
        quedan a lados distintos del salto entre pi y -pi.
        """
        return math.atan2(math.sin(angulo), math.cos(angulo))


def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
