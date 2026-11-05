#!/usr/bin/env python3
"""Puente de la sombra digital entre el EV3 y ROS 2.

Se conecta por TCP al programa swerve_gemelo.c que corre en el EV3.

Hacia el EV3 envía, a 10 Hz, la velocidad pedida con el joystick (/joy)
o por /cmd_vel, como una línea "vx,vy,omega,mision".

Del EV3 recibe su estado y lo publica en /joint_states, en la
transformación odom -> base_footprint, en /odom y en /gemelo/trayectoria
para verlo en RViz. Lo que se dibuja es lo que miden los encoders del
EV3, no lo que se le pide.

Cada línea de estado del EV3 trae 11 números separados por espacios
    t x y theta vx vy omega phi1 phi2 rueda1 rueda2
donde 1 es el módulo derecho y 2 el izquierdo.

Seguridad:
  - Si no se mantiene apretado el botón de hombre muerto, se envía
    velocidad cero.
  - Si no llegan mensajes de /joy ni /cmd_vel durante timeout_mando
    segundos, se envía velocidad cero.
  - El EV3 también frena solo si deja de recibir órdenes por 0.5 s.
  - Si se corta la conexión, el puente intenta reconectarse cada segundo.
"""

import math
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster


N_CAMPOS = 11


def zona_muerta(valor, zona):
    """Ignora el ruido del joystick cerca del centro y reescala el resto a [-1, 1]."""
    if abs(valor) < zona:
        return 0.0
    signo = 1.0 if valor > 0.0 else -1.0
    return signo * (abs(valor) - zona) / (1.0 - zona)


def leer_eje(msg, indice):
    if 0 <= indice < len(msg.axes):
        return float(msg.axes[indice])
    return 0.0


def leer_boton(msg, indice):
    if 0 <= indice < len(msg.buttons):
        return int(msg.buttons[indice])
    return 0


def interpretar_linea(linea):
    """Convierte una línea del EV3 en un diccionario. Devuelve None si no es válida."""
    partes = linea.split()
    if len(partes) != N_CAMPOS:
        return None
    try:
        v = [float(p) for p in partes]
    except ValueError:
        return None
    nombres = ['t', 'x', 'y', 'theta', 'vx', 'vy', 'omega',
               'phi1', 'phi2', 'rueda1', 'rueda2']
    return dict(zip(nombres, v))


def cuaternion_yaw(theta):
    """Cuaternión (x, y, z, w) de un giro theta alrededor de Z."""
    return 0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)


class GemeloPuente(Node):

    def __init__(self):
        super().__init__('gemelo_puente')

        # Parámetros (se pueden cambiar con -p o desde el launch)
        self.declare_parameter('ev3_ip', '192.168.1.103')
        self.declare_parameter('ev3_puerto', 6000)
        # Joystick (mismos botones que el cliente de joystick del laboratorio)
        self.declare_parameter('eje_vx', 1)          # palanca izquierda adelante/atrás
        self.declare_parameter('eje_vy', 0)          # palanca izquierda izquierda/derecha
        self.declare_parameter('eje_omega', 3)       # palanca derecha izquierda/derecha
        self.declare_parameter('boton_hombre_muerto', 6)
        self.declare_parameter('boton_terminar', 10)
        self.declare_parameter('v_max', 0.15)        # [m/s]   palanca a fondo
        self.declare_parameter('omega_max', 0.6)     # [rad/s] palanca a fondo
        self.declare_parameter('zona_muerta', 0.10)
        self.declare_parameter('timeout_mando', 0.5)  # [s]
        # Nombres de las articulaciones del URDF
        self.declare_parameter('joint_steer_der', 'der_steer_joint')
        self.declare_parameter('joint_steer_izq', 'izq_steer_joint')
        self.declare_parameter('joint_rueda_der', 'der_wheel_joint')
        self.declare_parameter('joint_rueda_izq', 'izq_wheel_joint')
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_footprint')
        self.declare_parameter('paso_trayectoria', 0.01)  # [m] distancia entre puntos del rastro

        p = self.get_parameter
        self.ev3_ip = p('ev3_ip').value
        self.ev3_puerto = int(p('ev3_puerto').value)
        self.eje_vx = int(p('eje_vx').value)
        self.eje_vy = int(p('eje_vy').value)
        self.eje_omega = int(p('eje_omega').value)
        self.boton_hm = int(p('boton_hombre_muerto').value)
        self.boton_fin = int(p('boton_terminar').value)
        self.v_max = float(p('v_max').value)
        self.omega_max = float(p('omega_max').value)
        self.zona = float(p('zona_muerta').value)
        self.timeout_mando = float(p('timeout_mando').value)
        self.joints = [p('joint_steer_der').value, p('joint_steer_izq').value,
                       p('joint_rueda_der').value, p('joint_rueda_izq').value]
        self.frame_odom = p('frame_odom').value
        self.frame_base = p('frame_base').value
        self.paso_tray = float(p('paso_trayectoria').value)

        # Estado del mando
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.mision = 0
        self.boton_fin_antes = 0
        self.t_ultimo_mando = 0.0

        # Comunicación con el EV3
        self.sock = None
        self.buffer = ''
        self.t_ultimo_intento = 0.0
        self.lineas_recibidas = 0

        # Suscripciones y publicaciones
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_tray = self.create_publisher(Path, '/gemelo/trayectoria', 10)
        self.tf = TransformBroadcaster(self)

        self.trayectoria = Path()
        self.trayectoria.header.frame_id = self.frame_odom

        self.create_timer(0.1, self.enviar_orden)    # 10 Hz hacia el EV3
        self.create_timer(0.02, self.recibir_estado)  # 50 Hz desde el EV3
        self.create_timer(5.0, self.informar)

        # Al arrancar, el modelo se dibuja en el origen con los módulos rectos
        self.publicar_estado({'t': 0.0, 'x': 0.0, 'y': 0.0, 'theta': 0.0,
                              'vx': 0.0, 'vy': 0.0, 'omega': 0.0,
                              'phi1': 0.0, 'phi2': 0.0, 'rueda1': 0.0, 'rueda2': 0.0})

        self.get_logger().info(
            f'Puente del gemelo listo. EV3 en {self.ev3_ip}:{self.ev3_puerto}. '
            f'Mantener el botón {self.boton_hm} (hombre muerto) para mover.')

    def cb_joy(self, msg):
        self.t_ultimo_mando = time.monotonic()
        fin = leer_boton(msg, self.boton_fin)
        if leer_boton(msg, self.boton_hm):
            self.vx = self.v_max * zona_muerta(leer_eje(msg, self.eje_vx), self.zona)
            self.vy = self.v_max * zona_muerta(leer_eje(msg, self.eje_vy), self.zona)
            self.omega = self.omega_max * zona_muerta(leer_eje(msg, self.eje_omega), self.zona)
            if fin and not self.boton_fin_antes:
                self.mision = 1   # solo al apretar, no mientras se mantiene
        else:
            self.vx = self.vy = self.omega = 0.0
        self.boton_fin_antes = fin

    def cb_cmd_vel(self, msg):
        # Alternativa sin joystick (por ejemplo, teleop_twist_keyboard)
        self.t_ultimo_mando = time.monotonic()
        self.vx = max(-self.v_max, min(self.v_max, msg.linear.x))
        self.vy = max(-self.v_max, min(self.v_max, msg.linear.y))
        self.omega = max(-self.omega_max, min(self.omega_max, msg.angular.z))

    def conectar(self):
        ahora = time.monotonic()
        if ahora - self.t_ultimo_intento < 1.0:
            return
        self.t_ultimo_intento = ahora
        try:
            s = socket.create_connection((self.ev3_ip, self.ev3_puerto), timeout=0.5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.setblocking(False)
            self.sock = s
            self.buffer = ''
            self.mision = 0
            self.trayectoria.poses = []
            self.get_logger().info('Conectado al EV3.')
        except OSError as e:
            self.get_logger().warn(
                f'No se pudo conectar al EV3 ({e}). ¿Está corriendo ./swerve_gemelo? '
                'Reintentando...', throttle_duration_sec=5.0)

    def desconectar(self, motivo):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.get_logger().warn(f'{motivo} Esperando a que vuelva el EV3...')

    def enviar_orden(self):
        if self.sock is None:
            self.conectar()
            return
        vx, vy, omega = self.vx, self.vy, self.omega
        if time.monotonic() - self.t_ultimo_mando > self.timeout_mando:
            vx = vy = omega = 0.0
        orden = f'{vx:.4f},{vy:.4f},{omega:.4f},{self.mision}\n'
        try:
            self.sock.sendall(orden.encode())
        except BlockingIOError:
            pass  # buffer lleno, se envía en el próximo ciclo
        except OSError as e:
            self.desconectar(f'Error enviando al EV3 ({e}).')
            return
        if self.mision:
            self.get_logger().info('Orden de terminar enviada al EV3.')
            self.mision = 0

    def recibir_estado(self):
        if self.sock is None:
            return
        try:
            datos = self.sock.recv(4096)
        except BlockingIOError:
            return
        except OSError as e:
            self.desconectar(f'Error leyendo del EV3 ({e}).')
            return
        if not datos:
            self.desconectar('El EV3 cerró la conexión.')
            return
        self.buffer += datos.decode(errors='ignore')
        *lineas, self.buffer = self.buffer.split('\n')
        estado = None
        for linea in lineas:
            e = interpretar_linea(linea)
            if e is not None:
                estado = e
                self.lineas_recibidas += 1
        if estado is not None:
            self.publicar_estado(estado)  # solo la última, que es la más reciente

    def informar(self):
        if self.sock is not None:
            hz = self.lineas_recibidas / 5.0
            self.get_logger().info(f'Recibiendo estado del EV3 a {hz:.0f} Hz.')
        self.lineas_recibidas = 0

    def publicar_estado(self, e):
        ahora = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = ahora
        js.name = list(self.joints)
        js.position = [e['phi1'], e['phi2'], e['rueda1'], e['rueda2']]
        self.pub_joint.publish(js)

        qx, qy, qz, qw = cuaternion_yaw(e['theta'])

        tf = TransformStamped()
        tf.header.stamp = ahora
        tf.header.frame_id = self.frame_odom
        tf.child_frame_id = self.frame_base
        tf.transform.translation.x = e['x']
        tf.transform.translation.y = e['y']
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

        od = Odometry()
        od.header.stamp = ahora
        od.header.frame_id = self.frame_odom
        od.child_frame_id = self.frame_base
        od.pose.pose.position.x = e['x']
        od.pose.pose.position.y = e['y']
        od.pose.pose.orientation = tf.transform.rotation
        od.twist.twist.linear.x = e['vx']
        od.twist.twist.linear.y = e['vy']
        od.twist.twist.angular.z = e['omega']
        self.pub_odom.publish(od)

        # Rastro del recorrido, un punto nuevo cada paso_trayectoria metros
        poses = self.trayectoria.poses
        if (not poses or math.hypot(e['x'] - poses[-1].pose.position.x,
                                    e['y'] - poses[-1].pose.position.y) >= self.paso_tray):
            ps = PoseStamped()
            ps.header.frame_id = self.frame_odom
            ps.header.stamp = ahora
            ps.pose = od.pose.pose
            poses.append(ps)
            self.trayectoria.header.stamp = ahora
            self.pub_tray.publish(self.trayectoria)


def main(args=None):
    rclpy.init(args=args)
    nodo = GemeloPuente()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo.sock is not None:
            try:
                nodo.sock.close()  # el EV3 detecta el corte y frena solo
            except OSError:
                pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
