#!/usr/bin/env python3
"""Controlador de punto objetivo en lazo cerrado para el modelo en Gazebo.

Lleva el robot por una secuencia de puntos (x, y, theta). Es la versión en
ROS 2 del programa de control del prototipo, con el mismo algoritmo y los
mismos valores por defecto, para que simulación y prototipo se puedan
comparar.

Cada ciclo hace lo siguiente:
  1. Estima la pose por odometría (cinemática directa a partir del ángulo
     de cada módulo y la velocidad de cada rueda).
  2. Calcula el error de posición y orientación respecto al punto actual.
  3. Revisa si llegó: el error debe quedar dentro de la tolerancia
     durante tiempo_llegada segundos seguidos.
  4. Calcula la velocidad deseada en el marco del mundo con un control
     proporcional, limitada entre v_min y v_max, con omega limitada a
     omega_max y con límites de aceleración. Primero se traslada y después rota en el lugar.
  5. Pasa la velocidad al marco del robot.
  6. Aplica la cinemática inversa a cada módulo.
  7. Elige para cada módulo el giro más corto (si conviene, gira al ángulo
     opuesto e invierte la rueda) y, si está activado, aplica el giro en
     espejo.
  8. Reduce la velocidad de ambas ruedas según el módulo peor alineado.
  9. Limita la velocidad de giro de la referencia de dirección.
Al terminar el recorrido, los módulos vuelven a su 0 más cercano.

Diferencias con el programa del prototipo:
  - El control de bajo nivel de los motores lo hacen los controladores de
    ros2_control, no los PI/PID del prototipo.
  - Se comanda directamente el módulo y la rueda, así que no se usan las
    relaciones de transmisión ni los signos de los motores.
  - La vibración de la referencia (dither) existe para destrabar un módulo
    del prototipo; el modelo no tiene esa fricción y por eso viene apagada.
  - Cada ciclo corresponde a un mensaje de /joint_states (50 Hz en tiempo
    de simulación) y dt se toma del sello de tiempo del mensaje. Así el
    control sigue el reloj de Gazebo aunque la simulación vaya más lenta o
    más rápida que el tiempo real.

El robot llega al punto según la odometría, que supone rodadura sin
deslizamiento. La pose verdadera se graba aparte desde Gazebo para
compararla después.

Uso:
    ros2 launch swerve_control swerve_punto_objetivo.launch.py
    ros2 launch swerve_control swerve_punto_objetivo.launch.py trayectoria:=triangulo
    ros2 launch swerve_control swerve_punto_objetivo.launch.py trayectoria:="0.3 0.2 90"
La última forma recibe puntos propios en grupos de tres números
(x [m], y [m], theta [grados]).

Suscripción:
    /joint_states   sensor_msgs/JointState
Publicaciones:
    /steer_position_controller/commands   ángulos de dirección [der, izq] [rad]
    /wheel_velocity_controller/commands   velocidades de rueda [der, izq] [rad/s]
    /odom                                 pose estimada por odometría

Archivo de salida: ~/ROS_TFG/resultados/sim_<trayectoria>_<fecha>.txt, o el
que indique el parámetro archivo_salida. Tiene una fila por ciclo con las
columnas del encabezado.
"""

import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


# Trayectorias de prueba: lista de puntos (x [m], y [m], theta [grados]).
# En las de ida y vuelta, cada giro de 180 grados se divide en dos de 90
# para que el sentido de giro no quede ambiguo.
TRAYECTORIAS = {
    'cuadrado': [(0.6, 0.0, 0.0), (0.6, 0.6, 0.0), (0.0, 0.6, 0.0), (0.0, 0.0, 0.0)],
    'triangulo': [(0.6, 0.0, 0.0), (0.3, 0.3 * math.sqrt(3.0), 0.0), (0.0, 0.0, 0.0)],
    'cuadrado_horario': [(0.0, 0.6, 0.0), (0.6, 0.6, 0.0), (0.6, 0.0, 0.0), (0.0, 0.0, 0.0)],
    'triangulo_horario': [(0.3, 0.3 * math.sqrt(3.0), 0.0), (0.6, 0.0, 0.0), (0.0, 0.0, 0.0)],
    'ida_vuelta_giro': [(0.6, 0.0, 0.0), (0.6, 0.0, 90.0), (0.6, 0.0, 180.0),
                        (0.0, 0.0, 180.0), (0.0, 0.0, 270.0), (0.0, 0.0, 360.0)],
    'ida_vuelta_giro_horario': [(0.6, 0.0, 0.0), (0.6, 0.0, -90.0), (0.6, 0.0, -180.0),
                                (0.0, 0.0, -180.0), (0.0, 0.0, -270.0), (0.0, 0.0, -360.0)],
}


def angulo_normalizado(angulo):
    """Lleva un ángulo al intervalo [-pi, pi]."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


def saturar(valor, limite):
    """Limita un valor al intervalo [-limite, limite]."""
    return max(-limite, min(limite, valor))


class SwervePuntoObjetivoNode(Node):

    def __init__(self):
        super().__init__('swerve_punto_objetivo_node')

        # Parámetros (mismos valores por defecto que el programa del prototipo)
        p = self.declare_parameter
        p('trayectoria', 'cuadrado')
        p('radio_rueda', 0.0275)       # nominal; el prototipo usa 0.02544 calibrado
        p('track', 0.163)
        p('kp_pos', 1.5)               # Kp = acel_max / v_max
        p('kp_theta', 1.5)
        p('v_max', 0.20)
        p('v_min', 0.04)
        p('omega_max', 0.8)
        p('acel_max', 0.3)
        p('alfa_max', 2.0)
        p('tol_pos', 0.01)
        p('tol_theta_grados', 3.0)
        p('tiempo_llegada', 0.2)       # 10 ciclos de 20 ms
        p('tiempo_max_punto', 30.0)
        p('pausa_entre_puntos', 0.5)
        p('v_muerta', 0.003)
        p('histeresis_grados', 10.0)
        p('angulo_max_empuje_grados', 25.0)
        p('zona_espejo_grados', 25.0)
        p('umbral_giro_espejo_grados', 30.0)
        p('modo_dos_fases', True)
        p('vel_max_direccion_grados', 90.0)
        p('usar_dither', False)
        p('umbral_dither_grados', 2.0)
        p('dither_amplitud', 0.1)
        p('dither_freq_hz', 3.0)
        p('espera_inicial', 8.0)       # [s de simulación] margen para empezar a grabar en Gazebo
        p('archivo_salida', '')

        g = lambda n: self.get_parameter(n).value
        rad = math.pi / 180.0
        self.r = g('radio_rueda')
        self.w = g('track')
        self.kp_pos = g('kp_pos')
        self.kp_theta = g('kp_theta')
        self.v_max = g('v_max')
        self.v_min = g('v_min')
        self.omega_max = g('omega_max')
        self.acel_max = g('acel_max')
        self.alfa_max = g('alfa_max')
        self.tol_pos = g('tol_pos')
        self.tol_theta = g('tol_theta_grados') * rad
        self.tiempo_llegada = g('tiempo_llegada')
        self.tiempo_max_punto = g('tiempo_max_punto')
        self.pausa_entre_puntos = g('pausa_entre_puntos')
        self.v_muerta = g('v_muerta')
        self.histeresis = g('histeresis_grados') * rad
        self.angulo_max_empuje = g('angulo_max_empuje_grados') * rad
        self.zona_espejo = g('zona_espejo_grados') * rad
        self.umbral_giro_espejo = g('umbral_giro_espejo_grados') * rad
        self.modo_dos_fases = g('modo_dos_fases')
        self.vel_max_direccion = g('vel_max_direccion_grados') * rad
        self.usar_dither = g('usar_dither')
        self.umbral_dither = g('umbral_dither_grados') * rad
        self.dither_amplitud = g('dither_amplitud')
        self.dither_freq = g('dither_freq_hz')
        self.espera_inicial = g('espera_inicial')

        # Puntos a recorrer
        nombre_tray = str(g('trayectoria')).strip()
        self.puntos = self.leer_puntos(nombre_tray)
        for k, (x, y, th) in enumerate(self.puntos):
            self.get_logger().info('  Punto %d: x=%.3f m  y=%.3f m  theta=%.1f grados'
                                   % (k + 1, x, y, th / rad))

        # Archivo de salida
        archivo = g('archivo_salida')
        if not archivo:
            carpeta = os.path.expanduser('~/ROS_TFG/resultados')
            os.makedirs(carpeta, exist_ok=True)
            etiqueta = nombre_tray if nombre_tray in TRAYECTORIAS else 'puntos'
            archivo = os.path.join(carpeta, 'sim_%s_%s.txt'
                                   % (etiqueta, datetime.now().strftime('%Y%m%d_%H%M%S')))
        self.archivo = archivo
        self.fich = open(archivo, 'w')
        self.fich.write('t x y theta vx vy omega phi1 phi2 w_rueda_der w_rueda_izq '
                        'x_obj y_obj theta_obj vx_w_cmd vy_w_cmd omega_cmd punto fase\n')

        # Estado. Los sufijos B y C (derecho e izquierdo) son los nombres de
        # los puertos de los motores en el prototipo.
        self.x = self.y = self.theta = 0.0
        self.vx_w_cmd = self.vy_w_cmd = self.omega_cmd = 0.0
        self.phi_cmd_B = self.phi_cmd_C = 0.0   # ángulo objetivo de cada módulo
        self.phi_ref_B = self.phi_ref_C = 0.0   # referencia enviada, con rampa
        self.t0 = None
        self.t_prev = None
        self.estado = 'espera_inicial'
        self.t_estado = 0.0
        self.indice = 0
        self.fase = 1
        self.theta_inicio_punto = 0.0
        self.t_dentro = 0.0
        self.x_obj = self.y_obj = self.theta_obj = 0.0

        self.steer_pub = self.create_publisher(
            Float64MultiArray, '/steer_position_controller/commands', 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(JointState, '/joint_states', self.ciclo, 10)

        self.get_logger().info('Controlador punto-objetivo listo. Grabando en %s' % archivo)
        self.get_logger().info('Arranca %.0f s (de simulacion) despues del primer /joint_states'
                               % self.espera_inicial)

    def leer_puntos(self, texto):
        """Devuelve la lista de puntos (x, y, theta en rad).

        Acepta el nombre de una trayectoria de TRAYECTORIAS o un texto con
        grupos de tres números "x y theta_grados". Si no lo reconoce, usa
        el cuadrado.
        """
        if texto in TRAYECTORIAS:
            return [(x, y, th * math.pi / 180.0) for (x, y, th) in TRAYECTORIAS[texto]]
        try:
            numeros = [float(v) for v in texto.replace(',', ' ').split()]
        except ValueError:
            numeros = []
        if len(numeros) == 0 or len(numeros) % 3 != 0:
            self.get_logger().error(
                'trayectoria "%s" no reconocida. Usar cuadrado, triangulo o '
                '"x y theta x y theta ...". Se usa cuadrado.' % texto)
            return self.leer_puntos('cuadrado')
        return [(numeros[k], numeros[k + 1], numeros[k + 2] * math.pi / 180.0)
                for k in range(0, len(numeros), 3)]

    def calcular_modulo(self, vix, viy, phi_cmd_prev, signo_preferido):
        """Calcula el nuevo ángulo y la velocidad de un módulo (pasos 6 y 7).

        Si el giro necesario supera 90 grados más la histéresis, el módulo
        gira al ángulo opuesto e invierte la rueda. Si signo_preferido no es
        cero y el giro está cerca de 90 grados, se fuerza ese sentido de
        giro (giro en espejo respecto al otro módulo).

        Devuelve (velocidad de rueda [m/s] con signo, nuevo phi_cmd, giro).
        """
        v = math.hypot(vix, viy)
        if v < self.v_muerta:
            return 0.0, phi_cmd_prev, 0.0   # sin velocidad, el módulo no se mueve

        phi_obj = math.atan2(viy, vix)
        delta = angulo_normalizado(phi_obj - phi_cmd_prev)
        delta_alt = angulo_normalizado(delta + math.pi)

        if signo_preferido != 0.0 and abs(abs(delta) - math.pi / 2.0) < self.zona_espejo:
            if delta * signo_preferido < 0.0:
                delta = delta_alt
                v = -v
        elif abs(delta) > math.pi / 2.0 + self.histeresis:
            delta = delta_alt
            v = -v
        return v, phi_cmd_prev + delta, delta

    def factor_alineacion(self, phi_cmd, phi_medido):
        """Factor entre 0 y 1 según qué tan alineado está el módulo.

        Vale 1 cuando el módulo está en su ángulo objetivo y 0 cuando el
        error llega a angulo_max_empuje.
        """
        c_lim = math.cos(self.angulo_max_empuje)
        return max(0.0, (math.cos(phi_cmd - phi_medido) - c_lim) / (1.0 - c_lim))

    def mover_referencias(self, objetivo_B, objetivo_C, dt):
        """Acerca la referencia de dirección al objetivo, a velocidad limitada."""
        paso = self.vel_max_direccion * dt
        self.phi_ref_B += saturar(objetivo_B - self.phi_ref_B, paso)
        self.phi_ref_C += saturar(objetivo_C - self.phi_ref_C, paso)

    def publicar(self, steer_der, steer_izq, w_der, w_izq):
        m = Float64MultiArray()
        m.data = [steer_der, steer_izq]
        self.steer_pub.publish(m)
        m = Float64MultiArray()
        m.data = [w_der, w_izq]
        self.wheel_pub.publish(m)

    def ciclo(self, msg: JointState):
        """Ciclo de control, se ejecuta con cada mensaje de /joint_states."""
        if self.estado == 'terminado':
            return

        # Las juntas se buscan por nombre porque el orden no está garantizado
        datos = {}
        for i, nombre in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            datos[nombre] = (pos, vel)
        try:
            phi1 = datos['der_steer_joint'][0]
            phi2 = datos['izq_steer_joint'][0]
            w1 = datos['der_wheel_joint'][1]
            w2 = datos['izq_wheel_joint'][1]
        except KeyError:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            # En el primer mensaje los comandos arrancan desde el ángulo
            # real de los módulos (en el prototipo esto lo asegura el
            # procedimiento de inicio).
            self.t0 = t
            self.t_prev = t
            self.phi_cmd_B = self.phi_ref_B = phi1
            self.phi_cmd_C = self.phi_ref_C = phi2
            return
        dt = t - self.t_prev
        if dt <= 0.0:
            return          # mensaje repetido o fuera de orden
        dt = min(dt, 0.1)   # protección ante un salto del reloj
        self.t_prev = t
        t_rel = t - self.t0

        # Odometría: velocidad del chasis a partir de los dos módulos e
        # integración de la pose en el marco del mundo
        v1 = w1 * self.r
        v2 = w2 * self.r
        v1x, v1y = v1 * math.cos(phi1), v1 * math.sin(phi1)
        v2x, v2y = v2 * math.cos(phi2), v2 * math.sin(phi2)
        vx_ch = (v1x + v2x) / 2.0
        vy_ch = (v1y + v2y) / 2.0
        om_ch = (v1x - v2x) / self.w
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx_ch * c - vy_ch * s) * dt
        self.y += (vx_ch * s + vy_ch * c) * dt
        self.theta += om_ch * dt

        # Máquina de estados:
        # espera_inicial -> punto -> pausa -> punto ... -> volver_cero -> terminado
        self.t_estado += dt
        if self.estado == 'espera_inicial':
            self.publicar(self.phi_ref_B, self.phi_ref_C, 0.0, 0.0)
            if self.t_estado >= self.espera_inicial:
                self.empezar_punto(0)

        elif self.estado == 'punto':
            self.paso_punto(dt, t_rel, phi1, phi2)

        elif self.estado == 'pausa':
            self.vx_w_cmd = self.vy_w_cmd = self.omega_cmd = 0.0
            self.mover_referencias(self.phi_cmd_B, self.phi_cmd_C, dt)
            self.publicar(self.phi_ref_B, self.phi_ref_C, 0.0, 0.0)
            if self.t_estado >= self.pausa_entre_puntos:
                if self.indice + 1 < len(self.puntos):
                    self.empezar_punto(self.indice + 1)
                else:
                    self.empezar_volver_cero()

        elif self.estado == 'volver_cero':
            self.mover_referencias(self.cero_B, self.cero_C, dt)
            self.publicar(self.phi_ref_B, self.phi_ref_C, 0.0, 0.0)
            en_cero = (self.phi_ref_B == self.cero_B and self.phi_ref_C == self.cero_C
                       and abs(angulo_normalizado(self.cero_B - phi1)) < 0.02
                       and abs(angulo_normalizado(self.cero_C - phi2)) < 0.02)
            if en_cero or self.t_estado >= 6.0:
                self.terminar(phi1, phi2)

        self.publicar_odom(msg, vx_ch, vy_ch, om_ch)
        if self.fich.closed:
            return
        self.fich.write('%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d\n' % (
            t_rel, self.x, self.y, self.theta, vx_ch, vy_ch, om_ch, phi1, phi2, w1, w2,
            self.x_obj, self.y_obj, self.theta_obj,
            self.vx_w_cmd, self.vy_w_cmd, self.omega_cmd,
            self.indice + 1, self.fase if self.estado == 'punto' else 0))

    def empezar_punto(self, k):
        self.indice = k
        self.x_obj, self.y_obj, self.theta_obj = self.puntos[k]
        self.theta_inicio_punto = self.theta
        self.fase = 1 if self.modo_dos_fases else 2
        self.t_dentro = 0.0
        self.estado = 'punto'
        self.t_estado = 0.0
        self.get_logger().info('Punto %d: yendo a (%.3f, %.3f) theta=%.1f...'
                               % (k + 1, self.x_obj, self.y_obj, math.degrees(self.theta_obj)))

    def paso_punto(self, dt, t_rel, phi1, phi2):
        """Un ciclo del control hacia el punto actual (pasos 2 a 9)."""
        # Error respecto al punto objetivo
        ex = self.x_obj - self.x
        ey = self.y_obj - self.y
        dist = math.hypot(ex, ey)
        eth = angulo_normalizado(self.theta_obj - self.theta)

        # En la fase 1 se mantiene la orientación con que empezó el punto;
        # en la fase 2 se corrige hacia la orientación objetivo.
        if self.fase == 1 and dist < self.tol_pos:
            self.fase = 2
        eth_control = (angulo_normalizado(self.theta_inicio_punto - self.theta)
                       if self.fase == 1 else eth)

        # Llegada o tiempo máximo agotado
        if dist < self.tol_pos and abs(eth) < self.tol_theta:
            self.t_dentro += dt
            if self.t_dentro >= self.tiempo_llegada - 1e-9:
                self.fin_punto(llego=True)
                return
        else:
            self.t_dentro = 0.0
        if self.t_estado >= self.tiempo_max_punto:
            self.fin_punto(llego=False)
            return

        # Velocidad deseada en el marco del mundo
        vx_w = self.kp_pos * ex
        vy_w = self.kp_pos * ey
        v_norma = math.hypot(vx_w, vy_w)
        if dist < self.tol_pos:
            vx_w = vy_w = 0.0
        elif v_norma > self.v_max:
            vx_w *= self.v_max / v_norma
            vy_w *= self.v_max / v_norma
        elif 0.0 < v_norma < self.v_min:
            vx_w *= self.v_min / v_norma
            vy_w *= self.v_min / v_norma
        if self.fase == 2 and self.modo_dos_fases and abs(eth_control) >= self.tol_theta:
            vx_w = vy_w = 0.0
        omega = saturar(self.kp_theta * eth_control, self.omega_max)
        if self.fase == 2 and abs(eth_control) < self.tol_theta:
            omega = 0.0

        # Límite de aceleración lineal y angular
        self.vx_w_cmd += saturar(vx_w - self.vx_w_cmd, self.acel_max * dt)
        self.vy_w_cmd += saturar(vy_w - self.vy_w_cmd, self.acel_max * dt)
        self.omega_cmd += saturar(omega - self.omega_cmd, self.alfa_max * dt)

        # Paso al marco del robot
        c, s = math.cos(self.theta), math.sin(self.theta)
        vx = c * self.vx_w_cmd + s * self.vy_w_cmd
        vy = -s * self.vx_w_cmd + c * self.vy_w_cmd

        # Cinemática inversa (módulo 1 = derecho, en y = -w/2)
        v1x = vx + self.omega_cmd * self.w / 2.0
        v2x = vx - self.omega_cmd * self.w / 2.0

        # Ángulo de cada módulo. Si el módulo derecho gira más que
        # umbral_giro_espejo, el izquierdo gira en sentido contrario.
        v1, self.phi_cmd_B, delta_B = self.calcular_modulo(v1x, vy, self.phi_cmd_B, 0.0)
        signo_pref_C = 0.0
        if abs(delta_B) > self.umbral_giro_espejo:
            signo_pref_C = -1.0 if delta_B > 0.0 else 1.0
        v2, self.phi_cmd_C, _ = self.calcular_modulo(v2x, vy, self.phi_cmd_C, signo_pref_C)

        # Ambas ruedas se escalan con el factor del módulo peor alineado.
        # La velocidad comandada también se escala, para que la rampa de
        # aceleración parta de la velocidad que reciben las ruedas. Sin esto,
        # la velocidad seguía subiendo mientras los módulos giraban y las
        # ruedas arrancaban de golpe al alinearse.
        f = min(self.factor_alineacion(self.phi_cmd_B, phi1),
                self.factor_alineacion(self.phi_cmd_C, phi2))
        v1 *= f
        v2 *= f
        self.vx_w_cmd *= f
        self.vy_w_cmd *= f
        self.omega_cmd *= f

        # Rampa de la referencia de dirección y dither opcional
        self.mover_referencias(self.phi_cmd_B, self.phi_cmd_C, dt)
        d_B = d_C = 0.0
        if self.usar_dither:
            onda = self.dither_amplitud * math.sin(2.0 * math.pi * self.dither_freq * t_rel)
            if abs(angulo_normalizado(self.phi_ref_B - phi1)) > self.umbral_dither:
                d_B = onda
            if abs(angulo_normalizado(self.phi_ref_C - phi2)) > self.umbral_dither:
                d_C = onda

        self.publicar(self.phi_ref_B + d_B, self.phi_ref_C + d_C, v1 / self.r, v2 / self.r)

    def fin_punto(self, llego):
        self.vx_w_cmd = self.vy_w_cmd = self.omega_cmd = 0.0
        ex = self.x_obj - self.x
        ey = self.y_obj - self.y
        eth = angulo_normalizado(self.theta_obj - self.theta)
        if llego:
            inicio = 'Punto %d: LLEGO en %.2f s.' % (self.indice + 1, self.t_estado)
        else:
            inicio = 'Punto %d: ATENCION, tope de %.0f s sin llegar.' % (
                self.indice + 1, self.tiempo_max_punto)
        self.get_logger().info('%s Error final (odometria): %.1f mm, %.2f grados. '
                               'Pose estimada: x=%.3f y=%.3f theta=%.1f' % (
                                   inicio, math.hypot(ex, ey) * 1000.0, math.degrees(eth),
                                   self.x, self.y, math.degrees(self.theta)))
        self.estado = 'pausa'
        self.t_estado = 0.0
        self.publicar(self.phi_ref_B, self.phi_ref_C, 0.0, 0.0)

    def empezar_volver_cero(self):
        # Se busca el múltiplo de 360 grados más cercano al ángulo actual
        self.cero_B = self.phi_cmd_B - angulo_normalizado(self.phi_cmd_B)
        self.cero_C = self.phi_cmd_C - angulo_normalizado(self.phi_cmd_C)
        self.estado = 'volver_cero'
        self.t_estado = 0.0
        self.get_logger().info('Volviendo modulos a 0 grados...')

    def terminar(self, phi1, phi2):
        self.estado = 'terminado'
        self.fich.close()
        self.get_logger().info('Modulos: der=%.1f izq=%.1f grados (respecto a su 0)' % (
            math.degrees(angulo_normalizado(phi1 - self.cero_B)),
            math.degrees(angulo_normalizado(phi2 - self.cero_C))))
        self.get_logger().info('Recorrido terminado. Pose final (odometria): '
                               'x=%.3f y=%.3f theta=%.1f' % (
                                   self.x, self.y, math.degrees(self.theta)))
        self.get_logger().info('Trayectoria guardada en %s' % self.archivo)

    def publicar_odom(self, msg, vx, vy, om):
        """Publica la pose estimada en /odom (orientación como cuaternión en z)."""
        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        o.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        o.twist.twist.linear.x = vx
        o.twist.twist.linear.y = vy
        o.twist.twist.angular.z = om
        self.odom_pub.publish(o)

    def destroy_node(self):
        if not self.fich.closed:
            self.fich.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwervePuntoObjetivoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
