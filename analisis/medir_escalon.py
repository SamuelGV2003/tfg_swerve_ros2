#!/usr/bin/env python3
"""Respuesta al escalón del controlador de punto objetivo en Gazebo.

Usa dos archivos de la misma corrida:
    gz_...txt    pose verdadera del robot, grabada con
                 gz topic -e -t /world/empty/dynamic_pose/info > gz_...txt
    sim_...txt   archivo de salida de swerve_punto_objetivo_node

A partir del instante en que el nodo empieza a ir al punto, calcula:
    Ts     tiempo desde el cual la posición queda dentro de la banda de
           ±2 % alrededor del valor final y ya no sale
    %OS    sobreimpulso respecto al valor final, en %
    error estacionario = punto pedido - valor final
Compara el resultado con un modelo ideal del mismo controlador (robot sin
dinámica ni deslizamiento, igual que control_punto_objetivo.m) y guarda una
gráfica de ambas respuestas. Criterio de aceptación: Ts <= 5 s y %OS <= 20 %.

Uso:
    python3 medir_escalon.py gz.txt sim.txt direccion L [Kp]
        direccion   x (escalón hacia adelante) o y (escalón lateral)
        L           distancia del escalón [m]
        Kp          ganancia usada en la corrida, para el modelo ideal (1.0)
    Ejemplo: python3 medir_escalon.py gz_escalon_y_1.txt sim_escalon_y_1.txt y 0.5 1.5
"""
import math
import re
import sys
import numpy as np

MODELO = 'swerve_robot'
UMBRAL_MOV = 0.005    # [m] desplazamiento a partir del cual el robot se considera en movimiento
MARGEN_FIN = 0.4      # [s] tiempo analizado después de llegar, antes de que los módulos vuelvan a 0


def _yaw(bloque, num):
    """Ángulo de giro sobre Z [grados] a partir del cuaternión de un bloque de pose."""
    s = re.search(r'orientation\s*{([^}]*)}', bloque)
    q = {}
    for eje in 'xyzw':
        v = re.search(eje + r':\s*' + num, s.group(1)) if s else None
        q[eje] = float(v.group(1)) if v else 0.0
    if q['x'] == q['y'] == q['z'] == q['w'] == 0.0:
        q['w'] = 1.0   # Gazebo omite los campos que valen 0
    return math.degrees(math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                                   1 - 2 * (q['y'] ** 2 + q['z'] ** 2)))


def leer_gz(ruta):
    """Lee la grabación de Gazebo.

    Devuelve t, x, y del robot y el ángulo del módulo derecho [grados],
    ordenados por tiempo y sin muestras repetidas.
    """
    texto = open(ruta).read()
    num = r'(-?[\d.eE+-]+)'
    t, x, y, phi = [], [], [], []
    for msg in texto.split('header {')[1:]:
        m_t = re.search(r'stamp\s*{\s*(?:sec:\s*(\d+))?\s*(?:nsec:\s*(\d+))?', msg)
        m_p = re.search(r'name:\s*"%s"(.*?)(?=\npose\s*{|\Z)' % MODELO, msg, re.S)
        if not m_t or not m_p:
            continue
        s = re.search(r'position\s*{([^}]*)}', m_p.group(1))
        px = py = 0.0
        if s:
            vx = re.search(r'x:\s*' + num, s.group(1))
            vy = re.search(r'y:\s*' + num, s.group(1))
            px = float(vx.group(1)) if vx else 0.0
            py = float(vy.group(1)) if vy else 0.0
        m_s = re.search(r'name:\s*"der_steer_link"(.*?)(?=\npose\s*{|\Z)', msg, re.S)
        t.append(int(m_t.group(1) or 0) + int(m_t.group(2) or 0) * 1e-9)
        x.append(px)
        y.append(py)
        phi.append(_yaw(m_s.group(1), num) if m_s else 0.0)
    t, x, y, phi = np.array(t), np.array(x), np.array(y), np.array(phi)
    o = np.argsort(t)
    t, x, y, phi = t[o], x[o], y[o], phi[o]
    _, u = np.unique(t, return_index=True)
    return t[u], x[u], y[u], phi[u]


def leer_sim(ruta):
    """Lee el archivo del nodo. Devuelve t, x, y, fase y phi1 [grados]."""
    d = np.loadtxt(ruta, skiprows=1)
    cab = open(ruta).readline().split()
    col = {n: i for i, n in enumerate(cab)}
    return (d[:, col['t']], d[:, col['x']], d[:, col['y']], d[:, col['fase']],
            np.degrees(d[:, col['phi1']]))


def modelo(L, direccion, kp=1.0, v_max=0.20, v_min=0.04, acel=0.3, tol=0.01, t_lleg=0.2,
           v_muerta=0.003, hist=10, ang_emp=25, vel_dir=90, dt=0.02, t_fin=10):
    """Simula el escalón con el mismo controlador sobre un robot ideal.

    Usa los mismos parámetros por defecto que el nodo. Devuelve el tiempo y
    la posición a lo largo del escalón.
    """
    def sat(v, l):
        return max(-l, min(l, v))

    def nor(a):
        return math.atan2(math.sin(a), math.cos(a))
    xo, yo = (L, 0.0) if direccion == 'x' else (0.0, L)
    x = y = vxc = vyc = phi_cmd = phi_ref = 0.0
    t_dentro, llego = 0.0, None
    cl, h, paso = math.cos(math.radians(ang_emp)), math.radians(hist), math.radians(vel_dir) * dt
    T, P = [], []
    for k in range(int(t_fin / dt) + 1):
        ex, ey = xo - x, yo - y
        d = math.hypot(ex, ey)
        if llego is None:
            if d < tol:
                t_dentro += dt
                if t_dentro >= t_lleg - 1e-9:
                    llego = k * dt
            else:
                t_dentro = 0.0
        vx = vy = 0.0
        if llego is None:
            vx, vy = kp * ex, kp * ey
            vn = math.hypot(vx, vy)
            if d < tol:
                vx = vy = 0.0
            elif vn > v_max:
                vx, vy = vx * v_max / vn, vy * v_max / vn
            elif 0 < vn < v_min:
                vx, vy = vx * v_min / vn, vy * v_min / vn
        vxc += sat(vx - vxc, acel * dt)
        vyc += sat(vy - vyc, acel * dt)
        if llego is not None:
            vxc = vyc = 0.0
        v = math.hypot(vxc, vyc)
        if v < v_muerta:
            v = 0.0
        else:
            de = nor(math.atan2(vyc, vxc) - phi_cmd)
            if abs(de) > math.pi / 2 + h:
                de, v = nor(de + math.pi), -v
            phi_cmd += de
        f = max(0.0, (math.cos(phi_cmd - phi_ref) - cl) / (1 - cl))
        vxc, vyc = vxc * f, vyc * f   # la rampa parte de la velocidad que reciben las ruedas
        phi_ref += sat(phi_cmd - phi_ref, paso)
        x += v * f * math.cos(phi_ref) * dt
        y += v * f * math.sin(phi_ref) * dt
        T.append(k * dt)
        P.append(x if direccion == 'x' else y)
    return np.array(T), np.array(P)


def ts_os(t, p, L, ventana_final=0.3):
    """Calcula Ts, %OS, valor final y error estacionario de una respuesta.

    Ts se mide con una banda de ±2 % alrededor del valor final de la
    respuesta (promedio de los últimos ventana_final segundos), según la
    definición de Nise. La diferencia entre el valor final y el punto
    pedido se reporta aparte como error estacionario.
    """
    p_final = p[t >= t[-1] - ventana_final].mean()
    fuera = np.where(np.abs(p - p_final) > 0.02 * abs(p_final))[0]
    if len(fuera) == 0:
        ts = 0.0
    elif fuera[-1] + 1 < len(t):
        ts = t[fuera[-1] + 1]
    else:
        ts = float('nan')
    os_ = max(0.0, (p.max() - p_final) / abs(p_final) * 100.0)
    return ts, os_, p_final, L - p_final


def main():
    if len(sys.argv) not in (5, 6):
        print(__doc__)
        sys.exit(1)
    ruta_gz, ruta_sim, direccion, L = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
    kp = float(sys.argv[5]) if len(sys.argv) == 6 else 1.0

    tg, xg, yg, phig = leer_gz(ruta_gz)
    ts_n, xs, ys, fase, phis = leer_sim(ruta_sim)
    if len(tg) < 10:
        sys.exit('No hay suficientes poses de "%s" en %s' % (MODELO, ruta_gz))

    # Posición a lo largo del escalón (verdadera y según la odometría del nodo)
    pg = (xg - xg[0]) if direccion == 'x' else (yg - yg[0])
    ps = xs if direccion == 'x' else ys

    # Sincronización de los dos archivos. En el escalón lateral se usa el
    # instante en que el módulo derecho pasa por la mitad de su giro, porque
    # mientras los módulos giran el chasis se arrastra un poco y el arranque
    # no es un buen punto de referencia. En el escalón hacia adelante los
    # módulos no giran y se usa el instante en que el robot empieza a moverse.
    giro = phis.max() - phis[0]
    if abs(giro) > 20.0:
        mitad = phis[0] + giro / 2.0
        desfase = tg[np.argmax(phig - phig[0] > giro / 2.0)] - ts_n[np.argmax(phis > mitad)]
        metodo = 'giro del modulo'
    else:
        i_g = np.argmax(np.abs(pg) > UMBRAL_MOV)
        i_s = np.argmax(np.abs(ps) > UMBRAL_MOV)
        desfase = tg[i_g] - ts_n[i_s]
        metodo = 'arranque del robot'
    perp = (yg - yg[0]) if direccion == 'x' else (xg - xg[0])

    # Intervalo analizado: desde que el nodo empieza a ir al punto hasta poco después de llegar
    i_ini = np.argmax(fase > 0)
    t_orden = ts_n[i_ini] + desfase
    despues = np.where(fase[i_ini:] == 0)[0]
    t_fin = (ts_n[i_ini + despues[0]] + desfase + MARGEN_FIN) if len(despues) else tg[-1]

    v = (tg >= t_orden) & (tg <= t_fin)
    t = tg[v] - t_orden
    p = pg[v]
    ts, os_, pf, ess = ts_os(t, p, L)

    tm, pm = modelo(L, direccion, kp=kp)
    ts_m, os_m, pf_m, ess_m = ts_os(tm, pm, L)

    print('Escalon de %.2f m en %s, Kp = %.2f  (%s)' % (L, direccion, kp, ruta_gz))
    print('  Gazebo : Ts = %.2f s   %%OS = %.2f %%   valor final = %.4f m   error estacionario = %.1f mm'
          % (ts, os_, pf, ess * 1000))
    print('  Modelo : Ts = %.2f s   %%OS = %.2f %%   valor final = %.4f m   error estacionario = %.1f mm'
          % (ts_m, os_m, pf_m, ess_m * 1000))
    print('  Desvio lateral final = %.1f mm   minimo durante el arranque = %.1f mm   (sincronizado por %s)'
          % (perp[v][-1] * 1000, min(0.0, p.min()) * 1000, metodo))
    print('  Indicador: Ts <= 5 s y %%OS <= 20 %%  ->  %s'
          % ('CUMPLE' if (ts <= 5.0 and os_ <= 20.0) else 'NO CUMPLE'))

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot(t, p, 'b', lw=1.6, label='Gazebo')
        ax.plot(tm, pm, 'k--', lw=1.2, label='Modelo ideal')
        ax.axhline(L, color='0.3', lw=0.8, label='Punto pedido')
        ax.axhspan(0.98 * pf, 1.02 * pf, color='0.85', zorder=0, label='Banda de ±2 % del valor final')
        if not math.isnan(ts):
            ax.axvline(ts, color='r', ls='--', lw=1, label='$T_s$ = %.2f s' % ts)
        ax.set_xlabel('t [s]')
        ax.set_ylabel('p(t) [m]')
        ax.set_xlim(0, max(t[-1], tm[-1] if ts_m != ts_m else ts_m + 2))   # ts_m != ts_m: Ts del modelo es NaN
        ax.grid(alpha=0.4)
        ax.legend(loc='lower right')
        salida = ruta_gz.rsplit('.', 1)[0] + '_respuesta.png'
        fig.tight_layout()
        fig.savefig(salida, dpi=150)
        print('  Grafica guardada en %s' % salida)
    except ImportError:
        print('  (matplotlib no instalado: no se genero la grafica)')


if __name__ == '__main__':
    main()
