#!/usr/bin/env python3
"""Análisis de una corrida de trayectoria: teórica, odometría y pose verdadera.

Entradas:
    archivo_nodo   archivo de salida de swerve_punto_objetivo_node, con la
                   pose según la odometría
    archivo_gz     grabación de la pose verdadera en Gazebo, hecha con
                   gz topic -e -t /world/empty/dynamic_pose/info > archivo_gz

La pose verdadera se expresa respecto a la pose inicial del robot, igual
que la odometría. Los relojes de los dos archivos se sincronizan buscando el
desfase con el que mejor se superponen ambas trayectorias.

Resultados:
    - Error final de posición y orientación al llegar al último punto.
    - Error a lo largo del camino: distancia de cada muestra a la
      trayectoria teórica (tramos rectos entre puntos), como RMS y máximo.
    - Error al terminar cada punto (pose verdadera).
    - Diferencia entre la odometría y la pose verdadera al llegar.
Se imprimen en la terminal y se guarda una figura PNG junto al archivo del nodo.

Uso:
    python3 analizar_trayectoria.py sim_cuadrado_1.txt gz_cuadrado_1.txt --trayectoria cuadrado
"""

import argparse
import math
import os
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Deben ser las mismas trayectorias definidas en swerve_punto_objetivo_node.
# (x [m], y [m], theta [grados])
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
UMBRAL_MOVIMIENTO = 0.005   # [m/s] velocidad a partir de la cual el robot se considera en movimiento


def leer_gz(ruta, modelo='swerve_robot'):
    """Lee la grabación de Gazebo y devuelve t, x, y, theta del robot.

    La pose se expresa respecto a la primera muestra, en el mismo marco
    que la odometría.
    """
    texto = open(ruta).read()
    T, X, Y, TH = [], [], [], []
    for bloque in texto.split('header {')[1:]:
        cab = bloque.split('pose {')[0]
        s = re.search(r'\bsec: (\d+)', cab)
        ns = re.search(r'nsec: (\d+)', cab)
        t = (int(s.group(1)) if s else 0) + (int(ns.group(1)) if ns else 0) * 1e-9
        i = bloque.find('name: "%s"' % modelo)
        if i < 0:
            continue
        pose = bloque[i:].split('pose {')[0]

        def valor(seccion, k):
            sec = re.search(seccion + r' \{([^}]*)\}', pose)
            if not sec:
                return 0.0
            m = re.search(r'\b' + k + r': ([-\deE.+]+)', sec.group(1))
            return float(m.group(1)) if m else 0.0

        qx, qy, qz, qw = (valor('orientation', k) for k in 'xyzw')
        if qx == qy == qz == qw == 0.0:
            qw = 1.0
        T.append(t)
        X.append(valor('position', 'x'))
        Y.append(valor('position', 'y'))
        TH.append(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    T, X, Y, TH = map(np.array, (T, X, Y, TH))
    TH = np.unwrap(TH)
    c, s = math.cos(-TH[0]), math.sin(-TH[0])
    dx, dy = X - X[0], Y - Y[0]
    return T, c * dx - s * dy, s * dx + c * dy, TH - TH[0]


def arranque(t, x, y):
    """Instante en que el robot empieza a moverse."""
    v = np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-9)
    idx = np.where(v > UMBRAL_MOVIMIENTO)[0]
    return t[idx[0]] if len(idx) else t[0]


def alinear(ts, xs, ys, tg, xg, yg):
    """Desfase entre los relojes de los dos archivos.

    Parte del desfase entre los instantes de arranque y busca, primero en
    pasos de 50 ms y luego de 2 ms, el que minimiza la distancia media entre
    la odometría y la pose verdadera. Alinear solo por el arranque no basta:
    cuando el primer tramo empieza con un giro de los módulos, el chasis se
    arrastra un poco en Gazebo antes de que giren las ruedas y la
    odometría no lo registra.
    """
    base = arranque(tg, xg, yg) - arranque(ts, xs, ys)
    m = np.hypot(np.gradient(xs, ts), np.gradient(ys, ts)) > UMBRAL_MOVIMIENTO
    mejor, mejor_err = base, np.inf
    for paso, rango in ((0.05, 3.0), (0.002, 0.06)):
        for off in np.arange(mejor - rango, mejor + rango, paso):
            ex = np.interp(ts[m] + off, tg, xg) - xs[m]
            ey = np.interp(ts[m] + off, tg, yg) - ys[m]
            err = np.mean(ex * ex + ey * ey)
            if err < mejor_err:
                mejor, mejor_err = off, err
    return mejor


def distancia_a_poligono(px, py, vertices):
    """Distancia mínima de cada punto (px, py) a la poligonal de vértices."""
    d = np.full(len(px), np.inf)
    for (ax, ay), (bx, by) in zip(vertices[:-1], vertices[1:]):
        abx, aby = bx - ax, by - ay
        L2 = abx * abx + aby * aby
        if L2 < 1e-12:
            d = np.minimum(d, np.hypot(px - ax, py - ay))
            continue
        u = np.clip(((px - ax) * abx + (py - ay) * aby) / L2, 0.0, 1.0)
        d = np.minimum(d, np.hypot(px - (ax + u * abx), py - (ay + u * aby)))
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('archivo_nodo')
    ap.add_argument('archivo_gz')
    ap.add_argument('--trayectoria', default='cuadrado')
    a = ap.parse_args()

    S = np.loadtxt(a.archivo_nodo, skiprows=1)
    ts, xs, ys, ths = S[:, 0], S[:, 1], S[:, 2], S[:, 3]
    tg, xg, yg, thg = leer_gz(a.archivo_gz)

    offset = alinear(ts, xs, ys, tg, xg, yg)
    tg = tg - offset   # reloj de Gazebo llevado al del nodo

    puntos = TRAYECTORIAS[a.trayectoria]
    vertices = [(0.0, 0.0)] + [(p[0], p[1]) for p in puntos]
    x_f, y_f, th_f = puntos[-1][0], puntos[-1][1], math.radians(puntos[-1][2])

    # Tramo en que el robot recorre la trayectoria (columna 18 = fase, distinta de 0 al ir a un punto)
    t_ini = arranque(ts, xs, ys)
    en_rec = S[:, 18] != 0
    t_fin = ts[np.where(en_rec)[0][-1]]
    mg = (tg >= t_ini) & (tg <= t_fin)
    ms = (ts >= t_ini) & (ts <= t_fin)

    def resumen(nombre, x, y, th, xf, yf, thf):
        d = distancia_a_poligono(x, y, vertices)
        e_pos = math.hypot(x_f - xf, y_f - yf)
        e_th = math.degrees(math.atan2(math.sin(th_f - thf), math.cos(th_f - thf)))
        print('%-18s  final: %6.1f mm  %6.2f grados | camino: RMS %6.1f mm, max %6.1f mm'
              % (nombre, e_pos * 1000, e_th, math.sqrt(np.mean(d ** 2)) * 1000, d.max() * 1000))

    print('\nTrayectoria: %s   (desfase de relojes: %.3f s)\n' % (a.trayectoria, offset))
    # El error final se toma al llegar al último punto y no al final del
    # archivo, porque la vuelta de los módulos a 0 mueve el chasis 1-2 cm.
    jf = np.argmin(abs(tg - t_fin))
    resumen('Odometria', xs[ms], ys[ms], ths[ms], xs[ms][-1], ys[ms][-1], ths[ms][-1])
    resumen('Verdadera (Gazebo)', xg[mg], yg[mg], thg[mg], xg[jf], yg[jf], thg[jf])
    print('\n(Tras volver los modulos a 0, pose verdadera a %.1f mm del punto final)'
          % (1000 * math.hypot(xg[-1] - x_f, yg[-1] - y_f)))

    # Error al terminar cada punto (columna 17 = número de punto)
    print('\nPunto a punto (pose verdadera al terminar cada punto):')
    print('  punto   objetivo (x, y, theta)        error pos   error ang')
    for k, (px, py, pth) in enumerate(puntos, start=1):
        idx = np.where((S[:, 17] == k) & (S[:, 18] != 0))[0]
        if len(idx) == 0:
            continue
        j = np.argmin(abs(tg - ts[idx[-1]]))
        e_ang = math.degrees(math.atan2(math.sin(math.radians(pth) - thg[j]),
                                        math.cos(math.radians(pth) - thg[j])))
        print('  %3d    (%5.2f, %5.2f, %6.1f)       %6.1f mm   %6.2f grados'
              % (k, px, py, pth, 1000 * math.hypot(xg[j] - px, yg[j] - py), e_ang))

    xg_i = np.interp(ts, tg, xg)
    yg_i = np.interp(ts, tg, yg)
    thg_i = np.interp(ts, tg, thg)
    deriva = np.hypot(xs - xg_i, ys - yg_i)
    kf = np.argmin(abs(ts - t_fin))
    print('Deriva odometria vs verdadera al llegar: %.1f mm, %.2f grados'
          % (deriva[kf] * 1000, math.degrees(ths[kf] - thg_i[kf])))

    # Figura: trayectoria en el plano y orientación en el tiempo
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.2))
    vx, vy = zip(*vertices)
    ax1.plot(vx, vy, 'k--', lw=1.2, label='Teorica')
    ax1.plot(xs, ys, color='tab:blue', lw=1.8, label='Odometria')
    ax1.plot(xg, yg, color='tab:red', lw=1.8, label='Verdadera (Gazebo)')
    ax1.plot(0, 0, 'ko', ms=5)
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')
    ax1.set_aspect('equal')
    ax1.grid(alpha=0.3)
    ax1.legend(loc='best')
    ax1.set_title('Trayectoria en el plano')

    ax2.plot(ts, np.degrees(ths), color='tab:blue', label='Odometria')
    ax2.plot(tg, np.degrees(thg), color='tab:red', label='Verdadera (Gazebo)')
    en_punto = S[:, 18] != 0
    ax2.plot(ts[en_punto], np.degrees(S[en_punto, 13]), 'k.', ms=1.5, label='Pedida')
    ax2.set_xlabel('t [s]')
    ax2.set_ylabel(r'$\theta$ [grados]')
    ax2.grid(alpha=0.3)
    ax2.legend(loc='best')
    ax2.set_title('Orientacion del chasis')
    fig.tight_layout()
    salida = os.path.splitext(a.archivo_nodo)[0] + '_analisis.png'
    fig.savefig(salida, dpi=150)
    print('\nFigura guardada en %s\n' % salida)


if __name__ == '__main__':
    main()
