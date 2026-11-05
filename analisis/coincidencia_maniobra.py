#!/usr/bin/env python3
"""Coincidencia de una maniobra en Gazebo con la velocidad pedida.

Lee la pose verdadera del robot grabada desde Gazebo, calcula su velocidad
en el marco del robot (vx, vy, omega) durante el tramo estable y la compara
con la velocidad pedida en /cmd_vel. Para cada componente no nula:

    C = (1 - |v_sim - v_ref| / |v_ref|) * 100 %

La coincidencia de la maniobra es la menor de esas componentes. El criterio
de aceptación es C >= 90 %.

Grabación (mientras corre la maniobra):
    gz topic -e -t /world/empty/dynamic_pose/info > archivo.txt

Uso:
    python3 coincidencia_maniobra.py archivo.txt vx vy omega
    Ejemplo: python3 coincidencia_maniobra.py recto.txt 0.3 0 0
"""
import math
import re
import sys
import numpy as np

MODELO = 'swerve_robot'
MARGEN = 1.0          # [s] que se descartan al inicio y al final del movimiento
UMBRAL_V = 0.02       # [m/s] velocidad a partir de la cual el robot se considera en movimiento
UMBRAL_W = 0.05       # [rad/s] lo mismo para la velocidad angular


def leer_grabacion(ruta):
    """Lee la grabación de gz topic y devuelve t, x, y, theta del robot.

    Toma solo la pose del modelo completo (no la de cada link), ordena por
    tiempo, elimina muestras repetidas y desenrolla el ángulo para que no
    salte entre pi y -pi.
    """
    texto = open(ruta).read()
    t, x, y, th = [], [], [], []
    num = r'(-?[\d.eE+-]+)'
    for msg in texto.split('header {')[1:]:
        m_t = re.search(r'stamp\s*{\s*(?:sec:\s*(\d+))?\s*(?:nsec:\s*(\d+))?', msg)
        if not m_t:
            continue
        sec = int(m_t.group(1) or 0)
        nsec = int(m_t.group(2) or 0)
        m_p = re.search(r'name:\s*"%s"(.*?)(?=\npose\s*{|\Z)' % MODELO, msg, re.S)
        if not m_p:
            continue
        bloque = m_p.group(1)

        def comp(seccion, eje):
            s = re.search(seccion + r'\s*{([^}]*)}', bloque)
            if not s:
                return 0.0
            v = re.search(eje + r':\s*' + num, s.group(1))
            return float(v.group(1)) if v else 0.0

        px, py = comp('position', 'x'), comp('position', 'y')
        qx, qy, qz = comp('orientation', 'x'), comp('orientation', 'y'), comp('orientation', 'z')
        qw = comp('orientation', 'w')
        if qw == 0.0 and qx == 0.0 and qy == 0.0 and qz == 0.0:
            qw = 1.0   # Gazebo omite los campos que valen 0
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        t.append(sec + nsec * 1e-9)
        x.append(px)
        y.append(py)
        th.append(yaw)
    t = np.array(t)
    orden = np.argsort(t)
    t, x, y, th = t[orden], np.array(x)[orden], np.array(y)[orden], np.array(th)[orden]
    _, unicos = np.unique(t, return_index=True)
    return t[unicos], x[unicos], y[unicos], np.unwrap(th[unicos])


def main():
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(1)
    ruta = sys.argv[1]
    ref = np.array([float(v) for v in sys.argv[2:5]])

    t, x, y, th = leer_grabacion(ruta)
    if len(t) < 10:
        print('No se encontraron suficientes poses de "%s" en %s' % (MODELO, ruta))
        sys.exit(1)

    # Velocidad en el marco del mundo y su paso al marco del robot
    vxw, vyw, w = np.gradient(x, t), np.gradient(y, t), np.gradient(th, t)
    c, s = np.cos(th), np.sin(th)
    vx = c * vxw + s * vyw
    vy = -s * vxw + c * vyw

    # Tramo estable: desde MARGEN después de arrancar hasta MARGEN antes de frenar
    moviendo = (np.hypot(vxw, vyw) > UMBRAL_V) | (np.abs(w) > UMBRAL_W)
    if not moviendo.any():
        print('El robot no se movio en la grabacion.')
        sys.exit(1)
    t_ini, t_fin = t[moviendo][0] + MARGEN, t[moviendo][-1] - MARGEN
    tramo = (t >= t_ini) & (t <= t_fin)
    sim = np.array([vx[tramo].mean(), vy[tramo].mean(), w[tramo].mean()])

    print('Archivo: %s  (%d poses, tramo estable %.1f s a %.1f s)'
          % (ruta, len(t), t_ini - t[0], t_fin - t[0]))
    print('%-6s %10s %10s %12s' % ('', 'Pedida', 'Gazebo', 'Coincidencia'))
    nombres = ['vx', 'vy', 'omega']
    unidades = ['m/s', 'm/s', 'rad/s']
    coincidencias = []
    for k in range(3):
        if abs(ref[k]) > 1e-9:
            ck = (1.0 - abs(sim[k] - ref[k]) / abs(ref[k])) * 100.0
            coincidencias.append(ck)
            txt = '%11.2f %%' % ck
        else:
            txt = '   (no aplica)'
        print('%-6s %10.4f %10.4f %s  [%s]' % (nombres[k], ref[k], sim[k], txt, unidades[k]))
    c_min = min(coincidencias)
    print('Coincidencia de la maniobra: %.2f %%  ->  %s'
          % (c_min, 'CUMPLE (>= 90 %)' if c_min >= 90.0 else 'NO CUMPLE (< 90 %)'))


if __name__ == '__main__':
    main()
