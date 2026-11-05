#!/usr/bin/env python3
"""Análisis de una campaña completa de corridas de trayectoria.

Analiza 3 trayectorias (cuadrado, triángulo e ida y vuelta con giro) en
2 sentidos, con 5 corridas cada una (30 corridas en total). Usa las mismas
funciones de analizar_trayectoria.py, así que los resultados son iguales a
los de ese script. analizar_trayectoria.py debe estar en la misma carpeta.

Los archivos de la carpeta deben llamarse
    sim_<trayectoria>_<n>_<sentido>.txt   (salida del nodo)
    gz_<trayectoria>_<n>_<sentido>.txt    (grabación de Gazebo)
con n de 1 a 5 y sentido antihorario u horario. Por ejemplo
sim_cuadrado_1_antihorario.txt. Si falta alguna corrida, se avisa y se
sigue con las demás.

Uso:
    python3 analizar_campana.py ~/ROS_TFG/resultados/obj4_final

Salida, dentro de la carpeta de las corridas:
    obj4_resultados.csv      una fila por corrida
    obj4_trayectorias.pdf    las 6 combinaciones con sus corridas superpuestas
                             (también en .png)
Además imprime en la terminal el resumen por trayectoria y sentido.
"""
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analizar_trayectoria as A

TRAYS = ['cuadrado', 'triangulo', 'ida_vuelta_giro']
SENTIDOS = ['antihorario', 'horario']
NOMBRES = {'cuadrado': 'Cuadrado', 'triangulo': 'Triángulo', 'ida_vuelta_giro': 'Ida y vuelta con giro'}
LIMITE_MM = 100.0   # error final máximo aceptado [mm]


def analizar_corrida(ruta_sim, ruta_gz, clave):
    """Analiza una corrida con el mismo procedimiento de analizar_trayectoria.py.

    clave es el nombre de la trayectoria en A.TRAYECTORIAS. Devuelve un
    diccionario con los errores y la trayectoria verdadera para la figura.
    """
    S = np.loadtxt(ruta_sim, skiprows=1)
    ts, xs, ys, ths = S[:, 0], S[:, 1], S[:, 2], S[:, 3]
    tg, xg, yg, thg = A.leer_gz(ruta_gz)
    offset = A.alinear(ts, xs, ys, tg, xg, yg)
    tg = tg - offset

    puntos = A.TRAYECTORIAS[clave]
    vertices = [(0.0, 0.0)] + [(p[0], p[1]) for p in puntos]
    x_f, y_f, th_f = puntos[-1][0], puntos[-1][1], math.radians(puntos[-1][2])

    t_ini = A.arranque(ts, xs, ys)
    en_rec = S[:, 18] != 0
    t_fin = ts[np.where(en_rec)[0][-1]]
    mg = (tg >= t_ini) & (tg <= t_fin)

    jf = np.argmin(abs(tg - t_fin))   # pose verdadera al llegar al último punto
    e_pos = math.hypot(x_f - xg[jf], y_f - yg[jf])
    e_th = math.degrees(math.atan2(math.sin(th_f - thg[jf]), math.cos(th_f - thg[jf])))
    d = A.distancia_a_poligono(xg[mg], yg[mg], vertices)

    kf = np.argmin(abs(ts - t_fin))
    xg_i, yg_i = np.interp(ts, tg, xg), np.interp(ts, tg, yg)
    thg_i = np.interp(ts, tg, thg)
    deriva = math.hypot(xs[kf] - xg_i[kf], ys[kf] - yg_i[kf])
    deriva_th = math.degrees(ths[kf] - thg_i[kf])

    return {
        'e_pos_mm': e_pos * 1000, 'e_ang_deg': e_th,
        'rms_mm': math.sqrt(np.mean(d ** 2)) * 1000, 'max_mm': d.max() * 1000,
        'deriva_mm': deriva * 1000, 'deriva_deg': deriva_th,
        'xg': xg[mg], 'yg': yg[mg], 'vertices': vertices,
    }


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    carpeta = os.path.expanduser(sys.argv[1])

    filas, faltan = [], []
    datos = {}
    for tray in TRAYS:
        for sent in SENTIDOS:
            clave = tray if sent == 'antihorario' else tray + '_horario'
            for n in range(1, 6):
                sim = os.path.join(carpeta, 'sim_%s_%d_%s.txt' % (tray, n, sent))
                gz = os.path.join(carpeta, 'gz_%s_%d_%s.txt' % (tray, n, sent))
                if not (os.path.exists(sim) and os.path.exists(gz)):
                    faltan.append('%s %s %d' % (tray, sent, n))
                    continue
                r = analizar_corrida(sim, gz, clave)
                datos.setdefault((tray, sent), []).append(r)
                filas.append((tray, sent, n, r))
                print('%-16s %-11s %d  final %6.1f mm %6.2f grados | RMS %5.1f mm, max %5.1f mm'
                      % (tray, sent, n, r['e_pos_mm'], r['e_ang_deg'], r['rms_mm'], r['max_mm']))

    if faltan:
        print('\nFaltan corridas: ' + ', '.join(faltan))

    # CSV con una fila por corrida
    ruta_csv = os.path.join(carpeta, 'obj4_resultados.csv')
    with open(ruta_csv, 'w') as f:
        f.write('trayectoria,sentido,corrida,error_final_mm,error_angular_deg,rms_mm,max_mm,'
                'deriva_odometria_mm,deriva_odometria_deg\n')
        for tray, sent, n, r in filas:
            f.write('%s,%s,%d,%.1f,%.2f,%.1f,%.1f,%.1f,%.2f\n'
                    % (tray, sent, n, r['e_pos_mm'], r['e_ang_deg'], r['rms_mm'], r['max_mm'],
                       r['deriva_mm'], r['deriva_deg']))

    # Resumen por trayectoria y sentido (promedio +- desviación estándar)
    print('\n=== Resumen (pose verdadera de Gazebo, al llegar al ultimo punto) ===')
    print('%-24s %-11s %18s %10s %16s %10s' % ('Trayectoria', 'Sentido', 'Error final [mm]',
                                               'Peor [mm]', 'Error ang. [gr]', 'RMS [mm]'))
    peor_total = 0.0
    for tray in TRAYS:
        for sent in SENTIDOS:
            rs = datos.get((tray, sent), [])
            if not rs:
                continue
            e = np.array([r['e_pos_mm'] for r in rs])
            a = np.array([r['e_ang_deg'] for r in rs])
            q = np.array([r['rms_mm'] for r in rs])
            peor_total = max(peor_total, e.max())
            print('%-24s %-11s %9.1f +- %5.1f %10.1f %9.2f +- %4.2f %10.1f'
                  % (NOMBRES[tray], sent, e.mean(), e.std(ddof=1) if len(e) > 1 else 0.0,
                     e.max(), a.mean(), a.std(ddof=1) if len(a) > 1 else 0.0, q.mean()))
    print('\nPeor error final de toda la campaña: %.1f mm (limite del indicador: %.0f mm) -> %s'
          % (peor_total, LIMITE_MM, 'CUMPLE' if peor_total <= LIMITE_MM else 'NO CUMPLE'))

    # Figura: 2 filas (sentido) x 3 columnas (trayectoria)
    plt.rcParams['mathtext.fontset'] = 'cm'
    plt.rcParams['font.family'] = 'serif'
    fig, axs = plt.subplots(2, 3, figsize=(11, 7))
    for i, sent in enumerate(SENTIDOS):
        for j, tray in enumerate(TRAYS):
            ax = axs[i, j]
            rs = datos.get((tray, sent), [])
            if rs:
                vx, vy = zip(*rs[0]['vertices'])
                ax.plot(vx, vy, 'k--', lw=1.1, label='Teórica')
                for k, r in enumerate(rs):
                    ax.plot(r['xg'], r['yg'], color='#1f3a93', lw=0.9, alpha=0.75,
                            label='Gazebo (5 corridas)' if k == 0 else None)
            ax.plot(0, 0, 'ko', ms=4)
            ax.set_aspect('equal')
            ax.grid(alpha=0.3)
            ax.set_title('%s, %s' % (NOMBRES[tray], sent), fontsize=10)
            ax.set_xlabel('$x$ [m]')
            if j == 0:
                ax.set_ylabel('$y$ [m]')
    axs[0, 0].legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    ruta_fig = os.path.join(carpeta, 'obj4_trayectorias.pdf')
    fig.savefig(ruta_fig)
    fig.savefig(ruta_fig.replace('.pdf', '.png'), dpi=140)
    print('\nCSV: %s\nFigura: %s' % (ruta_csv, ruta_fig))


if __name__ == '__main__':
    main()
