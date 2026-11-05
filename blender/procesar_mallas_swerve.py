"""Procesa las mallas exportadas de BrickLink Studio para el modelo en Gazebo.

Toma los 6 archivos Collada (.dae) exportados de Studio (chasis, apoyos,
pivote_derecho, pivote_izquierdo, rueda_derecha y rueda_izquierda) y los
deja listos para usarlos como <visual> en el xacro del robot.

Para cada archivo:
  1. Lo importa y borra las cámaras, luces y objetos vacíos que agrega
     Studio.
  2. Aplica todas las transformaciones y une las piezas en un solo objeto.
     Si el modelo viene reflejado en vertical (Studio usa el eje vertical
     al revés), lo corrige, y deja las caras mirando hacia afuera.
  3. Lo pasa de centímetros (unidad de Studio) a metros.
  4. Lo ubica en el sistema de referencia de su link del URDF (X adelante,
     Y izquierda, Z arriba, según REP-103), tomando como referencia el
     centro de las ruedas en el modelo de Studio.
  5. Lo simplifica para que Gazebo no se ponga lento y lo parte en bloques
     más pequeños, porque Gazebo dibuja mal las mallas con demasiados
     vértices en un solo bloque.
  6. Lo exporta a la carpeta "procesados".
Al final arma el robot con las mallas procesadas, en las posiciones del
URDF, para revisarlo en Blender.
"""
import bpy
import bmesh
import os
from mathutils import Vector, Matrix

# Carpeta donde están los 6 .dae exportados de Studio (cambiar)
CARPETA = r"C:\ruta\a\las\mallas_exportadas"

SALIDA = os.path.join(CARPETA, "procesados")

# Valores del URDF (swerve_robot.urdf.xacro), en metros
RADIO_RUEDA = 0.0275
ALTO_MODULO = 0.0805
ALTURA_BASE = RADIO_RUEDA + ALTO_MODULO   # base_link y steer_link sobre el piso

TRACK_URDF = 0.163                        # track_width del URDF

ESCALA = 0.01   # Studio exporta en cm

# Máximo de triángulos por bloque. El motor gráfico de Gazebo arma cada
# bloque con índices de 16 bits (hasta 65535 vértices) y al leer Collada
# puede repetir un vértice por cada esquina de triángulo. Con 15000
# triángulos quedan como máximo 45000 vértices, con margen.
TRIANGULOS_POR_BLOQUE = 15000

# Máximo de vértices por malla después de simplificar. Si alguna pieza se
# ve deformada en la revisión final, se sube su número y se vuelve a
# ejecutar. Si Gazebo va lento, se bajan.
MAX_VERTICES = {
    "chasis":           80000,
    "apoyos":           30000,
    "pivote_derecho":   25000,
    "pivote_izquierdo": 25000,
    "rueda_derecha":    10000,
    "rueda_izquierda":  10000,
}


def limpiar_escena():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for coleccion in (bpy.data.meshes, bpy.data.cameras, bpy.data.lights):
        for bloque in list(coleccion):
            if bloque.users == 0:
                coleccion.remove(bloque)


ESPEJO_Z = False   # se decide más abajo, comparando el chasis con las ruedas


def orientar_caras_afuera(malla):
    """Da vuelta las caras de una pieza si miran hacia adentro.

    Pasa con los reflejos, y se detecta porque el volumen con signo de la
    pieza sale negativo. Gazebo no dibuja bien las caras que miran hacia
    adentro. Se revisa pieza por pieza.
    """
    bm = bmesh.new()
    bm.from_mesh(malla)
    if bm.calc_volume(signed=True) < 0:
        bmesh.ops.reverse_faces(bm, faces=bm.faces)
        bm.to_mesh(malla)
        malla.update()
    bm.free()


def importar_y_unir(nombre):
    """Importa un .dae y devuelve un solo objeto malla con las transformaciones aplicadas (en cm)."""
    limpiar_escena()
    ruta = os.path.join(CARPETA, nombre + ".dae")
    if not os.path.isfile(ruta):
        raise FileNotFoundError("No se encontró " + ruta)
    bpy.ops.wm.collada_import(filepath=ruta)

    mallas = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    if not mallas:
        raise RuntimeError("El archivo " + nombre + " no tiene mallas")

    # Aplicar transformaciones: posición final de cada vértice en
    # coordenadas del mundo
    for o in mallas:
        o.data = o.data.copy()             # separa mallas compartidas (piezas repetidas)
        o.data.transform(o.matrix_world)
        if ESPEJO_Z:
            o.data.transform(Matrix.Diagonal((1.0, 1.0, -1.0, 1.0)))
        orientar_caras_afuera(o.data)      # pieza por pieza
    # Borrar todo lo que no sea malla (cámaras, luces y vacíos de Studio)
    for o in list(bpy.context.scene.objects):
        if o.type != 'MESH':
            bpy.data.objects.remove(o, do_unlink=True)
    for o in mallas:
        o.parent = None
        o.matrix_world = Matrix.Identity(4)

    bpy.ops.object.select_all(action='DESELECT')
    for o in mallas:
        o.select_set(True)
    bpy.context.view_layer.objects.active = mallas[0]
    if len(mallas) > 1:
        bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    obj.name = nombre
    obj.data.name = nombre

    # Sin normales personalizadas y con sombreado plano, para evitar sombras
    # raras después de reflejar o fundir vértices (las piezas tienen caras
    # planas)
    if obj.data.has_custom_normals:
        bpy.ops.mesh.customdata_custom_splitnormals_clear()
    for pol in obj.data.polygons:
        pol.use_smooth = False

    return obj


def caja(obj):
    """Devuelve (mínimo, máximo, centro) de la malla, en sus coordenadas."""
    vs = [v.co for v in obj.data.vertices]
    mn = Vector((min(v.x for v in vs), min(v.y for v in vs), min(v.z for v in vs)))
    mx = Vector((max(v.x for v in vs), max(v.y for v in vs), max(v.z for v in vs)))
    return mn, mx, (mn + mx) / 2


def caja_varios(objs):
    cajas = [caja(o) for o in objs]
    mn = Vector((min(c[0].x for c in cajas), min(c[0].y for c in cajas), min(c[0].z for c in cajas)))
    mx = Vector((max(c[1].x for c in cajas), max(c[1].y for c in cajas), max(c[1].z for c in cajas)))
    return mn, mx, (mn + mx) / 2


def partir_en_bloques(obj, nombre):
    """Parte la malla en bloques de hasta TRIANGULOS_POR_BLOQUE caras.

    Agrupa caras cercanas entre sí y no cambia la forma ni la posición.
    Devuelve la lista de bloques.
    """
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_mode(type='FACE')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.quads_convert_to_tris()     # para contar los triángulos reales
    while True:
        bm = bmesh.from_edit_mesh(obj.data)
        caras = list(bm.faces)
        if len(caras) <= TRIANGULOS_POR_BLOQUE:
            break
        # ordenar por posición (X y luego Y) para que cada bloque sea compacto
        caras.sort(key=lambda f: (round(f.calc_center_median().x, 3),
                                  f.calc_center_median().y))
        for f in bm.faces:
            f.select = False
        for f in caras[:TRIANGULOS_POR_BLOQUE]:
            f.select = True
        bm.select_flush_mode()
        bmesh.update_edit_mesh(obj.data)
        bpy.ops.mesh.separate(type='SELECTED')
    bpy.ops.object.mode_set(mode='OBJECT')

    bloques = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    for i, b in enumerate(sorted(bloques, key=lambda o: o.name)):
        b.name = "%s_%02d" % (nombre, i)
        b.data.name = b.name
    return bloques


# 1) Sistema de referencia del robot a partir de las ruedas

_, _, c_chasis = caja(importar_y_unir("chasis"))
_, _, c_der = caja(importar_y_unir("rueda_derecha"))
mn_i, mx_i, c_izq = caja(importar_y_unir("rueda_izquierda"))
_, _, c_apoyos = caja(importar_y_unir("apoyos"))

# El chasis tiene que quedar por encima del centro de las ruedas. Si queda
# por debajo, el modelo vino reflejado en vertical, así que se activa
# ESPEJO_Z y también se reflejan los centros ya medidos.
if c_chasis.z < (c_der.z + c_izq.z) / 2:
    ESPEJO_Z = True
    for c in (c_chasis, c_der, c_izq, c_apoyos):
        c.z = -c.z
    mn_i, mx_i = (Vector((mn_i.x, mn_i.y, -mx_i.z)),
                  Vector((mx_i.x, mx_i.y, -mn_i.z)))

radio_studio = (mx_i.z - mn_i.z) / 2          # cm (rueda vertical)
piso_z = (c_der.z + c_izq.z) / 2 - radio_studio

eje_y = Vector((c_izq.x - c_der.x, c_izq.y - c_der.y, 0.0))
track_studio = eje_y.length                   # cm
eje_y.normalize()                             # Y hacia la izquierda
eje_z = Vector((0.0, 0.0, 1.0))               # Z hacia arriba
eje_x = eje_y.cross(eje_z)                    # X hacia adelante (X = Y x Z)

medio = (c_der + c_izq) / 2
adelante_ok = (c_apoyos - medio).dot(eje_x) > 0

# Matriz de rotación, sus columnas son los ejes del robot expresados en Blender
R = Matrix((eje_x, eje_y, eje_z)).transposed().to_4x4()


def matriz_a_link(origen_cm):
    """Pasa un vértice de coordenadas de Blender (cm) a las del link (m, REP-103)."""
    return (Matrix.Scale(ESCALA, 4)
            @ R.transposed()
            @ Matrix.Translation(-origen_cm))


altura_base_cm = piso_z + ALTURA_BASE / ESCALA
origen_base = Vector((medio.x, medio.y, altura_base_cm))

# Origen del link al que pertenece cada malla (en cm, coordenadas de Blender)
PARTES = {
    "chasis":            origen_base,                                   # base_link
    "apoyos":            origen_base,                                   # base_link (opcional)
    "pivote_derecho":    Vector((c_der.x, c_der.y, altura_base_cm)),    # der_steer_link
    "pivote_izquierdo":  Vector((c_izq.x, c_izq.y, altura_base_cm)),    # izq_steer_link
    "rueda_derecha":     c_der.copy(),                                  # der_wheel_link
    "rueda_izquierda":   c_izq.copy(),                                  # izq_wheel_link
}


# 2) Procesar y exportar cada parte

os.makedirs(SALIDA, exist_ok=True)
resumen = []
for nombre, origen in PARTES.items():
    obj = importar_y_unir(nombre)
    obj.data.transform(matriz_a_link(origen))
    obj.data.update()
    v_antes = len(obj.data.vertices)

    # Fundir vértices duplicados (a menos de 0.01 mm). No cambia la forma,
    # solo quita los repetidos que deja la exportación de Studio.
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.00001)
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()

    # Simplificar hasta el máximo de vértices de esta pieza
    n = len(obj.data.vertices)
    if n > MAX_VERTICES[nombre]:
        mod = obj.modifiers.new("simplificar", 'DECIMATE')
        mod.ratio = MAX_VERTICES[nombre] / n
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    v_despues = len(obj.data.vertices)

    # Partir en bloques (ver TRIANGULOS_POR_BLOQUE)
    bloques = partir_en_bloques(obj, nombre)
    mn, mx, c = caja_varios(bloques)
    bpy.ops.object.select_all(action='DESELECT')
    for b in bloques:
        b.select_set(True)
    bpy.context.view_layer.objects.active = bloques[0]
    ruta_out = os.path.join(SALIDA, nombre + ".dae")
    bpy.ops.wm.collada_export(filepath=ruta_out, selected=True,
                              apply_modifiers=True, triangulate=True)
    resumen.append("  %-18s tamaño %.3f x %.3f x %.3f m | vértices %d -> %d | %d bloques"
                   % (nombre, mx.x - mn.x, mx.y - mn.y, mx.z - mn.z,
                      v_antes, v_despues, len(bloques)))


# 3) Armar el robot con las mallas procesadas, en las posiciones del URDF
#    y con los módulos a 0 grados, para revisarlo

limpiar_escena()
POS_URDF = {
    "chasis":           (0.0, 0.0, ALTURA_BASE),
    "apoyos":           (0.0, 0.0, ALTURA_BASE),
    "pivote_derecho":   (0.0, -TRACK_URDF / 2, ALTURA_BASE),
    "pivote_izquierdo": (0.0,  TRACK_URDF / 2, ALTURA_BASE),
    "rueda_derecha":    (0.0, -TRACK_URDF / 2, RADIO_RUEDA),
    "rueda_izquierda":  (0.0,  TRACK_URDF / 2, RADIO_RUEDA),
}
for nombre, pos in POS_URDF.items():
    antes = set(bpy.context.scene.objects)
    bpy.ops.wm.collada_import(filepath=os.path.join(SALIDA, nombre + ".dae"))
    for o in set(bpy.context.scene.objects) - antes:
        if o.parent is None:
            o.location = pos
for o in list(bpy.context.scene.objects):
    if o.type in ('CAMERA', 'LIGHT'):
        bpy.data.objects.remove(o, do_unlink=True)


# 4) Resumen en la consola del sistema

print("\nResumen de las mallas")
print("Distancia entre ruedas en Studio: %.2f cm (el URDF usa 16.30 cm)" % track_studio)
print("Diámetro de rueda en Studio: %.2f cm (el URDF usa 5.50 cm)" % (2 * radio_studio))
if adelante_ok:
    print("Orientación correcta, la barra de apoyos quedó hacia adelante (+X)")
else:
    print("Atención, la barra de apoyos quedó hacia atrás. Probablemente los")
    print("archivos de rueda o pivote derecho e izquierdo están cruzados.")
if ESPEJO_Z:
    print("El modelo venía reflejado en vertical y se corrigió")
else:
    print("El modelo venía sin reflejo")
print("Mallas exportadas en " + SALIDA)
for linea in resumen:
    print(linea)
print()
