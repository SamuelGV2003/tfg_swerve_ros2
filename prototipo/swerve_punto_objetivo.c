/*
 * swerve_punto_objetivo.c
 * Control de punto objetivo en el prototipo a escala.
 * TFG de Samuel Gómez Villanueva.
 *
 * Basado en el ejemplo remote.c de la biblioteca ev3dev-c (Vitaly Kravtsov).
 *
 * Recibe uno o varios puntos (x, y, theta) y en cada ciclo de 20 ms calcula
 * qué enviar a los dos módulos para llegar a cada uno. Sigue la misma lógica
 * del nodo swerve_punto_objetivo_node del modelo digital:
 *
 *   1. Estima la pose (x, y, theta) por odometría.
 *   2. Calcula el error respecto al punto: ex = x_obj - x, ey = y_obj - y,
 *      eth = theta_obj - theta (normalizado a +-180 grados).
 *   3. Da el punto por alcanzado si dist < TOL_POS y |eth| < TOL_THETA
 *      durante CICLOS_LLEGADA ciclos seguidos.
 *   4. Calcula la velocidad deseada en el marco del mundo con un control
 *      proporcional, con límites de aceleración:
 *          vx_w = KP_POS*ex, vy_w = KP_POS*ey   (entre V_MIN y V_MAX)
 *          omega = KP_THETA*eth                 (hasta OMEGA_MAX)
 *   5. Pasa la velocidad al marco del robot:
 *          vx =  cos(th)*vx_w + sin(th)*vy_w
 *          vy = -sin(th)*vx_w + cos(th)*vy_w
 *   6. Aplica la cinemática inversa. Los módulos están en x = 0, y = -+w/2
 *      (1 = derecho, motores B y A; 2 = izquierdo, motores C y D):
 *          v1x = vx + omega*w/2,  v1y = vy
 *          v2x = vx - omega*w/2,  v2y = vy
 *          phi_i = atan2(viy, vix),  v_i = sqrt(vix^2 + viy^2)
 *   7. Optimización de ángulo: si un módulo tendría que girar más de 90
 *      grados (más una histéresis), gira al ángulo opuesto e invierte su
 *      rueda. Giro en espejo: si el giro queda cerca de 90 grados, donde
 *      las dos opciones cuestan casi lo mismo, el módulo izquierdo gira en
 *      sentido contrario al derecho.
 *   8. Factor de alineación común: las dos ruedas usan el factor del módulo
 *      peor alineado (1 si apunta bien, 0 si el error supera
 *      ANGULO_MAX_EMPUJE), así ninguna rueda empuja mientras un módulo gira.
 *   9. Vibración (dither) de la referencia de dirección mientras un módulo
 *      está a más de UMBRAL_DITHER de su ángulo.
 *
 * Se supone rodadura sin deslizamiento, así que el robot llega al punto
 * según la odometría. La posición real se mide aparte en el piso.
 *
 * Compilar:
 *   gcc swerve_punto_objetivo.c -lev3dev-c -lm -lpthread -o swerve_punto
 *
 * Antes de correrlo hay que hacer el homing, porque los módulos deben
 * arrancar en 0 grados. Las coordenadas van en metros y theta en grados
 * (antihorario positivo), medidas desde donde arranca el robot (X adelante,
 * Y izquierda, REP-103):
 *   ./swerve_punto 0.30 0.20                     un punto, theta final 0
 *   ./swerve_punto 0.30 0.20 90                  un punto con orientación final
 *   ./swerve_punto 0.3 0 0  0.3 0.3 0  0 0 0     varios puntos, de a tres números
 * La trayectoria se guarda en trayectoria_punto.txt.
 *
 * Pose entre corridas: al terminar, los módulos vuelven a 0 grados y la
 * pose se guarda en pose_guardada.txt. La corrida siguiente arranca desde
 * esa pose, de modo que las coordenadas son siempre respecto al último
 * reset (./swerve_punto 0 0 0 vuelve al inicio).
 *   ./swerve_punto reset    borra la pose guardada
 * Conviene hacer homing y reset si se mueve el robot con la mano, si
 * aparece el aviso de módulos fuera de 0, si se apagó el controlador o
 * después de varias corridas, porque los pequeños errores de cada vuelta
 * a 0 se acumulan.
 *
 * Modo de dos fases (MODO_DOS_FASES = 1, por defecto):
 *   Fase 1: se traslada al punto manteniendo la orientación con la que
 *           empezó, así los módulos casi no se mueven.
 *   Fase 2: al llegar, rota en el lugar hasta theta_obj. Por la
 *           optimización de ángulo los módulos quedan en 0 o 180 grados y
 *           solo se invierten las ruedas. Si al rotar se corrió la
 *           posición, la corrige después, sin rotar.
 * Con traslación y rotación al mismo tiempo (MODO_DOS_FASES = 0), la
 * referencia de cada módulo cambia todo el tiempo y el control de
 * dirección se atrasa y oscila.
 *
 * Además, el control de dirección incluye:
 *   - Rampa de referencia (VEL_MAX_DIRECCION_GRADOS): la referencia de
 *     cada módulo avanza como máximo a esa velocidad.
 *   - Integral condicional (ERROR_MAX_INTEGRAL): la integral del PI de
 *     dirección solo acumula cuando el error es pequeño.
 *   - Fuerza mínima, desfase según el sentido de la rueda y giro en espejo,
 *     para compensar el juego y la fricción de los engranajes.
 */

// Parámetros del controlador
#define KP_POS            1.5    // [1/s]   Kp = ACEL_MAX / V_MAX, igual que en el nodo de simulación
#define KP_THETA          1.5    // [1/s]   velocidad angular pedida por rad de error
#define V_MAX             0.20   // [m/s]   velocidad maxima de traslacion del chasis
#define V_MIN             0.04   // [m/s]   piso de velocidad fuera de la tolerancia (0.0 = sin piso)
#define OMEGA_MAX         0.8    // [rad/s] velocidad angular maxima del chasis
#define ACEL_MAX          0.3    // [m/s^2] cuánto puede cambiar la velocidad pedida por segundo
#define ALFA_MAX          2.0    // [rad/s^2] lo mismo para omega
#define TOL_POS           0.01   // [m]     tolerancia de llegada en posición
#define TOL_THETA_GRADOS  3.0    // [grados] tolerancia de llegada en orientación
#define CICLOS_LLEGADA    10     // ciclos seguidos dentro de tolerancia para dar por llegado (10 = 0.2 s)
#define TIEMPO_MAX_PUNTO  30.0   // [s]     tope de seguridad por punto
#define PAUSA_ENTRE_PUNTOS 0.5   // [s]     quieto entre un punto y el siguiente
#define V_MUERTA          0.005  // [m/s]   por debajo de esto el módulo no cambia de ángulo
#define HISTERESIS_GRADOS 10.0   // [grados] margen extra antes de aplicar la optimización de ángulo
#define UMBRAL_DITHER_GRADOS 2.0 // [grados] dither solo si el módulo está más lejos que esto
#define DITHER_AMPLITUD   0.05    // [rad]   (unos 2.9 grados)
#define DITHER_FREQ_HZ    8.0
#define MAX_PUNTOS        20
#define ANGULO_MAX_EMPUJE_GRADOS 25.0 // [grados] si cualquier módulo está más lejos que esto de su ángulo, ninguna rueda empuja
#define ZONA_ESPEJO_GRADOS 25.0     // [grados] en los giros entre 90-25 y 90+25 grados se aplica el giro en espejo
#define UMBRAL_GIRO_ESPEJO_GRADOS 30.0 // [grados] el giro en espejo solo se activa si el módulo derecho gira al menos esto
#define ERROR_MAX_CERO_GRADOS 3.0   // [grados] si al terminar un módulo quedó más lejos que esto de 0, avisa que hay que hacer homing
#define ERROR_MAX_INTEGRAL 0.35    // [rad del motor] (unos 14 grados de módulo) la integral de dirección solo acumula por debajo de esto
#define MODO_DOS_FASES    1      // 1 = primero traslada (manteniendo la orientación) y después rota en el lugar
                                 // 0 = traslada y rota a la vez (más exigente para la dirección)
#define VEL_MAX_DIRECCION_GRADOS 90.0 // [grados/s] velocidad máxima de cambio de la referencia de cada módulo
#define ATRASO_MAX_GRADOS 999.0   // [grados] si un módulo se atrasa más que esto respecto a su referencia, la rampa de los dos espera (999 = desactivado)
#define DESFASE_B_ADELANTE 6.0  // [grados] módulo derecho, cuando su rueda empuja hacia adelante
#define DESFASE_B_ATRAS    6.0  // [grados] módulo derecho, cuando su rueda empuja hacia atrás
#define DESFASE_C_ADELANTE -3.0  // [grados] módulo izquierdo, rueda hacia adelante
#define DESFASE_C_ATRAS    0.0  // [grados] módulo izquierdo, rueda hacia atrás
#define ZONA_RECTA_GRADOS 10.0  // [grados] si el módulo está a menos de esto de 0 o 180, no se aplica el desfase
#define FUERZA_MIN_B 30.0          // [duty] fuerza mínima del motor B (derecho) lejos del objetivo
#define FUERZA_MIN_C 30.0          // [duty] fuerza mínima del motor C (izquierdo) lejos del objetivo
#define ZONA_SIN_MINIMO_GRADOS 3.0 // [grados de módulo] más cerca que esto, no se aplica la fuerza mínima
#define COMP_FRICCION_B 0.0       // [duty] fuerza extra contra la fricción, módulo derecho (0 = desactivada)
#define COMP_FRICCION_C 0.0       // [duty] fuerza extra contra la fricción, módulo izquierdo
#define ZONA_SIN_COMP_GRADOS 3.0   // [grados de módulo] más cerca del objetivo que esto, no se compensa

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<unistd.h>
#include<math.h>
#include"ev3_light.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "brick.h"
#include<time.h>
#include<sys/timeb.h>

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
//////////////////////////////////////////////////
#endif

// Variables de los controladores de los motores

int j;
char s[256];

float pi = 3.14159265358979;
float pul2rad = 0.0174533;

float q0_vel, q1_vel, q2_vel;
float q0_pos;

long i = 0;

float vel_ang_A, vel_ang_D;

float error_angulo_C = 0.0;
float error_angulo_B = 0.0;

float error_vel_A, error_vel_D;
float error_vel_A1=0, error_vel_D1=0;
float error_vel_A2=0, error_vel_D2=0;

float posAnueva = 0.0;
float posDnueva = 0.0;
float posCnueva = 0.0;
float posBnueva = 0.0;

float posAanterior = 0.0;
float posDanterior = 0.0;

float control_A = 0.0;
float control_D = 0.0;
float control_A1 = 0.0;
float control_D1 = 0.0;
float control_B = 0.0;
float control_C = 0.0;
float control_B1 = 0.0;
float control_C1 = 0.0;

float Ts = 0.02;

// Parámetros físicos
float k_phi_odom = 1.4;    // relación de transmisión de dirección (vueltas del motor por vuelta del módulo)
float k_v_odom = 1.0/1.5;  // relación de transmisión de tracción (la rueda gira 1.5 veces por vuelta del motor)
float r_odom = 0.02544;     // radio de rueda calibrado [m]
float w_odom = 0.163;      // distancia entre módulos, valor geométrico [m]
float signo_C = 1.0;       // signo del encoder del motor C (izquierdo)
float signo_B = -1.0;      // el motor B (derecho) mide con el signo invertido

// Control PI de dirección
float Ki_pos = 5.0;
float integral_B = 0.0;
float integral_C = 0.0;
float integral_max = 4.0;

// Pose estimada por odometría
float x_chasis = 0.0;
float y_chasis = 0.0;
float theta_chasis = 0.0;

// Variables del controlador (globales para poder registrarlas en el archivo)
float x_obj = 0.0, y_obj = 0.0, theta_obj = 0.0;
float vx_w_cmd = 0.0, vy_w_cmd = 0.0, omega_cmd = 0.0;   // velocidad pedida por el controlador (marco del mundo)
float phi_cmd_B = 0.0, phi_cmd_C = 0.0;  // ángulo que el controlador quiere para cada módulo
float phi_ref_B = 0.0, phi_ref_C = 0.0;  // referencia con rampa que se envía al PI (sin dither)
float desfase_B = 0.0, desfase_C = 0.0;  // desfase aplicado a cada módulo por el empuje de su rueda [rad]
FILE *fich_odom;

// Conversión de m/s de rueda a las unidades de vel_deseada que usa
// actualizar_control_velocidad (ahí se multiplica por 10 y se compara con
// rad/s del motor). Usa las mismas constantes de la odometría, para que lo
// que se envía y lo que se mide sean coherentes:
//   w_motor [rad/s] = v [m/s] * k_v_odom / r_odom ;  vel_deseada = w_motor / 10
float unidades_por_ms(void) {
	return k_v_odom / (r_odom * 10.0);
}

static float angulo_normalizado(float angulo) {
	return atan2(sin(angulo), cos(angulo));
}

static float saturar(float valor, float limite) {
	if (valor > limite) return limite;
	if (valor < -limite) return -limite;
	return valor;
}

/* Aplica un paso (20 ms) del control PI de dirección de los dos módulos. */
void actualizar_control_direccion(float phi_objetivo_B_cmd, float phi_objetivo_C_cmd) {

	posCnueva = tacho_get_position(OUTC, 0) * pul2rad;
	posBnueva = tacho_get_position(OUTB, 0) * pul2rad;

	float posB_objetivo = phi_objetivo_B_cmd * k_phi_odom / signo_B;
	float posC_objetivo = phi_objetivo_C_cmd * k_phi_odom / signo_C;

	error_angulo_C = posC_objetivo - posCnueva;
	error_angulo_B = posB_objetivo - posBnueva;

	control_C = q0_pos * error_angulo_C + Ki_pos * integral_C;
	control_B = q0_pos * error_angulo_B + Ki_pos * integral_B;

	// Integral condicional: solo acumula cuando el error ya es pequeño. Con
	// error grande, el término proporcional basta para mover el módulo, y si
	// la integral siguiera cargándose, el módulo se pasaría al llegar y
	// quedaría oscilando.
	if (fabs(error_angulo_C) < ERROR_MAX_INTEGRAL) integral_C += error_angulo_C * Ts;
	else integral_C = 0.0;
	if (fabs(error_angulo_B) < ERROR_MAX_INTEGRAL) integral_B += error_angulo_B * Ts;
	else integral_B = 0.0;

	if (integral_C > integral_max) integral_C = integral_max;
	if (integral_C < -integral_max) integral_C = -integral_max;
	if (integral_B > integral_max) integral_B = integral_max;
	if (integral_B < -integral_max) integral_B = -integral_max;

	// Fuerza mínima: lejos del objetivo, el motor nunca empuja menos que lo
	// necesario para vencer la fricción. Cerca del objetivo no se aplica,
	// para que el PI termine de acomodar el módulo sin pasarse.
	float zona_motor = ZONA_SIN_MINIMO_GRADOS * pi / 180.0 * k_phi_odom;
	if (fabs(error_angulo_B) > zona_motor && fabs(control_B) < FUERZA_MIN_B)
		control_B = (error_angulo_B > 0.0) ? FUERZA_MIN_B : -FUERZA_MIN_B;
	if (fabs(error_angulo_C) > zona_motor && fabs(control_C) < FUERZA_MIN_C)
		control_C = (error_angulo_C > 0.0) ? FUERZA_MIN_C : -FUERZA_MIN_C;

	// Compensación de fricción: lejos del objetivo se suma una fuerza fija
	// en el sentido del error. Cerca del objetivo no se aplica. Con
	// COMP_FRICCION_B y COMP_FRICCION_C en 0 queda desactivada.
	float zona_comp = ZONA_SIN_COMP_GRADOS * pi / 180.0 * k_phi_odom;
	if (fabs(error_angulo_B) > zona_comp) control_B += (error_angulo_B > 0.0) ? COMP_FRICCION_B : -COMP_FRICCION_B;
	if (fabs(error_angulo_C) > zona_comp) control_C += (error_angulo_C > 0.0) ? COMP_FRICCION_C : -COMP_FRICCION_C;

	float max_delta_control = 5.0;   // cambio máximo del comando por ciclo

	if (control_B - control_B1 > max_delta_control) control_B = control_B1 + max_delta_control;
	if (control_B - control_B1 < -max_delta_control) control_B = control_B1 - max_delta_control;
	if (control_C - control_C1 > max_delta_control) control_C = control_C1 + max_delta_control;
	if (control_C - control_C1 < -max_delta_control) control_C = control_C1 - max_delta_control;

	if (control_B > 100) control_B = 100;
	if (control_B < -100) control_B = -100;
	if (control_C > 100) control_C = 100;
	if (control_C < -100) control_C = -100;

	control_B1 = control_B;
	control_C1 = control_C;

	tacho_set_duty_cycle_sp(OUTB, control_B);
	tacho_set_duty_cycle_sp(OUTC, control_C);
	tacho_set_command(OUTB, TACHO_RUN_DIRECT);
	tacho_set_command(OUTC, TACHO_RUN_DIRECT);
}

/* Aplica un paso (20 ms) del control de velocidad de las dos ruedas. */
void actualizar_control_velocidad(float vel_deseada_A, float vel_deseada_D) {

	posAnueva = tacho_get_position(OUTA, 0) * pul2rad;
	posDnueva = tacho_get_position(OUTD, 0) * pul2rad;

	vel_ang_A = (posAnueva - posAanterior) / Ts;
	vel_ang_D = (posDnueva - posDanterior) / Ts;

	error_vel_A = vel_deseada_A*10.0 - vel_ang_A;
	error_vel_D = vel_deseada_D*10.0 - vel_ang_D;

	control_A = (q0_vel * error_vel_A) - (q1_vel * error_vel_A1) + (q2_vel * error_vel_A2) + control_A1;
	control_D = (q0_vel * error_vel_D) - (q1_vel * error_vel_D1) + (q2_vel * error_vel_D2) + control_D1;

	error_vel_A2 = error_vel_A1;
	error_vel_A1 = error_vel_A;
	control_A1 = control_A;

	error_vel_D2 = error_vel_D1;
	error_vel_D1 = error_vel_D;
	control_D1 = control_D;

	posAanterior = posAnueva;
	posDanterior = posDnueva;

	if (control_A > 100) control_A = 100;
	if (control_A < -100) control_A = -100;
	if (control_D > 100) control_D = 100;
	if (control_D < -100) control_D = -100;

	tacho_set_duty_cycle_sp(OUTD, control_D);
	tacho_set_duty_cycle_sp(OUTA, control_A);
	tacho_set_command(OUTD, TACHO_RUN_DIRECT);
	tacho_set_command(OUTA, TACHO_RUN_DIRECT);
}

/* Odometría por cinemática directa. Integra la pose y guarda una fila por
   ciclo en el archivo de trayectoria. */
void registrar_odometria(float t_total) {
	float phi1 = (posBnueva / k_phi_odom) * signo_B - desfase_B;   // ángulo físico = lectura del motor - desfase
	float phi2 = (posCnueva / k_phi_odom) * signo_C - desfase_C;

	float v1 = (vel_ang_A / k_v_odom) * r_odom;
	float v2 = (vel_ang_D / k_v_odom) * r_odom;

	float v1x = v1 * cos(phi1);
	float v1y = v1 * sin(phi1);
	float v2x = v2 * cos(phi2);
	float v2y = v2 * sin(phi2);

	float vx_chasis = (v1x + v2x) / 2.0;
	float vy_chasis = (v1y + v2y) / 2.0;
	float omega_chasis = (v1x - v2x) / w_odom;

	x_chasis = x_chasis + (vx_chasis * cos(theta_chasis) - vy_chasis * sin(theta_chasis)) * Ts;
	y_chasis = y_chasis + (vx_chasis * sin(theta_chasis) + vy_chasis * cos(theta_chasis)) * Ts;
	theta_chasis = theta_chasis + omega_chasis * Ts;

	if (fich_odom != NULL) {
		fprintf(fich_odom, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			t_total, x_chasis, y_chasis, theta_chasis,
			vx_chasis, vy_chasis, omega_chasis, phi1, phi2,
			vel_ang_A, vel_ang_D,
			x_obj, y_obj, theta_obj, vx_w_cmd, vy_w_cmd, omega_cmd);
	}
}

/* Ángulo medido de cada módulo (misma conversión que la odometría) */
static float phi_medido_B(void) { return (posBnueva / k_phi_odom) * signo_B; }
static float phi_medido_C(void) { return (posCnueva / k_phi_odom) * signo_C; }

/* Pasos 6 y 7 para un módulo. A partir de (vix, viy) en el marco del robot
   calcula el ángulo a pedir (continuo, sin saltos de 360 grados) y la
   velocidad de la rueda, con la optimización de ángulo y el giro en espejo.
   phi_cmd_prev:    último ángulo pedido a este módulo (se actualiza).
   phi_medido:      ángulo actual del módulo según el encoder.
   signo_preferido: +1 o -1 para girar en ese sentido cuando el giro está
                    cerca de 90 grados (giro en espejo); 0 sin preferencia.
   delta_out:       giro pedido a este módulo en este ciclo.
   Devuelve la velocidad de la rueda en m/s, con signo. */
float calcular_modulo(float vix, float viy, float *phi_cmd_prev, float phi_medido,
                      float signo_preferido, float *delta_out) {
	float v = sqrt(vix*vix + viy*viy);
	*delta_out = 0.0;

	if (v < V_MUERTA) {
		// Casi sin velocidad, atan2 no está bien definido y el módulo se
		// queda donde estaba.
		return 0.0;
	}

	float phi_obj = atan2(viy, vix);
	float delta = angulo_normalizado(phi_obj - *phi_cmd_prev);   // opción A, rueda en su sentido normal
	float delta_alt = angulo_normalizado(delta + pi);             // opción B, ángulo opuesto y rueda invertida
	float zona = ZONA_ESPEJO_GRADOS * pi / 180.0;

	if (signo_preferido != 0.0 && fabs(fabs(delta) - pi/2.0) < zona) {
		// Giro en espejo: el giro está cerca de 90 grados y las dos opciones
		// cuestan casi lo mismo, así que se elige la que gira en el sentido
		// preferido (contrario al otro módulo).
		if (delta * signo_preferido < 0.0) {
			delta = delta_alt;
			v = -v;
		}
	} else {
		// Optimización de ángulo: si hay que girar más de 90 grados (más
		// la histéresis), se va al ángulo opuesto y se invierte la rueda.
		float limite = pi/2.0 + HISTERESIS_GRADOS * pi / 180.0;
		if (fabs(delta) > limite) {
			delta = delta_alt;
			v = -v;
		}
	}
	*phi_cmd_prev = *phi_cmd_prev + delta;
	*delta_out = delta;

	return v;   // el factor de alineación se aplica afuera, común a las dos ruedas
}

/* Factor de alineación de un módulo. Vale 1 si apunta bien, baja con el
   coseno del error y llega a 0 cuando el error es ANGULO_MAX_EMPUJE. */
float factor_alineacion(float phi_cmd, float phi_medido) {
	float c_lim = cos(ANGULO_MAX_EMPUJE_GRADOS * pi / 180.0);
	float f = (cos(phi_cmd - phi_medido) - c_lim) / (1.0 - c_lim);
	if (f < 0.0) f = 0.0;
	return f;
}

/* Desfase de un módulo según el sentido en que empuja su rueda. Al
   empujar, la reacción de la tracción tuerce el módulo dentro del juego de
   los engranajes, y el encoder del motor no lo detecta. Se le pide un poco
   más de ángulo en el sentido contrario para que quede donde debe. */
static float desfase_por_rueda(float v_rueda, float grados_adelante, float grados_atras) {
	if (v_rueda > V_MUERTA) return grados_adelante * pi / 180.0;
	if (v_rueda < -V_MUERTA) return -grados_atras * pi / 180.0;
	return 0.0;
}

/* Rampa de la referencia de dirección. La referencia que se envía al PI
   avanza hacia el objetivo como máximo VEL_MAX_DIRECCION_GRADOS por
   segundo. Sin la rampa, un cambio grande de ángulo (por ejemplo 180
   grados) llega de golpe y el PI se pasa y oscila. */
void mover_referencias(float objetivo_B, float objetivo_C) {
	float paso = VEL_MAX_DIRECCION_GRADOS * pi / 180.0 * Ts;
	// Sincronización: si cualquiera de los dos módulos se quedó atrás de su
	// referencia más de ATRASO_MAX_GRADOS, la rampa de los dos espera a que
	// lo alcance, para que giren juntos y lleguen paralelos.
	float atraso_max = ATRASO_MAX_GRADOS * pi / 180.0;
	if (fabs(phi_ref_B - phi_medido_B()) > atraso_max ||
	    fabs(phi_ref_C - phi_medido_C()) > atraso_max) {
		paso = 0.0;
	}
	phi_ref_B += saturar(objetivo_B - phi_ref_B, paso);
	phi_ref_C += saturar(objetivo_C - phi_ref_C, paso);
}

/* Detiene las ruedas y mantiene los módulos quietos durante el tiempo indicado. */
void pausa_quieto(float segundos, float *t_total) {
	long pasos = (long)(segundos / Ts);
	for (i = 0; i < pasos; i++) {
		vx_w_cmd = 0.0; vy_w_cmd = 0.0; omega_cmd = 0.0;
		mover_referencias(phi_cmd_B, phi_cmd_C);
		actualizar_control_direccion(phi_ref_B, phi_ref_C);
		actualizar_control_velocidad(0.0, 0.0);
		registrar_odometria(*t_total);
		*t_total += Ts;
		sleep_ms(20);
	}
}

int main(int argc, char *argv[]){

	// Lectura de los puntos desde la línea de comandos
	float lista_x[MAX_PUNTOS], lista_y[MAX_PUNTOS], lista_th[MAX_PUNTOS];
	int n_puntos = 0;
	int n_args = argc - 1;

	if (n_args == 1 && strcmp(argv[1], "reset") == 0) {
		if (remove("pose_guardada.txt") == 0) {
			printf("Pose guardada borrada. La proxima corrida arranca en (0,0,0) donde este el robot.\n");
		} else {
			printf("No habia pose guardada, la proxima corrida ya arranca en (0,0,0).\n");
		}
		return 0;
	}

	if (n_args == 2) {
		lista_x[0] = atof(argv[1]);
		lista_y[0] = atof(argv[2]);
		lista_th[0] = 0.0;
		n_puntos = 1;
	} else if (n_args >= 3 && n_args % 3 == 0 && n_args / 3 <= MAX_PUNTOS) {
		int k;
		for (k = 0; k < n_args / 3; k++) {
			lista_x[k]  = atof(argv[1 + 3*k]);
			lista_y[k]  = atof(argv[2 + 3*k]);
			lista_th[k] = atof(argv[3 + 3*k]) * pi / 180.0;
		}
		n_puntos = n_args / 3;
	} else {
		printf("Uso:\n");
		printf("  %s x y                 (un punto, theta final 0)\n", argv[0]);
		printf("  %s x y theta_grados    (un punto con orientacion)\n", argv[0]);
		printf("  %s x1 y1 th1 x2 y2 th2 ...  (varios, de a 3, max %d)\n", argv[0], MAX_PUNTOS);
		printf("  %s reset               (olvida la pose guardada)\n", argv[0]);
		printf("Metros y grados (X adelante, Y izquierda), respecto a donde se hizo el ultimo reset.\n");
		return 1;
	}

	printf("Control de punto objetivo\n");
	int k;
	for (k = 0; k < n_puntos; k++) {
		printf("  Punto %d: x=%.3f m  y=%.3f m  theta=%.1f grados\n",
			k+1, lista_x[k], lista_y[k], lista_th[k]*180.0/pi);
	}

	tacho_reset(OUTA);
	tacho_reset(OUTB);
	tacho_reset(OUTC);
	tacho_reset(OUTD);

	// Inicialización de los motores
	#ifndef __ARM_ARCH_4T__
		ev3_brick_addr = "192.168.18.107";
	#endif

	if ( ev3_init() == -1 ) return ( 1 );

	#ifndef __ARM_ARCH_4T__
		printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );
	#else
		printf( "Waiting tacho is plugged...\n" );
	#endif

	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

	printf( "*** ( EV3 ) Hello! ***\n" );
	printf( "Found tacho motors:\n" );
	for ( j = 0; j < DESC_LIMIT; j++ ) {
		if ( ev3_tacho[ j ].type_inx != TACHO_TYPE__NONE_ ) {
			printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ j ].type_inx ));
			printf( "  port = %s\n", ev3_tacho_port_name( j, s ));
		}
	}
	printf( "Robot listo.\n" );

	tacho_reset(OUTA);
	tacho_reset(OUTD);
	tacho_reset(OUTB);
	tacho_reset(OUTC);

	sleep_ms(50);

	q0_vel = 6;
	q1_vel = 2.8;
	q2_vel = 0.9;

	q0_pos = 0.4 * 685.0;

	fich_odom = fopen("trayectoria_punto.txt", "w");
	if (fich_odom == NULL) {
		printf("Aviso: no se pudo abrir trayectoria_punto.txt\n");
	} else {
		fprintf(fich_odom, "t x y theta vx vy omega phi1 phi2 vel_ang_A vel_ang_D x_obj y_obj theta_obj vx_w_cmd vy_w_cmd omega_cmd\n");
	}

	x_chasis = 0.0;
	y_chasis = 0.0;
	theta_chasis = 0.0;
	integral_B = 0.0;
	integral_C = 0.0;
	control_B1 = 0.0;
	control_C1 = 0.0;
	phi_cmd_B = 0.0;   // arranca en 0 (posición de homing)
	phi_cmd_C = 0.0;

	// Pose guardada de la corrida anterior, si existe
	FILE *fich_pose = fopen("pose_guardada.txt", "r");
	if (fich_pose != NULL) {
		if (fscanf(fich_pose, "%f %f %f", &x_chasis, &y_chasis, &theta_chasis) == 3) {
			printf("Pose cargada de la corrida anterior: x=%.3f y=%.3f theta=%.1f\n",
				x_chasis, y_chasis, theta_chasis * 180.0 / pi);
		} else {
			x_chasis = 0.0; y_chasis = 0.0; theta_chasis = 0.0;
			printf("pose_guardada.txt ilegible, arrancando en (0,0,0)\n");
		}
		fclose(fich_pose);
	} else {
		printf("Sin pose guardada, arrancando en (0,0,0)\n");
	}

	printf("Arrancando en 2 segundos...\n");
	sleep_ms(2000);

	float t_total = 0.0;
	float tol_theta = TOL_THETA_GRADOS * pi / 180.0;
	float umbral_dither = UMBRAL_DITHER_GRADOS * pi / 180.0;
	float conv = unidades_por_ms();
	long pasos_max = (long)(TIEMPO_MAX_PUNTO / Ts);
	int p;

	for (p = 0; p < n_puntos; p++) {

		x_obj = lista_x[p];
		y_obj = lista_y[p];
		theta_obj = lista_th[p];
		int contador_llegada = 0;
		int llego = 0;
		float theta_inicio_punto = theta_chasis;   // orientación a mantener durante la fase 1
		int fase = MODO_DOS_FASES ? 1 : 2;         // 1 = trasladar, 2 = rotar (y corregir posicion)
		float t_inicio_punto = t_total;

		printf("Punto %d: yendo a (%.3f, %.3f) theta=%.1f...\n",
			p+1, x_obj, y_obj, theta_obj*180.0/pi);

		for (i = 0; i < pasos_max; i++) {

			// 1 y 2. Pose estimada y error respecto al punto
			float ex = x_obj - x_chasis;
			float ey = y_obj - y_chasis;
			float dist = sqrt(ex*ex + ey*ey);
			float eth = angulo_normalizado(theta_obj - theta_chasis);

			// Modo de dos fases: mientras no llegue a la posición, la
			// orientación a mantener es la del inicio de este punto. Al
			// llegar pasa a la fase 2 (rotar) y ya no vuelve a la 1.
			if (fase == 1 && dist < TOL_POS) fase = 2;
			float eth_control = (fase == 1) ? angulo_normalizado(theta_inicio_punto - theta_chasis) : eth;

			// 3. Llegada al punto
			if (dist < TOL_POS && fabs(eth) < tol_theta) {
				contador_llegada++;
				if (contador_llegada >= CICLOS_LLEGADA) { llego = 1; break; }
			} else {
				contador_llegada = 0;
			}

			// 4. Velocidad deseada en el marco del mundo (control proporcional)
			float vx_w = KP_POS * ex;
			float vy_w = KP_POS * ey;
			float v_norma = sqrt(vx_w*vx_w + vy_w*vy_w);
			if (dist < TOL_POS) {
				vx_w = 0.0; vy_w = 0.0;           // ya está en posición
			} else if (v_norma > V_MAX) {
				vx_w *= V_MAX / v_norma; vy_w *= V_MAX / v_norma;
			} else if (v_norma < V_MIN && v_norma > 0.0) {
				vx_w *= V_MIN / v_norma; vy_w *= V_MIN / v_norma;
			}
			// Fase 2: mientras falte rotar, solo rota (sin trasladarse), así
			// los módulos se quedan en 0 grados. Cuando termina de rotar, si
			// la posición se corrió, la corrige sin rotar.
			if (fase == 2 && MODO_DOS_FASES && fabs(eth_control) >= tol_theta) {
				vx_w = 0.0; vy_w = 0.0;
			}
			float omega = saturar(KP_THETA * eth_control, OMEGA_MAX);
			if (fase == 2 && fabs(eth_control) < tol_theta) omega = 0.0;

			// límite de aceleración (arranque y frenado suaves)
			vx_w_cmd += saturar(vx_w - vx_w_cmd, ACEL_MAX * Ts);
			vy_w_cmd += saturar(vy_w - vy_w_cmd, ACEL_MAX * Ts);
			omega_cmd += saturar(omega - omega_cmd, ALFA_MAX * Ts);

			// 5. Paso al marco del robot
			float c = cos(theta_chasis), sn = sin(theta_chasis);
			float vx =  c * vx_w_cmd + sn * vy_w_cmd;
			float vy = -sn * vx_w_cmd + c * vy_w_cmd;

			// 6. Cinemática inversa (módulo 1 = derecho, en y = -w/2)
			float v1x = vx + omega_cmd * w_odom / 2.0;
			float v1y = vy;
			float v2x = vx - omega_cmd * w_odom / 2.0;
			float v2y = vy;

			// 7 y 8. Optimización de ángulo y factor de alineación.
			// El módulo derecho decide primero con la regla normal. Si le toca
			// un giro grande, el izquierdo prefiere girar al revés (espejo).
			float delta_B, delta_C;
			float v1 = calcular_modulo(v1x, v1y, &phi_cmd_B, phi_medido_B(), 0.0, &delta_B);
			float signo_pref_C = 0.0;
			if (fabs(delta_B) > UMBRAL_GIRO_ESPEJO_GRADOS * pi / 180.0) signo_pref_C = (delta_B > 0.0) ? -1.0 : 1.0;
			float v2 = calcular_modulo(v2x, v2y, &phi_cmd_C, phi_medido_C(), signo_pref_C, &delta_C);

			// Cada módulo se tuerce distinto con el empuje de su rueda, así que
			// cada uno tiene su desfase según el sentido en que gira su rueda.
			desfase_B = desfase_por_rueda(v1, DESFASE_B_ADELANTE, DESFASE_B_ATRAS);
			desfase_C = desfase_por_rueda(v2, DESFASE_C_ADELANTE, DESFASE_C_ATRAS);
			// El desfase no se aplica cuando el módulo va recto (cerca de 0 o
			// 180 grados), para no alterar los tramos rectos.
			float s_recta = sin(ZONA_RECTA_GRADOS * pi / 180.0);
			if (fabs(sin(phi_cmd_B)) < s_recta) desfase_B = 0.0;
			if (fabs(sin(phi_cmd_C)) < s_recta) desfase_C = 0.0;
			float f_B = factor_alineacion(phi_cmd_B + desfase_B, phi_medido_B());
			float f_C = factor_alineacion(phi_cmd_C + desfase_C, phi_medido_C());
			float f_comun = (f_B < f_C) ? f_B : f_C;
			v1 *= f_comun;
			v2 *= f_comun;
			// Mientras los módulos todavía están girando (factor < 0.5), la
			// velocidad pedida vuelve a cero, para que las ruedas arranquen
			// desde cero cuando los módulos terminen de alinearse.
			if (f_comun < 0.5) {
				vx_w_cmd = 0.0;
				vy_w_cmd = 0.0;
				omega_cmd = 0.0;
			}

			// 9. Rampa de referencia y dither, solo si el módulo está lejos de su ángulo
			mover_referencias(phi_cmd_B + desfase_B, phi_cmd_C + desfase_C);
			float fase = 2.0 * pi * DITHER_FREQ_HZ * t_total;
			float dither_B = 0.0, dither_C = 0.0;
			if (fabs(phi_ref_B - phi_medido_B()) > umbral_dither) dither_B = DITHER_AMPLITUD * sin(fase);
			if (fabs(phi_ref_C - phi_medido_C()) > umbral_dither) dither_C = DITHER_AMPLITUD * sin(fase);

			actualizar_control_direccion(phi_ref_B + dither_B, phi_ref_C + dither_C);
			actualizar_control_velocidad(v1 * conv, v2 * conv);
			registrar_odometria(t_total);
			t_total += Ts;
			sleep_ms(20);
		}

		// Frenado suave, la velocidad pedida vuelve a cero
		vx_w_cmd = 0.0; vy_w_cmd = 0.0; omega_cmd = 0.0;
		desfase_B = 0.0; desfase_C = 0.0;   // ruedas quietas, sin empuje que compensar

		float ex_f = x_obj - x_chasis;
		float ey_f = y_obj - y_chasis;
		float eth_f = angulo_normalizado(theta_obj - theta_chasis);
		if (llego) {
			printf("Punto %d: llego en %.2f s.", p+1, t_total - t_inicio_punto);
		} else {
			printf("Punto %d: tope de %.0f s sin llegar.", p+1, (float)TIEMPO_MAX_PUNTO);
		}
		printf(" Error final (odometria): %.1f mm, %.2f grados\n",
			sqrt(ex_f*ex_f + ey_f*ey_f) * 1000.0, eth_f * 180.0 / pi);
		printf("   Pose estimada: x=%.3f y=%.3f theta=%.1f\n",
			x_chasis, y_chasis, theta_chasis * 180.0 / pi);

		pausa_quieto(PAUSA_ENTRE_PUNTOS, &t_total);
	}

	// Vuelta de los módulos a 0 grados. Como 0, 360, 720... son la misma
	// posición física, se busca el múltiplo de 360 más cercano para no dar
	// vueltas de más.
	printf("Volviendo modulos a 0 grados...\n");
	float cero_B = phi_cmd_B - angulo_normalizado(phi_cmd_B);
	float cero_C = phi_cmd_C - angulo_normalizado(phi_cmd_C);
	integral_B = 0.0;   // se descarga la integral acumulada en el
	integral_C = 0.0;   // movimiento anterior para que vuelva más rápido
	long pasos_cero = (long)(6.0 / Ts);   // tope de seguridad 6 s
	int modulos_en_cero = 0;
	for (i = 0; i < pasos_cero; i++) {
		float fase = 2.0 * pi * DITHER_FREQ_HZ * t_total;
		mover_referencias(cero_B, cero_C);
		float d_B = (fabs(phi_ref_B - phi_medido_B()) > umbral_dither) ? DITHER_AMPLITUD * sin(fase) : 0.0;
		float d_C = (fabs(phi_ref_C - phi_medido_C()) > umbral_dither) ? DITHER_AMPLITUD * sin(fase) : 0.0;
		actualizar_control_direccion(phi_ref_B + d_B, phi_ref_C + d_C);
		actualizar_control_velocidad(0.0, 0.0);
		registrar_odometria(t_total);
		t_total += Ts;
		sleep_ms(20);
		if (phi_ref_B == cero_B && phi_ref_C == cero_C
				&& fabs(cero_B - phi_medido_B()) < 0.02 && fabs(cero_C - phi_medido_C()) < 0.02) {
			modulos_en_cero = 1;
			break;
		}
	}
	float err_cero_B = fabs(cero_B - phi_medido_B()) * 180.0 / pi;
	float err_cero_C = fabs(cero_C - phi_medido_C()) * 180.0 / pi;
	if (!modulos_en_cero && (err_cero_B > ERROR_MAX_CERO_GRADOS || err_cero_C > ERROR_MAX_CERO_GRADOS)) {
		printf("Aviso: los modulos no volvieron bien a 0. Hacer ./homing y ./swerve_punto reset.\n");
	}
	printf("Modulos: B=%.1f C=%.1f grados (respecto a su 0)\n",
		(phi_medido_B() - cero_B) * 180.0 / pi, (phi_medido_C() - cero_C) * 180.0 / pi);

	// Pose guardada para la corrida siguiente
	fich_pose = fopen("pose_guardada.txt", "w");
	if (fich_pose != NULL) {
		fprintf(fich_pose, "%f %f %f\n", x_chasis, y_chasis, theta_chasis);
		fclose(fich_pose);
		printf("Pose guardada: x=%.3f y=%.3f theta=%.1f\n", x_chasis, y_chasis, theta_chasis * 180.0 / pi);
	}

	printf("Recorrido terminado, deteniendo motores...\n");

	tacho_set_stop_action(OUTA, TACHO_HOLD);
	tacho_set_stop_action(OUTB, TACHO_HOLD);
	tacho_set_stop_action(OUTC, TACHO_HOLD);
	tacho_set_stop_action(OUTD, TACHO_HOLD);

	tacho_set_command(OUTA, TACHO_STOP);
	tacho_set_command(OUTB, TACHO_STOP);
	tacho_set_command(OUTC, TACHO_STOP);
	tacho_set_command(OUTD, TACHO_STOP);

	tacho_reset(OUTA);
	tacho_reset(OUTB);
	tacho_reset(OUTC);
	tacho_reset(OUTD);

	if (fich_odom != NULL) {
		fclose(fich_odom);
		printf("Trayectoria guardada en trayectoria_punto.txt\n");
	}

	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );

	return 0;
}
