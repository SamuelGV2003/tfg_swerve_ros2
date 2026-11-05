/*
 * swerve_gemelo.c
 * Programa del prototipo para la sombra digital.
 * TFG de Samuel Gómez Villanueva.
 *
 * Servidor TCP que corre en el controlador del prototipo. Recibe órdenes de
 * velocidad del computador (mando y nodo puente de ROS 2) y en cada ciclo
 * devuelve el estado que miden los encoders, para que el modelo en RViz
 * reproduzca el movimiento del prototipo.
 *
 * Se escribió a partir de dos programas:
 *   - La comunicación por red y los hilos vienen de un programa previo del
 *     laboratorio para manejar el prototipo con un mando.
 *   - El control de los motores, la cinemática inversa, la optimización de
 *     ángulo, el giro en espejo, los desfases, el factor de alineación, la
 *     rampa, el dither y la odometría son los mismos de
 *     swerve_punto_objetivo.c. Solo se quitó el control de punto objetivo,
 *     porque aquí la velocidad del chasis la indica el operador.
 * Usa la biblioteca ev3dev-c.
 *
 * Mensajes
 *   Del computador al prototipo, una línea por orden (unos 10 Hz):
 *       "vx,vy,omega,mision\n"
 *       vx, vy  [m/s]   velocidad del chasis en el marco del robot
 *                       (X adelante, Y izquierda, REP-103)
 *       omega   [rad/s] velocidad angular del chasis (antihorario positivo)
 *       mision  0 = normal, 1 = terminar el programa
 *   Del prototipo al computador, una línea cada 20 ms, separada por espacios:
 *       t x y theta vx vy omega phi1 phi2 rueda1 rueda2
 *       t             [s]    tiempo desde que arrancó el control
 *       x y theta     [m, m, rad] pose de la odometría (arranca en 0, 0, 0)
 *       vx vy omega   [m/s, m/s, rad/s] velocidad medida del chasis (marco del robot)
 *       phi1 phi2     [rad]  ángulo físico de cada módulo (1 = derecho, motor B;
 *                            2 = izquierdo, motor C), normalizado a +-pi
 *       rueda1 rueda2 [rad]  ángulo acumulado de cada rueda (1 = motor A,
 *                            2 = motor D), con la misma conversión de la odometría
 *
 * Seguridad
 *   - Si no llega una orden válida en TIMEOUT_ORDEN segundos, la velocidad
 *     pedida pasa a cero y el robot frena con la rampa de aceleración.
 *   - Si el cliente se desconecta, si llega mision = 1 o con Ctrl+C, los
 *     módulos vuelven a 0 grados y los motores se detienen. Sin esto, al
 *     cerrar el programa los motores seguirían con el último comando.
 *   - Las velocidades recibidas se limitan a V_MAX y OMEGA_MAX.
 *
 * Compilar:
 *   gcc swerve_gemelo.c -lev3dev-c -lm -lpthread -o swerve_gemelo
 * Antes de correrlo hay que hacer el homing (los módulos deben arrancar en
 * 0 grados). Luego:
 *   ./swerve_gemelo
 * El programa espera la conexión del computador en el puerto 6000 y guarda
 * los datos en trayectoria_gemelo.txt (las mismas columnas que envía, más
 * las órdenes recibidas).
 */

// Parámetros de la sombra digital
#define PUERTO            6000
#define TIMEOUT_ORDEN     0.5    // [s]     sin una orden válida durante este tiempo, la velocidad pedida pasa a 0
#define V_MAX             0.20   // [m/s]   igual que swerve_punto_objetivo.c
#define OMEGA_MAX         0.8    // [rad/s] igual que swerve_punto_objetivo.c
#define ACEL_MAX          0.3    // [m/s^2] igual que swerve_punto_objetivo.c
#define ALFA_MAX          2.0    // [rad/s^2] igual que swerve_punto_objetivo.c

// Parámetros iguales a los de swerve_punto_objetivo.c (ver la explicación de cada uno ahí)
#define V_MUERTA          0.005
#define HISTERESIS_GRADOS 10.0
#define UMBRAL_DITHER_GRADOS 2.0
#define DITHER_AMPLITUD   0.05
#define DITHER_FREQ_HZ    8.0
#define ANGULO_MAX_EMPUJE_GRADOS 25.0
#define ZONA_ESPEJO_GRADOS 25.0
#define UMBRAL_GIRO_ESPEJO_GRADOS 30.0
#define ERROR_MAX_CERO_GRADOS 3.0
#define ERROR_MAX_INTEGRAL 0.35
#define VEL_MAX_DIRECCION_GRADOS 90.0
#define ATRASO_MAX_GRADOS 999.0
#define DESFASE_B_ADELANTE 6.0
#define DESFASE_B_ATRAS    6.0
#define DESFASE_C_ADELANTE -3.0
#define DESFASE_C_ATRAS    0.0
#define ZONA_RECTA_GRADOS 10.0
#define FUERZA_MIN_B 30.0
#define FUERZA_MIN_C 30.0
#define ZONA_SIN_MINIMO_GRADOS 3.0
#define COMP_FRICCION_B 0.0
#define COMP_FRICCION_C 0.0
#define ZONA_SIN_COMP_GRADOS 3.0

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include "ev3_light.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "brick.h"

#define Sleep( msec ) usleep(( msec ) * 1000 )

int j;
char s[256];

float pi = 3.14159265358979;
float pul2rad = 0.0174533;

float q0_vel, q1_vel, q2_vel;
float q0_pos;

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

// Parámetros físicos (iguales a los de swerve_punto_objetivo.c)
float k_phi_odom = 1.4;
float k_v_odom = 1.0/1.5;
float r_odom = 0.02544;
float w_odom = 0.163;
float signo_C = 1.0;
float signo_B = -1.0;

// Control PI de dirección
float Ki_pos = 5.0;
float integral_B = 0.0;
float integral_C = 0.0;
float integral_max = 4.0;

// Pose y velocidad estimadas por odometría
float x_chasis = 0.0;
float y_chasis = 0.0;
float theta_chasis = 0.0;
float vx_med = 0.0, vy_med = 0.0, omega_med = 0.0;   // velocidad medida del chasis
float phi1_med = 0.0, phi2_med = 0.0;                // ángulo físico medido de cada módulo

// Variables del control
float vx_cmd = 0.0, vy_cmd = 0.0, omega_cmd = 0.0;   // velocidad pedida (marco del robot) después de la rampa
float phi_cmd_B = 0.0, phi_cmd_C = 0.0;
float phi_ref_B = 0.0, phi_ref_C = 0.0;
float desfase_B = 0.0, desfase_C = 0.0;
FILE *fich_odom = NULL;

// Comunicación (compartida entre los hilos y protegida con un mutex)
pthread_mutex_t mutex;
float orden_vx = 0.0, orden_vy = 0.0, orden_omega = 0.0;
double t_ultima_orden = -1000.0;
volatile int terminar = 0;
int cliente_fd = -1;

static double ahora(void) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void manejador_ctrl_c(int sig) {
	(void)sig;
	terminar = 1;
}

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

/* Control PI de dirección, igual que en swerve_punto_objetivo.c */
void actualizar_control_direccion(float phi_objetivo_B_cmd, float phi_objetivo_C_cmd) {

	posCnueva = tacho_get_position(OUTC, 0) * pul2rad;
	posBnueva = tacho_get_position(OUTB, 0) * pul2rad;

	float posB_objetivo = phi_objetivo_B_cmd * k_phi_odom / signo_B;
	float posC_objetivo = phi_objetivo_C_cmd * k_phi_odom / signo_C;

	error_angulo_C = posC_objetivo - posCnueva;
	error_angulo_B = posB_objetivo - posBnueva;

	control_C = q0_pos * error_angulo_C + Ki_pos * integral_C;
	control_B = q0_pos * error_angulo_B + Ki_pos * integral_B;

	if (fabs(error_angulo_C) < ERROR_MAX_INTEGRAL) integral_C += error_angulo_C * Ts;
	else integral_C = 0.0;
	if (fabs(error_angulo_B) < ERROR_MAX_INTEGRAL) integral_B += error_angulo_B * Ts;
	else integral_B = 0.0;

	if (integral_C > integral_max) integral_C = integral_max;
	if (integral_C < -integral_max) integral_C = -integral_max;
	if (integral_B > integral_max) integral_B = integral_max;
	if (integral_B < -integral_max) integral_B = -integral_max;

	float zona_motor = ZONA_SIN_MINIMO_GRADOS * pi / 180.0 * k_phi_odom;
	if (fabs(error_angulo_B) > zona_motor && fabs(control_B) < FUERZA_MIN_B)
		control_B = (error_angulo_B > 0.0) ? FUERZA_MIN_B : -FUERZA_MIN_B;
	if (fabs(error_angulo_C) > zona_motor && fabs(control_C) < FUERZA_MIN_C)
		control_C = (error_angulo_C > 0.0) ? FUERZA_MIN_C : -FUERZA_MIN_C;

	float zona_comp = ZONA_SIN_COMP_GRADOS * pi / 180.0 * k_phi_odom;
	if (fabs(error_angulo_B) > zona_comp) control_B += (error_angulo_B > 0.0) ? COMP_FRICCION_B : -COMP_FRICCION_B;
	if (fabs(error_angulo_C) > zona_comp) control_C += (error_angulo_C > 0.0) ? COMP_FRICCION_C : -COMP_FRICCION_C;

	float max_delta_control = 5.0;

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

/* Control de velocidad de las ruedas, igual que en swerve_punto_objetivo.c */
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

/* Odometría, con la misma fórmula de swerve_punto_objetivo.c. Además guarda
   la velocidad y los ángulos medidos en variables globales para enviarlos. */
void registrar_odometria(float t_total) {
	float phi1 = (posBnueva / k_phi_odom) * signo_B - desfase_B;
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

	vx_med = vx_chasis; vy_med = vy_chasis; omega_med = omega_chasis;
	phi1_med = phi1; phi2_med = phi2;

	if (fich_odom != NULL) {
		fprintf(fich_odom, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
			t_total, x_chasis, y_chasis, theta_chasis,
			vx_chasis, vy_chasis, omega_chasis, phi1, phi2,
			vel_ang_A, vel_ang_D,
			vx_cmd, vy_cmd, omega_cmd);
	}
}

/* Envía el estado al computador. No bloquea, así que si el computador no
   lee a tiempo, la línea se descarta y el lazo de control sigue a 20 ms. */
void enviar_estado(float t_total) {
	char msg[256];
	// Ángulo de cada rueda, con la misma conversión de motor a rueda de la odometría
	float rueda1 = posAnueva / k_v_odom;
	float rueda2 = posDnueva / k_v_odom;
	int n = snprintf(msg, sizeof(msg),
		"%.3f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
		t_total, x_chasis, y_chasis, theta_chasis,
		vx_med, vy_med, omega_med,
		angulo_normalizado(phi1_med), angulo_normalizado(phi2_med),
		rueda1, rueda2);
	if (cliente_fd >= 0 && n > 0) {
		send(cliente_fd, msg, n, MSG_DONTWAIT | MSG_NOSIGNAL);
	}
}

static float phi_medido_B(void) { return (posBnueva / k_phi_odom) * signo_B; }
static float phi_medido_C(void) { return (posCnueva / k_phi_odom) * signo_C; }

/* Optimización de ángulo y giro en espejo de un módulo, igual que en swerve_punto_objetivo.c */
float calcular_modulo(float vix, float viy, float *phi_cmd_prev, float phi_medido,
                      float signo_preferido, float *delta_out) {
	(void)phi_medido;
	float v = sqrt(vix*vix + viy*viy);
	*delta_out = 0.0;

	if (v < V_MUERTA) {
		return 0.0;
	}

	float phi_obj = atan2(viy, vix);
	float delta = angulo_normalizado(phi_obj - *phi_cmd_prev);
	float delta_alt = angulo_normalizado(delta + pi);
	float zona = ZONA_ESPEJO_GRADOS * pi / 180.0;

	if (signo_preferido != 0.0 && fabs(fabs(delta) - pi/2.0) < zona) {
		if (delta * signo_preferido < 0.0) {
			delta = delta_alt;
			v = -v;
		}
	} else {
		float limite = pi/2.0 + HISTERESIS_GRADOS * pi / 180.0;
		if (fabs(delta) > limite) {
			delta = delta_alt;
			v = -v;
		}
	}
	*phi_cmd_prev = *phi_cmd_prev + delta;
	*delta_out = delta;

	return v;
}

/* Factor de alineación de un módulo, igual que en swerve_punto_objetivo.c */
float factor_alineacion(float phi_cmd, float phi_medido) {
	float c_lim = cos(ANGULO_MAX_EMPUJE_GRADOS * pi / 180.0);
	float f = (cos(phi_cmd - phi_medido) - c_lim) / (1.0 - c_lim);
	if (f < 0.0) f = 0.0;
	return f;
}

/* Desfase de un módulo según el sentido de su rueda, igual que en swerve_punto_objetivo.c */
static float desfase_por_rueda(float v_rueda, float grados_adelante, float grados_atras) {
	if (v_rueda > V_MUERTA) return grados_adelante * pi / 180.0;
	if (v_rueda < -V_MUERTA) return -grados_atras * pi / 180.0;
	return 0.0;
}

/* Rampa de la referencia de dirección, igual que en swerve_punto_objetivo.c */
void mover_referencias(float objetivo_B, float objetivo_C) {
	float paso = VEL_MAX_DIRECCION_GRADOS * pi / 180.0 * Ts;
	float atraso_max = ATRASO_MAX_GRADOS * pi / 180.0;
	if (fabs(phi_ref_B - phi_medido_B()) > atraso_max ||
	    fabs(phi_ref_C - phi_medido_C()) > atraso_max) {
		paso = 0.0;
	}
	phi_ref_B += saturar(objetivo_B - phi_ref_B, paso);
	phi_ref_C += saturar(objetivo_C - phi_ref_C, paso);
}

void detener_motores(void) {
	tacho_set_stop_action(OUTA, TACHO_HOLD);
	tacho_set_stop_action(OUTB, TACHO_HOLD);
	tacho_set_stop_action(OUTC, TACHO_HOLD);
	tacho_set_stop_action(OUTD, TACHO_HOLD);

	tacho_set_command(OUTA, TACHO_STOP);
	tacho_set_command(OUTB, TACHO_STOP);
	tacho_set_command(OUTC, TACHO_STOP);
	tacho_set_command(OUTD, TACHO_STOP);
}

/* Al terminar, detiene las ruedas y lleva los módulos a 0 grados, igual que
   al final de swerve_punto_objetivo.c. Sigue enviando el estado para que el
   modelo también vuelva a 0. */
void volver_modulos_a_cero(float *t_total) {
	printf("Volviendo modulos a 0 grados...\n");
	float umbral_dither = UMBRAL_DITHER_GRADOS * pi / 180.0;
	desfase_B = 0.0; desfase_C = 0.0;
	vx_cmd = 0.0; vy_cmd = 0.0; omega_cmd = 0.0;
	float cero_B = phi_cmd_B - angulo_normalizado(phi_cmd_B);
	float cero_C = phi_cmd_C - angulo_normalizado(phi_cmd_C);
	integral_B = 0.0;
	integral_C = 0.0;
	long pasos_cero = (long)(6.0 / Ts);
	int modulos_en_cero = 0;
	long k;
	for (k = 0; k < pasos_cero; k++) {
		float fase_dither = 2.0 * pi * DITHER_FREQ_HZ * (*t_total);
		mover_referencias(cero_B, cero_C);
		float d_B = (fabs(phi_ref_B - phi_medido_B()) > umbral_dither) ? DITHER_AMPLITUD * sin(fase_dither) : 0.0;
		float d_C = (fabs(phi_ref_C - phi_medido_C()) > umbral_dither) ? DITHER_AMPLITUD * sin(fase_dither) : 0.0;
		actualizar_control_direccion(phi_ref_B + d_B, phi_ref_C + d_C);
		actualizar_control_velocidad(0.0, 0.0);
		registrar_odometria(*t_total);
		enviar_estado(*t_total);
		*t_total += Ts;
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
		printf("Aviso: los modulos no volvieron bien a 0. Hacer ./homing antes de la proxima corrida.\n");
	}
	printf("Modulos: B=%.1f C=%.1f grados (respecto a su 0)\n",
		(phi_medido_B() - cero_B) * 180.0 / pi, (phi_medido_C() - cero_C) * 180.0 / pi);
}

/* Hilo de control, cada 20 ms */
void *hilo_control(void *parametro) {
	(void)parametro;
	float t_total = 0.0;
	float conv = unidades_por_ms();
	float umbral_dither = UMBRAL_DITHER_GRADOS * pi / 180.0;
	int aviso_timeout = 1;

	printf("Control arrancado.\n");

	while (!terminar) {

		// 1. Última orden recibida del computador
		float vx_ref, vy_ref, omega_ref;
		double edad;
		pthread_mutex_lock(&mutex);
		vx_ref = orden_vx;
		vy_ref = orden_vy;
		omega_ref = orden_omega;
		edad = ahora() - t_ultima_orden;
		pthread_mutex_unlock(&mutex);

		// 2. Sin órdenes recientes, la velocidad pedida pasa a cero
		if (edad > TIMEOUT_ORDEN) {
			vx_ref = 0.0; vy_ref = 0.0; omega_ref = 0.0;
			if (!aviso_timeout) {
				printf("Sin ordenes del PC hace mas de %.1f s, frenando.\n", (float)TIMEOUT_ORDEN);
				aviso_timeout = 1;
			}
		} else if (aviso_timeout) {
			printf("Recibiendo ordenes del PC.\n");
			aviso_timeout = 0;
		}

		// 3. Límites de velocidad del prototipo
		float v_norma = sqrt(vx_ref*vx_ref + vy_ref*vy_ref);
		if (v_norma > V_MAX) { vx_ref *= V_MAX / v_norma; vy_ref *= V_MAX / v_norma; }
		omega_ref = saturar(omega_ref, OMEGA_MAX);

		// 4. Límite de aceleración (arranque y frenado suaves)
		vx_cmd += saturar(vx_ref - vx_cmd, ACEL_MAX * Ts);
		vy_cmd += saturar(vy_ref - vy_cmd, ACEL_MAX * Ts);
		omega_cmd += saturar(omega_ref - omega_cmd, ALFA_MAX * Ts);

		// 5. Cinemática inversa (módulo 1 = derecho, en y = -w/2). La orden ya
		// viene en el marco del robot, así que no hace falta rotarla.
		float v1x = vx_cmd + omega_cmd * w_odom / 2.0;
		float v1y = vy_cmd;
		float v2x = vx_cmd - omega_cmd * w_odom / 2.0;
		float v2y = vy_cmd;

		// 6. Optimización de ángulo y giro en espejo
		float delta_B, delta_C;
		float v1 = calcular_modulo(v1x, v1y, &phi_cmd_B, phi_medido_B(), 0.0, &delta_B);
		float signo_pref_C = 0.0;
		if (fabs(delta_B) > UMBRAL_GIRO_ESPEJO_GRADOS * pi / 180.0) signo_pref_C = (delta_B > 0.0) ? -1.0 : 1.0;
		float v2 = calcular_modulo(v2x, v2y, &phi_cmd_C, phi_medido_C(), signo_pref_C, &delta_C);

		// 7. Desfase según el sentido de cada rueda
		desfase_B = desfase_por_rueda(v1, DESFASE_B_ADELANTE, DESFASE_B_ATRAS);
		desfase_C = desfase_por_rueda(v2, DESFASE_C_ADELANTE, DESFASE_C_ATRAS);
		float s_recta = sin(ZONA_RECTA_GRADOS * pi / 180.0);
		if (fabs(sin(phi_cmd_B)) < s_recta) desfase_B = 0.0;
		if (fabs(sin(phi_cmd_C)) < s_recta) desfase_C = 0.0;

		// 8. Factor de alineación común
		float f_B = factor_alineacion(phi_cmd_B + desfase_B, phi_medido_B());
		float f_C = factor_alineacion(phi_cmd_C + desfase_C, phi_medido_C());
		float f_comun = (f_B < f_C) ? f_B : f_C;
		v1 *= f_comun;
		v2 *= f_comun;
		if (f_comun < 0.5) {
			vx_cmd = 0.0;
			vy_cmd = 0.0;
			omega_cmd = 0.0;
		}

		// 9. Rampa de referencia, dither y comandos a los motores
		mover_referencias(phi_cmd_B + desfase_B, phi_cmd_C + desfase_C);
		float fase_dither = 2.0 * pi * DITHER_FREQ_HZ * t_total;
		float dither_B = 0.0, dither_C = 0.0;
		if (fabs(phi_ref_B - phi_medido_B()) > umbral_dither) dither_B = DITHER_AMPLITUD * sin(fase_dither);
		if (fabs(phi_ref_C - phi_medido_C()) > umbral_dither) dither_C = DITHER_AMPLITUD * sin(fase_dither);

		actualizar_control_direccion(phi_ref_B + dither_B, phi_ref_C + dither_C);
		actualizar_control_velocidad(v1 * conv, v2 * conv);

		// 10. Odometría y envío del estado al computador
		registrar_odometria(t_total);
		enviar_estado(t_total);

		t_total += Ts;
		sleep_ms(20);
	}

	volver_modulos_a_cero(&t_total);
	detener_motores();
	printf("Pose final (odometria): x=%.3f y=%.3f theta=%.1f grados\n",
		x_chasis, y_chasis, theta_chasis * 180.0 / pi);
	return NULL;
}

/* Hilo de recepción de órdenes */
void *hilo_recepcion(void *parametro) {
	(void)parametro;
	char buffer[256], linea[256];
	int idx = 0;

	while (!terminar) {
		int rc = recv(cliente_fd, buffer, sizeof(buffer), 0);
		if (rc < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;  // sin datos nuevos, se revisa terminar
			printf("Error de red, terminando.\n");
			terminar = 1;
			break;
		}
		if (rc == 0) {
			printf("El PC se desconecto, terminando.\n");
			terminar = 1;
			break;
		}

		int k;
		for (k = 0; k < rc; k++) {
			if (buffer[k] == '\n') {
				linea[idx] = '\0';
				float vx, vy, w, m;
				if (sscanf(linea, "%f,%f,%f,%f", &vx, &vy, &w, &m) == 4) {
					pthread_mutex_lock(&mutex);
					orden_vx = vx;
					orden_vy = vy;
					orden_omega = w;
					t_ultima_orden = ahora();
					pthread_mutex_unlock(&mutex);
					if (m >= 0.5) {
						printf("Mision terminada desde el PC.\n");
						terminar = 1;
					}
				} else {
					printf("Mensaje no valido: %s\n", linea);
				}
				idx = 0;
			} else if (buffer[k] != '\r') {
				if (idx < (int)sizeof(linea) - 1) linea[idx++] = buffer[k];
				else idx = 0;
			}
		}
	}
	return NULL;
}

int main(void) {

	// Ctrl+C y las desconexiones no cierran el programa de golpe, sino que lo detienen en orden
	signal(SIGPIPE, SIG_IGN);
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = manejador_ctrl_c;
	sigaction(SIGINT, &sa, NULL);

	// Inicialización de los motores
	#ifndef __ARM_ARCH_4T__
		ev3_brick_addr = "192.168.18.107";
	#endif

	if ( ev3_init() == -1 ) return ( 1 );

	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

	printf( "*** ( EV3 ) Hello! ***\n" );
	printf( "Found tacho motors:\n" );
	for ( j = 0; j < DESC_LIMIT; j++ ) {
		if ( ev3_tacho[ j ].type_inx != TACHO_TYPE__NONE_ ) {
			printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ j ].type_inx ));
			printf( "  port = %s\n", ev3_tacho_port_name( j, s ));
		}
	}

	q0_vel = 6;
	q1_vel = 2.8;
	q2_vel = 0.9;
	q0_pos = 0.4 * 685.0;

	// Servidor TCP
	int servidor = socket(AF_INET, SOCK_STREAM, 0);
	if (servidor < 0) { printf("Error al crear el socket\n"); ev3_uninit(); return 1; }

	int si = 1;
	setsockopt(servidor, SOL_SOCKET, SO_REUSEADDR, &si, sizeof(si));  // permite volver a lanzar el programa enseguida

	struct sockaddr_in dir_servidor;
	memset(&dir_servidor, 0, sizeof(dir_servidor));
	dir_servidor.sin_family = AF_INET;
	dir_servidor.sin_addr.s_addr = htonl(INADDR_ANY);   // cualquier IP del EV3 (no depende de la red)
	dir_servidor.sin_port = htons(PUERTO);

	if (bind(servidor, (struct sockaddr*)&dir_servidor, sizeof(dir_servidor)) < 0) {
		printf("Error en bind (puede que el puerto %d este ocupado)\n", PUERTO);
		close(servidor); ev3_uninit(); return 1;
	}
	listen(servidor, 1);

	printf("Esperando conexion del PC en el puerto %d (Ctrl+C para salir)\n", PUERTO);
	struct sockaddr_in dir_cliente;
	socklen_t longc = sizeof(dir_cliente);
	cliente_fd = accept(servidor, (struct sockaddr*)&dir_cliente, &longc);
	if (cliente_fd < 0) {
		printf("No se acepto ninguna conexion, saliendo.\n");
		close(servidor); ev3_uninit(); return 1;
	}
	printf("PC conectado desde %s\n", inet_ntoa(dir_cliente.sin_addr));

	// recv con espera máxima de 0.2 s, para poder revisar terminar
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 200000;
	setsockopt(cliente_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	setsockopt(cliente_fd, IPPROTO_TCP, TCP_NODELAY, &si, sizeof(si));   // para que el estado salga sin demora

	// Arranque del control
	fich_odom = fopen("trayectoria_gemelo.txt", "w");
	if (fich_odom == NULL) {
		printf("Aviso: no se pudo abrir trayectoria_gemelo.txt\n");
	} else {
		fprintf(fich_odom, "t x y theta vx vy omega phi1 phi2 vel_ang_A vel_ang_D vx_cmd vy_cmd omega_cmd\n");
	}

	tacho_reset(OUTA);
	tacho_reset(OUTB);
	tacho_reset(OUTC);
	tacho_reset(OUTD);
	sleep_ms(50);

	pthread_mutex_init(&mutex, NULL);

	pthread_t id_control, id_recepcion;
	if (pthread_create(&id_control, NULL, hilo_control, NULL) != 0 ||
	    pthread_create(&id_recepcion, NULL, hilo_recepcion, NULL) != 0) {
		perror("No se pudo crear el hilo");
		terminar = 1;
		detener_motores();
		ev3_uninit();
		return 1;
	}

	pthread_join(id_recepcion, NULL);
	pthread_join(id_control, NULL);

	close(cliente_fd);
	close(servidor);
	if (fich_odom != NULL) {
		fclose(fich_odom);
		printf("Datos guardados en trayectoria_gemelo.txt\n");
	}

	tacho_reset(OUTA);
	tacho_reset(OUTB);
	tacho_reset(OUTC);
	tacho_reset(OUTD);

	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );
	return 0;
}
