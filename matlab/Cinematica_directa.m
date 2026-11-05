%% CINEMATICA DIRECTA 
% TFG: Modelado, simulacion y control de una plataforma movil swerve
%      drive en entorno ROS 2 - Samuel Gomez Villanueva
%
% Este script implementa la cinematica directa del chasis:
%   dado lo que miden los 4 encoders (angulo de cada motor de direccion
%   y velocidad angular de cada motor de traccion), reconstruye el
%   movimiento real del chasis (vx, vy, omega).
%
%
% Convencion de ejes:
%   x: hacia adelante del robot
%   y: hacia la izquierda del robot
%   omega > 0: giro antihorario visto desde arriba
%
% Origen del sistema de referencia: punto medio entre los ejes de
% direccion de los 2 modulos delanteros.
%
clear; clc;
%% ------------------------------------------------------------------
%  PARAMETROS FISICOS DEL PROTOTIPO
%  ------------------------------------------------------------------
% Diametro y radio de la rueda de traccion
D_rueda = 0.055;           % [m] diametro medido
r = D_rueda / 2;           % [m] radio de rueda

% Track width: distancia centro-a-centro entre los ejes de direccion
% de los 2 modulos delanteros
w = 0.163;                 % [m]

% Wheelbase: distancia longitudinal modulo -> balin trasero
l_wheelbase = 0.131;       % [m]

% Separacion entre los 2 balines traseros
w_balines = 0.100;         % [m]

% Relaciones de transmisión motor:salida
k_phi = 1.4;                % direccion (40->56 dientes)
k_v   = 1/1.5;              % traccion  (36->24->24 dientes)
                            % la rueda gira 1.5 veces por cada vuelta del motor                         % la rueda gira 1.5 veces por cada vuelta del motor

% Posicion de cada modulo activo respecto al origen (punto medio
% entre los 2 ejes de direccion). Modulo 1 = derecho, Modulo 2 = izquierdo.
l1 = [0, -w/2];   % [l1x, l1y] modulo derecho
l2 = [0, +w/2];   % [l2x, l2y] modulo izquierdo
modulos_l = {l1, l2};
nombres_modulos = {'Derecho (1)', 'Izquierdo (2)'};

%% ------------------------------------------------------------------
%  ENTRADA DE DATOS
%  ------------------------------------------------------------------
% Al correr el script, pide las lecturas de los 4 motores (2 modulos,
% cada uno con angulo de direccion y velocidad de traccion) y calcula
% de una vez el movimiento reconstruido del chasis (vx, vy, omega).

fprintf('--- Cinematica directa swerve: lecturas de motor (odometria) ---\n');

fprintf('\nModulo Derecho (1):\n');
phi1_motor_deg = input('  phi1_motor [deg] : ');
w1_motor       = input('  w1_motor   [rad/s]: ');

fprintf('\nModulo Izquierdo (2):\n');
phi2_motor_deg = input('  phi2_motor [deg] : ');
w2_motor       = input('  w2_motor   [rad/s]: ');

phi1_motor = deg2rad(phi1_motor_deg);
phi2_motor = deg2rad(phi2_motor_deg);

[v1x, v1y] = odometria_modulo(phi1_motor, w1_motor, r, k_phi, k_v);
[v2x, v2y] = odometria_modulo(phi2_motor, w2_motor, r, k_phi, k_v);

% Reconstruccion del movimiento del chasis (formulas deducidas a mano
% para este caso particular: l1x = l2x = 0)
vx    = (v1x + v2x) / 2;
vy    = (v1y + v2y) / 2;
omega = (v1x - v2x) / w;

fprintf('\n%-20s | %10s\n', 'Chasis reconstruido', 'Valor');
fprintf('%s\n', repmat('-', 1, 35));
fprintf('%-20s | %10.4f m/s\n', 'vx', vx);
fprintf('%-20s | %10.4f m/s\n', 'vy', vy);
fprintf('%-20s | %10.4f rad/s\n', 'omega', omega);

%% ------------------------------------------------------------------
%  FUNCION: odometria de un modulo (motor -> velocidad del motor)
%  ------------------------------------------------------------------
% Dado lo que lee el encoder de un modulo (phi_motor, w_motor),
% deshace las relaciones de transmision y devuelve la velocidad del modulo en x e y (vix, viy).

function [vix, viy] = odometria_modulo(phi_motor, w_motor, r, k_phi, k_v)
% Paso 1: deshacer relaciones de transmisión (motor -> real)
phi_i    = phi_motor / k_phi;
w_rueda  = w_motor / k_v;
v_i      = w_rueda * r;

% Paso 2: descomponer en componentes x, y
vix = v_i * cos(phi_i);
viy = v_i * sin(phi_i);
end