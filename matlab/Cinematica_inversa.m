%% CINEMATICA INVERSA 
% TFG: Modelado, simulacion y control de una plataforma movil swerve
%      drive en entorno ROS 2 - Samuel Gomez Villanueva
%
% Este script implementa la cinematica inversa del chasis:
%   dado el movimiento deseado del robot (vx, vy, omega), calcula
%   la velocidad y el angulo que debe tener cada modulo swerve, y
%   finalmente los comandos que hay que enviarle a cada motor real
%   (aplicando las relaciones de transmisión de los engranajes).
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
                            % la rueda gira 1.5 veces por cada vuelta del motor
                            
% Posicion de cada modulo activo respecto al origen (punto medio
% entre los 2 ejes de direccion). Modulo 1 = derecho, Modulo 2 = izquierdo.
l1 = [0, -w/2];   % [l1x, l1y] modulo derecho
l2 = [0, +w/2];   % [l2x, l2y] modulo izquierdo
modulos_l = {l1, l2};
nombres_modulos = {'Derecho (1)', 'Izquierdo (2)'};

%% ------------------------------------------------------------------
%  ENTRADA DE DATOS 
%  ------------------------------------------------------------------
% Al correr el script, pide el movimiento deseado del chasis y calcula
% de una vez los comandos de motor para los 2 modulos.

fprintf('--- Cinematica inversa swerve: movimiento deseado del chasis ---\n');
vx    = input('vx    [m/s]  : ');
vy    = input('vy    [m/s]  : ');
omega = input('omega [rad/s]: ');

fprintf('\n%-14s | %8s | %10s | %14s | %14s\n', ...
    'Modulo', 'v_i [m/s]', 'phi_i [deg]', 'phi_motor[deg]', 'w_motor [rad/s]');
fprintf('%s\n', repmat('-', 1, 72));

for i = 1:numel(modulos_l)
    [v_i, phi_i, phi_motor, w_motor] = cinematica_modulo(vx, vy, omega, ...
        modulos_l{i}, r, k_phi, k_v);

    fprintf('%-14s | %8.4f | %10.2f | %14.2f | %14.2f\n', ...
        nombres_modulos{i}, v_i, rad2deg(phi_i), rad2deg(phi_motor), w_motor);
end

%% ------------------------------------------------------------------
%  FUNCION: cinematica inversa del chasis para un modulo
%  ------------------------------------------------------------------
% Dado (vx, vy, omega) del chasis y la posicion (lx, ly) del modulo,
% devuelve:
%   v_i       -> velocidad lineal real de la rueda [m/s]
%   phi_i     -> angulo real del modulo [rad], convencion atan2
%   phi_motor -> angulo a comandar al motor de direccion [rad]
%   w_motor   -> velocidad angular a comandar al motor de traccion [rad/s]

function [v_i, phi_i, phi_motor, w_motor] = cinematica_modulo(vx, vy, omega, l, r, k_phi, k_v)
lx = l(1);
ly = l(2);

% Paso 1: velocidad del modulo debida al movimiento del chasis
vix = vx - omega * ly;
viy = vy + omega * lx;

% Paso 2: magnitud y angulo real del modulo
v_i   = sqrt(vix^2 + viy^2);
phi_i = atan2(viy, vix);

% Paso 3: comandos de motor (aplicando relaciones de transmision)
phi_motor = k_phi * phi_i;

w_rueda   = v_i / r;
w_motor   = k_v * w_rueda;
end