%% CONTROLADOR DE PUNTO OBJETIVO
% TFG: Modelado, simulacion y control de una plataforma movil swerve
%      drive en entorno ROS 2 - Samuel Gomez Villanueva
%
% Modelo del controlador del Objetivo 3. Reproduce la logica de
% swerve_punto_objetivo_node.py para un escalon de posicion (solo
% traslacion, sin giro del chasis) y calcula:
%   Ts  : tiempo de establecimiento, banda de +-2 % alrededor del valor
%         final de la respuesta (definicion de Nise, seccion 2.5)
%   %OS : sobreimpulso respecto al valor final
%   ess : error en estado estacionario (punto pedido - valor final)
%
% Supuestos del modelo:
%   - El robot se mueve exactamente con la velocidad que se le pide
%     (sin deslizamiento).
%   - Los modulos siguen exactamente la rampa de orientacion.
%
% Se compara Kp = 1.0 (valor inicial, el mismo del prototipo) con
% Kp = 1.5 = a_max / v_max, el mayor Kp con el que el limite de
% aceleracion no interviene al frenar.
%
clear; clc; close all;

%% 1. PARAMETROS DEL CONTROLADOR (mismos valores que el nodo)
par.kp       = 1.5;    % [1/s]   ganancia proporcional (valor de diseno)
par.v_max    = 0.20;   % [m/s]   velocidad maxima de traslacion
par.v_min    = 0.04;   % [m/s]   piso de velocidad (evita stick-slip)
par.acel     = 0.3;    % [m/s^2] aceleracion maxima de la velocidad pedida
par.tol      = 0.01;   % [m]     tolerancia de llegada
par.t_lleg   = 0.2;    % [s]     tiempo dentro de la tolerancia para dar por llegado
par.v_muerta = 0.003;  % [m/s]   por debajo de esto el modulo no se reorienta
par.hist     = 10;     % [grados] histeresis de la optimizacion de angulo
par.ang_emp  = 25;     % [grados] angulo maximo de empuje (factor de alineacion)
par.vel_dir  = 90;     % [grados/s] rampa de orientacion de los modulos
par.dt       = 0.02;   % [s]     periodo del lazo (igual que el nodo)
par.t_fin    = 12;     % [s]     duracion de la simulacion

kp_arranque = 1.0;                      % valor heredado del prototipo
kp_limite   = par.acel / par.v_max;     % = 1.5, criterio de diseno

%% 2. ESCALONES DE PRUEBA (Kp de arranque y Kp de diseno)
L = 0.5;   % [m] tamano del escalon

p1 = par;  p1.kp = kp_arranque;
r1x = simular_escalon(L, 'x', p1);   r1y = simular_escalon(L, 'y', p1);
r2x = simular_escalon(L, 'x', par);  r2y = simular_escalon(L, 'y', par);

%% 3. RESULTADOS
fprintf('=== Escalon de %.2f m  (criterio: Kp <= a_max/v_max = %.2f) ===\n', L, kp_limite);
fprintf('%-10s %-9s %8s %8s %10s\n', 'Kp', 'Escalon', 'Ts [s]', '%OS', 'ess [mm]');
imprimir(kp_arranque, 'Adelante', r1x);  imprimir(kp_arranque, 'Al lado', r1y);
imprimir(par.kp,      'Adelante', r2x);  imprimir(par.kp,      'Al lado', r2y);
fprintf('Al lado, tiempo hasta que los modulos quedan alineados: %.2f s\n', r2y.t_alineado);
fprintf('Indicador: Ts <= 5 s y %%OS <= 20 %%\n');

%% 4. GRAFICAS DE LA RESPUESTA (Kp de arranque vs Kp de diseno)
figure('Name', 'Respuesta al escalon', 'Color', 'w');
graficar_respuesta(r1x, r2x, L, 1, 'Escalon hacia adelante', kp_arranque, par.kp);
graficar_respuesta(r1y, r2y, L, 2, 'Escalon hacia el lado',  kp_arranque, par.kp);

%% 5. BARRIDO DE PARAMETROS PARA EL DISENO
kp_vals   = 0.5:0.25:5.0;
vmax_vals = 0.10:0.05:0.40;
ts_kp = zeros(2, numel(kp_vals));   os_kp = zeros(2, numel(kp_vals));
ts_vm = zeros(2, numel(vmax_vals));

for k = 1:numel(kp_vals)
    p = par;  p.kp = kp_vals(k);
    rx = simular_escalon(L, 'x', p);  ry = simular_escalon(L, 'y', p);
    ts_kp(:,k) = [rx.ts; ry.ts];      os_kp(:,k) = [rx.os; ry.os];
end
for k = 1:numel(vmax_vals)
    p = par;  p.v_max = vmax_vals(k);
    ts_vm(1,k) = simular_escalon(L, 'x', p).ts;
    ts_vm(2,k) = simular_escalon(L, 'y', p).ts;
end

figure('Name', 'Barrido de parametros', 'Color', 'w');
subplot(1,3,1);
plot(kp_vals, ts_kp(1,:), 'o-', kp_vals, ts_kp(2,:), 's-', 'LineWidth', 1.2); hold on;
yline(5, 'r--', 'T_s maximo');
xline(kp_limite, 'k:', 'K_p = a_{max}/v_{max}');
xlabel('K_p [1/s]'); ylabel('T_s [s]'); grid on;
legend('Adelante', 'Al lado', 'Location', 'northeast');
title(sprintf('T_s vs K_p  (v_{max} = %.2f m/s)', par.v_max));

subplot(1,3,2);
plot(kp_vals, os_kp(1,:), 'o-', kp_vals, os_kp(2,:), 's-', 'LineWidth', 1.2); hold on;
xline(kp_limite, 'k:', 'K_p = a_{max}/v_{max}', 'LabelOrientation', 'horizontal', ...
      'LabelVerticalAlignment', 'middle', 'LabelHorizontalAlignment', 'right');
xlabel('K_p [1/s]'); ylabel('%OS'); grid on;
legend('Adelante', 'Al lado', 'Location', 'northwest');
title('Sobreimpulso vs K_p');

subplot(1,3,3);
plot(vmax_vals, ts_vm(1,:), 'o-', vmax_vals, ts_vm(2,:), 's-', 'LineWidth', 1.2); hold on;
yline(5, 'r--', 'T_s maximo');
xlabel('v_{max} [m/s]'); ylabel('T_s [s]'); grid on;
legend('Adelante', 'Al lado', 'Location', 'northeast');
title(sprintf('T_s vs v_{max}  (K_p = %.2f)', par.kp));

fprintf('\n--- Barrido de Kp ---\n%-6s %10s %10s %10s %10s\n', 'Kp', 'Ts adel.', 'Ts lado', 'OS adel.', 'OS lado');
for k = 1:numel(kp_vals)
    fprintf('%-6.2f %10.2f %10.2f %9.2f%% %9.2f%%\n', kp_vals(k), ts_kp(1,k), ts_kp(2,k), os_kp(1,k), os_kp(2,k));
end

%%  FUNCIONES LOCALES

function r = simular_escalon(L, direccion, par)
    if direccion == 'x'
        xo = L; yo = 0;
    else
        xo = 0; yo = L;
    end

    n = floor(par.t_fin / par.dt) + 1;
    T = zeros(n,1); P = zeros(n,1); PHI = zeros(n,1); F = zeros(n,1);

    x = 0; y = 0;              % posicion del robot
    vxc = 0; vyc = 0;          % velocidad pedida (con limite de aceleracion)
    phi_cmd = 0; phi_ref = 0;  % angulo pedido al modulo y angulo real (rampa)
    t_dentro = 0; t_llegada = NaN;

    c_lim = cosd(par.ang_emp);
    h     = deg2rad(par.hist);
    paso  = deg2rad(par.vel_dir) * par.dt;

    for k = 1:n
        t  = (k-1) * par.dt;
        ex = xo - x;  ey = yo - y;  d = hypot(ex, ey);

        % Llegada: dentro de la tolerancia durante t_lleg 
        if isnan(t_llegada)
            if d < par.tol
                t_dentro = t_dentro + par.dt;
                if t_dentro >= par.t_lleg - 1e-9 % margen por redondeo
                    t_llegada = t;
                end
            else
                t_dentro = 0;
            end
        end

        % Ley de control proporcional con saturaciones
        if isnan(t_llegada)
            vx = par.kp * ex;  vy = par.kp * ey;  vn = hypot(vx, vy);
            if d < par.tol
                vx = 0; vy = 0;
            elseif vn > par.v_max
                vx = vx * par.v_max / vn;  vy = vy * par.v_max / vn;
            elseif vn > 0 && vn < par.v_min
                vx = vx * par.v_min / vn;  vy = vy * par.v_min / vn;
            end
        else
            vx = 0; vy = 0;
        end

        % Limite de aceleracion 
        vxc = vxc + saturar(vx - vxc, par.acel * par.dt);
        vyc = vyc + saturar(vy - vyc, par.acel * par.dt);
        if ~isnan(t_llegada)
            vxc = 0; vyc = 0;   % al llegar, el nodo deja de mover el robot
        end

        % Cinematica inversa del modulo + optimizacion de angulo
        v = hypot(vxc, vyc);
        if v < par.v_muerta
            v = 0;
        else
            delta = normalizar(atan2(vyc, vxc) - phi_cmd);
            if abs(delta) > pi/2 + h
                delta = normalizar(delta + pi);
                v = -v;
            end
            phi_cmd = phi_cmd + delta;
        end

        % Factor de alineacion (frena la rueda si el modulo no apunta bien) 
        f = max(0, (cos(phi_cmd - phi_ref) - c_lim) / (1 - c_lim));

        % La rampa de aceleracion parte de la velocidad que reciben las ruedas (evita arranques bruscos al reorientar) 
        vxc = vxc * f;  vyc = vyc * f;

        % Rampa de orientacion (el modulo sigue la rampa exactamente) 
        phi_ref = phi_ref + saturar(phi_cmd - phi_ref, paso);

        % Planta ideal: el robot se mueve con la velocidad pedida
        ve = v * f;
        x = x + ve * cos(phi_ref) * par.dt;
        y = y + ve * sin(phi_ref) * par.dt;

        T(k) = t;
        if direccion == 'x', P(k) = x; else, P(k) = y; end
        PHI(k) = rad2deg(phi_ref);
        F(k) = f;
    end

    % Ts, %OS y error estacionario 
    p_final = mean(P(T >= T(end) - 0.3));
    fuera = find(abs(P - p_final) > 0.02 * abs(p_final));
    if isempty(fuera)
        ts = 0;
    elseif fuera(end) < n
        ts = T(fuera(end) + 1);
    else
        ts = NaN;
    end
    os  = max(0, (max(P) - p_final) / abs(p_final) * 100);
    ess = L - p_final;

    idx = find(F > 0.99, 1);
    if isempty(idx), t_al = NaN; else, t_al = T(idx); end

    r = struct('T', T, 'P', P, 'PHI', PHI, 'F', F, 'ts', ts, 'os', os, ...
               'p_final', p_final, 'ess', ess, ...
               't_llegada', t_llegada, 't_alineado', t_al);
end

function imprimir(kp, nombre, r)
    fprintf('%-10.2f %-9s %8.2f %7.2f%% %10.1f\n', kp, nombre, r.ts, r.os, r.ess*1000);
end

function graficar_respuesta(ra, rb, L, columna, titulo, kpa, kpb)
    subplot(2, 2, columna);
    plot(ra.T, ra.P, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); hold on;
    plot(rb.T, rb.P, 'b', 'LineWidth', 1.4);
    yline(L, 'k-');
    yline(1.02*rb.p_final, 'k:');  yline(0.98*rb.p_final, 'k:');
    if ~isnan(rb.ts)
        xline(rb.ts, 'r--', sprintf('T_s = %.2f s', rb.ts), ...
              'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'bottom', ...
              'LabelHorizontalAlignment', 'right');
    end
    xlabel('t [s]'); ylabel('p(t) [m]'); grid on;
    legend(sprintf('K_p = %.1f', kpa), sprintf('K_p = %.1f', kpb), 'Location', 'southeast');
    title(titulo);

    subplot(2, 2, columna + 2);
    yyaxis left;  plot(rb.T, rb.PHI, 'LineWidth', 1.2); ylabel('\phi del modulo [grados]');
    yyaxis right; plot(rb.T, rb.F, 'LineWidth', 1.2);   ylabel('Factor de alineacion');
    ylim([-0.05 1.05]);
    xlabel('t [s]'); grid on;
    title(sprintf('Orientacion de los modulos (K_p = %.1f)', kpb));
end

function y = saturar(v, lim)
    y = max(-lim, min(lim, v));
end

function a = normalizar(a)
    a = atan2(sin(a), cos(a));
end
