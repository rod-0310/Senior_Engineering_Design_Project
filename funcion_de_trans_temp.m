%% === Lectura en tiempo real desde ESP32 (MAX6675 + Heater) ===
% Lee t_s, u_heater, temp_c desde Serial
% Guarda en variables u (entrada) y y (salida)
clear; clc; close all;

%% === CONFIGURA TU PUERTO ===
PORT = "COM4";   % <-- cámbialo a tu puerto (ej: '/dev/ttyUSB0' en Linux)
BAUD = 115200;
DURACION = 300;  % segundos máximos de lectura (ajusta a tu ensayo)
SAVE_NAME = "heater_datos";  % archivo .mat

%% === ABRIR SERIAL ===
sp = serialport(PORT, BAUD, "Timeout", 2);
configureTerminator(sp, "LF");
flush(sp);

fprintf("Esperando datos de %s...\n", PORT);

%% === PREPARAR VECTORES ===
t = [];  u = [];  y = [];

tic;
while toc < DURACION
    if sp.NumBytesAvailable > 0
        line = strtrim(readline(sp));
        if isempty(line) || startsWith(line, "#"), continue; end
        if contains(lower(line), "t_s"), continue; end  % ignora cabecera
        parts = split(line, ',');
        if numel(parts) < 5, continue; end

        % columnas: t_s,u_heater,u_fan_in,u_fan_out,temp_c
        vals = str2double(parts(1:5));
        if any(isnan(vals)), continue; end

        t(end+1) = vals(1);
        u(end+1) = vals(2);  % entrada = heater
        y(end+1) = vals(5);  % salida = temperatura

        % imprimir cada 5 s
        if mod(round(t(end)),5)==0
            fprintf("t=%.1fs  u=%.2f  y=%.2f°C\n", t(end), u(end), y(end));
        end
    end
end

%% === GUARDAR VARIABLES ===
save(SAVE_NAME, 't', 'u', 'y');
fprintf("\nGuardado: %s.mat\n", SAVE_NAME);

%% === GRAFICAR ===
figure('Color','w');
subplot(2,1,1);
plot(t,y,'LineWidth',1.5);
xlabel('Tiempo [s]'); ylabel('Temperatura [°C]');
title('Salida (y)');
grid on;

subplot(2,1,2);
plot(t,u,'LineWidth',1.5);
xlabel('Tiempo [s]'); ylabel('Entrada (PWM 0..1)');
title('Entrada (u)');
grid on;

%% === Mostrar resumen ===
fprintf("\nDatos capturados:\n");
fprintf("  %d muestras\n", numel(t));
fprintf("  Tiempo total: %.1f s\n", max(t)-min(t));
fprintf("  Rango de temperatura: %.1f°C - %.1f°C\n", min(y), max(y));
