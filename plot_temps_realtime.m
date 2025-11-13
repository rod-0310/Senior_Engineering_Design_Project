% Guarda como plot_temps_realtime.m y ejecuta:
% plot_temps_realtime("COM5", 30)  % 30 minutos

function plot_temps_realtime(port, duration_minutes)
if nargin < 1, port = "COM5"; end
if nargin < 2, duration_minutes = 30; end

baud = 115200;
s = serialport(port, baud, "Timeout", 2);
configureTerminator(s, "LF");
flush(s);

try, header = readline(s); %#ok<NASGU>
catch, warning("No se pudo leer encabezado. Continuando..."); end

f = figure('Name','Temp en tiempo real (ESP32)','NumberTitle','off');
ax = axes('Parent', f); grid(ax, 'on'); hold(ax, 'on');
title(ax, 'Temperatura (°C) y salida PID (%)');
xlabel(ax, 'Tiempo (min)');
yyaxis left;
hRaw   = animatedline('LineStyle',':','DisplayName','T cruda');
hFilt  = animatedline('LineWidth',1.5,'DisplayName','T filtrada');
ylabel('Temperatura (°C)');
yyaxis right;
hU     = animatedline('LineWidth',1.0,'DisplayName','u PID (%)');
ylabel('u (%)');
legend('Location','best');

tStart = tic;
while ishandle(f) && toc(tStart) < duration_minutes*60
    if s.NumBytesAvailable == 0, pause(0.02); continue; end
    line = strtrim(readline(s));
    parts = split(line, ',');
    if numel(parts) ~= 7, continue; end
    vals = str2double(parts);
    if any(isnan(vals)), continue; end

    ms   = vals(1);  tRaw = vals(2);  tFilt = vals(3);  uPct = vals(4);
    tMin = ms/60000;

    yyaxis left
    addpoints(hRaw,  tMin, tRaw);
    addpoints(hFilt, tMin, tFilt);

    yyaxis right
    addpoints(hU,    tMin, uPct);

    xlim([max(0, tMin-5)  tMin+0.2]);  % ventana ~5 min
    drawnow limitrate
end
try, clear s; catch, end
end
