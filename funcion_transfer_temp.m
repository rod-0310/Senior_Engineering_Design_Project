%% ================== CONFIGURACIÓN DE CONEXIÓN ==================
PORT = "COM4";   % <-- Cambia a tu puerto (Windows: "COMx", Linux: "/dev/ttyUSB0")
BAUD = 115200;

BASE_SEC = 60;
STEP_SEC = 30*60;
POST_SEC = 60;
TMAX = BASE_SEC + STEP_SEC + POST_SEC + 120;  % margen extra

%% ================== CONFIGURACIÓN DEL PUERTO SERIE ==================
if ~isempty(instrfind) %#ok<INSTRFND>
    fclose(instrfind); delete(instrfind);
end

sp = serialport(PORT, BAUD);
configureTerminator(sp, "LF");
flush(sp);

disp("Esperando cabecera 't,u,y'...");
headerFound = false;
tic;
while toc < 5
    if sp.NumBytesAvailable > 0
        ln = strtrim(readline(sp));
        if strcmpi(ln, "t,u,y")
            headerFound = true;
            disp("Cabecera encontrada. Iniciando lectura...");
            break;
        end
    end
end
if ~headerFound
    warning("No se detectó cabecera, continuaré de todos modos...");
end

%% ================== VARIABLES DE ALMACENAMIENTO ==================
t = []; u = []; y = [];

%% ================== CONFIGURACIÓN DE LA GRÁFICA ==================
figure('Name','Temperatura en tiempo real','NumberTitle','off');
tiledlayout(2,1,"TileSpacing","compact","Padding","compact");

ax1 = nexttile;
h1 = plot(ax1, NaN, NaN, 'LineWidth',1.6,'Color',[0 0.4 1]);
title(ax1,'Entrada u(t) - Step del dimmer');
xlabel(ax1,'Tiempo (s)'); ylabel(ax1,'u [0..1]');
grid(ax1,'on'); ylim(ax1,[-0.1 1.1]);

ax2 = nexttile;
h2 = plot(ax2, NaN, NaN, 'r','LineWidth',1.8);
title(ax2,'Salida y(t) - Temperatura');
xlabel(ax2,'Tiempo (s)'); ylabel(ax2,'Temperatura [°C]');
grid(ax2,'on'); ylim(ax2,[20 80]);

%% ================== BUCLE DE ADQUISICIÓN EN TIEMPO REAL ==================
tStart = tic;
updateRate = 0.2; % actualizar gráfico cada 0.2 s
lastUpdate = tic;

while toc(tStart) < TMAX
    if sp.NumBytesAvailable == 0
        pause(0.01);
        continue;
    end

    % Leer línea CSV: t,u,y
    ln = strtrim(readline(sp));
    if isempty(ln) || ~contains(ln,",")
        continue;
    end
    parts = split(ln, ',');
    if numel(parts) ~= 3
        continue;
    end

    ti = str2double(parts{1});
    ui = str2double(parts{2});
    yi = str2double(parts{3});

    if any(isnan([ti,ui,yi]))
        continue;
    end

    % Guardar datos
    t(end+1,1) = ti;
    u(end+1,1) = ui;
    y(end+1,1) = yi;

    % Actualizar gráficos cada cierto tiempo
    if toc(lastUpdate) > updateRate
        set(h1, 'XData', t, 'YData', u);
        set(h2, 'XData', t, 'YData', y);
        xlim(ax1,[max(0,ti-120) ti+5]); % ventana móvil de ~2 minutos
        xlim(ax2,[max(0,ti-120) ti+5]);
        title(ax2, sprintf('Temperatura (%.1f °C)  |  t = %.0f s', yi, ti));
        drawnow limitrate;
        lastUpdate = tic;
    end
end

%% ================== CONVERTIR A VECTORES FILA ==================
u = u.'; y = y.'; t = t.';
disp('Adquisición terminada.');
whos t u y
