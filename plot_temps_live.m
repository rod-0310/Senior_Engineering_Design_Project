%% plot_realtime_pid_v2.m — versión con colores y estilos mejorados
clear; clc;

COM = "COM4";           % ⚠️ cambia a tu puerto
BAUD = 115200;
RUN_SECONDS = inf;
SAVE_CSV = true;
OUTFILE = "log_pid_realtime.csv";

colNames = ["ms","T_rawC","T_filtC","u_PID_pct","SP_C","Fan_in_pct","Fan_out_pct","safety","u_delay_us","gate"];

try
    sp = serialport(COM, BAUD, "Timeout", 1);
catch ME
    error("No se pudo abrir %s: %s", COM, ME.message);
end

configureTerminator(sp,"LF");
flush(sp);

% Esperar encabezado CSV
fprintf("Esperando encabezado desde ESP32...\n");
headerFound = false; tStart = tic;
while ~headerFound && toc(tStart)<10
    if sp.NumBytesAvailable > 0
        ln = strtrim(readline(sp));
        if strcmpi(ln, "ms,T_rawC,T_filtC,u_PID_pct,SP_C,Fan_in_pct,Fan_out_pct,safety,u_delay_us,gate")
            headerFound = true;
            fprintf("Encabezado detectado correctamente.\n");
        end
    end
end

%% === FIGURA ===
fig = figure('Name','PID temperatura tiempo real','NumberTitle','off');
tiledlayout(3,1);

ax1 = nexttile; hold on;
hTraw  = animatedline('Color',[0.85 0.33 0.1],'LineWidth',1.5);     % naranja
hTfilt = animatedline('Color',[0.0 0.45 0.74],'LineWidth',1.5);     % azul
hSP    = animatedline('Color',[0.47 0.67 0.19],'LineWidth',1.5,'LineStyle','--'); % verde punteado
title(ax1,'Temperaturas'); ylabel(ax1,'°C'); grid(ax1,'on');
legend(ax1,{'T_{raw}','T_{filt}','Setpoint'},'Location','best');

ax2 = nexttile; hold on;
hU = animatedline('Color',[0.85 0.33 0.1],'LineWidth',1.5); % naranja
title(ax2,'Salida PID (Heater)'); ylabel(ax2,'Potencia (%)');
ylim(ax2,[0 100]); grid(ax2,'on');

ax3 = nexttile; hold on;
hFanIn  = animatedline('Color',[0 0.6 1],'LineWidth',1.5);     % celeste
hFanOut = animatedline('Color',[0.93 0.69 0.13],'LineWidth',1.5); % dorado
title(ax3,'Ventiladores'); ylabel(ax3,'Duty (%)'); xlabel(ax3,'Tiempo (s)');
ylim(ax3,[0 100]); grid(ax3,'on');
legend(ax3,{'Fan In','Fan Out'},'Location','best');

%% === LOOP PRINCIPAL ===
data = [];
t0 = NaN;
fprintf("Leyendo datos... (cierra la ventana para terminar)\n");

while isvalid(fig)
    if sp.NumBytesAvailable == 0
        pause(0.02);
        continue;
    end

    ln = strtrim(readline(sp));
    if ln == "" || startsWith(ln,"Inicio") || startsWith(ln,"Comandos") || startsWith(ln,"ms,")
        continue;
    end

    parts = split(ln, ',');
    if numel(parts) ~= 10, continue; end
    nums = str2double(parts);
    if any(isnan(nums)), continue; end

    ms = nums(1);
    if isnan(t0), t0 = ms; end
    tsec = (ms - t0)/1000;

    % Añadir puntos
    addpoints(hTraw,  tsec, nums(2));
    addpoints(hTfilt, tsec, nums(3));
    addpoints(hSP,    tsec, nums(5));

    addpoints(hU,     tsec, nums(4));

    addpoints(hFanIn,  tsec, nums(6));
    addpoints(hFanOut, tsec, nums(7));

    % Scroll
    ax1.XLim = [max(0,tsec-120) tsec+2];
    ax2.XLim = ax1.XLim; ax3.XLim = ax1.XLim;

    drawnow limitrate nocallbacks;

    data = [data; nums(:)'];
end

%% === GUARDAR DATOS ===
if SAVE_CSV && ~isempty(data)
    T = array2table(data,'VariableNames',cellstr(colNames));
    try
        writetable(T, OUTFILE);
        fprintf("✅ Datos guardados en '%s'\n", OUTFILE);
    catch ME
        warning("⚠️ No se pudo guardar CSV: %s", ME.message);
    end
end

if exist('sp','var'), delete(sp); end
fprintf("Sesión finalizada.\n");
