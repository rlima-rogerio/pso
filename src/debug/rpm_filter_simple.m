% ==================================================================
% VERSÃO SIMPLIFICADA - Filtro de Bordas Espúrias
% ==================================================================
% Mais compacta e fácil de integrar
% ==================================================================

clear all;
close all;
clc;

%% CONFIGURAÇÃO
load daq_period_ticks_75

TICKS_MODE = true;
SAMPLING_RATE = 500;
BLADE_NUMBER = 2;

% Parâmetros do filtro (ajustar conforme necessário)
MIN_EDGE_INTERVAL_US = 500;      % Debouncing
MAX_PERIOD_VARIATION_PCT = 50;   % Consistência
MAX_VALID_PERIOD_US = 500000;    % Timeout
MIN_VALID_RPM = 60;              % RPM mínimo

%% CONVERSÃO
if (TICKS_MODE)
    period_us = (data.rpm + 20) / 40;
else
    period_us = data.rpm;
end

t = data.index * (1/SAMPLING_RATE);
n = length(t);

%% PROCESSAMENTO
rpm_filt = zeros(1, n);
last_idx = 0;
last_valid = 0;
buffer = zeros(1, 3);  % Buffer mediana
buf_cnt = 0;

stats.accept = 0;
stats.reject_deb = 0;
stats.reject_var = 0;
stats.reject_tout = 0;

for k = 1:n
    period = period_us(k);
    
    % === FILTRO 1: Debouncing ===
    if (last_idx > 0)
        dt = (k - last_idx) / SAMPLING_RATE * 1e6;  % μs
        if (dt < MIN_EDGE_INTERVAL_US)
            stats.reject_deb = stats.reject_deb + 1;
            continue;
        end
    end
    
    % === FILTRO 2: Timeout ===
    if (period > MAX_VALID_PERIOD_US)
        stats.reject_tout = stats.reject_tout + 1;
        rpm_filt(k) = 0;
        last_valid = 0;
        buffer = zeros(1,3);
        buf_cnt = 0;
        last_idx = k;
        continue;
    end
    
    % === FILTRO 3: Consistência ===
    if (last_valid > 0)
        var = abs(period - last_valid) / last_valid * 100;
        if (var > MAX_PERIOD_VARIATION_PCT)
            stats.reject_var = stats.reject_var + 1;
            last_idx = k;
            continue;
        end
    end
    
    % === FILTRO 4: Mediana ===
    buffer = circshift(buffer, -1);
    buffer(end) = period;
    buf_cnt = min(buf_cnt + 1, 3);
    
    if (buf_cnt == 1)
        filt_period = buffer(1);
    elseif (buf_cnt == 2)
        filt_period = mean(buffer(1:2));
    else
        filt_period = median(buffer);
    end
    
    % === CÁLCULO RPM ===
    rpm = 60000000 / (filt_period * BLADE_NUMBER);
    
    if (rpm >= MIN_VALID_RPM)
        rpm_filt(k) = rpm;
        last_valid = filt_period;
        stats.accept = stats.accept + 1;
    else
        rpm_filt(k) = 0;
    end
    
    last_idx = k;
end

%% ESTATÍSTICAS
total = n;
fprintf('\n=== ESTATÍSTICAS ===\n');
fprintf('Total:      %d\n', total);
fprintf('Aceitas:    %d (%.1f%%)\n', stats.accept, stats.accept/total*100);
fprintf('Rej. Deb:   %d (%.1f%%)\n', stats.reject_deb, stats.reject_deb/total*100);
fprintf('Rej. Var:   %d (%.1f%%)\n', stats.reject_var, stats.reject_var/total*100);
fprintf('Rej. Tout:  %d (%.1f%%)\n', stats.reject_tout, stats.reject_tout/total*100);
rej_total = stats.reject_deb + stats.reject_var + stats.reject_tout;
fprintf('Rejeição:   %.1f%%\n', rej_total/total*100);

%% GRÁFICOS
figure('Position', [100, 100, 1200, 700]);

subplot(3,1,1);
plot(t, data.throttle, 'g-', 'LineWidth', 1.5);
ylabel('Throttle');
title('Filtro de Bordas Espúrias');
grid on;

subplot(3,1,2);
plot(t, period_us/1000, 'Color', [0.7 0.7 0.7]);
ylabel('Período (ms)');
grid on;

subplot(3,1,3);
rpm_raw = 60000000 ./ (period_us * BLADE_NUMBER);
hold on;
plot(t, rpm_raw, 'Color', [1 0.5 0.5]);
plot(t, rpm_filt, 'g-', 'LineWidth', 2);
ylabel('RPM');
xlabel('Tempo (s)');
legend('Bruto', 'Filtrado');
grid on;
hold off;

% Zoom primeiros 0.5s
figure;
idx = find(t >= 0.5, 1, 'last');
if ~isempty(idx)
    hold on;
    plot(t(1:idx), rpm_raw(1:idx), 'r-');
    plot(t(1:idx), rpm_filt(1:idx), 'g-', 'LineWidth', 2);
    xlabel('Tempo (s)');
    ylabel('RPM');
    title('Zoom: Primeiros 0.5s');
    legend('Bruto', 'Filtrado');
    grid on;
end
