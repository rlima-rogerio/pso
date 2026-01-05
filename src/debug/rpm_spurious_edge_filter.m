% ==================================================================
% PSO/EPTA - RPM Spurious Edge Filter
% ==================================================================
% Aplica 4 camadas de filtro para eliminar bordas espúrias:
% 1. Debouncing (tempo mínimo entre edges)
% 2. Validação de consistência (variação máxima)
% 3. Filtro de mediana (3 amostras)
% 4. Timeout detection (motor parado)
% ==================================================================

clear all;
close all;
clc;

%% ===== CONFIGURAÇÃO =====
% load daq_period_us
load daq_period_ticks_75

TICKS_MODE = true;
SAMPLING_RATE = 500;            % Hz
BLADE_NUMBER = 2;               % Número de pás

% --- PARÂMETROS DO FILTRO ---
MIN_EDGE_INTERVAL_US = 500;     % Debouncing: 500 μs (permite até 60k RPM)
MAX_PERIOD_VARIATION_PCT = 50;  % Consistência: 50% variação máxima
MEDIAN_FILTER_SIZE = 3;         % Mediana: 3 amostras
MAX_VALID_PERIOD_US = 500000;   % Timeout: 500 ms (60 RPM mínimo)
MIN_VALID_RPM = 60;             % RPM mínimo considerado válido

%% ===== CONVERSÃO DE DADOS =====
if (TICKS_MODE)
    period_ticks = data.rpm;
    period_us = (period_ticks + 20) / 40;
else
    period_us = data.rpm;
end

t = data.index * (1/SAMPLING_RATE);
numSamples = length(t);

%% ===== INICIALIZAÇÃO =====
rpm_raw = 60000000 ./ (period_us * BLADE_NUMBER);  % RPM bruto (sem filtro)
rpm_filtered = zeros(1, numSamples);               % RPM filtrado
edge_status = zeros(1, numSamples);                % Status da edge (0=rejeitada, 1=aceita)

% Variáveis de estado
last_accepted_idx = 0;          % Índice da última edge aceita
last_valid_period_us = 0;       % Último período válido
median_buffer = zeros(1, MEDIAN_FILTER_SIZE);  % Buffer para mediana
median_count = 0;               % Contador do buffer

% Contadores de diagnóstico
edges_total = 0;
edges_rejected_debounce = 0;
edges_rejected_variation = 0;
edges_rejected_timeout = 0;
edges_accepted = 0;

%% ===== PROCESSAMENTO =====
fprintf('Processando %d amostras...\n', numSamples);

for k = 1:numSamples
    current_period_us = period_us(k);
    edges_total = edges_total + 1;
    
    % =====================================================================
    % FILTRO 1: DEBOUNCING - Rejeitar edges muito próximas
    % =====================================================================
    if (last_accepted_idx > 0)
        time_since_last = (k - last_accepted_idx) * (1/SAMPLING_RATE) * 1e6;  % em μs
        
        if (time_since_last < MIN_EDGE_INTERVAL_US)
            % Edge espúria - REJEITAR
            edges_rejected_debounce = edges_rejected_debounce + 1;
            edge_status(k) = 0;
            rpm_filtered(k) = 0;  % ou manter último valor válido
            continue;
        end
    end
    
    % =====================================================================
    % FILTRO 2: TIMEOUT - Período muito longo = motor parado
    % =====================================================================
    if (current_period_us > MAX_VALID_PERIOD_US)
        edges_rejected_timeout = edges_rejected_timeout + 1;
        edge_status(k) = 0;
        rpm_filtered(k) = 0;
        
        % Reset do estado
        last_valid_period_us = 0;
        median_buffer = zeros(1, MEDIAN_FILTER_SIZE);
        median_count = 0;
        
        last_accepted_idx = k;
        continue;
    end
    
    % =====================================================================
    % FILTRO 3: CONSISTÊNCIA - Validar variação
    % =====================================================================
    if (last_valid_period_us > 0)
        % Calcular variação percentual
        period_diff = abs(current_period_us - last_valid_period_us);
        variation_pct = (period_diff / last_valid_period_us) * 100;
        
        if (variation_pct > MAX_PERIOD_VARIATION_PCT)
            % Variação anormal - REJEITAR
            edges_rejected_variation = edges_rejected_variation + 1;
            edge_status(k) = 0;
            rpm_filtered(k) = 0;
            
            last_accepted_idx = k;  % Atualizar para debouncing
            continue;
        end
    end
    
    % =====================================================================
    % FILTRO 4: MEDIANA - Adicionar ao buffer e calcular mediana
    % =====================================================================
    % Adicionar ao buffer circular
    median_buffer = circshift(median_buffer, -1);
    median_buffer(end) = current_period_us;
    
    if (median_count < MEDIAN_FILTER_SIZE)
        median_count = median_count + 1;
    end
    
    % Calcular mediana
    if (median_count == 1)
        filtered_period_us = median_buffer(1);
    elseif (median_count == 2)
        filtered_period_us = mean(median_buffer(1:2));
    else
        filtered_period_us = median(median_buffer);
    end
    
    % =====================================================================
    % CÁLCULO DO RPM FILTRADO
    % =====================================================================
    calculated_rpm = 60000000 / (filtered_period_us * BLADE_NUMBER);
    
    if (calculated_rpm >= MIN_VALID_RPM)
        % RPM válido - ACEITAR
        rpm_filtered(k) = calculated_rpm;
        last_valid_period_us = filtered_period_us;
        edge_status(k) = 1;
        edges_accepted = edges_accepted + 1;
    else
        % RPM muito baixo - considerar motor parado
        rpm_filtered(k) = 0;
        edge_status(k) = 0;
    end
    
    % Atualizar última edge aceita (para debouncing)
    last_accepted_idx = k;
end

%% ===== ESTATÍSTICAS =====
fprintf('\n===== ESTATÍSTICAS DE FILTRAGEM =====\n');
fprintf('Total de edges:              %d\n', edges_total);
fprintf('Edges aceitas:               %d (%.1f%%)\n', edges_accepted, ...
    (edges_accepted/edges_total)*100);
fprintf('Rejeitadas (debouncing):     %d (%.1f%%)\n', edges_rejected_debounce, ...
    (edges_rejected_debounce/edges_total)*100);
fprintf('Rejeitadas (variação):       %d (%.1f%%)\n', edges_rejected_variation, ...
    (edges_rejected_variation/edges_total)*100);
fprintf('Rejeitadas (timeout):        %d (%.1f%%)\n', edges_rejected_timeout, ...
    (edges_rejected_timeout/edges_total)*100);

rejection_rate = ((edges_rejected_debounce + edges_rejected_variation + ...
    edges_rejected_timeout) / edges_total) * 100;
fprintf('Taxa de rejeição total:      %.1f%%\n', rejection_rate);
fprintf('======================================\n\n');

%% ===== GRÁFICOS =====
figure('Position', [100, 100, 1200, 900]);

% Subplot 1: Throttle
subplot(4,1,1);
plot(t, data.throttle, 'g-', 'LineWidth', 1.5);
ylabel('Throttle');
title('PSO/EPTA - Filtro de Bordas Espúrias em RPM');
grid on;
xlim([min(t) max(t)]);

% Subplot 2: Período (com marcadores de edges rejeitadas/aceitas)
subplot(4,1,2);
hold on;

% Plotar todas as medições
plot(t, period_us / 1000, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);

% Destacar edges aceitas (verde)
accepted_idx = find(edge_status == 1);
if ~isempty(accepted_idx)
    plot(t(accepted_idx), period_us(accepted_idx) / 1000, 'go', ...
        'MarkerSize', 3, 'MarkerFaceColor', 'g');
end

% Destacar edges rejeitadas (vermelho)
rejected_idx = find(edge_status == 0);
if ~isempty(rejected_idx)
    plot(t(rejected_idx), period_us(rejected_idx) / 1000, 'rx', ...
        'MarkerSize', 4);
end

ylabel('Período (ms)');
legend('Todas medições', 'Aceitas', 'Rejeitadas', 'Location', 'best');
grid on;
xlim([min(t) max(t)]);
hold off;

% Subplot 3: RPM Bruto vs Filtrado
subplot(4,1,3);
hold on;
plot(t, rpm_raw, 'Color', [1 0.5 0.5], 'LineWidth', 0.5);  % Rosa claro
plot(t, rpm_filtered, 'g-', 'LineWidth', 2);                % Verde
ylabel('RPM');
legend('RPM Bruto (sem filtro)', 'RPM Filtrado', 'Location', 'best');
grid on;
xlim([min(t) max(t)]);
ylim([0 max(rpm_filtered)*1.1]);
hold off;

% Subplot 4: Zoom em região com espúrias (primeiros 0.5s)
subplot(4,1,4);
zoom_end_idx = find(t <= 0.5, 1, 'last');
if ~isempty(zoom_end_idx)
    hold on;
    plot(t(1:zoom_end_idx), rpm_raw(1:zoom_end_idx), 'r-', 'LineWidth', 1);
    plot(t(1:zoom_end_idx), rpm_filtered(1:zoom_end_idx), 'g-', 'LineWidth', 2);
    ylabel('RPM');
    xlabel('Tempo (s)');
    legend('RPM Bruto', 'RPM Filtrado', 'Location', 'best');
    title('Zoom: Primeiros 0.5s (mostrando eliminação de espúrias)');
    grid on;
    xlim([0 0.5]);
    hold off;
else
    xlabel('Tempo (s)');
end

%% ===== ANÁLISE ADICIONAL =====
% Calcular taxa de rejeição por faixa de tempo
window_size = 1.0;  % 1 segundo
num_windows = floor(max(t) / window_size);

fprintf('Taxa de rejeição por intervalo de tempo:\n');
for w = 1:min(num_windows, 10)  % Mostrar apenas primeiros 10 segundos
    start_idx = find(t >= (w-1)*window_size, 1, 'first');
    end_idx = find(t < w*window_size, 1, 'last');
    
    if ~isempty(start_idx) && ~isempty(end_idx)
        window_rejected = sum(edge_status(start_idx:end_idx) == 0);
        window_total = end_idx - start_idx + 1;
        window_rejection_rate = (window_rejected / window_total) * 100;
        
        fprintf('  [%.1f - %.1f]s: %.1f%% rejeitadas\n', ...
            (w-1)*window_size, w*window_size, window_rejection_rate);
    end
end

%% ===== SALVAR RESULTADOS =====
results.t = t;
results.period_us = period_us;
results.rpm_raw = rpm_raw;
results.rpm_filtered = rpm_filtered;
results.edge_status = edge_status;
results.stats.edges_total = edges_total;
results.stats.edges_accepted = edges_accepted;
results.stats.edges_rejected_debounce = edges_rejected_debounce;
results.stats.edges_rejected_variation = edges_rejected_variation;
results.stats.edges_rejected_timeout = edges_rejected_timeout;
results.stats.rejection_rate = rejection_rate;

save('rpm_filtered_results.mat', 'results');
fprintf('\nResultados salvos em: rpm_filtered_results.mat\n');
