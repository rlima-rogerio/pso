function [t_valid, rpm_valid, rpm_filtered] = process_rpm_data_with_filtering(t, period_us, BLADE_NUMBER)
    % =========================================================================
    % RPM PROCESSING WITH SPURIOUS EDGE FILTERING
    % =========================================================================
    % Inputs:
    %   t - vetor de tempo (s)
    %   period_us - vetor de períodos entre bordas (μs)
    %   BLADE_NUMBER - número de pás da hélice
    %
    % Outputs:
    %   t_valid - timestamps válidos para cálculo de RPM
    %   rpm_valid - RPM calculado a partir de períodos válidos
    %   rpm_filtered - RPM filtrado (média móvel)
    % =========================================================================
    
    % 1. PARÂMETROS DE FILTRAGEM (ajustáveis conforme seu sistema)
    MIN_VALID_PERIOD_US = 1000;     % Mínimo período realista (1ms = 1000μs)
    MAX_VALID_PERIOD_US = 50000;    % Máximo período realista (50ms = 50000μs)
    MIN_CONSECUTIVE_VALID = 2;      % Número mínimo de bordas válidas consecutivas
    OUTLIER_THRESHOLD = 3;          % Desvio padrão para detecção de outliers
    FILTER_WINDOW = 5;              % Tamanho da janela de média móvel
    
    % 2. VALIDAÇÃO INICIAL DE PERÍODOS
    % Identificar períodos dentro da faixa realista
    valid_idx = (period_us >= MIN_VALID_PERIOD_US) & ...
                (period_us <= MAX_VALID_PERIOD_US);
    
    % 3. DETECÇÃO DE OUTLIERS ESTATÍSTICOS
    % Calcular estatísticas dos períodos válidos
    valid_periods = period_us(valid_idx);
    mean_period = mean(valid_periods);
    std_period = std(valid_periods);
    
    % Remover outliers baseado em desvio padrão
    outlier_idx = abs(period_us - mean_period) > (OUTLIER_THRESHOLD * std_period);
    valid_idx = valid_idx & ~outlier_idx;
    
    % 4. VALIDAÇÃO POR CONSISTÊNCIA TEMPORAL
    % Garantir que bordas válidas ocorram em sequência consistente
    valid_periods_filtered = period_us;
    valid_periods_filtered(~valid_idx) = NaN;
    
    % Vetores de saída
    t_valid = [];
    rpm_valid = [];
    
    % 5. PROCESSAMENTO POR JANELAS DESLIZANTES
    for i = 1:length(t)
        if valid_idx(i)
            % Verificar consistência com períodos vizinhos
            window_start = max(1, i-2);
            window_end = min(length(t), i+2);
            
            % Contar quantos períodos válidos na janela
            valid_in_window = sum(valid_idx(window_start:window_end));
            
            if valid_in_window >= MIN_CONSECUTIVE_VALID
                % Calcular RPM para este período válido
                current_rpm = 60000000 / (period_us(i) * BLADE_NUMBER);
                
                % Aplicar validação de plausibilidade de RPM
                if current_rpm >= 50 && current_rpm <= 20000
                    t_valid = [t_valid; t(i)];
                    rpm_valid = [rpm_valid; current_rpm];
                end
            end
        end
    end
    
    % 6. FILTRAGEM POR MÉDIA MÓVEL
    if length(rpm_valid) >= FILTER_WINDOW
        % Aplicar média móvel
        rpm_filtered = movmean(rpm_valid, FILTER_WINDOW);
    else
        rpm_filtered = rpm_valid;
    end
    
    % 7. INTERPOLAÇÃO PARA VETOR DE TEMPO ORIGINAL (opcional)
    % Interpolar os RPM válidos para o vetor de tempo original
    if ~isempty(t_valid) && length(t_valid) > 1
        % Remover duplicatas no tempo (pode acontecer)
        [t_unique, idx_unique] = unique(t_valid);
        rpm_unique = rpm_valid(idx_unique);
        
        % Interpolar usando interp1 (linear por padrão)
        rpm_interpolated = interp1(t_unique, rpm_unique, t, 'linear', 'extrap');
        
        % Aplicar filtro passa-baixas suave na interpolação
        rpm_smoothed = smoothdata(rpm_interpolated, 'movmean', FILTER_WINDOW);
        
        % Substituir NaNs por 0
        rpm_smoothed(isnan(rpm_smoothed)) = 0;
        
        % Ajustar valores fora da faixa realista
        rpm_smoothed(rpm_smoothed < 0) = 0;
        rpm_smoothed(rpm_smoothed > 20000) = 0;
        
        % Usar a versão suavizada como saída principal
        rpm_filtered = rpm_smoothed;
    else
        % Caso não haja dados suficientes
        rpm_filtered = zeros(size(t));
    end
    
    % 8. PLOTS DIAGNÓSTICOS (opcional)
    figure('Position', [100, 100, 1200, 800]);
    
    % Subplot 1: Períodos brutos vs filtrados
    subplot(3,1,1);
    plot(t, period_us, 'r.', 'MarkerSize', 5); hold on;
    plot(t(valid_idx), period_us(valid_idx), 'g.', 'MarkerSize', 8);
    xlabel('Tempo (s)');
    ylabel('Período (μs)');
    title('Períodos entre Bordas - Bruto (vermelho) vs Válido (verde)');
    grid on;
    legend('Bruto', 'Válido');
    
    % Subplot 2: RPM calculado
    subplot(3,1,2);
    if ~isempty(t_valid)
        plot(t_valid, rpm_valid, 'b-', 'LineWidth', 1.5); hold on;
        plot(t, rpm_filtered, 'r-', 'LineWidth', 1.5);
        xlabel('Tempo (s)');
        ylabel('RPM');
        title('RPM Calculado - Pontos válidos (azul) vs Filtrado (vermelho)');
        grid on;
        legend('RPM válido', 'RPM filtrado');
    end
    
    % Subplot 3: Histograma de períodos
    subplot(3,1,3);
    histogram(period_us, 'BinWidth', 100, 'FaceColor', 'r', 'EdgeColor', 'k');
    hold on;
    histogram(period_us(valid_idx), 'BinWidth', 100, 'FaceColor', 'g', 'EdgeColor', 'k');
    xlabel('Período (μs)');
    ylabel('Frequência');
    title('Distribuição de Períodos');
    grid on;
    legend('Todos os períodos', 'Períodos válidos');
    
    % 9. ESTATÍSTICAS DE SAÍDA
    fprintf('=============================================\n');
    fprintf('ESTATÍSTICAS DO PROCESSAMENTO DE RPM\n');
    fprintf('=============================================\n');
    fprintf('Total de amostras: %d\n', length(t));
    fprintf('Períodos válidos: %d (%.1f%%)\n', sum(valid_idx), ...
            100*sum(valid_idx)/length(t));
    fprintf('Período médio válido: %.2f μs\n', mean(period_us(valid_idx)));
    fprintf('RPM médio: %.0f\n', mean(rpm_valid));
    fprintf('RPM máximo: %.0f\n', max(rpm_valid));
    fprintf('RPM mínimo: %.0f\n', min(rpm_valid));
    fprintf('=============================================\n');
    
    % 10. FUNÇÃO AUXILIAR PARA DETECÇÃO DE MUDANÇAS BRUSCAS
    % (Útil para identificar transições válidas)
    function [transitions] = detect_transitions(signal, threshold)
        diff_signal = diff(signal);
        transitions = find(abs(diff_signal) > threshold) + 1;
    end
end