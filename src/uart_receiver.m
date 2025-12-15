% PSO Data Acquisition - MATLAB Receiver (VERSÃO FINAL CORRIGIDA)
% ==================================================================

clear all;
close all;
clc;

%% Configurações
sys = computer;

switch sys
    case {'PCWIN64', 'PCWIN'}
        PORT = 'COM3';
        
    case {'GLNXA64', 'GLNX86'}
        PORT = '/dev/ttyACM0';
        
    otherwise
        error('Sistema operacional não suportado: %s', sys);
end

BAUDRATE = 115200;
OUTPUT_MAT = 'daq.mat';
OUTPUT_TXT = 'daq.txt';

% Constantes do protocolo
STX = hex2dec('FE');
PACKET_LENGTH = 21;
MSG_ID_PSO_DATA_LEN = 14;

%% Funções de Checksum (X.25 CRC-16)
function crc = crc_init()
    crc = uint16(hex2dec('FFFF'));
end
function crc = accumulate_checksum(data, crc)
    % Implementação exata do código C com conversões corretas
    data = uint8(data);
    crc = uint16(crc);
    
    % tmp = data ^ (crc & 0xff)
    tmp = bitxor(uint8(data), uint8(bitand(crc, uint16(0x00FF))));
    
    % tmp ^= (tmp << 4) & 0xff
    tmp = bitxor(tmp, uint8(bitand(bitshift(uint16(tmp), 4), uint16(0x00FF))));
    
    % crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    crc = bitxor(bitshift(crc, -8), bitshift(uint16(tmp), 8));
    crc = bitxor(crc, bitshift(uint16(tmp), 3));
    crc = bitxor(crc, bitshift(uint16(tmp), -4));
    
    crc = uint16(crc);
end

function crc = calculate_checksum(packet_bytes)
    % Replica a lógica exata do C
    % create_checksum processa msg[1] até msg[18] (19 bytes)
    crc = crc_init();
    
    for i = 2:19  % Bytes 1-18 do protocolo (índices MATLAB 2-19)
        crc = accumulate_checksum(packet_bytes(i), crc);
    end
end

%% Inicializar porta serial
fprintf('Conectando a %s @ %d bps...\n', PORT, BAUDRATE);

try
    s = serialport(PORT, BAUDRATE);
    s.Timeout = 1;
    fprintf('Conectado com sucesso!\n\n');
catch ME
    error('Erro ao conectar: %s', ME.message);
end

%% Inicializar buffers de dados
max_samples = 10000;
idx = 0;

data.time = zeros(max_samples, 1);
data.index = zeros(max_samples, 1);
data.rpm = zeros(max_samples, 1);
data.throttle = zeros(max_samples, 1);
data.accel_x = zeros(max_samples, 1);
data.accel_y = zeros(max_samples, 1);
data.accel_z = zeros(max_samples, 1);
data.current = zeros(max_samples, 1);
data.voltage = zeros(max_samples, 1);
data.thrust = zeros(max_samples, 1);
data.power = zeros(max_samples, 1);

packet_count = 0;
error_count = 0;
checksum_errors = 0;
start_time = tic;

%% Coleta de dados
fprintf('=== Iniciando coleta de dados ===\n');
fprintf('Pressione Ctrl+C para parar\n');
fprintf('Timeout automático: 3 segundos sem dados\n\n');
fprintf('%-8s %-6s %-5s %-10s %-8s %-8s %-8s\n', ...
        'Pacotes', 'RPM', 'Thr%', 'Corrente', 'Tensão', 'Empuxo', 'Taxa');
fprintf('%s\n', repmat('-', 1, 70));

INACTIVITY_TIMEOUT = 5.0;
last_packet_time = tic;
sync_buffer = [];

try
    while true
        % Verificar timeout
        if toc(last_packet_time) > INACTIVITY_TIMEOUT
            fprintf('\n\n=== Timeout: %.1f segundos sem dados ===\n', INACTIVITY_TIMEOUT);
            break;
        end
        
        % Ler novos dados
        if s.NumBytesAvailable > 0
            new_bytes = read(s, s.NumBytesAvailable, 'uint8');
            sync_buffer = [sync_buffer; new_bytes(:)];
        else
            pause(0.001);
            continue;
        end
        
        % Processar buffer
        while length(sync_buffer) >= PACKET_LENGTH
            % Procurar STX
            stx_pos = find(sync_buffer == STX, 1, 'first');
            
            if isempty(stx_pos)
                sync_buffer = [];
                break;
            end
            
            % Alinhar ao STX
            if stx_pos > 1
                sync_buffer = sync_buffer(stx_pos:end);
            end
            
            % Verificar se temos pacote completo
            if length(sync_buffer) < PACKET_LENGTH
                break;
            end
            
            % Extrair pacote
            packet_data = sync_buffer(1:PACKET_LENGTH);
            
            % Validação 1: Verificar byte de length
            if packet_data(2) ~= MSG_ID_PSO_DATA_LEN
                error_count = error_count + 1;
                sync_buffer = sync_buffer(2:end);
                continue;
            end
            
            % Validação 2: Calcular e verificar checksum
            calculated_crc = calculate_checksum(packet_data);
            received_crc = uint16(packet_data(20)) * 256 + uint16(packet_data(21));
            
            if calculated_crc ~= received_crc
                checksum_errors = checksum_errors + 1;
                error_count = error_count + 1;
                sync_buffer = sync_buffer(2:end);
                continue;
            end
            
            % Parse do pacote (agora sabemos que é válido!)
            try
                % Index (bytes 2-3, big-endian)
                pkt_index = uint16(packet_data(3)) * 256 + uint16(packet_data(4));
                
                % Accel X (bytes 4-5, big-endian, signed)
                accel_x = double(int16(uint16(packet_data(5)) * 256 + uint16(packet_data(6))));
                
                % Accel Y (bytes 6-7)
                accel_y = double(int16(uint16(packet_data(7)) * 256 + uint16(packet_data(8))));
                
                % Accel Z (bytes 8-9)
                accel_z = double(int16(uint16(packet_data(9)) * 256 + uint16(packet_data(10))));
                
                % RPM (bytes 10-11, unsigned)
                rpm = uint16(packet_data(11)) * 256 + uint16(packet_data(12));
                
                % Current (bytes 12-13, signed, em mA)
                current_raw = double(uint16(uint16(packet_data(13)) * 256 + uint16(packet_data(14))));
                
                % Voltage (bytes 14-15, signed, em mV)
                voltage_raw = double(uint16(uint16(packet_data(15)) * 256 + uint16(packet_data(16))));
                
                % Thrust (bytes 16-17, signed, em cN)
                thrust_raw = double(int16(uint16(packet_data(17)) * 256 + uint16(packet_data(18))));
                
                % Throttle (byte 18)
                throttle = packet_data(19);
                
                % Converter para unidades reais
                current = current_raw / 1000.0;
                voltage = voltage_raw / 1000.0;
                thrust = thrust_raw / 100.0;
                power = voltage * current;
                
                % Armazenar dados
                idx = idx + 1;
                
                if idx > length(data.time)
                    new_size = length(data.time) + max_samples;
                    data.time(new_size) = 0;
                    data.index(new_size) = 0;
                    data.rpm(new_size) = 0;
                    data.throttle(new_size) = 0;
                    data.accel_x(new_size) = 0;
                    data.accel_y(new_size) = 0;
                    data.accel_z(new_size) = 0;
                    data.current(new_size) = 0;
                    data.voltage(new_size) = 0;
                    data.thrust(new_size) = 0;
                    data.power(new_size) = 0;
                end
                
                elapsed_time = toc(start_time);
                data.time(idx) = elapsed_time;
                data.index(idx) = double(pkt_index);
                data.rpm(idx) = double(rpm);
                data.throttle(idx) = double(throttle);
                data.accel_x(idx) = accel_x;
                data.accel_y(idx) = accel_y;
                data.accel_z(idx) = accel_z;
                data.current(idx) = current;
                data.voltage(idx) = voltage;
                data.thrust(idx) = thrust;
                data.power(idx) = power;
                
                packet_count = packet_count + 1;
                last_packet_time = tic;
                
                % Imprimir a cada 10 pacotes
                if mod(packet_count, 10) == 0
                    rate = packet_count / elapsed_time;
                    fprintf('%-8d %-6d %-5d %-10.3f %-8.2f %-8.2f %-8.1f\n', ...
                            packet_count, rpm, throttle, current, voltage, thrust, rate);
                end
                
                % Remover pacote processado
                sync_buffer = sync_buffer(PACKET_LENGTH+1:end);
                
            catch ME
                error_count = error_count + 1;
                sync_buffer = sync_buffer(2:end);
            end
        end
    end
    
catch ME
    if strcmp(ME.identifier, 'MATLAB:interruption')
        fprintf('\n\n=== Coleta interrompida ===\n');
    else
        fprintf('\n\nErro: %s\n', ME.message);
    end
end

%% Fechar porta
clear s;
fprintf('Porta serial fechada\n');

%% Processar dados coletados
if idx == 0
    fprintf('Nenhum dado foi coletado!\n');
    return;
end

SAMPLING_PERIOD = 0.002; % 500 Hz => 2 ms 

% Truncar arrays
data.time = (0:(idx-1)) * SAMPLING_PERIOD; %data.time(1:idx);
data.index = data.index(1:idx);
data.rpm = data.rpm(1:idx);
data.throttle = data.throttle(1:idx);
data.accel_x = data.accel_x(1:idx);
data.accel_y = data.accel_y(1:idx);
data.accel_z = data.accel_z(1:idx);
data.current = data.current(1:idx);
data.voltage = data.voltage(1:idx);
data.thrust = data.thrust(1:idx);
data.power = data.power(1:idx);

total_time = data.time(end);
avg_rate = packet_count / total_time;
error_rate = (error_count / (packet_count + error_count)) * 100;

fprintf('\n=== Estatísticas ===\n');
fprintf('Pacotes válidos: %d\n', packet_count);
fprintf('Erros totais: %d (%.2f%%)\n', error_count, error_rate);
fprintf('Erros de checksum: %d\n', checksum_errors);
fprintf('Tempo total: %.2f s\n', total_time);
fprintf('Taxa média: %.1f pkt/s\n', avg_rate);

%% Salvar dados em arquivo .mat
fprintf('\nSalvando dados em %s...\n', OUTPUT_MAT);
save(OUTPUT_MAT, 'data', 'packet_count', 'error_count', ...
     'total_time', 'avg_rate');
fprintf('Arquivo .mat salvo!\n');

%% Salvar dados em arquivo .txt
fprintf('Salvando dados em %s...\n', OUTPUT_TXT);
fid = fopen(OUTPUT_TXT, 'w');

fprintf(fid, 'Time(s),Index,RPM,Throttle(%%),AccelX,AccelY,AccelZ,');
fprintf(fid, 'Current(A),Voltage(V),Thrust(N),Power(W)\n');

for i = 1:idx
    fprintf(fid, '%.3f,%d,%d,%d,%d,%d,%d,%.3f,%.2f,%.2f,%.2f\n', ...
            data.time(i), data.index(i), data.rpm(i), data.throttle(i), ...
            data.accel_x(i), data.accel_y(i), data.accel_z(i), ...
            data.current(i), data.voltage(i), data.thrust(i), data.power(i));
end

fclose(fid);
fprintf('Arquivo .txt salvo!\n');

%% Plotar gráficos
fprintf('\nGerando gráficos...\n');

figure('Name', 'PSO Data Acquisition - Analysis', ...
       'Position', [100, 100, 1200, 800]);

% Subplot 1: RPM
subplot(3, 2, 1);
plot(data.time, data.rpm, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('RPM');
title('Rotação do Motor');

% Subplot 2: Throttle
subplot(3, 2, 2);
plot(data.time, data.throttle, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Throttle (%)');
title('Posição do Acelerador');
ylim([0 105]);

% Subplot 3: Corrente
subplot(3, 2, 3);
plot(data.time, data.current, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Corrente (A)');
title('Corrente Elétrica');

% Subplot 4: Tensão
subplot(3, 2, 4);
plot(data.time, data.voltage, 'Color', [1 0.5 0], 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Tensão (V)');
title('Tensão de Alimentação');

% Subplot 5: Empuxo
subplot(3, 2, 5);
plot(data.time, data.thrust, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Empuxo (N)');
title('Força de Empuxo');

% Subplot 6: Potência
subplot(3, 2, 6);
plot(data.time, data.power, 'c-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Potência (W)');
title('Potência Elétrica');

sgtitle('PSO Data Acquisition - Análise de Dados', 'FontSize', 14, 'FontWeight', 'bold');

% Figura adicional: Aceleração
figure('Name', 'PSO - Acelerômetro', 'Position', [150, 150, 800, 600]);

subplot(3, 1, 1);
plot(data.time, data.accel_x, 'r-', 'LineWidth', 1.2);
grid on;
ylabel('Accel X');
title('Aceleração nos 3 Eixos');

subplot(3, 1, 2);
plot(data.time, data.accel_y, 'g-', 'LineWidth', 1.2);
grid on;
ylabel('Accel Y');

subplot(3, 1, 3);
plot(data.time, data.accel_z, 'b-', 'LineWidth', 1.2);
grid on;
xlabel('Tempo (s)');
ylabel('Accel Z');

fprintf('Gráficos gerados!\n');

%% Análise estatística
fprintf('\n=== Análise Estatística ===\n');
fprintf('RPM:      min=%-6d  max=%-6d  média=%-7.1f\n', ...
        min(data.rpm), max(data.rpm), mean(data.rpm));
fprintf('Corrente: min=%-6.3f  max=%-6.3f  média=%-7.3f A\n', ...
        min(data.current), max(data.current), mean(data.current));
fprintf('Tensão:   min=%-6.2f  max=%-6.2f  média=%-7.2f V\n', ...
        min(data.voltage), max(data.voltage), mean(data.voltage));
fprintf('Empuxo:   min=%-6.2f  max=%-6.2f  média=%-7.2f N\n', ...
        min(data.thrust), max(data.thrust), mean(data.thrust));
fprintf('Potência: min=%-6.2f  max=%-6.2f  média=%-7.2f W\n', ...
        min(data.power), max(data.power), mean(data.power));

fprintf('\n=== Processo concluído ===\n');