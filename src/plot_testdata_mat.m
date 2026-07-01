% PSO Data Acquisition - plot saved MAT data
% ==================================================================
% Optional usage before running:
%   MAT_FILE = "/path/to/testdata/YYYY-MM-DD/test_name.mat";

clearvars -except MAT_FILE;
close all;
clc;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
repo_root = fileparts(script_dir);
testdata_root = fullfile(repo_root, "testdata");

if ~exist('MAT_FILE', 'var') || strlength(string(MAT_FILE)) == 0
    mat_files = dir(fullfile(testdata_root, "**", "*.mat"));
    if isempty(mat_files)
        error('Nenhum arquivo .mat encontrado em %s', testdata_root);
    end

    [~, newest_idx] = max([mat_files.datenum]);
    MAT_FILE = fullfile(mat_files(newest_idx).folder, mat_files(newest_idx).name);
end

fprintf('Carregando %s\n', MAT_FILE);
loaded = load(MAT_FILE);

if ~isfield(loaded, 'data')
    error('O arquivo selecionado nao contem a variavel "data": %s', MAT_FILE);
end

data = loaded.data;
data = normalize_data_fields(data);

%% Plotar graficos principais
fprintf('\nGerando graficos...\n');

figure('Name', 'PSO Data Acquisition - Analysis', ...
       'Position', [100, 100, 1200, 800]);

subplot(3, 2, 1);
plot(data.time, data.rpm, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('RPM');
title('Rotacao do Motor');

subplot(3, 2, 2);
plot(data.time, data.throttle, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Throttle (%)');
title('Posicao do Acelerador');
ylim([0 105]);

subplot(3, 2, 3);
plot(data.time, data.current, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Corrente (A)');
title('Corrente Eletrica');

subplot(3, 2, 4);
plot(data.time, data.voltage, 'Color', [1 0.5 0], 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Tensao (V)');
title('Tensao de Alimentacao');

subplot(3, 2, 5);
plot(data.time, data.thrust, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Empuxo (g.f)');
title('Forca de Empuxo');

subplot(3, 2, 6);
plot(data.time, data.power, 'c-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('Potencia (W)');
title('Potencia Eletrica');

sgtitle('PSO Data Acquisition - Analise de Dados', 'FontSize', 14, 'FontWeight', 'bold');

%% Figura adicional: acelerometro
if isfield(data, 'accel_x') && isfield(data, 'accel_y') && isfield(data, 'accel_z')
    figure('Name', 'PSO - Acelerometro', 'Position', [150, 150, 800, 600]);

    subplot(3, 1, 1);
    plot(data.time, data.accel_x, 'r-', 'LineWidth', 1.2);
    grid on;
    ylabel('Accel X');
    title('Aceleracao nos 3 Eixos');

    subplot(3, 1, 2);
    plot(data.time, data.accel_y, 'g-', 'LineWidth', 1.2);
    grid on;
    ylabel('Accel Y');

    subplot(3, 1, 3);
    plot(data.time, data.accel_z, 'b-', 'LineWidth', 1.2);
    grid on;
    xlabel('Tempo (s)');
    ylabel('Accel Z');
end

fprintf('Graficos gerados!\n');

%% Analise estatistica
fprintf('\n=== Analise Estatistica ===\n');
fprintf('Arquivo:   %s\n', MAT_FILE);
fprintf('Amostras:  %d\n', numel(data.time));
fprintf('RPM:       min=%-6d  max=%-6d  media=%-7.1f\n', ...
        min(data.rpm), max(data.rpm), mean(data.rpm));
fprintf('Corrente:  min=%-6.3f  max=%-6.3f  media=%-7.3f A\n', ...
        min(data.current), max(data.current), mean(data.current));
fprintf('Tensao:    min=%-6.2f  max=%-6.2f  media=%-7.2f V\n', ...
        min(data.voltage), max(data.voltage), mean(data.voltage));
fprintf('Empuxo:    min=%-6.2f  max=%-6.2f  media=%-7.2f g.f\n', ...
        min(data.thrust), max(data.thrust), mean(data.thrust));
fprintf('Potencia:  min=%-6.2f  max=%-6.2f  media=%-7.2f W\n', ...
        min(data.power), max(data.power), mean(data.power));

if isfield(loaded, 'packet_count')
    fprintf('Pacotes validos: %d\n', loaded.packet_count);
end
if isfield(loaded, 'error_count')
    fprintf('Erros totais:    %d\n', loaded.error_count);
end

function data = normalize_data_fields(data)
    required_fields = ["time", "rpm", "throttle", "current", "voltage", "thrust"];
    for k = 1:numel(required_fields)
        field_name = char(required_fields(k));
        if ~isfield(data, field_name)
            error('Campo ausente em data: %s', field_name);
        end
        data.(field_name) = data.(field_name)(:);
    end

    if ~isfield(data, 'power') || isempty(data.power)
        data.power = data.voltage .* data.current;
    else
        data.power = data.power(:);
    end

    optional_fields = ["index", "accel_x", "accel_y", "accel_z"];
    for k = 1:numel(optional_fields)
        field_name = char(optional_fields(k));
        if isfield(data, field_name)
            data.(field_name) = data.(field_name)(:);
        end
    end
end
