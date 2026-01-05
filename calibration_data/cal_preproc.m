%% Leitura e organização dos arquivos *_reduced.mat
% O script assume que está sendo executado no diretório onde os arquivos estão

clear; clc;

cd('..'); 
cd('calibration_data');

% Lista apenas arquivos *_reduced.mat
files = dir('*_reduced.mat');

% Estrutura de saída
calibrationData.up   = struct('mass_g', {}, 'calibration_mass', {}, 'data_thrust', {}, 'filename', {});
calibrationData.down = struct('mass_g', {}, 'calibration_mass', {}, 'data_thrust', {}, 'filename', {});

% Contadores
idx_up = 0;
idx_down = 0;

for k = 1:length(files)

    filename = files(k).name;

    % ---------------------------------------------------------
    % Parse do nome do arquivo
    % Exemplo: calibration_down_4256g_reduced.mat
    % ---------------------------------------------------------
    tokens = regexp(filename, ...
        'calibration_(up|down)_(\d+)g_reduced\.mat', ...
        'tokens');

    if isempty(tokens)
        warning('Arquivo ignorado (formato inesperado): %s', filename);
        continue;
    end

    type   = tokens{1}{1};          % 'up' ou 'down'
    mass_g = str2double(tokens{1}{2}); % massa em gramas

    % ---------------------------------------------------------
    % Carrega o arquivo .mat
    % ---------------------------------------------------------
    data = load(filename);

    % Verificação mínima das variáveis esperadas
    if ~isfield(data, 'calibration_mass') || ~isfield(data, 'data_thrust')
        warning('Arquivo %s não contém variáveis esperadas.', filename);
        continue;
    end

    % ---------------------------------------------------------
    % Armazenamento organizado
    % ---------------------------------------------------------
    if strcmp(type, 'up')
        idx_up = idx_up + 1;
        calibrationData.up(idx_up).mass_g = mass_g;
        calibrationData.up(idx_up).calibration_mass = data.calibration_mass;
        calibrationData.up(idx_up).data_thrust = data.data_thrust;
        calibrationData.up(idx_up).filename = filename;
    else
        idx_down = idx_down + 1;
        calibrationData.down(idx_down).mass_g = mass_g;
        calibrationData.down(idx_down).calibration_mass = data.calibration_mass;
        calibrationData.down(idx_down).data_thrust = data.data_thrust;
        calibrationData.down(idx_down).filename = filename;
        
        % There is an in the calibration mass for calibration_mass = 0.
        if calibrationData.down(idx_down).calibration_mass < 0
            calibrationData.down(idx_down).calibration_mass = mass_g;
        end
    end
end

%% Organização opcional: ordenar por massa crescente
if ~isempty(calibrationData.up)
    [~, idx] = sort([calibrationData.up.mass_g]);
    calibrationData.up = calibrationData.up(idx);
end

if ~isempty(calibrationData.down)
    [~, idx] = sort([calibrationData.down.mass_g]);
    calibrationData.down = calibrationData.down(idx);
end

%% Resumo
fprintf('Arquivos carregados:\n');
fprintf('  UP   : %d arquivos\n', numel(calibrationData.up));
fprintf('  DOWN : %d arquivos\n', numel(calibrationData.down));

%% (Opcional) salvar estrutura consolidada
save('calibrationData_all.mat', 'calibrationData');
