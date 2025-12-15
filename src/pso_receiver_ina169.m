%% ========================================================================
%% PSO TELEMETRY RECEIVER - MATLAB (INA169 VERSION)
%% ========================================================================
%% 
%% HARDWARE:
%%   - Voltage: Divisor resistivo (R3=1.5kΩ, R4=13.7kΩ)
%%   - Current: INA169 + Rshunt=0.5mΩ + RL=110kΩ
%%
%% CONVERSÃO:
%%   - Tensão:   V = v_motor / 1000  (mV → V)
%%   - Corrente: A = i_motor / 1000  (mA → A)
%%
%% FAIXAS:
%%   - Voltage: 0-33400 mV (0-33.4V)
%%   - Current: 0-60000 mA (0-60A)
%%
%% ========================================================================

clear; clc; close all;

%% Configuration
SERIAL_PORT = '/dev/ttyACM0';  % Change to 'COM3' on Windows
BAUDRATE = 115200;

%% Hardware Parameters (for reference)
% Voltage Divider
R3 = 1500;      % Ω
R4 = 13700;     % Ω
Vmax = 33.4;    % V

% Current Monitor (INA169)
Rshunt = 0.5e-3;    % 0.5 mΩ
RL = 110e3;         % 110 kΩ
gm = 1e-3;          % 1000 μA/V = 0.001 A/V
Vout_per_amp = Rshunt * gm * RL;  % 0.055 V/A
Imax = 3.3 / Vout_per_amp;        % 60 A

fprintf('PSO Telemetry Receiver (INA169 Configuration)\n');
fprintf('==============================================\n');
fprintf('Voltage range: 0-%.1f V\n', Vmax);
fprintf('Current range: 0-%.0f A\n', Imax);
fprintf('Transfer function: Vout = Is × %.3f [V/A]\n\n', Vout_per_amp);

%% Open Serial Port
fprintf('Opening serial port: %s @ %d baud...\n', SERIAL_PORT, BAUDRATE);

try
    s = serialport(SERIAL_PORT, BAUDRATE);
    configureTerminator(s, "LF");
    fprintf('Serial port opened successfully!\n\n');
catch ME
    fprintf('Error opening serial port: %s\n', ME.message);
    return;
end

%% Setup Real-time Plotting
fig = figure('Name', 'PSO Real-time Telemetry (INA169)', 'NumberTitle', 'off');
set(fig, 'Position', [100, 100, 1200, 800]);

% Subplots
ax1 = subplot(2, 3, 1); title('Voltage (V)'); ylabel('V'); xlabel('Sample'); grid on; hold on;
ax2 = subplot(2, 3, 2); title('Current (A)'); ylabel('A'); xlabel('Sample'); grid on; hold on;
ax3 = subplot(2, 3, 3); title('Power (kW)'); ylabel('kW'); xlabel('Sample'); grid on; hold on;
ax4 = subplot(2, 3, 4); title('RPM'); ylabel('RPM'); xlabel('Sample'); grid on; hold on;
ax5 = subplot(2, 3, 5); title('Throttle (%)'); ylabel('%'); xlabel('Sample'); grid on; hold on;
ax6 = subplot(2, 3, 6); title('Efficiency (W/RPM)'); ylabel('W/RPM'); xlabel('Sample'); grid on; hold on;

%% Data Buffers
MAX_SAMPLES = 1000;
v_buffer = zeros(1, MAX_SAMPLES);
i_buffer = zeros(1, MAX_SAMPLES);
p_buffer = zeros(1, MAX_SAMPLES);
rpm_buffer = zeros(1, MAX_SAMPLES);
throttle_buffer = zeros(1, MAX_SAMPLES);
efficiency_buffer = zeros(1, MAX_SAMPLES);
sample_count = 0;
start_time = tic;

%% Main Loop
fprintf('Starting data acquisition...\n');
fprintf('Press Ctrl+C to stop.\n\n');
fprintf('%-10s | %-10s | %-10s | %-10s | %-10s\n', ...
        'Voltage', 'Current', 'Power', 'RPM', 'Throttle');
fprintf('%-10s | %-10s | %-10s | %-10s | %-10s\n', ...
        '(V)', '(A)', '(kW)', '', '(%)');
fprintf('-----------|------------|------------|------------|------------\n');

try
    while true
        if s.NumBytesAvailable >= 16
            % Read packet (7 × uint16 + 1 × uint8)
            data = read(s, 8, 'uint16');
            
            % Extract fields
            accel_x  = data(1);
            accel_y  = data(2);
            accel_z  = data(3);
            rpm      = data(4);
            i_motor_ma = data(5);  % Current in mA
            v_motor_mv = data(6);  % Voltage in mV
            thrust   = data(7);
            throttle = bitand(data(8), 255);
            
            % =====================================================
            % CONVERSÃO DIRETA (INA169)
            % =====================================================
            
            % Voltage: mV → V
            v_motor_V = double(v_motor_mv) / 1000;
            
            % Current: mA → A
            i_motor_A = double(i_motor_ma) / 1000;
            
            % =====================================================
            
            % Calculate power
            power_W = v_motor_V * i_motor_A;
            power_kW = power_W / 1000;
            
            % Calculate efficiency (W/RPM)
            if rpm > 0
                efficiency = power_W / double(rpm);
            else
                efficiency = 0;
            end
            
            % Display
            fprintf('%10.3f | %10.2f | %10.2f | %10d | %10d\n', ...
                    v_motor_V, i_motor_A, power_kW, rpm, throttle);
            
            % Update buffers
            sample_count = sample_count + 1;
            idx = mod(sample_count - 1, MAX_SAMPLES) + 1;
            
            v_buffer(idx) = v_motor_V;
            i_buffer(idx) = i_motor_A;
            p_buffer(idx) = power_kW;
            rpm_buffer(idx) = double(rpm);
            throttle_buffer(idx) = double(throttle);
            efficiency_buffer(idx) = efficiency;
            
            % Update plots every 10 samples
            if mod(sample_count, 10) == 0
                samples_to_plot = min(sample_count, MAX_SAMPLES);
                x_axis = 1:samples_to_plot;
                
                plot(ax1, x_axis, v_buffer(1:samples_to_plot), 'b-', 'LineWidth', 1.5);
                xlim(ax1, [max(1, samples_to_plot-100), samples_to_plot]);
                
                plot(ax2, x_axis, i_buffer(1:samples_to_plot), 'r-', 'LineWidth', 1.5);
                xlim(ax2, [max(1, samples_to_plot-100), samples_to_plot]);
                
                plot(ax3, x_axis, p_buffer(1:samples_to_plot), 'g-', 'LineWidth', 1.5);
                xlim(ax3, [max(1, samples_to_plot-100), samples_to_plot]);
                
                plot(ax4, x_axis, rpm_buffer(1:samples_to_plot), 'k-', 'LineWidth', 1.5);
                xlim(ax4, [max(1, samples_to_plot-100), samples_to_plot]);
                
                plot(ax5, x_axis, throttle_buffer(1:samples_to_plot), 'm-', 'LineWidth', 1.5);
                xlim(ax5, [max(1, samples_to_plot-100), samples_to_plot]);
                ylim(ax5, [0, 100]);
                
                plot(ax6, x_axis, efficiency_buffer(1:samples_to_plot), 'c-', 'LineWidth', 1.5);
                xlim(ax6, [max(1, samples_to_plot-100), samples_to_plot]);
                
                drawnow;
            end
        end
        
        pause(0.001);
    end
    
catch ME
    if strcmp(ME.identifier, 'MATLAB:interrupt')
        fprintf('\n\nData acquisition stopped by user.\n');
    else
        fprintf('\n\nError: %s\n', ME.message);
    end
end

%% Cleanup
fprintf('Closing serial port...\n');
clear s;
fprintf('Done.\n');

%% Statistics
if sample_count > 0
    fprintf('\n=== Statistics ===\n');
    fprintf('Samples collected: %d\n', sample_count);
    fprintf('Average voltage: %.3f V\n', mean(v_buffer(1:min(sample_count, MAX_SAMPLES))));
    fprintf('Average current: %.2f A\n', mean(i_buffer(1:min(sample_count, MAX_SAMPLES))));
    fprintf('Average power: %.2f kW\n', mean(p_buffer(1:min(sample_count, MAX_SAMPLES))));
    fprintf('Max current: %.2f A\n', max(i_buffer(1:min(sample_count, MAX_SAMPLES))));
end

%% Save Data (Optional)
save_data = questdlg('Save acquired data?', 'Save Data', 'Yes', 'No', 'No');
if strcmp(save_data, 'Yes')
    samples_acquired = min(sample_count, MAX_SAMPLES);
    
    pso_data = struct();
    pso_data.voltage_V = v_buffer(1:samples_acquired);
    pso_data.current_A = i_buffer(1:samples_acquired);
    pso_data.power_kW = p_buffer(1:samples_acquired);
    pso_data.rpm = rpm_buffer(1:samples_acquired);
    pso_data.throttle = throttle_buffer(1:samples_acquired);
    pso_data.efficiency = efficiency_buffer(1:samples_acquired);
    pso_data.timestamp = datetime('now');
    pso_data.hardware = 'INA169 + 0.5mΩ shunt + 110kΩ RL';
    pso_data.config.Rshunt = Rshunt;
    pso_data.config.RL = RL;
    pso_data.config.gm = gm;
    pso_data.config.Imax = Imax;
    
    filename = sprintf('pso_data_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
    save(filename, 'pso_data');
    fprintf('Data saved to: %s\n', filename);
end
