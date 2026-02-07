% INA169 Current Sensor Calibration Analysis
% ============================================
% Hardware modification: Two 110kΩ resistors in parallel (RL = 55kΩ)
% This script performs linear regression on measured data to determine
% the actual conversion factor for ADC to current.

clear; clc;

%% Measured Data
% Current (A) | ADC Voltage (mV)
data = [
    57.2e-3,    3.55;
    0.33,      16.5;
    0.51,      25.3;
    1.02,      50.2;
    2.08,     101;
    3.08,     151;
    4.20,     203;
    5.00,     244;
    7.36,     358;
    8.12,     397;
    9.75,     475;
];

current_A = data(:, 1);
voltage_mV = data(:, 2);

%% Convert ADC voltage to ADC counts
% ADC: 12-bit, Vref = 3300 mV
ADC_VREF_MV = 3300;
ADC_MAX_VALUE = 4095;

adc_counts = round((voltage_mV / ADC_VREF_MV) * ADC_MAX_VALUE);

%% Linear Regression: Current (mA) vs ADC counts
current_mA = current_A * 1000;

% Fit: I(mA) = slope * ADC + offset
[p, S] = polyfit(adc_counts, current_mA, 1);
slope = p(1);      % mA per ADC count
offset = p(2);     % mA offset
r_squared = 1 - (S.normr/norm(current_mA - mean(current_mA)))^2;

% Predicted values
current_mA_fit = polyval(p, adc_counts);
residuals = current_mA - current_mA_fit;
max_error_percent = max(abs(residuals ./ current_mA)) * 100;

%% Calculate conversion factor for integer implementation
% Formula: I(mA) = (ADC × IMAX_MA) / 4095
% Solve for IMAX_MA when ADC = 4095:
IMAX_MA_calculated = slope * ADC_MAX_VALUE + offset;

% For cleaner integer math, we use the slope through origin
% (offset is negligible: ~0.3 mA)
IMAX_MA_clean = round(slope * ADC_MAX_VALUE);

%% Theoretical Analysis (with RL = 55kΩ parallel)
% INA169 transfer function: Vout = (Is × Rs) × gm × RL
Rs_ohm = 0.0005;           % Shunt: 0.5 mΩ
gm_AperV = 0.001;          % Transconductance: 1000 μA/V
RL_ohm = 55000;            % Load resistor: 55 kΩ (parallel combination)

% Sensitivity: mV per A
sensitivity_mV_per_A = Rs_ohm * gm_AperV * RL_ohm * 1000;

% Theoretical slope: mA per ADC count
theoretical_slope = (ADC_VREF_MV / ADC_MAX_VALUE) / sensitivity_mV_per_A * 1000;
theoretical_IMAX_MA = theoretical_slope * ADC_MAX_VALUE;

%% Display Results
fprintf('========================================\n');
fprintf('INA169 CALIBRATION RESULTS\n');
fprintf('========================================\n\n');

fprintf('HARDWARE CONFIGURATION:\n');
fprintf('  Shunt resistance (Rs):    %.1f mΩ\n', Rs_ohm * 1000);
fprintf('  Load resistance (RL):     %.1f kΩ (2× 110kΩ parallel)\n', RL_ohm / 1000);
fprintf('  Transconductance (gm):    %.0f μA/V\n', gm_AperV * 1e6);
fprintf('  Sensitivity:              %.3f mV/A\n', sensitivity_mV_per_A);
fprintf('\n');

fprintf('LINEAR REGRESSION (polyfit):\n');
fprintf('  Equation:                 I(mA) = %.4f × ADC + %.4f\n', slope, offset);
fprintf('  R²:                       %.6f\n', r_squared);
fprintf('  Max error:                %.2f%%\n', max_error_percent);
fprintf('  Slope (mA/count):         %.4f\n', slope);
fprintf('  Offset (mA):              %.4f\n', offset);
fprintf('\n');

fprintf('CONVERSION FACTOR (for firmware):\n');
fprintf('  IMAX_MA (calculated):     %.0f mA (%.1f A)\n', IMAX_MA_calculated, IMAX_MA_calculated/1000);
fprintf('  IMAX_MA (rounded):        %.0f mA (%.1f A)\n', IMAX_MA_clean, IMAX_MA_clean/1000);
fprintf('  Formula:                  I(mA) = (ADC × %d) / 4095\n', IMAX_MA_clean);
fprintf('\n');

fprintf('THEORETICAL vs MEASURED:\n');
fprintf('  Theoretical slope:        %.4f mA/count\n', theoretical_slope);
fprintf('  Measured slope:           %.4f mA/count\n', slope);
fprintf('  Difference:               %.2f%%\n', abs(theoretical_slope - slope)/theoretical_slope * 100);
fprintf('  Theoretical IMAX_MA:      %.0f mA\n', theoretical_IMAX_MA);
fprintf('  Measured IMAX_MA:         %.0f mA\n', IMAX_MA_clean);
fprintf('\n');

fprintf('RECOMMENDED FIRMWARE UPDATE:\n');
fprintf('  #define IMAX_MA           %dUL      /* %.1f A max */\n', IMAX_MA_clean, IMAX_MA_clean/1000);
fprintf('  #define RL_OHM            %dUL      /* 55 kΩ (parallel) */\n', RL_ohm);
fprintf('\n');

%% Plotting
figure('Position', [100, 100, 1200, 800]);

% Subplot 1: Current vs ADC Voltage
subplot(2, 2, 1);
plot(voltage_mV, current_A, 'bo', 'MarkerSize', 8, 'LineWidth', 1.5); hold on;
voltage_fit = linspace(0, max(voltage_mV)*1.1, 100);
current_fit = polyval([slope/1000, offset/1000], (voltage_fit/ADC_VREF_MV)*ADC_MAX_VALUE);
plot(voltage_fit, current_fit, 'r-', 'LineWidth', 2);
grid on;
xlabel('ADC Voltage (mV)');
ylabel('Current (A)');
title('Measured Current vs ADC Voltage');
legend('Measured', 'Linear Fit', 'Location', 'northwest');

% Subplot 2: Current vs ADC Counts
subplot(2, 2, 2);
plot(adc_counts, current_mA, 'bo', 'MarkerSize', 8, 'LineWidth', 1.5); hold on;
adc_fit = linspace(0, max(adc_counts)*1.1, 100);
current_fit_mA = polyval(p, adc_fit);
plot(adc_fit, current_fit_mA, 'r-', 'LineWidth', 2);
grid on;
xlabel('ADC Counts (0-4095)');
ylabel('Current (mA)');
title(sprintf('Current vs ADC Counts (R² = %.6f)', r_squared));
legend('Measured', sprintf('I = %.4f×ADC + %.2f', slope, offset), 'Location', 'northwest');

% Subplot 3: Residuals
subplot(2, 2, 3);
stem(adc_counts, residuals, 'filled', 'LineWidth', 1.5);
grid on;
xlabel('ADC Counts');
ylabel('Residual (mA)');
title('Fit Residuals');
yline(0, 'r--', 'LineWidth', 1.5);

% Subplot 4: Percent Error
subplot(2, 2, 4);
percent_error = (residuals ./ current_mA) * 100;
stem(adc_counts, percent_error, 'filled', 'LineWidth', 1.5);
grid on;
xlabel('ADC Counts');
ylabel('Error (%)');
title(sprintf('Percentage Error (max = %.2f%%)', max_error_percent));
yline(0, 'r--', 'LineWidth', 1.5);

sgtitle('INA169 Current Sensor Calibration Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% Save calibration data
calibration.data = data;
calibration.adc_counts = adc_counts;
calibration.current_mA = current_mA;
calibration.regression.slope = slope;
calibration.regression.offset = offset;
calibration.regression.r_squared = r_squared;
calibration.firmware.IMAX_MA = IMAX_MA_clean;
calibration.firmware.RL_OHM = RL_ohm;
calibration.theoretical.slope = theoretical_slope;
calibration.theoretical.IMAX_MA = theoretical_IMAX_MA;

save('ina169_calibration.mat', 'calibration');
fprintf('Calibration data saved to: ina169_calibration.mat\n');

%% Generate lookup table (optional)
fprintf('\nLOOKUP TABLE (sample points):\n');
fprintf('ADC   | Current (mA) | Current (A)\n');
fprintf('------|--------------|------------\n');
test_adc = [0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4095];
for adc = test_adc
    curr_mA = (adc * IMAX_MA_clean) / ADC_MAX_VALUE;
    fprintf('%4d  | %8.1f     | %7.3f\n', adc, curr_mA, curr_mA/1000);
end

fprintf('\n========================================\n');
fprintf('Analysis complete!\n');
fprintf('========================================\n');