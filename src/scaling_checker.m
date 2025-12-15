% ========================================================================
% PSO Scaling Functions - MATLAB Version (INA169)
% ========================================================================
% 
% HARDWARE:
%   - Voltage Divider: R3=1.5kΩ, R4=13.7kΩ
%   - Current Monitor: INA169 + Rshunt=0.5mΩ + RL=110kΩ
%
% CONVERSÃO:
%   - Tensão:   V = v_motor / 1000  (mV → V)
%   - Corrente: A = i_motor / 1000  (mA → A)
%
% FAIXAS:
%   - Voltage: 0-33400 mV (0-33.4V)
%   - Current: 0-60000 mA (0-60A)
%
% ========================================================================

clear; clc; close all;

load daq_analyze.mat;

%% Hardware Constants
% Voltage Divider
R3_OHM = 1500;              % Upper resistor: 1.5 kΩ
R4_OHM = 13700;             % Lower resistor: 13.7 kΩ
VBAT_MAX_MV = 33400;        % Max battery voltage: 33.4V (em mV)

% Current Monitor - INA169 Configuration
RSHUNT_MOHM = 0.5;          % Shunt resistance: 0.5 mΩ
RL_OHM = 110000;            % Load resistor: 110 kΩ
INA169_GM = 0.001;          % Transconductance: 1000 μA/V

% ADC Configuration
ADC_VREF_MV = 3300;         % ADC reference: 3.3V
ADC_MAX_VALUE = 4095;       % 12-bit ADC: 2^12 - 1
ADC_RESOLUTION = 4096;      % 12-bit: 2^12

% uint16_t limits
UINT16_MAX_VAL = 65535;     % Maximum uint16_t value

% Derived Constants
IMAX_MA = 60000;            % Maximum current: 60 A = 60000 mA


% Display Hardware Configuration
fprintf('========================================\n');
fprintf('PSO Scaling Configuration (INA169)\n');
fprintf('========================================\n');
fprintf('Voltage Divider:\n');
fprintf('  R3 = %d Ω\n', R3_OHM);
fprintf('  R4 = %d Ω\n', R4_OHM);
fprintf('  Vmax = %.1f V\n', VBAT_MAX_MV/1000);
fprintf('  V_div = %.5f\n', R3_OHM/(R3_OHM + R4_OHM));
fprintf('\n');
fprintf('Current Monitor (INA169):\n');
fprintf('  Rshunt = %.1f mΩ\n', RSHUNT_MOHM);
fprintf('  RL = %.0f kΩ\n', RL_OHM/1000);
fprintf('  gm = %.3f A/V\n', INA169_GM);
fprintf('  Vout/Is = %.3f V/A\n', RSHUNT_MOHM*1e-3 * INA169_GM * RL_OHM);
fprintf('  Imax = %.0f A\n', IMAX_MA/1000);
fprintf('\n');
fprintf('ADC Configuration:\n');
fprintf('  Vref = %.1f V\n', ADC_VREF_MV/1000);
fprintf('  Resolution = %d bits\n', log2(ADC_RESOLUTION));
fprintf('  Max value = %d\n', ADC_MAX_VALUE);
fprintf('========================================\n\n');


% Initialize time vector and measurement arrays
time = data.index;
adc_voltage_raw = data.voltage * 1000;
adc_current_raw = data.current * 1000;

% Scaled variables
voltage = voltage_adc_to_mv(adc_voltage_raw) / 1000;
current = current_adc_to_ma(adc_current_raw) / 1000;

figure,
% Set up the axes for voltage and current measurements
subplot(2, 1, 1);
plot(time, voltage);
title('Voltage Measurement');
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;
% Set up the axes for current measurements
subplot(2, 1, 2);
plot(time, current);
title('Current Measurement');
xlabel('Time (s)');
ylabel('Current (A)');
grid on;







% ========================================================================
% FUNCTION: voltage_adc_to_mv
% ========================================================================
% Convert ADC value to voltage in millivolts
%
% Inputs:
%   adc_value - Raw ADC reading (0-4095)
%
% Outputs:
%   voltage_mv - Voltage in millivolts (0-33400)
%
% Formula: V(mV) = (ADC × 33400) / 4095
% ========================================================================

function voltage_mv = voltage_adc_to_mv(adc_value)
    % Constants (defined locally for function portability)
    VBAT_MAX_MV = 33400;
    ADC_MAX_VALUE = 4095;
    
    % Clamp ADC value to maximum
    if adc_value > ADC_MAX_VALUE
        adc_value = ADC_MAX_VALUE;
    end
    
    % Calculate voltage in millivolts
    % Formula: V(mV) = (ADC × 33400) / 4095
    voltage_mv = floor((adc_value * VBAT_MAX_MV) / ADC_MAX_VALUE);
    
    % Ensure result fits in uint16 range
    % voltage_mv = uint16(voltage_mv);
end

% ========================================================================
% FUNCTION: current_adc_to_ma
% ========================================================================
% Convert ADC value to current in milliamperes (INA169)
%
% Inputs:
%   adc_value - Raw ADC reading (0-4095)
%
% Outputs:
%   current_ma - Current in milliamperes (0-60000)
%
% Formula: I(mA) = (ADC × 60000) / 4095
%
% INA169 Transfer Function:
%   Vout = (Is × Rs) × gm × RL
%   Vout = Is × 0.055 [V/A]
%   Is = Vout / 0.055
%   Is(mA) = (ADC × 60000) / 4095
% ========================================================================

function current_ma = current_adc_to_ma(adc_value)
    % Constants (defined locally for function portability)
    IMAX_MA = 60000;
    ADC_MAX_VALUE = 4095;
    
    % Clamp ADC value to maximum
    if adc_value > ADC_MAX_VALUE
        adc_value = ADC_MAX_VALUE;
    end
    
    % Calculate current in milliamperes
    % Formula: I(mA) = (ADC × 60000) / 4095
    current_ma = floor((adc_value * IMAX_MA) / ADC_MAX_VALUE);
    
    % Ensure result fits in uint16 range
    % current_ma = uint16(current_ma);
end

% ========================================================================
% FUNCTION: voltage_mv_to_adc (Reverse conversion)
% ========================================================================
% Convert voltage in millivolts back to expected ADC value
%
% Inputs:
%   voltage_mv - Voltage in millivolts (0-33400)
%
% Outputs:
%   adc_value - Expected ADC reading (0-4095)
% ========================================================================

function adc_value = voltage_mv_to_adc(voltage_mv)
    VBAT_MAX_MV = 33400;
    ADC_MAX_VALUE = 4095;
    
    % Clamp voltage to maximum
    if voltage_mv > VBAT_MAX_MV
        voltage_mv = VBAT_MAX_MV;
    end
    
    % Calculate ADC value
    adc_value = floor((voltage_mv * ADC_MAX_VALUE) / VBAT_MAX_MV);
    adc_value = uint16(adc_value);
end

% ========================================================================
% FUNCTION: current_ma_to_adc (Reverse conversion)
% ========================================================================
% Convert current in milliamperes back to expected ADC value
%
% Inputs:
%   current_ma - Current in milliamperes (0-60000)
%
% Outputs:
%   adc_value - Expected ADC reading (0-4095)
% ========================================================================

function adc_value = current_ma_to_adc(current_ma)
    IMAX_MA = 60000;
    ADC_MAX_VALUE = 4095;
    
    % Clamp current to maximum
    if current_ma > IMAX_MA
        current_ma = IMAX_MA;
    end
    
    % Calculate ADC value
    adc_value = floor((current_ma * ADC_MAX_VALUE) / IMAX_MA);
    adc_value = uint16(adc_value);
end

% ========================================================================
% VALIDATION FUNCTIONS
% ========================================================================

function valid = voltage_mv_is_valid(voltage_mv)
    VBAT_MAX_MV = 33400;
    valid = (voltage_mv <= VBAT_MAX_MV);
end

function valid = current_ma_is_valid(current_ma)
    IMAX_MA = 60000;
    valid = (current_ma <= IMAX_MA);
end

% ========================================================================
% TEST/VALIDATION CODE
% ========================================================================

fprintf('Testing Conversion Functions...\n');
fprintf('================================\n\n');

% Test values
test_adc_values = [0, 682, 1024, 2048, 3072, 4095];

fprintf('ADC  | Voltage (mV) |   V    | Current (mA) |   A   \n');
fprintf('-----|--------------|--------|--------------|-------\n');

for i = 1:length(test_adc_values)
    adc = test_adc_values(i);
    v_mv = voltage_adc_to_mv(adc);
    i_ma = current_adc_to_ma(adc);
    
    fprintf('%4d | %10d mV | %6.3f | %10d mA | %5.2f\n', ...
            adc, v_mv, double(v_mv)/1000, i_ma, double(i_ma)/1000);
end

fprintf('\n');

%% Reverse conversion test
fprintf('Testing Reverse Conversions...\n');
fprintf('===============================\n\n');

% Test voltage reverse conversion
v_test_mv = 20000;  % 20V
adc_calculated = voltage_mv_to_adc(v_test_mv);
v_reconstructed = voltage_adc_to_mv(adc_calculated);
fprintf('Voltage test: %d mV → ADC %d → %d mV\n', ...
        v_test_mv, adc_calculated, v_reconstructed);

% Test current reverse conversion
i_test_ma = 30000;  % 30A
adc_calculated = current_ma_to_adc(i_test_ma);
i_reconstructed = current_adc_to_ma(adc_calculated);
fprintf('Current test: %d mA → ADC %d → %d mA\n', ...
        i_test_ma, adc_calculated, i_reconstructed);

fprintf('\n');

%% Validation test
fprintf('Testing INA169 Transfer Function...\n');
fprintf('====================================\n\n');

% Hardware parameters
Rs = 0.0005;     % 0.5 mΩ in Ω
gm = 0.001;      % 1000 μA/V in A/V
RL = 110000;     % 110 kΩ
Vref = 3.3;      % V

% Transfer function
Vout_per_amp = Rs * gm * RL;
fprintf('Vout/Is = %.3f V/A\n', Vout_per_amp);

Imax_calculated = Vref / Vout_per_amp;
fprintf('Imax = %.2f A\n\n', Imax_calculated);

% Test with real current values
test_currents = [10, 30, 60];  % A

fprintf('Current | Vout Expected | ADC Expected | Measured (mA) | Error\n');
fprintf('--------|---------------|--------------|---------------|------\n');

for i = 1:length(test_currents)
    I_real = test_currents(i);
    Vout_expected = I_real * Vout_per_amp;
    adc_expected = floor((Vout_expected / Vref) * 4095);
    i_ma_measured = current_adc_to_ma(adc_expected);
    error_pct = ((double(i_ma_measured)/1000 - I_real) / I_real) * 100;
    
    fprintf('%5.0f A | %11.4f V | %10d   | %11d mA | %5.2f%%\n', ...
            I_real, Vout_expected, adc_expected, i_ma_measured, error_pct);
end

fprintf('\n✓ All tests completed!\n');