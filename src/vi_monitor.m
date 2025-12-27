%% Voltage and Current Sampling and Scaling
close all
clear all
clc

%% Voltage Monitor
R3 = 1500;
R4 = 13700;
Vmax = 33.4; % Maximum battery voltage

V_div = R3/(R3 + R4)


Vadc_max = V_div * Vmax

%% Current Monitor
Rshunt = 0.5e-3;
RL = 110e3;
IV_ratio = 1e-3; % 1000 uA/V = 1 mA/V = 0.001 A/V
Vadc_max = 3.3;
Vout_max = Vadc_max;
adc_range = 2^12;

Is_max = Vout_max / (Rshunt * IV_ratio * RL)

% Calculate the maximum output voltage based on the maximum current
Vout_max = Is_max * Rshunt * IV_ratio * RL;

Is = 10; %5 Example
Vout = Is * Rshunt *IV_ratio * RL


Imax = Vadc_max/Rshunt; % Maximum current 6600 A

current_meas_v = 30;
I = current_meas_v/Rshunt;

% Test
load raw_adc_current
current_adc = data.current * 1000;

Imeas = (current_adc/adc_range) * Vout_max / (Rshunt * IV_ratio * RL);
plot(data.time, current_adc); grid on; 
ylabel('Current (A)'); xlabel('Time (ms)');

