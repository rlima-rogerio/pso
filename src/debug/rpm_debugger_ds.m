% Carregar seus dados
clear all;
close all;
clc;

% load daq_period_us
load daq_period_ticks_75
% load daq_period_us_75

TICKS_MODE = true;
SAMPLING_RATE = 500;

if (TICKS_MODE)
    period_ticks = data.rpm;
    period_us = (period_ticks + 20)/40;
else
    period_us = data.rpm;
end


t = data.index * (1/SAMPLING_RATE);
numSamples = length(t);



BLADE_NUMBER = 2;  % Número de pás da sua hélice

% Processar dados
[t_valid, rpm_valid, rpm_filtered] = process_rpm_data_with_filtering(t, period_us, BLADE_NUMBER);

% Plot comparativo final
figure('Position', [100, 100, 1400, 600]);

% Plot 1: Período original
subplot(1,3,1);
plot(t, period_us, 'b.');
xlabel('Tempo (s)');
ylabel('Período (μs)');
title('Período entre bordas - Original');
grid on;

% Plot 2: RPM calculado
subplot(1,3,2);
plot(t_valid, rpm_valid, 'g-', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('RPM');
title('RPM calculado a partir de períodos válidos');
grid on;

% Plot 3: RPM filtrado
subplot(1,3,3);
plot(t, rpm_filtered, 'r-', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('RPM');
title('RPM filtrado (interpolado e suavizado)');
grid on;