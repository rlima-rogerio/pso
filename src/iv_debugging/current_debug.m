% Dados medidos
I = [ ...
    0.0572
    0.33
    0.51
    1.02
    2.08
    3.08
    4.20
    5.00
    7.36
    8.12
    9.75 ];

V = [ ...
    3.55
    16.5
    25.3
    50.2
    101
    151
    203
    244
    358
    397
    475 ] * 1e-3;   % mV -> V

% Regressão linear V = a*I + b
p = polyfit(I, V, 1);

gain_V_per_A = p(1);
offset_V     = p(2);

fprintf('Ganho:  %.4f V/A (%.2f mV/A)\n', gain_V_per_A, gain_V_per_A*1e3);
fprintf('Offset: %.4f V (%.2f mV)\n', offset_V, offset_V*1e3);

% Plot
I_fit = linspace(0, max(I), 100);
V_fit = polyval(p, I_fit);

figure;
plot(I, V, 'o', 'LineWidth', 1.5); hold on;
plot(I_fit, V_fit, '-', 'LineWidth', 2);
grid on;
xlabel('Current [A]');
ylabel('V_{out} [V]');
title('INA169 Output Voltage vs Current');
legend('Measured', 'Linear fit', 'Location', 'NorthWest');
