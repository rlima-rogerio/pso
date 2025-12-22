%% Calibration

clear all
close all
clc


load calibrationData_all.mat


% plot(calibrationData.up(1).data_thrust)
% yline(mean(calibrationData.up(1).data_thrust),'r')
% yline(median(calibrationData.up(1).data_thrust),'k')

% Number of calibration points per type
upSamples = length(calibrationData.up);
downSamples = length(calibrationData.down);

% Calibration data during ramp up
up_adc_thrust_proc = zeros(1, upSamples);
up_mass_g = zeros(1, upSamples);
for upIdx = 1 : upSamples
    up_adc_thrust_proc(upIdx) = mean(calibrationData.up(upIdx).data_thrust);
    up_mass_g(upIdx) = calibrationData.up(upIdx).calibration_mass;
end

% Calibration data during ramp down
down_adc_thrust_proc = zeros(1, downSamples);
down_mass_g = zeros(1, downSamples);
for downIdx = 1: downSamples
    down_adc_thrust_proc(downIdx) = mean(calibrationData.down(downIdx).data_thrust);
    down_mass_g(downIdx) = calibrationData.down(downIdx).calibration_mass;

end

% First order regression
P = polyfit([up_adc_thrust_proc down_adc_thrust_proc], [up_mass_g down_mass_g], 1);

% Check regression
adc_polyfit = linspace(min([up_adc_thrust_proc, down_adc_thrust_proc]),...
                       max([up_adc_thrust_proc, down_adc_thrust_proc]),...
                       1000);

thrust_polyfit = P(1)*adc_polyfit + P(2);

%%
fprintf(' -------------Calibration Coefficients -------------\r\n')
fprintf(' y = ax + b \r\n')
fprintf(' a = P(1) = %1.3f \r\n', P(1));
fprintf(' b = P(2) = %1.3f \r\n', P(2));


%% Plots
figure,
plot(up_adc_thrust_proc, up_mass_g, 'b*'); hold on;
plot(down_adc_thrust_proc, down_mass_g, 'r*');
plot(adc_polyfit, thrust_polyfit, 'k');
ylabel('Mass (g)'); xlabel('ADC Count'); title('Calibration curve');
legend('Up','Down', 'Calibration'); grid on;