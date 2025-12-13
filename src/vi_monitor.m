%% Voltage and Current Sampling and Scaling


%% Voltage Monitor
R3 = 1500;
R4 = 13700;
Vmax = 33.4; % Maximum battery voltage

V_div = R3/(R3 + R4)


Vadc_max = V_div * Vmax

%% Current Monitor
Rshunt = 0.5e-3;

Vadc_max = 3.3;
Imax = Vadc_max/Rshunt; % Maximum current 6600 A

current_meas_v = 30;
I = current_meas_v/Rshunt;