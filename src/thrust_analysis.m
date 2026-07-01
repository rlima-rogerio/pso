%% Momentum Theory applied to PSO thrust data
% This script reads a collected .mat file and estimates the slipstream
% produced by the motor-propeller set using static actuator-disk momentum
% theory:
%
%   T = 2*rho*A*vi^2
%   P_induced_ideal = T*vi
%   V_wake_far = 2*vi
%
% where T is measured thrust, A is propeller disk area, rho is air density,
% and vi is the induced velocity at the propeller disk.

clearvars -except MAT_FILE
close all
clc

%% User parameters
% Optional usage before running:
%   MAT_FILE = "../testdata/2026-04-11/test_name.mat";

% MAT_FILE = "../testdata/2026-02-14/test_3S_Flier-80A_GT2826-04_Carbon-12x6.mat";
% MAT_FILE = "../testdata/2026-02-14/test_3S_Flier-80A_GT2826-04_Nylon-10x5E.mat";
% MAT_FILE = "../testdata/2026-02-14/test_3S_Flier-80A_GT2826-04_Nylon-11x5.5.mat";
% MAT_FILE = "../testdata/2026-02-14/test_3S_Flier-80A_Propdrive-42-48_Carbon-12x6.5-Aeronaut-CAM.mat";
% MAT_FILE = "../testdata/2026-02-14/test_4S_Flier-80A_GT2826-04_Nylon-10x5E.mat";
% MAT_FILE = "../testdata/2026-02-14/test_4S_Flier-80A_GT2826-04_Nylon-11x5.5.mat";
% MAT_FILE = "../testdata/2026-02-14/test_4S_Flier-80A_Propdrive-42-48_Carbon-12x6.5-Aeronaut-CAM.mat";
% MAT_FILE = "../testdata/2026-02-14/test_4S_Flier-80A_Propdrive-42-48_Carbon-13x6.5-Aeronaut-CAM.mat";
% MAT_FILE = "../testdata/2026-02-14/test_4S_Flier-80A_Propdrive-42-48_Carbon-15x10-Carbon.mat";
% MAT_FILE = "../testdata/2026-04-11/test_4S_Flier-80A_Propdrive-42-48_Glass-Fiber-Nylon GEMFAN-15x8.mat";
MAT_FILE = "../testdata/2026-04-11/test_4S_Flier-80A_Propdrive-42-48_Glass-Fiber Nylon GEMFAN-15x8.mat";


rho = 1.225;                      % Air density at sea level, 15 degC [kg/m^3]
distanceProp2Elevons = 0.70;      % Distance from propeller plane to elevons [m]
min_rpm = 100;                    % Ignore stopped/near-stopped samples
min_power_w = 1.0;                % Ignore samples with negligible electrical power
min_thrust_n = 0.02;              % Ignore sensor noise around zero thrust
thrust_unit = "gf";               % Collected data.thrust is stored as grams-force

% Fallback propeller size used only when the .mat metadata/file name does not
% contain a size such as "15x8".
fallback_propeller_size_in = "15x8";

%% Load collected data
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
repo_root = fileparts(script_dir);
testdata_root = fullfile(repo_root, "testdata");

if ~exist('MAT_FILE', 'var') || strlength(string(MAT_FILE)) == 0
    mat_files = dir(fullfile(testdata_root, "**", "*.mat"));
    if isempty(mat_files)
        error('No .mat files found in %s', testdata_root);
    end

    [~, newest_idx] = max([mat_files.datenum]);
    MAT_FILE = fullfile(mat_files(newest_idx).folder, mat_files(newest_idx).name);
end

fprintf('Loading %s\n', MAT_FILE);
loaded = load(MAT_FILE);
if ~isfield(loaded, 'data')
    error('Selected file does not contain a "data" struct: %s', MAT_FILE);
end

data = normalize_data_fields(loaded.data);
[diameter_in, pitch_in, prop_size_source] = get_propeller_size(loaded, MAT_FILE, fallback_propeller_size_in);

propeller.diameter_m = inch2meter(diameter_in);
propeller.pitch_m = inch2meter(pitch_in);
propeller.area_m2 = pi*(propeller.diameter_m/2)^2;
propeller.radius_m = propeller.diameter_m/2;

fprintf('Propeller size: %.2fx%.2f in (%s)\n', diameter_in, pitch_in, char(prop_size_source));
fprintf('Disk area: %.4f m^2\n', propeller.area_m2);

%% Unit normalization and sample selection
time_s = data.time;
rpm = data.rpm;
voltage_v = data.voltage;
current_a = data.current;
power_electric_w = data.power;
thrust_n = thrust_to_newton(data.thrust, thrust_unit);
thrust_gf = thrust_n / 9.80665 * 1000;

valid = isfinite(time_s) & isfinite(rpm) & isfinite(voltage_v) & ...
        isfinite(current_a) & isfinite(power_electric_w) & isfinite(thrust_n) & ...
        rpm > min_rpm & power_electric_w > min_power_w & thrust_n > min_thrust_n;

if nnz(valid) < 5
    error('Not enough valid samples after filtering. Check thrust/RPM/power units.');
end

time_s = time_s(valid);
rpm = rpm(valid);
voltage_v = voltage_v(valid);
current_a = current_a(valid);
power_electric_w = power_electric_w(valid);
thrust_n = thrust_n(valid);
thrust_gf = thrust_gf(valid);

%% Momentum Theory calculations
analysis = struct();
analysis.time_s = time_s;
analysis.rpm = rpm;
analysis.rps = rpm/60;
analysis.omega_rad_s = 2*pi*analysis.rps;
analysis.voltage_v = voltage_v;
analysis.current_a = current_a;
analysis.power_electric_w = power_electric_w;
analysis.thrust_n = thrust_n;

% Static actuator-disk relation. vi is the speed through the disk induced by
% the propeller; the far wake speed increment is 2*vi for static inflow.
analysis.vi_disk_m_s = sqrt(analysis.thrust_n ./ (2*rho*propeller.area_m2));
analysis.vwake_far_m_s = 2*analysis.vi_disk_m_s;
analysis.power_induced_ideal_w = analysis.thrust_n .* analysis.vi_disk_m_s;
analysis.mass_flow_disk_kg_s = rho * propeller.area_m2 .* analysis.vi_disk_m_s;
analysis.volume_flow_disk_m3_s = propeller.area_m2 .* analysis.vi_disk_m_s;

% Electrical-to-ideal efficiency. This is not total propeller efficiency; it
% compares the ideal induced power needed for measured thrust against measured
% electrical input power, so motor/ESC/prop losses all reduce this value.
analysis.figure_of_merit_electric = analysis.power_induced_ideal_w ./ analysis.power_electric_w;
analysis.thrust_per_power_n_w = analysis.thrust_n ./ analysis.power_electric_w;
analysis.thrust_per_power_gf_w = (analysis.thrust_n/9.80665*1000) ./ analysis.power_electric_w;

% Non-dimensional coefficients are useful to compare propellers. Cp is based
% on electrical power because shaft torque was not measured.
analysis.ct = analysis.thrust_n ./ (rho .* analysis.rps.^2 .* propeller.diameter_m^4);
analysis.cp_electric = analysis.power_electric_w ./ (rho .* analysis.rps.^3 .* propeller.diameter_m^5);
analysis.static_advance_ratio = zeros(size(analysis.rps));

analysis.v_elevon_m_s = estimate_slipstream_velocity( ...
    analysis.vi_disk_m_s, analysis.vwake_far_m_s, propeller.radius_m, distanceProp2Elevons);

%% Summary
fprintf('\n=== Momentum Theory Summary ===\n');
fprintf('Valid samples:            %d\n', numel(analysis.time_s));
fprintf('Thrust:                   mean=%7.0f gf, max=%7.0f gf\n', mean(thrust_gf), max(thrust_gf));
fprintf('Thrust:                   mean=%7.3f N,  max=%7.3f N\n', mean(thrust_n), max(thrust_n));
fprintf('Electrical power:         mean=%7.2f W, max=%7.2f W\n', mean(power_electric_w), max(power_electric_w));
fprintf('RPM:                      mean=%7.0f, max=%7.0f\n', mean(rpm), max(rpm));
fprintf('Disk induced velocity vi: mean=%7.2f m/s, max=%7.2f m/s\n', ...
        mean(analysis.vi_disk_m_s), max(analysis.vi_disk_m_s));
fprintf('Far wake velocity 2vi:    mean=%7.2f m/s, max=%7.2f m/s\n', ...
        mean(analysis.vwake_far_m_s), max(analysis.vwake_far_m_s));
fprintf('Elevon velocity estimate: mean=%7.2f m/s, max=%7.2f m/s at %.2f m\n', ...
        mean(analysis.v_elevon_m_s), max(analysis.v_elevon_m_s), distanceProp2Elevons);
fprintf('Mass flow at disk:        mean=%7.3f kg/s, max=%7.3f kg/s\n', ...
        mean(analysis.mass_flow_disk_kg_s), max(analysis.mass_flow_disk_kg_s));
fprintf('Ideal induced power:      mean=%7.2f W, max=%7.2f W\n', ...
        mean(analysis.power_induced_ideal_w), max(analysis.power_induced_ideal_w));
fprintf('Electric figure of merit: mean=%7.3f, max=%7.3f\n', ...
        mean(analysis.figure_of_merit_electric), max(analysis.figure_of_merit_electric));
fprintf('Thrust efficiency:        mean=%7.2f gf/W, max=%7.2f gf/W\n', ...
        mean(analysis.thrust_per_power_gf_w), max(analysis.thrust_per_power_gf_w));

%% Plots
figure('Name', 'PSO - Momentum Theory Time Analysis', ...
       'Position', [100, 100, 1200, 800]);

subplot(3, 2, 1)
plot(time_s, thrust_n, 'm-', 'LineWidth', 1.3)
grid on
xlabel('Time (s)')
ylabel('Thrust (N)')
title('Measured Thrust')

subplot(3, 2, 2)
plot(time_s, power_electric_w, 'c-', 'LineWidth', 1.3)
hold on
plot(time_s, analysis.power_induced_ideal_w, 'k-', 'LineWidth', 1.3)
grid on
xlabel('Time (s)')
ylabel('Power (W)')
title('Electrical vs Ideal Induced Power')
legend('Electrical input', 'Ideal induced', 'Location', 'best')

subplot(3, 2, 3)
plot(time_s, analysis.vi_disk_m_s, 'b-', 'LineWidth', 1.3)
hold on
plot(time_s, analysis.vwake_far_m_s, 'r-', 'LineWidth', 1.3)
plot(time_s, analysis.v_elevon_m_s, 'Color', [0.1 0.55 0.1], 'LineWidth', 1.3)
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Slipstream Velocity Estimates')
legend('Disk vi', 'Far wake 2vi', 'At elevon', 'Location', 'best')

subplot(3, 2, 4)
plot(time_s, analysis.mass_flow_disk_kg_s, 'LineWidth', 1.3)
grid on
xlabel('Time (s)')
ylabel('Mass flow (kg/s)')
title('Air Mass Flow Through Disk')

subplot(3, 2, 5)
plot(time_s, analysis.figure_of_merit_electric, 'LineWidth', 1.3)
grid on
xlabel('Time (s)')
ylabel('P_{ideal}/P_{elec}')
title('Electric Figure of Merit')
ylim([0, max(1, 1.1*max(analysis.figure_of_merit_electric))])

subplot(3, 2, 6)
plot(time_s, analysis.thrust_per_power_gf_w, 'LineWidth', 1.3)
grid on
xlabel('Time (s)')
ylabel('gf/W')
title('Static Thrust per Electrical Power')

sgtitle('PSO Momentum Theory Analysis', 'FontSize', 14, 'FontWeight', 'bold')

figure('Name', 'PSO - Propeller Coefficients', ...
       'Position', [150, 150, 1100, 700]);

subplot(2, 2, 1)
scatter(rpm, thrust_n, 12, power_electric_w, 'filled')
grid on
xlabel('RPM')
ylabel('Thrust (N)')
title('Measured Thrust vs RPM')
cb = colorbar;
cb.Label.String = 'Electrical power (W)';

subplot(2, 2, 2)
scatter(power_electric_w, analysis.vwake_far_m_s, 12, rpm, 'filled')
grid on
xlabel('Electrical power (W)')
ylabel('Far wake velocity (m/s)')
title('Wake Velocity vs Power')
cb = colorbar;
cb.Label.String = 'RPM';

subplot(2, 2, 3)
scatter(rpm, analysis.ct, 12, thrust_n, 'filled')
grid on
xlabel('RPM')
ylabel('C_T')
title('Static Thrust Coefficient')
cb = colorbar;
cb.Label.String = 'Thrust (N)';

subplot(2, 2, 4)
scatter(rpm, analysis.cp_electric, 12, power_electric_w, 'filled')
grid on
xlabel('RPM')
ylabel('C_{P,elec}')
title('Electrical Power Coefficient')
cb = colorbar;
cb.Label.String = 'Electrical power (W)';

%% Heuristic wake development for the highest-thrust sample
[~, peak_idx] = max(thrust_n);
R = propeller.radius_m;
d = linspace(0, 6*R, 300);
lambda_list = [0.5 1 2 3]*R;

figure('Name', 'PSO - Wake Development Model', ...
       'Position', [200, 200, 900, 550]);
hold on
grid on
box on
for lambda = lambda_list
    v_profile = analysis.vi_disk_m_s(peak_idx) + ...
        (analysis.vwake_far_m_s(peak_idx) - analysis.vi_disk_m_s(peak_idx)) .* ...
        (1 - exp(-d/lambda));
    plot(d/R, v_profile, 'LineWidth', 2)
end
xline(distanceProp2Elevons/R, 'r--', 'Elevon', 'LineWidth', 1.5)
yline(analysis.vi_disk_m_s(peak_idx), 'k--', 'V_i', 'LineWidth', 1.2)
yline(analysis.vwake_far_m_s(peak_idx), 'k:', '2V_i', 'LineWidth', 1.2)
xlabel('Distance from propeller plane (d/R)')
ylabel('Slipstream velocity (m/s)')
title('Heuristic Slipstream Development at Peak Thrust')
legend('\lambda=0.5R', '\lambda=R', '\lambda=2R', '\lambda=3R', ...
       'Elevon', 'V_i', '2V_i', 'Location', 'southeast')

%% Functions
function data = normalize_data_fields(data)
    required_fields = ["time", "rpm", "current", "voltage", "thrust"];
    for k = 1:numel(required_fields)
        field_name = char(required_fields(k));
        if ~isfield(data, field_name)
            error('Missing required data field: %s', field_name);
        end
        data.(field_name) = double(data.(field_name)(:));
    end

    if ~isfield(data, 'power') || isempty(data.power)
        data.power = data.voltage .* data.current;
    else
        data.power = double(data.power(:));
    end
end

function thrust_n = thrust_to_newton(thrust_raw, thrust_unit)
    thrust_raw = double(thrust_raw(:));

    % Important: grams-force is not a metric prefix of newtons. A reading of
    % 2000 gf is approximately 19.6 N, not 2 N.
    switch lower(string(thrust_unit))
        case {"mn", "millinewton", "millinewtons"}
            thrust_n = thrust_raw / 1000;
        case {"n", "newton", "newtons"}
            thrust_n = thrust_raw;
        case {"gf", "gram-force", "grams-force"}
            thrust_n = thrust_raw * 9.80665e-3;
        otherwise
            error('Unsupported thrust unit: %s', char(thrust_unit));
    end
end

function [diameter_in, pitch_in, source] = get_propeller_size(loaded, mat_file, fallback_size)
    source = "fallback";
    size_text = "";

    if isfield(loaded, 'test_information') && isfield(loaded.test_information, 'propeller')
        propeller_info = loaded.test_information.propeller;
        if isfield(propeller_info, 'size_in')
            size_text = string(propeller_info.size_in);
            source = "test_information.propeller.size_in";
        elseif isfield(propeller_info, 'diameter_in') && isfield(propeller_info, 'pitch_in')
            diameter_in = double(propeller_info.diameter_in);
            pitch_in = double(propeller_info.pitch_in);
            source = "test_information propeller diameter/pitch";
            return
        end
    end

    if strlength(size_text) == 0
        [~, name, ~] = fileparts(mat_file);
        token = regexp(name, '(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)', 'tokens', 'once');
        if ~isempty(token)
            size_text = string(token{1}) + "x" + string(token{2});
            source = "file name";
        end
    end

    if strlength(size_text) == 0
        size_text = fallback_size;
    end

    tokens = regexp(char(size_text), '(\d+(?:\.\d+)?)\s*x\s*(\d+(?:\.\d+)?)', 'tokens', 'once');
    if isempty(tokens)
        error('Could not parse propeller size "%s". Expected format like 15x8.', char(size_text));
    end

    diameter_in = str2double(tokens{1});
    pitch_in = str2double(tokens{2});
end

function v_at_distance = estimate_slipstream_velocity(vi, v_wake, radius_m, distance_m)
    % This exponential model is only a near-wake heuristic. Momentum theory
    % provides the endpoints (vi at the disk and 2*vi far downstream), while
    % lambda controls how quickly the wake approaches the far value.
    lambda = 1.5*radius_m;
    v_at_distance = vi + (v_wake - vi).*(1 - exp(-distance_m/lambda));
end

function meters = inch2meter(inches)
    meters = inches*0.0254;
end
