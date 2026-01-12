%% Data Structure Creation and File Management for Testbench Information
% Author: Rogerio Lima
% Date: December 2025
% Description: Creates a structured data format for propulsion system testing
%              with automatic file naming and user confirmation dialog

function test_info = create_testbench_info(varargin)
% CREATE_TESTBENCH_INFO - Create and manage testbench information structure
%
% Usage:
%   test_info = create_testbench_info() - Create with default values
%   test_info = create_testbench_info('param1', value1, ...) - Create with custom values
%   test_info = create_testbench_info(test_info) - Edit existing structure
%
% Example:
%   test_info = create_testbench_info('battery.num_cells', 4, ...
%                                     'motor.kv', 980);

    %% Initialize default structure
    test_info = struct();
    
    % Get current date and time for filename
    current_time = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
    
    %% BATTERY Information
    test_info.battery = struct(...
        'num_cells', 3, ...
        'capacity_mAh', 2200, ...
        'chemistry', 'LiPo', ...
        'manufacturer', 'Turnigy', ...
        'model', 'Nano-Tech', ...
        'discharge_C', 25, ...
        'voltage_cell_nominal', 3.7, ...
        'voltage_cell_min', 3.0, ...
        'voltage_cell_max', 4.2);
    
    %% ESC (Electronic Speed Controller) Information
    test_info.esc = struct(...
        'model', 'BLHeli_32', ...
        'firmware_version', '32.8', ...
        'configuration', 'DShot600', ...
        'current_rating_A', 40, ...
        'voltage_max_V', 16.8, ...
        'manufacturer', 'Holybro', ...
        'protocol', 'DShot', ...
        'timing', 'Medium', ...
        'pwm_frequency_hz', 24);
    
    %% MOTOR Information
    test_info.motor = struct(...
        'model', 'T-Motor F60', ...
        'kv', 1750, ...
        'pole_count', 14, ...
        'resistance_ohm', 0.025, ...
        'max_current_A', 45, ...
        'max_power_W', 850, ...
        'weight_g', 45, ...
        'shaft_diameter_mm', 4, ...
        'manufacturer', 'T-Motor', ...
        'wiring_config', 'Delta');
    
    %% PROPELLER Information
    test_info.propeller = struct(...
        'model', 'Gemfan 51466', ...
        'diameter_in', 5.1, ...
        'pitch_in', 4.66, ...
        'rotation', 'CW', ...
        'balanced', 'Y', ...
        'blade_count', 3, ...
        'material', 'Carbon Composite', ...
        'hub_diameter_mm', 5, ...
        'manufacturer', 'Gemfan');
    
    %% TEST Data Information
    test_info.test = struct(...
        'date', datestr(now, 'dd-mmm-yyyy HH:MM:SS'), ...
        'start_time', char(current_time), ...
        'duration_min', 0, ...
        'ambient_temp_C', 25, ...
        'humidity_percent', 50, ...
        'location', 'Lab 101', ...
        'operator', 'Operator Name', ...
        'test_purpose', 'Static Thrust Characterization', ...
        'notes', 'Initial baseline test');
    
    %% FIRMWARE Information
    test_info.firmware = struct(...
        'version', 'PSO-e v2.0', ...
        'git_commit', 'a1b2c3d4', ...
        'compilation_date', '05-Dec-2025', ...
        'sampling_rate_hz', 5000, ...
        'streaming_rate_hz', 500, ...
        'pwm_resolution', '16-bit', ...
        'communication_protocol', 'UART @ 115200');
    
    %% FILE Information
    test_info.file = struct(...
        'base_name', sprintf('test_%s', char(current_time)), ...
        'extension', '.mat', ...
        'folder', pwd, ...
        'full_path', '', ...
        'description', 'Testbench data file');
    
    % Generate full path
    test_info.file.full_path = fullfile(...
        test_info.file.folder, ...
        [test_info.file.base_name test_info.file.extension]);
    
    %% SENSOR Information (Optional)
    test_info.sensors = struct(...
        'current_sensor_model', 'ACS712', ...
        'current_sensor_range_A', 50, ...
        'voltage_sensor_type', 'Voltage Divider', ...
        'rpm_sensor_type', 'Hall Effect', ...
        'thrust_sensor_model', 'Load Cell 5kg', ...
        'temperature_sensor_model', 'DS18B20', ...
        'data_acquisition', 'TM4C123 @ 40MHz');
    
    %% Handle input parameters
    if nargin > 0
        if isstruct(varargin{1})
            % Edit existing structure
            existing_info = varargin{1};
            test_info = merge_structures(test_info, existing_info);
        else
            % Apply parameter/value pairs
            test_info = apply_parameters(test_info, varargin{:});
        end
    end
    
    %% Generate confirmation dialog
    confirmed = launch_confirmation_dialog(test_info);
    
    if confirmed
        %% Save to file automatically
        save_testbench_data(test_info);
        
        %% Display success message
        fprintf('\n═══════════════════════════════════════════════════════\n');
        fprintf('✅ TESTBENCH INFORMATION SAVED SUCCESSFULLY!\n');
        fprintf('═══════════════════════════════════════════════════════\n');
        fprintf('File: %s\n', test_info.file.full_path);
        fprintf('Date: %s\n', test_info.test.date);
        fprintf('Test: %s\n', test_info.test.test_purpose);
        fprintf('Motor: %s (%d KV)\n', test_info.motor.model, test_info.motor.kv);
        fprintf('Prop: %s %.1f"x%.2f" %s\n', ...
            test_info.propeller.model, ...
            test_info.propeller.diameter_in, ...
            test_info.propeller.pitch_in, ...
            test_info.propeller.rotation);
        fprintf('Battery: %dS %dmAh %s\n', ...
            test_info.battery.num_cells, ...
            test_info.battery.capacity_mAh, ...
            test_info.battery.chemistry);
        fprintf('═══════════════════════════════════════════════════════\n\n');
    else
        fprintf('❌ Testbench setup cancelled by user.\n');
        test_info = [];
    end
end

%% Helper Functions
function merged = merge_structures(default, custom)
% Merge two structures, with custom values overriding defaults
    merged = default;
    fields = fieldnames(custom);
    for i = 1:length(fields)
        if isstruct(custom.(fields{i})) && isfield(default, fields{i})
            % Recursively merge substructures
            merged.(fields{i}) = merge_structures(...
                default.(fields{i}), custom.(fields{i}));
        else
            % Replace with custom value
            merged.(fields{i}) = custom.(fields{i});
        end
    end
end

function info = apply_parameters(info, varargin)
% Apply parameter/value pairs to structure
    for i = 1:2:length(varargin)
        param_path = strsplit(varargin{i}, '.');
        current_struct = info;
        
        % Navigate to the correct substructure
        for j = 1:length(param_path)-1
            if ~isfield(current_struct, param_path{j})
                current_struct.(param_path{j}) = struct();
            end
            current_struct = current_struct.(param_path{j});
        end
        
        % Set the value
        param_name = param_path{end};
        if isstruct(varargin{i+1})
            % Merge substructures
            if isfield(current_struct, param_name)
                current_struct.(param_name) = merge_structures(...
                    current_struct.(param_name), varargin{i+1});
            else
                current_struct.(param_name) = varargin{i+1};
            end
        else
            % Set simple value
            current_struct.(param_name) = varargin{i+1};
        end
    end
end

function confirmed = launch_confirmation_dialog(test_info)
% Create confirmation dialog with testbench information
    
    %% Create figure window
    fig = figure('Name', 'Testbench Configuration - Confirm Details', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 800, 600], ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Color', [0.95 0.95 0.95]);
    
    %% Title
    uicontrol('Style', 'text', ...
              'String', 'TESTBENCH CONFIGURATION - PLEASE REVIEW', ...
              'Position', [20, 550, 760, 30], ...
              'FontSize', 14, ...
              'FontWeight', 'bold', ...
              'HorizontalAlignment', 'center', ...
              'BackgroundColor', [0.7 0.8 1.0]);
    
    %% Create tabbed interface
    tab_group = uitabgroup(fig, 'Position', [0.02, 0.1, 0.96, 0.75]);
    
    %% Battery Tab
    battery_tab = uitab(tab_group, 'Title', '🔋 Battery');
    create_battery_panel(battery_tab, test_info.battery);
    
    %% ESC Tab
    esc_tab = uitab(tab_group, 'Title', '⚡ ESC');
    create_esc_panel(esc_tab, test_info.esc);
    
    %% Motor Tab
    motor_tab = uitab(tab_group, 'Title', '🔄 Motor');
    create_motor_panel(motor_tab, test_info.motor);
    
    %% Propeller Tab
    prop_tab = uitab(tab_group, 'Title', '✈️ Propeller');
    create_propeller_panel(prop_tab, test_info.propeller);
    
    %% Test Info Tab
    test_tab = uitab(tab_group, 'Title', '📊 Test Info');
    create_test_panel(test_tab, test_info.test, test_info.file);
    
    %% Firmware Tab
    fw_tab = uitab(tab_group, 'Title', '💾 Firmware');
    create_firmware_panel(fw_tab, test_info.firmware);
    
    %% Summary at bottom
    uicontrol('Style', 'text', ...
              'String', sprintf('Filename: %s', test_info.file.base_name), ...
              'Position', [20, 60, 760, 20], ...
              'FontSize', 10, ...
              'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.95 0.95 0.95]);
    
    uicontrol('Style', 'text', ...
              'String', sprintf('Full Path: %s', test_info.file.full_path), ...
              'Position', [20, 40, 760, 20], ...
              'FontSize', 9, ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.95 0.95 0.95]);
    
    %% Confirmation buttons
    uicontrol('Style', 'pushbutton', ...
              'String', '✅ CONFIRM & SAVE', ...
              'Position', [200, 10, 150, 30], ...
              'FontSize', 11, ...
              'FontWeight', 'bold', ...
              'BackgroundColor', [0.3 0.8 0.3], ...
              'ForegroundColor', 'white', ...
              'Callback', @(src, evt) confirm_callback());
    
    uicontrol('Style', 'pushbutton', ...
              'String', '❌ CANCEL', ...
              'Position', [450, 10, 150, 30], ...
              'FontSize', 11, ...
              'BackgroundColor', [0.9 0.3 0.3], ...
              'ForegroundColor', 'white', ...
              'Callback', @(src, evt) cancel_callback());
    
    %% Wait for user response
    confirmed = false;
    uiwait(fig);
    
    %% Callback functions
    function confirm_callback()
        confirmed = true;
        delete(fig);
    end
    
    function cancel_callback()
        confirmed = false;
        delete(fig);
    end
end

%% Panel Creation Functions
function create_battery_panel(parent, battery_info)
    y_pos = 250;
    row_height = 25;
    
    fields = {'num_cells', 'Cells:';
              'capacity_mAh', 'Capacity (mAh):';
              'chemistry', 'Chemistry:';
              'manufacturer', 'Manufacturer:';
              'model', 'Model:';
              'discharge_C', 'Discharge (C):';
              'voltage_cell_nominal', 'Nominal Voltage:';
              'voltage_cell_min', 'Min Voltage:';
              'voltage_cell_max', 'Max Voltage:'};
    
    for i = 1:size(fields, 1)
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', fields{i,2}, ...
                  'Position', [20, y_pos - i*row_height, 150, 20], ...
                  'HorizontalAlignment', 'right');
        
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', string(battery_info.(fields{i,1})), ...
                  'Position', [180, y_pos - i*row_height, 200, 20], ...
                  'FontWeight', 'bold', ...
                  'HorizontalAlignment', 'left');
    end
    
    % Calculate pack voltage
    pack_voltage = battery_info.num_cells * battery_info.voltage_cell_nominal;
    uicontrol('Parent', parent, ...
              'Style', 'text', ...
              'String', sprintf('Pack Voltage: %.1f V', pack_voltage), ...
              'Position', [20, 50, 200, 30], ...
              'FontSize', 12, ...
              'FontWeight', 'bold', ...
              'ForegroundColor', [0 0.4 0]);
end

function create_esc_panel(parent, esc_info)
    y_pos = 250;
    row_height = 25;
    
    fields = {'model', 'Model:';
              'firmware_version', 'Firmware:';
              'configuration', 'Configuration:';
              'current_rating_A', 'Current Rating (A):';
              'voltage_max_V', 'Max Voltage (V):';
              'manufacturer', 'Manufacturer:';
              'protocol', 'Protocol:';
              'timing', 'Timing:';
              'pwm_frequency_hz', 'PWM Freq (Hz):'};
    
    for i = 1:size(fields, 1)
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', fields{i,2}, ...
                  'Position', [20, y_pos - i*row_height, 150, 20], ...
                  'HorizontalAlignment', 'right');
        
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', string(esc_info.(fields{i,1})), ...
                  'Position', [180, y_pos - i*row_height, 200, 20], ...
                  'FontWeight', 'bold', ...
                  'HorizontalAlignment', 'left');
    end
end

function create_motor_panel(parent, motor_info)
    y_pos = 250;
    row_height = 25;
    
    fields = {'model', 'Model:';
              'kv', 'KV Rating:';
              'pole_count', 'Pole Count:';
              'resistance_ohm', 'Resistance (Ω):';
              'max_current_A', 'Max Current (A):';
              'max_power_W', 'Max Power (W):';
              'weight_g', 'Weight (g):';
              'manufacturer', 'Manufacturer:';
              'wiring_config', 'Wiring:'};
    
    for i = 1:size(fields, 1)
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', fields{i,2}, ...
                  'Position', [20, y_pos - i*row_height, 150, 20], ...
                  'HorizontalAlignment', 'right');
        
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', string(motor_info.(fields{i,1})), ...
                  'Position', [180, y_pos - i*row_height, 200, 20], ...
                  'FontWeight', 'bold', ...
                  'HorizontalAlignment', 'left');
    end
end

function create_propeller_panel(parent, prop_info)
    y_pos = 250;
    row_height = 25;
    
    fields = {'model', 'Model:';
              'diameter_in', 'Diameter (in):';
              'pitch_in', 'Pitch (in):';
              'rotation', 'Rotation:';
              'balanced', 'Balanced:';
              'blade_count', 'Blades:';
              'material', 'Material:';
              'manufacturer', 'Manufacturer:'};
    
    for i = 1:size(fields, 1)
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', fields{i,2}, ...
                  'Position', [20, y_pos - i*row_height, 150, 20], ...
                  'HorizontalAlignment', 'right');
        
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', string(prop_info.(fields{i,1})), ...
                  'Position', [180, y_pos - i*row_height, 200, 20], ...
                  'FontWeight', 'bold', ...
                  'HorizontalAlignment', 'left');
    end
    
    % Display propeller size in traditional format
    size_str = sprintf('%.1f"x%.2f"', prop_info.diameter_in, prop_info.pitch_in);
    uicontrol('Parent', parent, ...
              'Style', 'text', ...
              'String', sprintf('Size: %s', size_str), ...
              'Position', [20, 50, 200, 30], ...
              'FontSize', 14, ...
              'FontWeight', 'bold', ...
              'ForegroundColor', [0.6 0.2 0]);
end

function create_test_panel(parent, test_info, file_info)
    y_pos = 250;
    row_height = 25;
    
    fields = {'date', 'Date:';
              'start_time', 'Start Time:';
              'location', 'Location:';
              'operator', 'Operator:';
              'test_purpose', 'Test Purpose:';
              'ambient_temp_C', 'Temperature (°C):';
              'humidity_percent', 'Humidity (%):'};
    
    for i = 1:size(fields, 1)
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', fields{i,2}, ...
                  'Position', [20, y_pos - i*row_height, 150, 20], ...
                  'HorizontalAlignment', 'right');
        
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', string(test_info.(fields{i,1})), ...
                  'Position', [180, y_pos - i*row_height, 250, 20], ...
                  'FontWeight', 'bold', ...
                  'HorizontalAlignment', 'left');
    end
    
    % Notes area
    uicontrol('Parent', parent, ...
              'Style', 'text', ...
              'String', 'Notes:', ...
              'Position', [20, 80, 150, 20], ...
              'HorizontalAlignment', 'right');
    
    uicontrol('Parent', parent, ...
              'Style', 'edit', ...
              'String', test_info.notes, ...
              'Max', 3, ...
              'Position', [180, 20, 250, 80], ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', 'white');
end

function create_firmware_panel(parent, fw_info)
    y_pos = 250;
    row_height = 25;
    
    fields = {'version', 'Version:';
              'git_commit', 'Git Commit:';
              'compilation_date', 'Compiled:';
              'sampling_rate_hz', 'Sampling Rate (Hz):';
              'streaming_rate_hz', 'Streaming Rate (Hz):';
              'pwm_resolution', 'PWM Resolution:';
              'communication_protocol', 'Protocol:'};
    
    for i = 1:size(fields, 1)
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', fields{i,2}, ...
                  'Position', [20, y_pos - i*row_height, 150, 20], ...
                  'HorizontalAlignment', 'right');
        
        uicontrol('Parent', parent, ...
                  'Style', 'text', ...
                  'String', string(fw_info.(fields{i,1})), ...
                  'Position', [180, y_pos - i*row_height, 200, 20], ...
                  'FontWeight', 'bold', ...
                  'HorizontalAlignment', 'left');
    end
end

function save_testbench_data(test_info)
% Save testbench data to MAT file with metadata
    
    %% Create folder if it doesn't exist
    if ~exist(test_info.file.folder, 'dir')
        mkdir(test_info.file.folder);
    end
    
    %% Prepare data for saving
    save_data = struct();
    
    % Main structure
    save_data.testbench_info = test_info;
    
    % Add metadata
    save_data.metadata = struct(...
        'creation_date', datestr(now, 'dd-mmm-yyyy HH:MM:SS'), ...
        'matlab_version', version, ...
        'toolbox_dependencies', {'Signal Processing', 'Instrument Control'}, ...
        'author', 'PSO-e Testbench System');
    
    % Add calculation fields
    save_data.calculations = struct(...
        'battery_pack_voltage', test_info.battery.num_cells * test_info.battery.voltage_cell_nominal, ...
        'battery_energy_Wh', (test_info.battery.capacity_mAh / 1000) * ...
                             test_info.battery.num_cells * test_info.battery.voltage_cell_nominal, ...
        'max_power_kW', test_info.motor.max_power_W / 1000, ...
        'thrust_constant', NaN, ...  % Will be filled during testing
        'torque_constant', NaN);     % Will be filled during testing
    
    %% Save to file
    try
        save(test_info.file.full_path, '-struct', 'save_data');
        fprintf('Data saved to: %s\n', test_info.file.full_path);
        
        % Also save a human-readable JSON version
        json_file = strrep(test_info.file.full_path, '.mat', '.json');
        json_text = jsonencode(save_data, 'PrettyPrint', true);
        fid = fopen(json_file, 'w');
        fprintf(fid, '%s', json_text);
        fclose(fid);
        
    catch ME
        warning('Failed to save data: %s', ME.message);
        
        % Try alternative location
        alt_path = fullfile(pwd, [test_info.file.base_name test_info.file.extension]);
        save(alt_path, '-struct', 'save_data');
        fprintf('Data saved to alternative location: %s\n', alt_path);
    end
end

%% Standalone Usage Function
function run_testbench_setup()
% RUN_TESTBENCH_SETUP - Main function for standalone execution
    
    fprintf('\n═══════════════════════════════════════════════════════\n');
    fprintf('          PSO-e TESTBENCH CONFIGURATION SYSTEM\n');
    fprintf('═══════════════════════════════════════════════════════\n\n');
    
    % Ask user for configuration mode
    fprintf('Configuration options:\n');
    fprintf('  1. Use default values\n');
    fprintf('  2. Enter custom values\n');
    fprintf('  3. Load from existing file\n');
    
    choice = input('\nSelect option (1-3): ', 's');
    
    switch choice
        case '1'
            % Use defaults
            test_info = create_testbench_info();
            
        case '2'
            % Get custom values
            fprintf('\n--- Enter Custom Values ---\n');
            
            % Get battery info
            num_cells = input('Number of cells (3): ');
            if isempty(num_cells), num_cells = 3; end
            
            capacity = input('Capacity (mAh) [2200]: ');
            if isempty(capacity), capacity = 2200; end
            
            % Get motor info
            kv = input('Motor KV [1750]: ');
            if isempty(kv), kv = 1750; end
            
            % Create with custom values
            test_info = create_testbench_info(...
                'battery.num_cells', num_cells, ...
                'battery.capacity_mAh', capacity, ...
                'motor.kv', kv);
            
        case '3'
            % Load from file
            [file, path] = uigetfile('*.mat', 'Select testbench file');
            if file == 0
                fprintf('No file selected. Using defaults.\n');
                test_info = create_testbench_info();
            else
                loaded_data = load(fullfile(path, file));
                if isfield(loaded_data, 'testbench_info')
                    test_info = create_testbench_info(loaded_data.testbench_info);
                else
                    fprintf('Invalid file format. Using defaults.\n');
                    test_info = create_testbench_info();
                end
            end
            
        otherwise
            fprintf('Invalid choice. Using defaults.\n');
            test_info = create_testbench_info();
    end
    
    % Display results if not cancelled
    if ~isempty(test_info)
        fprintf('\n✅ Testbench configuration complete!\n');
        fprintf('   File: %s\n', test_info.file.full_path);
        fprintf('   Ready for data acquisition.\n\n');
        
        % Option to plot expected performance
        plot_expected_performance(test_info);
    end
end

function plot_expected_performance(test_info)
% Plot expected performance based on configuration
    
    figure('Name', 'Expected Performance', ...
           'NumberTitle', 'off', ...
           'Position', [100, 100, 900, 500]);
    
    % Subplot 1: Voltage vs Thrust (simplified model)
    subplot(2, 2, 1);
    
    voltage_range = linspace(10, 16.8, 100);
    thrust_estimated = 0.15 * voltage_range.^2;  % Simplified model
    
    plot(voltage_range, thrust_estimated, 'b-', 'LineWidth', 2);
    xlabel('Voltage (V)');
    ylabel('Estimated Thrust (kgf)');
    title('Thrust vs Voltage');
    grid on;
    
    % Subplot 2: Current vs RPM
    subplot(2, 2, 2);
    
    rpm_range = linspace(0, 15000, 100);
    current_estimated = (rpm_range / test_info.motor.kv) * 0.8;  % Simplified
    
    plot(rpm_range, current_estimated, 'r-', 'LineWidth', 2);
    xlabel('RPM');
    ylabel('Estimated Current (A)');
    title('Current vs RPM');
    grid on;
    
    % Subplot 3: Power consumption
    subplot(2, 2, 3);
    
    power_estimated = current_estimated .* voltage_range(1:length(current_estimated));
    
    plot(rpm_range, power_estimated, 'g-', 'LineWidth', 2);
    xlabel('RPM');
    ylabel('Power (W)');
    title('Power vs RPM');
    grid on;
    
    % Subplot 4: Efficiency
    subplot(2, 2, 4);
    
    efficiency_estimated = thrust_estimated(1:length(power_estimated)) ./ ...
                          (power_estimated / 100) * 8;  % g/W
    
    plot(rpm_range, efficiency_estimated, 'm-', 'LineWidth', 2);
    xlabel('RPM');
    ylabel('Efficiency (g/W)');
    title('Estimated Efficiency');
    grid on;
    
    sgtitle(sprintf('Expected Performance: %s %dKV | %dS %dmAh', ...
        test_info.motor.model, test_info.motor.kv, ...
        test_info.battery.num_cells, test_info.battery.capacity_mAh));
end

%% Usage Examples
% 
% Example 1: Create with defaults
%   test_info = create_testbench_info();
%
% Example 2: Create with custom values
%   test_info = create_testbench_info(...
%       'battery.num_cells', 4, ...
%       'battery.capacity_mAh', 3000, ...
%       'motor.kv', 2300, ...
%       'propeller.diameter_in', 5.5, ...
%       'propeller.pitch_in', 4.5, ...
%       'test.operator', 'Rogerio Lima');
%
% Example 3: Edit existing structure
%   test_info_modified = create_testbench_info(test_info);
%
% Example 4: Run standalone GUI
%   run_testbench_setup();