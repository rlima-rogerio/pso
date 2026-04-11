function info = test_info_text()
% Lightweight, text-based PSO / e-PSA testbench info collector (no GUI).
% 1) Shows pre-configured options first
% 2) User can accept all or edit
% 3) If editing, prompts each field (ENTER keeps default)
% Returns struct 'info'.false

    % ---------------- Defaults ----------------
    info = struct( ...
        'battery',   struct('num_cells', 4, 'capacity_mAh', 3000, 'charge_state', "80"), ...
        'esc',       struct('model', "Flier-80A", 'configuration', "Default"), ...
        'motor',     struct('model', "Propdrive-42-48", 'kv', 650), ...
        'propeller', struct('model', "Glass-Fiber-Nylon GEMFAN", 'size_in', "15x8", 'rotation', "CW", 'balanced', true), ...
        'test_data', struct('date', datetime("now"), 'filename', ""), ...
        'firmware',  struct('version', "v2.0.0") ...
    );

    % Timestamped filename (kept for reference)
    ts_str = datestr(datetime("now"), "yyyy-mm-dd_HH-MM-SS");
    info.test_data.filename = "pso_test_" + string(ts_str);

    % Base filename default (matches your UI code)
    base = "test_" + ...
        info.battery.num_cells + "S_" + ...
        info.esc.model + "_" + ...
        info.motor.model + "_" + ...
        info.propeller.model + "-" + ...
        info.propeller.size_in;

    info.test_data.filename = base; % use base as default visible filename

    % ---------------- Show config first ----------------
    fprintf('\n=== Confirm Testbench Information (e-PSA) ===\n');
    fprintf('Pre-configured options:\n\n');
    print_summary(info);

    resp = upper(strtrim(input('Accept this configuration? [Y]es / [E]dit / [C]ancel: ', 's')));
    if isempty(resp), resp = "Y"; end

    switch resp
        case "Y"
            info.test_data.date = datetime("now");
            return;
        case "E"
            % proceed to per-field editing below
        case "C"
            error('User cancelled. No test_info returned.');
        otherwise
            fprintf('Invalid choice. Defaulting to Edit.\n');
    end

    % ---------------- Edit (grouped) ----------------
    fprintf('\n(Press ENTER to keep the default shown in brackets)\n\n');

    fprintf('[Battery]\n');
    info.battery.num_cells    = ask_int('# Cells', info.battery.num_cells, 1, inf);
    info.battery.capacity_mAh = ask_int('Capacity (mAh)', info.battery.capacity_mAh, 0, inf);
    info.battery.charge_state = ask_str('Charge', info.battery.charge_state);

    fprintf('\n[ESC]\n');
    info.esc.model           = ask_str('Model', info.esc.model);
    info.esc.configuration   = ask_str('Configuration', info.esc.configuration);

    fprintf('\n[Motor]\n');
    info.motor.model         = ask_str('Model', info.motor.model);
    info.motor.kv            = ask_int('KV', info.motor.kv, 0, inf);

    fprintf('\n[Propeller]\n');
    info.propeller.model     = ask_str('Model', info.propeller.model);
    info.propeller.size_in   = ask_str('Size (in)', info.propeller.size_in);
    info.propeller.rotation  = ask_choice('Rotation (CW/CCW)', info.propeller.rotation, ["CW","CCW"]);
    info.propeller.balanced  = ask_bool('Balanced (Y/N)', info.propeller.balanced);

    fprintf('\n[Firmware]\n');
    info.firmware.version    = ask_str('Version', info.firmware.version);

    fprintf('\n[Test]\n');
    info.test_data.filename  = ask_str('Filename', info.test_data.filename);

    % Confirmation time
    info.test_data.date = datetime("now");

    % ---------------- Final confirm loop ----------------
    while true
        fprintf('\n--- Summary ---\n');
        print_summary(info);

        resp = upper(strtrim(input('\nOK? [Y]es / [E]dit again / [C]ancel: ', 's')));
        if isempty(resp) || resp == "Y"
            return;
        elseif resp == "E"
            info = test_info_text(); % simple restart
            return;
        elseif resp == "C"
            error('User cancelled. No test_info returned.');
        else
            fprintf('Please type Y, E, or C.\n');
        end
    end
end

% ---------------- Summary printer ----------------
function print_summary(info)
    fprintf('Battery:  %dS, %d mAh, Charge: %s\n', info.battery.num_cells, info.battery.capacity_mAh, info.battery.charge_state);
    fprintf('ESC:      %s (%s)\n', info.esc.model, info.esc.configuration);
    fprintf('Motor:    %s, %d KV\n', info.motor.model, info.motor.kv);
    fprintf('Prop:     %s, %s, %s, Balanced: %s\n', info.propeller.model, info.propeller.size_in, info.propeller.rotation, ternary(info.propeller.balanced,"Y","N"));
    fprintf('Firmware: %s\n', info.firmware.version);
    fprintf('Filename: %s\n', info.test_data.filename);
end

% ---------------- Helpers ----------------
function v = ask_str(label, def)
    s = input(sprintf('%s [%s]: ', label, string(def)), 's');
    if isempty(strtrim(s))
        v = string(def);
    else
        v = string(strtrim(s));
    end
end

function v = ask_int(label, def, vmin, vmax)
    while true
        s = input(sprintf('%s [%d]: ', label, def), 's');
        if isempty(strtrim(s))
            v = def;
            return;
        end
        x = str2double(strtrim(s));
        if isfinite(x)
            x = round(x);
            if x >= vmin && x <= vmax
                v = x;
                return;
            end
        end
        fprintf('  Invalid value. Enter an integer in [%g, %g].\n', vmin, vmax);
    end
end

function v = ask_choice(label, def, choices)
    def = upper(string(def));
    while true
        s = input(sprintf('%s [%s]: ', label, def), 's');
        if isempty(strtrim(s))
            v = def;
            return;
        end
        s = upper(string(strtrim(s)));
        if any(s == choices)
            v = s;
            return;
        end
        fprintf('  Invalid. Choices: %s\n', strjoin(choices, "/"));
    end
end

function v = ask_bool(label, def)
    defChar = ternary(def,'Y','N');
    while true
        s = input(sprintf('%s [%s]: ', label, defChar), 's');
        if isempty(strtrim(s))
            v = logical(def);
            return;
        end
        t = upper(string(strtrim(s)));
        if any(t == ["Y","YES","TRUE","1"])
            v = true; return;
        elseif any(t == ["N","NO","FALSE","0"])
            v = false; return;
        end
        fprintf('  Please enter Y or N.\n');
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
