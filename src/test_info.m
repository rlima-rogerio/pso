%% PSO / e-PSA Testbench Info Collector (LANDSCAPE UI)
function test_info = test_info()

    %% --------- Default values ----------
    test_info = struct( ...
        'battery', struct('num_cells', 4, 'capacity_mAh', 2200, 'charge_state', "Full"), ...
        'esc',     struct('model', "ESC-Model", 'configuration', "Default"), ...
        'motor',   struct('model', "Motor-Model", 'kv', 650), ...
        'propeller', struct('model', "Prop-Model", 'size_in', "15x7.5", 'rotation', "CW", 'balanced', true), ...
        'test_data', struct('date', datetime("now"), 'filename', ""), ...
        'firmware', struct('version', "v0.0.0") ...
    );

    %% --------- Timestamp filename ----------
    ts = datetime("now");
    ts_str = datestr(ts, "yyyy-mm-dd_HH-MM-SS");
    test_info.test_data.filename = "pso_test_" + string(ts_str);

    %% --------- Prompts + defaults ----------
    prompt = { ...
        '# Cells', ...              % Battery
        'Capacity (mAh)', ...       % Battery
        'Charge', ...               % Battery
        'Model', ...                % ESC
        'Configuration', ...        % ESC
        'Model', ...                % Motor
        'KV', ...                   % Motor
        'Model', ...                % Propeller
        'Size (in)', ...            % Propeller
        'Rotation (CW/CCW)', ...    % Propeller
        'Balanced (Y/N)', ...       % Propeller
        'Version', ...              % Firmware
        'Filename' ...              % Test
    };

    defaults = { ...
        num2str(test_info.battery.num_cells), ...
        num2str(test_info.battery.capacity_mAh), ...
        char(test_info.battery.charge_state), ...
        char(test_info.esc.model), ...
        char(test_info.esc.configuration), ...
        char(test_info.motor.model), ...
        num2str(test_info.motor.kv), ...
        char(test_info.propeller.model), ...
        char(test_info.propeller.size_in), ...
        char(test_info.propeller.rotation), ...
        ternary(test_info.propeller.balanced, 'Y', 'N'), ...
        char(test_info.firmware.version), ...
        char(test_info.test_data.filename) ...
    };

    dlg_title = 'Confirm Testbench Information (e-PSA)';

    %% --------- LANDSCAPE dialog (replaces inputdlg) ----------
    % answer = inputdlg_landscape(prompt, defaults, dlg_title, 3); % 3 columns
    answer = inputdlg_landscape_grouped(prompt, defaults, dlg_title); % 3 columns

    if isempty(answer)
        error('User cancelled. No test_info returned.');
    end

    %% --------- Parse results ----------
    test_info.battery.num_cells    = max(1, round(str2double(answer{1})));
    test_info.battery.capacity_mAh = max(0, round(str2double(answer{2})));
    test_info.battery.charge_state = string(strtrim(answer{3}));

    test_info.esc.model           = string(strtrim(answer{4}));
    test_info.esc.configuration   = string(strtrim(answer{5}));

    test_info.motor.model         = string(strtrim(answer{6}));
    test_info.motor.kv            = max(0, round(str2double(answer{7})));

    test_info.propeller.model     = string(strtrim(answer{8}));
    test_info.propeller.size_in   = string(strtrim(answer{9}));

    rot = upper(string(strtrim(answer{10})));
    if rot ~= "CW" && rot ~= "CCW"
        rot = "CW";
    end
    test_info.propeller.rotation  = rot;

    bal = upper(string(strtrim(answer{11})));
    test_info.propeller.balanced  = (bal == "Y" || bal == "YES" || bal == "TRUE" || bal == "1");

    test_info.firmware.version    = string(strtrim(answer{12}));

    test_info.test_data.date      = datetime("now");
    test_info.test_data.filename  = string(strtrim(answer{13}));

    %% --------- Final confirmation dialog ----------
    summary = sprintf([ ...
        "Battery: %dS, %d mAh, Charge: %s\n" + ...
        "ESC: %s (%s)\n" + ...
        "Motor: %s, %d KV\n" + ...
        "Prop: %s, %s, %s, Balanced: %s\n" + ...
        "Firmware: %s\n" + ...
        "Date: %s\n" + ...
        "Filename: %s\n"], ...
        test_info.battery.num_cells, ...
        test_info.battery.capacity_mAh, ...
        test_info.battery.charge_state, ...
        test_info.esc.model, ...
        test_info.esc.configuration, ...
        test_info.motor.model, ...
        test_info.motor.kv, ...
        test_info.propeller.model, ...
        test_info.propeller.size_in, ...
        test_info.propeller.rotation, ...
        ternary(test_info.propeller.balanced, "Y", "N"), ...
        test_info.firmware.version, ...
        datestr(test_info.test_data.date, "yyyy-mm-dd HH:MM:SS"), ...
        test_info.test_data.filename);

    % choice = questdlg(summary, 'Confirm?', 'OK', 'Edit', 'Cancel', 'OK');
    % 
    % switch choice
    %     case 'OK'
    %         % proceed
    %     case 'Edit'
    %         test_info = test_info(); % restart once
    %     otherwise
    %         error('User cancelled. No test_info returned.');
    % end
end

%% ---------- LANDSCAPE input dialog helper ----------
function answer = inputdlg_landscape(prompt, defaults, titleStr, nCols)
% Returns cell array of strings like inputdlg.
% Uses uifigure + uigridlayout; resizable and "landscape".

    if nargin < 4 || isempty(nCols), nCols = 3; end

    n = numel(prompt);
    if numel(defaults) ~= n
        error('prompt and defaults must have same length.');
    end

    fig = uifigure('Name', titleStr, 'Position', [100 100 680 480]);
    fig.Resize = 'on';

    outer = uigridlayout(fig, [2 1]);
    outer.RowHeight = {'1x', 44};
    outer.ColumnWidth = {'1x'};
    outer.Padding = [10 10 10 10];

    % Scrollable panel for many fields
    sp = uipanel(outer, 'Scrollable', 'on');
    sp.Layout.Row = 1;
    sp.Layout.Column = 1;

    % Grid inside scrollable panel
    rowsPerCol = ceil(n / nCols);
    nRows = rowsPerCol; % rows in each column block

    gl = uigridlayout(sp, [nRows, 2*nCols]); % (Label, Edit) per column
    gl.ColumnWidth = repmat({200, '1x'}, 1, nCols);
    gl.RowHeight = repmat({28}, 1, nRows);
    gl.Padding = [5 5 5 5];
    gl.RowSpacing = 8;
    gl.ColumnSpacing = 16;

    edits = cell(n,1);

    for k = 1:n
        colIdx = floor((k-1)/rowsPerCol) + 1;         % 1..nCols
        rowIdx = mod((k-1), rowsPerCol) + 1;          % 1..rowsPerCol
        baseCol = 2*(colIdx-1);

        lbl = uilabel(gl, 'Text', prompt{k}, 'HorizontalAlignment', 'right');
        lbl.Layout.Row = rowIdx;
        lbl.Layout.Column = baseCol + 1;

        ed = uieditfield(gl, 'text', 'Value', string(defaults{k}));
        ed.Layout.Row = rowIdx;
        ed.Layout.Column = baseCol + 2;

        edits{k} = ed;
    end

    % Buttons row
    btnGrid = uigridlayout(outer, [1 3]);
    btnGrid.Layout.Row = 2;
    btnGrid.ColumnWidth = {'1x', 110, 110};
    btnGrid.Padding = [0 0 0 0];

    uilabel(btnGrid, 'Text', ''); % spacer

    answer = []; % default cancel

    uibutton(btnGrid, 'Text', 'Cancel', ...
        'ButtonPushedFcn', @(~,~) onCancel());

    uibutton(btnGrid, 'Text', 'OK', ...
        'ButtonPushedFcn', @(~,~) onOK());

    uiwait(fig);

    function onOK()
        out = cell(n,1);
        for i = 1:n
            out{i} = char(edits{i}.Value);
        end
        answer = out;
        uiresume(fig);
        delete(fig);
    end

    function onCancel()
        answer = [];
        uiresume(fig);
        delete(fig);
    end
end


function answer = inputdlg_landscape_grouped(prompt, defaults, titleStr)
% Grouped, resizable, scrollable UI dialog (landscape) with 2 columns of categories
% Returns cell array of strings like inputdlg.

    n = numel(prompt);
    if numel(defaults) ~= n
        error('prompt and defaults must have same length.');
    end

    fig = uifigure('Name', titleStr, 'Position', [100 100 800 450]);
    fig.Resize = 'on';

    outer = uigridlayout(fig, [2 1]);
    outer.RowHeight = {'1x', 44};
    outer.ColumnWidth = {'1x'};
    outer.Padding = [2 2 2 2];

    sp = uipanel(outer, 'Scrollable', 'on');
    sp.Layout.Row = 1;

    % --- Two-column category container inside scrollable panel ---
    catCols = 2;
    catGrid = uigridlayout(sp, [1, catCols]);  % we'll grow rows dynamically
    catGrid.ColumnWidth = {'1x','1x'};
    catGrid.RowHeight = {'fit'};
    catGrid.Padding = [2 2 2 2];
    catGrid.RowSpacing = 6;
    catGrid.ColumnSpacing = 6;

    % ---- Define groups (by index) ----
    idx.battery  = 1:3;
    idx.esc      = 4:5;
    idx.motor    = 6:7;
    idx.prop     = 8:11;
    idx.firmware = 12;
    idx.test     = 13;

    groups = { ...
        struct('name','Battery',  'indices', idx.battery), ...
        struct('name','ESC',      'indices', idx.esc), ...
        struct('name','Motor',    'indices', idx.motor), ...
        struct('name','Propeller','indices', idx.prop), ...
        struct('name','Firmware', 'indices', idx.firmware), ...
        struct('name','Test',     'indices', idx.test) ...
    };

    edits = cell(n,1);

    % Place each category panel into catGrid (2 columns), row-major
    for g = 1:numel(groups)
        row = ceil(g / catCols);
        col = mod(g-1, catCols) + 1;

        % grow rows if needed
        if row > numel(catGrid.RowHeight)
            catGrid.RowHeight{end+1} = 'fit';
        end

        inds = groups{g}.indices(:)';

        p = uipanel(catGrid, 'Title', groups{g}.name);
        p.Layout.Row = row;
        p.Layout.Column = col;

        gl = uigridlayout(p, [numel(inds), 2]);
        gl.ColumnWidth = {120, '1x'};
        gl.RowHeight = repmat({24}, 1, numel(inds));
        gl.Padding = [4 6 4 6];
        gl.RowSpacing = 6;
        gl.ColumnSpacing = 6;

        for r = 1:numel(inds)
            k = inds(r);

            lbl = uilabel(gl, 'Text', prompt{k}, 'HorizontalAlignment', 'right');
            lbl.Layout.Row = r; lbl.Layout.Column = 1;

            ed = uieditfield(gl, 'text', 'Value', string(defaults{k}));
            ed.Layout.Row = r; ed.Layout.Column = 2;

            edits{k} = ed;
        end
    end

    % Buttons row
    btnGrid = uigridlayout(outer, [1 3]);
    btnGrid.Layout.Row = 2;
    btnGrid.ColumnWidth = {'1x', 110, 110};
    btnGrid.Padding = [0 0 0 0];

    uilabel(btnGrid, 'Text', ''); % spacer

    answer = []; % default cancel

    uibutton(btnGrid, 'Text', 'Cancel', ...
        'ButtonPushedFcn', @(~,~) onCancel());

    uibutton(btnGrid, 'Text', 'OK', ...
        'ButtonPushedFcn', @(~,~) onOK());

    uiwait(fig);

    function onOK()
        out = cell(n,1);
        for i = 1:n
            out{i} = char(edits{i}.Value);
        end
        answer = out;
        uiresume(fig);
        delete(fig);
    end

    function onCancel()
        answer = [];
        uiresume(fig);
        delete(fig);
    end
end



%% ---- small helper ----
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
