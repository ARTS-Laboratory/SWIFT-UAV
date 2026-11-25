function swift_app
%SWIFT_APP  Point-and-click window for the SWIFT simulation - no commands needed.
%
%   Open this file in MATLAB and press Run (or type  swift_app  in this folder).
%
%   STANDALONE tab   activate the setup, run swift_6dof (with or without the
%                    payload), then battery / lift & drag / V-tail plots
%   ARDUPILOT SITL   pick a session, Check (plan only), Fly, Return & land,
%   tab              Stop, Re-analyse, open the results folder
%
%   It only presses the same buttons you could type: swift_setup, swift_run,
%   swift_battery_plot ... Progress of a flight is shown here and in the
%   Command Window.
%
%   Lives in the top simulation folder, outside both setups, so it never
%   mixes their files: each button first makes the right setup active.
%
%   Needs MATLAB R2020b or newer (uifigure apps).
%
%   Ryan Bilock, ARTS Lab | v0.1 (25 Nov 2026)

root = fileparts(mfilename('fullpath'));
dirS = fullfile(root, 'Standalone');
dirA = fullfile(root, 'ArduPilot_SITL');

old = findall(0, 'Type', 'figure', 'Tag', 'SWIFT_APP');
if ~isempty(old), figure(old(1)); return, end

% ---- look --------------------------------------------------------------------
C.bg   = [0.97 0.97 0.98];   C.panel = [1 1 1];        C.txt  = [0.10 0.10 0.12];
C.dim  = [0.40 0.40 0.45];   C.blue  = [0.11 0.38 0.75]; C.red  = [0.75 0.16 0.16];
C.green = [0.13 0.55 0.25];  C.amber = [0.85 0.55 0.05];

f = uifigure('Name', 'SWIFT UAV Simulation', 'Tag', 'SWIFT_APP', ...
             'Position', [100 100 720 600], 'Color', C.bg);
g = uigridlayout(f, [4 1], 'RowHeight', {44, '1x', 22, 150}, 'Padding', [14 12 14 12], ...
                 'RowSpacing', 8, 'BackgroundColor', C.bg);

hd = uilabel(g, 'Text', 'SWIFT UAV  -  6-DOF simulation', 'FontSize', 20, ...
             'FontWeight', 'bold', 'FontColor', C.txt);
hd.Layout.Row = 1;

tg = uitabgroup(g);  tg.Layout.Row = 2;

% ---- Standalone tab ------------------------------------------------------------------
tS = uitab(tg, 'Title', '  Standalone  ', 'BackgroundColor', C.panel);
gS = uigridlayout(tS, [6 2], 'RowHeight', {36, 22, 36, 22, 36, '1x'}, ...
                  'ColumnWidth', {'1x', '1x'}, 'Padding', [16 14 16 14], 'BackgroundColor', C.panel);
note(gS, 1, 'The airframe on its own: 60 s of trimmed flight on fixed control inputs.');
cbPay = uicheckbox(gS, 'Text', 'With 4.5 kg payload', 'Value', true, 'FontColor', C.txt);
cbPay.Layout.Row = 2;  cbPay.Layout.Column = 1;
b = btn(gS, 3, 1, 'Run the model', C.blue, @runStandalone);  b.Tooltip = 'Activates Standalone, rebuilds P, runs swift_6dof';
btn(gS, 3, 2, 'Open the model', C.dim, @(~,~) withSetup('Standalone', @() open_system('swift_6dof')));
note(gS, 4, 'After a run:');
gP = uigridlayout(gS, [1 3], 'Padding', 0, 'ColumnSpacing', 8, 'BackgroundColor', C.panel);
gP.Layout.Row = 5;  gP.Layout.Column = [1 2];
btn(gP, 1, 1, 'Battery plots',    C.dim, @(~,~) plotOut('swift_battery_plot(out)'));
btn(gP, 1, 2, 'Lift & drag plots', C.dim, @(~,~) plotOut('swift_liftdrag_add(''plot'', out)'));
btn(gP, 1, 3, 'V-tail check',      C.dim, @(~,~) plotOut('swift_vtail_check(out)'));

% ---- ArduPilot SITL tab -----------------------------------------------------------------
tA = uitab(tg, 'Title', '  ArduPilot SITL  ', 'BackgroundColor', C.panel);
gA = uigridlayout(tA, [7 3], 'RowHeight', {36, 30, 26, 36, 36, 36, '1x'}, ...
                  'ColumnWidth', {'1x', '1x', '1x'}, 'Padding', [16 14 16 14], 'BackgroundColor', C.panel);
note(gA, 1, 'ArduPlane flies the model: takeoff, mission, landing, then the plots.');
[items, data] = sessionList();
dd = uidropdown(gA, 'Items', items, 'ItemsData', data, 'FontSize', 13);
dd.Layout.Row = 2;  dd.Layout.Column = [1 3];
cbMap = uicheckbox(gA, 'Text', 'Map + console windows', 'Value', true);   place(cbMap, 3, 1);
cbAna = uicheckbox(gA, 'Text', 'Analyse after landing', 'Value', true);   place(cbAna, 3, 2);
cbVid = uicheckbox(gA, 'Text', 'Make the flight video', 'Value', true);   place(cbVid, 3, 3);
b = btn(gA, 4, 1, 'Check (plan only)', C.dim,   @checkSession); b.Tooltip = 'Plans the mission and shows the ArduPlane script; launches nothing';
b = btn(gA, 4, 2, 'Fly',               C.green, @flySession);   b.Tooltip = 'swift_run <session>: launch, fly, land, analyse';
btn(gA, 4, 3, 'Open results folder', C.dim, @openResults);
b = btn(gA, 5, 1, 'Return & land',     C.amber, @(~,~) sitlCmd('rtl'));  b.Tooltip = 'ArduPlane RTL - lands on the runway';
b = btn(gA, 5, 2, 'Stop now',          C.red,   @(~,~) sitlCmd('stop')); b.Tooltip = 'Stops Simulink, then ArduPlane';
btn(gA, 5, 3, 'Re-analyse last run', C.dim, @reanalyse);
btn(gA, 6, 1, 'Flight status',       C.dim, @(~,~) sitlCmd('status'));
btn(gA, 6, 2, 'Open the model',      C.dim, @(~,~) withSetup('ArduPilot_SITL', @() open_system('swift_6dof_sitl')));
btn(gA, 6, 3, 'README',              C.dim, @(~,~) openFile(fullfile(root, 'README.md')));

% ---- status line + log -----------------------------------------------------------------------
st = uilabel(g, 'Text', 'Ready.', 'FontColor', C.dim);  st.Layout.Row = 3;
lg = uitextarea(g, 'Editable', 'off', 'FontName', monoFont(), 'FontSize', 11, ...
                'Value', {'Pick a tab. Each button makes the right setup active first.'});
lg.Layout.Row = 4;

tm = timer('Name', 'SWIFT_APP_STATUS', 'Period', 2, 'ExecutionMode', 'fixedSpacing', ...
           'BusyMode', 'drop', 'TimerFcn', @(~,~) refreshStatus());
f.CloseRequestFcn = @(~,~) closeApp();
start(tm);

% =========================================================================
%  Standalone
% =========================================================================
    function runStandalone(~, ~)
        withSetup('Standalone', @go);
        function go
            evalin('base', sprintf('clear P; P_with_payload = %d; swift_placeholder;', cbPay.Value));
            open_system('swift_6dof');
            set_param('swift_6dof', 'SimulationCommand', 'start');
            say('swift_6dof running (60 s of flight). When it stops, use the plot buttons.');
        end
    end

    function plotOut(cmd)
        if ~evalin('base', 'exist(''out'', ''var'')')
            uialert(f, 'Run a model first - there is no ''out'' in the workspace yet.', 'No run');
            return
        end
        try
            evalin('base', cmd);  say(['Plotted: ' cmd]);
        catch e
            uialert(f, e.message, 'Plot failed');
        end
    end

% =========================================================================
%  ArduPilot SITL
% =========================================================================
    function checkSession(~, ~)
        s = dd.Value;  if isempty(s), return, end
        withSetup('ArduPilot_SITL', @go);
        function go
            say(sprintf('Checking Session %s (plan only) ...', s));  drawnow;
            txt = evalc(sprintf('swift_run(''%s'', ''dry'', true, ''force'', true, ''map'', %d);', s, cbMap.Value));
            L = strsplit(strtrim(txt), newline);
            lg.Value = [lg.Value; {''}; L(:)];  toBottom();
            say(sprintf('Session %s planned. Nothing was launched.', s));
        end
    end

    function flySession(~, ~)
        s = dd.Value;  if isempty(s), return, end
        if ~isempty(timerfind('Name', 'SWIFT_RUN'))
            uialert(f, 'A flight is already running. Stop it first.', 'Busy');  return
        end
        ok = uiconfirm(f, sprintf(['Fly Session %s?\n\nArduPlane opens in its own window, the model ' ...
            'runs and the aircraft takes off on its own. Progress shows here and in the ' ...
            'Command Window.'], s), 'Fly', 'Options', {'Fly', 'Cancel'}, 'DefaultOption', 1);
        if ~strcmp(ok, 'Fly'), return, end
        withSetup('ArduPilot_SITL', @go);
        function go
            args = {'map', cbMap.Value, 'analyze', cbAna.Value};
            if ~cbVid.Value, args = [args, {'skip', {'video'}}]; end
            say(sprintf('Session %s: preflight, plan and launch ...', s));  drawnow;
            swift_run(s, args{:});
            say(sprintf('Session %s is flying. Use Return & land or Stop if needed.', s));
        end
    end

    function sitlCmd(c)
        if isempty(which('swift_run'))
            say('Activate ArduPilot SITL first (press Fly or Check).');  return
        end
        txt = evalc(sprintf('swift_run(''%s'');', c));
        say(strtrim(strrep(txt, newline, '  ')));
    end

    function reanalyse(~, ~)
        s = dd.Value;  if isempty(s), return, end
        v = ['out' s];
        if evalin('base', sprintf('exist(''%s'', ''var'')', v))
            evalin('base', sprintf('out = %s;', v));
        elseif ~evalin('base', 'exist(''out'', ''var'')')
            uialert(f, sprintf('No run of Session %s in the workspace (%s or out).', s, v), 'No run');
            return
        end
        withSetup('ArduPilot_SITL', @() swift_run('analyze', s));
        say(sprintf('Session %s analysed again - see data/runs.', s));
    end

    function openResults(~, ~)
        s = dd.Value;  d = fullfile(dirA, 'data', 'runs');
        if ~isempty(s)
            try
                fn = sessionFile(s);  tg_ = regexp(fileread(fn), 'C\.tag\s*=\s*''([^'']+)''', 'tokens', 'once');
                if ~isempty(tg_) && isfolder(fullfile(d, tg_{1})), d = fullfile(d, tg_{1}); end
            catch
            end
        end
        openFolder(d);
    end

% =========================================================================
%  helpers
% =========================================================================
    function withSetup(name, fn)
        %  Make 'name' the active setup (if it is not already), then run fn.
        try
            if ~isappdata(0, 'SWIFT_active_setup') || ~strcmp(getappdata(0, 'SWIFT_active_setup'), name)
                d = fullfile(root, name);
                say(sprintf('Activating %s ...', name));  drawnow;
                cd(d);  evalin('base', 'swift_setup');
            end
            fn();
        catch e
            say(['Error: ' e.message]);
            uialert(f, e.message, 'SWIFT');
        end
    end

    function refreshStatus()
        if ~isvalid(f), return, end
        try
            a = '';  if isappdata(0, 'SWIFT_active_setup'), a = getappdata(0, 'SWIFT_active_setup'); end
            t = timerfind('Name', 'SWIFT_RUN');
            if ~isempty(t)
                u = t(1).UserData;  ln = '';
                fl = fullfile(u.runDir, 'autofly.log');
                if isfile(fl)
                    L = strsplit(strtrim(fileread(fl)), newline);  ln = strtrim(L{end});
                end
                st.Text = sprintf('Session %s  |  %s  |  %.1f min  |  %s', u.C.name, u.phase, toc(u.t0)/60, ln);
                st.FontColor = C.green;
            elseif ~isempty(a)
                st.Text = sprintf('Active setup: %s.  Ready.', a);  st.FontColor = C.dim;
            end
        catch
        end
    end

    function say(m)
        lg.Value = [lg.Value; {[datestr(now, 'HH:MM:SS') '  ' m]}];
        toBottom();
    end

    function toBottom()
        try, scroll(lg, 'bottom'); catch, end          % scroll() needs R2020b+
    end

    function closeApp()
        try, stop(tm); delete(tm); catch, end
        delete(f);
    end

    function [items, data] = sessionList()
        d = dir(fullfile(dirA, 'sitl', 'run', 'sessions', 'swift_session_*.m'));
        items = {};  data = {};
        for k = 1:numel(d)
            n = erase(erase(d(k).name, 'swift_session_'), '.m');
            if strcmpi(n, 'template'), continue, end
            ttl = regexp(fileread(fullfile(d(k).folder, d(k).name)), 'C\.title\s*=\s*''([^'']*)''', 'tokens', 'once');
            if isempty(ttl), ttl = {''}; end
            items{end+1} = sprintf('Session %s  -  %s', n, ttl{1});  %#ok<AGROW>
            data{end+1}  = n;                                         %#ok<AGROW>
        end
        if isempty(items), items = {'(no sessions found)'}; data = {''}; end
    end

    function fn = sessionFile(s)
        fn = fullfile(dirA, 'sitl', 'run', 'sessions', ['swift_session_' s '.m']);
    end
end

% =========================================================================
%  layout helpers (no shared state)
% =========================================================================
function b = btn(parent, r, c, txt, col, cb)
b = uibutton(parent, 'push', 'Text', txt, 'FontSize', 13, 'FontWeight', 'bold', ...
             'FontColor', [1 1 1], 'BackgroundColor', col, 'ButtonPushedFcn', cb);
b.Layout.Row = r;  b.Layout.Column = c;
end

function l = note(parent, r, txt)
l = uilabel(parent, 'Text', txt, 'FontColor', [0.25 0.25 0.30], 'FontSize', 13, 'WordWrap', 'on');
l.Layout.Row = r;  l.Layout.Column = [1 numel(parent.ColumnWidth)];
end

function place(h, r, c)
h.Layout.Row = r;  h.Layout.Column = c;
end

function openFolder(d)
if ~isfolder(d), mkdir(d); end
if ispc, winopen(d); elseif ismac, system(sprintf('open "%s"', d)); else, system(sprintf('xdg-open "%s" &', d)); end
end

function openFile(p)
if ispc, winopen(p); elseif ismac, system(sprintf('open "%s"', p)); else, system(sprintf('xdg-open "%s" &', p)); end
end

function m = monoFont()
m = 'Consolas';
if ~any(strcmpi(listfonts, m)), m = 'FixedWidth'; end
end
