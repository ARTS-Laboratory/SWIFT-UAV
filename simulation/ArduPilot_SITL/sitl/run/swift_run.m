function R = swift_run(varargin)
%SWIFT_RUN  Fly a SWIFT ArduPlane SITL session with one command.
%
%   swift_run E                 plan Session E, start ArduPlane (WSL on Windows), start
%                               swift_6dof_sitl, fly the mission, stop, analyse
%   swift_run('E', 'name', value, ...)        same, with options (below)
%
%   WHILE IT FLIES (the Command Window stays free)
%   swift_run status            where the flight is
%   swift_run rtl               tell ArduPlane to return and land (RTL_AUTOLAND)
%   swift_run stop              stop now: Simulink first, then ArduPlane
%
%   OTHER
%   swift_run list              sessions you can fly
%   swift_run analyze E         re-run Session E's analysis on 'out' in the workspace
%   swift_run('E', 'dry', true) do everything except launch (writes the ArduPlane script)
%
%   WHAT IT DOES - the whole RUN ORDER of the session READMEs
%     1  swift_setup, swift_sitl_preflight          (stops if preflight fails)
%     2  the session's planner (swift_sitl_cub_e ...) writes .waypoints + .param
%     3  opens a terminal running sim_vehicle.py with the session's
%        parameter files, the runway location and the two UDP outputs
%     4  presses Run on swift_6dof_sitl
%     5  swift_autofly.py (next to ArduPlane) uploads the mission, checks EVERY
%        parameter against the .param files, waits for GPS/EKF, then
%        FBWA -> arm -> AUTO. Its progress is echoed here.
%     6  after touchdown and auto-disarm: Simulink Stop, then ArduPlane is
%        closed and its .BIN log copied into data\runs\<tag>\
%     7  'out' is kept as out<session> (outE ...) and the session's analysis
%        runs: flight plots, range check, V-tail check, video ...
%     Results: workspace variable swiftRun, files in data\runs\<tag>\.
%
%   OPTIONS
%     'addr'       ''      Address ArduPlane sends to. Blank = automatic: on Windows the
%                          host address seen from WSL (remembered), on macOS/Linux 127.0.0.1.
%     'plan'       {}      extra name/value pairs for the planner, e.g.
%                          swift_run('E', 'plan', {'land_mAh', 4000})
%     'analyze'    true    run the analysis after landing
%     'skip'       {}      analysis steps to skip, e.g. {'video'}
%     'wipe'       true    sim_vehicle.py -w (fresh parameters - keep it on)
%     'map'        true    MAVProxy map and console windows
%     'auto'       true    false = no autofly: type the MAVProxy steps yourself
%     'force'      false   launch even when preflight reports a problem
%     'ardupilot'  '~/ardupilot'   ArduPilot folder (inside WSL on Windows)
%     'distro'     ''      Windows only: WSL distribution (blank = default)
%     'dry'        false   stop before launching (check the plan and script)
%
%   NEW SESSION: copy sessions\swift_session_template.m to
%   sessions\swift_session_F.m and fill it in. Then  swift_run F.
%
%   PLATFORMS
%     Windows   ArduPilot inside WSL (Ubuntu); a console window opens for it
%     macOS     ArduPilot built natively; opens in Terminal.app
%     Linux     ArduPilot built natively; opens in gnome-terminal / x-terminal-emulator
%               / xterm, or runs in the background (log: cache/swift_run/sitl_console.log)
%   NEEDS: ArduPilot built (sim_vehicle.py), MAVProxy (brings pymavlink),
%   python3, swift_autofly.py in this folder.
%
%   Ryan Bilock, ARTS Lab | v0.2 (25 Nov 2026)  v0.2: macOS/Linux, sitl/ subfolders

here = fileparts(mfilename('fullpath'));          % ...\ArduPilot_SITL\sitl\run
addpath(fullfile(here, 'sessions'));
if nargin < 1, help swift_run; return, end
a1 = lower(char(varargin{1}));

switch a1
    case 'list',    list_sessions(here);  return
    case 'status',  show_status();        return
    case 'rtl',     send_cmd(here, 'RTL'); return
    case {'stop', 'abort', 'kill'}, stop_now(here); return
    case {'analyze', 'analyse'}
        assert(nargin >= 2, 'swift_run:args', 'Which session?  swift_run analyze E');
        R = analyze_only(here, char(varargin{2}), varargin(3:end));
        return
end
R = fly(here, char(varargin{1}), varargin(2:end));
end

% =========================================================================
%  FLY
% =========================================================================
function R = fly(here, name, args)
o = options(args);
sitl = fileparts(here);                             % ...\ArduPilot_SITL\sitl
root = fileparts(sitl);                             % ...\ArduPilot_SITL
M    = 'swift_6dof_sitl';
hr   = repmat('=', 1, 74);

if ~isempty(timerfind('Name', 'SWIFT_RUN'))
    error('swift_run:busy', ['A session is already running (swift_run status). ' ...
          'Stop it with  swift_run stop  first.']);
end
C = load_session(name);
fprintf('\n%s\n SWIFT RUN  Session %s  -  %s\n%s\n', hr, C.name, C.title, hr);

% ---- 1 setup + preflight ------------------------------------------------------
addr = find_addr(o);
fprintf(' ArduPlane address: %s\n', addr);
okPF = swift_sitl_preflight(addr);
if ~okPF && ~o.force
    error('swift_run:preflight', ['Preflight found a problem (red lines above). Fix it, or ' ...
          'run anyway with  swift_run(''%s'', ''force'', true)'], C.name);
end
addpath(fullfile(here, 'sessions'));                % swift_setup rebuilt the path

% ---- 2 plan ---------------------------------------------------------------------
S = feval(C.planner, 'addr', addr, C.plan_args{:}, o.plan{:});
fprintf('%s\n swift_run does the RUN ORDER above for you - nothing to type.\n%s\n', hr, hr);

% ---- files ----------------------------------------------------------------------
pf = cellfun(@(f) find_param(sitl, f), C.params, 'UniformOutput', false);
miss = pf(~cellfun(@isfile, pf));
assert(isempty(miss), 'swift_run:nofile', 'Parameter file missing: %s', strjoin(miss, ', '));
wp = S.wp_file;
assert(isfile(wp), 'swift_run:nofile', 'Waypoint file missing: %s', wp);
af = fullfile(here, 'swift_autofly.py');
assert(isfile(af) || ~o.auto, 'swift_run:nofile', 'swift_autofly.py is missing from %s', here);
loc = C.location;
if isempty(loc), loc = [S.airport.lat0, S.airport.lon0, S.airport.elev, S.airport.hdg]; end

runDir = fullfile(root, 'cache', 'swift_run');
if ~isfolder(runDir), mkdir(runDir); end
for f = {'done.json', 'done.json.tmp', 'autofly.log', 'cmd.txt'}
    if isfile(fullfile(runDir, f{1})), delete(fullfile(runDir, f{1})); end
end
outDir = fullfile(root, 'data', 'runs', C.tag);
if ~isfolder(outDir), mkdir(outDir); end

% ---- 3 ArduPlane script --------------------------------------------------------------
apd = strrep(o.ardupilot, '~', '$HOME');
term = find_terminal();                             % how the ArduPlane window opens
L = { '#!/bin/bash'
      '# Written by swift_run.m - rewritten on every run, do not edit.'
      'unset LD_LIBRARY_PATH   # MATLAB''s libraries break python/sim_vehicle when launched from MATLAB'
      sprintf('# Session %s, %s', C.name, datestr(now, 'dd mmm yyyy HH:MM'))
      sprintf('D="%s"', wsl_path(here))
      sprintf('R="%s"', wsl_path(runDir))
      'W="$HOME/swift_run"'
      'mkdir -p "$W"' };
for k = 1:numel(pf)
    L{end+1, 1} = sprintf('cp "%s" "$W/"', wsl_path(pf{k})); %#ok<AGROW>
end
L{end+1} = sprintf('cp "%s" "$W/"', wsl_path(wp));
L{end+1} = sprintf('cd "%s/ArduPlane" || { echo "ArduPilot not found at %s"; sleep 30; exit 1; }', apd, o.ardupilot);
[~, wpn, wpe] = fileparts(wp);
if o.auto
    pl = strjoin(cellfun(@(f) ['"$W/' file_name(f) '"'], pf, 'UniformOutput', false), ' ');
    L{end+1} = 'python3 -c "import pymavlink" 2>/dev/null || echo "pymavlink is missing: pip3 install --user pymavlink MAVProxy"';
    L{end+1} = sprintf(['python3 "$D/swift_autofly.py" --wp "$W/%s%s" --params %s ' ...
                        '--done "$R/done.json" --cmd "$R/cmd.txt" --max-min %g > "$R/autofly.log" 2>&1 &'], ...
                       wpn, wpe, pl, C.max_min);
end
sv = sprintf(['../Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:%s -N%s ' ...
              '--custom-location=%.7f,%.7f,%.1f,%.1f ' ...
              '--add-param-file=../Tools/autotest/models/plane.parm'], ...
             addr, tern(o.wipe, ' -w', ''), loc);
for k = 1:numel(pf)
    sv = [sv sprintf(' --add-param-file="$W/%s"', file_name(pf{k}))]; %#ok<AGROW>
end
sv = [sv sprintf(' --out=udp:%s:14550 --out=udp:127.0.0.1:14551', addr)];
if strcmp(term, 'none')
    sv = [sv ' --mavproxy-args="--non-interactive"'];   % no terminal: MAVProxy without a prompt
elseif o.map
    sv = [sv ' --console --map'];
end
L{end+1} = 'echo "SWIFT SITL - closes by itself when MATLAB is done."';
L{end+1} = sv;
L{end+1} = 'echo; echo "ArduPlane closed."; sleep 3';
sh = write_sh(fullfile(runDir, 'swift_run_sitl.sh'), L);

st = struct('C', C, 'S', S, 'o', o, 'here', here, 'root', root, 'runDir', runDir, ...
            'outDir', outDir, 'M', M, 'phase', 'flying', 'seenRun', false, ...
            't0', tic, 'logPos', 0, 'done', [], 'aborted', false, 'sh', sh);
R = struct('session', C.name, 'tag', C.tag, 'S', S, 'script', sh, 'out_dir', outDir);
assignin('base', 'swiftRun', R);

if o.dry
    fprintf(' DRY RUN - nothing launched. ArduPlane script:\n   %s\n\n', sh);
    type(sh);
    return
end

% ---- 4 launch ArduPlane, then press Run ----------------------------------------------
launch_sh(o, sh, sprintf('SWIFT SITL - Session %s', C.name), term, runDir);
try
    open_system(M);
    if ~strcmp(get_param(M, 'SimulationStatus'), 'stopped')
        error('swift_run:simbusy', '%s is already running. Stop it first.', M);
    end
    set_param(M, 'SimulationCommand', 'start');
catch err
    fprintf(2, ' Could not start %s: %s\n Closing ArduPlane.\n', M, err.message);
    close_sitl(st);
    rethrow(err);
end
fprintf(' %s is running (it waits for ArduPlane - normal).\n', M);
if o.auto
    fprintf(' Autofly will upload the mission, check parameters and take off.\n');
else
    fprintf(' auto = false: type the MAVProxy steps from the session README yourself.\n');
end
fprintf(' Abort:  swift_run rtl  (lands)   |   swift_run stop  (now)\n%s\n', hr);

t = timer('Name', 'SWIFT_RUN', 'Period', 2, 'ExecutionMode', 'fixedSpacing', ...
          'BusyMode', 'drop', 'StartDelay', 2, 'UserData', st, ...
          'TimerFcn', @tick, 'ErrorFcn', @(tt, ~) fprintf(2, ' swift_run timer error\n'));
start(t);
end

% =========================================================================
%  TIMER - runs every 2 s while the flight is on
% =========================================================================
function tick(t, ~)
st = t.UserData;
try
    st = echo_log(st);
    if bdIsLoaded(st.M), sim = get_param(st.M, 'SimulationStatus'); else, sim = 'stopped'; end
    if ~strcmp(sim, 'stopped'), st.seenRun = true; end
    stopped = strcmp(sim, 'stopped') && (st.seenRun || toc(st.t0) > 60);

    switch st.phase
        case 'flying'
            fd = fullfile(st.runDir, 'done.json');
            if isfile(fd)
                st.done = jsondecode(fileread(fd));
                if strcmp(st.done.status, 'landed')
                    fprintf('\n SWIFT RUN  landed - stopping Simulink.\n');
                else
                    fprintf(2, '\n SWIFT RUN  autofly ended: %s\n', st.done.status);
                end
                if ~stopped, set_param(st.M, 'SimulationCommand', 'stop'); end
                st.phase = 'stopping';
            elseif stopped
                fprintf(2, '\n SWIFT RUN  Simulink stopped before the flight ended.\n');
                st.aborted = true;
                st.phase = 'stopping';
            end
        case 'stopping'
            if stopped
                stop(t);
                try
                    finish(st);
                catch err
                    fprintf(2, ' swift_run: finishing failed: %s\n', err.message);
                end
                delete(t);                         % always free swift_run for the next session
                return
            end
    end
    t.UserData = st;
catch err
    fprintf(2, ' swift_run: %s\n', err.message);
    t.UserData = st;
end
end

function st = echo_log(st)
f = fullfile(st.runDir, 'autofly.log');
if ~isfile(f), return, end
s = fileread(f);
if numel(s) > st.logPos
    new = s(st.logPos+1:end);
    cut = find(new == newline, 1, 'last');         % whole lines only
    if isempty(cut), return, end
    new = new(1:cut);
    st.logPos = st.logPos + cut;
    lines = strsplit(strtrim(new), newline);
    for k = 1:numel(lines)
        ln = strtrim(lines{k});
        if contains(ln, {'[FAIL]', 'would not', 'failed', 'missing'})
            fprintf(2, '   %s\n', ln);
        else
            fprintf('   %s\n', ln);
        end
    end
end
end

% =========================================================================
%  FINISH - after Simulink has stopped
% =========================================================================
function finish(st)
hr = repmat('=', 1, 74);
close_sitl(st);
st = echo_log(st);
C = st.C;

if evalin('base', 'exist(''swiftRun'', ''var'')') == 1
    R = evalin('base', 'swiftRun');
else
    R = struct('session', C.name, 'tag', C.tag, 'S', st.S, 'out_dir', st.outDir);
end
R.done = st.done;
R.aborted = st.aborted;
haveOut = evalin('base', 'exist(''out'', ''var'')') == 1;
flew = ~isempty(st.done) && isfield(st.done, 't_takeoff') && ~isempty(st.done.t_takeoff);
if haveOut
    out = evalin('base', 'out');
    vn = ['out' C.name];
    assignin('base', vn, out);
    R.out_var = vn;
    fprintf(' Simulink output kept as  %s\n', vn);
else
    fprintf(2, ' No ''out'' in the workspace - nothing to analyse.\n');
end

if haveOut && st.o.analyze && (flew || st.aborted)
    R.results = run_analysis(C, st.S, out, st.o.skip);
elseif haveOut && ~flew
    fprintf(' The aircraft never took off - analysis skipped.\n');
end

fprintf('%s\n SWIFT RUN  Session %s  finished', hr, C.name);
if ~isempty(st.done)
    fprintf('  |  %s', st.done.status);
    if flew && ~isempty(st.done.t_disarmed)
        fprintf('  |  %.1f min in the air', (st.done.t_disarmed - st.done.t_takeoff) / 60);
    end
end
fprintf('\n Files: %s\n%s\n', st.outDir, hr);
assignin('base', 'swiftRun', R);
end

function res = run_analysis(C, S, out, skip)
res = struct();
for k = 1:size(C.analysis, 1)
    lab = C.analysis{k, 1};
    if any(strcmpi(lab, skip)), fprintf(' [skip] %s\n', lab); continue, end
    fprintf(' [....] %s\n', lab);
    try
        res.(matlab.lang.makeValidName(lab)) = C.analysis{k, 2}(out, S, C.tag);
        fprintf(' [ OK ] %s\n', lab);
    catch err
        fprintf(2, ' [FAIL] %s: %s\n', lab, err.message);
    end
end
end

function close_sitl(st)
%  Close ArduPlane, MAVProxy and autofly, then copy the newest .BIN log.
apd = strrep(st.o.ardupilot, '~', '$HOME');
L = { '#!/bin/bash'
      'unset LD_LIBRARY_PATH'
      '#  Patterns match only the processes swift_run started (not editors or shells).'
      'pkill -f "swift_autofly[.]py --wp"; pkill -f "sim_vehicle[.]py -v ArduPlane"'
      'pkill -f "mavproxy[.]py.*--master"; pkill -x arduplane'
      'sleep 2'
      sprintf('f=$(ls -t "%s/ArduPlane/logs/"*.BIN 2>/dev/null | head -1)', apd)
      sprintf('[ -n "$f" ] && cp "$f" "%s/%s.BIN" && echo "ArduPilot log copied: $f"', ...
              wsl_path(st.outDir), st.C.tag) };
sh = write_sh(fullfile(st.runDir, 'swift_run_close.sh'), L);
[~, msg] = system(sh_cmd(st.o, sh));
msg = strtrim(strrep(msg, char(0), ''));
fprintf(' ArduPlane closed.\n');
if ~isempty(msg), fprintf(' %s\n', msg); end
end

% =========================================================================
%  STATUS / RTL / STOP / ANALYZE / LIST
% =========================================================================
function show_status()
t = timerfind('Name', 'SWIFT_RUN');
if isempty(t), fprintf(' No SWIFT session is running.\n'); return, end
st = t(1).UserData;
fprintf(' Session %s  |  %s  |  %.1f min since launch  |  Simulink %s\n', st.C.name, st.phase, ...
        toc(st.t0) / 60, get_param(st.M, 'SimulationStatus'));
f = fullfile(st.runDir, 'autofly.log');
if isfile(f)
    s = strsplit(strtrim(fileread(f)), newline);
    fprintf('   %s\n', s{max(1, end-4):end});
end
end

function send_cmd(here, c)
t = timerfind('Name', 'SWIFT_RUN');
if isempty(t), fprintf(' No SWIFT session is running.\n'); return, end
f = fullfile(fileparts(fileparts(here)), 'cache', 'swift_run', 'cmd.txt');
fid = fopen(f, 'w');  fprintf(fid, '%s\n', c);  fclose(fid);
fprintf(' Sent %s to autofly.\n', c);
end

function stop_now(here)
t = timerfind('Name', 'SWIFT_RUN');
M = 'swift_6dof_sitl';
if isempty(t)
    %  Nothing tracked: stop what is there and close ArduPlane anyway.
    if bdIsLoaded(M) && ~strcmp(get_param(M, 'SimulationStatus'), 'stopped')
        set_param(M, 'SimulationCommand', 'stop');
    end
    st.o = options({});  st.runDir = fullfile(fileparts(fileparts(here)), 'cache', 'swift_run');
    st.outDir = st.runDir;  st.C.tag = 'last_stopped';
    close_sitl(st);
    return
end
st = t(1).UserData;
fprintf(' Stopping Simulink first, then ArduPlane ...\n');
if ~strcmp(get_param(M, 'SimulationStatus'), 'stopped')
    set_param(M, 'SimulationCommand', 'stop');
end
st.aborted = true;
st.phase = 'stopping';
t(1).UserData = st;
end

function R = analyze_only(here, name, args)
o = options(args);
C = load_session(name);
assert(evalin('base', 'exist(''out'', ''var'')') == 1, 'swift_run:noout', ...
       'No ''out'' in the workspace. Load one first, e.g.  out = outE;');
out = evalin('base', 'out');
R = [];
if evalin('base', 'exist(''swiftRun'', ''var'')') == 1
    R = evalin('base', 'swiftRun');
end
if ~isempty(R) && isfield(R, 'session') && strcmpi(R.session, C.name)
    S = R.S;
else
    %  Rebuild the plan without writing files (needs the planner's 'write' option).
    if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder; P = swift_sitl_params(P);'); end
    S = feval(C.planner, 'write', false, C.plan_args{:});
end
res = run_analysis(C, S, out, o.skip);
R = struct('session', C.name, 'tag', C.tag, 'S', S, 'results', res);
end

function list_sessions(here)
d = dir(fullfile(here, 'sessions', 'swift_session_*.m'));
fprintf('\n SWIFT SITL sessions  (fly one with  swift_run <letter>)\n');
for k = 1:numel(d)
    n = erase(erase(d(k).name, 'swift_session_'), '.m');
    if strcmpi(n, 'template'), continue, end
    try
        C = load_session(n);
        fprintf('   %-3s %-14s %s\n', C.name, C.tag, C.title);
    catch err
        fprintf(2, '   %-3s (could not read: %s)\n', n, err.message);
    end
end
fprintf('\n');
end

% =========================================================================
%  HELPERS
% =========================================================================
function o = options(args)
o = struct('addr', '', 'plan', {{}}, 'analyze', true, 'skip', {{}}, 'wipe', true, ...
           'map', true, 'auto', true, 'force', false, 'ardupilot', '~/ardupilot', ...
           'distro', '', 'dry', false);
assert(mod(numel(args), 2) == 0, 'swift_run:args', 'Options come in name, value pairs.');
for k = 1:2:numel(args)
    f = lower(char(args{k}));
    assert(isfield(o, f), 'swift_run:args', 'Unknown option ''%s''.', f);
    o.(f) = args{k+1};
end
if ischar(o.skip), o.skip = {o.skip}; end
end

function C = load_session(name)
fn = ['swift_session_' upper(name)];
if exist(fn, 'file') ~= 2
    error('swift_run:nosession', ['No session ''%s'' (looked for sessions\\%s.m). ' ...
          'See  swift_run list'], name, fn);
end
C = feval(fn);
need = {'name', 'title', 'tag', 'planner', 'params'};
for k = 1:numel(need)
    assert(isfield(C, need{k}), 'swift_run:session', '%s.m does not set C.%s', fn, need{k});
end
if ~isfield(C, 'plan_args'), C.plan_args = {}; end
if ~isfield(C, 'analysis'),  C.analysis  = cell(0, 2); end
if ~isfield(C, 'location'),  C.location  = []; end
if ~isfield(C, 'max_min'),   C.max_min   = 60; end
end

function a = find_addr(o)
if ~isempty(o.addr)
    a = char(o.addr);  setpref('SWIFT', 'sitl_addr', a);  return
end
if ~ispc
    a = '127.0.0.1';  return                       % ArduPlane and MATLAB on the same machine
end
a = '';
[s, r] = system([wsl_exe(o) ' -e sh -c "wslinfo --networking-mode 2>/dev/null; ip route show default"']);
r = strrep(r, char(0), '');
if s == 0
    if contains(lower(r), 'mirrored')
        a = '127.0.0.1';
    else
        tk = regexp(r, 'default via (\d+\.\d+\.\d+\.\d+)', 'tokens', 'once');
        if ~isempty(tk), a = tk{1}; end
    end
end
if isempty(a) && ispref('SWIFT', 'sitl_addr'), a = getpref('SWIFT', 'sitl_addr'); end
if isempty(a)
    error('swift_run:addr', ['Could not find the Windows address from WSL. Give it once:\n' ...
          '   swift_run(''E'', ''addr'', ''172.20.240.1'')   (it is remembered after that)']);
end
setpref('SWIFT', 'sitl_addr', a);
end

function c = sh_cmd(o, sh)
%  Command that runs a bash script and waits: through WSL on Windows, directly elsewhere.
if ispc
    c = sprintf('%s -e bash -l "%s"', wsl_exe(o), wsl_path(sh));
else
    c = sprintf('bash -l "%s"', sh);
end
end

function t = find_terminal()
%  'windows' | 'mac' | a Linux terminal program | 'none' (run in the background)
if ispc, t = 'windows'; return, end
if ismac, t = 'mac'; return, end
t = 'none';
if isempty(getenv('DISPLAY')) && isempty(getenv('WAYLAND_DISPLAY')), return, end
for c = {'gnome-terminal', 'x-terminal-emulator', 'konsole', 'xterm'}
    [s, ~] = system(['command -v ' c{1} ' >/dev/null 2>&1']);
    if s == 0, t = c{1}; return, end
end
end

function launch_sh(o, sh, title, term, runDir)
%  Start the ArduPlane script without waiting for it.
switch term
    case 'windows'
        system(sprintf('start "%s" %s -e bash -l "%s"', title, wsl_exe(o), wsl_path(sh)));
        msg = 'console window (WSL)';
    case 'mac'
        system(sprintf(['osascript -e ''tell application "Terminal" to do script ' ...
                        '"bash -l \\"%s\\""'' >/dev/null'], sh));
        msg = 'Terminal window';
    case 'gnome-terminal'
        system(sprintf('gnome-terminal --title="%s" -- bash -l "%s" &', title, sh));
        msg = 'terminal window';
    case {'x-terminal-emulator', 'konsole', 'xterm'}
        system(sprintf('%s -e bash -l "%s" &', term, sh));
        msg = 'terminal window';
    otherwise
        lg = fullfile(runDir, 'sitl_console.log');
        system(sprintf('nohup bash -l "%s" > "%s" 2>&1 &', sh, lg));
        msg = sprintf('background (no terminal found) - console log: %s', lg);
end
fprintf(' ArduPlane starting: %s\n', msg);
end

function w = wsl_exe(o)
if isempty(o.distro), w = 'wsl.exe'; else, w = sprintf('wsl.exe -d "%s"', o.distro); end
end

function p = wsl_path(p)
%  C:\<folder>\... -> /mnt/c/<folder>/...   (Windows drive path as WSL sees it)
p = char(p);
if numel(p) > 2 && p(2) == ':'
    p = ['/mnt/' lower(p(1)) strrep(p(3:end), '\', '/')];
end
end

function f = write_sh(f, L)
%  LF line ends - bash refuses CRLF.
fid = fopen(f, 'w');
assert(fid > 0, 'swift_run:write', 'Could not write %s', f);
fprintf(fid, '%s\n', L{:});
fclose(fid);
end

function p = find_param(sitl, f)
%  A session's .param file: the shared ones live in sitl/ardupilot, the
%  session's own (written by its planner) next to the planner in sitl/missions.
p = fullfile(sitl, 'ardupilot', f);
if ~isfile(p), p = fullfile(sitl, 'missions', f); end
end

function n = file_name(f)
[~, b, e] = fileparts(f);  n = [b e];
end

function s = tern(c, a, b), if c, s = a; else, s = b; end, end
