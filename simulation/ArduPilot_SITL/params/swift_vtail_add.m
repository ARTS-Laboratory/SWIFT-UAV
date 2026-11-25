function swift_vtail_add(action)
%SWIFT_VTAIL_ADD  Put the V-tail into the ACTIVE setup (Standalone or ArduPilot_SITL).
%
%   swift_vtail_add            install: parameters, model, ArduPlane file
%   swift_vtail_add params     rewrite swift_sitl_vtail.param only (SITL)
%
%   Run it ONCE IN EACH FOLDER, after that folder's swift_setup:
%       cd('...\Simulink\Standalone');       swift_setup;   swift_vtail_add
%       cd('...\Simulink\ArduPilot_SITL');   swift_setup;   swift_vtail_add
%   It works on whichever model swift_setup made active, and only on that
%   folder's own files. Running it again changes nothing.
%
%   NEEDS, IN THE ACTIVE FOLDER'S params\
%     swift_vtail.m (and this file). Each folder gets its own copy.
%
%   WHAT IT CHANGES
%     1. swift_placeholder.m  One line after section 4, "P = swift_vtail(P);",
%        and the elevator/rudder travel limits now follow the ruddervator
%        travel P.tail.drv_max. The old file is kept as
%        swift_placeholder_<date>.bak next to it.
%     2. The model  A "V-tail limit" block inside Controls, just before the
%        delta_e and delta_r outputs: mixes to the two ruddervators, stops
%        each at +/- P.tail.drv_max, and un-mixes. Ruddervator angles are
%        logged as out_rv = [left; right]. The model file is backed up
%        first with a timestamp.
%     3. swift_6dof_sitl only  PWM decode now reads the LEFT ruddervator on
%        channel P.sitl.ch(2) and the RIGHT on P.sitl.ch(3) and un-mixes
%        them, because ArduPlane sends V-tail outputs:
%            left  = (yaw + pitch) * MIXING_GAIN
%            right = (-yaw + pitch) * MIXING_GAIN
%        P.sitl.sgn(2) and (3) keep their meaning (elevator sign, rudder
%        sign), so the signs you already confirmed still hold.
%     4. swift_6dof_sitl only  writes sitl\ardupilot\swift_sitl_vtail.param:
%            SERVO2_FUNCTION 79   SERVO4_FUNCTION 80   MIXING_GAIN
%        Load it in ArduPlane AFTER your other parameter files.
%
%   Ryan Bilock, ARTS Lab | V-tail | v0.1

if nargin < 1, action = 'install'; end

% --- which setup is active --------------------------------------------------
hasSA   = ~isempty(which('swift_6dof.slx'));
hasSITL = ~isempty(which('swift_6dof_sitl.slx'));
if hasSA == hasSITL
    error('swift_vtail_add:setup', ...
        ['Expected exactly one of swift_6dof.slx / swift_6dof_sitl.slx on the path.\n' ...
         'cd into Standalone or ArduPilot_SITL and run swift_setup first.']);
end
if hasSITL, M = 'swift_6dof_sitl'; else, M = 'swift_6dof'; end
if isempty(which('swift_vtail.m'))
    error('swift_vtail_add:missing', 'swift_vtail.m is not on the path. Copy it into this folder''s params\\.');
end

switch lower(char(action))
    case 'params'
        if ~hasSITL, error('swift_vtail_add:params', 'The parameter file is for the ArduPilot_SITL setup only.'); end
        refresh_P(hasSITL);
        write_params();
        return
    case 'install'
    otherwise
        error('swift_vtail_add:action', 'Use swift_vtail_add or swift_vtail_add params.');
end

fprintf('\nV-tail install into %s\n', M);

% --- 1: swift_placeholder.m --------------------------------------------------
patch_placeholder();
refresh_P(hasSITL);
P = evalin('base', 'P');
if ~isfield(P, 'tail')
    error('swift_vtail_add:notail', 'swift_placeholder ran but P.tail is missing - the swift_vtail call is not in effect.');
end

% --- 2: the model ------------------------------------------------------------
mdlFile = which([M '.slx']);
if bdIsLoaded(M) && strcmp(get_param(M, 'Dirty'), 'on')
    error('swift_vtail_add:dirty', '%s has unsaved changes. Save it (Ctrl+S) and run again.', M);
end
load_system(mdlFile);
CTRL = [M '/Controls'];
assert_block(CTRL);

changed = false;
if getSimulinkBlockHandle([CTRL '/V-tail limit']) == -1
    bak = backup_file(mdlFile);
    say('Backup ', '%s', bak);
    add_limit(CTRL, hasSITL);
    changed = true;
    say('Model  ', 'Controls/V-tail limit added: ruddervators stop at +/- P.tail.drv_max, logged as out_rv');
else
    say('Model  ', 'Controls/V-tail limit is already there - left alone');
end

% --- 3: SITL PWM decode --------------------------------------------------------
if hasSITL
    DEC = [CTRL '/PWM decode'];
    if getSimulinkBlockHandle(DEC) == -1
        warning('swift_vtail_add:decode', ...
            '%s not found, so the PWM decode was not changed. ArduPlane must stay on elevator/rudder outputs.', DEC);
    elseif getSimulinkBlockHandle([DEC '/V-tail demix']) == -1
        if ~changed
            bak = backup_file(mdlFile);
            say('Backup ', '%s', bak);
        end
        patch_decode(DEC);
        changed = true;
        say('Decode ', 'ch %d = left ruddervator, ch %d = right ruddervator, then un-mixed to delta_e, delta_r', ...
            P.sitl.ch(2), P.sitl.ch(3));
    else
        say('Decode ', 'PWM decode/V-tail demix is already there - left alone');
    end
end

if changed
    save_system(M);
    try
        set_param(M, 'SimulationCommand', 'update');
        say('Check  ', 'Update Diagram (Ctrl+D) passed');
    catch err
        fprintf(2, '\nUpdate Diagram failed - the model is saved, fix this and press Ctrl+D:\n%s\n', ...
            err.getReport('basic'));
    end
end

% --- 4: ArduPlane parameters ---------------------------------------------------
if hasSITL, write_params(); end

fprintf('\nDone with %s.\n', M);
if hasSITL
    fprintf(['  Next: load swift_sitl_vtail.param in ArduPlane after your other files, then check in\n' ...
             '  MANUAL: stick back -> delta_e moves the same way as before; right rudder -> delta_r\n' ...
             '  the same way as before. One axis wrong: swap 79 and 80. Both wrong: flip that sign in\n' ...
             '  P.sitl.sgn (swift_sitl_params.m).\n\n']);
else
    fprintf('  Next: run the model. out_rv shows the two ruddervator angles.\n\n');
end
end

% =========================================================================
%  swift_placeholder.m
% =========================================================================
function patch_placeholder()
f = which('swift_placeholder.m');
if isempty(f), error('swift_vtail_add:placeholder', 'swift_placeholder.m is not on the path.'); end
txt = fileread(f);
if contains(txt, sprintf('\r\n')), nl = sprintf('\r\n'); else, nl = newline; end

orig = txt;

% the call, straight after the aerodynamic derivatives
if ~contains(txt, 'swift_vtail(')
    anchor = '%% 5 | Propulsion';
    i = strfind(txt, anchor);
    if isempty(i)
        error('swift_vtail_add:anchor', ...
            ['Could not find the line starting "%s" in\n  %s\n' ...
             'Add this line by hand after the last P.aero line and before the trim (section 8):\n' ...
             '    P = swift_vtail(P);\nthen run swift_vtail_add again.'], anchor, f);
    end
    ins = ['%  V-tail: overwrite the tail-driven derivatives from the CAD geometry and' nl ...
           '%  define the ruddervator mixing (swift_vtail.m).' nl ...
           'P = swift_vtail(P);' nl nl];
    txt = [txt(1:i(1)-1) ins txt(i(1):end)];
end

% elevator and rudder travel are now one ruddervator travel
txt = regexprep(txt, '(P\.act\.de_max\s*=\s*)deg2rad\(\s*[\d\.]+\s*\)', '$1P.tail.drv_max');
txt = regexprep(txt, '(P\.act\.dr_max\s*=\s*)deg2rad\(\s*[\d\.]+\s*\)', '$1P.tail.drv_max');

if strcmp(txt, orig)
    say('Params ', 'swift_placeholder.m already calls swift_vtail - left alone');
    return
end
bak = backup_file(f);
fid = fopen(f, 'w', 'n', 'UTF-8');
assert(fid > 0, 'Could not write %s', f);
fwrite(fid, txt, 'char');
fclose(fid);
rehash;
say('Params ', 'swift_placeholder.m now calls swift_vtail (old file: %s)', bak);
end

function refresh_P(hasSITL)
evalin('base', 'swift_placeholder;');
if hasSITL, evalin('base', 'P = swift_sitl_params(P);'); end
end

% =========================================================================
%  Controls: mix, limit, un-mix
% =========================================================================
function add_limit(CTRL, hasSITL)
oe = [CTRL '/delta_e'];   orr = [CTRL '/delta_r'];   mx = [CTRL '/Mux1'];
assert_block(oe);  assert_block(orr);

[se, sBe, sPe] = driver(oe, 1);
[sr, sBr, sPr] = driver(orr, 1);

% every Mux1 input fed by the same two signals gets the limited ones instead
muxE = [];  muxR = [];
if getSimulinkBlockHandle(mx) ~= -1
    pc = get_param(mx, 'PortConnectivity');
    for k = 1:numel(pc)
        n = str2double(pc(k).Type);
        if isnan(n) || isempty(pc(k).SrcBlock) || isequal(pc(k).SrcBlock, -1), continue, end
        if pc(k).SrcBlock == sBe && pc(k).SrcPort == sPe, muxE(end+1) = n; end %#ok<AGROW>
        if pc(k).SrcBlock == sBr && pc(k).SrcPort == sPr, muxR(end+1) = n; end %#ok<AGROW>
    end
end

delete_line(CTRL, se, inport(oe, 1));
delete_line(CTRL, sr, inport(orr, 1));
for n = muxE, delete_line(CTRL, se, inport(mx, n)); end
for n = muxR, delete_line(CTRL, sr, inport(mx, n)); end

p  = get_param(oe, 'Position');
VT = [CTRL '/V-tail limit'];
add('built-in/Subsystem', VT, [p(1)-170 p(2)-25 p(1)-70 p(2)+45]);

add('simulink/Sources/In1', [VT '/de_cmd'], [30 43 60 57]);
add('simulink/Sources/In1', [VT '/dr_cmd'], [30 103 60 117]);
add('simulink/Signal Routing/Mux', [VT '/cmd'], [110 35 115 125], 'Inputs', '2', 'DisplayOption', 'bar');
add('simulink/Math Operations/Gain', [VT '/mix'], [160 65 250 95], ...
    'Gain', 'P.tail.mix', 'Multiplication', 'Matrix(K*u)');
add('simulink/Discontinuities/Saturation', [VT '/ruddervator travel'], [300 65 340 95], ...
    'LowerLimit', '-P.tail.drv_max', 'UpperLimit', 'P.tail.drv_max');
add('simulink/Math Operations/Gain', [VT '/demix'], [400 65 490 95], ...
    'Gain', 'P.tail.demix', 'Multiplication', 'Matrix(K*u)');
add('simulink/Signal Routing/Demux', [VT '/out'], [540 35 545 125], 'Outputs', '2', 'DisplayOption', 'bar');
add('simulink/Sinks/Out1', [VT '/delta_e'], [600 43 630 57]);
add('simulink/Sinks/Out1', [VT '/delta_r'], [600 103 630 117]);

% log the ruddervators the same way the other signal taps are logged
M   = bdroot(CTRL);
fmt = 'Timeseries';
tw0 = find_system(M, 'BlockType', 'ToWorkspace');
if ~isempty(tw0), fmt = get_param(tw0{1}, 'SaveFormat'); end
add('simulink/Sinks/To Workspace', [VT '/out_rv'], [400 150 480 180], ...
    'VariableName', 'out_rv', 'SaveFormat', fmt);
if hasSITL, set_param([VT '/out_rv'], 'Decimation', 'P.sitl.log_decim'); end

wire(VT, outport([VT '/de_cmd'], 1), inport([VT '/cmd'], 1));
wire(VT, outport([VT '/dr_cmd'], 1), inport([VT '/cmd'], 2));
wire(VT, outport([VT '/cmd'], 1), inport([VT '/mix'], 1));
wire(VT, outport([VT '/mix'], 1), inport([VT '/ruddervator travel'], 1));
wire(VT, outport([VT '/ruddervator travel'], 1), inport([VT '/demix'], 1));
wire(VT, outport([VT '/ruddervator travel'], 1), inport([VT '/out_rv'], 1));
wire(VT, outport([VT '/demix'], 1), inport([VT '/out'], 1));
wire(VT, outport([VT '/out'], 1), inport([VT '/delta_e'], 1));
wire(VT, outport([VT '/out'], 2), inport([VT '/delta_r'], 1));

wire(CTRL, se, inport(VT, 1));
wire(CTRL, sr, inport(VT, 2));
wire(CTRL, outport(VT, 1), inport(oe, 1));
wire(CTRL, outport(VT, 2), inport(orr, 1));
for n = muxE, wire(CTRL, outport(VT, 1), inport(mx, n)); end
for n = muxR, wire(CTRL, outport(VT, 2), inport(mx, n)); end
end

% =========================================================================
%  SITL PWM decode: two ruddervator channels -> delta_e, delta_r
% =========================================================================
function patch_decode(DEC)
nm = {'delta_e', 'delta_r'};
for k = 1:2
    assert_block([DEC '/scale ' nm{k}]);
    assert_block([DEC '/limit ' nm{k}]);
    assert_block([DEC '/' nm{k}]);
    % raw ruddervator angle: no sign here, the signs go in the un-mix
    set_param([DEC '/scale ' nm{k}], 'Gain', 'P.tail.drv_max/500');
    set_param([DEC '/limit ' nm{k}], 'LowerLimit', '-P.tail.drv_max', 'UpperLimit', 'P.tail.drv_max');
    delete_line(DEC, outport([DEC '/limit ' nm{k}], 1), inport([DEC '/' nm{k}], 1));
end

p = get_param([DEC '/limit delta_e'], 'Position');
x = p(3) + 40;   y = p(2);
add('simulink/Signal Routing/Mux', [DEC '/rv'], [x y x+5 y+120], 'Inputs', '2', 'DisplayOption', 'bar');
%  ArduPlane: left = yaw + pitch, right = -yaw + pitch, so
%  pitch = (L + R)/2 and yaw = (L - R)/2, each with its own sign.
add('simulink/Math Operations/Gain', [DEC '/V-tail demix'], [x+45 y+40 x+175 y+80], ...
    'Gain', '0.5*[P.sitl.sgn(2) P.sitl.sgn(2); P.sitl.sgn(3) -P.sitl.sgn(3)]', ...
    'Multiplication', 'Matrix(K*u)');
add('simulink/Signal Routing/Demux', [DEC '/er'], [x+215 y x+220 y+120], 'Outputs', '2', 'DisplayOption', 'bar');

% move the two Outports clear of the new blocks
for k = 1:2
    q = get_param([DEC '/' nm{k}], 'Position');
    w = q(3) - q(1);
    set_param([DEC '/' nm{k}], 'Position', [x+270 q(2) x+270+w q(4)]);
end

wire(DEC, outport([DEC '/limit delta_e'], 1), inport([DEC '/rv'], 1));     % left
wire(DEC, outport([DEC '/limit delta_r'], 1), inport([DEC '/rv'], 2));     % right
wire(DEC, outport([DEC '/rv'], 1), inport([DEC '/V-tail demix'], 1));
wire(DEC, outport([DEC '/V-tail demix'], 1), inport([DEC '/er'], 1));
wire(DEC, outport([DEC '/er'], 1), inport([DEC '/delta_e'], 1));
wire(DEC, outport([DEC '/er'], 2), inport([DEC '/delta_r'], 1));
end

% =========================================================================
%  ArduPlane parameters
% =========================================================================
function write_params()
P = evalin('base', 'P');
folder = fullfile(fileparts(fileparts(which('ap_json_step.m'))), 'ardupilot');   % sitl/bridge -> sitl/ardupilot
if isempty(which('ap_json_step.m')), folder = ''; end
if isempty(folder), folder = fileparts(mfilename('fullpath')); end
f = fullfile(folder, 'swift_sitl_vtail.param');

prm = { ...
  sprintf('SERVO%d_FUNCTION', P.sitl.ch(2)), 79,              'left V-tail (was 19, elevator)';
  sprintf('SERVO%d_FUNCTION', P.sitl.ch(3)), 80,              'right V-tail (was 21, rudder)';
  'MIXING_GAIN',                             P.tail.mix_gain, 'P.tail.mix_gain; ArduPlane default 0.5'};

fid = fopen(f, 'w');
assert(fid > 0, 'Could not write %s', f);
fprintf(fid, '# SWIFT UAV - ArduPlane SITL V-tail outputs. Load AFTER the other parameter files.\n');
fprintf(fid, '# Written by swift_vtail_add, %s\n', datestr(now, 'yyyy-mm-dd HH:MM'));
for k = 1:size(prm, 1)
    fprintf(fid, '%s,%g\n', prm{k,1}, prm{k,2});
end
fclose(fid);

say('ArduPlane', 'wrote %s', f);
for k = 1:size(prm, 1)
    fprintf('           param set %-16s %-4g  %% %s\n', prm{k,1}, prm{k,2}, prm{k,3});
end
end

% =========================================================================
%  SMALL HELPERS
% =========================================================================
function [p, srcBlk, srcPort] = driver(blk, k)
% Output port that feeds input k of blk (works on branched lines).
pc = get_param(blk, 'PortConnectivity');
srcBlk = [];  srcPort = [];
for i = 1:numel(pc)
    if strcmp(pc(i).Type, num2str(k)) && ~isempty(pc(i).SrcBlock) && ~isequal(pc(i).SrcBlock, -1)
        srcBlk = pc(i).SrcBlock;  srcPort = pc(i).SrcPort;
    end
end
if isempty(srcBlk)
    error('swift_vtail_add:unwired', 'Nothing is wired to input %d of %s.', k, blk);
end
ph = get_param(srcBlk, 'PortHandles');
p  = ph.Outport(srcPort + 1);
end

function bak = backup_file(f)
[d, n, e] = fileparts(f);
stamp = datestr(now, 'yyyymmdd_HHMMSS');
if strcmpi(e, '.m')
    bak = fullfile(d, [n '_' stamp '.bak']);          % not a .m, so never on the path
else
    bak = fullfile(d, [n '_backup_' stamp e]);
end
copyfile(f, bak);
end

function h = add(lib, path, pos, varargin)
h = add_block(lib, path, 'Position', pos, varargin{:});
end

function wire(sys, src, dst)
add_line(sys, src, dst, 'autorouting', 'on');
end

function p = outport(blk, k)
ph = get_param(blk, 'PortHandles');  p = ph.Outport(k);
end

function p = inport(blk, k)
ph = get_param(blk, 'PortHandles');  p = ph.Inport(k);
end

function assert_block(p)
if getSimulinkBlockHandle(p) == -1
    error('swift_vtail_add:block', ...
        'Expected block not found: %s\nThe model has changed since this script was written.', p);
end
end

function say(step, fmt, varargin)
fprintf(['  %s  ' fmt '\n'], step, varargin{:});
end
