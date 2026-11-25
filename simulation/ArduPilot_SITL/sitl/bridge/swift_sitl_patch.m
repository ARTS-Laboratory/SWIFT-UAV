function swift_sitl_patch(M)
%SWIFT_SITL_PATCH  Stage 3b: make swift_6dof_sitl able to take off.
%
%   swift_sitl_patch                 patch swift_6dof_sitl and save it
%   swift_sitl_patch(modelName)      patch another copy (used by the build)
%
%   Already applied to the shipped swift_6dof_sitl.slx - only needed if the model
%   is ever rebuilt from swift_6dof (swift_sitl_build, retired Nov 2026). Safe to run
%   again: every change checks whether it is already there.
%
%   WHAT IT CHANGES (in the SITL copy only - swift_6dof is never opened)
%     1  6DOF initial state   reads P.sitl.ic.*  instead of P.init.*
%                             -> parked on the runway when ArduPlane flies
%     2  Wind Constant        reads P.sitl.wind_ned instead of P.env.wind_ned
%                             -> 0.5 m/s headwind floor while parked
%     3  Thrust table         reads P.sitl.thrust_map instead of P.prop.thrust_map
%                             -> SITL-test propulsion (see swift_sitl_params)
%     4  Ground block         three-wheel tricycle: per-wheel spring-damper,
%                             rolling and sideways friction, and the MOMENTS
%                             those wheel forces make. New input: body rates.
%                             New output M_gnd, added into Mxyz.
%     5  ground params        reads P.gnd.vec (15 numbers)
%
%   In "verify" mode (use_ap = 0, gnd.on = 0) every one of these collapses
%   back to the original values, so the step-20 comparison still holds.
%
%   NEEDS  swift_setup run, swift_6dof_sitl.slx built, swift_sitl_params v0.2
%
%   Ryan Bilock, ARTS Lab | SITL stage 3b | v0.1

if nargin < 1, M = 'swift_6dof_sitl'; end
standalone = (nargin < 1);

if isempty(which([M '.slx'])) && ~bdIsLoaded(M)
    error('swift_sitl_patch:missing', '%s.slx not found. Run swift_setup in ArduPilot_SITL first.', M);
end
load_system(M);

% Fresh parameters, so P.sitl.ic / P.gnd.vec exist for the compile check
if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
evalin('base', 'P = swift_sitl_params(P);');

fprintf('\nPatching %s (stage 3b) ...\n', M);

% --- 1. initial state ---------------------------------------------------
EOM = [M '/6DOF (Euler Angles)'];
need_block(EOM);
ic = {'xme_0', 'P.sitl.ic.pos0''';  'Vm_0',  'P.sitl.ic.uvw0''';
      'eul_0', 'P.sitl.ic.euler0'''; 'pm_0', 'P.sitl.ic.om0'''};
for k = 1:size(ic, 1)
    set_param(EOM, ic{k,1}, ic{k,2});
end
say('1', '6DOF initial state -> P.sitl.ic (ground start when ArduPlane flies)');

% --- 2. wind --------------------------------------------------------------
wc = find_system(M, 'SearchDepth', 1, 'BlockType', 'Constant');
nW = 0;
for k = 1:numel(wc)
    v = strtrim(get_param(wc{k}, 'Value'));
    if any(strcmp(v, {'P.env.wind_ned', 'P.sitl.wind_ned'}))
        set_param(wc{k}, 'Value', 'P.sitl.wind_ned');  nW = nW + 1;
    end
end
if nW ~= 1
    warning('swift_sitl_patch:wind', 'Expected 1 wind Constant, found %d. Check it by hand.', nW);
end
say('2', 'Wind Constant -> P.sitl.wind_ned (%d block)', nW);

% --- 3. thrust table ---------------------------------------------------------
TH = [M '/Thrust'];
need_block(TH);
set_param(TH, 'Table', 'P.sitl.thrust_map');
say('3', 'Thrust table -> P.sitl.thrust_map');

% --- 4. ground block ---------------------------------------------------------
GB  = [M '/Ground'];
GP  = [M '/ground params'];
MX  = [M '/Mxyz'];
need_block(GB); need_block(GP); need_block(MX);
set_param(GP, 'Value', 'P.gnd.vec');

rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', GB);
assert(~isempty(ch), 'swift_sitl_patch: cannot open the Ground MATLAB Function block.');
if contains(ch.Script, 'GROUND v2')
    say('4', 'Ground block already v2 - left alone');
else
    ch.Script = ground_v2();

    % body rates: 6DOF output 6 -> Ground input 5
    pE = get_param(EOM, 'PortHandles');
    pG = get_param(GB,  'PortHandles');
    if numel(pG.Inport) ~= 5
        % Some releases only rebuild the ports on the next diagram update
        try, set_param(M, 'SimulationCommand', 'update'); catch, end
        pG = get_param(GB, 'PortHandles');
    end
    assert(numel(pG.Inport) == 5 && numel(pG.Outport) == 2, ...
        'swift_sitl_patch: Ground block did not update to 5 inputs / 2 outputs.');
    add_line(M, pE.Outport(6), pG.Inport(5), 'autorouting', 'on');

    % M_gnd -> new third input of Mxyz
    nIn = numel(strrep(get_param(MX, 'Inputs'), '|', ''));
    set_param(MX, 'Inputs', ['|' repmat('+', 1, nIn + 1)]);
    pM = get_param(MX, 'PortHandles');
    add_line(M, pG.Outport(2), pM.Inport(end), 'autorouting', 'on');
    l = get_param(pG.Outport(2), 'Line');
    if l ~= -1, set_param(l, 'Name', 'M_gnd'); end
    say('4', 'Ground v2: 3 wheels, friction, moments -> Mxyz input %d', nIn + 1);
end
say('5', 'ground params -> P.gnd.vec');

save_system(M);

if standalone
    try
        set_param(M, 'SimulationCommand', 'update');
        say('Check', 'Update Diagram (Ctrl+D) passed');
    catch err
        fprintf(2, '\nUpdate Diagram failed - the model is saved, fix this and press Ctrl+D:\n%s\n', ...
            err.getReport('basic'));
    end
    open_system(M);
end
fprintf('Stage 3b patch done. Next: swift_sitl_preflight.\n\n');
end

% =========================================================================
function c = ground_v2()
c = sprintf([ ...
'function [F_b, M_b] = ground(Xe, Ve, DCM, g, om)\n' ...
'%%GROUND v2  Three-wheel spring-damper runway with friction (stage 3b).\n' ...
'%%   Xe  NED position (m)      Ve  NED velocity (m/s)\n' ...
'%%   DCM NED -> body            om  body rates (rad/s)\n' ...
'%%   g = [on; k; c; mu_roll; mu_lat; v_eps; wheels(:)]   (15 x 1)\n' ...
'%%   F_b, M_b  wheel forces and moments about the CG, body axes.\n' ...
'F_b = zeros(3,1);\n' ...
'M_b = zeros(3,1);\n' ...
'if g(1) < 0.5\n' ...
'    return\n' ...
'end\n' ...
'k = g(2);  c = g(3);  mur = g(4);  mul = g(5);  ve = max(g(6), 1e-3);\n' ...
'R   = reshape(g(7:15), 3, 3);       %% wheel contact points, body axes\n' ...
'Cbe = DCM'';                         %% body -> NED\n' ...
'xh  = [Cbe(1,1); Cbe(2,1); 0];       %% nose direction along the runway\n' ...
'n   = sqrt(xh(1)^2 + xh(2)^2);\n' ...
'if n < 1e-6\n' ...
'    xh = [1; 0; 0];\n' ...
'else\n' ...
'    xh = xh / n;\n' ...
'end\n' ...
'yh = [-xh(2); xh(1); 0];             %% right wing direction along the runway\n' ...
'for i = 1:3\n' ...
'    r   = R(:, i);\n' ...
'    p   = Xe + Cbe * r;              %% wheel position, NED\n' ...
'    if p(3) > 0                      %% below the runway surface\n' ...
'        v = Ve + Cbe * cross(om, r); %% wheel velocity, NED\n' ...
'        N = k * p(3) + c * v(3);     %% normal load, N\n' ...
'        if N > 0\n' ...
'            vl = xh'' * v;\n' ...
'            vs = yh'' * v;\n' ...
'            F  = [0; 0; -N] - mur*N*tanh(vl/ve)*xh - mul*N*tanh(vs/ve)*yh;\n' ...
'            Fb = DCM * F;\n' ...
'            F_b = F_b + Fb;\n' ...
'            M_b = M_b + cross(r, Fb);\n' ...
'        end\n' ...
'    end\n' ...
'end\n']);
end

function need_block(p)
if getSimulinkBlockHandle(p) == -1
    error('swift_sitl_patch:block', ...
        'Expected block not found: %s\nThe SITL copy has changed since this patch was written.', p);
end
end

function say(step, fmt, varargin)
fprintf(['  %-6s ' fmt '\n'], step, varargin{:});
end
