function swift_setup
%SWIFT_SETUP  Make the ArduPilot_SITL folder the active SWIFT setup.
%
%   >> swift_setup
%
%   Run it once per MATLAB session, and again whenever you switch over
%   from the other setup. It:
%     1  switches the OTHER setup off: closes swift_6dof (refuses if it
%        has unsaved changes), takes ..\Standalone off the MATLAB path, and
%        clears P and the cached SWIFT functions - both folders have files
%        with the same names, and MATLAB would otherwise run whichever copy
%        it found first
%     2  puts THIS folder's subfolders on the path
%     3  sends Simulink's generated files (slprj, *.slxc) to cache\
%     4  changes MATLAB's current folder to this one
%
%   To switch setups:  cd to the other folder and run its swift_setup.
%
%   FOLDERS (ArduPilot_SITL)
%     models/   swift_6dof_sitl.slx  (6-DOF model flown by ArduPlane SITL)
%     params/   swift_placeholder.m  swift_aero_matrix.m  swift_sitl_params.m
%     sitl/     everything ArduPlane-side, all subfolders on the path:
%       run/        swift_run.m (one command per flight), swift_autofly.py,
%                   sessions/swift_session_*.m
%       missions/   session planners swift_sitl_cub*.m + their .param/.waypoints
%       ardupilot/  shared ArduPlane parameters: swift_sitl, _batt, _vtail .param
%       bridge/     ap_json_step.m (JSON bridge), swift_sitl_patch.m
%       checks/     swift_sitl_preflight.m, swift_sitl_signcheck.m, swift_plot_selftest.m
%       analysis/   swift_plot_flight.m, swift_range_check.m, swift_vtail_check.m,
%                   swift_motor_check.m, swift_flight_video.m, event markers
%     docs/     swift_run and session READMEs, step-by-step session guides
%     battery/  swift_battery_add.m  swift_battery_fcn.m  swift_battery_plot.m
%     data/     UAV Specs.xlsx (source of the numbers in swift_placeholder)
%               runs/ (swift_plot_flight and battery plot output)
%     cache/    Simulink build files - safe to delete at any time
%
%   AUTOMATIC
%     To make this the default every time MATLAB starts, add the line
%         run('<repo>\simulation\ArduPilot_SITL\swift_setup.m')
%     to startup.m  (>> edit(fullfile(userpath,'startup.m')) ). Use one
%     setup's line only, never both.
%
%   Ryan Bilock, ARTS Lab | v0.4
%     v0.4  sitl/ split into run, missions, ardupilot, bridge, checks, analysis
%     v0.3  one setup per folder (ArduPilot_SITL / Standalone), switches the other off
%     v0.2  battery/ folder on the path

root   = fileparts(mfilename('fullpath'));
parent = fileparts(root);
me     = 'ArduPilot_SITL';

%% 1 | switch the other setup off --------------------------------------------
oM = 'swift_6dof';
if bdIsLoaded(oM)
    if strcmp(get_param(oM, 'Dirty'), 'on')
        error('swift_setup:dirty', ['%s is open with unsaved changes. Save it (Ctrl+S) ' ...
              'or close it without saving, then run swift_setup again.'], oM);
    end
    close_system(oM, 0);
end
%  Anything on the path under the parent folder that is not this setup:
%  the other setup, the old single-folder layout, delete\.
p    = strsplit(path, pathsep);
pre  = [parent filesep];
mine = [root filesep];
if ispc, cmp = @(a, b) strncmpi(a, b, numel(b)); else, cmp = @(a, b) strncmp(a, b, numel(b)); end
drop = p(cellfun(@(d) cmp([d filesep], pre) && ~cmp([d filesep], mine), p));
if ~isempty(drop), rmpath(drop{:}); end
evalin('base', 'clear P');
clear swift_placeholder swift_aero_matrix swift_battery_fcn swift_battery_plot swift_battery_add
clear ap_json_step swift_sitl_params swift_plot_flight

%% 2 | this setup on the path -------------------------------------------------
onPath = {'models', 'params', 'battery', 'data'};
for k = 1:numel(onPath)
    d = fullfile(root, onPath{k});
    if isfolder(d), addpath(d); end
end
addpath(genpath(fullfile(root, 'sitl')));   % sitl/ and every subfolder (run, missions, ...)
rehash path;

%% 3 | Simulink cache ----------------------------------------------------------
cache = fullfile(root, 'cache');
if ~isfolder(cache), mkdir(cache); end
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, ...
                        'createDir', true);

%% 4 | here --------------------------------------------------------------------
cd(root);
setappdata(0, 'SWIFT_active_setup', me);
fprintf('SWIFT setup: %s is ACTIVE  (%s)\n', upper(me), root);
if ~isempty(drop)
    fprintf('             switched off %d folder(s) from the other setup / old layout\n', numel(drop));
end
fprintf('             model: swift_6dof_sitl.slx   |   Simulink cache -> cache\\\n');
end
