function swift_setup
%SWIFT_SETUP  Make the Standalone folder the active SWIFT setup.
%
%   >> swift_setup
%
%   Run it once per MATLAB session, and again whenever you switch over
%   from the other setup. It:
%     1  switches the OTHER setup off: closes swift_6dof_sitl (refuses if it
%        has unsaved changes), takes ..\ArduPilot_SITL off the MATLAB path, and
%        clears P and the cached SWIFT functions - both folders have files
%        with the same names, and MATLAB would otherwise run whichever copy
%        it found first
%     2  puts THIS folder's subfolders on the path
%     3  sends Simulink's generated files (slprj, *.slxc) to cache\
%     4  changes MATLAB's current folder to this one
%
%   To switch setups:  cd to the other folder and run its swift_setup.
%
%   FOLDERS (Standalone)
%     models/   swift_6dof.slx  (standalone 6-DOF model, no ArduPilot)
%     params/   swift_placeholder.m  swift_aero_matrix.m  swift_mission.m
%     battery/  swift_battery_add.m  swift_battery_fcn.m  swift_battery_plot.m
%     data/     UAV Specs.xlsx (source of the numbers in swift_placeholder)
%               runs/ (battery plot output)
%     cache/    Simulink build files - safe to delete at any time
%
%   AUTOMATIC
%     To make this the default every time MATLAB starts, add the line
%         run('<repo>\simulation\Standalone\swift_setup.m')
%     to startup.m  (>> edit(fullfile(userpath,'startup.m')) ). Use one
%     setup's line only, never both.
%
%   Ryan Bilock, ARTS Lab | v0.3
%     v0.3  one setup per folder (Standalone / ArduPilot_SITL), switches the other off
%     v0.2  battery/ folder on the path

root   = fileparts(mfilename('fullpath'));
parent = fileparts(root);
me     = 'Standalone';

%% 1 | switch the other setup off --------------------------------------------
oM = 'swift_6dof_sitl';
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
clear ap_json_step swift_sitl_params swift_plot_flight  % SITL-only functions, if cached

%% 2 | this setup on the path -------------------------------------------------
onPath = {'models', 'params', 'battery', 'data'};
for k = 1:numel(onPath)
    d = fullfile(root, onPath{k});
    if isfolder(d), addpath(d); end
end
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
fprintf('             model: swift_6dof.slx   |   Simulink cache -> cache\\\n');
end
