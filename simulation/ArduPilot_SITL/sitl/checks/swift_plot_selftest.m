function T = swift_plot_selftest(varargin)
%SWIFT_PLOT_SELFTEST  Run every SWIFT plotting script on many kinds of flight.
%
%   swift_plot_selftest                  all cases, figures hidden, PNGs saved
%   T = swift_plot_selftest('cases', {'D_full', 'D_reverse_rwy'})
%   T = swift_plot_selftest('show', true)  also leave the figures open
%
%   Rebuilds flights from the saved runs in data/runs (sessionD_cub,
%   sessionC_cub, sessionA_ground) plus edited copies that cover what a new
%   flight could look like, then runs swift_plot_flight (which also runs
%   swift_battery_plot and swift_events_figure), swift_motor_check and
%   swift_flight_video (three stills per flight, in .../video_frames) on each.
%
%   CASES
%     D_full         Session D with the runway data from swift_sitl_cub_d
%     D_noS          Session D without S in the workspace (runway unknown)
%     C_full         Session C, its own mission
%     A_ground       Session A: ground run, no battery logger, no mission
%     D_cut_in_air   log stops at 8 min, still flying (no landing)
%     D_pre_takeoff  log stops at 1 min, never left the ground
%     D_no_battery   no out_batt (battery figures skipped)
%     D_no_wp        no .waypoints file (flight events only)
%     D_reverse_rwy  flight mirrored to land on runway 31 (far threshold)
%     D_air_start    starts 50 m up (no takeoff or landing figures)
%     D_events_off   'events', false
%     D_5s_log       only the first 5 s of the log
%
%   T (also printed): one row per case - PASS/FAIL for each script, the
%   error and where it happened, and how many PNGs were written to
%   data/runs/selftest_<case>/. Open those folders to check the pictures.
%
%   Leaves P and S in the base workspace as they were.
%
%   Ryan Bilock, ARTS Lab | SITL plotting | v0.2
%     v0.2  also checks swift_flight_video (start, middle and end stills)

o.cases = {};  o.show = false;
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
here = fileparts(mfilename('fullpath'));
runs = fullfile(fileparts(fileparts(here)), 'data', 'runs');   % sitl/checks -> data/runs

% ---- keep the user's workspace as it was --------------------------------------
hadP = evalin('base', 'exist(''P'',''var'')');  hadS = evalin('base', 'exist(''S'',''var'')');
if hadP, P0 = evalin('base', 'P'); end
if hadS, S0 = evalin('base', 'S'); end
cleanup = onCleanup(@() restore(hadP, hadS, iff(hadP, @() P0), iff(hadS, @() S0)));

% ---- saved runs -----------------------------------------------------------------
D  = load(fullfile(runs, 'sessionD_cub', 'signals.mat'));
Cc = [];  A = [];
if isfile(fullfile(runs, 'sessionC_cub', 'signals.mat')),    Cc = load(fullfile(runs, 'sessionC_cub', 'signals.mat')); end
if isfile(fullfile(runs, 'sessionA_ground', 'signals.mat')), A  = load(fullfile(runs, 'sessionA_ground', 'signals.mat')); end
assignin('base', 'P', D.P);                       % the parameter set the flights were run with
mis = fullfile(fileparts(here), 'missions');
wpD = fullfile(mis, 'swift_sitl_cub_d.waypoints');  wpC = fullfile(mis, 'swift_sitl_cub.waypoints');
Sd  = struct('wp_file', wpD, 'airport', struct('hdg', 125.55, 'len', 1528, 'dthr', 119));

hdg = 125.55;  L = 1528;                          % runway 13 frame, for the mirrored flight
x = D.pos(:,1)*cosd(hdg) + D.pos(:,2)*sind(hdg);  y = -D.pos(:,1)*sind(hdg) + D.pos(:,2)*cosd(hdg);
x2 = L - x;  y2 = -y;
Drev = D;  Drev.pos(:,1) = x2*cosd(hdg) - y2*sind(hdg);  Drev.pos(:,2) = x2*sind(hdg) + y2*cosd(hdg);
Dair = D;  Dair.pos(:,3) = D.pos(:,3) - 50;

%           name             run   keep               batt   wp    S                       events
C = {'D_full',        D,    [],                true,  wpD,  Sd,                     true
     'D_noS',         D,    [],                true,  wpD,  [],                     true
     'C_full',        Cc,   [],                true,  wpC,  struct('wp_file', wpC),  true
     'A_ground',      A,    [],                false, '',   [],                     true
     'D_cut_in_air',  D,    D.t < 8*60,        true,  wpD,  Sd,                     true
     'D_pre_takeoff', D,    D.t < 60,          true,  wpD,  Sd,                     true
     'D_no_battery',  D,    [],                false, wpD,  Sd,                     true
     'D_no_wp',       D,    [],                true,  '',   [],                     true
     'D_reverse_rwy', Drev, [],                true,  '',   setfield(Sd, 'wp_file', ''), true
     'D_air_start',   Dair, [],                true,  wpD,  Sd,                     true
     'D_events_off',  D,    [],                true,  wpD,  Sd,                     false
     'D_5s_log',      D,    D.t < D.t(1) + 5,  true,  wpD,  Sd,                     true};
if ~isempty(o.cases), C = C(ismember(C(:,1), o.cases), :); end

T = cell2table(cell(0, 7), 'VariableNames', {'case', 'plot_flight', 'motor_check', 'video', 'error', 'where', 'pngs'});
fprintf('\n%s\n SWIFT plotting self-test  (%d cases)\n%s\n', repmat('-', 1, 96), size(C, 1), repmat('-', 1, 96));
for c = 1:size(C, 1)
    [nm, S, kp, bt, wp, Sb, ev] = C{c, :};
    if isempty(S), fprintf(' %-14s skipped (no saved run)\n', nm);  continue, end
    if isempty(Sb), evalin('base', 'clear S'); else, assignin('base', 'S', Sb); end
    out = build_out(S, kp, bt);
    tag = ['selftest_' nm];
    r1 = 'PASS';  r2 = '-';  em = '';  wh = '';
    try
        evalc('swift_plot_flight(out, ''tag'', tag, ''show'', o.show, ''wp'', wp, ''events'', ev);');
    catch err
        r1 = 'FAIL';  em = err.message;  wh = where(err);
    end
    if bt
        r2 = 'PASS';
        try
            evalc('swift_motor_check(out, ''tag'', tag, ''show'', o.show, ''wp'', wp, ''events'', ev);');
        catch err
            r2 = 'FAIL';
            if isempty(em), em = err.message;  wh = where(err); end
        end
    end
    r3 = '-';                                        % flight video: three stills (start, middle, end)
    if exist('swift_flight_video', 'file') == 2
        r3 = 'PASS';
        tt = out.out_pos.time;  at = [tt(1), mean(tt([1 end])), tt(end)] / 60;
        try
            evalc('swift_flight_video(out, ''tag'', tag, ''show'', false, ''wp'', wp, ''at'', at, ''size'', [1280 720]);');
        catch err
            r3 = 'FAIL';
            if isempty(em), em = err.message;  wh = where(err); end
        end
    end
    if ~o.show, close all force; end
    n = numel(dir(fullfile(runs, tag, '*.png')));
    T(end+1, :) = {nm, r1, r2, r3, em, wh, n}; %#ok<AGROW>
    fprintf(' %-14s  plot_flight %-4s  motor_check %-4s  video %-4s  %2d PNGs  %s\n', nm, r1, r2, r3, n, ...
            strtrim([em ' ' wh]));
end
nf = sum(strcmp(T.plot_flight, 'FAIL') | strcmp(T.motor_check, 'FAIL') | strcmp(T.video, 'FAIL'));
fprintf('%s\n %d of %d cases passed. PNGs: %s%sselftest_<case>\n%s\n\n', repmat('-', 1, 96), ...
        height(T) - nf, height(T), runs, filesep, repmat('-', 1, 96));
end

% =========================================================================
function out = build_out(S, keep, batt)
%  A saved signals.mat back into the shape the plot scripts read.
if isempty(keep), keep = true(size(S.t)); end
mk = @(t, y) struct('time', t, 'signals', struct('values', y));
t = S.t(keep);
out.out_pos   = mk(t, S.pos(keep,:));    out.out_V    = mk(t, S.V(keep));
out.out_euler = mk(t, S.eul(keep,:));    out.out_om   = mk(t, S.om(keep,:));
out.out_alpha = mk(t, S.alpha(keep));    out.out_beta = mk(t, S.beta(keep));
out.out_qbar  = mk(t, S.qbar(keep));     out.out_delta = mk(t, S.delta(keep,:));
out.out_accel = mk(t, S.accel(keep,:));
if batt && isfield(S, 'batt_t') && ~isempty(S.batt_t)
    kb = S.batt_t <= t(end);
    out.out_batt = mk(S.batt_t(kb), S.batt(kb,:));
end
end

function s = where(err)
s = '';
if ~isempty(err.stack), s = sprintf('(%s line %d)', err.stack(1).name, err.stack(1).line); end
end

function restore(hadP, hadS, P0, S0)
if hadP, assignin('base', 'P', P0); else, evalin('base', 'clear P'); end
if hadS, assignin('base', 'S', S0); else, evalin('base', 'clear S'); end
end

function v = iff(c, f)
if c, v = f(); else, v = []; end
end
