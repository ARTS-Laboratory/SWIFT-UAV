function R = swift_plot_flight(out, varargin)
%SWIFT_PLOT_FLIGHT  Plots and a summary of one swift_6dof_sitl run (task 4).
%
%   swift_plot_flight                     plot 'out' from the base workspace
%   swift_plot_flight(out)                plot a SimulationOutput you kept
%   R = swift_plot_flight(out, Name, Value, ...)
%
%   OPTIONS
%     'save'    true     write PNGs, signals and summary to data/runs/<tag>/
%     'tag'     auto     run folder name (default run_yyyymmdd_HHMM)
%     'wp'      auto     .waypoints file to draw (swift_run passes the session's; none = no route)
%     'aplog'   ''       Mission Planner "Create Matlab File" export of the
%                        ArduPilot .BIN log -> overlays desired roll/pitch
%     'apshift' 0        s added to ArduPilot log time if the traces are offset
%     'show'    true     false = build and save the figures without showing them
%     'events'  true     numbered markers on every graph + the events list image
%     'names'   auto     mission item names for the list (S.seg(:,1) is used
%                        automatically when S.wp_file is the 'wp' file)
%
%   FIGURES
%     1  Flight overview   altitude, airspeed vs stall, throttle, load factor
%     2  Ground track      north/east path coloured by altitude, planned route
%                          dashed, same numbered badges as the battery map
%     3  Attitude & rates  roll/pitch/yaw (+ ArduPilot desired), p/q/r
%     4  Aero & controls   alpha vs stall alpha, sideslip, surfaces vs travel
%     5  Takeoff roll      speed, height and pitch against runway distance
%     6  Landing           height, airspeed, sink rate and pitch against
%                          distance from touchdown; runway shaded, the moment
%                          it comes over the runway (threshold) and the moment
%                          it touches down marked on every panel; rollout
%     7  Battery on track  one image: ground track coloured by pack current
%                          on top, pack current against time beneath (same
%                          colour scale, grey altitude behind), same event
%                          badges in both panels (which manoeuvre costs what)
%     +  swift_battery_plot  SOC, pack voltage, current/power, energy used,
%                          saved in the same run folder (needs out_batt)
%     +  Flight events     altitude profile with every numbered point on it,
%                          plus a key: time, what is happening, airspeed and
%                          charge (swift_events_figure)
%
%   MARKERS (v0.5, 'events' on by default)
%     swift_flight_events numbers each mission item (timed from the flight)
%     and each flight event (throttle up, liftoff, highest point, peak
%     current, battery warnings, touchdown, stop). The same numbers appear as
%     filled badges over dotted lines on every time plot, as badges on both
%     maps, on the takeoff and landing distance plots inside their windows,
%     and in the list. Blue-grey = mission item, orange = flight event,
%     red = battery. Time plots read in minutes from the start of the log.
%   Figure 1 gets a fifth panel (state of charge and current) when the run
%   logged out_batt.
%
%   R (also printed)  duration, max altitude, minimum airborne airspeed and
%   stall margin, time below 1.1 x stall, max bank/pitch/alpha/load factor,
%   takeoff distance and time, surface saturation, throttle use, touchdown
%   airspeed/sink/distance past the threshold, rollout, and (R.batt) the
%   swift_battery_plot summary.
%
%   The model logs at 100 Hz through its To Workspace blocks. With
%   Model Settings > Data Import/Export > "Single simulation output" ticked
%   (the default) they arrive inside 'out' when you press Stop.
%
%   Ryan Bilock, ARTS Lab | SITL task 4 | v1.2
%     v1.2  every time on a figure in minutes (takeoff title too)
%     v1.1  'out' may also be a struct of saved signals (used by
%           swift_plot_selftest to check the figures on many kinds of flight)
%     v1.0  landing figure: runway band, threshold crossing ("over the runway")
%           and touchdown marked on every panel, icons on the plot and the
%           numbers in one key under the figure;
%           threshold height, speed and distance to touchdown in R and the summary
%     v0.9  ground track drawn like the battery map (same badges); time plots
%           in minutes from the start of the log; filled event badges on every
%           time plot (swift_mark_events v0.2); legends no longer pick up
%           'data1'; reference-line labels moved apart
%     v0.8  battery figure badges never overlap: each badge is kept clear of
%           other badges, event dots, the traces and leader lines (any flight)
%     v0.7  battery figure panels linked: shared current colour scale and one
%           colorbar, same filled event badges on the map and the time line
%     v0.6  battery-on-track figure restacked: map on top, altitude and
%           pack power in a short box beneath (battery_track_fig)
%     v0.5  numbered event markers on every figure, events list image,
%           events.csv; default 'wp' = S.wp_file when S is in the workspace
%     v0.4  readable(): light theme + black text on every figure (dark-theme fix)
%     v0.3  battery panel + battery-on-track figure + swift_battery_plot in
%           the same run folder; landing figure and touchdown summary
%     v0.2  read_wp draws LOITER, LAND and LOITER_TO_ALT points

%% ---- inputs -------------------------------------------------------------
if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); else, out = []; end
end
here = fileparts(mfilename('fullpath'));
o.save = true;  o.tag = ['run_' datestr(now, 'yyyymmdd_HHMM')];
o.wp = fullfile(fileparts(here), 'missions', 'swift_sitl_box.waypoints');
o.aplog = '';  o.apshift = 0;  o.show = true;  o.events = true;  o.names = {};
gotWP = any(strcmpi(varargin(1:2:end), 'wp'));
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
Sb = [];
if evalin('base', 'exist(''S'',''var'')'), Sb = evalin('base', 'S'); end
if ~gotWP && isstruct(Sb) && isfield(Sb, 'wp_file') && isfile(Sb.wp_file)
    o.wp = Sb.wp_file;                     % the mission swift_sitl_cub / _cub_d just wrote
end
if isempty(o.names) && isstruct(Sb) && isfield(Sb, 'seg') && isfield(Sb, 'wp_file') && ...
        strcmp(Sb.wp_file, o.wp)
    o.names = Sb.seg(:, 1);
end
Arp = [];                                  % runway data, only from the mission being plotted
if isstruct(Sb) && isfield(Sb, 'airport') && isfield(Sb, 'wp_file') && strcmp(Sb.wp_file, o.wp)
    Arp = Sb.airport;
end

if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
P  = evalin('base', 'P');
g  = P.const.g;
Vs = P.perf.V_stall;

%% ---- signals -------------------------------------------------------------
[t, pos] = sig(out, 'out_pos');
if isempty(t), error('swift_plot_flight:nolog', 'No out_pos found - run the model first.'); end
rs  = @(name, n) resample_to(out, name, t, n);
V   = rs('out_V', 1);       eul = rs('out_euler', 3);   om  = rs('out_om', 3);
al  = rs('out_alpha', 1);   be  = rs('out_beta', 1);    qb  = rs('out_qbar', 1);
dl  = rs('out_delta', 4);   acc = rs('out_accel', 3);

h    = -pos(:,3);
vned = [gradient(pos(:,1), t), gradient(pos(:,2), t), gradient(pos(:,3), t)];
gs   = hypot(vned(:,1), vned(:,2));
nz   = -acc(:,3) / g;
dist = [0; cumsum(hypot(diff(pos(:,1)), diff(pos(:,2))))];

%  Battery (v0.3) - out_batt from swift_battery_add, 10 columns
[tb, yb] = sig(out, 'out_batt');
hasB = ~isempty(tb) && size(yb, 2) == 10;
if hasB
    bt  = interp1(tb, yb, t, 'linear', 'extrap');
    SOC = 100 * bt(:,1);  Vpk = bt(:,2);  Ipk = bt(:,3);  Ppk = bt(:,4);
else
    SOC = nan(size(t));  Vpk = SOC;  Ipk = SOC;  Ppk = SOC;
end

h0     = h(1);
onGnd0 = h0 < 1;                                     % started on the runway
air    = h > h0 + 0.5;
iLift  = find(air & V > 0.9*Vs, 1);
if ~onGnd0, air(:) = true; iLift = []; end
if ~any(air), air(:) = true; end                   % never left the ground

%% ---- ArduPilot log (optional) ---------------------------------------------
AP = struct('ok', false);
if ~isempty(o.aplog)
    AP = load_aplog(o.aplog, o.apshift);
end

%% ---- mission (optional) ------------------------------------------------------
WP = read_wp(o.wp);

%% ---- numbered events (v0.5) --------------------------------------------------
E = [];
if o.events && exist('swift_flight_events', 'file') == 2
    try
        E = swift_flight_events(out, P, 'wp', o.wp, 'names', o.names);
    catch err
        fprintf(2, '  Event markers skipped: %s\n', err.message);  E = [];
    end
end
hasE = ~isempty(E) && ~isempty(E.t);

%% ---- style ----------------------------------------------------------------------
C.blue = [0.13 0.40 0.75];  C.orange = [0.90 0.45 0.10];  C.green = [0.18 0.60 0.30];
C.red  = [0.80 0.15 0.15];  C.grey = [0.45 0.45 0.45];    C.purple = [0.50 0.30 0.70];
vis = 'on';  if ~o.show, vis = 'off'; end
figs = gobjects(0);
r2d = 180/pi;
tm  = t / 60;                                      % min from start of log (all time plots)

%% ---- 1  overview ------------------------------------------------------------------
f = newfig('SWIFT SITL - flight overview', vis);  figs(end+1) = f;
tl = tiledlayout(f, 4 + hasB, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);  plot(ax, tm, h, 'Color', C.blue, 'LineWidth', 1.4);
ylabel(ax, 'Altitude (m)');  yline(ax, 121.9, ':', '400 ft', 'Color', C.grey);
mark_lift(ax, tm, iLift, C);  title(ax, 'Altitude above the runway');
ax = nexttile(tl);  plot(ax, tm, V, 'Color', C.blue, 'LineWidth', 1.4); hold(ax, 'on');
plot(ax, tm, gs, 'Color', C.grey, 'LineWidth', 0.9);
yline(ax, Vs, '--', 'stall', 'Color', C.red, 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
yline(ax, 1.2*Vs, ':', '1.2 V_s', 'Color', C.orange, 'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom');
yline(ax, P.init.V0, ':', 'cruise', 'Color', C.green, 'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'top');
ylabel(ax, 'Speed (m/s)');
legend(ax, {'airspeed', 'groundspeed'}, 'Location', 'south', 'Orientation', 'horizontal', 'AutoUpdate', 'off');
mark_lift(ax, tm, iLift, C);  title(ax, 'Airspeed against stall');
ax = nexttile(tl);  plot(ax, tm, 100*dl(:,4), 'Color', C.orange, 'LineWidth', 1.2);
ylabel(ax, 'Throttle (%)');  ylim(ax, [0 105]);  title(ax, 'Throttle');
ax = nexttile(tl);  plot(ax, tm, nz, 'Color', C.purple, 'LineWidth', 1.0);
yline(ax, P.struct.n_max, '--', sprintf('n_{max} %g', P.struct.n_max), 'Color', C.red);
yline(ax, 1, ':', 'Color', C.grey);
ylabel(ax, 'n_z (g)');  title(ax, 'Load factor');
if hasB
    ax = nexttile(tl);
    yyaxis(ax, 'left');   plot(ax, tm, SOC, 'LineWidth', 1.4);  ylabel(ax, 'SOC (%)');
    yyaxis(ax, 'right');  plot(ax, tm, Ipk, 'LineWidth', 1.0);  ylabel(ax, 'Current (A)');
    grid(ax, 'on');  title(ax, 'Battery: state of charge and pack current');
end
xlabel(ax, 'Time from start of log (min)');
linkaxes(findobj(f, 'Type', 'axes'), 'x');
mark_fig(f, E, 1/60);

%% ---- 2  ground track (v0.9: same map style and badges as the battery figure) ----------------
f = ground_track_fig('SWIFT SITL - ground track', vis, t, pos, h, h0, WP, E, hasE, C);  figs(end+1) = f;

%% ---- 3  attitude and rates ------------------------------------------------------------
f = newfig('SWIFT SITL - attitude and rates', vis);  figs(end+1) = f;
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
lab = {'Roll \phi', 'Pitch \theta', 'Yaw \psi'};  rl = {'p', 'q', 'r'};
lim = [rad2deg(P.gnc.phi_max), rad2deg(P.gnc.theta_max), NaN];
apDes = {'DesRoll', 'DesPitch', ''};  apAct = {'Roll', 'Pitch', ''};
for k = 1:3
    ax = nexttile(tl, 2*k - 1);  hold(ax, 'on');
    y = r2d * eul(:,k);  if k == 3, y = mod(y, 360); end
    plot(ax, tm, y, 'Color', C.blue, 'LineWidth', 1.3);
    leg = {'model'};
    if AP.ok && ~isempty(apDes{k}) && isfield(AP.att, apDes{k})
        plot(ax, AP.att.t / 60, AP.att.(apDes{k}), '--', 'Color', C.orange, 'LineWidth', 1.1);
        leg{end+1} = 'ArduPilot desired'; %#ok<AGROW>
        if isfield(AP.att, apAct{k})
            plot(ax, AP.att.t / 60, AP.att.(apAct{k}), ':', 'Color', C.grey, 'LineWidth', 1);
            leg{end+1} = 'ArduPilot estimate'; %#ok<AGROW>
        end
    end
    if ~isnan(lim(k)), yline(ax, lim(k), ':', 'Color', C.red); yline(ax, -lim(k), ':', 'Color', C.red); end
    ylabel(ax, [lab{k} ' (deg)']);  if k == 1, legend(ax, leg, 'Location', 'best', 'AutoUpdate', 'off'); end
    ax = nexttile(tl, 2*k);
    plot(ax, tm, r2d * om(:,k), 'Color', C.green, 'LineWidth', 1.0);
    ylabel(ax, [rl{k} ' (deg/s)']);
end
xlabel(nexttile(tl, 5), 'Time from start of log (min)');  xlabel(nexttile(tl, 6), 'Time from start of log (min)');
title(nexttile(tl, 1), 'Attitude');  title(nexttile(tl, 2), 'Body rates');
linkaxes(findobj(f, 'Type', 'axes'), 'x');
mark_fig(f, E, 1/60);

%% ---- 4  aero and controls ------------------------------------------------------------------
f = newfig('SWIFT SITL - aerodynamics and controls', vis);  figs(end+1) = f;
tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl, 1);  plot(ax, tm, r2d*al, 'Color', C.blue, 'LineWidth', 1.2);
yline(ax, rad2deg(P.aero.alpha_stall), '--', 'stall \alpha', 'Color', C.red);
ylabel(ax, '\alpha (deg)');  title(ax, 'Angle of attack');
ax = nexttile(tl, 3);  plot(ax, tm, r2d*be, 'Color', C.purple, 'LineWidth', 1.0);
ylabel(ax, '\beta (deg)');  title(ax, 'Sideslip');
ax = nexttile(tl, 5);  plot(ax, tm, qb, 'Color', C.grey, 'LineWidth', 1.0);
ylabel(ax, 'q (Pa)');  xlabel(ax, 'Time from start of log (min)');  title(ax, 'Dynamic pressure');
sn = {'Aileron \delta_a', 'Elevator \delta_e', 'Rudder \delta_r'};
dm = r2d * [P.act.da_max, P.act.de_max, P.act.dr_max];
cc = {C.blue, C.orange, C.green};
for k = 1:3
    ax = nexttile(tl, 2*k);
    plot(ax, tm, r2d*dl(:,k), 'Color', cc{k}, 'LineWidth', 1.1);
    yline(ax, dm(k), ':', 'Color', C.red);  yline(ax, -dm(k), ':', 'Color', C.red);
    ylim(ax, 1.15*dm(k)*[-1 1]);  ylabel(ax, [sn{k} ' (deg)']);
end
title(nexttile(tl, 2), 'Control surfaces (dotted = travel limit)');
xlabel(nexttile(tl, 6), 'Time from start of log (min)');
linkaxes(findobj(f, 'Type', 'axes'), 'x');
mark_fig(f, E, 1/60);

%% ---- 5  takeoff roll ---------------------------------------------------------------------------
xTO = NaN;  tTO = NaN;
if onGnd0 && ~isempty(iLift)
    iA = find(dl(:,4) > 0.3, 1);                    % brake release = throttle up
    if isempty(iA), iA = 1; end
    xTO = dist(iLift) - dist(iA);  tTO = t(iLift) - t(iA);
    w = iA:min(numel(t), iLift + round(15 / max(mean(diff(t)), eps)));
    f = newfig('SWIFT SITL - takeoff', vis);  figs(end+1) = f;
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    xr = dist(w) - dist(iA);
    ax = nexttile(tl);  plot(ax, xr, V(w), 'Color', C.blue, 'LineWidth', 1.4); hold(ax, 'on');
    yline(ax, Vs, '--', 'stall', 'Color', C.red);
    yline(ax, round(1.05*Vs), ':', 'rotate', 'Color', C.orange);
    ylabel(ax, 'Airspeed (m/s)');  title(ax, sprintf('Takeoff: %.0f m, %.2f min to liftoff', xTO, tTO / 60));
    ax = nexttile(tl);  plot(ax, xr, h(w) - h0, 'Color', C.green, 'LineWidth', 1.4);
    ylabel(ax, 'Height (m)');
    ax = nexttile(tl);  plot(ax, xr, r2d*eul(w,2), 'Color', C.purple, 'LineWidth', 1.4);
    ylabel(ax, 'Pitch (deg)');  xlabel(ax, 'Runway distance from throttle-up (m)');
    for a = findobj(f, 'Type', 'axes')'
        xline(a, xTO, '-', 'liftoff', 'Color', C.grey, 'LabelVerticalAlignment', 'bottom');
    end
    mark_fig(f, E, 1, dist - dist(iA), w);
end

%% ---- 6  landing ------------------------------------------------------------------------------------
%  Touchdown = first sample back on the ground after the last airborne one.
%  Over the runway (v1.0) = the moment the aircraft crosses the runway end it
%  lands toward (the threshold), found in the runway frame: x along the runway
%  from the parked start point on P.sitl.hdg0, y right of the centreline.
LD = struct('ok', false, 'V', NaN, 'sink', NaN, 'x_thr', NaN, 'rollout', NaN, 't', NaN, ...
            'thr_ok', false, 'thr_h', NaN, 'thr_V', NaN, 'thr_y', NaN, 'thr_to_td', NaN, 'thr_t', NaN);
if onGnd0 && ~isempty(iLift)
    iLast = find(air, 1, 'last');
    if ~isempty(iLast) && iLast > iLift && iLast < numel(t)
        iTD = iLast + 1;  dtm = max(mean(diff(t)), eps);
        sink = vned(:,3);                              % m/s, + = down
        k0 = max(1, iTD - round(0.3 / dtm));
        LD.ok = true;  LD.t = t(iTD);  LD.V = V(iTD);  LD.sink = mean(sink(k0:iTD));
        LD.x_thr = runway_x(pos(iTD,1:2), P);
        iStop = find(t > t(iTD) & gs < 0.5, 1);
        if isempty(iStop), iStop = numel(t); end
        LD.rollout = dist(iStop) - dist(iTD);

        % ---- threshold crossing on the final approach
        RW = runway_info(Arp, P);
        [xa, ya] = rwy_xy(pos(:,1:2), RW.hdg);
        dirS = sign(xa(min(iTD + round(5 / dtm), numel(t))) - xa(max(iTD - round(5 / dtm), 1)));
        if dirS == 0, dirS = 1; end
        xe = 0;  if dirS < 0, xe = RW.len; end           % landing the other way: far end
        iThr = [];
        if isfinite(xe)
            b = find(dirS * (xa(1:iTD) - xe) < 0, 1, 'last');
            if ~isempty(b) && b < iTD && abs(ya(b + 1)) < RW.halfw + 40, iThr = b + 1; end
        end
        if ~isempty(iThr)
            LD.thr_ok = true;  LD.thr_t = t(iThr);  LD.thr_h = h(iThr) - h0;  LD.thr_V = V(iThr);
            LD.thr_y = ya(iThr);  LD.thr_to_td = dist(iTD) - dist(iThr);
            LD.x_thr = dirS * (xa(iTD) - xe);              % touchdown distance past the threshold
        end

        w0 = max(1, iTD - round(70 / dtm));              % last ~70 s of the approach + rollout
        if LD.thr_ok, w0 = min(w0, max(1, iThr - round(15 / dtm))); end
        w = w0:iStop;
        xr = dist(w) - dist(iTD);
        f = newfig('SWIFT SITL - landing', vis);  figs(end+1) = f;
        tl = tiledlayout(f, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        axL = gobjects(1, 4);
        ax = nexttile(tl);  axL(1) = ax;
        plot(ax, xr, h(w) - h0, 'Color', C.green, 'LineWidth', 1.4);  hold(ax, 'on');
        yline(ax, 3, ':', 'flare 3 m', 'Color', C.orange, 'LabelHorizontalAlignment', 'left');
        ylabel(ax, 'Height (m)');
        if LD.thr_ok
            title(ax, sprintf(['Landing: over the runway at %.1f m and %.1f m/s, touchdown %.0f m later ' ...
                  'at %.1f m/s (%.2f m/s sink), %.0f m rollout'], LD.thr_h, LD.thr_V, LD.thr_to_td, ...
                  LD.V, LD.sink, LD.rollout));
        else
            title(ax, sprintf('Landing: %.1f m/s, %.2f m/s sink at touchdown, %.0f m rollout', ...
                  LD.V, LD.sink, LD.rollout));
        end
        ax = nexttile(tl);  axL(2) = ax;
        plot(ax, xr, V(w), 'Color', C.blue, 'LineWidth', 1.4);  hold(ax, 'on');
        plot(ax, xr, gs(w), 'Color', C.grey, 'LineWidth', 0.9);
        yline(ax, Vs, '--', 'stall', 'Color', C.red, 'LabelHorizontalAlignment', 'left');
        ylabel(ax, 'Speed (m/s)');  legend(ax, {'airspeed', 'groundspeed'}, 'Location', 'southwest', 'AutoUpdate', 'off');
        ax = nexttile(tl);  axL(3) = ax;
        plot(ax, xr, sink(w), 'Color', C.purple, 'LineWidth', 1.2);
        yline(ax, 0.6, ':', 'TECS\_LAND\_SINK 0.6', 'Color', C.orange, 'LabelHorizontalAlignment', 'left');
        ylabel(ax, 'Sink (m/s)');
        ax = nexttile(tl);  axL(4) = ax;
        plot(ax, xr, r2d*eul(w,2), 'Color', C.purple, 'LineWidth', 1.2);  hold(ax, 'on');
        plot(ax, xr, 100*dl(w,4) / 5, 'Color', C.orange, 'LineWidth', 1.0);
        ylabel(ax, 'Pitch (deg)');  legend(ax, {'pitch', 'throttle / 5 (%)'}, 'Location', 'northwest', 'AutoUpdate', 'off');
        xlabel(ax, 'Distance from touchdown (m)');
        linkaxes(axL, 'x');  xlim(axL(1), [xr(1) xr(end)]);

        % ---- runway band, threshold and touchdown on every panel; the words go
        %      in one key under the figure so nothing sits on the data
        cK = [0.10 0.10 0.10];  cD = [0.45 0.45 0.45];
        hK = gobjects(0);  kL = {};
        for j = 1:4
            a = axL(j);  grid(a, 'on');  hold(a, 'on');
            yl = get(a, 'YLim');
            if LD.thr_ok
                xb = [xr(iThr - w0 + 1), min(xr(end), xr(iThr - w0 + 1) + RW.len)];
                if ~isfinite(xb(2)), xb(2) = xr(end); end
                hq = patch(a, xb([1 2 2 1]), yl([1 1 2 2]), [0.55 0.55 0.55], 'FaceAlpha', 0.10, ...
                           'EdgeColor', 'none', 'HandleVisibility', 'off');
                if j == 1, hp = hq; end                  % key uses the height panel's own objects
                if RW.dthr > 0
                    hq = xline(a, xb(1) + RW.dthr, ':', 'Color', cD, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                    if j == 1, hd = hq; end
                end
                xline(a, xb(1), '--', 'Color', cK, 'LineWidth', 1.1, 'HandleVisibility', 'off');
            end
            xline(a, 0, '-', 'Color', cK, 'LineWidth', 1.4, 'HandleVisibility', 'off');
            set(a, 'YLim', yl);
        end
        a = axL(1);                                      % the two moments as icons on the height panel
        if LD.thr_ok
            hK(end+1) = plot(a, xr(iThr - w0 + 1), LD.thr_h, 'v', 'MarkerSize', 8, ...
                             'MarkerFaceColor', cK, 'MarkerEdgeColor', 'w', 'LineStyle', '--', 'Color', cK);
            kL{end+1} = sprintf('Over the runway (threshold): %.1f m high, %.1f m/s, %.0f m before touchdown', ...
                                LD.thr_h, LD.thr_V, LD.thr_to_td);
        end
        hK(end+1) = plot(a, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', cK, 'MarkerEdgeColor', 'w', ...
                         'LineStyle', '-', 'Color', cK);
        kL{end+1} = sprintf('Touchdown: %.1f m/s, %.2f m/s sink', LD.V, LD.sink);
        if LD.thr_ok
            set(hp, 'HandleVisibility', 'on');  hK(end+1) = hp;  kL{end+1} = 'Over the runway';
            if RW.dthr > 0
                set(hd, 'HandleVisibility', 'on');  hK(end+1) = hd;
                kL{end+1} = sprintf('Displaced threshold (%.0f m in)', RW.dthr);
            end
        end
        lg = legend(a, hK, kL, 'AutoUpdate', 'off', 'Orientation', 'horizontal', 'FontSize', 10);
        try
            lg.NumColumns = 2;  lg.Layout.Tile = 'south';     % R2020b+: one key under all four panels
        catch
            lg.Location = 'southoutside';
        end
        mark_fig(f, E, 1, dist - dist(iTD), w);
    end
end

%% ---- 7  battery on the track (v0.7: linked map + time line, one colour scale) -----------------
if hasB
    f = battery_track_fig('SWIFT SITL - battery on track', vis, t, pos, h, h0, Ipk, Ppk, SOC, Vpk, ...
                          WP, E, hasE, C);
    figs(end+1) = f;
end

%% ---- 8  events list (v0.5) ---------------------------------------------------------------
if hasE
    figs = [figs, swift_events_figure(E, o.tag, vis)];
end

%% ---- summary ------------------------------------------------------------------------------------
dtm = mean(diff(t));
%  Airborne samples only: before ArduPlane finishes booting it holds every
%  output at 1000 us for about 1.5 s, which used to count as "at the stop".
sat = 100 * mean(abs(r2d*dl(air,1:3)) >= 0.98 * dm, 1);  % % of airborne time at the stop
R = struct();
R.tag            = o.tag;
R.duration_s     = t(end) - t(1);
R.max_alt_m      = max(h - h0);
R.min_V_air_ms   = min(V(air));
R.min_stall_margin = min(V(air)) / Vs;
R.t_below_1p1Vs  = sum(air & V < 1.1*Vs) * dtm;
R.max_bank_deg   = max(abs(r2d*eul(air,1)));
R.max_pitch_deg  = max(r2d*eul(air,2));
R.max_alpha_deg  = max(r2d*al(air));
R.max_nz_g       = max(nz(air));
R.takeoff_m      = xTO;
R.takeoff_s      = tTO;
R.sat_pct        = sat;                              % [ail elev rud]
R.thr_mean_air   = 100 * mean(dl(air,4));
R.thr_max        = 100 * max(dl(:,4));
R.ap_log         = AP.ok;
R.touchdown_V    = LD.V;                             % m/s
R.touchdown_sink = LD.sink;                          % m/s
R.touchdown_x_m  = LD.x_thr;                         % m along the runway from the start point
R.rollout_m      = LD.rollout;
R.threshold_h_m  = LD.thr_h;                         % height crossing the runway end
R.threshold_V    = LD.thr_V;                         % m/s
R.threshold_to_td_m = LD.thr_to_td;                  % m flown from the runway end to touchdown
R.batt           = struct();
R.events         = E;

fprintf('\n=== %s ===\n', R.tag);
fprintf('  Duration              %8.1f s\n',  R.duration_s);
fprintf('  Max altitude          %8.1f m\n',  R.max_alt_m);
fprintf('  Min airspeed (air)    %8.1f m/s   (%.2f x stall)\n', R.min_V_air_ms, R.min_stall_margin);
fprintf('  Time below 1.1 Vs     %8.1f s\n',  R.t_below_1p1Vs);
fprintf('  Max bank / pitch      %8.1f / %.1f deg\n', R.max_bank_deg, R.max_pitch_deg);
fprintf('  Max alpha             %8.1f deg   (stall %.0f)\n', R.max_alpha_deg, rad2deg(P.aero.alpha_stall));
fprintf('  Max load factor       %8.2f g\n',  R.max_nz_g);
fprintf('  Takeoff               %8.0f m in %.1f s\n', R.takeoff_m, R.takeoff_s);
fprintf('  At travel stop        A %.0f%%  E %.0f%%  R %.0f%% of the time\n', sat);
fprintf('  Throttle              mean %.0f%% airborne, max %.0f%%\n', R.thr_mean_air, R.thr_max);
if LD.ok
    fprintf('  Touchdown             %8.1f m/s, %.2f m/s sink, %.0f m down the runway\n', ...
            LD.V, LD.sink, LD.x_thr);
    fprintf('  Rollout               %8.0f m\n', LD.rollout);
    if LD.thr_ok
        fprintf('  Over the runway       %8.1f m high, %.1f m/s, %.1f m off centreline, %.0f m before touchdown\n', ...
                LD.thr_h, LD.thr_V, LD.thr_y, LD.thr_to_td);
    else
        fprintf('  Over the runway       threshold crossing not found (not lined up, or runway length unknown)\n');
    end
elseif onGnd0
    fprintf('  Touchdown             none found (run stopped in the air?)\n');
end
if ~AP.ok && ~isempty(o.aplog), fprintf('  ArduPilot log         could not be read (%s)\n', AP.msg); end
if hasE
    fprintf('  Events                %d numbered points (%d mission items) - list figure + events.csv\n', ...
            numel(E.t), E.n_mission);
    if ~isempty(E.note), fprintf('  Events note           %s\n', E.note); end
end

%  Battery: the full 4-panel figure and summary, same tag = same run folder.
if hasB && ~isempty(which('swift_battery_plot'))
    try
        bo = struct('out_batt', struct('time', tb, 'signals', struct('values', yb)));
        R.batt = swift_battery_plot(bo, 'tag', o.tag, 'save', o.save, 'show', o.show, 'events', E);
    catch err
        fprintf(2, '  Battery plot failed: %s\n', err.message);
    end
elseif ~hasB
    fprintf('  Battery               no out_batt in this run (swift_battery_add swift_6dof_sitl)\n');
end

for k = 1:numel(figs), readable(figs(k)); end   % black text before showing/saving

%% ---- save ----------------------------------------------------------------------------------------
if o.save
    d = fullfile(fileparts(fileparts(here)), 'data', 'runs', o.tag);   % sitl/analysis -> ArduPilot_SITL/data/runs
    if ~isfolder(d), mkdir(d); end
    for k = 1:numel(figs)
        nm = regexprep(lower(get(figs(k), 'Name')), '[^a-z0-9]+', '_');
        try
            exportgraphics(figs(k), fullfile(d, [nm '.png']), 'Resolution', 200);
        catch
            saveas(figs(k), fullfile(d, [nm '.png']));
        end
    end
    S = struct('t', t, 'pos', pos, 'V', V, 'eul', eul, 'om', om, 'alpha', al, ...
               'beta', be, 'qbar', qb, 'delta', dl, 'accel', acc, 'P', P, 'R', R, ...
               'batt_t', tb, 'batt', yb, 'events', E); %#ok<NASGU>
    save(fullfile(d, 'signals.mat'), '-struct', 'S');
    if hasE, write_events_csv(fullfile(d, 'events.csv'), E); end
    fid = fopen(fullfile(d, 'summary.txt'), 'w');
    fprintf(fid, '%s\n', evalc('disp(R)'));  fclose(fid);
    fprintf('  Saved to %s\n\n', d);
end
end

% =========================================================================
function mark_fig(f, E, scale, xv, w)
%  Event markers on every axes of a figure; numbers only on the top row.
if isempty(E) || isempty(E.t) || exist('swift_mark_events', 'file') ~= 2, return, end
if nargin < 4, xv = []; w = []; end
axs = findobj(f, 'Type', 'axes');
axs = axs(~ismember(get(axs, 'Tag'), {'legend', 'colorbar'}));   % Octave draws these as axes
if isempty(axs), return, end
pos = get(axs, 'Position');  if iscell(pos), pos = cell2mat(pos); end   % one axes -> plain vector
for k = 1:numel(axs)
    same = abs(pos(:,1) - pos(k,1)) < 0.05;              % axes in the same column
    top  = pos(k,2) >= max(pos(same, 2)) - 1e-6;
    swift_mark_events(axs(k), E, 'scale', scale, 'labels', top, 'x', xv, 'win', w);
end
end

function write_events_csv(file, E)
fid = fopen(file, 'w');
if fid < 0, return, end
fprintf(fid, 'num,time_s,kind,item,what,alt_m,airspeed_ms,throttle_pct,current_A,SOC_pct\n');
for k = 1:numel(E.t)
    fprintf(fid, '%d,%.2f,%s,%s,"%s",%.1f,%.2f,%.1f,%.2f,%.2f\n', E.num(k), E.t(k), E.kind{k}, E.item{k}, ...
            strrep(E.what{k}, '"', ''''), E.alt(k), E.V(k), E.thr(k), E.I(k), E.SOC(k));
end
fclose(fid);
end

function f = newfig(name, vis)
f = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, ...
           'Position', [80 80 1100 760]);
try, theme(f, 'light'); catch, end   % readable on screen too (R2025a+)
end

function mark_lift(ax, t, i, C)
if ~isempty(i), xline(ax, t(i), '-', 'liftoff', 'Color', C.grey, 'LabelVerticalAlignment', 'bottom'); end
grid(ax, 'on');
end

function y = resample_to(out, name, t, n)
[ts, ys] = sig(out, name);
if isempty(ts)
    y = nan(numel(t), n);  return
end
if size(ys, 2) < n, ys(:, end+1:n) = NaN; end
y = interp1(ts, ys(:, 1:n), t, 'linear', 'extrap');
end

function [t, y] = sig(o, name)
t = [];  y = [];  s = [];
if isa(o, 'Simulink.SimulationOutput')
    try, s = o.get(name); catch, s = []; end
elseif isstruct(o) && isfield(o, name)
    s = o.(name);                          % a saved run rebuilt as a struct
end
if isempty(s) && evalin('base', sprintf('exist(''%s'',''var'')', name))
    s = evalin('base', name);
end
if isempty(s), return, end
if isa(s, 'timeseries')
    t = s.Time;  y = squeeze(s.Data);
elseif isstruct(s)
    t = s.time;  y = squeeze(s.signals.values);
else
    y = s;  t = (0:size(y,1)-1)' * 0.01;
end
if isvector(y), y = y(:); end
if size(y, 1) ~= numel(t), y = y.'; end
[t, iu] = unique(t(:));  y = y(iu, :);
end

function WP = read_wp(file)
%  North/east (m) of every point the aircraft flies to, relative to HOME:
%  WAYPOINT 16, LOITER 17/18/19, LAND 21, LOITER_TO_ALT 31 (v0.2).
WP = [];
if isempty(file) || ~isfile(file), return, end
L = strsplit(fileread(file), {'\r\n', '\n'});
H = [];  A = zeros(0, 2);
for k = 2:numel(L)
    v = sscanf(L{k}, '%f');
    if numel(v) < 11, continue, end
    if v(1) == 0, H = v(9:10)'; elseif any(v(4) == [16 17 18 19 21 31]) && any(v(9:10)), A(end+1, :) = v(9:10)'; end %#ok<AGROW>
end
if isempty(H) || isempty(A), return, end
Re = 6378137;
WP = [deg2rad(A(:,1) - H(1)) * Re, deg2rad(A(:,2) - H(2)) * Re * cosd(H(1))];
end

function x = runway_x(ne, P)
%  Distance along the runway from the parked start point (the threshold),
%  along the parked heading P.sitl.hdg0 (125.55 deg for KCUB runway 13).
hdg = 0;
if isfield(P, 'sitl') && isfield(P.sitl, 'hdg0'), hdg = P.sitl.hdg0; end
x = ne(1) * cosd(hdg) + ne(2) * sind(hdg);
end

function RW = runway_info(A, P)
%  Runway used for the landing marks: heading from the mission's airport data
%  (S.airport from swift_sitl_cub_d), else P.sitl.hdg0; length, displaced
%  threshold and width when the mission gives them (width default 75 ft).
RW = struct('hdg', 0, 'len', NaN, 'dthr', 0, 'halfw', 0.5 * 75 * 0.3048);
if isfield(P, 'sitl') && isfield(P.sitl, 'hdg0'), RW.hdg = P.sitl.hdg0; end
if isstruct(A)
    if isfield(A, 'hdg'),   RW.hdg   = A.hdg;   end
    if isfield(A, 'len'),   RW.len   = A.len;   end
    if isfield(A, 'dthr'),  RW.dthr  = A.dthr;  end
    if isfield(A, 'width'), RW.halfw = A.width / 2; end
end
end

function [x, y] = rwy_xy(ne, hdg)
%  North/east (m) to the runway frame: x along the runway heading, y to the right.
x =  ne(:,1) * cosd(hdg) + ne(:,2) * sind(hdg);
y = -ne(:,1) * sind(hdg) + ne(:,2) * cosd(hdg);
end

function AP = load_aplog(file, shift)
%  Mission Planner > DataFlash Logs > Create Matlab File  ->  ATT, ATT_label
AP = struct('ok', false, 'msg', '');
try
    S = load(file);
    lab = S.ATT_label;  A = S.ATT;
    if ischar(lab), lab = cellstr(lab); end
    lab = strtrim(lab);
    c = @(n) find(strcmp(lab, n), 1);
    AP.att.t = A(:, c('TimeUS')) / 1e6 + shift;
    for n = {'DesRoll', 'Roll', 'DesPitch', 'Pitch'}
        i = c(n{1});
        if ~isempty(i), AP.att.(n{1}) = A(:, i); end
    end
    AP.ok = true;
catch err
    AP.msg = err.message;
end
end

% =========================================================================
function readable(f)
%READABLE  Black, easy-to-read text on a white figure, whatever MATLAB theme is on.
%  MATLAB R2025a+ in dark theme draws light-grey text that almost vanishes on
%  the white background these figures are saved with. This forces the light
%  theme, black text, a white plot area and slightly larger fonts so the
%  saved PNGs read well on slides. Coloured yyaxis sides keep their colours.
if ~isgraphics(f), return; end
try, theme(f, 'light'); catch, end            % R2025a+; older releases skip this
k = [0 0 0];
set(f, 'Color', 'w');
for a = findall(f, 'Type', 'axes')'
    if ~(ischar(a.Color) && strcmp(a.Color, 'none')), a.Color = 'w'; end   % keep see-through overlays
    set(a, 'XColor', k, 'ZColor', k, 'GridColor', [0.15 0.15 0.15], 'FontSize', max(a.FontSize, 11));
    if numel(a.YAxis) == 1, a.YColor = k; end
    a.Title.Color = k;   a.Title.FontSize = max(a.Title.FontSize, 12);
end
set(findall(f, 'Type', 'legend'), 'TextColor', k, 'Color', 'w', 'EdgeColor', [0.6 0.6 0.6]);
set(findall(f, 'Type', 'colorbar'), 'Color', k);
set(findall(f, 'Type', 'subplottext'), 'Color', k);   % sgtitle
end

function f = battery_track_fig(name, vis, t, pos, h, h0, Ipk, Ppk, SOC, Vpk, WP, E, hasE, C) %#ok<INUSL>
%BATTERY_TRACK_FIG  Battery use along the flight (v0.9): one image, two linked panels.
%  Top: ground track coloured by pack current. Bottom: pack current against
%  time, coloured with the same scale, over a grey altitude profile. One tall
%  colorbar reads both panels, and the numbered events use the same filled
%  badges in both (and in the flight-events figure), so point 21 on the map is
%  point 21 on the time line. Every badge is kept clear of the other badges,
%  the event dots, the traces and the leader lines, for any flight.
W  = 1380;  L = 100;  mw = 1010;                   % figure width, left margin, plot width (px)
[ex, nx, a] = map_frame(pos, WP);
mh = round(mw * a);  bh = 330;                     % map and lower box heights (px)
top = 132;  gap = 112;  bot = 78;
H  = top + mh + gap + bh + bot;
cmap = turbo_or_jet();
Imax = max(5, ceil(max(Ipk) / 10) * 10);           % shared colour limit, A

f = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, 'Position', [40 20 W H]);
try, theme(f, 'light'); catch, end
sub = sprintf(['Colour = pack current in both panels  |  SOC %.0f%% \\rightarrow %.0f%%  |  ' ...
               'peak %.0f A  |  min pack %.1f V'], SOC(1), SOC(end), max(Ipk), min(Vpk));
header(f, W, H, L, 'Battery use along the flight', sub);

% ---- map -------------------------------------------------------------------
ax = axes('Parent', f, 'Units', 'pixels', 'Position', [L, bot + bh + gap, mw, mh]);
draw_track_map(ax, [mw mh], pos, Ipk, cmap, [0 Imax], ex, nx, WP, false, E, hasE, C);
title(ax, 'Where the current goes', 'FontSize', 12, 'FontWeight', 'normal');

% ---- time line: altitude backdrop (right axis), current on top (left axis) --
tm = t / 60;  ha = h - h0;  hmax = max([ha; 10]);
xl = [tm(1) tm(end)];
axA = axes('Parent', f, 'Units', 'pixels', 'Position', [L, bot, mw, bh], 'Color', 'w', ...
           'YAxisLocation', 'right', 'XLim', xl, 'YLim', [0 1.55 * hmax], 'FontSize', 11, ...
           'XTickLabel', [], 'TickDir', 'out', 'Box', 'off');
hold(axA, 'on');
fill(axA, [tm; flipud(tm)], [max(ha, 0); zeros(size(ha))], [0.55 0.55 0.55], ...
     'FaceAlpha', 0.13, 'EdgeColor', 'none');
plot(axA, tm, max(ha, 0), '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.1);
ylabel(axA, 'Altitude (m, grey)');
ytk = get(axA, 'YTick');  set(axA, 'YTick', ytk(ytk <= hmax * 1.05));   % no altitude ticks in the badge band

Itop = max(1.30 * max(Ipk), 1);                   % a run with no current still gets a valid axis
axI = axes('Parent', f, 'Units', 'pixels', 'Position', [L, bot, mw, bh], 'Color', 'none', ...
           'XLim', xl, 'YLim', [0 Itop], 'FontSize', 11, 'TickDir', 'out', 'Box', 'on');
hold(axI, 'on');  grid(axI, 'on');
cline(axI, tm, Ipk, Ipk, 1.6);
colormap(axI, cmap);  caxis(axI, [0 Imax]);
ylabel(axI, 'Pack current (A, colour)');
xlabel(axI, 'Time from start of log (min)');
title(axI, 'When the current is drawn', 'FontSize', 12, 'FontWeight', 'normal');

if hasE                                            % badges sit on the altitude line
    axB = axes('Parent', f, 'Units', 'pixels', 'Position', [L, bot, mw, bh], 'Color', 'none', ...
               'XLim', xl, 'YLim', [0 1.55 * hmax], 'Visible', 'off', 'HitTest', 'off');
    hold(axB, 'on');
    px = @(v) (v - xl(1)) / diff(xl) * mw;
    yA = [0 1.55 * hmax];  yI = [0 Itop];
    pa = @(v) (v - yA(1)) / diff(yA) * bh;  pi_ = @(v) (v - yI(1)) / diff(yI) * bh;
    lines = {[px(tm) pa(max(ha, 0))], [px(tm) pi_(Ipk)]};      % altitude and current traces
    place_badges(axB, E.t / 60, max(E.alt, 0), E, [mw bh], 'up', lines, [px(E.t(:)/60) pa(max(E.alt(:), 0))]);
end

side_colorbar(ax, 'Pack current (A), both panels', [L + mw + 82, bot, 22, mh + gap + bh]);
end

function f = ground_track_fig(name, vis, t, pos, h, h0, WP, E, hasE, C) %#ok<INUSL>
%GROUND_TRACK_FIG  Ground track (v0.9), drawn exactly like the battery map:
%  track coloured by altitude above the start point, the planned route dashed
%  grey, the start square, and the same overlap-free numbered badges.
W  = 1380;  L = 100;  mw = 1010;
[ex, nx, a] = map_frame(pos, WP);
mh = round(mw * a);  top = 110;  bot = 78;
H  = top + mh + bot;
ha = h - h0;  hmax = max([ha; 10]);
km = sum(hypot(diff(pos(:,1)), diff(pos(:,2)))) / 1000;

f = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', 'Visible', vis, 'Position', [40 20 W H]);
try, theme(f, 'light'); catch, end
sub = sprintf('Colour = altitude above start  |  dashed grey = planned route  |  %.1f km flown  |  max %.0f m', ...
              km, max(ha));
if isempty(WP), sub = strrep(sub, '  |  dashed grey = planned route', ''); end
header(f, W, H, L, 'Ground track', sub);

ax = axes('Parent', f, 'Units', 'pixels', 'Position', [L, bot, mw, mh]);
draw_track_map(ax, [mw mh], pos, max(ha, 0), parula(256), [0 ceil(hmax / 10) * 10], ex, nx, WP, true, E, hasE, C);
side_colorbar(ax, 'Altitude above start (m)', [L + mw + 82, bot, 22, mh]);
end

function [ex, nx, a] = map_frame(pos, WP)
%  Map limits (m) around the track and the mission, padded, and the box shape
%  (height / width, kept between 0.45 and 1.05) so the map fills its width.
E_ = pos(:,2);  N_ = pos(:,1);
ex = [min(E_) max(E_)];  nx = [min(N_) max(N_)];
if ~isempty(WP), ex = [min([ex WP(:,2)']) max([ex WP(:,2)'])];  nx = [min([nx WP(:,1)']) max([nx WP(:,1)'])]; end
sp = max(diff(ex), diff(nx));  pad = 0.07 * sp + 30;
ex = ex + [-pad pad];  nx = nx + [-pad pad];
a  = min(max(diff(nx) / diff(ex), 0.45), 1.05);
if diff(nx) / diff(ex) < a                         % widen the short span to fill the box
    c = mean(nx);  nx = c + [-0.5 0.5] * a * diff(ex);
elseif diff(nx) / diff(ex) > a
    c = mean(ex);  ex = c + [-0.5 0.5] * diff(nx) / a;
end
end

function draw_track_map(ax, axpx, pos, cv, cmap, clim, ex, nx, WP, route, E, hasE, C)
%  Colour-coded track, mission points (route = true also joins them dashed),
%  start square and the numbered badges, on an axes of axpx pixels.
E_ = pos(:,2);  N_ = pos(:,1);
hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
cline(ax, E_, N_, cv, 2.6);
px = @(v) (v - ex(1)) / diff(ex) * axpx(1);  py = @(v) (v - nx(1)) / diff(nx) * axpx(2);
obst = {[px(E_) py(N_)]};
if ~isempty(WP)
    if route
        plot(ax, WP(:,2), WP(:,1), '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
        obst{end+1} = [px(WP(:,2)) py(WP(:,1))];
    end
    plot(ax, WP(:,2), WP(:,1), 'o', 'Color', C.grey, 'MarkerFaceColor', 'w', 'MarkerSize', 4.5, 'LineStyle', 'none');
end
plot(ax, pos(1,2), pos(1,1), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
colormap(ax, cmap);  caxis(ax, clim);
set(ax, 'XLim', ex, 'YLim', nx, 'DataAspectRatio', [1 1 1], 'FontSize', 11, 'Layer', 'top');
xlabel(ax, 'East of start (m)');  ylabel(ax, 'North of start (m)');
if hasE
    dots = [px(E.e(:)) py(E.n(:)); px(pos(1,2)) py(pos(1,1))];
    if ~isempty(WP), dots = [dots; px(WP(:,2)) py(WP(:,1))]; end
    place_badges(ax, E.e, E.n, E, axpx, 'around', obst, dots);
end
end

function header(f, W, H, L, ttl, sub)
%  Figure title and grey subtitle, top left, in pixels.
ov = axes('Parent', f, 'Units', 'pixels', 'Position', [0 0 W H], 'XLim', [0 W], 'YLim', [0 H], ...
          'YDir', 'reverse', 'Visible', 'off', 'HitTest', 'off');
text(ov, L - 60, 38, ttl, 'FontSize', 18, 'FontWeight', 'bold', 'Color', [0 0 0]);
text(ov, L - 60, 68, sub, 'FontSize', 11.5, 'Color', [0.30 0.30 0.30]);
end

function side_colorbar(ax, lab, posPx)
%  Colorbar at a fixed pixel spot; colorbar() shrinks its axes, so put it back.
p0 = get(ax, 'Position');
cb = colorbar(ax);
set(ax, 'Position', p0);
cb.Label.String = lab;  cb.Label.FontSize = 12;
set(cb, 'Units', 'pixels', 'Position', posPx, 'FontSize', 11);
end

% =========================================================================
function cline(ax, x, y, c, lw)
%  Line coloured point by point (same colormap and limits as the axes).
x = x(:);  y = y(:);  c = c(:);
patch(ax, [x; NaN], [y; NaN], [c; NaN], 'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', lw);
end

function place_badges(ax, x, y, E, axpx, mode, lines, dots)
%  Filled number badges with leader lines. Every badge is kept clear of the
%  other badges, every event dot, the plotted traces and the leader lines
%  already drawn, using a 2 px occupancy grid over the axes. Candidate spots
%  are tried nearest first; if a crowded flight leaves no clear spot, the
%  badge goes where it covers the least (badges never sit on badges first).
%  'up'     : above the point first (time line), then sideways, then below
%  'around' : rings of positions around the point (map)
%  lines    : cell of [px py] traces in axes pixels;  dots : [px py] points
cs = 2;  R = 13.5;                                 % grid cell (px), badge radius (px)
nc = ceil(axpx / cs);  G = zeros(nc(2), nc(1));    % 0 free, 1 trace/dot/leader, 2 badge
for k = 1:numel(lines), G = rast(G, lines{k}(:,1), lines{k}(:,2), 2.5, cs, 1); end
for k = 1:size(dots, 1), G = stamp(G, dots(k,1), dots(k,2), 5, cs, 1); end
xl = get(ax, 'XLim');  yl = get(ax, 'YLim');
P  = [(x(:) - xl(1)) / diff(xl) * axpx(1), (y(:) - yl(1)) / diff(yl) * axpx(2)];
D  = 2 * R;
if strcmp(mode, 'up')
    cand = zeros(0, 2);
    for r = 1.15:0.55:12                           % straight up, then up and to the side
        for s = [0 -0.6 0.6 -1.2 1.2]
            cand(end+1, :) = [s r] * D; %#ok<AGROW>
        end
    end
    for r = 1.15:0.55:4, cand(end+1, :) = [0 -r] * D; end %#ok<AGROW>
else
    ang = (45 + [0 90 180 270 45 135 225 315 22.5 67.5 112.5 157.5 202.5 247.5 292.5 337.5]) * pi/180;
    cand = zeros(0, 2);
    for r = [1.2 1.7 2.3 3.0 3.8 4.8 6.0 7.5]
        cand = [cand; r * D * [cos(ang') sin(ang')]]; %#ok<AGROW>
    end
end
[ox, oy] = meshgrid(-ceil((R + 3) / cs):ceil((R + 3) / cs));
disk = [ox(:) oy(:)];  disk = disk(hypot(disk(:,1), disk(:,2)) * cs <= R + 3, :);
for k = 1:size(P, 1)
    best = [];  bestCost = inf;
    for m = 1:size(cand, 1)
        c = P(k, :) + cand(m, :);
        if c(1) < R + 2 || c(1) > axpx(1) - R - 2 || c(2) < R + 2 || c(2) > axpx(2) - R - 2, continue, end
        v = G(cells(c, disk, cs, nc));
        if ~any(v), best = c;  break, end
        cost = 1000 * sum(v == 2) + sum(v == 1) + m * 1e-3;   % badge overlap is the worst case
        if cost < bestCost, bestCost = cost;  fall = c; end
    end
    if isempty(best)
        if isinf(bestCost), fall = [min(max(P(k,1), R + 2), axpx(1) - R - 2), min(P(k,2) + D, axpx(2) - R - 2)]; end
        best = fall;
    end
    G(cells(best, disk(hypot(disk(:,1), disk(:,2)) * cs <= R + 1, :), cs, nc)) = 2;
    G = rast(G, [P(k,1); best(1)], [P(k,2); best(2)], 1, cs, 1);   % leader line
    lx = xl(1) + best(1) / axpx(1) * diff(xl);  ly = yl(1) + best(2) / axpx(2) * diff(yl);
    c = kcol(E.kind{k});
    line(ax, [x(k) lx], [y(k) ly], 'Color', 1 - 0.55 * (1 - c), 'LineWidth', 0.9);
    line(ax, x(k), y(k), 'Marker', 'o', 'MarkerSize', 4.5, 'MarkerFaceColor', c, ...
         'MarkerEdgeColor', 'w', 'LineStyle', 'none');
    line(ax, lx, ly, 'Marker', 'o', 'MarkerSize', 19, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', ...
         'LineWidth', 1, 'LineStyle', 'none');
    text(ax, lx, ly, sprintf('%d', E.num(k)), 'FontSize', 8.5 - 1.5 * (E.num(k) > 99), ...
         'FontWeight', 'bold', 'Color', 'w', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
end
end

function ix = cells(c, disk, cs, nc)
%  Linear indices of the grid cells under a disk centred at c (px).
col = floor(c(1) / cs) + 1 + disk(:,1);  row = floor(c(2) / cs) + 1 + disk(:,2);
ok  = col >= 1 & col <= nc(1) & row >= 1 & row <= nc(2);
ix  = sub2ind([nc(2) nc(1)], row(ok), col(ok));
end

function G = stamp(G, x, y, r, cs, val)
[ox, oy] = meshgrid(-ceil(r / cs):ceil(r / cs));
d = [ox(:) oy(:)];  d = d(hypot(d(:,1), d(:,2)) * cs <= r, :);
ix = cells([x y], d, cs, [size(G, 2) size(G, 1)]);
G(ix) = max(G(ix), val);
end

function G = rast(G, x, y, r, cs, val)
%  Mark a polyline (px) on the grid, thickened to radius r (px).
ok = isfinite(x) & isfinite(y);  x = x(ok);  y = y(ok);
if numel(x) < 2, return, end
s = [0; cumsum(hypot(diff(x(:)), diff(y(:))))];
[s, iu] = unique(s);  x = x(iu);  y = y(iu);
if numel(s) < 2, return, end
si = (0:cs/2:s(end))';
xi = interp1(s, x(:), si);  yi = interp1(s, y(:), si);
[ox, oy] = meshgrid(-ceil(r / cs):ceil(r / cs));
d = [ox(:) oy(:)];  d = d(hypot(d(:,1), d(:,2)) * cs <= r, :);
col = floor(xi / cs) + 1;  row = floor(yi / cs) + 1;
nc = [size(G, 2) size(G, 1)];
C = col + d(:,1)';  Rw = row + d(:,2)';
ok = C >= 1 & C <= nc(1) & Rw >= 1 & Rw <= nc(2);
ix = sub2ind([nc(2) nc(1)], Rw(ok), C(ok));
G(ix) = max(G(ix), val);
end

function c = kcol(kind)
%  Same colours as swift_mark_events and swift_events_figure.
switch kind
    case 'mission', c = [0.20 0.33 0.55];
    case 'battery', c = [0.80 0.15 0.15];
    otherwise,      c = [0.90 0.45 0.10];
end
end

function m = turbo_or_jet()
try, m = turbo(256); catch, m = jet(256); end
end
