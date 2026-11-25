function V = swift_flight_video(out, varargin)
%SWIFT_FLIGHT_VIDEO  MP4 of a whole swift_6dof_sitl flight with live data panels.
%
%   swift_flight_video                    video of 'out' in the base workspace
%   swift_flight_video(out)               a SimulationOutput you kept
%   V = swift_flight_video(out, Name, Value, ...)
%
%   LAYOUT (1920 x 1080 design, scaled to 'size')
%     centre   3D chase view: SWIFT model, its trail and ground shadow, the
%              runway and the planned route; mini-map of the whole mission in
%              the corner with the flown part and the aircraft
%     left     flight instruments: artificial horizon (roll and pitch),
%              heading tape, airspeed, altitude, climb rate, groundspeed
%              The model is the SWIFT layout: twin booms, inverted-V tail
%              with the two ruddervators moving as logged (out_rv), pusher prop.
%              V-tail inset (lower left of the view): rear view of the tail and
%              side sections of the left and right ruddervator with their angles
%     right    aero and power: throttle, angle of attack against stall,
%              lift-to-drag ratio (L/D), load factor (n_z), pack current;
%              battery: state of charge (SOC) and pack voltage
%     bottom   timeline: altitude profile of the whole flight with the same
%              numbered event badges as every figure, a moving cursor, and
%              the last event passed written out
%
%   OPTIONS
%     'tag'    auto     run folder; the video is data/runs/<tag>/swift_flight.mp4
%     'speed'  10       playback speed (10 = ten times real time)
%     'fps'    30       video frame rate
%     'size'   auto     [W H] in px; 1920x1080 if the screen fits it, else 1280x720
%     'span'   []       [t0 t1] minutes from the start of the log, to film only part
%     'wp'     auto     .waypoints file (default S.wp_file, as swift_plot_flight)
%     'scale'  2        aircraft model size multiplier (easier to see)
%     'chase'  [20 5]    camera distance behind and height above the aircraft (m)
%     'show'   true     false = draw the frames without showing the window
%     'file'   ''       full output path instead of the run folder
%     'at'     []       minutes: write only these moments, as PNG stills, to
%                       'pngdir' (default data/runs/<tag>/video_frames)
%
%   Lift and drag come from the logged alpha, dynamic pressure, pitch rate and
%   elevator with the P.aero equations the model uses (as the aero figures).
%   Times are in minutes from the start of the log, as on every figure.
%
%   Needs MATLAB VideoWriter (core MATLAB). MPEG-4 is written on Windows and
%   macOS; elsewhere it falls back to Motion JPEG AVI.
%
%   Ryan Bilock, ARTS Lab | SITL plotting | v0.3
%     v0.3  tail drawn from the CAD numbers in swift_vtail.m with the two
%           ruddervators moving as logged; V-tail inset with both
%           ruddervator angles (from out_rv, or mixed from out_delta when
%           out_rv was not logged)
%     v0.2  model matches the real SWIFT: red pod and wing, twin booms,
%           inverted-V tail, pusher prop

%% ---- options -----------------------------------------------------------------
if nargin < 1 || isempty(out)
    if evalin('base', 'exist(''out'',''var'')'), out = evalin('base', 'out'); else, out = []; end
end
here = fileparts(mfilename('fullpath'));
o.tag = ['run_' datestr(now, 'yyyymmdd_HHMM')];  o.speed = 10;  o.fps = 30;  o.size = [];
o.span = [];  o.wp = '';  o.scale = 2;  o.chase = [20 5];  o.show = true;  o.file = '';
o.pngdir = '';  o.at = [];                                   % test hook: write PNG frames instead of a video
for k = 1:2:numel(varargin), o.(lower(varargin{k})) = varargin{k+1}; end
mono = mono_font();                                % Consolas on Windows, portable monospace elsewhere
gotWP = ~isempty(o.wp);
Sb = [];
if evalin('base', 'exist(''S'',''var'')'), Sb = evalin('base', 'S'); end
if ~gotWP && isstruct(Sb) && isfield(Sb, 'wp_file') && isfile(Sb.wp_file), o.wp = Sb.wp_file; end
names = {};  Arp = [];
if isstruct(Sb) && isfield(Sb, 'wp_file') && strcmp(Sb.wp_file, o.wp)
    if isfield(Sb, 'seg'), names = Sb.seg(:, 1); end
    if isfield(Sb, 'airport'), Arp = Sb.airport; end
end
if ~evalin('base', 'exist(''P'',''var'')'), evalin('base', 'swift_placeholder;'); end
P = evalin('base', 'P');

%% ---- signals --------------------------------------------------------------------
[t, pos] = sig(out, 'out_pos');
if numel(t) < 2, error('swift_flight_video:nolog', 'No out_pos found - run the model first.'); end
rs  = @(name, n) resample_to(out, name, t, n);
Va  = rs('out_V', 1);   eul = rs('out_euler', 3);  om = rs('out_om', 3);
al  = rs('out_alpha', 1);  qb = rs('out_qbar', 1);  dl = rs('out_delta', 4);  acc = rs('out_accel', 3);
eul(isnan(eul)) = 0;  om(isnan(om)) = 0;  al(isnan(al)) = 0;  dl(isnan(dl)) = 0;
[trv, yrv] = sig(out, 'out_rv');                 % V-tail ruddervators [left right], rad
rvlim = deg2rad(25);  if isfield(P, 'tail') && isfield(P.tail, 'drv_max'), rvlim = P.tail.drv_max; end
hasRV = ~isempty(trv) && size(yrv, 2) >= 2;
if hasRV
    rv = interp1(trv, yrv(:, 1:2), t, 'linear', 'extrap');
else
    rv = min(max([dl(:,2) + dl(:,3), dl(:,2) - dl(:,3)], -rvlim), rvlim);   % what the V-tail mix gives
end
rv(isnan(rv)) = 0;
[tb, yb] = sig(out, 'out_batt');
hasB = ~isempty(tb) && size(yb, 2) == 10;
if hasB, bt = interp1(tb, yb, t, 'linear', 'extrap'); else, bt = nan(numel(t), 10); end

g  = P.const.g;  Vs = P.perf.V_stall;
h  = -pos(:,3);  h0 = h(1);  ha = h - h0;
vn = gradient(pos(:,1), t);  ve = gradient(pos(:,2), t);  vd = gradient(pos(:,3), t);
gs = hypot(vn, ve);
nz = -acc(:,3) / g;  nz(isnan(nz)) = 1;
c2V = P.geom.c ./ (2 * max(Va, 0.5));
CL = P.aero.CL0 + P.aero.CLalpha * al + P.aero.CLq * om(:,2) .* c2V + P.aero.CLde * dl(:,2);
if isfield(P.aero, 'k_ind'), k_ind = P.aero.k_ind;
else, k_ind = 1 / (pi * P.aero.e_osw * P.geom.b^2 / P.geom.S); end
CD = P.aero.CD0 + k_ind * CL.^2;
fly = Va > 0.6 * Vs;
LoD = CL ./ CD;  LoD(~fly) = NaN;
LDmax = 1 / (2 * sqrt(P.aero.CD0 * k_ind));
cells = 12;  if isfield(P, 'batt') && isfield(P.batt, 'cells'), cells = P.batt.cells; end

% track direction for the chase camera: velocity when moving, body heading otherwise
psi = eul(:,3);
trk = atan2(ve, vn);  trk(gs < 3) = psi(gs < 3);
dtm = median(diff(t));
nsm = max(1, round(2 / dtm));
cam_c = movmean(cos(trk), nsm);  cam_s = movmean(sin(trk), nsm);

%% ---- events, mission, runway ---------------------------------------------------------
E = [];
if exist('swift_flight_events', 'file') == 2
    try, E = swift_flight_events(out, P, 'wp', o.wp, 'names', names); catch, E = []; end
end
hasE = ~isempty(E) && ~isempty(E.t);
WP = read_wp3(o.wp);                               % [north east alt] above home
RW = runway_info(Arp, P);

%% ---- frame times ---------------------------------------------------------------------
dtf = o.speed / o.fps;                             % s of flight per frame
t0 = t(1);  t1 = t(end);
if ~isempty(o.span), t0 = max(t0, 60 * o.span(1));  t1 = min(t1, 60 * o.span(2)); end
tf = (t0:dtf:t1)';
if ~isempty(o.at), tf = 60 * o.at(:); end          % stills at chosen minutes (checks, slides)
if isempty(tf), tf = t0; end
nF = numel(tf);
ip = @(y) interp1(t, y, tf, 'linear', 'extrap');
F.n = ip(pos(:,1));  F.e = ip(pos(:,2));  F.u = ip(ha);
F.phi = ip(eul(:,1));  F.th = ip(eul(:,2));  F.psi = ip(unwrap(eul(:,3)));
F.V = ip(Va);  F.gs = ip(gs);  F.vz = ip(-vd);  F.al = ip(al);  F.thr = ip(dl(:,4));
F.nz = ip(nz);  F.LD = interp1(t, LoD, tf, 'nearest');  F.CL = ip(CL);
F.cc = ip(cam_c);  F.cs = ip(cam_s);
F.rvL = ip(rv(:,1));  F.rvR = ip(rv(:,2));  F.de = ip(dl(:,2));  F.dr = ip(dl(:,3));
F.soc = ip(100 * bt(:,1));  F.vp = ip(bt(:,2));  F.I = ip(bt(:,3));

%% ---- figure size -------------------------------------------------------------------------
sz = o.size;
if isempty(sz)
    scr = get(0, 'ScreenSize');
    if scr(3) >= 1940 && scr(4) >= 1140, sz = [1920 1080]; else, sz = [1280 720]; end
end
W = sz(1);  H = sz(2);  s = W / 1920;              % design units are 1920 x 1080
fs = @(v) v * s;                                   % font scaling
vis = 'on';  if ~o.show, vis = 'off'; end
col.bg    = [0.055 0.078 0.118];  col.panel = [0.090 0.122 0.176];  col.line = [0.20 0.26 0.35];
col.txt   = [0.93 0.95 0.98];     col.dim   = [0.60 0.66 0.75];   col.acc  = [0.30 0.70 1.00];
col.green = [0.30 0.80 0.45];     col.amber = [1.00 0.72 0.20];   col.red  = [0.95 0.32 0.30];
col.orng  = [0.95 0.55 0.15];

f = figure('Name', 'SWIFT flight video', 'NumberTitle', 'off', 'Color', col.bg, 'Visible', vis, ...
           'Units', 'pixels', 'Position', [20 40 W H], 'Resize', 'off', 'MenuBar', 'none', ...
           'ToolBar', 'none', 'InvertHardcopy', 'off');
try, theme(f, 'light'); catch, end
set(f, 'Color', col.bg);
nrm = @(p) [p(1) / 1920, p(2) / 1080, p(3) / 1920, p(4) / 1080];

% overlay: panels and all fixed text, in design pixels (y up)
ov = axes('Parent', f, 'Units', 'normalized', 'Position', [0 0 1 1], 'XLim', [0 1920], 'YLim', [0 1080], ...
          'Visible', 'off', 'HitTest', 'off');
hold(ov, 'on');
box_ = @(x, y, w, hh) patch(ov, x + [0 w w 0], y + [0 0 hh hh], col.panel, 'EdgeColor', col.line, 'LineWidth', 1);
box_(20, 250, 380, 770);  box_(1520, 250, 380, 770);  box_(20, 20, 1880, 215);
ttl = 'SWIFT UAV  |  simulated flight';
if ~isempty(o.tag), ttl = [ttl '  |  ' o.tag]; end
text(ov, 24, 1050, ttl, 'Color', col.txt, 'FontSize', fs(19), 'FontWeight', 'bold', 'Interpreter', 'none');
hClock = text(ov, 1896, 1050, '', 'Color', col.txt, 'FontSize', fs(19), 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'right', 'FontName', mono);
text(ov, 1896, 1028, sprintf('%g%s speed  |  time in minutes from the start of the log', o.speed, '\times'), ...
     'Color', col.dim, 'FontSize', fs(10.5), 'HorizontalAlignment', 'right');
hdr = @(x, y, str) text(ov, x, y, str, 'Color', col.acc, 'FontSize', fs(11), 'FontWeight', 'bold');
hdr(36, 996, 'FLIGHT INSTRUMENTS');  hdr(1536, 996, 'AERO AND POWER');  hdr(1536, 556, 'BATTERY');

%% ---- centre: 3D chase view -------------------------------------------------------------
pv = uipanel('Parent', f, 'Units', 'normalized', 'Position', nrm([420 250 1080 770]), ...
             'BorderType', 'none', 'BackgroundColor', [0.55 0.74 0.92]);   % clips the 3D view to its box
a3 = axes('Parent', pv, 'Units', 'normalized', 'Position', [0 0 1 1], ...
          'Color', [0.55 0.74 0.92], 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none', ...
          'Projection', 'perspective', 'DataAspectRatio', [1 1 1]);
hold(a3, 'on');
ext = [min(pos(:,2)) max(pos(:,2)) min(pos(:,1)) max(pos(:,1))];
if ~isempty(WP), ext = [min([ext(1); WP(:,2)]) max([ext(2); WP(:,2)]) min([ext(3); WP(:,1)]) max([ext(4); WP(:,1)])]; end
gpad = 3000;
gx = [ext(1) - gpad, ext(2) + gpad];  gy = [ext(3) - gpad, ext(4) + gpad];
patch(a3, gx([1 2 2 1]), gy([1 1 2 2]), -0.05 * [1 1 1 1], [0.40 0.58 0.34], 'EdgeColor', 'none');
gstep = 200;                                       % ground grid, m
for x = ceil(gx(1) / gstep) * gstep : gstep : gx(2)
    plot3(a3, [x x], gy, [0 0], '-', 'Color', [0.47 0.64 0.40], 'LineWidth', 0.5);
end
for y = ceil(gy(1) / gstep) * gstep : gstep : gy(2)
    plot3(a3, gx, [y y], [0 0], '-', 'Color', [0.47 0.64 0.40], 'LineWidth', 0.5);
end
draw_runway(a3, pos(1, 1:2), RW);
if ~isempty(WP)
    plot3(a3, WP(:,2), WP(:,1), WP(:,3), '--', 'Color', [0.96 0.96 0.96], 'LineWidth', 1.1);
    plot3(a3, WP(:,2), WP(:,1), WP(:,3), 'o', 'MarkerSize', 5, 'MarkerFaceColor', [1 1 1], ...
          'MarkerEdgeColor', [0.3 0.3 0.3], 'LineStyle', 'none');
end
nTr = max(2, round(60 / dtm));                     % trail: last 60 s of flight
hTrail  = plot3(a3, NaN, NaN, NaN, '-', 'Color', [0.95 0.35 0.20], 'LineWidth', 2.2);
hShadow = plot3(a3, NaN, NaN, NaN, '-', 'Color', [0.18 0.28 0.16], 'LineWidth', 2.0);
hDrop   = plot3(a3, NaN, NaN, NaN, ':', 'Color', [0.92 0.92 0.92], 'LineWidth', 0.8);
[Vb, Fc, Cc, MT] = swift_model(o.scale);
hAC = patch(a3, 'Vertices', Vb, 'Faces', Fc, 'FaceVertexCData', Cc, 'FaceColor', 'flat', ...
            'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 0.4);
set(a3, 'CameraViewAngle', 38, 'XLim', gx, 'YLim', gy, 'ZLim', [-50 max(ha) + 400]);

% mini-map inset
am = axes('Parent', pv, 'Units', 'normalized', 'Position', [(1262-420)/1080, (772-250)/770, 230/1080, 240/770], ...
          'Color', [0.08 0.11 0.16], 'XColor', 'none', 'YColor', 'none', 'Box', 'on');
hold(am, 'on');
plot(am, pos(:,2), pos(:,1), '-', 'Color', [0.45 0.50 0.58], 'LineWidth', 0.8);
if ~isempty(WP), plot(am, WP(:,2), WP(:,1), 'o', 'MarkerSize', 3, 'Color', [0.85 0.88 0.92], 'MarkerFaceColor', [0.85 0.88 0.92]); end
rwE = pos(1,2) + [0 RW.len_draw] * sind(RW.hdg);  rwN = pos(1,1) + [0 RW.len_draw] * cosd(RW.hdg);
plot(am, rwE, rwN, '-', 'Color', [0.9 0.9 0.9], 'LineWidth', 2.5);
hFlown = plot(am, NaN, NaN, '-', 'Color', col.orng, 'LineWidth', 1.6);
hMapAC = patch(am, NaN, NaN, col.txt, 'EdgeColor', 'k', 'LineWidth', 0.5);
mx = [min(pos(:,2)) max(pos(:,2))];  my = [min(pos(:,1)) max(pos(:,1))];
if ~isempty(WP), mx = [min([mx(1); WP(:,2)]) max([mx(2); WP(:,2)])];  my = [min([my(1); WP(:,1)]) max([my(2); WP(:,1)])]; end
sp = max([diff(mx), diff(my), 200]) * 0.56;
set(am, 'XLim', mean(mx) + [-sp sp], 'YLim', mean(my) + [-sp sp], 'DataAspectRatio', [1 1 1], ...
        'XTick', [], 'YTick', [], 'XColor', col.line, 'YColor', col.line);
text(am, mean(mx) - sp * 0.94, mean(my) + sp * 0.88, 'N', 'Color', col.dim, 'FontSize', fs(9), 'FontWeight', 'bold');
mapLen = sp * 0.09;                                % aircraft icon size on the map

% V-tail inset (lower left of the chase view): rear view + side sections of both ruddervators
av = axes('Parent', pv, 'Units', 'normalized', 'Position', [12/1080, 12/770, 300/1080, 210/770], ...
          'Color', [0.08 0.11 0.16], 'XLim', [0 300], 'YLim', [0 210], 'XTick', [], 'YTick', [], ...
          'XColor', col.line, 'YColor', col.line, 'Box', 'on');
hold(av, 'on');
text(av, 10, 196, 'V-TAIL', 'Color', col.acc, 'FontSize', fs(10), 'FontWeight', 'bold');
vtSub = 'ruddervators, + = trailing edge down';  if ~hasRV, vtSub = 'mixed from \delta_e, \delta_r (no out\_rv)'; end
text(av, 68, 196, vtSub, 'Color', col.dim, 'FontSize', fs(8));
vA = [150 172];  vT = [72 136; 228 136];          % rear view: apex up, tips at the booms (viewer = behind)
plot(av, [vT(1,1) vA(1) vT(2,1)], [vT(1,2) vA(2) vT(2,2)], '-', 'Color', [0.55 0.60 0.68], 'LineWidth', 4);
plot(av, vT(:,1), vT(:,2), 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.35 0.35 0.38], 'MarkerEdgeColor', col.dim);
fr_ = @(u, k) vT(k,:) + u * (vA - vT(k,:));
qL = [fr_(0.17, 1); fr_(0.83, 1)];  qR = [fr_(0.17, 2); fr_(0.83, 2)];
hRvL = plot(av, qL(:,1), qL(:,2), '-', 'Color', col.green, 'LineWidth', 6);
hRvR = plot(av, qR(:,1), qR(:,2), '-', 'Color', col.green, 'LineWidth', 6);
text(av, vT(1,1) - 6, vT(1,2) + 2, 'L', 'Color', col.txt, 'FontSize', fs(9), 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
text(av, vT(2,1) + 6, vT(2,2) + 2, 'R', 'Color', col.txt, 'FontSize', fs(9), 'FontWeight', 'bold');
secX = [80 220];  secY = 74;  secL = 44;          % side sections, flow left to right
hFlap = cell(1, 2);  hRvT = cell(1, 2);  secN = {'LEFT', 'RIGHT'};
for kk = 1:2
    plot(av, secX(kk) + [-62 6], secY * [1 1], '-', 'Color', [0.55 0.60 0.68], 'LineWidth', 5);
    plot(av, secX(kk) + 6 + [0 secL], secY * [1 1], ':', 'Color', col.dim, 'LineWidth', 1);
    for a_ = [-1 1] * rad2deg(rvlim)               % travel stops
        plot(av, secX(kk) + 6 + [0.78 1.05] * secL * cosd(a_), secY - [0.78 1.05] * secL * sind(a_), '-', ...
             'Color', col.red, 'LineWidth', 1.2);
    end
    hFlap{kk} = plot(av, NaN, NaN, '-', 'Color', col.green, 'LineWidth', 4);
    plot(av, secX(kk) + 6, secY, 'o', 'MarkerSize', 4, 'MarkerFaceColor', col.txt, 'MarkerEdgeColor', 'none');
    text(av, secX(kk) - 28, secY + 30, secN{kk}, 'Color', col.dim, 'FontSize', fs(8.5), ...
         'HorizontalAlignment', 'center');
    hRvT{kk} = text(av, secX(kk), 30, '', 'Color', col.txt, 'FontSize', fs(15), 'FontWeight', 'bold', ...
                    'HorizontalAlignment', 'center', 'FontName', mono);
end
hRvM = text(av, 150, 9, '', 'Color', col.dim, 'FontSize', fs(8.5), 'HorizontalAlignment', 'center', 'FontName', mono);

%% ---- left: instruments -----------------------------------------------------------------
ah = axes('Parent', f, 'Units', 'normalized', 'Position', nrm([60 660 300 300]), ...
          'XLim', [-1.05 1.05], 'YLim', [-1.05 1.05], 'Visible', 'off', 'DataAspectRatio', [1 1 1]);
hold(ah, 'on');
hSky = patch(ah, NaN, NaN, [0.25 0.55 0.90], 'EdgeColor', 'none');
hGnd = patch(ah, NaN, NaN, [0.55 0.36 0.20], 'EdgeColor', 'none');
hHor = plot(ah, NaN, NaN, '-', 'Color', 'w', 'LineWidth', 2);
hLad = plot(ah, NaN, NaN, '-', 'Color', 'w', 'LineWidth', 1.1);
pk = 1 / 35;                                       % instrument units per degree of pitch
th_ = linspace(0, 2*pi, 181)';
ring = [cos(th_) sin(th_)];  outer = 1.6 * [cos(th_) sin(th_)];
patch(ah, [ring(:,1); flipud(outer(:,1))], [ring(:,2); flipud(outer(:,2))], col.panel, 'EdgeColor', 'none');
plot(ah, ring(:,1), ring(:,2), '-', 'Color', col.line, 'LineWidth', 2);
for a = [-60 -45 -30 -20 -10 0 10 20 30 45 60]       % bank scale
    r1 = 0.90;  r2 = 0.98 + 0.04 * any(a == [-60 -30 0 30 60]);
    plot(ah, [r1 r2] * sind(a), [r1 r2] * cosd(a), '-', 'Color', 'w', 'LineWidth', 1.2);
end
hBankPtr = patch(ah, NaN, NaN, col.amber, 'EdgeColor', 'none');
plot(ah, [-0.55 -0.18 -0.10], [0 0 -0.08], '-', 'Color', col.amber, 'LineWidth', 3.5);
plot(ah, [0.55 0.18 0.10], [0 0 -0.08], '-', 'Color', col.amber, 'LineWidth', 3.5);
plot(ah, 0, 0, 's', 'MarkerSize', 4, 'MarkerFaceColor', col.amber, 'MarkerEdgeColor', col.amber);
hAtt = text(ov, 210, 643, '', 'Color', col.dim, 'FontSize', fs(10.5), 'HorizontalAlignment', 'center', 'FontName', mono);

% heading tape
ahd = axes('Parent', f, 'Units', 'normalized', 'Position', nrm([40 570 340 46]), 'Color', [0.06 0.09 0.13], ...
           'XLim', [-45 45], 'YLim', [0 1], 'XColor', 'none', 'YColor', 'none', 'Box', 'on');
hold(ahd, 'on');
for d = -360:5:720
    big = mod(d, 10) == 0;
    plot(ahd, [d d], [0 0.22 + 0.13 * big], '-', 'Color', col.dim, 'LineWidth', 1);
    if mod(d, 30) == 0
        lab = sprintf('%d', mod(d, 360));
        if mod(d, 90) == 0, lab = 'NESW';  lab = lab(mod(d, 360) / 90 + 1); end
        text(ahd, d, 0.66, lab, 'Color', col.txt, 'FontSize', fs(10), 'HorizontalAlignment', 'center', 'Clipping', 'on');
    end
end
patch(ahd, [-3 3 0], [1 1 0.72], col.amber, 'EdgeColor', 'none');
hHdg = text(ov, 210, 545, '', 'Color', col.txt, 'FontSize', fs(13), 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'FontName', mono);

% readouts
rd = @(x, y, lab) deal( ...
    text(ov, x, y + 52, lab, 'Color', col.dim, 'FontSize', fs(10)), ...
    text(ov, x, y + 14, '', 'Color', col.txt, 'FontSize', fs(26), 'FontWeight', 'bold', 'FontName', mono));
[~, hVa]  = rd(40, 420, 'AIRSPEED (m/s)');
[~, hAlt] = rd(215, 420, 'ALTITUDE (m)');
[~, hVz]  = rd(40, 310, 'CLIMB RATE (m/s)');
[~, hGs]  = rd(215, 310, 'GROUNDSPEED (m/s)');
hStall = text(ov, 210, 278, '', 'Color', col.red, 'FontSize', fs(11), 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

%% ---- right: aero and power, battery ---------------------------------------------------
Imax = 10;  if hasB, Imax = max(10, ceil(max(bt(:,3)) / 10) * 10); end
G(1) = mkbar(ov, 950, 'Throttle', '%', [0 100], [], fs, col);
G(2) = mkbar(ov, 870, 'Angle of attack', 'deg', [-5 15], rad2deg(P.aero.alpha_stall), fs, col);
G(3) = mkbar(ov, 790, 'Lift-to-drag (L/D)', '', [0 ceil(LDmax * 1.2)], LDmax, fs, col);
G(4) = mkbar(ov, 710, 'Load factor n_z', 'g', [0 max(3, P.struct.n_max)], P.struct.n_max, fs, col);
G(5) = mkbar(ov, 630, 'Pack current', 'A', [0 Imax], [], fs, col);
text(ov, 1536, 607, sprintf('Marks: stall %.0f%s, best L/D %.1f, n_{max} %g', rad2deg(P.aero.alpha_stall), ...
     '^{\circ}', LDmax, P.struct.n_max), 'Color', col.dim, 'FontSize', fs(9));

% battery: tall SOC bar + numbers
socX = 1550;  socY = 290;  socW = 90;  socH = 230;
patch(ov, socX + [0 socW socW 0], socY + [0 0 socH socH], [0.06 0.09 0.13], 'EdgeColor', col.line, 'LineWidth', 1.2);
patch(ov, socX + socW/2 + [-18 18 18 -18], socY + socH + [0 0 10 10], col.line, 'EdgeColor', 'none');
hSoc = patch(ov, NaN, NaN, col.green, 'EdgeColor', 'none');
plot(ov, socX + [-6 socW + 6], socY + 0.2 * socH * [1 1], '--', 'Color', col.amber, 'LineWidth', 1.2);
text(ov, socX - 8, socY + 0.2 * socH, '20%', 'Color', col.amber, 'FontSize', fs(8.5), 'VerticalAlignment', 'middle', ...
     'HorizontalAlignment', 'right');
if hasB
    text(ov, 1680, 520, 'STATE OF CHARGE', 'Color', col.dim, 'FontSize', fs(10));
    hSocT = text(ov, 1680, 482, '', 'Color', col.txt, 'FontSize', fs(26), 'FontWeight', 'bold', 'FontName', mono);
    text(ov, 1680, 440, 'PACK VOLTAGE', 'Color', col.dim, 'FontSize', fs(10));
    hVp   = text(ov, 1680, 404, '', 'Color', col.txt, 'FontSize', fs(20), 'FontWeight', 'bold', 'FontName', mono);
    hVc   = text(ov, 1680, 372, '', 'Color', col.dim, 'FontSize', fs(11), 'FontName', mono);
    text(ov, 1680, 336, 'POWER', 'Color', col.dim, 'FontSize', fs(10));
    hPw   = text(ov, 1680, 302, '', 'Color', col.txt, 'FontSize', fs(20), 'FontWeight', 'bold', 'FontName', mono);
else
    hSocT = [];  hVp = [];  hVc = [];  hPw = [];
    text(ov, 1770, 420, {'no battery log', 'in this run'}, 'Color', col.dim, 'FontSize', fs(12), 'HorizontalAlignment', 'center');
end

%% ---- bottom: timeline --------------------------------------------------------------------
tm = t / 60;
at = axes('Parent', f, 'Units', 'normalized', 'Position', nrm([90 58 1780 120]), 'Color', [0.06 0.09 0.13], ...
          'XColor', col.dim, 'YColor', col.dim, 'FontSize', fs(9.5), 'TickDir', 'out', 'Box', 'off');
hold(at, 'on');
hmax = max([ha; 10]);
fill(at, [tm; flipud(tm)], [max(ha, 0); zeros(size(ha))], [0.30 0.70 1.00], 'FaceAlpha', 0.16, 'EdgeColor', 'none');
plot(at, tm, max(ha, 0), '-', 'Color', [0.45 0.75 1.0], 'LineWidth', 1.3);
xl = [tm(1) tm(end)];
set(at, 'XLim', xl, 'YLim', [0 1.25 * hmax], 'GridColor', col.line, 'GridAlpha', 0.6);
grid(at, 'on');
xlabel(at, 'Time from start of log (min)', 'Color', col.dim, 'FontSize', fs(9.5));
ylabel(at, 'Alt (m)', 'Color', col.dim, 'FontSize', fs(9.5));
hPlayed = patch(at, NaN, NaN, [1 1 1], 'FaceAlpha', 0.06, 'EdgeColor', 'none');
if hasE                                           % same badges as every figure, packed in rows
    pxw = 1780 * s;  dB = 15 * s;  rowsL = [];
    for k = 1:numel(E.t)
        x = E.t(k) / 60;  r = 0;
        while true
            if r + 1 > numel(rowsL), rowsL(r + 1) = -inf; end %#ok<AGROW>
            if (x - rowsL(r + 1)) / diff(xl) * pxw >= dB, rowsL(r + 1) = x;  break, end
            r = r + 1;
        end
        y = max(E.alt(k), 0);  yb_ = 1.25 * hmax * (0.86 - 0.17 * min(r, 3));
        c = kcol(E.kind{k});
        plot(at, [x x], [y yb_], '-', 'Color', 0.5 * c + 0.5 * [0.06 0.09 0.13], 'LineWidth', 0.6);
        plot(at, x, yb_, 'o', 'MarkerSize', 11 * s, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'LineWidth', 0.5);
        text(at, x, yb_, sprintf('%d', E.num(k)), 'Color', 'w', 'FontSize', fs(6.5), 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
end
hCur  = plot(at, [NaN NaN], [0 1.25 * hmax], '-', 'Color', col.amber, 'LineWidth', 1.6);
hCurD = plot(at, NaN, NaN, 'o', 'MarkerSize', 7, 'MarkerFaceColor', col.amber, 'MarkerEdgeColor', 'w');
hEvB  = plot(ov, 46, 212, 'o', 'MarkerSize', 19 * s, 'MarkerFaceColor', col.dim, 'MarkerEdgeColor', 'w');
hEvN  = text(ov, 46, 212, '', 'Color', 'w', 'FontSize', fs(8.5), 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
hEvT  = text(ov, 68, 212, '', 'Color', col.txt, 'FontSize', fs(12), 'Interpreter', 'none');
set([hEvB hEvN], 'Visible', 'off');
if ~hasE, set(hEvT, 'String', 'No numbered events in this run', 'Color', col.dim); end

%% ---- writer ----------------------------------------------------------------------------------
d = fullfile(fileparts(fileparts(here)), 'data', 'runs', o.tag);   % sitl/analysis -> ArduPilot_SITL/data/runs
file = o.file;  if isempty(file), file = fullfile(d, 'swift_flight.mp4'); end
usePNG = ~isempty(o.pngdir) || ~isempty(o.at) || exist('VideoWriter', 'class') ~= 8 && exist('VideoWriter', 'file') ~= 2;
if usePNG
    if isempty(o.pngdir), o.pngdir = fullfile(d, 'video_frames'); end
    if ~isfolder(o.pngdir), mkdir(o.pngdir); end
else
    [fd, ~, ~] = fileparts(file);  if ~isempty(fd) && ~isfolder(fd), mkdir(fd); end
    try
        vw = VideoWriter(file, 'MPEG-4');  vw.Quality = 95;
    catch
        file = regexprep(file, '\.mp4$', '.avi');
        vw = VideoWriter(file, 'Motion JPEG AVI');  vw.Quality = 95;
    end
    vw.FrameRate = o.fps;
    open(vw);
    cleanV = onCleanup(@() close(vw));
end
if isempty(o.at)
    fprintf('\n  SWIFT flight video: %d frames (%.1f min of flight at %gx, %g fps -> %.0f s of video)\n', ...
            nF, (tf(end) - tf(1)) / 60, o.speed, o.fps, nF / o.fps);
else
    fprintf('\n  SWIFT flight video: %d stills\n', nF);
end

%% ---- frames -------------------------------------------------------------------------------
lastE = 0;  sz0 = [];  tic;
for i = 1:nF
    ti = tf(i);
    % 3D: aircraft, trail, shadow, camera
    R = rot_bn(F.phi(i), F.th(i), F.psi(i));
    pN = (R * set_rv(Vb, MT, F.rvL(i), F.rvR(i))')';  % body -> NED, ruddervators as flown
    P3 = [F.e(i) + pN(:,2), F.n(i) + pN(:,1), F.u(i) - pN(:,3)];
    set(hAC, 'Vertices', P3);
    j1 = find(t <= ti, 1, 'last');  if isempty(j1), j1 = 1; end
    j0 = max(1, j1 - nTr);  jj = j0:max(1, round((j1 - j0) / 300)):j1;
    set(hTrail, 'XData', [pos(jj,2); F.e(i)], 'YData', [pos(jj,1); F.n(i)], 'ZData', [ha(jj); F.u(i)]);
    set(hShadow, 'XData', [pos(jj,2); F.e(i)], 'YData', [pos(jj,1); F.n(i)], 'ZData', 0.02 + 0 * [ha(jj); 0]);
    set(hDrop, 'XData', F.e(i) * [1 1], 'YData', F.n(i) * [1 1], 'ZData', [0 F.u(i)]);
    cd_ = [F.cs(i) F.cc(i)];  cd_ = cd_ / max(norm(cd_), 1e-6);         % east, north of travel
    tgt = [F.e(i), F.n(i), F.u(i)];
    cam = tgt - o.chase(1) * [cd_ 0] + [0 0 o.chase(2)];
    cam(3) = max(cam(3), 1.5);
    aim = tgt + 30 * [cd_ 0] + [0 0 1];             % look a little ahead so the horizon stays in view
    set(a3, 'CameraTarget', aim, 'CameraPosition', cam, 'CameraUpVector', [0 0 1]);

    % mini-map
    set(hFlown, 'XData', pos(1:j1, 2), 'YData', pos(1:j1, 1));
    hp = F.psi(i);  tri = mapLen * [1.0 0; -0.6 0.55; -0.3 0; -0.6 -0.55];   % nose, right, notch, left (n, e)
    tn = tri(:,1) * cos(hp) - tri(:,2) * sin(hp);  te = tri(:,1) * sin(hp) + tri(:,2) * cos(hp);
    set(hMapAC, 'XData', F.e(i) + te, 'YData', F.n(i) + tn);

    % horizon
    ph = F.phi(i);  thd = rad2deg(F.th(i));
    Rr = [cos(ph) -sin(ph); sin(ph) cos(ph)];
    off = -thd * pk;
    sky = ([-3 off; 3 off; 3 3; -3 3]) * Rr';   gnd = ([-3 off; 3 off; 3 -3; -3 -3]) * Rr';
    set(hSky, 'XData', sky(:,1), 'YData', sky(:,2));  set(hGnd, 'XData', gnd(:,1), 'YData', gnd(:,2));
    hz = ([-1.2 off; 1.2 off]) * Rr';  set(hHor, 'XData', hz(:,1), 'YData', hz(:,2));
    L = [];
    for pp = [-20 -10 10 20]
        w_ = 0.18 + 0.12 * (abs(pp) == 20);
        seg = ([-w_ off + pp * pk; w_ off + pp * pk]) * Rr';
        L = [L; seg; NaN NaN]; %#ok<AGROW>
    end
    set(hLad, 'XData', L(:,1), 'YData', L(:,2));
    bp = [0 0.88; -0.05 0.78; 0.05 0.78] * Rr';     % bank pointer turns with the horizon
    set(hBankPtr, 'XData', bp(:,1), 'YData', bp(:,2));
    set(hAtt, 'String', sprintf('roll %+5.1f%s   pitch %+5.1f%s', rad2deg(ph), '^{\circ}', thd, '^{\circ}'));
    hdg = mod(rad2deg(F.psi(i)), 360);
    set(ahd, 'XLim', hdg + [-45 45]);
    set(hHdg, 'String', sprintf('HDG %03.0f%s', hdg, '^{\circ}'));

    % readouts
    set(hVa, 'String', sprintf('%5.1f', F.V(i)));  set(hAlt, 'String', sprintf('%5.1f', F.u(i)));
    set(hVz, 'String', sprintf('%+5.1f', F.vz(i)));  set(hGs, 'String', sprintf('%5.1f', F.gs(i)));
    if F.V(i) < 1.1 * Vs && F.u(i) > 2
        set(hVa, 'Color', col.red);  set(hStall, 'String', 'LOW AIRSPEED');
    else
        set(hVa, 'Color', col.txt);  set(hStall, 'String', '');
    end

    % bars
    setbar(G(1), 100 * F.thr(i), '%3.0f');
    setbar(G(2), rad2deg(F.al(i)), '%5.1f', rad2deg(F.al(i)) > 0.85 * rad2deg(P.aero.alpha_stall));
    setbar(G(3), F.LD(i), '%5.1f');
    setbar(G(4), F.nz(i), '%4.2f', F.nz(i) > 0.8 * P.struct.n_max);
    setbar(G(5), F.I(i), '%5.1f');

    % V-tail inset
    rvi = [F.rvL(i) F.rvR(i)];
    for kk = 1:2
        a_ = rvi(kk);  cF = col.green;
        if abs(a_) > 0.6 * rvlim, cF = col.amber; end
        if abs(a_) > rvlim - deg2rad(0.3), cF = col.red; end
        set(hFlap{kk}, 'XData', secX(kk) + 6 + [0 secL * cos(a_)], 'YData', secY - [0 secL * sin(a_)], 'Color', cF);
        set(hRvT{kk}, 'String', sprintf('%+5.1f%s', rad2deg(a_), '^{\circ}'), 'Color', cF * 0.35 + col.txt * 0.65);
        if kk == 1, set(hRvL, 'Color', cF); else, set(hRvR, 'Color', cF); end
    end
    set(hRvM, 'String', sprintf('pitch (\\delta_e) %+5.1f%s   yaw (\\delta_r) %+5.1f%s', rad2deg(F.de(i)), '^{\circ}', ...
        rad2deg(F.dr(i)), '^{\circ}'));

    % battery
    if hasB
        sc = max(0, min(100, F.soc(i)));
        cS = col.green;  if sc < 40, cS = col.amber; end;  if sc < 20, cS = col.red; end
        set(hSoc, 'XData', socX + 4 + [0 socW-8 socW-8 0], 'YData', socY + 4 + [0 0 1 1] * (socH - 8) * sc / 100, 'FaceColor', cS);
        set(hSocT, 'String', sprintf('%5.1f %%', sc));
        set(hVp, 'String', sprintf('%5.2f V', F.vp(i)));  set(hVc, 'String', sprintf('%5.3f V/cell', F.vp(i) / cells));
        set(hPw, 'String', sprintf('%5.0f W', F.vp(i) * F.I(i)));
    end

    % timeline + clock + event
    xm = ti / 60;
    set(hCur, 'XData', [xm xm]);  set(hCurD, 'XData', xm, 'YData', max(F.u(i), 0));
    set(hPlayed, 'XData', [xl(1) xm xm xl(1)], 'YData', [0 0 1.25 1.25] * hmax);
    set(hClock, 'String', sprintf('t = %6.2f min', xm));
    if hasE
        ke = find(E.t <= ti, 1, 'last');
        if ~isempty(ke) && ke ~= lastE
            lastE = ke;
            set(hEvB, 'MarkerFaceColor', kcol(E.kind{ke}), 'Visible', 'on');
            set(hEvN, 'String', sprintf('%d', E.num(ke)), 'Visible', 'on');
            set(hEvT, 'String', sprintf('%.2f min   %s', E.t(ke) / 60, shorten(E.what{ke}, 150)));
        elseif isempty(ke)
            set(hEvT, 'String', 'Waiting for the first event');
        end
    end

    % capture
    drawnow;
    if o.show
        fr = getframe(f);
    else
        fr.cdata = print(f, '-RGBImage', '-r0');  fr.colormap = [];   % hidden window: render off screen
    end
    if isempty(sz0), sz0 = size(fr.cdata);  sz0(1:2) = 2 * floor(sz0(1:2) / 2); end   % MP4 needs even sizes
    if ~isequal(size(fr.cdata), sz0), fr.cdata = fitframe(fr.cdata, sz0); end
    if usePNG
        imwrite(fr.cdata, fullfile(o.pngdir, sprintf('f%05d.png', i)));
    else
        writeVideo(vw, fr);
    end
    if mod(i, 150) == 0 || i == nF
        el = toc;  fprintf('    frame %5d / %d   (%3.0f %%, about %.1f min left)\n', i, nF, 100 * i / nF, el / i * (nF - i) / 60);
    end
end
if ~usePNG, clear cleanV; end
if usePNG
    fprintf('  PNG frames in %s\n\n', o.pngdir);
else
    fprintf('  Saved %s\n\n', file);
end
V = struct('file', file, 'frames', nF, 'speed', o.speed, 'fps', o.fps, 'size', [W H]);
if ~o.show, close(f); end
end

% =========================================================================
function G = mkbar(ov, y, lab, unit, lim, mark, fs, col)
%  Horizontal gauge in the right column: label, value, bar, optional mark.
x0 = 1540;  w = 340;  hb = 16;
text(ov, x0, y + 30, lab, 'Color', col.dim, 'FontSize', fs(10.5));
G.val = text(ov, x0 + w, y + 30, '', 'Color', col.txt, 'FontSize', fs(14), 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'right', 'FontName', mono_font());
G.unit = unit;
patch(ov, x0 + [0 w w 0], y + [0 0 hb hb], [0.06 0.09 0.13], 'EdgeColor', col.line);
G.bar = patch(ov, NaN, NaN, col.acc, 'EdgeColor', 'none');
if lim(1) < 0
    xz = x0 + (0 - lim(1)) / diff(lim) * w;
    plot(ov, [xz xz], y + [-3 hb + 3], '-', 'Color', col.dim, 'LineWidth', 1);
end
if ~isempty(mark)
    xm = x0 + (mark - lim(1)) / diff(lim) * w;
    plot(ov, [xm xm], y + [-5 hb + 5], '-', 'Color', col.amber, 'LineWidth', 2);
end
G.x0 = x0;  G.w = w;  G.y = y;  G.hb = hb;  G.lim = lim;  G.acc = col.acc;  G.red = col.red;
end

function setbar(G, v, fmt, warn)
if nargin < 4, warn = false; end
if isnan(v)
    set(G.bar, 'XData', NaN, 'YData', NaN);  set(G.val, 'String', '  --');  return
end
z = max(G.lim(1), 0);
a = (min(max(v, G.lim(1)), G.lim(2)) - G.lim(1)) / diff(G.lim);
b = (z - G.lim(1)) / diff(G.lim);
xa = G.x0 + G.w * min(a, b);  xb = G.x0 + G.w * max(a, b);
c = G.acc;  if warn, c = G.red; end
set(G.bar, 'XData', [xa xb xb xa], 'YData', G.y + 2 + [0 0 G.hb - 4 G.hb - 4], 'FaceColor', c);
str = sprintf(fmt, v);  if ~isempty(G.unit), str = [str ' ' G.unit]; end
set(G.val, 'String', str);
end

function [Vb, Fc, Cc, MT] = swift_model(sc)
%  SWIFT shape in body axes (x forward, y right, z down), metres x sc:
%  red pod and wing, grey twin booms, inverted-V tail (apex up) from the CAD
%  numbers in swift_vtail.m, ruddervators as their own faces (set_rv moves
%  them), pusher propeller behind the pod.
fus = [ 0.55  0.08  0.08;  0.55 -0.08  0.08;  0.55 -0.08 -0.09;  0.55  0.08 -0.09;
       -0.55  0.07  0.07; -0.55 -0.07  0.07; -0.55 -0.07 -0.08; -0.55  0.07 -0.08];
yb = 0.3825;                                       % boom centres, P.tail.boom_sep / 2
bm = @(y0) [ 0.10 y0+0.02 -0.05;  0.10 y0-0.02 -0.05;  0.10 y0-0.02 -0.09;  0.10 y0+0.02 -0.09;
            -1.10 y0+0.02 -0.05; -1.10 y0-0.02 -0.05; -1.10 y0-0.02 -0.09; -1.10 y0+0.02 -0.09];
wing = [0.12 -1.30 -0.09; 0.12 1.30 -0.09; -0.21 1.30 -0.09; -0.21 -1.30 -0.09];
xLE = -0.895;  xH = -1.016;  xTE = -1.10;  c_rv = xH - xTE;   % chord 0.2045, ruddervator 0.0843
zr = -0.07;  za = zr - 0.19;                       % root on the boom, apex 0.19 m above it
pL = @(u) [0, -yb*(1-u), zr + u*(za - zr)];        % point along the left panel, u = 0 root .. 1 apex
pR = @(u) [0,  yb*(1-u), zr + u*(za - zr)];
at = @(p, x) [x p(2) p(3)];
tl = [at(pL(0), xLE); at(pL(1), xLE); at(pL(1), xH); at(pL(0), xH)];
tr = [at(pR(0), xLE); at(pR(1), xLE); at(pR(1), xH); at(pR(0), xH)];
u1 = 0.17;  u2 = 0.83;                             % ruddervator span 0.296 of the 0.4535 panel
rl = [at(pL(u1), xH); at(pL(u2), xH); at(pL(u2), xTE); at(pL(u1), xTE)];
rr = [at(pR(u1), xH); at(pR(u2), xH); at(pR(u2), xTE); at(pR(u1), xTE)];
prop = [-0.58 -0.267 -0.01; -0.58 0.267 -0.01; -0.58 0.267 0.01; -0.58 -0.267 0.01;
        -0.58 -0.01 -0.267; -0.58 0.01 -0.267; -0.58 0.01 0.267; -0.58 -0.01 0.267];
nose = [0.62 0.04 0.05; 0.62 -0.04 0.05; 0.62 -0.04 -0.04; 0.62 0.04 -0.04];
Vb = [fus; bm(-yb); bm(yb); wing; tl; tr; rl; rr; prop; nose];
bx = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
Fc = [bx; bx + 8; bx + 16; 25:28; 29:32; 33:36; 37:40; 41:44; 45:48; 49:52; 53:56];
red = [0.84 0.20 0.16];  dred = [0.67 0.15 0.12];  grey = [0.78 0.78 0.80];  dark = [0.14 0.14 0.15];
cyan = [0.15 0.75 0.95];                           % ruddervators stand out against the red tail
Cc = [repmat(red, 6, 1); repmat(grey, 12, 1); red; dred; dred; cyan; cyan; dark; dark; dred];
% ruddervator geometry for set_rv: hinge points, chord, trailing-edge-down normal of each panel
aL = pL(1) - pL(0);  aL = aL / norm(aL);  nL = [0, -aL(3), aL(2)];  if nL(3) < 0, nL = -nL; end
aR = pR(1) - pR(0);  aR = aR / norm(aR);  nR = [0, aR(3), -aR(2)];  if nR(3) < 0, nR = -nR; end
MT = struct('iL', 37:40, 'iR', 41:44, 'hL', rl(1:2, :) * sc, 'hR', rr(1:2, :) * sc, ...
            'nL', nL, 'nR', nR, 'c', c_rv * sc);
Vb = Vb * sc;
end

function Vb = set_rv(Vb, MT, rvL, rvR)
%  Move the two ruddervators: positive = trailing edge DOWN (the model's sign).
%  Each one turns about its hinge line; its trailing edge goes back and along
%  the panel's downward normal.
x = [-1 0 0];
te = @(h, n, a) h + MT.c * (cos(a) * x + sin(a) * n);
Vb(MT.iL, :) = [MT.hL; te(MT.hL(2,:), MT.nL, rvL); te(MT.hL(1,:), MT.nL, rvL)];
Vb(MT.iR, :) = [MT.hR; te(MT.hR(2,:), MT.nR, rvR); te(MT.hR(1,:), MT.nR, rvR)];
end

function R = rot_bn(phi, th, psi)
%  Body to NED rotation (3-2-1 Euler angles).
cf = cos(phi);  sf = sin(phi);  ct = cos(th);  st = sin(th);  cp = cos(psi);  sp = sin(psi);
R = [ct*cp, sf*st*cp - cf*sp, cf*st*cp + sf*sp;
     ct*sp, sf*st*sp + cf*cp, cf*st*sp - sf*cp;
     -st,   sf*ct,            cf*ct];
end

function draw_runway(ax, p0, RW)
%  Runway rectangle from the start point along the runway heading, with
%  centreline dashes and threshold bars (east = x, north = y, up = z).
L = RW.len_draw;  w = 2 * RW.halfw;
u = [sind(RW.hdg) cosd(RW.hdg)];  v = [cosd(RW.hdg) -sind(RW.hdg)];   % along, right (east, north)
c0 = [p0(2) p0(1)] - u * RW.dthr_back;
corner = @(a, b) c0 + a * u + b * v;
q = [corner(0, -w/2); corner(L, -w/2); corner(L, w/2); corner(0, w/2)];
patch(ax, q(:,1), q(:,2), 0.01 * [1 1 1 1], [0.22 0.22 0.24], 'EdgeColor', [0.85 0.85 0.85]);
for a = 20:60:L - 40
    d = [corner(a, -0.3); corner(a + 30, -0.3); corner(a + 30, 0.3); corner(a, 0.3)];
    patch(ax, d(:,1), d(:,2), 0.02 * [1 1 1 1], [0.95 0.95 0.95], 'EdgeColor', 'none');
end
for e = [0 L]
    for b = linspace(-w/2 + 2, w/2 - 2, 6)
        s = sign(L/2 - e);
        d = [corner(e + s * 3, b - 0.7); corner(e + s * 3 + s * 25, b - 0.7); corner(e + s * 3 + s * 25, b + 0.7); corner(e + s * 3, b + 0.7)];
        patch(ax, d(:,1), d(:,2), 0.02 * [1 1 1 1], [0.95 0.95 0.95], 'EdgeColor', 'none');
    end
end
end

function RW = runway_info(A, P)
RW = struct('hdg', 0, 'len', NaN, 'dthr', 0, 'halfw', 0.5 * 75 * 0.3048);
if isfield(P, 'sitl') && isfield(P.sitl, 'hdg0'), RW.hdg = P.sitl.hdg0; end
if isstruct(A)
    if isfield(A, 'hdg'),   RW.hdg   = A.hdg;   end
    if isfield(A, 'len'),   RW.len   = A.len;   end
    if isfield(A, 'dthr'),  RW.dthr  = A.dthr;  end
    if isfield(A, 'width'), RW.halfw = A.width / 2; end
end
RW.len_draw = RW.len;  if ~isfinite(RW.len_draw), RW.len_draw = 800; end   % unknown length: a short strip
RW.dthr_back = 0;
end

function WP = read_wp3(file)
%  [north east alt] (m, alt above HOME) of every positioned mission point.
WP = [];
if isempty(file) || ~ischar(file) || ~isfile(file), return, end
L = strsplit(fileread(file), {sprintf('\r\n'), sprintf('\n')});
H = [];  A = zeros(0, 3);
for k = 2:numel(L)
    v = sscanf(L{k}, '%f');
    if numel(v) < 11, continue, end
    if v(1) == 0, H = v(9:10)'; elseif any(v(4) == [16 17 18 19 21 31]) && any(v(9:10)), A(end+1, :) = v([9 10 11])'; end %#ok<AGROW>
end
if isempty(H) || isempty(A), return, end
Re = 6378137;
WP = [deg2rad(A(:,1) - H(1)) * Re, deg2rad(A(:,2) - H(2)) * Re * cosd(H(1)), A(:,3)];
end

function c = kcol(kind)
switch kind
    case 'mission', c = [0.33 0.50 0.80];
    case 'battery', c = [0.85 0.20 0.20];
    otherwise,      c = [0.95 0.55 0.15];
end
end

function s = shorten(s, n)
s = regexprep(s, '\s*\(#\d+\)', '');
if numel(s) > n, s = [s(1:n-3) '...']; end
end

function A = fitframe(A, sz)
%  Crop or pad a captured frame to the first frame's size (window resized).
B = zeros(sz, 'like', A);
r = min(size(A, 1), sz(1));  c = min(size(A, 2), sz(2));
B(1:r, 1:c, :) = A(1:r, 1:c, :);
A = B;
end

function y = resample_to(out, name, t, n)
[ts, ys] = sig(out, name);
if isempty(ts), y = nan(numel(t), n);  return, end
if size(ys, 2) < n, ys(:, end+1:n) = NaN; end
y = interp1(ts, ys(:, 1:n), t, 'linear', 'extrap');
end

function [t, y] = sig(o, name)
t = [];  y = [];  s = [];
if isa(o, 'Simulink.SimulationOutput')
    try, s = o.get(name); catch, s = []; end
elseif isstruct(o) && isfield(o, name)
    s = o.(name);
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

function m = mono_font()
%  Monospace font for the readouts. Consolas ships with Windows only; elsewhere
%  MATLAB's 'FixedWidth' alias picks the system monospace font.
persistent mm
if isempty(mm)
    mm = 'Consolas';
    if ~any(strcmpi(listfonts, mm)), mm = 'FixedWidth'; end
end
m = mm;
end
