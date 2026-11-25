function pwm = ap_json_step(x)
%AP_JSON_STEP  One lockstep frame of the ArduPilot JSON SITL interface.
%
%   pwm = ap_json_step(x)
%
%   Called once per fixed step by the "ArduPilot bridge" block in
%   swift_6dof_sitl. You never call it yourself.
%
%   x    17x1  [t; gyro(3); accel_body(3); pos(3); vel(3); euler(3); airspeed]
%              s, body rad/s, body m/s^2 (specific force), NED m, NED m/s,
%              rad, m/s
%        19x1  the same, plus [battery voltage V; battery current A] from
%              the Battery subsystem (swift_battery_add). Sent as
%              "battery":{"voltage":..,"current":..} unless
%              P.sitl.send_batt = 0. ArduPlane builds that do not know the
%              field ignore it and fall back to SIM_BATT_VOLTAGE.
%   pwm  16x1  servo outputs, microseconds
%
%   EACH CALL
%     1. Sends the state for this instant back to ArduPlane (skipped on the
%        very first call - there is nobody to reply to yet).
%     2. Blocks until the next servo frame arrives and returns its PWM.
%     Replies go to whatever address the last servo frame came from, so no
%     destination is configured anywhere.
%
%   WAITING
%     The first frame may take up to 120 s, so start the model first and
%     SITL second (guide step 15). After that, 10 s with no frame stops
%     the run with an error instead of hanging forever.
%
%   STALE FRAMES
%     Before the bridge answers, SITL re-sends its servo frame every few
%     seconds and they queue up. Each call drains the queue and keeps only
%     the newest, so the model never runs frames behind the autopilot.
%
%   CLEAN-UP
%     The model's StopFcn runs  clear ap_json_step , which releases UDP 9002.
%     If a crash skipped that and the next run says the port is in use, run
%     it yourself.
%
%   VIRTUAL TILT TABLE (swift_sitl_signcheck('jig'))
%     If P.sitl.jig holds a schedule when the run starts, the state SENT to
%     ArduPlane is replaced by a parked aircraft tilted to the scheduled
%     roll/pitch, with matching gyro rates and gravity on the
%     accelerometers. The model itself keeps sitting on the runway. Rows:
%         [t_start  t_end  roll_deg  pitch_deg]   2 s ramps in and out
%     After the last row ends the real state is sent again.
%
%   Ryan Bilock, ARTS Lab | SITL guide steps 10 and 13 | v0.3
%     v0.3  optional battery voltage and current (x elements 18-19)
%     v0.2  virtual tilt table for the surface-direction check

persistent u addr port nrx jig sendb

if isempty(u)
    u    = udpport("datagram", "IPV4", "LocalPort", 9002, "Timeout", 120);
    addr = "";
    port = 0;
    nrx  = 0;
    jig  = [];
    try
        jig = evalin('base', 'P.sitl.jig');
    catch
    end
    sendb = true;
    try
        sendb = evalin('base', 'P.sitl.send_batt') > 0.5;
    catch
    end
    if numel(x) >= 19 && sendb
        fprintf('[ap_json_step] Sending battery voltage and current to ArduPlane.\n');
    end
    if ~isempty(jig)
        fprintf('[ap_json_step] Virtual tilt table ON for the first %g s of sim time.\n', max(jig(:,2)));
    end
    fprintf(['[ap_json_step] Listening on UDP 9002. Start SITL now:\n' ...
             '    sim_vehicle.py --console --map -f JSON:<your step-6 address>\n']);
end

% ---- 1. send this instant's state -----------------------------------------
if ~isempty(jig) && x(1) <= max(jig(:,2))
    x = tilt_state(x, jig);
end
if port > 0
    s = sprintf(['\n{"timestamp":%.6f,' ...
        '"imu":{"gyro":[%.6f,%.6f,%.6f],"accel_body":[%.6f,%.6f,%.6f]},' ...
        '"position":[%.4f,%.4f,%.4f],' ...
        '"velocity":[%.4f,%.4f,%.4f],' ...
        '"attitude":[%.6f,%.6f,%.6f],' ...
        '"airspeed":%.4f'], double(x(1:17)));
    if numel(x) >= 19 && sendb
        s = [s sprintf(',"battery":{"voltage":%.3f,"current":%.3f}', ...
                       double(x(18)), double(x(19)))];
    end
    s = [s sprintf('}\n')];
    write(u, uint8(s), "uint8", addr, port);
end

% ---- 2. take the newest servo frame -----------------------------------------
n  = max(1, u.NumDatagramsAvailable);
dg = read(u, n, "uint8");
if isempty(dg)
    error('ap_json_step:timeout', ...
        ['No servo frame from ArduPlane in %g s. Is SITL running with ' ...
         '-f JSON:<your step-6 address>, and is the UDP 9002 firewall rule in?'], u.Timeout);
end
dg = dg(end);

b = uint8(dg.Data);
if numel(b) < 40
    error('ap_json_step:short', 'Servo frame only %d bytes, expected 40 or more.', numel(b));
end
w = typecast(b(1:2*floor(numel(b)/2)), 'uint16');
switch w(1)
    case {18458, 29569}          % 16 or 32 channels - the first 16 are the same
    otherwise
        error('ap_json_step:magic', ...
            'Expected magic 18458 (or 29569), got %d. That is not an ArduPilot servo frame.', w(1));
end

addr = dg.SenderAddress;
port = dg.SenderPort;
if nrx == 0
    u.Timeout = 10;
    fprintf('[ap_json_step] Connected to ArduPlane at %s:%d (%d Hz).\n', addr, port, w(2));
end
nrx = nrx + 1;

pwm = double(w(5:20));
pwm = pwm(:);
end

% =========================================================================
function x = tilt_state(x, jig)
%TILT_STATE  Parked aircraft on a virtual tilt table (see header).
g  = 9.80665;                     % m/s^2
t  = x(1);
tr = 2;                           % s ramp time
phi = 0;  th = 0;  dphi = 0;  dth = 0;
for k = 1:size(jig, 1)
    ts = jig(k,1);  te = jig(k,2);
    if t >= ts && t <= te
        a = min([1, (t - ts)/tr, (te - t)/tr]);
        s = 0;                                  % ramp slope sign
        if (t - ts) < tr && (t - ts) <= (te - t), s = 1;  end
        if (te - t) < tr && (te - t) <  (t - ts), s = -1; end
        phi  = a * deg2rad(jig(k,3));   dphi = s * deg2rad(jig(k,3)) / tr;
        th   = a * deg2rad(jig(k,4));   dth  = s * deg2rad(jig(k,4)) / tr;
    end
end
x(2:4)   = [dphi; dth; 0];                                   % gyro, rad/s
x(5:7)   = [g*sin(th); -g*sin(phi)*cos(th); -g*cos(phi)*cos(th)];  % specific force
x(11:13) = [0; 0; 0];                                        % NED velocity
x(14:15) = [phi; th];                                        % roll, pitch (yaw kept)
end
