function P = swift_mission(P)
%SWIFT_MISSION  The flight profile the step-23 Mission Sequencer walks.
%
%   P = swift_mission(P)
%
%   Adds P.mission and nothing else. Gains, battery and every aircraft
%   number stay in swift_placeholder.m, so this file never fights it.
%
%   WHO CALLS IT
%     The model's InitFcn calls it on every run, straight after making sure
%     P exists. It always rebuilds P.mission from this file, so:
%       - edit the mission HERE, not at the command line - a command-line
%         change to P.mission is overwritten on the next run
%       - switching payload (P_with_payload = false; swift_placeholder)
%         re-scales every airspeed below automatically
%
%   AIRSPEEDS ARE MULTIPLES OF STALL, NOT NUMBERS
%     The SWIFT at max payload stalls near 18 m/s and empty near 16 m/s, so
%     a fixed number would be right for one case and below stall for the
%     other. Every speed below is tied to P.perf.V_stall or P.init.V0
%     (1.3 x stall), both computed by swift_placeholder.m.
%
%   ALTITUDES STAY UNDER 400 FT (122 m)
%     The run starts at P.init.h0 = 100 m. The highest command is 115 m.
%
%   TABLE COLUMNS (one row per segment, walked top to bottom)
%     1  V_cmd   m/s      commanded airspeed
%     2  h_cmd   m        commanded altitude above launch
%     3  psi_cmd rad      commanded heading, 0 = North, +CW toward East
%     4  exit    -        1 = altitude reached within tol
%                         2 = time in segment reached t_max
%                         3 = heading reached within tol
%     5  tol     m | rad  tolerance for exit modes 1 and 3
%     6  t_max   s        hard timeout - ALWAYS advances, so a segment the
%                         aircraft cannot fly never stalls the profile
%
%   Tags:  [OPS] mission-planning choice, a test knob - change freely
%
%   Ryan Bilock, ARTS Lab | step 23 | v0.2  (v0.1 was sized for the old
%   4 kg placeholder: its 14-18 m/s speeds are below the SWIFT's stall)

%% Reference speeds - derived, never typed --------------------------------
Vs   = P.perf.V_stall;          % m/s  1-g stall, from swift_aero_matrix
V_cr = P.init.V0;               % m/s  cruise, 1.3 x stall           [OPS]
V_lo = 1.20 * Vs;               % m/s  loiter and approach           [OPS]
V_hi = 1.40 * Vs;               % m/s  fast leg                      [OPS]

d = pi/180;                     % deg -> rad, keeps the table readable

%% The profile -----------------------------------------------------------
%  A box pattern: climb, out north, east leg with a step down, loiter south,
%  home west, then down to a 20 m gate on the final heading.
seg = { ...
% name                        V      h     psi      exit  tol     t_max
  'Climb to pattern',         V_cr,  115,    0*d,   1,    3,       90;
  'Cruise north',             V_cr,  115,    0*d,   2,    0,       60;
  'Turn to east',             V_cr,  115,   90*d,   3,    5*d,     60;
  'Fast leg east',            V_hi,  115,   90*d,   2,    0,       45;
  'Step down to 90 m',        V_cr,   90,   90*d,   1,    3,      120;
  'Turn to south',            V_cr,   90,  180*d,   3,    5*d,     60;
  'Loiter south',             V_lo,   90,  180*d,   2,    0,       60;
  'Turn to west (home leg)',  V_cr,   90,  270*d,   3,    5*d,     60;
  'Descend to 60 m',          V_cr,   60,  270*d,   1,    3,      120;
  'Turn to final heading',    V_lo,   60,    0*d,   3,    5*d,     60;
  'Descend to 20 m gate',     V_lo,   20,    0*d,   1,    3,      150;
  };

P.mission.names = seg(:,1);
P.mission.table = cell2mat(seg(:,2:7));
P.mission.n     = size(P.mission.table, 1);

%  Fixed-size, zero-padded copy for the chart's TAB constant, which must be
%  a constant size at compile time. Raise NMAX here AND the TAB size in the
%  chart together if you ever need more than 32 segments.
NMAX = 32;
assert(P.mission.n <= NMAX, 'swift_mission: more than %d segments.', NMAX);
P.mission.table32 = zeros(NMAX, 6);
P.mission.table32(1:P.mission.n, :) = P.mission.table;

%  Recorded so you can see which aircraft state the speeds were sized for.
P.mission.V_stall_basis = Vs;

%% Guard rails -------------------------------------------------------------
if any(P.mission.table(:,1) < 1.15*Vs)
    warning('swift_mission:slow', ...
        'A segment commands under 1.15 x stall (%.1f m/s). Check the table.', 1.15*Vs);
end
if any(P.mission.table(:,2) > 121.9)
    warning('swift_mission:high', ...
        'A segment commands above 400 ft (121.9 m). Check the table.');
end
end
