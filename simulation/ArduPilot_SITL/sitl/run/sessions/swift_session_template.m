function C = swift_session_template
%SWIFT_SESSION_TEMPLATE  Copy this to make a new SITL session.
%
%   1  Copy to  sessions\swift_session_F.m  and rename the function to
%      swift_session_F (it must match the file name).
%   2  Fill in the fields below.
%   3  Fly it:  swift_run F        Check it first:  swift_run('F', 'dry', true)
%
%   A session is only settings. The flying (launch, mission upload, parameter
%   check, arm, AUTO, stop, log copy, analysis) is all done by swift_run, so a
%   new session never needs a new copy of that code.
%
%   THE PLANNER
%   Any function that, called as  S = planner('addr', addr, plan_args{:}),
%     - writes the .waypoints file and returns its path in  S.wp_file
%     - writes any session .param file named in C.params
%     - returns the start point in  S.airport  (lat0, lon0, elev, hdg) -
%       or set C.location instead
%     - accepts 'write', false  (used by  swift_run analyze F)
%   swift_sitl_cub, swift_sitl_cub_d and swift_sitl_cub_e all qualify. To fly
%   a variant of one, reuse it with plan_args. Note that a reused planner
%   writes to its own file names (e.g. swift_sitl_cub_e.waypoints).

C.name      = 'F';                              % the letter you type: swift_run F
C.title     = 'Session E at 120 m with a 4000 mAh landing reserve';
C.tag       = 'sessionF_cub';                   % results go to data\runs\<tag>\
C.planner   = @swift_sitl_cub_e;                % mission + param writer
C.plan_args = {'alt', 120, 'land_mAh', 4000};   % passed to the planner
C.params    = {'swift_sitl.param', 'swift_sitl_cub_e.param', ...   % ArduPlane loads these
               'swift_sitl_batt.param', 'swift_sitl_vtail.param'}; % in this order (later wins)
C.location  = [];                               % [lat lon alt_m hdg_deg]; [] = from S.airport
C.max_min   = 40;                               % autofly warns past this many minutes aloft
C.analysis  = {                                 % run after landing, in order; name, function
    'flight plots', @(out, S, tag) swift_plot_flight(out, 'tag', tag, 'wp', S.wp_file)
    'range check',  @(out, S, tag) swift_range_check(out, 'tag', tag)
    };
end
