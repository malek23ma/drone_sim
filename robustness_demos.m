%% robustness_demos.m  --  robustness scenarios (run a CELL at a time)
% Each scenario stresses the closed loop and shows the controller/estimator cope.
% Faults are pure P-flags read live by existing blocks -- NO model rewiring.
% Real-time pacing should be ON (Run v -> Simulation Pacing, rate 1.0).
%
% Tip: clear the stateful viz/sensor blocks before a fresh run:
%   clear viz_islfcn sensors_islfcn

%% ============================================================================
%  SCENARIO 1 -- GPS DROPOUT  (loses station-keeping in wind, then recovers)
%  Hover while a STEADY wind blows. With GPS the PID rejects the wind and holds.
%  Drop GPS -> the estimator can't see the wind-driven drift -> the drone blows
%  downwind (visible in the 3D window). Restore GPS -> it flies back to station.
%  (A still hover with GPS frozen shows nothing -- the frozen fix is correct.
%   The drone must be DISTURBED for the dropout to matter.)
% ============================================================================
clear viz_islfcn sensors_islfcn
model_params
P.pilot.on  = 0;
P.traj.mode = 'step';   P.traj.set = [0; 0; -3; 0];   % hover at origin, 3 m up
P.viz.on    = 1;
P.wind.mode = 'none';                                  % calm to start
assignin('base','P',P);
x0 = zeros(12,1);  x0(3) = -3;   assignin('base','x0',x0);
set_param('drone_plant','StopTime','inf');
set_param('drone_plant','SimulationCommand','start');
% --- once it hovers steady, DROP GPS + KICK WIND together (paste this line) ---
%   P.fault.gps.on=1; P.wind.mode='const'; P.wind.F=[6;0;0]; P.wind.t_on=0; assignin('base','P',P);
%   -> blind to the new wind, the drone blows North and does NOT recover
% --- a few seconds later, RESTORE GPS (wind still blowing) ---
%   P.fault.gps.on=0; assignin('base','P',P);     % regains truth -> flies back & holds

%% ============================================================================
%  SCENARIO 2 -- MOTOR DEGRADATION  (partial rotor failure, controller trims)
%  M1 drops to 60% thrust. The mixer still commands nominal allocation, so the
%  other rotors must ramp and the attitude loop re-trims to hold position.
% ============================================================================
clear viz_islfcn sensors_islfcn
model_params
P.pilot.on  = 0;
P.traj.mode = 'step';   P.traj.set = [0; 0; -3; 0];   % hover in place
P.viz.on    = 1;
assignin('base','P',P);
x0 = zeros(12,1);  x0(3) = -3;   assignin('base','x0',x0);
set_param('drone_plant','StopTime','inf');
set_param('drone_plant','SimulationCommand','start');
% --- after it is hovering, FAIL motor 1 to 60% (paste this) ---
%   P.fault.motor.on = 1;  P.fault.motor.scale = [0.6;1;1;1];  assignin('base','P',P);
% --- to fail harder / restore ---
%   P.fault.motor.scale = [0.3;1;1;1];  assignin('base','P',P);   % severe
%   P.fault.motor.on = 0;               assignin('base','P',P);   % healthy again

%% ============================================================================
%  SCENARIO 3 -- GUST DURING SPRAY  (disturbance + mission together, automatic)
%  A North gust hits mid-mission while auto-spraying. The coverage map shows the
%  swath pushed off-line; the controller rejects the gust and re-acquires.
% ============================================================================
clear viz_islfcn sensors_islfcn
model_params
P.pilot.on   = 0;
P.traj.mode  = 'field';   P.spray.mode = 'field';   P.viz.on = 1;
P.wind.mode  = 'gust';    P.wind.F = [6; 0; 0];                 % 6 N North gust
P.wind.t_on  = 8;         P.wind.t_off = 11;                    % 3 s burst
assignin('base','P',P);
x0 = zeros(12,1);  x0(3) = -P.field.alt;   assignin('base','x0',x0);
set_param('drone_plant','StopTime','inf');
set_param('drone_plant','SimulationCommand','start');
% Watch t=8..11 s: the green swath bends, then straightens as the loop recovers.
