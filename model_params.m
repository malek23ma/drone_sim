%% model_params.m  ---  Quadcopter simulation parameters & CONVENTIONS
% Run this FIRST. It populates base workspace + a struct P used by the model.
% Every value here is defensible: source/derivation noted inline.
%
% ============================ CONVENTIONS (LOCKED) ============================
% World frame  : NED   (North-East-Down).  +Z points DOWN. Gravity = +Z.
% Body frame   : FRD   (Forward-Right-Down). x=nose, y=right wing, z=belly.
% Rotation     : ZYX Euler (yaw psi -> pitch theta -> roll phi).
%                R_body_to_world = Rz(psi)*Ry(theta)*Rx(phi).
% Rotor layout : "X" configuration, viewed from ABOVE (looking in +Z / downward):
%
%                   (front)
%             M1 (CCW)     M2 (CW)
%                  \       /
%                   \     /
%                    \   /
%                     \ /         x = forward
%                     / \         y = right
%                    /   \
%                   /     \
%             M4 (CW)      M3 (CCW)
%                   (rear)
%
%   Motor spin (yaw reaction): M1,M3 = CCW ; M2,M4 = CW.
%   Each rotor thrust acts along -z_body (pushes craft "up" = toward -Z world).
%   CCW rotor produces +z_body reaction torque, CW produces -z_body. This is
%   what lets us yaw. Get these signs wrong -> yaw diverges. We verify later.
%
% State vector (12) the plant integrates:
%   pos   = [x;  y;  z]    world NED   [m]
%   vel   = [u;  v;  w]    world NED   [m/s]   (world-frame velocity)
%   eul   = [phi; theta; psi]          [rad]
%   omega = [p;  q;  r]    body rates  [rad/s]
% =============================================================================

clear P;

%% ---- Force light/white figure theme for ALL plots (override dark theme) ----
set(groot, 'defaultFigureColor','w', ...
           'defaultAxesColor','w', ...
           'defaultAxesXColor','k', 'defaultAxesYColor','k', 'defaultAxesZColor','k', ...
           'defaultAxesGridColor',[0.15 0.15 0.15]);

%% ---- Physical body (REAL drone: agricultural sprayer, quad X) ----
P.g       = 9.81;        % gravity [m/s^2]
P.m_dry   = 1.80;        % dry mass, empty tank [kg]
P.tank_full = 0.38;      % full tank payload, 380 mL water+medicine [kg]
P.m       = P.m_dry + P.tank_full;   % 2.18 kg full (nominal for hover/controller)
P.L       = 0.22;        % center -> motor [m]

% Inertia (point-mass estimate: central mass r_c=0.10 m + 4 motors 118 g at L).
% Jx=Jy = 1/4 Mc rc^2 + 2 m_m L^2 ;  Jz = 1/2 Mc rc^2 + 4 m_m L^2.
P.Jx  = 0.016;
P.Jy  = 0.016;
P.Jz  = 0.032;
P.J   = diag([P.Jx P.Jy P.Jz]);
P.Jinv= inv(P.J);

%% ---- Rotor / aerodynamics ----
% Thrust  : T_i  = kf * omega_i^2     [N]      (omega_i = rotor speed [rad/s])
% Reaction: Q_i  = km * omega_i^2     [N*m]    (about z_body, sign by spin dir)
% SunnySky X2814 900KV + 1045 prop on 4S. kf derived from max-thrust point:
%   T_max~1.9kg=18.6N at omega_max~970 rad/s -> kf = 18.6/970^2 ~ 2.0e-5.
P.kf  = 2.00e-5;        % thrust coeff [N/(rad/s)^2]  (derive/bench-verify!)
P.km  = 3.20e-7;        % torque coeff [N*m/(rad/s)^2] (~0.016*kf, 1045 prop)

% Translational aerodynamic drag: F = -cd*|v|*v  (quadratic, opposes velocity).
% Built from physical quantities so it is defensible and tunable. The lumped
% constant is cd = 0.5*rho*Cd*A  [N/(m/s)^2]  -- this is NOT the dimensionless Cd.
%   rho = air density, Cd = dimensionless drag coeff (multirotor bluff body 0.3-0.5),
%   A   = frontal reference area. Terminal velocity v_term = sqrt(m*g/cd).
% Set P.cd = 0 to disable (the analytic free-fall unit test does this).
P.rho = 1.225;          % air density [kg/m^3] (sea level, 15 C)
P.Cd  = 0.4;            % dimensionless drag coefficient (multirotor)
P.A   = 0.10;           % frontal reference area [m^2] (frame + arms + tank)
P.cd  = 0.5*P.rho*P.Cd*P.A;   % lumped drag constant [N/(m/s)^2]  (-> ~0.0245)

% ---- Hover sanity check (derived, not assumed) ----
% At hover: 4*kf*omega_h^2 = m*g  ->  omega_h = sqrt(m*g/(4*kf))
P.omega_hover = sqrt(P.m*P.g/(4*P.kf));   % ~458 rad/s  (~4370 RPM) -> sane
P.omega_max   = 970;     % rotor saturation [rad/s] (~9300 RPM loaded, 4S/900KV)
P.omega_min   = 0;

%% ---- Mixer / control allocation (X-config, motor order M1..M4) ----
% s = [w1^2; w2^2; w3^2; w4^2]  ->  [T; tau_x; tau_y; tau_z] = Mix * s
% Geometry (see header diagram): M1 front-left CCW, M2 front-right CW,
%                                M3 rear-right CCW, M4 rear-left CW.
% Signs:  roll = left(1,4) - right(2,3) ; pitch = front(1,2) - rear(3,4) ;
%         yaw  = CCW(1,3) - CW(2,4)  (drag-reaction torque).
P.d   = P.L/sqrt(2);             % x/y moment arm of each motor in X layout
P.Mix = [ P.kf,      P.kf,      P.kf,      P.kf;
          P.kf*P.d, -P.kf*P.d, -P.kf*P.d,  P.kf*P.d;
          P.kf*P.d,  P.kf*P.d, -P.kf*P.d, -P.kf*P.d;
          P.km,     -P.km,      P.km,     -P.km   ];
P.Mix_inv = inv(P.Mix);         % allocation: s = Mix_inv * wrench

%% ---- Motor (first-order spool dynamics) ----
% Real ESC+motor: commanded speed not instant. omega_dot = (omega_cmd-omega)/tau
P.tau_motor = 0.02;      % motor time constant [s]  (~20 ms, typical small rotor)

%% ---- Motor electrical model + battery (current draw -> endurance) ----
% First-order DC-equivalent of the BLDC: torque ~ current, back-EMF ~ speed.
% Lets us compute amp draw and flight endurance for the spray mission.
P.Kv   = 900;                 % velocity constant [RPM/V]  (SunnySky X2814 900KV)
P.Ke   = 60/(2*pi*P.Kv);      % back-EMF constant [V*s/rad] = 0.01061 (= 1/Kv in SI)
P.Kt   = P.Ke;                % torque constant  [N*m/A]   (equals Ke in SI units)
P.Rm   = 0.054;               % motor internal resistance [ohm] (datasheet 54 mOhm;
                              %   the 0.8 ohm phase-phase DMM read is lead/contact-dominated)
P.I0   = 1.176;               % no-load current [A] (friction+iron losses as equiv current)
P.batt.V       = 14.8;        % 4S nominal pack voltage [V]
P.batt.cap_Ah  = 5.0;         % pack capacity [Ah]  (5000 mAh)
P.batt.Q_full  = P.batt.cap_Ah*3600;   % usable charge [Coulomb] = 18000
P.batt.reserve = 0.20;        % keep 20% in reserve (LiPo longevity)

%% ---- Simulation ----
P.Ts   = 0.002;          % fixed-step solver step [s] (500 Hz, plenty for this BW)
P.Tend = 20;             % sim duration [s]

%% ---- Attitude controller (inner loop) : pole placement ----
% Each axis ~ double integrator  J*thddot = tau.  PD  tau = J(Kp*e - Kd*rate)
% gives  thddot + Kd*thdot + Kp*th = Kp*th_ref  ->  wn=sqrt(Kp), zeta=Kd/(2 wn).
% Choose [wn; zeta] per axis (roll,pitch,yaw) and DERIVE the gains.
P.wn_att   = [9; 9; 4];               % attitude bandwidth [rad/s] (yaw weaker)
P.zeta_att = [0.8; 0.8; 0.8];         % damping (~1.5% overshoot, no ringing)
P.Kp_att   = P.wn_att.^2;             % proportional gains [1/s^2]
P.Kd_att   = 2*P.zeta_att.*P.wn_att;  % derivative gains   [1/s]

%% ---- Position controller (outer loop) : pole placement ----
% Each axis ~ double integrator  posddot = acc.  PD  acc = Kp*e - Kd*vel.
% MUST be slower than the inner attitude loop (time-scale separation):
%   inner wn=9 ; outer wn_xy=1.5 -> ratio 6 (>3-5 rule). z faster (direct thrust).
P.wn_pos   = [1.5; 1.5; 2.5];         % position bandwidth [rad/s] (x,y,z)
P.zeta_pos = [0.9; 0.9; 0.9];         % damping (near-critical, no overshoot of setpoint)
P.Kp_pos   = P.wn_pos.^2;
P.Kd_pos   = 2*P.zeta_pos.*P.wn_pos;
P.tilt_max = 30*pi/180;               % clamp commanded tilt -> keep small-angle valid

% Integral action (PID) -> rejects CONSTANT disturbances (steady wind droop).
P.Ki_pos      = [1.0; 1.0; 2.0];      % integral gains (x,y,z)
P.use_integral= 1;                    % 1=PID, 0=PD (for before/after demo)
P.int_limit   = 3;                    % anti-windup clamp on integral state [m*s]

%% ---- LQR controller (drop-in alternative to the cascaded PID) -------------
% ONE optimal full-state gain replaces outer-PID + tilt inversion + inner-PD.
% Linearize the SAME plant about hover (psi=0, small angle), NED z-down.
% State x = [pos(3); vel(3); eul(3); omega(3)] ; input u = [dT; tau_x; tau_y; tau_z].
%   pos_dot = vel
%   u_dot(N) = -g*theta   v_dot(E) = g*phi   w_dot(D) = -dT/m
%   eul_dot  = omega      omega_dot = [tau_x/Jx; tau_y/Jy; tau_z/Jz]
% (This A,B is exactly the small-angle model the PID's tilt inversion assumes.)
A = zeros(12);
A(1:3,4:6)   = eye(3);                 % pos_dot = vel
A(4,8)       = -P.g;                   % u_dot(N) = -g*theta
A(5,7)       =  P.g;                   % v_dot(E) =  g*phi
A(7:9,10:12) = eye(3);                 % eul_dot  = omega
B = zeros(12,4);
B(6,1)  = -1/P.m;                      % w_dot  = -dT/m
B(10,2) =  1/P.Jx;                     % p_dot  = tau_x/Jx
B(11,3) =  1/P.Jy;                     % q_dot  = tau_y/Jy
B(12,4) =  1/P.Jz;                     % r_dot  = tau_z/Jz

% Bryson's rule: weight each state/input by 1/(max allowed)^2 -> defensible, unit-aware.
qpos = 1/1.0^2;  qvel = 1/1.0^2;  qatt = 1/(20*pi/180)^2;  qrate = 1/2^2;
rT   = 1/(0.5*P.m*P.g)^2;            rtau = 1/0.5^2;
P.lqr.Q = diag([qpos qpos qpos,  qvel qvel qvel,  qatt qatt qatt,  qrate qrate qrate]);
P.lqr.R = diag([rT rtau rtau rtau]);
P.Klqr   = lqr(A,B,P.lqr.Q,P.lqr.R);   % 4x12 optimal gain
P.u_hover= [P.m*P.g; 0; 0; 0];         % equilibrium wrench (hover thrust, zero torque)
P.lqr.poles = eig(A - B*P.Klqr);       % closed-loop poles (defensibility vs PID BW)
fprintf('LQR closed-loop |pole| range : %.2f .. %.2f rad/s\n', ...
        min(abs(P.lqr.poles)), max(abs(P.lqr.poles)));

%% ---- Active controller select (live-switchable from the panel) ----
P.ctrl.mode = 'pid';   % 'pid' = cascaded PID+FF | 'lqr' = full-state LQR

%% ---- Reference / trajectory generator ----
% Block outputs ref = [pos(3); vel(3); acc(3); psi]  (10x1).
% vel/acc are FEEDFORWARD terms (differential flatness) for tight tracking.
P.traj.mode = 'step';                 % 'step' | 'circle'  (tests override)
P.traj.set  = [1; 1; -2; 0];          % step setpoint [x y z psi]
P.traj.R    = 2;                      % circle radius [m]
P.traj.T    = 10;                     % circle period [s]
P.traj.h    = 2;                      % circle altitude [m]

%% ---- Pilot / interactive joystick (manual velocity command) ----
% Sliders command world-frame velocity; the PilotRef subsystem integrates that
% to a position reference. P.pilot.on selects the model's reference SOURCE:
%   0 = automatic trajectory (traj_islfcn)  -> the whole validation suite uses this
%   1 = manual sliders (PilotRef)           -> interactive flying
% Default 0 so every test compiles unchanged (the Switch can't be left wrong).
P.pilot.on      = 0;          % 0 = auto trajectory (default) | 1 = manual control
P.pilot.src     = 0;          % manual source: 0 = on-screen sliders | 1 = gamepad
P.viz.on        = 0;          % live 3D window: 0 = off (tests) | 1 = on (any flight/mission)
P.pilot.vmax_xy = 3;          % max commanded horizontal speed [m/s] (slider range)
P.pilot.vmax_z  = 2;          % max commanded climb speed [m/s]
P.pilot.rmax    = 1;          % max commanded yaw rate [rad/s]
P.pilot.tgt_lead= 0.6;        % viz only: target star leads by vel_ref*this [s] (chase visible)
P.pilot.spray_btn = 2;        % gamepad button index that TOGGLES spray (calibrated to
                              %   this pad via find(button(vrjoystick(1))); change if pad differs)

%% ---- Field coverage mission (autonomous lawnmower / boustrophedon) ----------
% Flies back-and-forth swaths over a rectangular field, spraying each pass.
% Run it with  P.traj.mode = 'field'  and  P.pilot.on = 0.
P.field.W       = 16;     % field width  (East extent)   [m]
P.field.Lp      = 24;     % field length (North extent)  [m]
P.field.sw      = 4;      % swath spacing between passes [m]
P.field.v       = 3;      % ground speed along a pass    [m/s]
P.field.alt     = 3;      % spray altitude               [m]
P.field.corner  = [0; 0]; % start corner [North; East]   [m]
P.field.spray_w = 4.5;    % spray swath width [m] (>sw -> ~0.5 m deliberate overlap,
                          %   covers turn transients so no gaps; real ag uses ~10%)
% Tank budget for FULL coverage on one 0.38 kg tank (a fine-spray diffuser nozzle):
%   passes = round(W/sw)+1 = 5 ;  spray time = passes*Lp/v = 5*24/3 = 40 s.
%   mass = rate*40 <= 0.38  ->  rate <= 0.0095 kg/s.  Pick 0.008 -> 0.32 kg (16% reserve).
% Separate from the bench P.spray.rate (0.0275, no diffuser) so test_14 is unaffected.
P.field.rate    = 0.008;  % diffuser mission flow [kg/s] (used by 'field' auto-spray)

%% ---- Wind / disturbance (world-frame force) ----
P.wind.mode  = 'none';        % 'none' | 'gust' | 'const'
P.wind.F     = [0; 0; 0];     % force [N], world NED
P.wind.t_on  = 6;             % onset time [s]
P.wind.t_off = 8;             % end time [s] (gust only)

%% ---- Spray system (tank depletion -> time-varying mass) ----
% Pump R385, ~1.65 L/min ~ 0.0275 kg/s. Tank 380 mL drains in ~14 s of spraying.
% m(t) = m_dry + tank_mass(t).  Tank integrator depletes while spraying.
P.spray.mode  = 'none';       % 'none' | 'on' | 'field' (autonomous auto-spray)
P.spray.rate  = 0.0275;       % mass flow [kg/s] (1.65 L/min water, full pump)
P.spray.t_on  = 4;            % start spraying [s]
P.spray.t_off = 20;           % stop spraying [s]  (long enough to empty)
P.spray.joy    = 0;           % live manual spray toggle (set by joystick button)
P.spray.cone_ang = 12;        % viz: spray cone half-angle [deg] (footprint grows with height)

%% ---- Fault injection (robustness scenarios; all default OFF) ------------------
% Live toggles: set the flag then  assignin('base','P',P)  mid-sim to inject.
% Read by sensors_islfcn (GPS) and forces_islfcn (motors); no model rewiring.
P.fault.gps.on      = 0;            % 1 = GPS dropout: estimator gets a FROZEN fix
                                    %     (last good position held) -> dead-reckon drift
P.fault.motor.on    = 0;            % 1 = apply per-rotor thrust scale (partial failure)
P.fault.motor.scale = [1;1;1;1];   % thrust multiplier per rotor M1..M4
                                    %   e.g. [0.6;1;1;1] = M1 at 60% -> controller must trim

%% ---- Sensor models (truth -> noisy measurement) ----
% Measured: y = [ pos(GPS) ; eul(AHRS) ; omega(gyro) ]  (9x1). NO velocity sensor.
P.sen.gps_std   = 0.08;                       % GPS position noise [m]
P.sen.att_std   = 0.5*pi/180;                 % attitude noise [rad] (~0.5 deg)
P.sen.gyro_std  = 0.01;                        % gyro noise [rad/s]
P.sen.gyro_bias = [0.5; -0.3; 0.2]*pi/180;    % constant gyro bias [rad/s] (the drift source)

%% ---- State estimator (continuous model-aided observer) ----
% Position/vel: per-axis [pos;vel] double integrator, GPS measures pos.
%   place observer poles via (wn,zeta): Lp=2 zeta wn (pos corr), Lv=wn^2 (vel corr).
% Faster than outer loop (1.5) so the estimate leads the control.
P.obs_wn   = 4;  P.obs_zeta = 0.7;
P.Lp   = 2*P.obs_zeta*P.obs_wn;       % GPS->pos_hat correction gain
P.Lv   = P.obs_wn^2;                  % GPS->vel_hat correction gain
P.Katt = 3;                           % AHRS->attitude correction (complementary crossover)
P.tau_w= 0.03;                        % gyro low-pass time constant [s]

%% ---- Initial state + default reference (base vars the model's ICs read) ----
% The Plant and Estimator Integrators use x0 as their InitialCondition, and the
% model also reads a base 'ref'. Defining them here means opening or running the
% model never errors with "x0 undefined". Test scripts and the joystick setup
% override these for their own scenarios (e.g. x0(3) = -2 to start hovering).
x0  = zeros(12,1);     % start at origin, level, at rest
ref = P.traj.set;      % default setpoint [x y z psi] = [1 1 -2 0], matches P.traj

%% ---- Echo the sanity numbers so you SEE them every run ----
fprintf('--- model_params loaded ---\n');
fprintf('hover rotor speed : %.1f rad/s  (%.0f RPM)\n', P.omega_hover, P.omega_hover*30/pi);
fprintf('hover thrust/motor: %.2f N   (weight = %.2f N)\n', P.kf*P.omega_hover^2, P.m*P.g);
fprintf('thrust margin     : %.1f%% headroom to omega_max\n', 100*(P.kf*P.omega_max^2*4/(P.m*P.g)-1));
fprintf('drag (Cd=%.2f,A=%.2f): cd=%.4f N/(m/s)^2 -> terminal v = %.1f m/s\n', ...
        P.Cd, P.A, P.cd, sqrt(P.m*P.g/P.cd));
Ihov = P.km*P.omega_hover^2/P.Kt + P.I0;   % per-motor hover current [A]
fprintf('hover current     : %.1f A/motor (%.0f A pack) -> ~%.1f min usable\n', ...
        Ihov, 4*Ihov, (1-P.batt.reserve)*P.batt.Q_full/(4*Ihov)/60);
