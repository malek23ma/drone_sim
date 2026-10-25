%% test_16_battery.m  --  Motor electrical model + battery endurance (ORACLE)
% Validates the current / voltage / endurance math against hand calculations,
% BEFORE mirroring in Simulink. Pure .m -- no model dependence (like test_2x).
%
% Chain proven here:  rotor speed omega --(km)--> prop drag torque
%                      --(Kt)--> current --(Rm,Ke)--> terminal voltage
%                      --> pack current --> battery endurance.
clear; clc;
model_params;

wh   = P.omega_hover;                 % hover rotor speed (full tank) [rad/s]
Ih   = motor_current(wh, P);          % per-motor hover current [A]
Itot = 4*Ih;                          % total pack current at hover [A]
Vh   = Ih*P.Rm + P.Ke*wh;             % motor terminal voltage at hover [V]
eta  = (P.Ke*wh)/Vh;                  % electrical efficiency (back-EMF / terminal)

% endurance at hover (constant-current approximation)
t_empty  =                P.batt.Q_full / Itot;   % [s] to 0%
t_usable = (1-P.batt.reserve)*P.batt.Q_full / Itot;   % [s] to reserve

% thrust vs current is LINEAR above I0:  T = (kf*Kt/km) * (I - I0)
slope = P.kf*P.Kt/P.km;               % [N/A]

% max omega the pack can drive (Vbatt = (km w^2/Kt + I0) Rm + Ke w), quadratic in w
a = P.km*P.Rm/P.Kt;  b = P.Ke;  c = P.I0*P.Rm - P.batt.V;
wmax_elec = (-b + sqrt(b^2 - 4*a*c))/(2*a);
wmax_eff  = min(wmax_elec, P.omega_max);          % structural/aero cap also applies
TW        = 4*P.kf*wmax_eff^2 / (P.m*P.g);

fprintf('\n--- motor electrical / battery ---\n');
fprintf('Kt = Ke              : %.5f N*m/A  (from %d KV)\n', P.Kt, P.Kv);
fprintf('hover current/motor  : %.2f A   (pack %.1f A)\n', Ih, Itot);
fprintf('hover terminal V     : %.2f V of %.1f V   (elec eff %.0f%%)\n', Vh, P.batt.V, 100*eta);
fprintf('thrust/current slope : %.3f N/A  (%.0f g/A)\n', slope, slope/P.g*1000);
fprintf('max omega            : %.0f rad/s elec, cap %.0f -> T/W = %.2f\n', wmax_elec, P.omega_max, TW);
fprintf('hover endurance      : %.1f min to empty,  %.1f min to %.0f%% reserve\n', ...
        t_empty/60, t_usable/60, 100*P.batt.reserve);

%% asserts (physics gates)
assert(abs(Ih - 9.24) < 0.15, 'hover current off expected ~9.24 A');
assert(Vh < P.batt.V,         'hover infeasible: needs more than pack voltage');
assert(wmax_elec > wh,        'cannot even reach hover speed on this pack');
assert(TW > 2.0,              'thrust-to-weight too low; check Rm / Kv');
assert(abs(slope*(Ih-P.I0) - P.kf*wh^2) < 1e-9, 'thrust-current linearity broken');
fprintf('\n>>> Step 16 PASSED. Motor draws %.1f A at hover -> ~%.1f min usable flight.\n', ...
        Itot, t_usable/60);
