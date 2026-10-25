function u = pwm_to_forces(pwm, P)
%PWM_TO_FORCES  Firmware motor PWMs -> plant input u = [T; tau_x; tau_y; tau_z].
%
%   pwm = [M1 M2 M3 M4] FIRMWARE order = [FR FL RR RL]
%         FR CW, FL CCW, RR CCW, RL CW   (matches mixer.c + real frame)
%   NOTE: sim's model_params motor numbering (M1=front-LEFT...) is DIFFERENT;
%   everything here uses positions/spins explicitly, so no confusion.
%
%   Simplification: omega = omega_max*(pwm-1000)/1000 (linear ESC).
%   Thrust T_i = kf*w^2, drag torque Q_i = km*w^2 (sign by spin).
    w_max = 970;                              % rad/s (SunnySky X2814, model_params)
    w = w_max * (max(min(pwm,2000),1000) - 1000) / 1000;
    T = P.kf * w.^2;                          % per-motor thrust [N]
    Q = P.km * w.^2;                          % per-motor drag torque [N*m]

    d = P.L / sqrt(2);
    %        FR      FL      RR      RL      (body FRD: x fwd, y right)
    xs = [   d,      d,     -d,     -d ];
    ys = [   d,     -d,      d,     -d ];
    % CCW prop (viewed from above) -> +tau_z in NED(z down); CW -> -tau_z
    ss = [  -1,     +1,     +1,     -1 ];     % FR CW, FL CCW, RR CCW, RL CW

    u = [ sum(T);
          -sum(ys .* T);      % tau_x  (r x F with F = -T z_b)
           sum(xs .* T);      % tau_y
           sum(ss .* Q) ];    % tau_z
end
