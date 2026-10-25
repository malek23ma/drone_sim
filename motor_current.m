function I = motor_current(omega, P)
%MOTOR_CURRENT  Per-motor current draw from rotor speed (first-order BLDC model).
%   omega : rotor speed(s) [rad/s]   (scalar or 4x1)
%   P     : parameter struct (needs km, Kt, I0)
%   I     : current per motor [A]
%
%   Physics: the motor must supply the propeller's aerodynamic drag torque.
%     prop drag torque   Q_aero = km * omega^2          [N*m]
%     motor torque       tau    = Kt * (I - I0)         [N*m]   (Kt = torque const)
%   At steady state tau = Q_aero  ->  I = Q_aero/Kt + I0.
%   I0 (no-load current) lumps friction + iron losses as an equivalent current.
%
%   The angular-acceleration term J_rot*omega_dot is neglected: zero at steady
%   state (hover/cruise -> the dominant case for battery draw), small in transients.
    Q_aero = P.km .* omega.^2;        % aerodynamic drag torque [N*m]
    I = Q_aero ./ P.Kt + P.I0;        % current per motor [A]
end
