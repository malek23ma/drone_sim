function xdot = quad_plant(t, x, u, P, Fwind, m_now)
% QUAD_PLANT  6-DOF rigid-body quadcopter dynamics (state-derivative form).
%   xdot = quad_plant(t, x, u, P [, Fwind, m_now])
%   Fwind = world disturbance force [N];  m_now = current mass [kg] (tank drains)
%
%   Conventions: world = NED, body = FRD, attitude = ZYX Euler. See model_params.m
%
%   x = [ pos(1:3) ; vel(4:6) ; eul(7:9) ; omega(10:12) ]
%         pos   = [x y z]   world NED      [m]
%         vel   = [u v w]   world NED      [m/s]
%         eul   = [phi theta psi]          [rad]   (roll pitch yaw)
%         omega = [p q r]   body rates     [rad/s]
%   u = [ T ; tau_x ; tau_y ; tau_z ]      thrust [N] + body torques [N*m]

    % ---- unpack ----
    pos   = x(1:3);    %#ok<NASGU>  (not needed for derivatives, kept for clarity)
    vel   = x(4:6);
    eul   = x(7:9);
    omega = x(10:12);

    phi = eul(1);  theta = eul(2);  psi = eul(3);
    T   = u(1);
    tau = u(2:4);

    % ---- rotation matrix body->world, R = Rz(psi)*Ry(theta)*Rx(phi) ----
    cphi=cos(phi); sphi=sin(phi);
    cth =cos(theta); sth=sin(theta);
    cpsi=cos(psi);  spsi=sin(psi);
    R = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
          spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
          -sth,      cth*sphi,                   cth*cphi ];

    % =========================== 2a: TRANSLATIONAL ===========================
    if nargin < 5, Fwind = [0;0;0]; end              % optional wind/disturbance
    if nargin < 6 || m_now <= 0, m_now = P.m; end    % current mass (tank depletes)
    pos_dot = vel;                                   % kinematics: x_dot = v
    F_body  = [0; 0; -T];                            % thrust along -z_body
    % aerodynamic drag: quadratic, opposes world velocity (default off if P.cd unset)
    if isfield(P,'cd') && P.cd > 0
        F_drag = -P.cd * norm(vel) * vel;            % F = -cd*|v|*v  -> terminal velocity
    else
        F_drag = [0;0;0];
    end
    vel_dot = [0; 0; P.g] + (1/m_now)*(R*F_body + Fwind + F_drag);   % Newton: grav + thrust + wind + drag
    % NOTE: gravity is an acceleration (mass-independent); thrust/wind/drag scale by 1/m(t).

    % =========================== 2b: ROTATIONAL =============================
    % Euler's equation, body frame:  J*wdot = tau - w x (J w)
    % cross() keeps the gyroscopic coupling term sign-correct by construction.
    omega_dot = P.Jinv * ( tau - cross(omega, P.J*omega) );

    % =========================== 2c: KINEMATICS =============================
    % Euler-rate kinematics:  eul_dot = W(phi,theta) * omega
    % NOTE the 1/cos(theta) terms -> singular (gimbal lock) at theta = +-90 deg.
    tth = tan(theta);
    W = [ 1, sphi*tth,      cphi*tth;
          0, cphi,         -sphi;
          0, sphi/cth,      cphi/cth ];
    eul_dot = W * omega;

    xdot = [pos_dot; vel_dot; eul_dot; omega_dot];
end
