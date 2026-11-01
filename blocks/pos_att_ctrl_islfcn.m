function wrench = pos_att_ctrl_islfcn(in, P)
%POS_ATT_CTRL_ISLFCN  Cascaded position(outer)+attitude(inner) controller
%                     with velocity/acceleration FEEDFORWARD + integral action.
%   Input  in = [ state(1:12) ; ref(13:22) ; e_int(23:25) ]
%            ref   = [pos_ref(3); vel_ref(3); acc_ref(3); psi_ref(1)]
%            e_int = integral of position error (optional; 0 if not wired)
%          P  = (optional) parameter struct. Simulink calls with ONE arg -> P is
%               read from base. The PSO optimizer passes P directly (fast, no
%               per-step evalin over thousands of closed-loop runs).
%   Output wrench = [ T ; tau_x ; tau_y ; tau_z ]
%
%   Live controller select: if P.ctrl.mode = 'lqr', hand the same input bus to
%   the full-state LQR (identical I/O) and return. Lets the panel flip PID<->LQR
%   mid-flight with no model rewiring.
    if nargin < 2, P = evalin('base','P'); end
    if isfield(P,'ctrl') && strcmp(P.ctrl.mode,'lqr')
        wrench = lqr_ctrl_islfcn(in);
        return;
    end

    pos   = in(1:3);   vel   = in(4:6);
    eul   = in(7:9);   omega = in(10:12);
    pos_ref = in(13:15);  vel_ref = in(16:18);  acc_ref = in(19:21);  psi_ref = in(22);
    if numel(in) >= 25, e_int = in(23:25); else, e_int = [0;0;0]; end

    % ---------- OUTER LOOP: PID + feedforward -> desired accel (world NED) ----
    acc_des = acc_ref + P.Kp_pos.*(pos_ref - pos) + P.Kd_pos.*(vel_ref - vel) ...
              + P.use_integral * P.Ki_pos .* e_int;

    % ---------- thrust + desired tilt (small-angle inversion of plant R) ------
    cpsi = cos(psi_ref);  spsi = sin(psi_ref);
    T = P.m*(P.g - acc_des(3));
    theta_des = -(1/P.g)*( cpsi*acc_des(1) + spsi*acc_des(2) );
    phi_des   =  (1/P.g)*( cpsi*acc_des(2) - spsi*acc_des(1) );
    phi_des   = max(min(phi_des,   P.tilt_max), -P.tilt_max);
    theta_des = max(min(theta_des, P.tilt_max), -P.tilt_max);

    % ---------- INNER LOOP: attitude PD -> torque ----------
    att_ref = [phi_des; theta_des; psi_ref];
    e   = att_ref - eul;
    Jv  = [P.Jx; P.Jy; P.Jz];
    tau = Jv .* ( P.Kp_att.*e - P.Kd_att.*omega );

    wrench = [T; tau];
end
