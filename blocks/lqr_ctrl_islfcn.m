function wrench = lqr_ctrl_islfcn(in)
%LQR_CTRL_ISLFCN  Full-state LQR controller — DROP-IN for pos_att_ctrl_islfcn.
%   Identical interface, so it substitutes the cascaded PID 1-for-1 at the
%   wrench boundary (feed it the same input bus; route output to the Mixer).
%
%   Input  in = [ state(1:12) ; ref(13:22) ; e_int(23:25) ]
%            ref   = [pos_ref(3); vel_ref(3); acc_ref(3); psi_ref(1)]
%            (acc_ref and e_int are accepted but UNUSED by the regulator)
%   Output wrench = [ T ; tau_x ; tau_y ; tau_z ]
%
%   Law:  wrench = u_hover - Klqr*(x - xref)
%         u_hover = [m*g;0;0;0] is the hover equilibrium; -Klqr*e adds the
%         optimal deviation control [dT; tau]. Gain built in model_params.m.
    P = evalin('base','P');

    x       = in(1:12);
    pos_ref = in(13:15);  vel_ref = in(16:18);  psi_ref = in(22);

    % Reference state: track position + velocity, level attitude (phi=theta=0),
    % commanded yaw, zero body rates.
    xref = [pos_ref; vel_ref; 0; 0; psi_ref; 0; 0; 0];

    e    = x - xref;
    e(9) = atan2(sin(e(9)), cos(e(9)));    % wrap yaw error to [-pi,pi] (short way)

    % --- heading rotation: the gain is linearized about yaw=0, so it assumes
    % roll->East, pitch->North in WORLD axes. At nonzero heading that mapping is
    % wrong and the horizontal feedback points the wrong way (drone circles, then
    % tumbles). Rotate the horizontal position+velocity error into the body-heading
    % frame (Rz(-psi)) so the small-angle map holds for ANY yaw. tau stays body-frame.
    psi = x(9);  c = cos(psi);  s = sin(psi);
    Rz  = [c s; -s c];                     % world -> heading frame
    e(1:2) = Rz * e(1:2);                  % horizontal position error
    e(4:5) = Rz * e(4:5);                  % horizontal velocity error

    wrench = P.u_hover - P.Klqr * e;       % [T; tau_x; tau_y; tau_z]
end
