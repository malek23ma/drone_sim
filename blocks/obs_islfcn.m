function xhat_dot = obs_islfcn(in)
%OBS_ISLFCN  Continuous model-aided state observer (Luenberger/Kalman-Bucy form).
%   Input  in = [ xhat(1:12) ; y(13:21) ; wrench(22:25) ; m(26) ]
%            xhat   = current estimate [pos;vel;eul;omega]
%            y      = measurements [pos_gps; eul_ahrs; omega_gyro]
%            wrench = [T; tau]  (known control input, for model prediction)
%            m      = current mass from tank-level sensor (optional)
%   Output xhat_dot = d/dt estimate   (held by an Integrator block)
    P = evalin('base','P');

    pos_h = in(1:3);   vel_h = in(4:6);   eul_h = in(7:9);   om_h = in(10:12);
    ypos  = in(13:15); yeul  = in(16:18); yom   = in(19:21);
    T     = in(22);
    if numel(in) >= 26, m_now = in(26); else, m_now = P.m; end   % tank-level sensor

    phi=eul_h(1); th=eul_h(2); ps=eul_h(3);
    cphi=cos(phi); sphi=sin(phi); cth=cos(th); sth=sin(th); cps=cos(ps); sps=sin(ps);

    % R body->world, W euler-rate map (evaluated at the ESTIMATE)
    R = [ cps*cth, cps*sth*sphi-sps*cphi, cps*sth*cphi+sps*sphi;
          sps*cth, sps*sth*sphi+cps*cphi, sps*sth*cphi-cps*sphi;
          -sth,    cth*sphi,              cth*cphi ];
    tth = tan(th);
    W = [ 1, sphi*tth, cphi*tth;
          0, cphi,    -sphi;
          0, sphi/cth, cphi/cth ];

    % ---- attitude: gyro predicts, AHRS corrects (complementary observer) ----
    eul_dot = W*yom + P.Katt*(yeul - eul_h);

    % ---- body rates: low-pass the gyro ----
    om_dot = (yom - om_h)/P.tau_w;

    % ---- position/velocity: thrust+attitude predict accel, GPS corrects ----
    a_hat   = [0;0;P.g] + (1/m_now)*R*[0;0;-T];    % model-predicted world accel (true mass)
    pos_dot = vel_h + P.Lp*(ypos - pos_h);
    vel_dot = a_hat + P.Lv*(ypos - pos_h);

    xhat_dot = [pos_dot; vel_dot; eul_dot; om_dot];
end
