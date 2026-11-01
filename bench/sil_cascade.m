function sil_cascade()
%SIL_CASCADE  Software-in-the-loop: EXACT firmware controller (PID cascade
%   from fc_tasks.c + mixer.c ports) closed against the drone_sim 6-DOF
%   plant (quad_plant.m). No hardware needed.
%
%   What it proves:
%     - mixer signs are correct for the real frame layout (or not!)
%     - default firmware gains stabilize the modeled airframe (or not!)
%   Also prints a static mixer/torque sign-check table first.
%
%   Controller measurement path mirrors the firmware convention:
%   IMU chip frame is z-up => fw_roll = phi, fw_pitch = -theta,
%   fw gyro = [p, -q, -r] in dps  (see sensor_synth.m).
    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    addpath(root);                           % model_params adds core/ (quad_plant.m)
    run(fullfile(root, 'model_params.m'));   % populates P

    %% ---------- static sign check ----------
    fprintf('=== MIXER / TORQUE SIGN CHECK (hover PWM, single-axis commands) ===\n');
    w_hover_pwm = 1000 + round(P.omega_hover/970*1000);
    cmds = { '+roll',  [200 0 0];
             '+pitch', [0 200 0];
             '+yaw',   [0 0 200] };
    expect = { 'tau_x > 0 (roll right)', 'tau_y > 0 (see note)', 'tau_z > 0 (nose right)' };
    for i = 1:3
        c = cmds{i,2};
        m = fw_mixer(w_hover_pwm, c(1), c(2), c(3));
        u = pwm_to_forces(m, P);
        fprintf('%-7s cmd -> PWM [%4d %4d %4d %4d] -> tau=[%+.3f %+.3f %+.3f] N*m  (%s)\n', ...
                cmds{i,1}, m, u(2), u(3), u(4), expect{i});
    end
    fprintf(['NOTE +pitch: firmware "positive pitch" = chip frame = -theta(NED).\n' ...
             'So a +pitch command must give tau_y < 0 (nose DOWN in NED = pitch\n' ...
             'up in chip frame). Judge the closed loop below, not this raw sign.\n\n']);

    %% ---------- closed-loop sim ----------
    dt   = 1/250;                 % firmware loop rate
    Tend = 8;
    N    = round(Tend/dt);

    % firmware gains (fc_tasks.c defaults)
    ang_cfg  = struct('kp',4.0,'ki',0,'kd',0,'i_max',0,'out_max',200,'tau',0);
    rate_cfg = struct('kp',1.2,'ki',0.05,'kd',0.03,'i_max',100,'out_max',400,'tau',0.015);
    yaw_cfg  = struct('kp',3.0,'ki',0.10,'kd',0.00,'i_max',100,'out_max',400,'tau',0.015);

    s_ang_r = fw_pid_init(); s_ang_p = fw_pid_init();
    s_rat_r = fw_pid_init(); s_rat_p = fw_pid_init(); s_rat_y = fw_pid_init();

    % initial disturbance: 15 deg roll, -10 deg pitch, level command
    x = zeros(12,1);
    x(7) = deg2rad(15); x(8) = deg2rad(-10);

    thr_pwm = w_hover_pwm;        % manual hover throttle (no alt controller)
    log = zeros(N, 12);

    for k = 1:N
        % ---- "sensor" values exactly as the firmware would see them ----
        vd  = zeros(3,1);          % accel feedback unused by controller
        [~, gyr_dps, ~] = sensor_synth(x, vd, P);
        c_roll  =  rad2deg(x(7));            % fw roll  = phi
        c_pitch = -rad2deg(x(8));            % fw pitch = -theta

        % ---- firmware flight_task control law, verbatim ----
        des_roll = 0; des_pitch = 0; des_yaw_rate = 0;
        if abs(des_roll  - c_roll)  < 2, des_roll  = c_roll;  end   % deadband
        if abs(des_pitch - c_pitch) < 2, des_pitch = c_pitch; end

        [tr_r, s_ang_r] = fw_pid(ang_cfg, s_ang_r, des_roll,  c_roll,  dt);
        [tr_p, s_ang_p] = fw_pid(ang_cfg, s_ang_p, des_pitch, c_pitch, dt);

        [mx_r, s_rat_r] = fw_pid(rate_cfg, s_rat_r, tr_r,        gyr_dps(1), dt);
        [mx_p, s_rat_p] = fw_pid(rate_cfg, s_rat_p, tr_p,        gyr_dps(2), dt);
        [mx_y, s_rat_y] = fw_pid(yaw_cfg,  s_rat_y, des_yaw_rate,gyr_dps(3), dt);

        pwm = fw_mixer(thr_pwm, mx_r, mx_p, mx_y);
        u   = pwm_to_forces(pwm, P);

        % ---- plant, RK4 ----
        x = rk4(@(t,xx) quad_plant(t, xx, u, P), x, dt);
        log(k,:) = [k*dt, rad2deg(x(7:9))', rad2deg(x(10:12))', pwm, -x(3)];
    end

    %% ---------- plots + verdict ----------
    t = log(:,1);
    figure('Name','SIL: firmware controller vs sim plant','Color','w');
    subplot(3,1,1); plot(t, log(:,2:4)); grid on;
        legend('\phi roll','\theta pitch','\psi yaw'); ylabel('euler [deg]');
        title('SIL closed loop - firmware PID + mixer vs 6-DOF plant');
    subplot(3,1,2); plot(t, log(:,5:7)); grid on;
        legend('p','q','r'); ylabel('body rates [deg/s]');
    subplot(3,1,3); plot(t, log(:,8:11)); grid on;
        legend('M1 FR','M2 FL','M3 RR','M4 RL'); ylabel('PWM'); xlabel('t [s]');

    tail_idx = round(0.9*N):N;
    fin_ang  = mean(abs(log(tail_idx, 2:3)), 'all');
    fin_rate = mean(abs(log(tail_idx, 5:6)), 'all');
    % Firmware has a +/-2 deg angle deadband -> craft may legitimately park
    % up to ~2 deg off level. Judge stability by angle bound + rates dead.
    if fin_ang < 2.5 && fin_rate < 5.0
        fprintf(['VERDICT: converged. |roll,pitch| last 10%% = %.2f deg (inside the 2 deg\n' ...
                 'firmware deadband), rates %.2f deg/s -> mixer signs + gains OK on model.\n'], ...
                fin_ang, fin_rate);
    else
        fprintf('VERDICT: NOT converged (%.2f deg, %.2f deg/s). Wrong mixer sign or unstable gains -> investigate before flight!\n', ...
                fin_ang, fin_rate);
    end
end

function x = rk4(f, x, h)
    k1 = f(0, x);
    k2 = f(0, x + h/2*k1);
    k3 = f(0, x + h/2*k2);
    k4 = f(0, x + h*k3);
    x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
