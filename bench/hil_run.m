function hil_run(port, Tend)
%HIL_RUN  True hardware-in-the-loop: MATLAB simulates the 6-DOF plant,
%   the REAL ESP32-S3 (flashed with HIL_MODE=1) runs sensor fusion + PID +
%   mixer and returns motor PWMs over USB.
%
%   hil_run("COM5")        20 s run
%
%   Sequence: 2 s disarmed level (Madgwick converges) -> arm (throttle low)
%   -> hover throttle with a 10 deg roll kick at t=8 s -> watch the real
%   firmware stabilize the simulated airframe.
%
%   REQUIRES firmware built with HIL_MODE=1 (components/common/nav_state.h).
    if nargin < 2, Tend = 20; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    addpath(root);                           % quad_plant.m lives in sim root
    run(fullfile(root, 'model_params.m'));   % P

    sp = fc_open(port);
    cleanup = onCleanup(@() delete(sp)); %#ok<NASGU>

    dt_ctl  = 1/50;                  % serial exchange rate
    n_sub   = 5;                     % plant substeps -> 250 Hz integration
    dt_sub  = dt_ctl / n_sub;
    N       = round(Tend / dt_ctl);

    hover_pwm = 1000 + round(P.omega_hover/970*1000);
    x   = zeros(12,1);
    pwm = [1000 1000 1000 1000];
    u   = pwm_to_forces(pwm, P);
    vd  = zeros(3,1);

    LOG = nan(N, 14);   % t, plant euler(3), fw euler(3), pwm(4), armed, thr, kick
    fprintf('HIL run: %g s. Arm at t=2, roll kick at t=8.\n', Tend);

    for k = 1:N
        t = k * dt_ctl;

        % ---- 1. synth sensors from current plant state, send to FC ----
        [acc, gyr, mag] = sensor_synth(x, vd, P);
        writeline(sp, sprintf('S,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f', ...
                              acc, gyr, mag));

        % ---- 2. RC schedule ----
        if t < 2
            thr = 1000; arm = 1000;          % disarmed, filter converging
        elseif t < 2.5
            thr = 1000; arm = 1700;          % arm at low throttle
        else
            thr = hover_pwm; arm = 1700;     % hover, level command
        end
        writeline(sp, sprintf('R,1500,1500,%d,1500,%d', thr, arm));

        % ---- 3. read newest D line (drain buffer) ----
        s = [];
        while sp.NumBytesAvailable > 60
            cand = fc_parse_line(readline(sp));
            if ~isempty(cand), s = cand; end
        end
        if isempty(s)
            s = fc_parse_line(readline(sp));   % block for one
        end
        if ~isempty(s)
            pwm = s.pwm(:)';
            u   = pwm_to_forces(pwm, P);
        end

        % ---- 4. kick disturbances: roll at t=8, pitch at t=13 ----
        kick = 0;
        if abs(t - 8) < dt_ctl/2
            x(10) = x(10) + deg2rad(120);      % p += 120 deg/s impulse
            kick = 1;
        end
        if abs(t - 13) < dt_ctl/2
            x(11) = x(11) + deg2rad(120);      % q += 120 deg/s impulse
            kick = 1;
        end

        % ---- 5. integrate plant (hold u), capture accel for next synth ----
        for j = 1:n_sub
            xd = quad_plant(0, x, u, P);
            x  = x + dt_sub * xd;              % Euler @250 Hz: fine here
        end
        vd = xd(4:6);

        if ~isempty(s)
            LOG(k,:) = [t, rad2deg(x(7:9))', s.euler, pwm, s.armed, thr, kick];
        end
    end

    %% ---- plots ----
    L = LOG(~isnan(LOG(:,1)),:);
    t = L(:,1);
    figure('Name','HIL run','Color','w');
    subplot(3,1,1); hold on; grid on;
        plot(t, L(:,2), 'b', t, L(:,3), 'r', t, L(:,4), 'g');
        plot(t, L(:,5), 'b--', t, -L(:,6), 'r--', t, -L(:,7), 'g--');
        legend('\phi plant','\theta plant','\psi plant', ...
               'fw roll','-fw pitch','-fw yaw');
        ylabel('deg'); title('plant truth (solid) vs firmware estimate (dashed, mapped to NED)');
    subplot(3,1,2); plot(t, L(:,8:11)); grid on;
        legend('M1 FR','M2 FL','M3 RR','M4 RL'); ylabel('PWM');
    subplot(3,1,3); plot(t, L(:,12), t, L(:,13)/2000, t, L(:,14)); grid on;
        legend('armed','thr/2000','kick'); xlabel('t [s]');

    % estimator agreement (roll axis, after convergence, NED mapping)
    w = t > 4;
    est_err = rms(L(w,2) - L(w,5));
    fprintf('RMS(plant phi - fw roll) after t=4s: %.2f deg\n', est_err);
    fprintf(['If attitude tracks and the quad re-levels after the kick: the REAL\n' ...
             'firmware (fusion+PID+mixer, actual timing) flies the model. If an\n' ...
             'axis diverges -> same sign issue the SIL run points at.\n']);
end
