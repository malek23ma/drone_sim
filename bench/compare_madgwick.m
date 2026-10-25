function compare_madgwick(logname)
%COMPARE_MADGWICK  Onboard Madgwick vs offline replay vs MATLAB filters.
%   compare_madgwick("log_moves")   loads bench/logs/log_moves.mat
%
%   Three traces per axis:
%     1. onboard euler (what the FC computed live)
%     2. offline madgwick_m replay of the same raw data (should overlay #1
%        almost exactly -> proves logging + port are faithful)
%     3. MATLAB Fusion Toolbox estimate (imufilter; ahrsfilter if mag ok)
%        -> independent reference, differences show Madgwick quality
    here = fileparts(mfilename('fullpath'));
    L = load(fullfile(here, 'logs', logname + ".mat"));
    T = L.T;

    acc = [T.ax T.ay T.az];          % g
    gyr = [T.gx T.gy T.gz];          % dps
    mag = [T.mx T.my T.mz];          % uT
    t   = T.t - T.t(1);
    dt  = [median(diff(t)); diff(t)];
    fs  = 1/median(diff(t));
    fprintf('Log: %d samples, %.1f s, %.1f Hz\n', height(T), t(end), fs);

    % --- 2. offline replay, firmware beta ---
    [~, eul_replay] = madgwick_m(acc, gyr, mag, dt, 0.04);

    % --- 3. MATLAB reference (needs Sensor Fusion & Tracking / Nav toolbox) ---
    eul_ref = []; ref_name = '';
    if exist('imufilter', 'class') || exist('imufilter', 'file')
        try
            % SI units: accel m/s^2, gyro rad/s. Sensor frame is z-up (NED
            % filters expect NED body) -> imufilter assumes accel = -gravity
            % when static with z-up reading +1g: matches our data directly.
            f = imufilter('SampleRate', fs);
            qref = f(acc * 9.81, deg2rad(gyr));
            e = eulerd(qref, 'ZYX', 'frame');      % [yaw pitch roll]
            eul_ref = [e(:,3) e(:,2) e(:,1)];
            ref_name = 'imufilter (6-DOF)';
        catch err
            warning('imufilter failed: %s', err.message);
        end
    else
        warning(['imufilter not found (Sensor Fusion and Tracking Toolbox ' ...
                 'or Navigation Toolbox missing) - skipping MATLAB reference.']);
    end

    % --- plots ---
    names = {'Roll','Pitch','Yaw'};
    figure('Name','Madgwick comparison','Color','w');
    for i = 1:3
        subplot(3,1,i); hold on; grid on;
        plot(t, T{:,14+i}, 'k', 'DisplayName', 'onboard');
        plot(t, eul_replay(:,i), 'b--', 'DisplayName', 'offline replay');
        if ~isempty(eul_ref)
            plot(t, eul_ref(:,i), 'r:', 'LineWidth', 1.2, 'DisplayName', ref_name);
        end
        ylabel([names{i} ' [deg]']); legend('Location','best');
    end
    xlabel('t [s]');

    % --- numbers ---
    d = wrap180(eul_replay - [T.roll T.pitch T.yaw]);
    fprintf('\nReplay vs onboard   RMS [deg]: R %.3f  P %.3f  Y %.3f\n', rms(d));
    fprintf('  (should be small; larger = dt/logging effects, NOT a bug per se)\n');
    if ~isempty(eul_ref)
        d2 = wrap180(eul_ref(:,1:2) - [T.roll T.pitch]);
        fprintf('MATLAB ref vs onboard RMS [deg]: R %.3f  P %.3f  (yaw refs differ)\n', rms(d2));
    end
end

function x = wrap180(x)
    x = mod(x + 180, 360) - 180;
end
