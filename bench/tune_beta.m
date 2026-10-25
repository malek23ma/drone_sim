function tune_beta(static_log, moving_log)
%TUNE_BETA  Sweep Madgwick beta on logged raw data.
%   tune_beta("log_static", "log_moves")
%
%   Two competing metrics:
%     static log : euler std-dev  (noise passed from accel/mag -> want LOW beta)
%     moving log : final settling error after motion, i.e. how fast the
%                  filter recovers from gyro-integration drift (-> want HIGH beta)
%   Pick beta at the knee. Firmware value: 0.04 (fusion.c MADGWICK_BETA).
    here  = fileparts(mfilename('fullpath'));
    Ls    = load(fullfile(here, 'logs', static_log + ".mat"));  Ts = Ls.T;
    Lm    = load(fullfile(here, 'logs', moving_log + ".mat"));  Tm = Lm.T;

    betas = [0.005 0.01 0.02 0.04 0.08 0.15 0.3];
    noise = zeros(numel(betas), 1);
    recov = zeros(numel(betas), 1);

    for i = 1:numel(betas)
        b = betas(i);

        % static: std of roll/pitch after 5 s warmup
        [~, e] = replay(Ts, b);
        w = Ts.t - Ts.t(1) > 5;
        noise(i) = mean(std(e(w,1:2)));

        % moving: mean |roll/pitch error vs its own final resting value|
        % over the last 20% of the log (assumes log ENDS at rest, level)
        [~, e2] = replay(Tm, b);
        tail = round(0.8*size(e2,1)) : size(e2,1);
        recov(i) = mean(mean(abs(e2(tail,1:2) - mean(e2(tail(end-20:end),1:2)))));
        fprintf('beta %.3f : static noise %.4f deg, recovery err %.4f deg\n', ...
                b, noise(i), recov(i));
    end

    figure('Name','Beta sweep','Color','w');
    yyaxis left;  semilogx(betas, noise, '-o'); ylabel('static noise std [deg]');
    yyaxis right; semilogx(betas, recov, '-s'); ylabel('post-motion recovery err [deg]');
    xlabel('\beta'); grid on; title('Madgwick \beta sweep (firmware = 0.04)');
    xline(0.04, 'k--', 'firmware');
end

function [q, e] = replay(T, beta)
    t  = T.t - T.t(1);
    dt = [median(diff(t)); diff(t)];
    [q, e] = madgwick_m([T.ax T.ay T.az], [T.gx T.gy T.gz], ...
                        [T.mx T.my T.mz], dt, beta);
end
