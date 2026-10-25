function T = fc_record(port, duration_s, outname)
%FC_RECORD  Record the firmware debug stream to a timetable + .mat/.csv.
%   T = fc_record("COM5", 120, "log_static")
%   Produces bench/logs/log_static.mat and .csv.
%   Columns: t, acc(g), gyro(dps), mag(uT), quat, euler(deg), fdt, cdt, pwm...
    arguments
        port (1,1) string
        duration_s (1,1) double = 60
        outname (1,1) string = "fc_log"
    end
    sp = fc_open(port);
    cleanup = onCleanup(@() delete(sp)); %#ok<NASGU>

    N = ceil(duration_s * 55);           % 50 Hz + margin
    data = nan(N, 25);
    n = 0; t0 = tic;
    fprintf('Recording %g s from %s ...\n', duration_s, port);
    while toc(t0) < duration_s
        s = fc_parse_line(readline(sp));
        if isempty(s), continue; end
        n = n + 1;
        data(n,:) = [s.t, s.acc, s.gyro, s.mag, s.quat, s.euler, ...
                     s.fdt, s.cdt, s.pwm, s.armed, s.hb];
        if mod(n, 250) == 0
            fprintf('  %5d samples (%.0f s)\n', n, toc(t0));
        end
    end
    data = data(1:n,:);

    names = ["t","ax","ay","az","gx","gy","gz","mx","my","mz", ...
             "q0","q1","q2","q3","roll","pitch","yaw","fdt","cdt", ...
             "m1","m2","m3","m4","armed","hb"];
    T = array2table(data, 'VariableNames', names);

    here = fileparts(mfilename('fullpath'));
    logdir = fullfile(here, 'logs');
    if ~exist(logdir, 'dir'), mkdir(logdir); end
    save(fullfile(logdir, outname + ".mat"), 'T');
    writetable(T, fullfile(logdir, outname + ".csv"));
    fprintf('Saved %d samples -> %s\n', n, fullfile(logdir, outname + ".mat"));
end
