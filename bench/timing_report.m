function timing_report(logname)
%TIMING_REPORT  Loop timing audit from a recorded log.
%   timing_report("log_static")
%   fdt = fusion loop dt (target 1/250 = 4 ms)
%   cdt = flight loop dt (target 4 ms)
%   hb  = fusion heartbeat -> true average fusion rate
    here = fileparts(mfilename('fullpath'));
    L = load(fullfile(here, 'logs', logname + ".mat"));
    T = L.T;
    t = T.t - T.t(1);

    stats = @(x) [mean(x) std(x) min(x) max(x)] * 1000;  % ms
    f = stats(T.fdt); c = stats(T.cdt);
    fprintf('Fusion loop dt: mean %.3f ms  std %.3f  min %.3f  max %.3f (target 4.000)\n', f);
    fprintf('Flight loop dt: mean %.3f ms  std %.3f  min %.3f  max %.3f (target 4.000)\n', c);

    hb_rate = (T.hb(end) - T.hb(1)) / (t(end) - t(1));
    fprintf('Fusion heartbeat rate: %.2f Hz (target 250)\n', hb_rate);
    fprintf('NOTE: dt sampled at 50 Hz -> catches sustained jitter, may miss\n');
    fprintf('      single spikes. Clamps in firmware: fusion 1..10 ms, flight 2..6 ms;\n');
    fprintf('      values pinned AT a clamp mean the real dt went beyond it.\n');

    figure('Name','Loop timing','Color','w');
    subplot(2,1,1); plot(t, T.fdt*1000); grid on;
    ylabel('fusion dt [ms]'); yline(4,'k--');
    subplot(2,1,2); plot(t, T.cdt*1000); grid on;
    ylabel('flight dt [ms]'); yline(4,'k--'); xlabel('t [s]');
end
