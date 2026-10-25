function compare_pid()
%COMPARE_PID  Prove the PSO-tuned gains beat the nominal hand-derived gains.
%   Runs the SAME step maneuver (origin -> 2,2,-3 m) with both gain sets on the
%   .m oracle, overlays the responses, and reports overshoot / settling / ISE.
%   "Nominal" = wn/zeta-derived defaults; "tuned" = whatever is in base P now
%   (i.e. the PSO result after Auto-tune). Run after Auto-tune.

    P = evalin('base','P');

    % nominal (hand-derived) gains
    Pn = P;  Pn.ctrl.mode = 'pid';
    Pn.Kp_pos = Pn.wn_pos.^2;  Pn.Kd_pos = 2*Pn.zeta_pos.*Pn.wn_pos;  Pn.Ki_pos = [1;1;2];
    Pn.Kp_att = Pn.wn_att.^2;  Pn.Kd_att = 2*Pn.zeta_att.*Pn.wn_att;

    % tuned gains = current base P (PSO result)
    Pt = P;  Pt.ctrl.mode = 'pid';

    [t, Pn_pos] = sim_step(Pn);
    [~, Pt_pos] = sim_step(Pt);
    dt = t(2)-t(1);  sp = [2 2 -3];

    [osN, tsN, iseN] = local_metrics(t, Pn_pos, sp, dt);
    [osT, tsT, iseT] = local_metrics(t, Pt_pos, sp, dt);

    en = sqrt(sum((Pn_pos - sp).^2, 2));        % 3D error magnitude vs time
    et = sqrt(sum((Pt_pos - sp).^2, 2));

    figure('Name','PID: nominal vs PSO-tuned','NumberTitle','off','Color','w','Position',[200 150 760 560]);

    subplot(2,1,1); hold on; grid on;
    plot(t, Pn_pos(:,1), 'Color',[0.2 0.4 0.9],'LineWidth',1.8);
    plot(t, Pt_pos(:,1), 'Color',[0.85 0.2 0.2],'LineWidth',1.8);
    yline(2,'k--');
    xlabel('time [s]'); ylabel('North position [m]');
    title('Step response (North axis):  0 \rightarrow 2 m');
    legend({sprintf('nominal  (OS %.0f%%, t_s %.2fs)', osN, tsN), ...
            sprintf('PSO-tuned (OS %.0f%%, t_s %.2fs)', osT, tsT), 'setpoint'}, ...
            'Location','southeast');

    subplot(2,1,2); hold on; grid on;
    plot(t, en, 'Color',[0.2 0.4 0.9],'LineWidth',1.8);
    plot(t, et, 'Color',[0.85 0.2 0.2],'LineWidth',1.8);
    xlabel('time [s]'); ylabel('3D position error [m]');
    title('Tracking error magnitude');
    legend({sprintf('nominal   (ISE %.2f)', iseN), ...
            sprintf('PSO-tuned (ISE %.2f)', iseT)}, 'Location','northeast');

    fprintf('\n=== PID comparison (step 0->2,2,-3 m) ===\n');
    fprintf('              overshoot   settling(2%%)   ISE(3D)\n');
    fprintf('  nominal   :   %5.1f%%      %6.2f s     %6.2f\n', osN, tsN, iseN);
    fprintf('  PSO-tuned :   %5.1f%%      %6.2f s     %6.2f\n', osT, tsT, iseT);
    fprintf('  improvement: ISE %.0f%% lower, settling %.0f%% faster\n', ...
            100*(1-iseT/iseN), 100*(1-tsT/max(tsN,eps)));
end

function [os, ts, ise] = local_metrics(t, POS, sp, dt)
    n  = POS(:,1);                                   % North axis (0 -> 2 step)
    os = max(0, (max(n) - sp(1)) / sp(1) * 100);     % % overshoot
    idx = find(abs(n - sp(1)) > 0.02*abs(sp(1)), 1, 'last');   % 2% settling
    if isempty(idx), ts = 0; else, ts = t(idx); end
    e   = sqrt(sum((POS - sp).^2, 2));
    ise = sum(e.^2) * dt;                            % integral squared error (3D)
end
