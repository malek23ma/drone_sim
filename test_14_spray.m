%% test_14_spray.m  --  Tank depletion: altitude hold while spraying (mass drops)
% Drone hovers at 2 m, sprays from t=4 s. Mass falls 2.18 -> 1.80 kg over ~14 s.
% The controller assumes full mass (P.m); as the craft lightens it would CLIMB,
% but the integral term holds altitude. Compare PID vs PD.
% Requires: tank subsystem feeding m(t) into the plant, + To Workspace m_log (M_sl).

clear; clc;
model_params;
rng(0);
P.traj.mode = 'step';  P.traj.set = [0;0;-2;0];     % hover at 2 m
P.spray.mode = 'on';   P.spray.t_on = 4;  P.spray.t_off = 20;
x0 = zeros(12,1);  assignin('base','x0', x0);

% ----- PID (integral on): should hold altitude -----
P.use_integral = 1;
out1 = sim('drone_plant','StopTime','35');     % tank empties ~18s; allow integral to recover
X1 = out1.get('X_sl');  t = out1.get('tout');
alt_pid = -X1(:,3);
M = []; try, M = out1.get('M_sl'); catch, end

% ----- PD (integral off): should drift up as it lightens -----
P.use_integral = 0;
out0 = sim('drone_plant','StopTime','35');
X0 = out0.get('X_sl');  alt_pd = -X0(:,3);

ss = t > 31;                                    % AFTER the ramp ends + integral recovers
fprintf('\nfinal mass (sprayed dry?)        : %.2f kg  (dry = %.2f)\n', ...
        P.m_dry + max(0, P.tank_full - P.spray.rate*(min(P.spray.t_off,22)-P.spray.t_on)), P.m_dry);
fprintf('altitude hold, PID (integral on) : mean %.3f m  (target 2.000)\n', mean(alt_pid(ss)));
fprintf('altitude drift, PD  (integral off): mean %.3f m\n', mean(alt_pd(ss)));
assert(abs(mean(alt_pid(ss)) - 2) < 0.05, 'PID did not hold altitude while spraying');
assert(mean(alt_pd(ss)) - 2 > 0.10, 'PD did not drift -> mass not changing (tank not wired)');
fprintf('\n>>> Step 14 PASSED. Altitude held while spraying (tank depletion handled by integral).\n');

%% plots
figure('Color','w');
subplot(1,2,1);
plot(t, alt_pid,'b','LineWidth',1.8); hold on; plot(t, alt_pd,'r','LineWidth',1.4);
yline(2,'k--'); xline(4,'k:','spray on'); grid on;
xlabel('t [s]'); ylabel('altitude [m]');
legend('PID (holds)','PD (drifts up)','Location','best'); title('altitude while spraying');
subplot(1,2,2);
if ~isempty(M)
    plot(t, M,'b','LineWidth',1.8); grid on; hold on;
    yline(P.m_dry,'k--','dry');
    xlabel('t [s]'); ylabel('total mass [kg]'); title('tank depletion m(t)');
else
    text(0.1,0.5,'add To Workspace m\_log (M\_sl) on m(t)'); axis off;
end
whiten(gcf);
