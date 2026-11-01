%% test_13_integral.m  --  PID vs PD under CONSTANT wind (droop elimination)
% Hover, then a steady 1 N North wind switches on at t=6 s. Compare:
%   PD  (use_integral=0) -> permanent droop  e_ss = F/(m*Kp)
%   PID (use_integral=1) -> integral drives steady error to ~0

clear; clc;
model_params;
rng(0);
P.traj.mode = 'step';   P.traj.set = [0;0;-2;0];
P.wind.mode = 'const';  P.wind.F = [1;0;0];  P.wind.t_on = 6;
x0 = zeros(12,1);  assignin('base','x0', x0);

% ----- PID -----
P.use_integral = 1;
out1 = sim('drone_plant','StopTime','15');
X1 = out1.get('X_sl');  t = out1.get('tout');
north_pid = X1(:,1);

% ----- PD (same scenario, integral off) -----
P.use_integral = 0;
out0 = sim('drone_plant','StopTime','15');
north_pd = out0.get('X_sl');  north_pd = north_pd(:,1);

ss = t > 12;                                   % steady window
e_pid = mean(north_pid(ss));
e_pd  = mean(north_pd(ss));

fprintf('\nsteady North error under 1 N wind:\n');
fprintf('  PD  (no integral) = %.3f m   (textbook droop F/(m*Kp)=%.3f)\n', e_pd, 1/(P.m*P.Kp_pos(1)));
fprintf('  PID (integral on) = %.3f m\n', e_pid);
assert(abs(e_pid) < 0.05,             'PID did not remove droop');
assert(abs(e_pd)  > 4*abs(e_pid)+0.05,'integral made no clear difference');
fprintf('\n>>> Step 13 PASSED. Integral action eliminates steady-state wind droop.\n');

figure('Color','w');
plot(t, north_pd,'r','LineWidth',1.6); hold on;
plot(t, north_pid,'b','LineWidth',1.8); grid on;
yline(0,'k--'); xline(6,'k:','wind on');
xlabel('t [s]'); ylabel('North position [m]');
legend('PD (droops)','PID (returns to 0)','Location','best');
title('constant wind: PD droop vs PID rejection');
whiten(gcf);
