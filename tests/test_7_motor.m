%% test_7_motor.m  --  Closed-loop flight WITH motor spool dynamics
% The 20 ms first-order lag now sits between Mixer and Forces. Verify:
%   - the drone still flies to the setpoint and holds (lag << loop time -> stable)
%   - rotor speeds spool smoothly (no instant jumps) -> the realism we added
%
% Requires a "To Workspace" block named w_log (variable W_sl) on the motor output.

clear; clc;
model_params;

x0  = zeros(12,1);     assignin('base','x0', x0);
ref = [1; 1; -2; 0];   assignin('base','ref', ref);

out = sim('drone_plant','StopTime','12');
X = out.get('X_sl');  t = out.get('tout');
pos = X(:,1:3);

perr = pos(end,:)' - ref(1:3);
dist = vecnorm(pos - ref(1:3)', 2, 2);
is   = find(dist > 0.05, 1, 'last');  ts = t(max(is,1));
fprintf('\nfinal position error = %.3e m\n', norm(perr));
fprintf('settles within 5 cm at t = %.2f s  (Step 5 was 2.59 s; lag adds a little)\n', ts);
assert(norm(perr) < 0.05, 'did not reach setpoint with motor lag');
assert(all(isfinite(X(:))), 'sim diverged');
fprintf('\n>>> Step 7 PASSED. Flight stable with realistic motor spool dynamics.\n');

%% rotor-speed plot (if logged)
figure('Color','w');
subplot(1,2,1);
plot(t, -pos(:,3),'g', t, pos(:,1),'b', t, pos(:,2),'r','LineWidth',1.4); grid on;
xlabel('t [s]'); ylabel('position [m]'); legend('Up','North','East','Location','best');
title('flight with motor lag');
try
    W = out.get('W_sl');  tw = t;
    if size(W,1) ~= numel(tw), tw = linspace(t(1),t(end),size(W,1)); end
    subplot(1,2,2);
    plot(tw, W, 'LineWidth',1.2); grid on; hold on;
    yline(P.omega_hover,'k--');
    xlabel('t [s]'); ylabel('\omega_i [rad/s]');
    legend('\omega_1','\omega_2','\omega_3','\omega_4','hover','Location','best');
    title('rotor spool dynamics (\tau = 20 ms)');
catch
    fprintf('(add a To Workspace "w_log" / var W_sl on the motor output to see rotor speeds)\n');
end
