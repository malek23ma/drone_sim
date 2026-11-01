%% test_9_estimator.m  --  FULL closed loop on ESTIMATED state
% The controller now flies on xhat (from sensors only), not on truth.
% Verify: (1) drone still reaches the setpoint despite noise+bias,
%         (2) the estimator reconstructs VELOCITY it never measured,
%         (3) estimation error stays bounded.
%
% Requires To Workspace "xhat_log" (variable Xhat_sl) on the estimator output.

clear; clc;
model_params;
rng(0);

x0  = zeros(12,1);     assignin('base','x0', x0);
ref = [1; 1; -2; 0];   assignin('base','ref', ref);

out = sim('drone_plant','StopTime','12');
X  = out.get('X_sl');  Xh = out.get('Xhat_sl');  t = out.get('tout');
n = min([size(X,1) size(Xh,1) numel(t)]);  X=X(1:n,:); Xh=Xh(1:n,:); t=t(1:n);

pos  = X(:,1:3);   vel  = X(:,4:6);
posh = Xh(:,1:3);  velh = Xh(:,4:6);

perr   = pos(end,:)' - ref(1:3);
e_pos  = sqrt(mean(sum((pos - posh).^2,2)));        % RMS position estimate error
e_vel  = sqrt(mean(sum((vel - velh).^2,2)));        % RMS velocity estimate error (the prize)

fprintf('\nfinal TRUE position error vs setpoint : %.3f m\n', norm(perr));
fprintf('RMS position estimate error           : %.3f m\n', e_pos);
fprintf('RMS velocity estimate error           : %.3f m/s  (velocity was NEVER measured)\n', e_vel);

assert(norm(perr) < 0.20, 'did not hold setpoint on estimated state');
assert(e_vel < 0.25,      'velocity reconstruction poor');
assert(all(isfinite(X(:))), 'sim diverged');
fprintf('\n>>> Step 9 PASSED. Drone flies on estimated state. ALL 7 BLOCKS COMPLETE.\n');

%% plots: truth vs estimate
figure('Color','w');
subplot(1,2,1);
plot(t,-pos(:,3),'b','LineWidth',1.8); hold on; plot(t,-posh(:,3),'r--','LineWidth',1.2);
plot(t,pos(:,1),'g',t,posh(:,1),'m--','LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('position [m]');
legend('Up truth','Up est','North truth','North est','Location','best');
title('position: truth vs estimate');
subplot(1,2,2);
plot(t,-vel(:,3),'b','LineWidth',1.8); hold on; plot(t,-velh(:,3),'r--','LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('vertical speed [m/s]');
legend('truth','estimate','Location','best');
title('VELOCITY reconstructed (never measured)');
whiten(gcf);
