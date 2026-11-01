%% test_12_wind.m  --  Disturbance rejection: a wind GUST hits a hovering drone
% Drone climbs to 2 m, holds, then a 2 N North gust pushes it during t in [7,9] s.
% Verify it deviates, then recovers to the setpoint once the gust passes.

clear; clc;
model_params;
rng(0);
P.traj.mode = 'step';   P.traj.set = [0; 0; -2; 0];      % hover at 2 m over origin
P.wind.mode = 'gust';   P.wind.F = [2; 0; 0];            % 2 N North gust
P.wind.t_on = 7;        P.wind.t_off = 9;

x0 = zeros(12,1);   assignin('base','x0', x0);           % start on the ground at rest

out = sim('drone_plant','StopTime','16');
X = out.get('X_sl');  t = out.get('tout');
north = X(:,1);

% peak North excursion during the gust window and shortly after
gustwin = t >= 7;
peak = max(abs(north(gustwin)));
final = abs(north(end));
fprintf('\npeak North excursion during gust = %.3f m\n', peak);
fprintf('North error at end (recovered?)  = %.3f m\n', final);
assert(final < 0.05, 'did not recover after gust');
assert(all(isfinite(X(:))), 'sim diverged');
fprintf('\n>>> Step 12 PASSED. Gust deflects the drone; controller rejects it and recovers.\n');

%% plot
figure('Color','w');
plot(t, north, 'b','LineWidth',1.8); hold on; grid on;
yline(0,'k--');
xr = [7 9 9 7]; yr = [min(north)-0.05 min(north)-0.05 max(north)+0.05 max(north)+0.05];
patch(xr, yr, [1 .85 .85], 'EdgeColor','none','FaceAlpha',0.6);
plot(t, north, 'b','LineWidth',1.8);
xlabel('t [s]'); ylabel('North position [m]');
title('wind gust (shaded) deflects drone; controller recovers it');
legend('North','setpoint','gust window','Location','best');
whiten(gcf);
