%% test_10_trajectory.m  --  Track a moving CIRCLE on estimated state
% Full system: estimator-in-the-loop, feedforward controller, 2 laps of a circle.
% Drone starts at origin -> spirals onto the circle -> tracks it tightly.

clear; clc;
model_params;
rng(0);
P.traj.mode = 'circle';                 % override default 'step' (P is in base)

x0 = zeros(12,1);                        % start at origin (off the circle)
assignin('base','x0', x0);

out = sim('drone_plant','StopTime','20'); % 2 laps (T=10 s)
X = out.get('X_sl');  t = out.get('tout');

% reconstruct reference for comparison
w = 2*pi/P.traj.T;  R = P.traj.R;
xr = R*cos(w*t);  yr = R*sin(w*t);  zr = -P.traj.h*ones(size(t));

% RMS 3D tracking error AFTER the catch-up transient (t > 5 s)
idx = t > 5;
e_track = sqrt(mean( (X(idx,1)-xr(idx)).^2 + (X(idx,2)-yr(idx)).^2 + (X(idx,3)-zr(idx)).^2 ));
fprintf('\nRMS 3D tracking error (steady, t>5s) = %.3f m\n', e_track);
fprintf('peak tilt during tracking            = %.1f deg\n', ...
        max(vecnorm(X(:,7:8),2,2))*180/pi);
assert(e_track < 0.15, 'tracking error too large');
assert(all(isfinite(X(:))), 'sim diverged');
fprintf('\n>>> Step 11 PASSED. Tight circular trajectory tracking with feedforward.\n');

%% plots
figure('Color','w');
subplot(1,2,1);
plot(X(:,2), X(:,1), 'b','LineWidth',1.6); hold on;
plot(yr, xr, 'r--','LineWidth',1.2);
plot(0,0,'ko','MarkerFaceColor','k'); grid on; axis equal;
xlabel('East [m]'); ylabel('North [m]');
legend('flown','reference','start','Location','best'); title('top-down: circle tracking');
subplot(1,2,2);
err = sqrt((X(:,1)-xr).^2 + (X(:,2)-yr).^2 + (X(:,3)-zr).^2);
plot(t, err, 'b','LineWidth',1.4); grid on;
xlabel('t [s]'); ylabel('3D tracking error [m]');
title('error: catch-up then tight track');
whiten(gcf);
