%% test_5_position.m  --  Closed-loop cascaded position control
% Command a 3D setpoint and verify the drone flies there and holds it.
% ref = [x; y; z; psi]  in NED  (z = -2 means 2 m ALTITUDE, since +z is down).

clear; clc;
model_params;

x0  = zeros(12,1);          assignin('base','x0', x0);
ref = [1; 1; -2; 0];        assignin('base','ref', ref);   % 1m N, 1m E, 2m up, yaw 0

out = sim('drone_plant','StopTime','12');
X = out.get('X_sl');  t = out.get('tout');
pos = X(:,1:3);  eul = X(:,7:9);

%% steady-state + settling
perr = pos(end,:)' - ref(1:3);
fprintf('\nfinal pos  = [% .3f % .3f % .3f] m\n', pos(end,:));
fprintf('target     = [% .3f % .3f % .3f] m\n', ref(1:3));
fprintf('final error= %.3e m\n', norm(perr));

% settling time on the slowest axis (within 5 cm of target)
dist = vecnorm(pos - ref(1:3)', 2, 2);
is   = find(dist > 0.05, 1, 'last');
ts   = t(max(is,1));
fprintf('settles within 5 cm at t = %.2f s\n', ts);
fprintf('peak tilt during flight = %.1f deg\n', max(vecnorm(eul(:,1:2),2,2))*180/pi);

assert(norm(perr) < 0.05, 'did not reach setpoint');
assert(all(isfinite(X(:))), 'sim diverged');
fprintf('\n>>> Step 5 PASSED. Cascaded position control: drone flies to setpoint and holds.\n');

%% plots
figure('Color','w');
subplot(1,2,1);
plot3(pos(:,2), pos(:,1), -pos(:,3), 'b','LineWidth',1.6); hold on;   % E,N,Up
plot3(ref(2), ref(1), -ref(3), 'rp','MarkerSize',14,'MarkerFaceColor','r');
plot3(0,0,0,'ko','MarkerFaceColor','k'); grid on; axis equal;
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
legend('path','target','start','Location','best'); title('3D trajectory to setpoint');
subplot(1,2,2);
plot(t, pos(:,1),'b', t, pos(:,2),'r', t, -pos(:,3),'g','LineWidth',1.4); grid on; hold on;
yline(ref(1),'b--'); yline(ref(2),'r--'); yline(-ref(3),'g--');
xlabel('t [s]'); ylabel('position [m]');
legend('North','East','Up','Location','best'); title('position vs time');
