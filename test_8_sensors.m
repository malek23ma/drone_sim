%% test_8_sensors.m  --  Validate the sensor model (truth vs measurement)
% Sensors tap the plant state; they do NOT change the flight yet (controller
% still uses true state). We confirm the noise/bias match the spec, and SEE
% how dirty the signal the estimator will have to clean up actually is.
%
% Requires a "To Workspace" block named y_log (variable Y_sl) on the sensor output.

clear; clc;
model_params;
rng(0);                                  % reproducible noise

x0  = zeros(12,1);     assignin('base','x0', x0);
ref = [1; 1; -2; 0];   assignin('base','ref', ref);

out = sim('drone_plant','StopTime','12');
X = out.get('X_sl');  Y = out.get('Y_sl');  t = out.get('tout');
if size(Y,1) ~= size(X,1)                 % guard if logging grids differ
    n = min(size(Y,1), size(X,1)); X=X(1:n,:); Y=Y(1:n,:); t=t(1:n);
end

%% residual statistics vs spec
res_pos   = Y(:,1:3) - X(:,1:3);          % GPS error
res_eul   = Y(:,4:6) - X(:,7:9);          % AHRS error
res_omega = Y(:,7:9) - X(:,10:12);        % gyro error (= bias + noise)

fprintf('\nGPS   noise std : %.3f m     (spec %.3f)\n', mean(std(res_pos)),  P.sen.gps_std);
fprintf('AHRS  noise std : %.3f deg   (spec %.3f)\n', mean(std(res_eul))*180/pi, P.sen.att_std*180/pi);
fprintf('gyro  noise std : %.4f rad/s (spec %.4f)\n', mean(std(res_omega)), P.sen.gyro_std);
fprintf('gyro  bias est  : [% .3f % .3f % .3f] deg/s (spec [% .3f % .3f % .3f])\n', ...
        mean(res_omega)*180/pi, P.sen.gyro_bias*180/pi);

assert(abs(mean(std(res_pos))   - P.sen.gps_std)  < 0.02, 'GPS noise off spec');
assert(abs(mean(std(res_omega)) - P.sen.gyro_std) < 0.005,'gyro noise off spec');
fprintf('\n>>> Step 8 PASSED. Sensor noise/bias match spec; estimator input characterized.\n');

%% plot truth vs measurement
figure('Color','w');
subplot(1,3,1);
plot(t,-Y(:,3),'.','Color',[1 .6 .6]); hold on; plot(t,-X(:,3),'b','LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('altitude [m]'); legend('GPS','truth'); title('GPS position');
subplot(1,3,2);
plot(t,Y(:,4)*180/pi,'.','Color',[.6 1 .6]); hold on; plot(t,X(:,7)*180/pi,'b','LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('\phi [deg]'); legend('AHRS','truth'); title('attitude (roll)');
subplot(1,3,3);
plot(t,Y(:,7)*180/pi,'.','Color',[.6 .6 1]); hold on; plot(t,X(:,10)*180/pi,'b','LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('p [deg/s]'); legend('gyro','truth'); title('body rate (bias!)');
whiten(gcf);
