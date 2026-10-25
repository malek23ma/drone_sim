%% test_15_lqr.m  --  LQR full-state controller vs the cascaded PID
% Pure-MATLAB closed loop (ode4 + quad_plant oracle), identical step setpoint
% through BOTH controllers, overlaid. Validates the LQR drop-in BEFORE the
% Simulink canvas is rewired -- if the oracle flies, the block will too.
% ref = [x; y; z; psi] NED  (z = -2 => 2 m altitude).

clear; clc;
model_params;

dt = P.Ts;  Tend = 8;  N = round(Tend/dt);  t = (0:N)'*dt;
pos_ref = [1; 1; -2];  vel_ref = [0;0;0];  acc_ref = [0;0;0];  psi_ref = 0;

%% ---- LQR run (no integral term; pure regulator) ----
x = zeros(12,1);  XL = zeros(N+1,12);  XL(1,:) = x';
for k = 1:N
    in = [x; pos_ref; vel_ref; acc_ref; psi_ref; 0;0;0];
    u  = lqr_ctrl_islfcn(in);
    x  = rk4_step(x, u, P, dt);
    XL(k+1,:) = x';
end

%% ---- PID run (same plant, same setpoint, with integral action) ----
x = zeros(12,1);  eint = zeros(3,1);  XP = zeros(N+1,12);  XP(1,:) = x';
for k = 1:N
    in = [x; pos_ref; vel_ref; acc_ref; psi_ref; eint];
    u  = pos_att_ctrl_islfcn(in);
    x  = rk4_step(x, u, P, dt);
    eint = eint + (pos_ref - x(1:3))*dt;                 % integrate pos error
    eint = max(min(eint, P.int_limit), -P.int_limit);    % anti-windup clamp
    XP(k+1,:) = x';
end

%% ---- metrics ----
perrL = XL(end,1:3)' - pos_ref;
perrP = XP(end,1:3)' - pos_ref;
distL = vecnorm(XL(:,1:3) - pos_ref', 2, 2);
tsL   = t(max(find(distL > 0.05, 1, 'last'), 1));
tiltL = max(vecnorm(XL(:,7:8), 2, 2))*180/pi;

fprintf('\nLQR final error = %.3e m   settles(5cm) = %.2f s   peak tilt = %.1f deg\n', ...
        norm(perrL), tsL, tiltL);
fprintf('PID final error = %.3e m\n', norm(perrP));

assert(norm(perrL) < 0.05,        'LQR did not reach setpoint');
assert(all(isfinite(XL(:))),      'LQR sim diverged');
assert(tiltL < 45,                'LQR demanded excessive tilt');
fprintf('\n>>> Step 15 PASSED. LQR drop-in flies to setpoint and holds.\n');

%% ---- plots (LQR vs PID overlay) ----
figure('Color','w');
subplot(1,2,1);
plot3(XP(:,2), XP(:,1), -XP(:,3), 'b',  'LineWidth',1.4); hold on;   % PID
plot3(XL(:,2), XL(:,1), -XL(:,3), 'r--','LineWidth',1.6);            % LQR
plot3(pos_ref(2), pos_ref(1), -pos_ref(3), 'kp','MarkerSize',14,'MarkerFaceColor','k');
plot3(0,0,0,'ko','MarkerFaceColor','k'); grid on; axis equal;
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
legend('PID','LQR','target','start','Location','best'); title('LQR vs PID: 3D path');

subplot(1,2,2);
plot(t, -XP(:,3), 'b',  'LineWidth',1.4); hold on;
plot(t, -XL(:,3), 'r--','LineWidth',1.6);
plot(t,  XP(:,1), 'b',  'LineWidth',1.0);
plot(t,  XL(:,1), 'r--','LineWidth',1.2);
yline(-pos_ref(3),'k:'); yline(pos_ref(1),'k:'); grid on;
xlabel('t [s]'); ylabel('position [m]');
legend('PID alt','LQR alt','PID North','LQR North','Location','best');
title('LQR vs PID: step response');

%% ---- local ode4 (RK4) step: controller wrench held over the step ----
function xn = rk4_step(x, u, P, dt)
    f  = @(xx) quad_plant(0, xx, u, P, [0;0;0], P.m);
    k1 = f(x);  k2 = f(x + 0.5*dt*k1);  k3 = f(x + 0.5*dt*k2);  k4 = f(x + dt*k3);
    xn = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end
