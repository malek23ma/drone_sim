%% test_2a_translational.m  --  Unit-test the translational path of quad_plant
% Two checks, both analytic so the verdict is objective:
%   (A) FREE FALL : T=0  -> drone accelerates +Z (down) at g. z(t)=0.5*g*t^2
%   (B) HOVER     : T=mg -> nothing moves. xdot stays ~0.
% Attitude is level and rotational dynamics are stubbed, so this isolates 2a only.

clear; clc;
model_params;                      % loads P + prints sanity block
P.cd = 0;                          % isolate ideal translation: no drag (analytic free fall)

x0 = zeros(12,1);                  % at origin, level, at rest

%% (A) FREE FALL  ---------------------------------------------------------
u_ff = [0;0;0;0];                  % zero thrust, zero torque
[tA, xA] = ode45(@(t,x) quad_plant(t,x,u_ff,P), [0 2], x0);

z_sim   = xA(:,3);                 % world Z (down +)
w_sim   = xA(:,6);                 % world Z velocity
z_exact = 0.5*P.g*tA.^2;           % analytic free fall
err = max(abs(z_sim - z_exact));
fprintf('\n[A] FREE FALL  : z(2s) sim=%.3f m  exact=%.3f m  | maxerr=%.2e m\n',...
        z_sim(end), z_exact(end), err);
assert(err < 1e-3, 'FREE FALL FAILED -> check gravity/thrust signs');

%% (B) HOVER  -------------------------------------------------------------
u_hov = [P.m*P.g; 0; 0; 0];        % exactly weight
xdot0 = quad_plant(0, x0, u_hov, P);
fprintf('[B] HOVER      : |vel_dot| = %.2e m/s^2  (should be ~0)\n', norm(xdot0(4:6)));
assert(norm(xdot0(4:6)) < 1e-9, 'HOVER FAILED -> thrust does not cancel weight');

fprintf('\n>>> Step 2a PASSED. Translational dynamics + signs verified.\n');

%% plot
figure('Color','w');
plot(tA, z_sim, 'b-', 'LineWidth',1.6); hold on;
plot(tA, z_exact, 'r--', 'LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('z (down +) [m]');
legend('sim','0.5 g t^2','Location','NorthWest');
title('Step 2a: free-fall matches analytic');
