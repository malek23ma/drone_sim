%% test_2b_rotational.m  --  Unit-test the rotational path of quad_plant
% Two analytic checks that pin down BOTH the J^-1 mapping and the gyroscopic term:
%   (A) ENERGY CONSERVATION : with tau=0, rotational KE = 0.5*w'*J*w is INVARIANT.
%       The cross-product term does no work, so any drift = a sign/structure bug.
%   (B) SPIN-UP             : from rest, tau=[tx;0;0] -> p(t) ~ (tx/Jx)*t  (short t,
%       before coupling kicks in). Pins the J^-1 scaling.
% Kinematics still stubbed -> attitude frozen, so this isolates rotation only.

clear; clc;
model_params;

%% (A) ENERGY CONSERVATION  ----------------------------------------------
x0 = zeros(12,1);
x0(10:12) = [2.0; -1.0; 3.0];          % arbitrary tumbling, all 3 axes (forces coupling)
u_free = [0;0;0;0];                    % NO torque
[tA, xA] = ode45(@(t,x) quad_plant(t,x,u_free,P), [0 5], x0, ...
                 odeset('RelTol',1e-9,'AbsTol',1e-9));

w  = xA(:,10:12);
KE = 0.5*sum((w*P.J).*w, 2);           % 0.5 * w' J w  at each step
drift = max(abs(KE - KE(1)))/KE(1);
fprintf('\n[A] ENERGY     : KE0=%.6f J  drift=%.2e (relative)\n', KE(1), drift);
assert(drift < 1e-6, 'ENERGY NOT CONSERVED -> gyroscopic term sign/structure wrong');

%% (B) SPIN-UP LINEARITY  -------------------------------------------------
x0 = zeros(12,1);
tx = 0.01;                             % small torque about body x [N*m]
u_spin = [0; tx; 0; 0];
[tB, xB] = ode45(@(t,x) quad_plant(t,x,u_spin,P), [0 0.3], x0);
p_sim   = xB(:,10);
p_exact = (tx/P.Jx)*tB;                % ideal: Jx*pdot = tx
err = max(abs(p_sim - p_exact));
fprintf('[B] SPIN-UP    : p(0.3s) sim=%.4f  ideal=%.4f rad/s | maxerr=%.2e\n',...
        p_sim(end), p_exact(end), err);
assert(err < 1e-3, 'SPIN-UP scaling wrong -> check Jinv');

fprintf('\n>>> Step 2b PASSED. Rotational dynamics + gyroscopic coupling verified.\n');

%% plots
figure('Color','w');
subplot(1,2,1);
plot(tA, KE, 'b-','LineWidth',1.6); grid on;
xlabel('t [s]'); ylabel('rotational KE [J]');
title('(A) energy invariant (tau=0)'); ylim([KE(1)*0.99 KE(1)*1.01]);
subplot(1,2,2);
plot(tB, p_sim,'b-','LineWidth',1.6); hold on;
plot(tB, p_exact,'r--','LineWidth',1.2); grid on;
xlabel('t [s]'); ylabel('p [rad/s]');
legend('sim','(\tau_x/J_x) t','Location','NorthWest');
title('(B) spin-up linear');
