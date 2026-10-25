%% test_4_attitude.m  --  Inner attitude-loop DESIGN check (analytic, pure .m)
% Validates the pole-placement gains directly: the closed inner loop on each axis
% is th'' + Kd*th' + Kp*th = Kp*ref (the J cancels). We simulate the roll axis
% step and confirm overshoot/settle/rise match the 2nd-order target (wn, zeta).
% Independent of the full model -> never rots as the architecture grows.

clear; clc;
model_params;

Kp = P.Kp_att(1);  Kd = P.Kd_att(1);          % roll-axis designed gains
wn = P.wn_att(1);  zeta = P.zeta_att(1);
ref = 10*pi/180;                               % 10 deg step

f = @(t,x) [ x(2); Kp*(ref - x(1)) - Kd*x(2) ];   % closed-loop 2nd-order
[t, x] = ode45(f, [0 3], [0;0]);
phi = x(:,1);

ss   = ref;
Mp   = max(0,(max(phi)-ss))/ss*100;            % overshoot %
ilast= find(abs(phi-ss) > 0.02*ss, 1, 'last'); ts = t(max(ilast,1));   % 2% settle
i10  = find(phi>=0.1*ss,1); i90 = find(phi>=0.9*ss,1); tr = t(i90)-t(i10);

Mp_pred = exp(-zeta*pi/sqrt(1-zeta^2))*100;
ts_pred = 4/(zeta*wn);
tr_pred = (1 + 1.1*zeta + 1.4*zeta^2)/wn;

fprintf('\n              measured     predicted\n');
fprintf('overshoot %%   %6.2f       %6.2f\n', Mp, Mp_pred);
fprintf('settle 2%% s   %6.3f       %6.3f\n', ts, ts_pred);
fprintf('rise 10-90 s  %6.3f       %6.3f\n', tr, tr_pred);

assert(abs(Mp - Mp_pred) < 1.0,  'overshoot off design');
assert(abs(ts - ts_pred) < 0.25, 'settling off design');
fprintf('\n>>> Step 4 PASSED. Inner attitude loop matches its 2nd-order pole-placement design.\n');

figure('Color','w');
plot(t, phi*180/pi,'b','LineWidth',1.8); hold on; yline(ref*180/pi,'r--','LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('\phi [deg]');
legend('roll response','command','Location','SouthEast');
title('Step 4: attitude loop vs 2nd-order design');
whiten(gcf);
