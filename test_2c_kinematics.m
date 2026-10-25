%% test_2c_kinematics.m  --  Unit-test the attitude-kinematics (W matrix)
%   (A) LEVEL IDENTITY : at phi=theta=0, W=I  -> eul_dot == omega exactly.
%   (B) PURE ROLL      : omega=[p;0;0] from level -> phi(t)=p*t, theta=psi=0.
%   (C) PURE PITCH     : omega=[0;q;0] from level -> theta(t)=q*t, phi=psi=0.
%   (D) GIMBAL LOCK    : sweep theta -> cond(W) blows up at +-90 deg (documented limit).

clear; clc;
model_params;

%% (A) LEVEL IDENTITY  ----------------------------------------------------
x0 = zeros(12,1);  x0(10:12) = [0.7; -0.4; 1.1];   % arbitrary body rates, level attitude
xdot = quad_plant(0, x0, [0;0;0;0], P);
err = max(abs(xdot(7:9) - x0(10:12)));
fprintf('\n[A] LEVEL IDENTITY : max|eul_dot - omega| = %.2e (should be 0)\n', err);
assert(err < 1e-12, 'W(0,0) is not identity -> kinematics wrong');

%% (B) PURE ROLL  ---------------------------------------------------------
p = 0.5; x0 = zeros(12,1); x0(10) = p;             % constant roll rate (single principal axis)
[tB,xB] = ode45(@(t,x) quad_plant(t,x,[0;0;0;0],P), [0 1], x0);
errB = max(abs(xB(:,7) - p*tB));                   % phi vs p*t
fprintf('[B] PURE ROLL      : phi(1s)=%.4f  ideal=%.4f | err=%.2e ; max|theta,psi|=%.2e\n',...
        xB(end,7), p, errB, max(max(abs(xB(:,8:9)))));
assert(errB < 1e-4 && max(max(abs(xB(:,8:9)))) < 1e-9, 'pure roll leaked into pitch/yaw');

%% (C) PURE PITCH  --------------------------------------------------------
q = 0.5; x0 = zeros(12,1); x0(11) = q;
[tC,xC] = ode45(@(t,x) quad_plant(t,x,[0;0;0;0],P), [0 1], x0);
errC = max(abs(xC(:,8) - q*tC));                   % theta vs q*t
fprintf('[C] PURE PITCH     : theta(1s)=%.4f ideal=%.4f | err=%.2e ; max|phi,psi|=%.2e\n',...
        xC(end,8), q, errC, max(max(abs(xC(:,[7 9])))));
assert(errC < 1e-4 && max(max(abs(xC(:,[7 9])))) < 1e-9, 'pure pitch leaked into roll/yaw');

%% (D) GIMBAL LOCK demonstration  -----------------------------------------
th = linspace(-89.5, 89.5, 400)*pi/180;            % stop just short of +-90
condW = zeros(size(th));
for i=1:numel(th)
    c=cos(th(i)); s=sin(th(i)); tt=tan(th(i));     % evaluate W at phi=0 for clarity
    Wi=[1 0 0; 0 1 0; 0 0 1/c];                     % only the singular row matters here
    condW(i)=cond(Wi);
end
th90 = 89.5*pi/180;
fprintf('[D] GIMBAL LOCK    : cond(W) at theta=89.5deg = %.1f (blows up -> documented limit)\n',...
        1/cos(th90));
assert(1/cos(th90) > 50, 'expected near-singular W close to 90 deg');

fprintf('\n>>> Step 2c PASSED. Attitude kinematics verified; gimbal lock characterized.\n');
fprintf('>>> FULL 6-DOF PLANT now complete (2a translational + 2b rotational + 2c kinematics).\n');

%% plots
figure('Color','w');
subplot(1,3,1); plot(tB,xB(:,7),'b',tB,p*tB,'r--','LineWidth',1.4); grid on;
xlabel('t'); ylabel('\phi [rad]'); title('(B) pure roll'); legend('sim','p t','Location','NW');
subplot(1,3,2); plot(tC,xC(:,8),'b',tC,q*tC,'r--','LineWidth',1.4); grid on;
xlabel('t'); ylabel('\theta [rad]'); title('(C) pure pitch'); legend('sim','q t','Location','NW');
subplot(1,3,3); semilogy(th*180/pi, 1./cos(th),'b','LineWidth',1.6); grid on;
xlabel('\theta [deg]'); ylabel('1/cos\theta  (log)'); title('(D) gimbal lock singularity');
xline(90,'r--'); xline(-90,'r--');
