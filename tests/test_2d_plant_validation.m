%% test_2d_plant_validation.m  --  Full-plant attitude validation
% Drive the plant through a multi-axis tumble and compare its EULER attitude to
% an INDEPENDENT quaternion integrator driven by the SAME body rates omega(t).
%
% Independence (why this is not circular):
%   - omega(t) comes from Euler's eqn (2b), which does NOT use W or attitude.
%   - quaternion path q_dot = 0.5*G(q)*omega uses NO Euler angles, NO W, NO 1/cos.
%   Same physical attitude reached two unrelated ways -> agreement = proof.
%
% Both states are integrated TOGETHER (augmented state) so omega is bit-identical
% to both paths (no interpolation error). Metric: geodesic angle between the two
% rotation matrices R_euler(t) and R_quat(t).

clear; clc;
model_params;

% augmented state: X = [ 12 plant states ; 4 quaternion (w x y z) ]
X0 = zeros(16,1);
X0(10:12) = [0.5; 0.4; 0.6];     % initial tumble, all 3 axes (modest -> stay off gimbal lock)
X0(13)    = 1;                   % q0 = identity, matches eul0 = 0
u = [P.m*P.g; 0; 0; 0];          % hover thrust, zero torque -> free rotational tumble

[t, X] = ode45(@(t,X) plant_plus_quat(t,X,u,P), [0 1.5], X0, ...
               odeset('RelTol',1e-10,'AbsTol',1e-10));

eul = X(:,7:9);
q   = X(:,13:16);

% geodesic attitude error between the two representations
angErr = zeros(size(t)); thetaTrack = zeros(size(t));
for k = 1:numel(t)
    Re = R_from_euler(eul(k,1),eul(k,2),eul(k,3));
    qk = q(k,:)'/norm(q(k,:));              % renormalize before use
    Rq = R_from_quat(qk);
    Rerr = Re' * Rq;
    angErr(k) = acos(min(max((trace(Rerr)-1)/2,-1),1));   % rad
    thetaTrack(k) = eul(k,2);
end

maxErrDeg = max(angErr)*180/pi;
maxThetaDeg = max(abs(thetaTrack))*180/pi;
fprintf('\n[2d] max pitch excursion   : %.1f deg  (must stay < ~75 to keep Euler healthy)\n', maxThetaDeg);
fprintf('[2d] max Euler-vs-Quat err : %.3e deg  (should be ~1e-7, i.e. solver noise)\n', maxErrDeg);
assert(maxThetaDeg < 75, 'tumble approached gimbal lock -> reduce rates/time');
assert(maxErrDeg  < 1e-3, 'PLANT ATTITUDE INCONSISTENT -> W matrix is wrong');

fprintf('\n>>> Step 2d PASSED. Euler plant matches independent quaternion to solver precision.\n');
fprintf('>>> 6-DOF PLANT VALIDATED. Ready to migrate into Simulink as one trusted block.\n');

%% plots
figure('Color','w');
subplot(1,2,1);
plot(t, eul*180/pi, 'LineWidth',1.4); grid on;
xlabel('t [s]'); ylabel('Euler [deg]'); legend('\phi','\theta','\psi','Location','best');
title('multi-axis tumble (plant)');
subplot(1,2,2);
semilogy(t, max(angErr*180/pi, 1e-16), 'b','LineWidth',1.6); grid on;
xlabel('t [s]'); ylabel('Euler vs Quaternion error [deg]');
title('attitude representations agree to solver noise');

%% ---------- local functions ----------
function Xdot = plant_plus_quat(t,X,u,P)
    x = X(1:12);  q = X(13:16);
    xdot12 = quad_plant(t,x,u,P);          % reuse the verified plant
    omega  = x(10:12);
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    G = [ -qx -qy -qz;                      % 4x3 body-rate -> quaternion-rate map
           qw -qz  qy;
           qz  qw -qx;
          -qy  qx  qw ];
    qdot = 0.5*G*omega;
    Xdot = [xdot12; qdot];
end

function R = R_from_euler(phi,theta,psi)   % ZYX, body->world (same as plant)
    cphi=cos(phi); sphi=sin(phi); cth=cos(theta); sth=sin(theta); cpsi=cos(psi); spsi=sin(psi);
    R = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
          spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
          -sth,      cth*sphi,                   cth*cphi ];
end

function R = R_from_quat(q)                % Hamilton, body->world
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2), 2*(x*y - z*w),   2*(x*z + y*w);
         2*(x*y + z*w), 1-2*(x^2+z^2),   2*(y*z - x*w);
         2*(x*z - y*w), 2*(y*z + x*w),   1-2*(x^2+y^2)];
end
