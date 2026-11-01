%% test_6_allocation.m  --  Validate mixer + forces (control allocation)
%   (A) ROUND-TRIP : forces(mixer(wrench)) == wrench  for feasible wrenches.
%       Pins Mix_inv as the true inverse of Mix. Pure math, no sim.
%   (B) HOVER      : a hover wrench maps to 4 equal rotor speeds = omega_hover.
%   (C) CLOSED-LOOP: with mixer+forces inserted, the Step-5 setpoint flight is
%       UNCHANGED (M*M^-1 = I), proving the chain is transparent when ideal.

clear; clc;
model_params;

%% (A) ROUND-TRIP  --------------------------------------------------------
maxerr = 0;
for k = 1:1000
    T   = P.m*P.g*(0.8 + 0.4*rand);           % thrust near hover
    tau = 0.05*(rand(3,1)-0.5);               % small torques (stay feasible)
    wrench = [T; tau];
    w  = mixer_islfcn(wrench);                % allocate
    wr = forces_islfcn(w);                    % physics forward
    maxerr = max(maxerr, norm(wr - wrench));
end
fprintf('\n[A] ROUND-TRIP : max||forces(mixer(w)) - w|| = %.3e over 1000 wrenches\n', maxerr);
assert(maxerr < 1e-9, 'Mix_inv is not the inverse of Mix');

%% (B) HOVER MAPPING  -----------------------------------------------------
w_hover = mixer_islfcn([P.m*P.g; 0; 0; 0]);
fprintf('[B] HOVER      : rotor speeds = [%.1f %.1f %.1f %.1f] rad/s (omega_hover=%.1f)\n',...
        w_hover, P.omega_hover);
assert(max(abs(w_hover - P.omega_hover)) < 1e-6, 'hover allocation wrong');

%% (C) CLOSED-LOOP UNCHANGED  --------------------------------------------
x0 = zeros(12,1);   assignin('base','x0', x0);
ref = [1; 1; -2; 0]; assignin('base','ref', ref);
out = sim('drone_plant','StopTime','12');
X = out.get('X_sl');
perr = X(end,1:3)' - ref(1:3);
fprintf('[C] CLOSED-LOOP: final position error = %.3e m (flight unchanged)\n', norm(perr));
assert(norm(perr) < 0.05, 'inserting mixer/forces broke the flight');

fprintf('\n>>> Step 6 PASSED. Allocation invertible, hover correct, flight transparent.\n');
