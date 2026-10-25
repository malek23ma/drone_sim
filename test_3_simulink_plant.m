%% test_3_simulink_plant.m  --  Simulink plant BRIDGE == .m oracle (by construction)
% The Simulink "Dynamics" block calls plant_islfcn, which wraps the validated
% quad_plant. This unit test proves the bridge reproduces the oracle EXACTLY for
% random states / wrenches / wind -- so the Simulink plant cannot drift from the
% physics validated in tests 2a-2d. (Pure .m; independent of the evolving model.)

clear; clc;
model_params;

maxerr = 0;
for k = 1:2000
    x    = [randn(6,1); 0.3*randn(3,1); randn(3,1)];     % plausible state
    u    = [P.m*P.g*(0.7+0.6*rand); 0.1*randn(3,1)];     % wrench near hover
    wind = randn(3,1);                                    % arbitrary disturbance
    a = quad_plant(0, x, u, P, wind);                     % oracle
    b = plant_islfcn([x; u; wind]);                       % Simulink bridge
    maxerr = max(maxerr, norm(a - b));
end
fprintf('\n[3] plant bridge vs oracle: max||plant_islfcn - quad_plant|| = %.3e (2000 cases)\n', maxerr);
assert(maxerr < 1e-12, 'Simulink plant bridge disagrees with oracle');
fprintf('>>> Step 3 PASSED. Simulink plant kernel is identical to the validated oracle.\n');
