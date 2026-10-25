function [tt, POS] = sim_step(P)
%SIM_STEP  Closed-loop step response of the controller in P (for comparison).
%   Same fast oracle loop as ctrl_cost, but returns the trajectory.
%   Step: origin -> (N,E,D) = (2,2,-3). Returns time tt and position POS (Nx3).
    dt  = 0.004;  T = 6;  N = round(T/dt);
    sp  = [2; 2; -3];
    ref = [sp; 0;0;0; 0;0;0; 0];
    xk    = zeros(12,1);  e_int = zeros(3,1);  lim = P.int_limit;
    tt = zeros(N,1);  POS = zeros(N,3);
    for k = 1:N
        u  = pos_att_ctrl_islfcn([xk; ref; e_int], P);
        xk = xk + dt * quad_plant(0, xk, u, P);
        e_int = max(min(e_int + dt*(sp - xk(1:3)), lim), -lim);
        tt(k) = k*dt;  POS(k,:) = xk(1:3).';
    end
end
