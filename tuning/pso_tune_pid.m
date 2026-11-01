function [best, hist] = pso_tune_pid(apply, np, iters)
%PSO_TUNE_PID  Particle-Swarm tuning of the position+attitude PID gains.
%   best = pso_tune_pid(apply, np, iters)
%     apply : (default true) write the optimized gains back into base P
%     np    : swarm size  (default 30)
%     iters : iterations  (default 40)
%
%   Tunes x = [Kp_pos_xy, Kd_pos_xy, Ki_pos_xy, Kp_att_rp, Kd_att_rp] by
%   minimizing ctrl_cost (closed-loop step response on the .m oracle). No
%   toolbox required -- hand-rolled PSO. Standard PSO: each particle is pulled
%   toward its own best (pb) and the swarm best (gb); inertia w keeps momentum.

    if nargin < 1 || isempty(apply), apply = true;  end
    if nargin < 2 || isempty(np),    np    = 20;    end
    if nargin < 3 || isempty(iters), iters = 100;   end

    P  = evalin('base','P');
    % bounds kept in a stable band so tuned gains stay safe in the full loop
    %        Kp_pos Kd_pos Ki_pos Kp_att Kd_att
    lb = [   0.5    0.5    0.0    30      9  ];   % Kd_att floor -> keeps damping
    ub = [   6.0    6.0    3.0   110     28  ];
    nv = numel(lb);

    % baseline (current default gains) for a before/after comparison
    x0   = [P.Kp_pos(1) P.Kd_pos(1) P.Ki_pos(1) P.Kp_att(1) P.Kd_att(1)];
    Jdef = ctrl_cost(min(max(x0,lb),ub), P);

    % ---- init swarm ----
    X  = lb + (ub-lb).*rand(np,nv);
    X(1,:) = min(max(x0,lb),ub);         % seed one particle at the current gains
    V  = zeros(np,nv);
    pb = X;  pbf = inf(np,1);
    gb = X(1,:);  gbf = inf;
    w = 0.7;  c1 = 1.5;  c2 = 1.5;

    fprintf('\nPSO PID tuning: %d particles x %d iters  (baseline cost %.3f)\n', np, iters, Jdef);
    stagn = 0;  prev = inf;  hist = [];
    for it = 1:iters
        for i = 1:np
            J = ctrl_cost(X(i,:), P);
            if J < pbf(i), pbf(i) = J;  pb(i,:) = X(i,:); end
            if J < gbf,    gbf    = J;  gb      = X(i,:); end
        end
        for i = 1:np
            r1 = rand(1,nv);  r2 = rand(1,nv);
            V(i,:) = w*V(i,:) + c1*r1.*(pb(i,:)-X(i,:)) + c2*r2.*(gb-X(i,:));
            X(i,:) = min(max(X(i,:)+V(i,:), lb), ub);     % step + clamp to bounds
        end
        fprintf('  iter %2d/%d   best cost = %.4f\n', it, iters, gbf);
        hist(end+1) = gbf;                                 %#ok<AGROW>  convergence history
        if gbf < prev - 1e-3, prev = gbf;  stagn = 0;     % early stop on convergence
        else, stagn = stagn + 1;  if stagn >= 8, fprintf('  converged (no gain for 8 iters)\n'); break; end
        end
    end

    best = gb;
    fprintf('\n--- PSO result ---\n');
    fprintf('  Kp_pos_xy = %.2f   Kd_pos_xy = %.2f   Ki_pos_xy = %.2f\n', gb(1), gb(2), gb(3));
    fprintf('  Kp_att_rp = %.1f   Kd_att_rp = %.1f\n', gb(4), gb(5));
    fprintf('  cost: default %.3f  ->  optimized %.3f   (%.0f%% lower)\n', ...
            Jdef, gbf, 100*(1 - gbf/Jdef));

    if apply
        P.Kp_pos(1:2) = gb(1);  P.Kd_pos(1:2) = gb(2);  P.Ki_pos(1:2) = gb(3);
        P.Kp_att(1:2) = gb(4);  P.Kd_att(1:2) = gb(5);
        assignin('base','P',P);
        fprintf('  -> applied to base P (live; panel sliders reflect on reopen).\n');
    end
end
