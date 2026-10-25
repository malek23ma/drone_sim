function [best, hist] = ga_tune_pid(apply, popN, gens)
%GA_TUNE_PID  Genetic-Algorithm tuning of the position+attitude PID gains.
%   best = ga_tune_pid(apply, popN, gens)
%     apply : (default true) write the optimized gains back into base P
%     popN  : population size (default 30)
%     gens  : generations    (default 60)
%
%   Tunes x = [Kp_pos_xy, Kd_pos_xy, Ki_pos_xy, Kp_att_rp, Kd_att_rp] by
%   minimizing ctrl_cost (same closed-loop step cost as the PSO tuner), so the
%   two methods are directly comparable. No toolbox required -- hand-rolled GA:
%   tournament selection, blend crossover, Gaussian mutation, elitism.

    if nargin < 1 || isempty(apply), apply = true;  end
    if nargin < 2 || isempty(popN),  popN  = 20;    end
    if nargin < 3 || isempty(gens),  gens  = 40;    end

    P  = evalin('base','P');
    %        Kp_pos Kd_pos Ki_pos Kp_att Kd_att   (same band as the PSO tuner)
    lb = [   0.5    0.5    0.0    30      9  ];   % Kd_att floor -> keeps damping
    ub = [   6.0    6.0    3.0   110     28  ];
    nv = numel(lb);

    x0   = [P.Kp_pos(1) P.Kd_pos(1) P.Ki_pos(1) P.Kp_att(1) P.Kd_att(1)];
    Jdef = ctrl_cost(min(max(x0,lb),ub), P);

    % ---- init population (seed one individual at the current gains) ----
    pop = lb + (ub-lb).*rand(popN,nv);
    pop(1,:) = min(max(x0,lb),ub);
    fit = zeros(popN,1);
    for i = 1:popN, fit(i) = ctrl_cost(pop(i,:), P); end
    [gbf, gi] = min(fit);  gb = pop(gi,:);

    pm    = 0.20;                 % per-gene mutation probability
    sigma = 0.15*(ub-lb);        % mutation step size per gene
    stagn = 0;  prev = inf;

    fprintf('\nGA PID tuning: pop %d x %d gens  (baseline cost %.3f)\n', popN, gens, Jdef);
    hist = [];
    for g = 1:gens
        newpop = zeros(popN,nv);
        newpop(1,:) = gb;                                   % elitism: keep the best
        for k = 2:popN
            p1 = tourn(pop,fit);  p2 = tourn(pop,fit);      % tournament selection
            a  = rand(1,nv);  child = a.*p1 + (1-a).*p2;    % blend crossover
            mk = rand(1,nv) < pm;                           % Gaussian mutation
            child = child + mk.*(sigma.*randn(1,nv));
            newpop(k,:) = min(max(child, lb), ub);          % clamp to bounds
        end
        pop = newpop;
        for i = 1:popN, fit(i) = ctrl_cost(pop(i,:), P); end
        [bf, bi] = min(fit);
        if bf < gbf, gbf = bf;  gb = pop(bi,:); end
        fprintf('  gen %2d/%d   best cost = %.4f\n', g, gens, gbf);
        hist(end+1) = gbf;                                 %#ok<AGROW>  convergence history
        if gbf < prev - 1e-3, prev = gbf;  stagn = 0;
        else, stagn = stagn + 1;  if stagn >= 10, fprintf('  converged (no gain for 10 gens)\n'); break; end
        end
    end

    best = gb;
    fprintf('\n--- GA result ---\n');
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

function x = tourn(pop, fit)
%TOURN  3-way tournament: pick 3 at random, return the fittest (lowest cost).
    idx = randi(size(pop,1), 1, 3);
    [~, b] = min(fit(idx));
    x = pop(idx(b), :);
end
