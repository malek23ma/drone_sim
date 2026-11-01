%% test_field_preview.m  --  Preview the autonomous lawnmower coverage path
% Plots the reference path traj_islfcn produces in 'field' mode, BEFORE wiring
% it into the closed loop. Confirms swath spacing / field size look right.
clear; clc;
model_params;
P.traj.mode = 'field';  assignin('base','P',P);     % traj_islfcn reads P from base

% rough total mission time = path length / speed (+ margin)
N      = max(1, round(P.field.W/P.field.sw));
Lpath  = (N+1)*P.field.Lp + N*P.field.sw;            % passes + east crossovers
Tend   = 1.1 * Lpath / P.field.v;

tt = 0:0.1:Tend;
XY = zeros(numel(tt),2);
for k = 1:numel(tt)
    r = traj_islfcn(tt(k));
    XY(k,:) = r(1:2)';                               % [North, East]
end

figure('Color','w'); hold on; grid on; axis equal;
rectangle('Position',[P.field.corner(2) P.field.corner(1) P.field.W P.field.Lp], ...
          'EdgeColor',[0 0.6 0],'LineWidth',1.5);
hPath  = plot(XY(:,2), XY(:,1), 'b-', 'LineWidth',1.4);
hStart = plot(XY(1,2), XY(1,1), 'ko','MarkerFaceColor','k');
xlabel('East [m]'); ylabel('North [m]');
title(sprintf('Lawnmower path  -  %d passes, %.0f m swath, %.0f s mission', ...
              N+1, P.field.sw, Tend));
legend([hPath hStart], {'path','start'}, 'Location','bestoutside');

fprintf('Field %gx%g m, swath %g m -> %d passes, ~%.0f s at %g m/s\n', ...
        P.field.W, P.field.Lp, P.field.sw, N+1, Tend, P.field.v);
