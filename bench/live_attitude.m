function live_attitude(port, duration_s)
%LIVE_ATTITUDE  Live 3D board visualization from the onboard Madgwick quaternion.
%   live_attitude("COM5")          runs for 60 s
%   live_attitude("COM5", 300)     runs for 300 s
%   Rotate the board by hand: the box must follow with no lag, no flips,
%   and yaw must hold steady (mag ok) or drift slowly (6-DOF fallback).
%   Baro altitude shows relative to the first samples (auto-zeroed);
%   lift the board ~1 m and the bar should rise. Expect +/-0.2 m noise.
    if nargin < 2, duration_s = 60; end
    sp = fc_open(port);
    cleanup = onCleanup(@() delete(sp)); %#ok<NASGU>

    % board-shaped box (sensor frame: x fwd, y left-ish, z up)
    [V, F] = box_geom(1.0, 0.6, 0.15);

    fig = figure('Name','FC Live Attitude','NumberTitle','off','Color','w');
    ax  = axes('Parent',fig); hold(ax,'on'); grid(ax,'on');
    h = patch(ax,'Vertices',V,'Faces',F,'FaceColor',[0.2 0.5 0.9], ...
              'FaceAlpha',0.85,'EdgeColor','k');
    quiver3(ax,0,0,0,1.4,0,0,'r','LineWidth',2);   % +x nose (red)
    axis(ax,'equal'); axis(ax,[-1.5 1.5 -1.5 1.5 -1.5 1.5]);
    xlabel(ax,'x (fwd)'); ylabel(ax,'y'); zlabel(ax,'z');
    view(ax,135,20);
    ttl = title(ax,'waiting for data...');

    % altitude bar: thin axes on the right, auto-zeroed on first samples
    axAlt = axes('Parent',fig,'Position',[0.92 0.15 0.04 0.7]);
    hBar = bar(axAlt, 0, 'FaceColor', [0.9 0.4 0.1]);
    ylim(axAlt, [-1 2]); xlim(axAlt, [0.5 1.5]);
    set(axAlt,'XTick',[]); ylabel(axAlt,'\Delta alt (m)');
    title(axAlt,'baro');

    alt0 = NaN;          % baseline (median of first N samples)
    alt_warm = [];       % warm-up buffer
    ALT_WARMUP = 25;     % ~0.5 s at 50 Hz

    t0 = tic;
    while toc(t0) < duration_s && ishghandle(fig)
        % Drain the serial buffer, keep only the NEWEST valid line —
        % otherwise the 50 Hz stream outruns the plot and the box freezes
        % on stale data.
        s = [];
        while sp.NumBytesAvailable > 120
            cand = fc_parse_line(readline(sp));
            if ~isempty(cand), s = cand; end
        end
        if isempty(s)
            s = fc_parse_line(readline(sp));   % block for one fresh line
        end
        if isempty(s), continue; end
        if ~ishghandle(fig), break; end   % figure closed mid-read
        q = s.quat;  % [w x y z]
        R = quat2rotm_local(q);
        set(h, 'Vertices', (R*V')');

        % relative baro altitude (NaN on old firmware without alt field)
        dalt = NaN;
        if isfield(s,'alt') && isfinite(s.alt)
            if isnan(alt0)
                alt_warm(end+1) = s.alt; %#ok<AGROW>
                if numel(alt_warm) >= ALT_WARMUP
                    alt0 = median(alt_warm);
                end
            else
                dalt = s.alt - alt0;
                set(hBar, 'YData', dalt);
            end
        end

        set(ttl, 'String', sprintf( ...
            'R %6.1f  P %6.1f  Y %6.1f   \\Deltaalt %+5.2f m   |q|=%.3f   %4.0f Hz fusion', ...
            s.euler(1), s.euler(2), s.euler(3), dalt, norm(q), 1/max(s.fdt,1e-4)));
        drawnow limitrate;
    end
end

function R = quat2rotm_local(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end

function [V,F] = box_geom(lx, ly, lz)
    [X,Y,Z] = ndgrid([-lx lx]/2, [-ly ly]/2, [-lz lz]/2);
    V = [X(:) Y(:) Z(:)];
    F = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
end
