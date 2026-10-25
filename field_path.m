function [pos, vel, is_pass] = field_path(t, P)
%FIELD_PATH  Boustrophedon (lawnmower) path over a rectangle, constant speed.
%   SHARED source of truth, called by traj_islfcn (reference), spray_islfcn
%   (auto-spray) and viz_islfcn (coverage map) so the three never disagree.
%
%   Returns:
%     pos     NED position [N;E;D] at time t  (D = -alt)
%     vel     NED velocity [m/s]
%     is_pass true while on a North-South spraying PASS; false on the short East
%             crossover hops between swaths and when parked at the end.
%
%   The path snakes North<->South and steps East by f.sw each swath. Position is
%   derived purely from t (stateless) -> Simulink Interpreted-block compatible.
    f = P.field;
    N = max(1, round(f.W / f.sw));          % number of east steps

    % build waypoint list [North, East], snaking up/down and stepping east
    wp = zeros(0,2);
    e  = f.corner(2);
    for i = 0:N
        if mod(i,2) == 0
            wp(end+1,:) = [f.corner(1),        e];   %#ok<AGROW>  south end
            wp(end+1,:) = [f.corner(1)+f.Lp,   e];   %#ok<AGROW>  north end
        else
            wp(end+1,:) = [f.corner(1)+f.Lp,   e];   %#ok<AGROW>
            wp(end+1,:) = [f.corner(1),        e];   %#ok<AGROW>
        end
        e = min(e + f.sw, f.corner(2)+f.W);
    end

    % walk the path at constant speed until distance d is consumed
    d   = t * f.v;
    p   = wp(end,:);  dir = [0 0];          % default: parked at the end
    for k = 1:size(wp,1)-1
        seg = wp(k+1,:) - wp(k,:);
        L   = norm(seg);
        if L < 1e-9, continue; end
        if d <= L
            u   = seg / L;
            p   = wp(k,:) + u*d;
            dir = u;
            break;
        end
        d = d - L;
        p = wp(k+1,:);  dir = [0 0];
    end

    pos     = [p(1); p(2); -f.alt];         % NED (z = -alt)
    vel     = [dir(1)*f.v; dir(2)*f.v; 0];
    is_pass = abs(dir(1)) > abs(dir(2));    % North-South pass, not the East hop
end
