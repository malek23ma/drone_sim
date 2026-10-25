function mag_frame_check(logname)
%MAG_FRAME_CHECK  Test whether mag axes match the IMU body frame.
%   mag_frame_check("tumble")   — analyzes bench/logs/tumble.mat
%
%   Physics: for a rigid body rotating in Earth's constant field,
%   the body-frame field obeys  dm/dt = -omega x m.
%   Tries all 48 signed axis permutations R (mag -> IMU frame) and ranks
%   them by how well R satisfies that equation using the recorded gyro.
%   If "identity" is NOT the clear winner, the mag is mounted rotated /
%   flipped relative to the IMU, and the winner tells you the remap.
%
%   Record input with:  fc_record("COM4", 30, "tumble")
%   while rotating the board slowly about ALL three axes.
    here = fileparts(mfilename('fullpath'));
    L = load(fullfile(here, 'logs', logname + ".mat"));
    T = L.T;

    t  = T.t;
    w  = [T.gx, T.gy, T.gz] * 0.0174533;   % dps -> rad/s (body frame)
    m  = [T.mx, T.my, T.mz];               % uT (mag frame, cal applied)

    % light smoothing, then central-difference derivative
    m_s = movmean(m, 5, 1);
    w_s = movmean(w, 5, 1);
    dt  = gradient(t);
    dm  = [gradient(m_s(:,1)), gradient(m_s(:,2)), gradient(m_s(:,3))] ./ dt;

    % only samples with real rotation (SNR); discard still segments
    wn = vecnorm(w_s, 2, 2);
    use = wn > deg2rad(20);          % >20 dps
    if nnz(use) < 100
        error('Only %d samples with rotation - tumble the board more.', nnz(use));
    end
    fprintf('%d/%d samples have enough rotation\n', nnz(use), numel(use));

    % all 48 signed permutations
    P = perms(1:3);
    best = [];
    results = [];
    for i = 1:size(P,1)
        for sx = [-1 1], for sy = [-1 1], for sz = [-1 1] %#ok<*ALIGN>
            R = zeros(3);
            R(1,P(i,1)) = sx; R(2,P(i,2)) = sy; R(3,P(i,3)) = sz;
            mR  = (R * m_s')';
            dmR = (R * dm')';
            pred = -cross(w_s, mR, 2);          % -omega x m
            e = dmR(use,:) - pred(use,:);
            res = mean(vecnorm(e,2,2)) / mean(vecnorm(dmR(use,:),2,2));
            results(end+1,:) = [res, R(1,:), R(2,:), R(3,:), det(R)]; %#ok<AGROW>
        end, end, end
    end

    [~, order] = sort(results(:,1));
    fprintf('\nrank  residual  det  R rows (mag->IMU)\n');
    for k = 1:5
        r = results(order(k),:);
        fprintf('%2d    %6.3f    %+d   [%+d %+d %+d] [%+d %+d %+d] [%+d %+d %+d]%s\n', ...
            k, r(1), r(11), r(2:4), r(5:7), r(8:10), ...
            isequal(reshape(r(2:10),3,3)', eye(3)) * ' <-- identity' );
    end

    Rbest = reshape(results(order(1),2:10), 3, 3)';
    if isequal(Rbest, eye(3))
        fprintf(['\nVERDICT: mag frame MATCHES the IMU frame. ' ...
                 'The fusion bug is elsewhere.\n']);
    else
        fprintf(['\nVERDICT: mag frame is ROTATED/FLIPPED vs the IMU.\n' ...
                 'Apply in firmware before Madgwick: m_imu = R * m_mag with rows above.\n' ...
                 'det = %+d (%s)\n'], det(Rbest), ...
                 tern(det(Rbest) > 0, 'proper rotation - mounting', ...
                      'IMPROPER - an axis sign is wrong in a driver'));
    end

    % secondary check: did |B| stay inside the firmware validity gate
    % (10..120 uT) through the whole motion? If it left, the mag was
    % silently dropped (6-DOF) during those samples.
    Bn = vecnorm(m, 2, 2);
    fprintf('|B| over log: min %.1f  max %.1f uT (gate 10..120)\n', min(Bn), max(Bn));
    if min(Bn) < 10 || max(Bn) > 120
        fprintf('  *** |B| LEFT THE GATE during motion -> mag dropped out! ***\n');
    end

    % identity residual for reference
    id_row = find(arrayfun(@(k) isequal(reshape(results(k,2:10),3,3)', eye(3)), ...
                  1:size(results,1)));
    fprintf('identity residual: %.3f (winner: %.3f)\n', ...
            results(id_row,1), results(order(1),1));
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end
