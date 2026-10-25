function mag_cal(port, duration_s)
%MAG_CAL  One-shot FULL magnetometer calibration (hard-iron + soft-iron).
%   mag_cal("COM4")        records 60 s
%   mag_cal("COM4", 90)    records 90 s
%
%   Rotate the board through as many orientations as possible — no
%   pattern needed, random tumbling is fine. Just make sure every side
%   of the board points down at some point (including upside-down).
%   Keep >50 cm from PC, speakers, steel.
%
%   Fits an ellipsoid to the raw cloud, computes offset + 3x3 soft-iron
%   matrix mapping it to a sphere of radius |B|_true, sends "MAGSET,..."
%   (ABSOLUTE — replaces stored cal, no stacking), firmware saves to NVS.
%   Run once; survives reboots.
    if nargin < 2, duration_s = 60; end
    B_TRUE = 48.1;   % uT, WMM2025 Istanbul/Basaksehir (user-verified wrldmagm)

    sp = fc_open(port);
    cleanup = onCleanup(@() delete(sp)); %#ok<NASGU>

    % reset to raw pass-through so we record uncalibrated data
    writeline(sp, "MAGSET,0,0,0");
    pause(0.5); flush(sp);

    fprintf('Recording %d s - tumble the board through ALL orientations\n', duration_s);
    M = zeros(0, 3);
    t0 = tic; last_msg = -1;
    while toc(t0) < duration_s
        s = fc_parse_line(readline(sp));
        if isempty(s), continue; end
        M(end+1, :) = s.mag; %#ok<AGROW>
        remaining = duration_s - floor(toc(t0));
        if remaining ~= last_msg
            fprintf('  %3d s left   (%d samples)\n', remaining, size(M,1));
            last_msg = remaining;
        end
    end
    if size(M,1) < 500
        error('Only %d samples - aborting.', size(M,1));
    end

    % ---- coverage check: 8 octants around the cloud centroid ----
    C = M - mean(M);
    oct = unique(sign(C) * [4;2;1]);       % signed octant code
    n_oct = numel(oct);
    fprintf('\nSamples: %d   octant coverage: %d/8\n', size(M,1), n_oct);
    if n_oct < 7
        warning(['Only %d/8 octants covered - some orientations missed ' ...
                 '(did the board go upside-down?). Fit may be poor; ' ...
                 'consider re-running.'], n_oct);
    end

    % ---- algebraic ellipsoid fit:  x'Qx + p'x = 1 ----
    x = M(:,1); y = M(:,2); z = M(:,3);
    D = [x.^2, y.^2, z.^2, 2*x.*y, 2*x.*z, 2*y.*z, x, y, z];
    v = D \ ones(size(M,1),1);
    Q = [v(1) v(4) v(5); v(4) v(2) v(6); v(5) v(6) v(3)];
    p = v(7:9);
    if any(eig(Q) <= 0)
        error(['Ellipsoid fit failed (not positive definite) - data too ' ...
               'thin or too distorted. Re-run with fuller rotations.']);
    end
    off = -0.5 * (Q \ p);                  % ellipsoid center = hard iron
    k   = 1 + off.' * Q * off;
    S   = B_TRUE * real(sqrtm(Q / k));     % maps to sphere radius B_TRUE
    S   = (S + S.') / 2;                   % enforce symmetry

    % ---- quality: calibrated magnitudes should sit tight on B_TRUE ----
    Mc = (S * (M - off.').').';
    mags = sqrt(sum(Mc.^2, 2));
    fprintf('Offset [uT]: %+7.2f %+7.2f %+7.2f\n', off);
    fprintf('S diag:      %7.3f %7.3f %7.3f\n', S(1,1), S(2,2), S(3,3));
    fprintf('|B| after cal: %.1f +/- %.1f uT (target %.1f)\n', ...
            mean(mags), std(mags), B_TRUE);
    if std(mags) > 0.1 * B_TRUE
        warning(['Residual spread %.1f uT is high - field was unstable ' ...
                 'during recording (moving metal? PC too close?).'], std(mags));
    end

    % ---- send absolutely, firmware saves to NVS ----
    St = S.';   % row-major for the firmware
    writeline(sp, sprintf(['MAGSET,%.4f,%.4f,%.4f,' ...
        '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f'], ...
        off(1), off(2), off(3), St(:)));
    pause(1);

    % ---- live verify: |B| from the now-calibrated stream ----
    flush(sp);
    vB = nan(1, 100);
    for i = 1:100
        s = fc_parse_line(readline(sp));
        if ~isempty(s), vB(i) = norm(s.mag); end
    end
    vB = vB(~isnan(vB));
    fprintf('Live |B| after MAGSET: %.1f +/- %.1f uT  <-- should be ~%.1f\n', ...
            mean(vB), std(vB), B_TRUE);

    figure('Name','Mag cal','Color','w');
    subplot(1,2,1); plot3(M(:,1),M(:,2),M(:,3),'.'); axis equal; grid on;
    title('raw (ellipsoid)'); xlabel x; ylabel y; zlabel z;
    subplot(1,2,2); plot3(Mc(:,1),Mc(:,2),Mc(:,3),'.'); axis equal; grid on;
    title(sprintf('calibrated (sphere r=%.0f)', B_TRUE));
    xlabel x; ylabel y; zlabel z;
end
