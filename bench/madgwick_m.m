function [quat, euler] = madgwick_m(acc, gyr, mag, dt, beta, q_init)
%MADGWICK_M  Offline replay of the firmware Madgwick AHRS (exact port of
%   components/fusion/fusion.c, incl. the 10..120 uT mag magnitude gate and
%   6-DOF fallback). Lets you re-run logged raw data with any beta.
%
%   acc  Nx3 [g]      gyr Nx3 [dps]     mag Nx3 [uT]
%   dt   Nx1 or scalar [s]              beta  scalar (firmware: 0.04)
%   q_init optional [w x y z]
%
%   quat Nx4, euler Nx3 [deg] (same quat_to_euler as firmware)
    N = size(acc,1);
    if isscalar(dt), dt = repmat(dt, N, 1); end
    if nargin < 6, q_init = [1 0 0 0]; end
    q = q_init(:)';
    quat = zeros(N,4); euler = zeros(N,3);

    for k = 1:N
        g = gyr(k,:) * 0.0174533;   % dps -> rad/s (same constant as firmware)
        a = acc(k,:);
        m = mag(k,:);
        m2 = m*m';
        mag_ok = (m2 > 100) && (m2 < 14400);   % firmware gate

        qDot = 0.5 * quatmul(q, [0 g]);

        if any(a ~= 0)
            a = a / norm(a);
            if mag_ok
                s = grad_9dof(q, a, m / norm(m));
            else
                s = grad_6dof(q, a);
            end
            qDot = qDot - beta * s / norm(s);
        end

        q = q + qDot .* dt(k);
        q = q / norm(q);
        quat(k,:) = q;
        euler(k,:) = q2eul(q);
    end
end

function r = quatmul(p, q)
    r = [p(1)*q(1) - p(2)*q(2) - p(3)*q(3) - p(4)*q(4), ...
         p(1)*q(2) + p(2)*q(1) + p(3)*q(4) - p(4)*q(3), ...
         p(1)*q(3) - p(2)*q(4) + p(3)*q(1) + p(4)*q(2), ...
         p(1)*q(4) + p(2)*q(3) - p(3)*q(2) + p(4)*q(1)];
end

function s = grad_6dof(q, a)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    ax=a(1); ay=a(2); az=a(3);
    s0 = 4*q0*q2^2 + 2*q2*ax + 4*q0*q1^2 - 2*q1*ay;
    s1 = 4*q1*q3^2 - 2*q3*ax + 4*q0^2*q1 - 2*q0*ay - 4*q1 + 8*q1^3 + 8*q1*q2^2 + 4*q1*az;
    s2 = 4*q0^2*q2 + 2*q0*ax + 4*q2*q3^2 - 2*q3*ay - 4*q2 + 8*q2*q1^2 + 8*q2^3 + 4*q2*az;
    s3 = 4*q1^2*q3 - 2*q1*ax + 4*q2^2*q3 - 2*q2*ay;
    s = [s0 s1 s2 s3];
end

function s = grad_9dof(q, a, m)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    ax=a(1); ay=a(2); az=a(3);
    mx=m(1); my=m(2); mz=m(3);

    hx = mx*q0^2 - 2*q0*my*q3 + 2*q0*mz*q2 + mx*q1^2 + 2*q1*my*q2 ...
       + 2*q1*mz*q3 - mx*q2^2 - mx*q3^2;
    hy = 2*q0*mx*q3 + my*q0^2 - 2*q0*mz*q1 + 2*q1*mx*q2 - my*q1^2 ...
       + my*q2^2 + 2*q2*mz*q3 - my*q3^2;
    b2x = sqrt(hx^2 + hy^2);
    b2z = -2*q0*mx*q2 + 2*q0*my*q1 + mz*q0^2 + 2*q1*mx*q3 - mz*q1^2 ...
        + 2*q2*my*q3 - mz*q2^2 + mz*q3^2;
    b4x = 2*b2x; b4z = 2*b2z;

    fA = 2*(q1*q3 - q0*q2) - ax;
    fB = 2*(q0*q1 + q2*q3) - ay;
    fC = b2x*(0.5 - q2^2 - q3^2) + b2z*(q1*q3 - q0*q2) - mx;
    fD = b2x*(q1*q2 - q0*q3)     + b2z*(q0*q1 + q2*q3) - my;
    fE = b2x*(q0*q2 + q1*q3)     + b2z*(0.5 - q1^2 - q2^2) - mz;

    s0 = -2*q2*fA + 2*q1*fB - b2z*q2*fC + (-b2x*q3 + b2z*q1)*fD + b2x*q2*fE;
    s1 =  2*q3*fA + 2*q0*fB - 4*q1*(1 - 2*q1^2 - 2*q2^2 - az) ...
       + b2z*q3*fC + (b2x*q2 + b2z*q0)*fD + (b2x*q3 - b4z*q1)*fE;
    s2 = -2*q0*fA + 2*q3*fB - 4*q2*(1 - 2*q1^2 - 2*q2^2 - az) ...
       + (-b4x*q2 - b2z*q0)*fC + (b2x*q1 + b2z*q3)*fD + (b2x*q0 - b4z*q2)*fE;
    s3 =  2*q1*fA + 2*q2*fB ...
       + (-b4x*q3 + b2z*q1)*fC + (-b2x*q0 + b2z*q2)*fD + b2x*q1*fE;
    s = [s0 s1 s2 s3];
end

function e = q2eul(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    r = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    sp = 2*(q0*q2 - q3*q1);
    sp = max(min(sp, 1), -1);
    p = asin(sp);
    y = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
    e = [r p y] * 57.2957795;
end
