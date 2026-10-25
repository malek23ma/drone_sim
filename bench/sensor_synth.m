function [acc_g, gyr_dps, mag_uT] = sensor_synth(x, vel_dot, P)
%SENSOR_SYNTH  Synthesize IMU/mag readings from plant state (for SIL / HIL).
%
%   Plant frames: world NED, body FRD (model_params.m).
%   The IMU chip frame is taken as x-forward, z-UP (the convention the
%   firmware Madgwick expects: accel z = +1 g when level). FRD -> chip is a
%   180 deg rotation about x:  chip = diag(1,-1,-1) * FRD.
%
%   x       plant state (12x1), vel_dot world accel [m/s^2]
%   acc_g   specific force [g], gyr_dps [deg/s], mag_uT [uT] (chip frame)
    S = diag([1 -1 -1]);
    eul   = x(7:9);  omega = x(10:12);
    phi = eul(1); th = eul(2); psi = eul(3);
    cph=cos(phi); sph=sin(phi); cth=cos(th); sth=sin(th);
    cps=cos(psi); sps=sin(psi);
    R = [ cps*cth, cps*sth*sph - sps*cph, cps*sth*cph + sps*sph;
          sps*cth, sps*sth*sph + cps*cph, sps*sth*cph - cps*sph;
          -sth,    cth*sph,               cth*cph ];   % body -> world

    % accelerometer = specific force = (a - g)/|g| resolved in body
    f_frd = R' * (vel_dot - [0; 0; P.g]) / P.g;
    acc_g = (S * f_frd)';

    gyr_dps = (S * omega)' * 180/pi;

    % Geomagnetic field ~47 uT, inclination ~59 deg (down +), decl ignored.
    % Bursa model values; Istanbul is ~48.1 uT (WMM2025), close enough here
    B_ned = [47*cosd(59); 0; 47*sind(59)];
    mag_uT = (S * (R' * B_ned))';
end
