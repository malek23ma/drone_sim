function y = sensors_islfcn(in)
%SENSORS_ISLFCN  Corrupt the true state into noisy measurements.
%   Input  in = state(1:12)
%   Output y  = [ pos_meas(1:3) ; eul_meas(4:6) ; omega_meas(7:9) ]   (9x1)
%
%   GPS  : position + white noise   (FROZEN during a fault.gps dropout)
%   AHRS : attitude + white noise
%   Gyro : body rates + CONSTANT bias + white noise  (bias -> integrated drift)
    persistent gps_hold
    P = evalin('base','P');
    pos   = in(1:3);
    eul   = in(7:9);
    omega = in(10:12);

    pos_m   = pos   + P.sen.gps_std  * randn(3,1);
    eul_m   = eul   + P.sen.att_std  * randn(3,1);
    omega_m = omega + P.sen.gyro_bias + P.sen.gyro_std * randn(3,1);

    % ---- GPS dropout: serve the last good fix while the receiver has no lock ----
    if isfield(P,'fault') && P.fault.gps.on
        if ~isempty(gps_hold), pos_m = gps_hold; end   % outage -> stale position
    else
        gps_hold = pos_m;                              % healthy -> remember last fix
    end

    y = [pos_m; eul_m; omega_m];
end
