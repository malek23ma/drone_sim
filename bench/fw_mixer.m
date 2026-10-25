function m = fw_mixer(throttle, roll, pitch, yaw)
%FW_MIXER  Exact MATLAB port of components/mixer/mixer.c (quad X + AirMode).
%   Inputs: throttle 1000..2000, roll/pitch/yaw PID outputs (PWM units).
%   Output m = [M1 M2 M3 M4] = [FR FL RR RL]  (FIRMWARE numbering!)
%   FR CW, FL CCW, RR CCW, RL CW.
    roll  = fix(roll);  pitch = fix(pitch);  yaw = fix(yaw);  % int16_t casts
    % Signs = mixer.c after the 2026-07-07 polarity fix (chip x-fwd/z-up:
    % +pitch -> rear motors up, +yaw -> CW motors up).
    mix = [ throttle - roll - pitch + yaw;    % FR
            throttle + roll - pitch - yaw;    % FL
            throttle - roll + pitch - yaw;    % RR
            throttle + roll + pitch + yaw ];  % RL

    over = max(mix) - 2000;
    if over > 0, mix = mix - over; end
    under = 1000 - min(mix);
    if under > 0, mix = mix + under; end

    m = min(max(mix, 1000), 2000)';
end
