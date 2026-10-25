function w_cmd = mixer_islfcn(in)
%MIXER_ISLFCN  Control allocation: desired wrench -> 4 rotor speed commands.
%   Input  in = wrench = [T; tau_x; tau_y; tau_z]
%   Output w_cmd = [w1; w2; w3; w4]  [rad/s]
%
%   s = Mix_inv * wrench   (rotor speeds squared)
%   w = sqrt(s), clamped to physical rotor limits.
    P = evalin('base','P');
    s = P.Mix_inv * in;                       % [w1^2..w4^2]
    s = max(s, 0);                            % cannot spin "negative^2"
    w_cmd = sqrt(s);
    w_cmd = min(max(w_cmd, P.omega_min), P.omega_max);   % saturate to limits
end
