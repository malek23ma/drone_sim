function wrench = forces_islfcn(in)
%FORCES_ISLFCN  Rotor speeds -> actual wrench produced (the physics, forward).
%   Input  in = w = [w1; w2; w3; w4]  [rad/s]
%   Output wrench = [T; tau_x; tau_y; tau_z]
%
%   wrench = Mix * (w.^2).  This is the TRUTH; the mixer only inverts a model
%   of it. When motor lag/saturation perturb w, the wrench here is what the
%   plant actually feels (mixer's assumption breaks -> realistic imperfection).
    P = evalin('base','P');
    w  = in;
    s2 = w.^2;                                % each rotor's thrust-squared term
    % ---- motor fault: scale a rotor's actual output (thrust + its moments) ----
    if isfield(P,'fault') && isfield(P.fault,'motor') && P.fault.motor.on
        s2 = s2 .* P.fault.motor.scale(:);    % degraded motor -> mixer's model breaks
    end
    wrench = P.Mix * s2;
end
