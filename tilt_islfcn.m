function out = tilt_islfcn(in)
%TILT_ISLFCN  Desired-acceleration -> thrust + desired tilt (nonlinear transform).
%   Input  in  = [ acc_des(1:3) ; psi_ref(4) ]
%   Output out = [ T ; phi_des ; theta_des ]
%   Small-angle inversion of the plant's R, consistent with quad_plant.
    P = evalin('base','P');
    acc_des = in(1:3);  psi_ref = in(4);
    cpsi = cos(psi_ref);  spsi = sin(psi_ref);

    T = P.m*(P.g - acc_des(3));
    theta_des = -(1/P.g)*( cpsi*acc_des(1) + spsi*acc_des(2) );
    phi_des   =  (1/P.g)*( cpsi*acc_des(2) - spsi*acc_des(1) );
    phi_des   = max(min(phi_des,   P.tilt_max), -P.tilt_max);
    theta_des = max(min(theta_des, P.tilt_max), -P.tilt_max);

    out = [T; phi_des; theta_des];
end
