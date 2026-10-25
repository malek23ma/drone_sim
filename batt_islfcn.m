function I_batt = batt_islfcn(omega)
%BATT_ISLFCN  Simulink bridge: 4 rotor speeds -> total pack current [A].
%   Driven by an "Interpreted MATLAB Function" block. Tap it off the rotor-speed
%   signal (Motor -> Forces) and feed its output into a charge Integrator to get
%   battery state-of-charge / endurance. Does NOT touch the control loop.
%
%   Input  omega = [w1; w2; w3; w4]  [rad/s]
%   Output I_batt = total pack current [A]
%
%   Calls the SAME motor_current() that test_16 validates, so the block and the
%   oracle can never disagree.
    P = evalin('base','P');
    I_batt = sum( motor_current(omega, P) );
end
