function xdot = plant_islfcn(in)
%PLANT_ISLFCN  Simulink bridge to the verified quad_plant dynamics.
%   Driven by an "Interpreted MATLAB Function" block.
%   Input  in = [ x(1:12) ; u(13:16) ]   (state stacked on top of wrench)
%   Output xdot(12)
%   Runs in the BASE workspace, so the parameter struct P is fetched from there.
    P = evalin('base','P');
    x = in(1:12);
    u = in(13:16);
    if numel(in) >= 19, Fwind = in(17:19); else, Fwind = [0;0;0]; end
    if numel(in) >= 20, m_now = in(20);    else, m_now = P.m;     end
    xdot = quad_plant(0, x, u, P, Fwind, m_now);   % same function the oracle uses
end
