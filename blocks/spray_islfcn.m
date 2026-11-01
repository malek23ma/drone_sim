function flow = spray_islfcn(t)
%SPRAY_ISLFCN  Spray pump flow command. Input = time t. Output = mass flow [kg/s].
%   modes: 'none' (0) | 'on' (rate during [t_on,t_off]) | 'field' (auto-spray:
%          rate only while on a lawnmower PASS, off on the East crossover hops).
%   The tank Integrator depletes by this flow; mass loss feeds the plant. When the
%   tank empties the Integrator clamps at 0, so a constant rate command is fine.
    P = evalin('base','P');
    s = P.spray;
    flow = 0;
    if strcmp(s.mode,'on')
        if t >= s.t_on && t <= s.t_off, flow = s.rate; end
    elseif strcmp(s.mode,'field')
        [~, ~, is_pass] = field_path(t, P);     % spray only on coverage passes
        if is_pass, flow = P.field.rate; end    % diffuser flow (sized for full field)
    end
    % manual joystick spray (button toggle) overrides -> full pump flow
    if isfield(s,'joy') && s.joy, flow = s.rate; end
end
