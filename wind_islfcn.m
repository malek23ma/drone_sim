function F = wind_islfcn(t)
%WIND_ISLFCN  Disturbance force generator. Input = time t. Output F(3) world NED [N].
%   modes: 'none' (0), 'gust' (F during [t_on,t_off]), 'const' (F after t_on).
    P = evalin('base','P');
    w = P.wind;
    F = [0; 0; 0];
    if strcmp(w.mode,'gust')
        if t >= w.t_on && t <= w.t_off, F = w.F; end
    elseif strcmp(w.mode,'const')
        if t >= w.t_on, F = w.F; end
    end
end
