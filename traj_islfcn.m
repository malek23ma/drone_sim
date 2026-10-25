function ref = traj_islfcn(t)
%TRAJ_ISLFCN  Reference generator. Input = time t. Output ref(10):
%   ref = [ pos_ref(1:3) ; vel_ref(4:6) ; acc_ref(7:9) ; psi_ref(10) ]
%   vel/acc are feedforward terms for differential-flatness tracking.
%   modes: 'step' | 'circle' | 'field' (autonomous lawnmower coverage).
    P = evalin('base','P');
    if strcmp(P.traj.mode,'circle')
        w = 2*pi/P.traj.T;  R = P.traj.R;
        pos = [ R*cos(w*t);     R*sin(w*t);    -P.traj.h ];
        vel = [-R*w*sin(w*t);   R*w*cos(w*t);   0 ];
        acc = [-R*w^2*cos(w*t);  -R*w^2*sin(w*t);  0 ];
        psi = 0;
    elseif strcmp(P.traj.mode,'field')
        [pos, vel] = field_path(t, P);
        acc = [0;0;0];
        psi = 0;
    else                                   % 'step'
        s   = P.traj.set;
        pos = [s(1); s(2); s(3)];
        vel = [0;0;0];
        acc = [0;0;0];
        psi = s(4);
    end
    ref = [pos; vel; acc; psi];
end
