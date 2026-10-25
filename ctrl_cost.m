function J = ctrl_cost(x, P)
%CTRL_COST  Closed-loop performance cost for a PID gain set (for PSO/GA tuning).
%   x = [Kp_pos_xy, Kd_pos_xy, Ki_pos_xy, Kp_att_rp, Kd_att_rp]
%   P = base parameter struct (gains below are overridden by x; z/yaw kept).
%
%   Runs a fast fixed-step closed loop of quad_plant + pos_att_ctrl_islfcn on a
%   step command, TRUE-state feedback (no observer -> isolates the controller).
%   J = ISE(position) + effort + overshoot + steady-state error.  Diverged,
%   tumbled, or erroring runs get a huge penalty so PSO avoids unstable gains.

    try
        % ---- install candidate gains (horizontal position + roll/pitch attitude) ----
        P.Kp_pos(1:2) = x(1);  P.Kd_pos(1:2) = x(2);  P.Ki_pos(1:2) = x(3);
        P.Kp_att(1:2) = x(4);  P.Kd_att(1:2) = x(5);
        P.ctrl.mode   = 'pid';

        % ---- step maneuver: from origin/hover to (N,E,D)=(2,2,-3), yaw 0 ----
        dt  = 0.004;  T = 6;  N = round(T/dt);
        sp  = [2; 2; -3];                      % position setpoint
        ref = [sp; 0;0;0; 0;0;0; 0];           % pos; vel_ff; acc_ff; psi  (10x1)

        xk    = zeros(12,1);                    % start level, at rest, at origin
        e_int = zeros(3,1);
        lim   = P.int_limit;
        Jpos  = 0;  Ju = 0;  Jdu = 0;  over = 0;
        u_prev = [P.m*P.g; 0; 0; 0];            % hover wrench (for control-rate term)

        for k = 1:N
            u  = pos_att_ctrl_islfcn([xk; ref; e_int], P);   % wrench [T; tau]
            xk = xk + dt * quad_plant(0, xk, u, P);          % Euler step (true state)

            epos  = sp - xk(1:3);
            e_int = max(min(e_int + dt*epos, lim), -lim);    % anti-windup integral

            Jpos = Jpos + dt*(epos.'*epos);                  % ISE position error
            Ju   = Ju   + dt*(u(2:4).'*u(2:4));              % control torque effort
            du   = u - u_prev;  u_prev = u;
            Jdu  = Jdu  + du.'*du;                           % control-RATE -> penalizes oscillation
            over = max(over, max(xk(1:3).*sign(sp) - abs(sp)));  % overshoot past setpoint

            if any(~isfinite(xk)) || any(abs(xk(7:8)) > pi/2)    % diverged / tumbled
                J = 1e6;  return;
            end
        end

        ess = norm(sp - xk(1:3));                            % residual steady error
        J   = Jpos + 1e-3*Ju + 2e-3*Jdu + 20*max(over,0) + 50*ess;   % weighted total
    catch
        J = 1e6;                                             % any failure -> reject candidate
    end
end
