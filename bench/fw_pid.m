function [out, st] = fw_pid(cfg, st, setpoint, measured, dt)
%FW_PID  Exact MATLAB port of components/pid/pid.c (firmware PID).
%   cfg: struct kp ki kd i_max out_max tau
%   st : struct integral prev_meas d_filtered   (fw_pid_init to reset)
    err = setpoint - measured;
    P = err * cfg.kp;

    if dt > 0
        st.integral = st.integral + err * dt * cfg.ki;
        st.integral = min(max(st.integral, -cfg.i_max), cfg.i_max);
    end
    I = st.integral;

    D = 0;
    if dt > 0
        raw_d = -(measured - st.prev_meas) / dt;
        alpha = 1;
        if cfg.tau > 0
            alpha = dt / (dt + cfg.tau);
        end
        st.d_filtered = st.d_filtered + alpha * (raw_d - st.d_filtered);
        D = st.d_filtered * cfg.kd;
    end
    st.prev_meas = measured;

    out = min(max(P + I + D, -cfg.out_max), cfg.out_max);
end
