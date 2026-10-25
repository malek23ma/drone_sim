function st = fw_pid_init()
%FW_PID_INIT  Reset firmware-PID state (pid_init in pid.c).
    st = struct('integral', 0, 'prev_meas', 0, 'd_filtered', 0);
end
