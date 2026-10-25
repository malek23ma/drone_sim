function s = fc_parse_line(line)
%FC_PARSE_LINE  Parse one "D,..." CSV line from the firmware debug stream.
%   Returns [] if the line is not a valid D-record.
%   Fields (27 total incl. tag; alt_m added 2026-07, old 26-field
%   logs still parse with alt = NaN):
%   D,t_us,ax,ay,az,gx,gy,gz,mx,my,mz,q0,q1,q2,q3,roll,pitch,yaw,
%     fusion_dt,loop_dt,m1,m2,m3,m4,armed,heartbeat,alt_m
    s = [];
    line = char(line);
    if numel(line) < 2 || line(1) ~= 'D' || line(2) ~= ','
        return
    end
    v = sscanf(line(3:end), '%f,');
    if numel(v) ~= 25 && numel(v) ~= 26
        return
    end
    if numel(v) == 26, alt = v(26); else, alt = NaN; end
    s = struct( ...
        't',      v(1)*1e-6, ...          % seconds
        'acc',    v(2:4).', ...           % [g]
        'gyro',   v(5:7).', ...           % [dps]
        'mag',    v(8:10).', ...          % [uT]
        'quat',   v(11:14).', ...         % [q0 q1 q2 q3]
        'euler',  v(15:17).', ...         % [roll pitch yaw] deg (onboard)
        'fdt',    v(18), ...              % fusion loop dt [s]
        'cdt',    v(19), ...              % flight loop dt [s]
        'pwm',    v(20:23).', ...         % M1..M4
        'armed',  v(24), ...
        'hb',     v(25), ...
        'alt',    alt);                   % baro altitude [m] (NaN on old fw)
end
