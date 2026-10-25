function hil_fly(port, duration_s)
%HIL_FLY  Piloted hardware-in-the-loop: YOU fly the simulated quad through
%   the REAL flight controller, joystick or keyboard.
%
%   hil_fly("COM4")          fly for 300 s
%   hil_fly("COM4", 600)     fly for 600 s
%
%   REQUIRES firmware flashed with HIL_MODE=1.
%
%   Loop: stick -> R line -> real FC (fusion+PID+mixer) -> motor PWMs ->
%   6-DOF plant -> synthetic sensors -> S line -> FC. Live 3D view = plant
%   truth. So you are flying the actual firmware, physics by quad_plant.m.
%
%   CONTROLS
%   Joystick (if Simulink 3D Animation's vrjoystick is available):
%     axis 1 roll, axis 2 pitch, axis 3 throttle, axis 4 yaw,
%     button 1 = arm/disarm toggle. Edit AXIS_SIGN below if reversed.
%   Keyboard (fallback; click the figure so it has focus!):
%     arrows   = roll left/right, pitch fwd(nose down)/back
%     W / S    = throttle up / down      A / D = yaw left / right
%     SPACE    = arm/disarm toggle       ESC   = quit
%
%   Arming rule (real firmware logic!): throttle must be LOW to arm.
%   Hover throttle for this model ~ 1533.
    if nargin < 2, duration_s = 300; end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    addpath(root);
    run(fullfile(root, 'model_params.m'));   % P

    % --- joystick config (edit signs if your stick is reversed) ---
    AXIS_SIGN = [ +1, +1, -1, +1 ];   % roll, pitch, throttle, yaw
    JOY_ID = 1;

    sp = fc_open(port);
    cleanup = onCleanup(@() delete(sp)); %#ok<NASGU>

    % --- input source: sim3d.io.Joystick (R2024a+) > vrjoystick > keyboard ---
    use_joy = false; joy = []; joy_api = '';
    try
        joy = sim3d.io.Joystick(JOY_ID);
        read(joy);  % test
        use_joy = true; joy_api = 'sim3d';
    catch
        try
            joy = vrjoystick(JOY_ID); %#ok<*VRJOY>
            axis(joy);  % test read (NB: axis(), not axes() - that's graphics)
            use_joy = true; joy_api = 'vr';
        catch
        end
    end
    if use_joy
        fprintf('Joystick detected (%s API). Button 1 = arm toggle.\n', joy_api);
    else
        fprintf('No joystick - keyboard mode. CLICK THE FIGURE for focus.\n');
    end

    % --- keyboard state (shared with callbacks) ---
    K = struct('roll',0,'pitch',0,'yawk',0,'thr',1000,'arm',false,'quit',false);
    setappdata(0,'hil_keys',K);

    % --- figure: live 3D ---
    [V, F] = box_geom(1.0, 0.6, 0.15);
    fig = figure('Name','HIL FLY - you are the pilot','NumberTitle','off', ...
                 'Color','w','KeyPressFcn',@key_dn,'KeyReleaseFcn',@key_up);
    ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on');
    h  = patch(ax,'Vertices',V,'Faces',F,'FaceColor',[0.2 0.5 0.9], ...
               'FaceAlpha',0.85,'EdgeColor','k');
    nose = quiver3(ax,0,0,0,1.4,0,0,'r','LineWidth',2);
    axis(ax,'equal'); axis(ax,[-1.5 1.5 -1.5 1.5 -1.5 1.5]);
    view(ax,135,20); xlabel(ax,'x (fwd)');
    ttl = title(ax,'DISARMED - lower throttle, then arm');

    dt_ctl = 1/50; n_sub = 5; dt_sub = dt_ctl/n_sub;
    N = round(duration_s/dt_ctl);
    x = zeros(12,1); pwm = [1000 1000 1000 1000];
    u = pwm_to_forces(pwm, P); vd = zeros(3,1);
    arm_ch = 1000; prev_b1 = false;
    LOG = nan(N, 9);   % t, eul, thr, pwm

    for k = 1:N
        t = k*dt_ctl;

        % ---- pilot input -> RC channels ----
        K = getappdata(0,'hil_keys');
        if K.quit || ~ishghandle(fig), break; end
        if use_joy
            try
                if strcmp(joy_api, 'sim3d')
                    [a, b] = read(joy);
                else
                    a = axis(joy); b = button(joy);
                end
            catch, a = [0 0 0 0]; b = false;
            end
            a = a(:)'; a(end+1:4) = 0;
            rc_roll  = 1500 + round(500*AXIS_SIGN(1)*a(1));
            rc_pitch = 1500 + round(500*AXIS_SIGN(2)*a(2));
            rc_thr   = 1500 + round(500*AXIS_SIGN(3)*a(3));
            rc_yaw   = 1500 + round(500*AXIS_SIGN(4)*a(4));
            if b(1) && ~prev_b1, K.arm = ~K.arm; setappdata(0,'hil_keys',K); end
            prev_b1 = b(1);
        else
            rc_roll  = 1500 + K.roll;
            rc_pitch = 1500 + K.pitch;
            rc_yaw   = 1500 + K.yawk;
            rc_thr   = K.thr;
        end
        arm_ch = 1000 + 700*K.arm;

        % ---- serial exchange (same as hil_run) ----
        [acc, gyr, mag] = sensor_synth(x, vd, P);
        writeline(sp, sprintf('S,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f', acc, gyr, mag));
        writeline(sp, sprintf('R,%d,%d,%d,%d,%d', rc_roll, rc_pitch, rc_thr, rc_yaw, arm_ch));

        s = [];
        while sp.NumBytesAvailable > 60
            cand = fc_parse_line(readline(sp));
            if ~isempty(cand), s = cand; end
        end
        if isempty(s), s = fc_parse_line(readline(sp)); end
        if ~isempty(s)
            pwm = s.pwm(:)';
            u = pwm_to_forces(pwm, P);
        end

        % ---- plant ----
        for j = 1:n_sub
            xd = quad_plant(0, x, u, P);
            x = x + dt_sub*xd;
        end
        vd = xd(4:6);
        % keep it near the origin (position is uncontrolled - rate/angle mode)
        x(1:2) = 0;  x(4:5) = 0;   % lock horizontal drift, keep alt free
        if -x(3) < 0, x(3) = 0; x(6) = min(x(6),0); end   % ground

        % ---- draw at ~25 Hz ----
        if mod(k,2) == 0 && ishghandle(fig)
            phi = x(7); th = x(8); psi = x(9);
            Rv = eul_chip(phi, -th, -psi);
            set(h, 'Vertices', (Rv*V')');
            nv = Rv*[1.4;0;0];
            set(nose,'UData',nv(1),'VData',nv(2),'WData',nv(3));
            armed_fw = 0; if ~isempty(s), armed_fw = s.armed; end
            if armed_fw
                st = 'ARMED';
            elseif K.arm
                st = 'ARM BLOCKED - lower throttle to 1000 (S key)!';
            else
                st = 'DISARMED - press SPACE';
            end
            set(ttl,'String',sprintf( ...
                '%s | thr %d | alt %.1f m | R %5.1f P %5.1f Y %5.1f', ...
                st, rc_thr, -x(3), rad2deg(phi), rad2deg(th), rad2deg(psi)));
            drawnow limitrate;
        end

        LOG(k,:) = [t, rad2deg(x(7:9))', rc_thr, pwm];
    end

    % ---- session summary ----
    L = LOG(~isnan(LOG(:,1)),:);
    if ~isempty(L)
        figure('Name','HIL FLY session','Color','w');
        subplot(2,1,1); plot(L(:,1), L(:,2:4)); grid on;
            legend('\phi','\theta','\psi'); ylabel('deg');
        subplot(2,1,2); plot(L(:,1), L(:,6:9)); grid on;
            legend('M1 FR','M2 FL','M3 RR','M4 RL'); ylabel('PWM'); xlabel('t [s]');
    end
end

% ---- keyboard callbacks (rate-command style) ----
function key_dn(~, e)
    K = getappdata(0,'hil_keys');
    switch e.Key
        case 'leftarrow',  K.roll  = -150;
        case 'rightarrow', K.roll  = +150;
        case 'uparrow',    K.pitch = +150;   % nose down (chip +pitch)
        case 'downarrow',  K.pitch = -150;
        case 'a',          K.yawk  = +150;   % nose left (chip +yaw)
        case 'd',          K.yawk  = -150;
        case 'w',          K.thr   = min(K.thr+15, 2000);
        case 's',          K.thr   = max(K.thr-15, 1000);
        case 'space',      K.arm   = ~K.arm;
        case 'escape',     K.quit  = true;
    end
    setappdata(0,'hil_keys',K);
end

function key_up(~, e)
    K = getappdata(0,'hil_keys');
    switch e.Key
        case {'leftarrow','rightarrow'}, K.roll  = 0;
        case {'uparrow','downarrow'},    K.pitch = 0;
        case {'a','d'},                  K.yawk  = 0;
    end
    setappdata(0,'hil_keys',K);
end

function R = eul_chip(r, p, y)
    cr=cos(r); sr=sin(r); cp=cos(p); sp=sin(p); cy=cos(y); sy=sin(y);
    R = [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
         sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
         -sp,   cp*sr,          cp*cr];
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end

function [V,F] = box_geom(lx, ly, lz)
    [X,Y,Z] = ndgrid([-lx lx]/2, [-ly ly]/2, [-lz lz]/2);
    V = [X(:) Y(:) Z(:)];
    F = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
end
