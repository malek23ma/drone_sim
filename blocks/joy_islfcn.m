function cmd = joy_islfcn(psi)
%JOY_ISLFCN  Read the keyboard and/or a gamepad -> BODY-FRAME velocity commands.
%   Interpreted MATLAB Function block.
%   Input  psi = current drone yaw [rad]  (wire the state's yaw, element 9, here --
%               NOT the Clock).  Used to fly relative to the nose.
%   Output cmd = [vN; vE; vClimb; rYaw]  (world-frame velocity + yaw rate).
%
%   Pitch = forward along the NOSE; roll = the drone's right. Rotating the airframe
%   re-aims "forward", so it flies like a real pilot's-eye drone.
%
%   Keyboard (keys held in the 3D window, see drone_keys): arrows move, W/S climb,
%   A/D yaw, Space sprays. Keys are on/off, so they ramp in over ~0.15 s. A gamepad,
%   when plugged in, adds on top; the sum is clipped to the pilot limits.
%
%   Robust to drop-outs: on a sleep/glitch the gamepad reads NEUTRAL and silently
%   reconnects. Only reads input when P.pilot.on = 1.
%
%   PERF: the HID poll (axis(joy)) is a slow blocking USB read. At the 500 Hz
%   solver step it dominates wall-clock and breaks real-time. We poll only every
%   POLL-th step (~50 Hz), HOLD the body-frame stick values between polls, and
%   recompute the body->world yaw rotation EVERY step with the live psi (cheap
%   trig) so heading response stays crisp. P is also refreshed at the poll rate
%   instead of every step.
    persistent joy retry tick body Pc btnprev sprayst kb
    cmd = zeros(4,1);

    if isempty(tick), tick = 0; end
    tick = tick + 1;

    POLL    = 10;                                  % poll/refresh every 10 steps -> 50 Hz at Ts=2ms
    do_poll = isempty(body) || mod(tick,POLL) == 0;

    if do_poll, Pc = evalin('base','P'); end       % refresh params at 50 Hz, not 500 Hz
    P = Pc;
    if isempty(P) || ~isfield(P,'pilot') || ~P.pilot.on
        body = [];  kb = [];                       % reset hold so it re-arms cleanly next enable
        return;
    end

    if do_poll
        % ---- keyboard: held keys -> body axes, ramped (keys are on/off) ----
        k   = drone_keys('read');
        tgt = [(k.up - k.down)    * P.pilot.vmax_xy;   % Up/Down    -> forward/back
               (k.right - k.left) * P.pilot.vmax_xy;   % Left/Right -> left/right
               (k.w - k.s)        * P.pilot.vmax_z;    % W/S        -> climb/descend
               (k.d - k.a)        * P.pilot.rmax];     % A/D        -> yaw
        if isempty(kb), kb = zeros(4,1); end
        kb  = kb + 0.12*(tgt - kb);                    % ~0.15 s time constant at 50 Hz
        pad = zeros(4,1);

        if isempty(joy)
            if isempty(retry), retry = 250; end
            retry = retry + 1;
            if retry >= 250
                retry = 0;
                try
                    ws = warning('off','all');  joy = vrjoystick(1);  warning(ws);
                catch
                    joy = [];
                end
            end
        end
        if ~isempty(joy)
            try
                a = axis(joy);
                % ---- Mode 2 sticks -> body axes (deadzone + rescale) ----
                dz = 0.08;
                fwd   = -dzf(a(6),dz) * P.pilot.vmax_xy;   % RV up    -> forward (pitch)
                right =  dzf(a(3),dz) * P.pilot.vmax_xy;   % RH right -> right   (roll)
                clm   = -dzf(a(2),dz) * P.pilot.vmax_z;    % LV up    -> climb   (thrust)
                yawr  =  dzf(a(1),dz) * P.pilot.rmax;      % LH right -> yaw
                pad   = [fwd; right; clm; yawr];
            catch
                joy = [];                                  % stick drop-out -> reconnect next poll
            end
        end
        % ---- spray button read ISOLATED: a button error must NOT kill the sticks ----
        if ~isempty(joy)
            try
                bb = button(joy);
                sb = P.pilot.spray_btn;
                sprayst = logical(P.spray.joy);            % stay in sync with the panel/base
                if isempty(btnprev), btnprev = false; end
                if numel(bb) >= sb
                    pressed = bb(sb) > 0.5;
                    if pressed && ~btnprev                 % new press -> toggle spray
                        sprayst = ~sprayst;
                        Pb = evalin('base','P');  Pb.spray.joy = sprayst;
                        assignin('base','P',Pb);
                    end
                    btnprev = pressed;
                end
            catch
                % ignore button-read errors; sticks keep working
            end
        end

        lim  = [P.pilot.vmax_xy; P.pilot.vmax_xy; P.pilot.vmax_z; P.pilot.rmax];
        body = max(-lim, min(lim, kb + pad));          % hold until next poll
    end

    if isempty(body), return; end                  % no input yet -> hover

    % ---- body -> world: rotate forward/right by CURRENT yaw every step ----
    c = cos(psi);  s = sin(psi);
    vN = body(1)*c - body(2)*s;
    vE = body(1)*s + body(2)*c;
    cmd = [vN; vE; body(3); body(4)];
end

function v = dzf(x, dz)
    if abs(x) < dz, v = 0; else, v = (x - sign(x)*dz)/(1 - dz); end
end
