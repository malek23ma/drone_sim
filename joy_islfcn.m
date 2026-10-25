function cmd = joy_islfcn(psi)
%JOY_ISLFCN  Read a gamepad (PS4/DualShock) -> BODY-FRAME velocity commands.
%   Interpreted MATLAB Function block.
%   Input  psi = current drone yaw [rad]  (wire the state's yaw, element 9, here --
%               NOT the Clock).  Used to fly relative to the nose.
%   Output cmd = [vN; vE; vClimb; rYaw]  (world-frame velocity + yaw rate).
%
%   Pitch = forward along the NOSE; roll = the drone's right. Rotating the airframe
%   re-aims "forward", so it flies like a real pilot's-eye drone.
%
%   Robust to drop-outs: on a sleep/glitch it outputs NEUTRAL (hover) and silently
%   reconnects. Only reads hardware when P.pilot.on = 1.
%
%   PERF: the HID poll (axis(joy)) is a slow blocking USB read. At the 500 Hz
%   solver step it dominates wall-clock and breaks real-time. We poll only every
%   POLL-th step (~50 Hz), HOLD the body-frame stick values between polls, and
%   recompute the body->world yaw rotation EVERY step with the live psi (cheap
%   trig) so heading response stays crisp. P is also refreshed at the poll rate
%   instead of every step.
    persistent joy retry tick body Pc btnprev sprayst
    cmd = zeros(4,1);

    if isempty(tick), tick = 0; end
    tick = tick + 1;

    POLL    = 10;                                  % poll/refresh every 10 steps -> 50 Hz at Ts=2ms
    do_poll = isempty(body) || mod(tick,POLL) == 0;

    if do_poll, Pc = evalin('base','P'); end       % refresh params at 50 Hz, not 500 Hz
    P = Pc;
    if isempty(P) || ~isfield(P,'pilot') || ~P.pilot.on
        body = [];                                 % reset hold so it re-arms cleanly next enable
        return;
    end

    if do_poll
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
                body  = [fwd; right; clm; yawr];           % hold until next poll
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
    end

    if isempty(body), return; end                  % not connected yet -> hover

    % ---- body -> world: rotate forward/right by CURRENT yaw every step ----
    c = cos(psi);  s = sin(psi);
    vN = body(1)*c - body(2)*s;
    vE = body(1)*s + body(2)*c;
    cmd = [vN; vE; body(3); body(4)];
end

function v = dzf(x, dz)
    if abs(x) < dz, v = 0; else, v = (x - sign(x)*dz)/(1 - dz); end
end
