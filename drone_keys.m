function out = drone_keys(cmd, fig)
%DRONE_KEYS  Keyboard input for manual flight (no gamepad needed).
%   drone_keys('attach', fig)   listen to the keys while FIG has focus
%   k = drone_keys('read')      held keys as a struct of 0/1
%   drone_keys('clear')         release all keys (e.g. on a new mission)
%
%   W / S       climb / descend        A / D         yaw left / right
%   Up / Down   forward / back         Left / Right  left / right
%   Space       spray on / off
%
%   The key state lives in the root appdata, so the 3D window's callbacks
%   write it and joy_islfcn reads it without any extra wiring in the model.
    out = [];
    switch cmd
        case 'attach'
            set(fig, 'WindowKeyPressFcn',   @(~,e) on_key(e.Key, 1), ...
                     'WindowKeyReleaseFcn', @(~,e) on_key(e.Key, 0));
        case 'read'
            out = getappdata(groot, 'drone_keys');
            if isempty(out), out = blank(); end
        case 'clear'
            setappdata(groot, 'drone_keys', blank());
    end
end

function on_key(key, down)
    k = drone_keys('read');
    switch key
        case 'uparrow',          k.up    = down;
        case 'downarrow',        k.down  = down;
        case 'leftarrow',        k.left  = down;
        case 'rightarrow',       k.right = down;
        case {'w','a','s','d'},  k.(key) = down;
        case 'space'
            if down && ~k.space                    % one toggle per press, ignore key repeat
                P = evalin('base','P');  P.spray.joy = ~P.spray.joy;
                assignin('base','P',P);
                sync_panel(P.spray.joy);
            end
            k.space = down;
        otherwise
            return;
    end
    setappdata(groot, 'drone_keys', k);
end

function sync_panel(on)
%SYNC_PANEL  Keep the control panel's spray button in step with the Space key.
    h = findobj(groot, 'Type','uicontrol', 'Tag','spray');
    if isempty(h), return; end
    if on, set(h,'Value',1,'String','Spray: ON', 'BackgroundColor',[0.4 0.7 1.0]);
    else,  set(h,'Value',0,'String','Spray: OFF','BackgroundColor',[0.85 0.85 0.85]); end
end

function k = blank()
    k = struct('up',0,'down',0,'left',0,'right',0,'w',0,'a',0,'s',0,'d',0,'space',0);
end
