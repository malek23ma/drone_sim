function fault_panel()
%FAULT_PANEL  Live clickable panel to inject faults/disturbances mid-flight.
%   Click instead of typing commands. Each control edits the base-workspace P;
%   the model's Interpreted blocks read P every step, so changes apply instantly
%   to the RUNNING simulation. No model rewiring -- pure base-workspace control.
%
%   Usage:  model_params; <start the sim>;  fault_panel
%           then toggle GPS / fail a motor / drag the wind slider while it flies.
%
%   Demo flow that clearly shows GPS matters (ORDER MATTERS):
%     1) click GPS: DROPPED  first (calm -> nothing moves yet, fix is frozen-correct).
%     2) THEN drag Wind up to ~8 N -> blind to the new wind, the drone blows away.
%     3) click GPS: OK       -> regains truth, flies back and holds.
%   (Raising wind BEFORE dropping GPS hides the effect: the PID integral trims the
%    steady wind out, and that trim persists once GPS is frozen.)

    f = figure('Name','Fault & disturbance panel','NumberTitle','off', ...
        'Color','w','Position',[120 120 290 330],'MenuBar','none','ToolBar','none');

    uicontrol(f,'Style','togglebutton','String','GPS: OK','FontSize',11, ...
        'Position',[25 270 240 42],'BackgroundColor',[0.60 0.88 0.60], ...
        'Callback',@gps_cb);

    uicontrol(f,'Style','togglebutton','String','Motor M1: healthy','FontSize',11, ...
        'Position',[25 218 240 42],'BackgroundColor',[0.60 0.88 0.60], ...
        'Callback',@motor_cb);

    uicontrol(f,'Style','text','String','Wind (N, blowing North):','FontSize',10, ...
        'BackgroundColor','w','HorizontalAlignment','left','Position',[25 178 180 20]);
    wtxt = uicontrol(f,'Style','text','String','0.0','FontSize',12,'FontWeight','bold', ...
        'BackgroundColor','w','Position',[205 178 60 20]);
    uicontrol(f,'Style','slider','Min',0,'Max',12,'Value',0, ...
        'Position',[25 150 240 24],'Callback',@(s,~)wind_cb(s,wtxt));

    uicontrol(f,'Style','pushbutton','String','Reset all (calm + healthy)', ...
        'FontSize',10,'Position',[25 100 240 36],'Callback',@(~,~)reset_cb(f));

    uicontrol(f,'Style','text','FontSize',9,'BackgroundColor','w', ...
        'HorizontalAlignment','left','Position',[25 20 245 70], ...
        'String',['Effects apply live to the running sim.' newline ...
                  'Order: DROP GPS first, THEN raise wind' newline ...
                  'to see it lose station-keeping.']);
end

function gps_cb(src,~)
    P = evalin('base','P');
    P.fault.gps.on = (get(src,'Value')==1);
    if P.fault.gps.on
        set(src,'String','GPS: DROPPED','BackgroundColor',[0.95 0.5 0.5]);
    else
        set(src,'String','GPS: OK','BackgroundColor',[0.60 0.88 0.60]);
    end
    assignin('base','P',P);
end

function motor_cb(src,~)
    P = evalin('base','P');
    P.fault.motor.on = (get(src,'Value')==1);
    if P.fault.motor.on
        P.fault.motor.scale = [0.6;1;1;1];
        set(src,'String','Motor M1: 60% FAIL','BackgroundColor',[0.95 0.5 0.5]);
    else
        P.fault.motor.scale = [1;1;1;1];
        set(src,'String','Motor M1: healthy','BackgroundColor',[0.60 0.88 0.60]);
    end
    assignin('base','P',P);
end

function wind_cb(src,wtxt)
    P = evalin('base','P');
    F = get(src,'Value');
    if F < 0.05
        P.wind.mode = 'none';  P.wind.F = [0;0;0];
    else
        P.wind.mode = 'const'; P.wind.t_on = 0;  P.wind.F = [F;0;0];
    end
    set(wtxt,'String',sprintf('%.1f',F));
    assignin('base','P',P);
end

function reset_cb(f)
    P = evalin('base','P');
    P.fault.gps.on      = 0;
    P.fault.motor.on    = 0;  P.fault.motor.scale = [1;1;1;1];
    P.wind.mode         = 'none';  P.wind.F = [0;0;0];
    assignin('base','P',P);
    % resync the control labels/states
    h = findobj(f,'Style','togglebutton');  set(h,'Value',0);
    set(findobj(f,'String','GPS: DROPPED'),'String','GPS: OK','BackgroundColor',[0.60 0.88 0.60]);
    set(findobj(f,'String','Motor M1: 60% FAIL'),'String','Motor M1: healthy','BackgroundColor',[0.60 0.88 0.60]);
    set(findobj(f,'Style','slider'),'Value',0);
    set(findobj(f,'Style','text','-and','String','0.0'),'String','0.0');
end
