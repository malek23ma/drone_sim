function drone_panel()
%DRONE_PANEL  Single master control panel for the interactive demo. One window,
%   all clicks: pick a mission, start/stop, spray, inject faults, change wind,
%   switch controller (PID<->LQR), tune all PID gains live, and auto-tune (PSO).
%   Each control edits the base-workspace P (read live by the model's Interpreted
%   blocks) and/or drives the solver. No command-window typing after launch.
%
%   Usage:  open drone_plant.slx, then  >> drone_panel

    mdl = 'drone_plant';
    if ~bdIsLoaded(mdl), try, load_system(mdl); catch, end, end
    if ~evalin('base','exist(''P'',''var'')'), evalin('base','model_params'); end

    f = figure('Name','Drone demo control','NumberTitle','off','Color','w', ...
        'Position',[60 30 600 770],'MenuBar','none','ToolBar','none', ...
        'DefaultUicontrolForegroundColor',[0 0 0],'DefaultUicontrolFontSize',10);
    P = evalin('base','P');
    K = [0 0 0];                                  % black text

    % =================== LEFT COLUMN: mission / sim / faults ===================
    hdr(f,'MISSION',[20 730 260 16]);
    uicontrol(f,'Style','pushbutton','String','Manual fly','ForegroundColor',K, ...
        'Position',[20 688 80 36],'Callback',@(~,~)mission(f,'manual'));
    uicontrol(f,'Style','pushbutton','String','Field mission','FontSize',9,'ForegroundColor',K, ...
        'Position',[108 688 80 36],'Callback',@(~,~)mission(f,'field'));
    uicontrol(f,'Style','pushbutton','String','Hover','ForegroundColor',K, ...
        'Position',[196 688 80 36],'Callback',@(~,~)mission(f,'hover'));

    uicontrol(f,'Style','pushbutton','String','> START','FontSize',11,'FontWeight','bold', ...
        'ForegroundColor',K,'BackgroundColor',[0.7 0.9 0.7],'Position',[20 636 125 38], ...
        'Callback',@(~,~)simcmd('start'));
    uicontrol(f,'Style','pushbutton','String','# STOP','FontSize',11,'FontWeight','bold', ...
        'ForegroundColor',K,'BackgroundColor',[0.95 0.8 0.8],'Position',[152 636 128 38], ...
        'Callback',@(~,~)simcmd('stop'));

    hdr(f,'SPRAY (manual)',[20 600 260 16]);
    uicontrol(f,'Style','togglebutton','String','Spray: OFF','FontSize',11,'Tag','spray', ...
        'ForegroundColor',K,'BackgroundColor',[0.85 0.85 0.85],'Position',[20 560 260 36], ...
        'Callback',@spray_cb);

    hdr(f,'DISTURBANCES / FAULTS',[20 522 260 16]);
    uicontrol(f,'Style','togglebutton','String','GPS: OK','FontSize',11,'Tag','gps', ...
        'ForegroundColor',K,'BackgroundColor',[0.60 0.88 0.60],'Position',[20 480 260 36], ...
        'Callback',@gps_cb);
    uicontrol(f,'Style','togglebutton','String','Motor M1: healthy','FontSize',11,'Tag','motor', ...
        'ForegroundColor',K,'BackgroundColor',[0.60 0.88 0.60],'Position',[20 436 260 36], ...
        'Callback',@motor_cb);
    uicontrol(f,'Style','text','String','Wind (N, North):','ForegroundColor',K,'BackgroundColor','w', ...
        'HorizontalAlignment','left','Position',[20 402 150 16]);
    uicontrol(f,'Style','text','String','0.0','FontSize',12,'FontWeight','bold','Tag','wtxt', ...
        'ForegroundColor',K,'BackgroundColor','w','Position',[200 402 75 16]);
    uicontrol(f,'Style','slider','Min',0,'Max',12,'Value',0,'Tag','wind', ...
        'Position',[20 372 260 24],'Callback',@(s,~)wind_cb(s,f));
    uicontrol(f,'Style','pushbutton','String','Reset faults (calm + healthy)','ForegroundColor',K, ...
        'Position',[20 322 260 32],'Callback',@(~,~)reset_faults(f));

    uicontrol(f,'Style','text','FontSize',9,'BackgroundColor','w','ForegroundColor',[0.25 0.25 0.25], ...
        'HorizontalAlignment','left','Position',[20 30 285 270],'String', ...
        ['Demo flow:' newline newline ...
         ' 1) pick MISSION  ->  START' newline newline ...
         ' 2) Manual: gamepad + spray button (2)' newline newline ...
         ' 3) GPS drop, then raise Wind  ->  drifts' newline newline ...
         ' 4) Motor M1  ->  partial failure' newline newline ...
         ' 5) Controller PID <-> LQR  (right)' newline newline ...
         ' 6) drop Att D  ->  rings;  raise  ->  calm' newline newline ...
         ' 7) fly into the ground  ->  CRASH / LAND']);

    % =================== RIGHT COLUMN: controller + PID gains ==================
    islqr = isfield(P,'ctrl') && strcmp(P.ctrl.mode,'lqr');
    uicontrol(f,'Style','togglebutton','Tag','ctrl','FontSize',11,'FontWeight','bold','ForegroundColor',K, ...
        'Position',[315 728 265 36],'Value',islqr, ...
        'String',ternstr(islqr,'Controller: LQR','Controller: PID'), ...
        'BackgroundColor',ternvec(islqr,[0.75 0.85 1.0],[0.80 0.90 0.75]),'Callback',@ctrl_cb);

    hdr(f,'OUTER LOOP  (position)',[315 696 265 16]);
    mkslider(f,'Kp', 'Pos P (stiffness)', 0, 12, P.Kp_pos(1), 644);
    mkslider(f,'Kd', 'Pos D (damping)',   0, 10, P.Kd_pos(1), 584);
    mkslider(f,'Ki', 'Pos I (offset)',    0,  6, P.Ki_pos(1), 524);

    hdr(f,'INNER LOOP  (attitude roll/pitch)',[315 492 265 16]);
    mkslider(f,'AKp','Att P (stiffness)', 0, 200, P.Kp_att(1), 440);
    mkslider(f,'AKd','Att D (damping)',   0,  40, P.Kd_att(1), 380);

    hdr(f,'YAW',[315 348 265 16]);
    mkslider(f,'YKp','Yaw P', 0, 60, P.Kp_att(3), 296);
    mkslider(f,'YKd','Yaw D', 0, 20, P.Kd_att(3), 236);

    uicontrol(f,'Style','pushbutton','Tag','autotune','String','Tune: PSO', ...
        'FontWeight','bold','ForegroundColor',K,'BackgroundColor',[0.80 0.74 1.0], ...
        'Position',[315 196 130 32],'Callback',@(~,~)tune_cb(f,'pso'));
    uicontrol(f,'Style','pushbutton','Tag','gatune','String','Tune: GA', ...
        'FontWeight','bold','ForegroundColor',K,'BackgroundColor',[0.74 0.86 1.0], ...
        'Position',[451 196 129 32],'Callback',@(~,~)tune_cb(f,'ga'));
    uicontrol(f,'Style','pushbutton','String','Compare (plot)','FontWeight','bold', ...
        'ForegroundColor',K,'BackgroundColor',[0.95 0.92 0.75], ...
        'Position',[315 160 130 32],'Callback',@(~,~)compare_cb());
    uicontrol(f,'Style','pushbutton','String','Reset gains','ForegroundColor',K, ...
        'Position',[451 160 129 32],'Callback',@(~,~)reset_gains(f));
    uicontrol(f,'Style','text','FontSize',9,'BackgroundColor','w','ForegroundColor',[0.25 0.25 0.25], ...
        'HorizontalAlignment','left','Position',[315 30 265 122],'String', ...
        ['Tune: PSO or GA search the gains for the' newline newline ...
         'lowest step cost (sim freezes ~8-15 s).' newline newline ...
         'Compare: plots nominal vs tuned step response' newline newline ...
         '-> shows the gain quality the slow cruise hides.']);
end

% =============================== helpers ====================================
function hdr(f, s, pos)
    uicontrol(f,'Style','text','String',s,'FontSize',9,'FontWeight','bold', ...
        'ForegroundColor',[0.15 0.15 0.45],'BackgroundColor','w', ...
        'HorizontalAlignment','left','Position',pos);
end

function mkslider(f, tag, label, lo, hi, val, y)
    uicontrol(f,'Style','text','String',label,'BackgroundColor','w','ForegroundColor',[0 0 0], ...
        'HorizontalAlignment','left','Position',[315 y+30 195 18]);
    uicontrol(f,'Style','text','Tag',[tag 'txt'],'String',sprintf('%.2f',val),'FontSize',11, ...
        'FontWeight','bold','BackgroundColor','w','ForegroundColor',[0 0 0],'Position',[505 y+30 75 18]);
    uicontrol(f,'Style','slider','Tag',tag,'Min',lo,'Max',hi,'Value',min(max(val,lo),hi), ...
        'Position',[315 y+6 265 20],'Callback',@(s,~)gain_cb(s,tag,f));
end

% --------------------------- mission / sim ----------------------------------
function mission(f, mode)
  try
    mdl = 'drone_plant';
    if ~bdIsLoaded(mdl), load_system(mdl); end
    evalin('base','clear joy_islfcn viz_islfcn sensors_islfcn');
    if ~evalin('base','exist(''P'',''var'')'), evalin('base','model_params'); end
    P = evalin('base','P');
    P.viz.on = 1;  P.spray.joy = 0;
    P.fault.gps.on = 0;  P.fault.motor.on = 0;  P.fault.motor.scale = [1;1;1;1];
    P.wind.mode = 'none';  P.wind.F = [0;0;0];
    switch mode
        case 'manual'
            P.pilot.on = 1;  P.pilot.src = 1;  P.traj.mode = 'step';  P.spray.mode = 'none';
            x0 = zeros(12,1);  x0(3) = -2;
        case 'field'
            P.pilot.on = 0;  P.traj.mode = 'field';  P.spray.mode = 'field';
            x0 = zeros(12,1);  x0(3) = -P.field.alt;
        case 'hover'
            P.pilot.on = 0;  P.traj.mode = 'step';  P.traj.set = [0;0;-3;0];  P.spray.mode = 'none';
            x0 = zeros(12,1);  x0(3) = -3;
    end
    assignin('base','P',P);  assignin('base','x0',x0);
    reset_controls(f);
    set_param(mdl,'StopTime','inf');
    restart_sim(mdl);                 % stop any running/paused sim, then start clean
  catch e
    errordlg(sprintf('Mission "%s" failed:\n%s', mode, e.message), 'drone_panel');
  end
end

function simcmd(cmd)
    try
        if ~bdIsLoaded('drone_plant'), load_system('drone_plant'); end
        if strcmp(cmd,'start'), restart_sim('drone_plant');     % clean (re)start
        else,                   set_param('drone_plant','SimulationCommand',cmd); end
    catch e, errordlg(e.message,'sim');
    end
end

function restart_sim(mdl)
%RESTART_SIM  Force a clean fresh run. 'start' only launches a STOPPED model
%   (from running it is ignored, from paused it just resumes) -- so if a prior
%   mission/tune left the sim non-stopped, the new mission would silently do
%   nothing. Stop first, wait for the async stop to land, then start.
    if ~strcmp(get_param(mdl,'SimulationStatus'),'stopped')
        set_param(mdl,'SimulationCommand','stop');
        for k = 1:200                                   % wait up to ~4 s for 'stopped'
            if strcmp(get_param(mdl,'SimulationStatus'),'stopped'), break; end
            pause(0.02);
        end
    end
    set_param(mdl,'SimulationCommand','start');
end

% --------------------------- live toggles -----------------------------------
function spray_cb(src,~)
    P = evalin('base','P');  on = get(src,'Value')==1;  P.spray.joy = on;
    if on, set(src,'String','Spray: ON','BackgroundColor',[0.4 0.7 1.0]);
    else,  set(src,'String','Spray: OFF','BackgroundColor',[0.85 0.85 0.85]); end
    assignin('base','P',P);
end
function gps_cb(src,~)
    P = evalin('base','P');  P.fault.gps.on = get(src,'Value')==1;
    if P.fault.gps.on, set(src,'String','GPS: DROPPED','BackgroundColor',[0.95 0.5 0.5]);
    else,             set(src,'String','GPS: OK','BackgroundColor',[0.60 0.88 0.60]); end
    assignin('base','P',P);
end
function motor_cb(src,~)
    P = evalin('base','P');  P.fault.motor.on = get(src,'Value')==1;
    if P.fault.motor.on
        P.fault.motor.scale = [0.6;1;1;1];
        set(src,'String','Motor M1: 60% FAIL','BackgroundColor',[0.95 0.5 0.5]);
    else
        P.fault.motor.scale = [1;1;1;1];
        set(src,'String','Motor M1: healthy','BackgroundColor',[0.60 0.88 0.60]);
    end
    assignin('base','P',P);
end
function wind_cb(src,f)
    P = evalin('base','P');  F = get(src,'Value');
    if F < 0.05, P.wind.mode='none';  P.wind.F=[0;0;0];
    else,        P.wind.mode='const'; P.wind.t_on=0;  P.wind.F=[F;0;0]; end
    set(findobj(f,'Tag','wtxt'),'String',sprintf('%.1f',F));
    assignin('base','P',P);
end
function gain_cb(s, tag, f)
    P = evalin('base','P');  v = get(s,'Value');
    switch tag
        case 'Kp',  P.Kp_pos(1:2) = v;     case 'Kd',  P.Kd_pos(1:2) = v;
        case 'Ki',  P.Ki_pos(1:2) = v;     case 'AKp', P.Kp_att(1:2) = v;
        case 'AKd', P.Kd_att(1:2) = v;     case 'YKp', P.Kp_att(3)   = v;
        case 'YKd', P.Kd_att(3)   = v;
    end
    set(findobj(f,'Tag',[tag 'txt']),'String',sprintf('%.2f',v));
    assignin('base','P',P);
end
function ctrl_cb(src,~)
    P = evalin('base','P');  islqr = get(src,'Value')==1;
    if islqr, P.ctrl.mode='lqr'; set(src,'String','Controller: LQR','BackgroundColor',[0.75 0.85 1.0]);
    else,     P.ctrl.mode='pid'; set(src,'String','Controller: PID','BackgroundColor',[0.80 0.90 0.75]); end
    assignin('base','P',P);
end

% --------------------------- resets / autotune ------------------------------
function reset_faults(f)
    P = evalin('base','P');
    P.spray.joy = 0;  P.fault.gps.on = 0;  P.fault.motor.on = 0;  P.fault.motor.scale = [1;1;1;1];
    P.wind.mode = 'none';  P.wind.F = [0;0;0];
    assignin('base','P',P);  reset_controls(f);
end
function reset_gains(f)
    P = evalin('base','P');
    P.Kp_pos = P.wn_pos.^2;  P.Kd_pos = 2*P.zeta_pos.*P.wn_pos;  P.Ki_pos = [1.0;1.0;2.0];
    P.Kp_att = P.wn_att.^2;  P.Kd_att = 2*P.zeta_att.*P.wn_att;
    assignin('base','P',P);
    setsl(f,'Kp',P.Kp_pos(1));  setsl(f,'Kd',P.Kd_pos(1));  setsl(f,'Ki',P.Ki_pos(1));
    setsl(f,'AKp',P.Kp_att(1)); setsl(f,'AKd',P.Kd_att(1));
    setsl(f,'YKp',P.Kp_att(3)); setsl(f,'YKd',P.Kd_att(3));
end
function tune_cb(f, method)
    % The sim auto-suspends while this callback runs and resumes when it returns
    % -- no pause/continue (that risks a stuck-paused sim). It freezes for the
    % compute, then keeps flying with the tuned gains.
    if strcmp(method,'ga'), tag = 'gatune'; lbl = 'Tune: GA'; else, tag = 'autotune'; lbl = 'Tune: PSO'; end
    b = findobj(f,'Tag',tag);
    set(b,'String','tuning...','Enable','off');  drawnow;
    err = '';
    try
        if strcmp(method,'ga'), ga_tune_pid(true); else, pso_tune_pid(true); end
    catch e, err = e.message;
    end
    P = evalin('base','P');
    setsl(f,'Kp',P.Kp_pos(1));  setsl(f,'Kd',P.Kd_pos(1));  setsl(f,'Ki',P.Ki_pos(1));
    setsl(f,'AKp',P.Kp_att(1)); setsl(f,'AKd',P.Kd_att(1));
    set(b,'String',lbl,'Enable','on');
    if ~isempty(err), errordlg(err,'tuning'); end
end
function compare_cb()
    try, compare_pid; catch e, errordlg(e.message,'compare'); end
end
function reset_controls(f)
    set(findobj(f,'Tag','spray'),'Value',0,'String','Spray: OFF','BackgroundColor',[0.85 0.85 0.85]);
    set(findobj(f,'Tag','gps'),  'Value',0,'String','GPS: OK','BackgroundColor',[0.60 0.88 0.60]);
    set(findobj(f,'Tag','motor'),'Value',0,'String','Motor M1: healthy','BackgroundColor',[0.60 0.88 0.60]);
    set(findobj(f,'Tag','wind'), 'Value',0);
    set(findobj(f,'Tag','wtxt'), 'String','0.0');
end
function setsl(f, tag, v)
    set(findobj(f,'Tag',tag),'Value',v);
    set(findobj(f,'Tag',[tag 'txt']),'String',sprintf('%.2f',v));
end
function s = ternstr(c,a,b),  if c, s=a; else, s=b; end,  end
function v = ternvec(c,a,b),  if c, v=a; else, v=b; end,  end
