function dummy = viz_islfcn(in)
%VIZ_ISLFCN  Live 3D animation of the quad (Interpreted MATLAB Function block).
%   Real-world style view: perspective camera, ground shadow + altitude line,
%   North = away, East = right, Up = up. Draws the drone (X-airframe + rotors +
%   green nose), its target star, the flown path, and a command vector.
%
%   Input  in = [ X(1:12) ; ref(13:22) ; t(23) ]
%   Output dummy = 0  -> wire to a Terminator / To Workspace.
%   Draws ONLY when P.viz.on = 1.
    persistent Pc H trail lastDraw lastT CM wet crashed batt_used tank_used

    dummy = 0;
    t = in(23);

    newsim = isempty(lastT) || t < lastT - 1e-9;
    lastT  = t;
    if isempty(Pc) || newsim, Pc = evalin('base','P'); end
    P = Pc;
    if ~isfield(P,'viz') || ~P.viz.on, return; end

    pos = in(1:3);  eul = in(7:9);  ref_pos = in(13:15);  vel_ref = in(16:18);

    if isempty(H) || ~isfield(H,'fig') || ~ishandle(H.fig)
        H = local_init();  trail = zeros(0,3);  lastDraw = -inf;  CM = [];  wet = zeros(0,2);
        crashed = false;  batt_used = 0;  tank_used = 0;
        set(H.keys,'Visible',onoff(P.pilot.on));
    elseif newsim
        trail = zeros(0,3);  lastDraw = -inf;  CM = [];  wet = zeros(0,2);
        crashed = false;  batt_used = 0;  tank_used = 0;
        set(H.banner,'Visible','off');
        set(H.keys,'Visible',onoff(P.pilot.on));
    end

    % ---- ground contact: latch, banner, stop the sim (runs every step) ----
    if ~crashed && -pos(3) <= 0
        crashed = true;
        vimp = norm(in(4:6));                         % impact speed [m/s]
        if vimp > 1.5
            set(H.banner,'String',sprintf('CRASHED    impact %.1f m/s', vimp), ...
                'Color',[0.85 0 0],'Visible','on');
        else
            set(H.banner,'String',sprintf('LANDED    %.1f m/s', vimp), ...
                'Color',[0 0.55 0],'Visible','on');
        end
        drawnow;
        try, set_param('drone_plant','SimulationCommand','stop'); catch, end
        return;
    end
    prevDraw = lastDraw;
    if (t - lastDraw) < 1/30, return; end
    dt = t - prevDraw;                                % sim time since last frame
    lastDraw = t;
    P = evalin('base','P');  Pc = P;                  % live refresh at 30 Hz so panel/
                                                      % gamepad toggles (spray, etc.) show now

    % attitude R (body->world, NED, ZYX)
    phi=eul(1); th=eul(2); ps=eul(3);
    cphi=cos(phi);sphi=sin(phi);cth=cos(th);sth=sin(th);cps=cos(ps);sps=sin(ps);
    R = [cps*cth, cps*sth*sphi-sps*cphi, cps*sth*cphi+sps*sphi;
         sps*cth, sps*sth*sphi+cps*cphi, sps*sth*cphi-cps*sphi;
         -sth,    cth*sphi,              cth*cphi];

    vscale = 4;                                       % visual-only airframe scale
    d  = vscale*P.L/sqrt(2);
    Mb = [ d -d 0;  d  d 0; -d  d 0; -d -d 0]';
    Mned = pos + R*Mb;
    Mp = [Mned(2,:); Mned(1,:); -Mned(3,:)];          % plot coords (East,North,Up)
    C  = [pos(2); pos(1); -pos(3)];

    nose_ned = pos + R*[vscale*P.L*0.9; 0; 0];
    Np = [nose_ned(2); nose_ned(1); -nose_ned(3)];

    if isfield(P.pilot,'tgt_lead'), tlead = P.pilot.tgt_lead; else, tlead = 0.6; end
    tgt_ned = ref_pos + tlead*vel_ref;
    Tp = [tgt_ned(2); tgt_ned(1); -tgt_ned(3)];

    set(H.arm1,'XData',Mp(1,[1 3]),'YData',Mp(2,[1 3]),'ZData',Mp(3,[1 3]));
    set(H.arm2,'XData',Mp(1,[2 4]),'YData',Mp(2,[2 4]),'ZData',Mp(3,[2 4]));
    set(H.mot ,'XData',Mp(1,:),'YData',Mp(2,:),'ZData',Mp(3,:));
    set(H.body,'XData',C(1),'YData',C(2),'ZData',C(3));
    set(H.nose,'XData',Np(1),'YData',Np(2),'ZData',Np(3));
    set(H.tgt ,'XData',Tp(1),'YData',Tp(2),'ZData',Tp(3));
    set(H.cmd ,'XData',[C(1) Tp(1)],'YData',[C(2) Tp(2)],'ZData',[C(3) Tp(3)]);

    % ground shadow + altitude line (depth / height cues)
    set(H.shadow ,'XData',C(1),'YData',C(2),'ZData',0);
    set(H.altline,'XData',[C(1) C(1)],'YData',[C(2) C(2)],'ZData',[0 C(3)]);

    trail(end+1,:) = C';
    if size(trail,1) > 3000, trail(1,:) = []; end
    set(H.trail,'XData',trail(:,1),'YData',trail(:,2),'ZData',trail(:,3));

    if isfield(P.viz,'cam') && strcmp(P.viz.cam,'field')      % fixed view of the whole field
        f = P.field;
        xlim(H.ax, f.corner(2) + [-3, f.W+3]);  ylim(H.ax, f.corner(1) + [-3, f.Lp+3]);
        zlim(H.ax, [0 7]);
    else                                                      % chase camera
        xlim(H.ax, C(1)+[-10 10]);  ylim(H.ax, C(2)+[-10 10]);  zlim(H.ax,[0 max(7, C(3)+3)]);
    end
    title(H.ax, sprintf('%s     t = %.1f s     altitude = %.2f m', upper(P.ctrl.mode), t, -pos(3)));

    % ---- cockpit gauges: altitude, speed, battery SoC, tank ----
    dtb   = min(dt, 0.1);                                  % clamp (frame 1 dt = inf)
    Test  = P.m*P.g / max(cos(eul(1))*cos(eul(2)), 0.5);   % thrust to hold weight at tilt
    w_est = sqrt(max(Test/(4*P.kf), 0));                   % implied rotor speed
    I_pk  = 4*(P.km*w_est^2/P.Kt + P.I0);                  % pack current via battery model
    batt_used = batt_used + I_pk*dtb;
    soc   = max(0, 100*(1 - batt_used/(P.batt.Q_full*(1-P.batt.reserve))));
    if local_is_spraying(P,t)
        if strcmp(P.spray.mode,'field'), fr = P.field.rate; else, fr = P.spray.rate; end
        tank_used = min(tank_used + fr*dtb, P.tank_full);
    end
    tankpct = max(0, 100*(1 - tank_used/P.tank_full));
    set(H.gauge,'String',sprintf([' ALT %.1f m  |  SPD %.1f m/s  |  POS N %.1f E %.1f m  |  ' ...
        'RPY %.0f/%.0f/%.0f deg  |  BATT %.0f%%  |  TANK %.0f%% '], ...
        -pos(3), norm(in(4:6)), pos(1), pos(2), ...
        eul(1)*180/pi, eul(2)*180/pi, eul(3)*180/pi, soc, tankpct));

    % ---- spray visual: tilted belly plume to the ground + wet-ground trail ----
    if local_is_spraying(P, t)
        ang  = 12;  if isfield(P.spray,'cone_ang'), ang = P.spray.cone_ang; end
        bdir = R(:,3);                          % belly (+z body) direction in world NED
        if bdir(3) > 0.15                       % belly points down enough to reach ground
            lam  = min(-pos(3)/bdir(3), 60);    % slant length drone->ground (clamped)
            hit  = pos + lam*bdir;              % NED point where the plume meets the ground
            G    = [hit(2); hit(1); -hit(3)];   % plot coords (East,North,Up~0)
            r    = max(0.2, lam*tand(ang));     % footprint radius grows with height/tilt
            th   = linspace(0,2*pi,17);
            base = [G(1)+r*cos(th); G(2)+r*sin(th); zeros(1,17)];   % ground ring
            apex = repmat(C,1,17);                                  % all rays meet at the nozzle
            set(H.spray,'XData',[apex(1,:);base(1,:)], ...
                        'YData',[apex(2,:);base(2,:)], ...
                        'ZData',[apex(3,:);base(3,:)],'Visible','on');
            np  = 10;                            % droplets per frame, filling the footprint
            rr  = r*sqrt(rand(np,1));            % sqrt -> uniform over the disk (no center clump)
            aa  = 2*pi*rand(np,1);
            wet = [wet; G(1)+rr.*cos(aa), G(2)+rr.*sin(aa)];     %#ok<AGROW>
            if size(wet,1) > 12000, wet(1:np,:) = []; end
            set(H.wet,'XData',wet(:,1),'YData',wet(:,2),'ZData',0.02+zeros(size(wet,1),1));
        else
            set(H.spray,'Visible','off');       % drone tipped too far -> no ground hit
        end
    else
        set(H.spray,'Visible','off');
    end

    % ---- live coverage map (only during an auto-spray field mission) ----
    if strcmp(P.traj.mode,'field') && strcmp(P.spray.mode,'field')
        if isempty(CM) || ~ishandle(CM.fig), CM = local_cov_init(P); end
        ddt = min(dt, 0.1);                                 % clamp (frame 1 dt=inf)
        [~, ~, is_pass] = field_path(t, P);
        spraying = is_pass && (CM.sprayed < P.tank_full);
        if spraying
            if ~CM.prevspray, CM.passID = CM.passID + 1; end  % a new pass began
            CM.sprayed = CM.sprayed + P.field.rate*ddt;       % mirror tank drain (diffuser flow)
            r2   = (P.field.spray_w/2)^2;                     % circular nozzle footprint
            mask = (CM.N - pos(1)).^2 + (CM.E - pos(2)).^2 <= r2;
            ov   = mask & (CM.lastpass>0) & (CM.lastpass ~= CM.passID);
            CM.over(ov)       = true;                         % re-sprayed by another pass
            CM.lastpass(mask) = CM.passID;
            if CM.sprayed >= P.tank_full && isempty(CM.dryPt) % just ran dry
                CM.dryPt = [pos(2) pos(1)];                   % [East North]
                set(CM.dry,'XData',CM.dryPt(1),'YData',CM.dryPt(2));
            end
        end
        CM.prevspray = spraying;
        cdata = double(CM.lastpass>0) + double(CM.over);      % 0 none, 1 sprayed, 2 overlap
        set(CM.img,'CData', cdata);
        ncov = nnz(CM.lastpass>0);  nov = nnz(CM.over);
        pct  = 100*ncov/numel(CM.lastpass);
        if ncov>0, pov = 100*nov/ncov; else, pov = 0; end
        tankpct = max(0, 100*(1 - CM.sprayed/P.tank_full));
        set(CM.ttl,'String', sprintf(['Coverage %.0f%%      Overlap %.0f%%' ...
            '      Tank %.0f%%      t = %.1f s'], pct, pov, tankpct, t));
    end

    drawnow limitrate;
    local_rec(H, CM, t);
end

function local_rec(H, CM, t)
%LOCAL_REC  Append frames to the video started by record_mission: 3D view on the
%   left, coverage map on the right. Frames follow SIM time (duplicated when the
%   draw rate falls behind), so the video plays at real speed.
    rec = getappdata(groot,'drone_rec');
    if isempty(rec), return; end
    nwant = floor(t*rec.fps) + 1;                     % frames that should exist by now
    if rec.n >= nwant, return; end
    drawnow;
    F = getframe(H.fig);  img = imresize(F.cdata, rec.size3d);
    if ~isempty(CM) && ishandle(CM.fig)
        F = getframe(CM.fig);  map = imresize(F.cdata, rec.sizemap);
    else
        map = 255*ones([rec.sizemap 3],'uint8');
    end
    frame = [img, map];
    while rec.n < nwant
        writeVideo(rec.vw, frame);  rec.n = rec.n + 1;
    end
    setappdata(groot,'drone_rec',rec);
end

function local_place(fig, field)
%LOCAL_PLACE  While recording, give the window a fixed size so every frame matches.
    rec = getappdata(groot,'drone_rec');
    if isempty(rec), return; end
    set(fig,'WindowStyle','normal','Units','pixels','Position',rec.(field));
end

function light_theme(fig)
%LIGHT_THEME  Keep the plots light even when MATLAB runs in dark mode (legends
%   and titles would otherwise turn dark on the white figure).
    if isprop(fig,'Theme'), fig.Theme = 'light'; end
end

function H = local_init()
    H.fig = figure('Name','Quad - live flight','NumberTitle','off','Color','w');
    drone_keys('attach', H.fig);             % fly with the keyboard while this window has focus
    local_place(H.fig, 'pos3d');
    light_theme(H.fig);
    H.ax  = axes('Parent',H.fig); hold(H.ax,'on'); grid(H.ax,'on');
    daspect(H.ax,[1 1 1]);
    view(H.ax,-20,22);                       % North = away, East = right, Up = up
    camproj(H.ax,'perspective');             % real-world depth
    set(H.ax,'Color',[0.90 0.95 1.0]);       % sky tint
    xlabel(H.ax,'East [m]'); ylabel(H.ax,'North [m]'); zlabel(H.ax,'Up [m]');
    patch('Parent',H.ax,'XData',[-40 40 40 -40],'YData',[-40 -40 40 40], ...
          'ZData',[0 0 0 0],'FaceColor',[0.80 0.90 0.72],'FaceAlpha',0.9, ...
          'EdgeColor',[0.6 0.75 0.55]);                          % ground / field
    H.altline= plot3(H.ax,[0 0],[0 0],[0 0],':','Color',[0.5 0.5 0.5],'LineWidth',1.0);
    H.shadow = plot3(H.ax,0,0,0,'o','MarkerFaceColor',[0.45 0.45 0.45], ...
                     'MarkerEdgeColor','none','MarkerSize',6,'LineStyle','none');
    H.cmd    = plot3(H.ax,[0 0],[0 0],[0 0],'--','Color',[1 0.4 0.4],'LineWidth',1.0);
    H.trail  = plot3(H.ax,0,0,0,'-','Color',[0.2 0.5 1],'LineWidth',1.4);
    H.wet    = plot3(H.ax,nan,nan,nan,'.','Color',[0.34 0.22 0.11], ...
                     'MarkerSize',9,'LineStyle','none');           % wet-soil patches on ground
    H.spray = surf(H.ax, nan(2,17), nan(2,17), nan(2,17), ...
                   'FaceColor',[0.2 0.5 1.0],'FaceAlpha',0.18, ...
                   'EdgeColor','none','Visible','off');            % belly spray plume
    H.arm1   = plot3(H.ax,[0 0],[0 0],[0 0],'-','Color',[0 0.2 0.8],'LineWidth',6);
    H.arm2   = plot3(H.ax,[0 0],[0 0],[0 0],'-','Color',[0 0.2 0.8],'LineWidth',6);
    H.mot    = plot3(H.ax,0,0,0,'o','MarkerFaceColor','k','MarkerEdgeColor','k', ...
                     'MarkerSize',9,'LineStyle','none');
    H.body   = plot3(H.ax,0,0,0,'o','MarkerFaceColor',[0 0.2 0.8],'MarkerEdgeColor','k', ...
                     'MarkerSize',7,'LineStyle','none');
    H.nose   = plot3(H.ax,0,0,0,'^','MarkerFaceColor',[0 0.7 0],'MarkerEdgeColor','k', ...
                     'MarkerSize',9,'LineStyle','none');
    H.tgt    = plot3(H.ax,0,0,0,'p','MarkerFaceColor','r','MarkerEdgeColor','r', ...
                     'MarkerSize',9,'LineStyle','none');
    legend(H.ax,[H.trail H.tgt H.nose],{'flown path','target','nose (heading)'}, ...
           'Location','east','AutoUpdate','off');      % clear of the gauge bar on top
    H.banner = text(H.ax, 0.5, 0.90, '', 'Units','normalized','FontSize',22, ...
        'FontWeight','bold','HorizontalAlignment','center','Color','r','Visible','off');
    H.gauge  = text(H.ax, 0.5, 0.995, '', 'Units','normalized','FontName','Consolas', ...
        'FontSize',10,'FontWeight','bold','VerticalAlignment','top', ...
        'HorizontalAlignment','center','Color',[0 0 0.35], ...
        'BackgroundColor',[0.96 0.96 0.96],'EdgeColor',[0.7 0.7 0.7],'Margin',4);
    H.keys   = uicontrol(H.fig,'Style','text','Units','normalized','Position',[0 0 1 0.04], ...
        'String','Arrows: move     W / S: up / down     A / D: turn     Space: spray', ...
        'FontSize',10,'BackgroundColor',[0.96 0.96 0.96],'ForegroundColor',[0.2 0.2 0.2], ...
        'Enable','inactive','Visible','off');   % manual-mode key hint
end

function s = onoff(tf)
    if tf, s = 'on'; else, s = 'off'; end
end

function s = local_is_spraying(P, t)
%LOCAL_IS_SPRAYING  True when water is coming out: manual joystick toggle, or an
%   autonomous field pass. Used to drive the spray cone + wet-ground visual.
    s = false;
    if ~isfield(P,'spray'), return; end
    if isfield(P.spray,'joy') && P.spray.joy
        s = true;                                  % manual: joystick button ON
    elseif strcmp(P.spray.mode,'field') && strcmp(P.traj.mode,'field')
        [~, ~, s] = field_path(t, P);              % auto: only on a coverage pass
    end
end

function C = local_cov_init(P)
%LOCAL_COV_INIT  Build the top-down coverage map (own figure) + spray grid.
    f   = P.field;
    res = 0.25;                                             % grid cell [m] (finer = smoother)
    Ec  = (f.corner(2)+res/2) : res : (f.corner(2)+f.W);    % East cell centers
    Nc  = (f.corner(1)+res/2) : res : (f.corner(1)+f.Lp);   % North cell centers
    [C.E, C.N] = meshgrid(Ec, Nc);                          % rows=North, cols=East
    C.lastpass = zeros(size(C.E));                          % which pass last sprayed a cell
    C.over     = false(size(C.E));                          % sprayed by >1 pass (waste)
    C.passID   = 0;                                         % current pass counter
    C.prevspray= false;                                     % rising-edge detect for passID
    C.sprayed  = 0;                                         % cumulative mass [kg]
    C.dryPt    = [];                                        % where tank ran dry

    C.fig = figure('Name','Spray coverage map','NumberTitle','off','Color','w');
    local_place(C.fig, 'posmap');
    light_theme(C.fig);
    C.ax  = axes('Parent',C.fig);  hold(C.ax,'on');  box(C.ax,'on');
    C.img = imagesc(C.ax, Ec, Nc, zeros(size(C.E)));
    set(C.ax,'YDir','normal','Layer','top');
    colormap(C.ax, [0.96 0.96 0.96;     % 0 = not sprayed (light gray)
                    0.20 0.70 0.30;     % 1 = sprayed once (green)
                    1.00 0.55 0.00]);   % 2 = double-sprayed / overlap (orange)
    caxis(C.ax,[0 2]);
    daspect(C.ax,[1 1 1]);
    axis(C.ax,[f.corner(2)-1, f.corner(2)+f.W+1, f.corner(1)-1, f.corner(1)+f.Lp+1]);
    rectangle('Parent',C.ax,'Position',[f.corner(2) f.corner(1) f.W f.Lp], ...
              'EdgeColor',[0 0.45 0],'LineWidth',2);        % field boundary
    C.dry = plot(C.ax, nan, nan, 'kx','MarkerSize',16,'LineWidth',3);
    xlabel(C.ax,'East [m]','Color','k');  ylabel(C.ax,'North [m]','Color','k');
    set(C.ax,'XColor','k','YColor','k');
    C.ttl = title(C.ax,'Spray coverage','FontSize',13,'FontWeight','bold','Color','k');

    % legend via dummy patches (imagesc colors aren't legend-able directly)
    hC = patch(C.ax,'XData',nan,'YData',nan,'FaceColor',[0.20 0.70 0.30],'EdgeColor','none');
    hO = patch(C.ax,'XData',nan,'YData',nan,'FaceColor',[1.00 0.55 0.00],'EdgeColor','none');
    lg = legend([hC hO C.dry], {'sprayed','double-sprayed','tank empty'}, ...
                'Location','northeastoutside','AutoUpdate','off');
    set(lg,'TextColor','k');
end
