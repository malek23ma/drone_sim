function make_report_figs()
%MAKE_REPORT_FIGS  Generate every report/presentation figure from the .m oracle.
%   Runs closed-loop scenarios (true-state feedback) and saves PNG + SVG into
%   figs/. Self-contained: calls model_params, no Simulink needed.
%   Each figure is isolated in try/catch so one failure does not stop the rest.
%
%   Usage:  >> make_report_figs
%   Output: figs/*.png and figs/*.svg, ready to drop into the report and deck.

    here = fileparts(mfilename('fullpath'));
    figdir = fullfile(here,'figs');
    if ~exist(figdir,'dir'), mkdir(figdir); end
    evalin('base','clear');               % clean base so model_params is fresh
    run(fullfile(here,'model_params.m')); %#ok<*NASGU>   % populate P in this workspace
    assignin('base','P',P);               % some islfcn helpers read base P
    set(groot,'defaultFigureColor','w','defaultAxesColor','w', ...
        'defaultAxesXColor','k','defaultAxesYColor','k','defaultAxesZColor','k');
    try, set(groot,'defaultFigureTheme','light'); catch, end   % R2025a+ kill dark theme

    gen = {@()fig_estimator(P,figdir), ...      % fig01 — was free-fall (dropped)
           @()fig_step_pid_lqr(P,figdir), ...   % fig02
           @()fig_tuned_step(P,figdir), ...     % fig03
           @()fig_pso_ga(figdir), ...           % fig04
           @()fig_drag_coast(P,figdir), ...     % fig05 — honest coast-down (was terminal)
           @()fig_gust(P,figdir), ...           % fig06 — gust (was cruise-tilt, dropped)
           @()fig_wind_pd_pid(P,figdir), ...    % fig07
           @()fig_motor_fault(P,figdir), ...    % fig08
           @()fig_battery(P,figdir), ...        % fig09
           @()fig_coverage(P,figdir)};          % fig10 — closed-loop actual position
    names = {'estimator','step PID vs LQR','nominal vs tuned','PSO vs GA', ...
             'drag coast-down','wind gust','wind PD vs PID','motor fault', ...
             'battery endurance','coverage (realistic)'};
    for i = 1:numel(gen)
        try
            f = gen{i}();  savefig_both(f, figdir, sprintf('fig%02d', i));
            fprintf('  [ok]  fig%02d  %s\n', i, names{i});
        catch e
            fprintf('  [FAIL] fig%02d %s : %s\n', i, names{i}, e.message);
        end
    end
    fprintf('\nDone. Figures in: %s\n', figdir);
end

% =================== shared closed-loop oracle ===================
function [t,X] = clsim(P, sp, T, dt, wind, twind, mscale, mt)
%CLSIM  fixed-step closed loop, true-state feedback. Optional wind + motor fault.
    N = round(T/dt);  X = zeros(N,12);  t = (1:N)'*dt;
    xk = zeros(12,1);  e_int = zeros(3,1);  lim = P.int_limit;
    ref = [sp; 0;0;0; 0;0;0; 0];
    assignin('base','P',P);                       % keep LQR-delegation evalin consistent
    for k = 1:N
        tk = k*dt;
        u_cmd = pos_att_ctrl_islfcn([xk; ref; e_int], P);
        sc = [1;1;1;1];  if nargin>=8 && ~isempty(mscale) && tk>=mt, sc = mscale; end
        s2 = max(P.Mix_inv*u_cmd, 0);             % mixer + motor fault + forces (inline)
        wrench = P.Mix*(s2.*sc);
        Fw = [0;0;0];
        if nargin>=6 && ~isempty(wind) && tk>=twind(1) && tk<=twind(2), Fw = wind; end
        xk = xk + dt*quad_plant(0, xk, wrench, P, Fw, P.m);
        e_int = max(min(e_int + dt*(sp - xk(1:3)), lim), -lim);
        X(k,:) = xk.';
        if any(~isfinite(xk)), X(k:end,:) = repmat(X(max(k-1,1),:),N-k+1,1); break; end
    end
end

% =================== individual figures ===================
function f = fig_estimator(P, ~)
    % closed loop flown on the ESTIMATE: true plant -> noisy sensors -> observer
    dt = P.Ts;  T = 8;  N = round(T/dt);  t = (1:N)'*dt;
    sp = [2;2;-3];  ref = [sp; 0;0;0; 0;0;0; 0];
    x = zeros(12,1);  xh = zeros(12,1);  e_int = zeros(3,1);  lim = P.int_limit;
    assignin('base','P',P);
    Vt = zeros(N,3);  Vh = zeros(N,3);
    for k = 1:N
        u  = pos_att_ctrl_islfcn([xh; ref; e_int], P);       % control on the ESTIMATE
        wr = P.Mix*max(P.Mix_inv*u, 0);
        x  = x + dt*quad_plant(0, x, wr, P);                  % true plant
        y  = sensors_islfcn(x);                               % noisy GPS/AHRS/gyro
        xh = xh + dt*obs_islfcn([xh; y; wr; P.m]);            % observer update
        e_int = max(min(e_int + dt*(sp - xh(1:3)), lim), -lim);
        Vt(k,:) = x(4:6).';  Vh(k,:) = xh(4:6).';
    end
    rms = sqrt(mean(sum((Vt-Vh).^2,2)));
    f = figure('Position',[100 100 740 420]); hold on; grid on;
    plot(t, Vt(:,1), 'b','LineWidth',1.7); plot(t, Vh(:,1), 'r--','LineWidth',1.5);
    plot(t, Vt(:,2), 'Color',[0 0.5 0],'LineWidth',1.2); plot(t, Vh(:,2),'--','Color',[0.6 0 0.6],'LineWidth',1.2);
    xlabel('time [s]'); ylabel('velocity [m/s]');
    title(sprintf('State estimator: true vs estimated velocity (%.1f cm/s RMS)', rms*100));
    legend('North true','North est','East true','East est','Location','northeast');
end

function f = fig_step_pid_lqr(P, ~)
    Pp = P; Pp.ctrl.mode='pid';  Pl = P; Pl.ctrl.mode='lqr';
    [t,Xp] = clsim(Pp,[2;2;-3],6,0.004);
    [~,Xl] = clsim(Pl,[2;2;-3],6,0.004);
    f = figure('Position',[100 100 760 520]);
    subplot(2,1,1); hold on; grid on;
    plot(t,-Xp(:,3),'b','LineWidth',1.6); plot(t,-Xl(:,3),'r','LineWidth',1.6); yline(3,'k--');
    ylabel('altitude [m]'); title('Step response: PID vs LQR'); legend('PID','LQR','cmd','Location','southeast');
    subplot(2,1,2); hold on; grid on;
    plot(t,Xp(:,1),'b','LineWidth',1.6); plot(t,Xl(:,1),'r','LineWidth',1.6); yline(2,'k--');
    xlabel('time [s]'); ylabel('North position [m]'); legend('PID','LQR','cmd','Location','southeast');
end

function f = fig_tuned_step(P, ~)
    best = pso_tune_pid(false);                   % tuned gains (no apply)
    Pn = P;  Pn.ctrl.mode='pid';
    Pn.Kp_pos=Pn.wn_pos.^2; Pn.Kd_pos=2*Pn.zeta_pos.*Pn.wn_pos; Pn.Ki_pos=[1;1;2];
    Pn.Kp_att=Pn.wn_att.^2; Pn.Kd_att=2*Pn.zeta_att.*Pn.wn_att;
    Pt = Pn; Pt.Kp_pos(1:2)=best(1); Pt.Kd_pos(1:2)=best(2); Pt.Ki_pos(1:2)=best(3);
    Pt.Kp_att(1:2)=best(4); Pt.Kd_att(1:2)=best(5);
    [t,Xn]=clsim(Pn,[2;2;-3],6,0.004);  [~,Xt]=clsim(Pt,[2;2;-3],6,0.004);
    f = figure('Position',[100 100 760 420]); hold on; grid on;
    plot(t,Xn(:,1),'b','LineWidth',1.7); plot(t,Xt(:,1),'r','LineWidth',1.7); yline(2,'k--');
    xlabel('time [s]'); ylabel('North position [m]');
    title('Nominal vs PSO-tuned PID (step response)'); legend('nominal','PSO-tuned','cmd','Location','southeast');
end

function f = fig_pso_ga(~)
    [~,hp] = pso_tune_pid(false);  [~,hg] = ga_tune_pid(false);
    f = figure('Position',[100 100 700 440]); hold on; grid on;
    plot(1:numel(hp), hp,'-o','Color',[0.85 0.2 0.2],'LineWidth',1.6,'MarkerSize',4);
    plot(1:numel(hg), hg,'-s','Color',[0.2 0.4 0.9],'LineWidth',1.6,'MarkerSize',4);
    xlabel('iteration / generation'); ylabel('best cost J');
    title('Convergence: PSO vs GA'); legend('PSO','GA','Location','northeast');
end

function f = fig_drag_coast(P, ~)
    % honest drag: drone moving at 10 m/s, level hover thrust, no command ->
    % aerodynamic drag decelerates it to rest (what you see when you release the stick)
    dt=0.01; T=12; N=round(T/dt); t=(1:N)'*dt; v=zeros(N,1); x=zeros(12,1); x(4)=10;
    for k=1:N
        u=[P.m*P.g;0;0;0];                 % hover thrust, level
        x(7:12)=0;                          % keep level (isolate translational drag)
        x = x + dt*quad_plant(0,x,u,P); v(k)=norm(x(4:6));
    end
    f=figure('Position',[100 100 700 420]); plot(t,v,'b','LineWidth',1.9); grid on;
    xlabel('time [s]'); ylabel('speed [m/s]');
    title('Drag coast-down: 10 m/s \rightarrow rest (no command, drag only)');
end

function f = fig_gust(P, ~)
    % transient gust hits a hovering drone; controller rejects it
    [t,X] = clsim(P,[0;0;-3],10,0.004,[8;0;0],[4 5]);     % 8 N North gust, t=4..5 s
    f  = figure('Position',[100 100 720 420]);
    ax = axes('Parent',f);  hold(ax,'on');  grid(ax,'on');
    h  = plot(ax, t, X(:,1), 'b','LineWidth',1.7);
    plot(ax, t, zeros(size(t)), 'k--');                   % setpoint (plain line, not yline)
    yl = ylim(ax);
    plot(ax, [4 4], yl, ':', 'Color',[0.5 0.5 0.5]);      % gust window
    plot(ax, [5 5], yl, ':', 'Color',[0.5 0.5 0.5]);
    xlabel(ax,'time [s]'); ylabel(ax,'North position [m]');
    title(ax,'Wind-gust rejection (8 N gust at t = 4-5 s, then recovery)');
    legend(ax, h, 'drone position', 'Location','northeast');
end

function f = fig_wind_pd_pid(P, ~)
    Ppd=P; Ppd.use_integral=0;  Ppi=P; Ppi.use_integral=1;
    w=[5;0;0];  [t,Xpd]=clsim(Ppd,[0;0;-3],14,0.004,w,[3 14]);
    [~,Xpi]=clsim(Ppi,[0;0;-3],14,0.004,w,[3 14]);
    f=figure('Position',[100 100 740 420]); hold on; grid on;
    plot(t,Xpd(:,1),'b','LineWidth',1.6); plot(t,Xpi(:,1),'r','LineWidth',1.6); yline(0,'k--');
    xline(3,':','wind on');
    xlabel('time [s]'); ylabel('North position [m]');
    title('Constant wind: PD droops, PID returns to zero'); legend('PD','PID','setpoint','Location','northeast');
end

function f = fig_motor_fault(P, ~)
    [t,X]=clsim(P,[0;0;-3],8,0.004,[],[],[0.6;1;1;1],3);    % M1 -> 60% at t=3
    f=figure('Position',[100 100 740 460]);
    subplot(2,1,1); hold on; grid on; plot(t,-X(:,3),'b','LineWidth',1.6); yline(3,'k--'); xline(3,':','M1 fail');
    ylabel('altitude [m]'); title('Partial motor failure (M1 \rightarrow 60%)');
    legend('altitude','setpoint','Location','southeast');
    subplot(2,1,2); hold on; grid on;
    plot(t,X(:,7)*180/pi,'r','LineWidth',1.4); plot(t,X(:,8)*180/pi,'Color',[0 0.6 0],'LineWidth',1.4);
    xline(3,':'); xlabel('time [s]'); ylabel('attitude [deg]'); legend('roll','pitch','Location','northeast');
end

function f = fig_battery(P, ~)
    % battery state-of-charge vs flight time: gentle hover vs aggressive flight
    Qu = P.batt.Q_full*(1-P.batt.reserve);                 % usable charge to 20% reserve
    Ih = 4*(P.km*P.omega_hover^2/P.Kt + P.I0);             % hover pack current
    Ia = 4*(P.km*(1.25*P.omega_hover)^2/P.Kt + P.I0);      % ~25% extra thrust (climbs/manoeuvres)
    th = 0:2:ceil(Qu/Ih);  sh = max(0,100*(1 - Ih*th/Qu));
    ta = 0:2:ceil(Qu/Ia);  sa = max(0,100*(1 - Ia*ta/Qu));
    f=figure('Position',[100 100 720 420]); hold on; grid on;
    plot(th/60, sh, 'b','LineWidth',1.9); plot(ta/60, sa, 'r','LineWidth',1.9); yline(0,'k--');
    xlabel('flight time [min]'); ylabel('battery state of charge [%]');
    title(sprintf('Battery endurance to 20%% reserve  (hover ~%.1f min)', th(end)/60));
    legend(sprintf('hover (%.0f A)',Ih), sprintf('aggressive (%.0f A)',Ia), 'reserve','Location','northeast');
end

function f = fig_coverage(P, ~)
    % CLOSED-LOOP field mission: the drone TRACKS the lawnmower path and coverage
    % is marked at its ACTUAL position -> realistic gaps, not a perfect plan.
    fld = P.field;  res=0.25;
    Ec=(fld.corner(2)+res/2):res:(fld.corner(2)+fld.W);  Nc=(fld.corner(1)+res/2):res:(fld.corner(1)+fld.Lp);
    [E,Nn]=meshgrid(Ec,Nc);  cov=zeros(size(E));
    dt=0.02; Tt=1.1*((round(fld.W/fld.sw)+1)*fld.Lp + round(fld.W/fld.sw)*fld.sw)/fld.v;
    x=zeros(12,1); x(3)=-fld.alt; e_int=zeros(3,1); lim=P.int_limit;  assignin('base','P',P);
    for tk=0:dt:Tt
        [pref,vref,is_pass]=field_path(tk,P);
        ref=[pref; vref; 0;0;0; 0];
        u = pos_att_ctrl_islfcn([x; ref; e_int], P);
        x = x + dt*quad_plant(0, x, P.Mix*max(P.Mix_inv*u,0), P);
        e_int = max(min(e_int + dt*(pref - x(1:3)), lim), -lim);
        if is_pass
            m=(Nn-x(1)).^2+(E-x(2)).^2 <= (fld.spray_w/2)^2; cov(m)=cov(m)+1;
        end
    end
    fig=figure('Position',[100 100 520 640]); ax=axes('Parent',fig); set(ax,'YDir','normal'); hold(ax,'on');
    imagesc(ax,Ec,Nc,min(cov,2)); colormap(ax,[0.96 0.96 0.96;0.2 0.7 0.3;1 0.55 0]); clim(ax,[0 2]);
    axis(ax,'equal','tight'); rectangle('Parent',ax,'Position',[fld.corner(2) fld.corner(1) fld.W fld.Lp],'EdgeColor',[0 0.45 0],'LineWidth',2);
    xlabel(ax,'East [m]'); ylabel(ax,'North [m]');
    pct=100*nnz(cov>0)/numel(cov);
    title(ax,sprintf('Spray coverage, closed-loop  (%.0f%% covered)',pct));
    f = fig;
end

% =================== save helper ===================
function savefig_both(f, figdir, name)
    try, f.Theme = 'light'; catch, end                 % R2025a+: force light theme
    set(f,'Color','w');
    ax = findall(f,'Type','axes');
    set(ax,'Color','w','GridColor',[0.15 0.15 0.15], ...
           'XColor','k','YColor','k','ZColor','k');
    set(findall(f,'Type','text'),'Color','k');          % titles / axis labels
    lg = findall(f,'Type','legend');  set(lg,'TextColor','k','Color','w');
    try, exportgraphics(f, fullfile(figdir,[name '.png']),'Resolution',200,'BackgroundColor','white');
    catch, print(f, fullfile(figdir,[name '.png']),'-dpng','-r200'); end
    try, exportgraphics(f, fullfile(figdir,[name '.svg']),'BackgroundColor','white'); catch, end
    close(f);
end
