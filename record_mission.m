function record_mission(file, T, ctrl)
%RECORD_MISSION  Fly the field mission and save it as an MP4 (H.264): the 3D view
%   and the coverage map side by side. One frame is written per 1/30 s of SIM
%   time, so the video plays at real speed however slowly the simulation runs.
%
%   record_mission                         -> field_mission.mp4, 50 s, LQR
%   record_mission('out.mp4', 60, 'pid')   -> other file / length [s of sim time] / controller
%
%   The frames are grabbed by viz_islfcn (local_rec) while the model runs.
    if nargin < 1, file = 'field_mission.mp4'; end
    if nargin < 2, T = 50; end
    if nargin < 3, ctrl = 'lqr'; end
    mdl = 'drone_plant';
    load_system(mdl);

    evalin('base','model_params');
    P = evalin('base','P');
    P.viz.on = 1;  P.viz.cam = 'field';            % fixed camera over the whole field
    P.pilot.on = 0;  P.traj.mode = 'field';  P.spray.mode = 'field';  P.spray.joy = 0;
    P.ctrl.mode = ctrl;
    x0 = zeros(12,1);  x0(3) = -P.field.alt;
    assignin('base','P',P);  assignin('base','x0',x0);

    close(findobj(groot,'Type','figure','-regexp','Name','^(Quad - live flight|Spray coverage map)$'));
    evalin('base','clear viz_islfcn');              % fresh trail, map and gauges

    folder = fileparts(file);
    if ~isempty(folder) && ~exist(folder,'dir'), mkdir(folder); end
    vw = VideoWriter(file,'MPEG-4');
    vw.FrameRate = 30;  vw.Quality = 60;
    open(vw);
    rec = struct('vw',vw, 'fps',30, 'n',0, ...
                 'size3d',[720 1200], 'sizemap',[720 600], ...      % output pixels [h w]
                 'pos3d',[20 60 1200 720], 'posmap',[860 60 600 720]);
    setappdata(groot,'drone_rec',rec);
    done = onCleanup(@() local_finish(vw));         % close the file even if the sim errors

    fprintf('Recording %g s of the field mission (%s) to %s ...\n', T, upper(ctrl), file);
    sim(mdl,'StopTime',num2str(T));
    rec = getappdata(groot,'drone_rec');
    clear done
    d = dir(file);
    fprintf('Saved %s: %d frames (%.1f s), %.1f MB\n', file, rec.n, rec.n/rec.fps, d.bytes/1e6);
end

function local_finish(vw)
    close(vw);
    if isappdata(groot,'drone_rec'), rmappdata(groot,'drone_rec'); end
end
