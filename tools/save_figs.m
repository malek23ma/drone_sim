function save_figs()
%SAVE_FIGS  Run the presentation-worthy tests and export their figures as vector SVG.
%   Outputs to figures\*.svg  (and also high-res PNG as a fallback).
    outdir = fullfile(pwd,'figures');
    if ~exist(outdir,'dir'), mkdir(outdir); end

    sel = { ...
        'test_2a_translational' , 'freefall_validation'   ; ...
        'test_2b_rotational'    , 'energy_conservation'   ; ...
        'test_9_estimator'      , 'estimator_velocity'    ; ...
        'test_10_trajectory'    , 'circle_tracking'       ; ...
        'test_12_wind'          , 'wind_gust'             ; ...
        'test_13_integral'      , 'wind_droop_pid_vs_pd'  ; ...
        'test_14_spray'         , 'spray_altitude_mass'   };

    for i = 1:size(sel,1)
        try
            close all force;
            evalin('base','clear');
            evalin('base',[sel{i,1} ';']);            % runs the test -> opens its figure(s)
            figs = flipud(findall(0,'Type','figure'));% in creation order
            for k = 1:numel(figs)
                try, whiten(figs(k)); catch, end
                if numel(figs) == 1
                    base = sel{i,2};
                else
                    base = sprintf('%s_%d', sel{i,2}, k);
                end
                svg = fullfile(outdir,[base '.svg']);
                png = fullfile(outdir,[base '.png']);
                exportgraphics(figs(k), svg, 'ContentType','vector');
                exportgraphics(figs(k), png, 'Resolution',300);
                fprintf('saved %s\n', base);
            end
        catch ME
            fprintf('FAILED %s : %s\n', sel{i,1}, ME.message);
        end
    end
    close all force;
    fprintf('\n>>> figures exported to %s\n', outdir);
end
