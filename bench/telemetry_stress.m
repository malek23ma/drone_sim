function telemetry_stress(capture_file)
%TELEMETRY_STRESS  Analyze a captured C6 telemetry CSV dump for packet loss.
%   telemetry_stress("c6_capture.txt")
%
%   Capture: any text file with the 39-field CSV lines the S3 sends to the
%   C6 at 4 Hz (save the C6 dashboard/serial output for a few minutes,
%   ideally at increasing distances). Uses field 39 (loop DT) + line count
%   vs wall time to estimate delivery rate and gaps.
    txt = fileread(capture_file);
    lines = splitlines(string(txt));
    ok = 0; bad = 0; dts = [];
    for i = 1:numel(lines)
        f = split(lines(i), ",");
        if numel(f) == 39
            ok = ok + 1;
            dts(end+1) = str2double(f(39)); %#ok<AGROW>
        elseif strlength(strtrim(lines(i))) > 0
            bad = bad + 1;   % truncated/corrupted line = radio loss artifact
        end
    end
    fprintf('Valid packets   : %d\n', ok);
    fprintf('Corrupt/partial : %d  (%.1f%%)\n', bad, 100*bad/max(ok+bad,1));
    fprintf('Expected rate 4 Hz -> capture of %.1f s worth of packets\n', ok/4);
    if ~isempty(dts)
        fprintf('Flight loop DT over link: mean %.4f s, max %.4f s\n', ...
                mean(dts,'omitnan'), max(dts));
    end
    fprintf('Repeat at 1 m / 10 m / 30 m etc. and compare corrupt %% for a range test.\n');
end
