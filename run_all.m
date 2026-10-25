function run_all()
%RUN_ALL  Master validation: run every test, print a pass/fail summary.
%   run_all is a FUNCTION so its loop variables live in its own workspace and
%   are not disturbed by each test's `clear`. Tests are executed IN BASE (via
%   evalin) so model_params populates the base workspace that the Simulink
%   Interpreted blocks read (P, x0, ...).
%
%   Tests 3-12 require drone_plant.slx to be built and on the path.

    close all;                            % clear figures from previous runs first

    tests = { ...
        'test_2a_translational'   ; 'test_2b_rotational'      ; ...
        'test_2c_kinematics'      ; 'test_2d_plant_validation'; ...
        'test_3_simulink_plant'   ; 'test_4_attitude'         ; ...
        'test_5_position'         ; 'test_6_allocation'       ; ...
        'test_7_motor'            ; 'test_8_sensors'          ; ...
        'test_9_estimator'        ; 'test_10_trajectory'      ; ...
        'test_12_wind'            ; 'test_13_integral'        ; ...
        'test_14_spray'           ; 'test_15_lqr' };

    res = strings(numel(tests),1);
    for i = 1:numel(tests)
        try
            evalin('base', [tests{i} ';']);   % run the test in the base workspace
            res(i) = "PASS";
        catch ME
            res(i) = "FAIL: " + ME.message;
        end
    end

    fprintf('\n=================== VALIDATION SUMMARY ===================\n');
    np = 0;
    for i = 1:numel(tests)
        fprintf('  %-26s %s\n', tests{i}, res(i));
        np = np + (res(i)=="PASS");
    end
    fprintf('---------------------------------------------------------\n');
    fprintf('  %d / %d PASSED\n', np, numel(tests));
    fprintf('=========================================================\n');
end
