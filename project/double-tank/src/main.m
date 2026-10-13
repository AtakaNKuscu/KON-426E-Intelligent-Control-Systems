%% ============================================================
%  FILE: benchmark_our_NN_same_conditions_dt01.m
%
%  PURPOSE:
%     Test OUR NN controller under the same benchmark conditions
%     as friend's model.
%
%     No PI controller.
%     dt = 0.1 s.
%
%  TESTS:
%     1) Step response:
%        h_ref = 0.25 m
%
%     2) Staircase reference:
%        0.02 -> 0.10 -> 0.20 -> 0.28 -> 0.15 -> 0.05
%
%     3) Disturbance rejection:
%        h_ref = 0.2 m
%        at t = 200 s, h2 suddenly increases by +0.03 m
%
%     4) Noise rejection:
%        h_ref = 0.2 m
%        Band-limited white noise equivalent:
%        Noise Power = 1e-6, Sample Time = 5 s
%
%     5) Noise + disturbance:
%        h_ref = 0.2 m
%
%  INITIAL CONDITION:
%     h2(0) = 0.02 m
%     h1(0) = 0.04 m
%
%  REQUIREMENT:
%     Trained NN controller "net" must exist in workspace,
%     or trained_NN_controller.mat must contain net.
% ============================================================

clearvars -except net tr X_data Y_data;
clc; close all;

%% ============================================================
% 0) Load trained NN controller if needed
% ============================================================

if ~exist('net', 'var')
    if isfile('trained_NN_controller.mat')
        load('trained_NN_controller.mat', 'net');
        disp('Loaded trained NN controller from trained_NN_controller.mat');
    else
        error(['Trained neural network "net" was not found in workspace. ', ...
               'Run the training script first or save net into trained_NN_controller.mat.']);
    end
end

%% ============================================================
% 1) Plant parameters
% ============================================================

c1 = 1.035454e-2;
c2 = 1.143e-3;

H_max = 0.6;
V_max = 10;

h2_ref_max = 0.3;

%% ============================================================
% 2) Initial condition settings
% ============================================================

h2_initial = 0.02;
h1_initial = 2*h2_initial;   % h1(0) = 0.04, approximately steady-state consistent

fprintf('\nInitial conditions:\n');
fprintf('h1(0) = %.4f m\n', h1_initial);
fprintf('h2(0) = %.4f m\n', h2_initial);

%% ============================================================
% 3) Simulation settings
% ============================================================

dt = 0.1;

t_end_step  = 600;
t_end_stair = 480;
t_end_dist  = 600;
t_end_noise = 600;

%% ============================================================
% 4) Noise settings
% ============================================================
% Friend's Simulink block:
% Band-Limited White Noise
% Noise Power = 1e-6
% Sample Time = 5
%
% Approximate discrete equivalent:
% variance = NoisePower / SampleTime

noise_power = 1e-6;
noise_sample_time = 5;
noise_std = sqrt(noise_power / noise_sample_time);

fprintf('\nNoise settings:\n');
fprintf('Noise power       = %.2e\n', noise_power);
fprintf('Noise sample time = %.2f s\n', noise_sample_time);
fprintf('Noise std approx  = %.10f m\n', noise_std);

%% ============================================================
% 5) Disturbance settings
% ============================================================
% Sudden h2 level increase at t = 200 s

disturbance_time = 200;
disturbance_jump_h2 = 0.03;

fprintf('\nDisturbance settings:\n');
fprintf('Disturbance time       = %.2f s\n', disturbance_time);
fprintf('Disturbance h2 jump    = +%.4f m\n', disturbance_jump_h2);

%% ============================================================
% 6) Plot settings
% ============================================================

set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultLineLineWidth', 1.8);

%% ============================================================
% TEST 1: Step response, h_ref = 0.25
% ============================================================

t_step = 0:dt:t_end_step;
h_ref_step = 0.25 * ones(size(t_step));

res_step = simulate_our_NN(net, t_step, h_ref_step, ...
    c1, c2, H_max, V_max, dt, ...
    false, noise_std, noise_sample_time, ...
    false, disturbance_time, disturbance_jump_h2, ...
    h1_initial, h2_initial);

metrics_step = compute_metrics(t_step, h_ref_step, res_step.h2, res_step.u);

print_metric_table('STEP RESPONSE TEST: h_ref = 0.25', metrics_step);

plot_single_result(t_step, h_ref_step, res_step, ...
    'Step Response Test: h_{ref}=0.25 m', ...
    H_max, V_max, false, disturbance_time);

plot_h2_only(t_step, h_ref_step, res_step.h2, ...
    'h_2 vs Step Input: h_{ref}=0.25 m');

%% ============================================================
% TEST 2: Staircase reference
% 0.02 -> 0.10 -> 0.20 -> 0.28 -> 0.15 -> 0.05
% ============================================================

t_stair = 0:dt:t_end_stair;
h_ref_stair = generate_custom_staircase(t_stair, h2_ref_max);

res_stair = simulate_our_NN(net, t_stair, h_ref_stair, ...
    c1, c2, H_max, V_max, dt, ...
    false, noise_std, noise_sample_time, ...
    false, disturbance_time, disturbance_jump_h2, ...
    h1_initial, h2_initial);

metrics_stair = compute_metrics(t_stair, h_ref_stair, res_stair.h2, res_stair.u);

print_metric_table('STAIRCASE TEST: 0.02-0.10-0.20-0.28-0.15-0.05', metrics_stair);

plot_single_result(t_stair, h_ref_stair, res_stair, ...
    'Staircase Reference Test', ...
    H_max, V_max, false, disturbance_time);

plot_h2_only(t_stair, h_ref_stair, res_stair.h2, ...
    'h2 vs Step Input Over Time');

print_custom_staircase_segment_metrics(t_stair, h_ref_stair, res_stair.h2);

%% ============================================================
% TEST 3: Disturbance rejection, h_ref = 0.2
% ============================================================

t_dist = 0:dt:t_end_dist;
h_ref_dist = 0.2 * ones(size(t_dist));

res_dist = simulate_our_NN(net, t_dist, h_ref_dist, ...
    c1, c2, H_max, V_max, dt, ...
    false, noise_std, noise_sample_time, ...
    true, disturbance_time, disturbance_jump_h2, ...
    h1_initial, h2_initial);

metrics_dist = compute_metrics(t_dist, h_ref_dist, res_dist.h2, res_dist.u);
metrics_dist.RecoveryTime2 = compute_recovery_time(t_dist, h_ref_dist, res_dist.h2, disturbance_time, 0.02);

print_metric_table('DISTURBANCE TEST: h_ref = 0.2, h2 jump = +0.03 m', metrics_dist);

plot_single_result(t_dist, h_ref_dist, res_dist, ...
    'Disturbance Rejection Test: h_{ref}=0.2 m, +0.03 m h_2 jump', ...
    H_max, V_max, true, disturbance_time);

plot_h2_only_with_disturbance(t_dist, h_ref_dist, res_dist.h2, ...
    'h_2 Response with Disturbance: +0.03 m at t=200 s', ...
    disturbance_time);

%% ============================================================
% TEST 4: Noise rejection, h_ref = 0.2
% ============================================================

rng(42);

t_noise = 0:dt:t_end_noise;
h_ref_noise = 0.2 * ones(size(t_noise));

res_noise = simulate_our_NN(net, t_noise, h_ref_noise, ...
    c1, c2, H_max, V_max, dt, ...
    true, noise_std, noise_sample_time, ...
    false, disturbance_time, disturbance_jump_h2, ...
    h1_initial, h2_initial);

metrics_noise = compute_metrics(t_noise, h_ref_noise, res_noise.h2, res_noise.u);

print_metric_table('NOISE TEST: h_ref = 0.2, Noise Power = 1e-6, Ts_noise = 5', metrics_noise);

plot_single_result(t_noise, h_ref_noise, res_noise, ...
    'Noise Rejection Test: h_{ref}=0.2 m', ...
    H_max, V_max, false, disturbance_time);

plot_noise_measurements(t_noise, res_noise, ...
    'Noise Measurement Test: True and Measured Levels');

plot_h2_only(t_noise, h_ref_noise, res_noise.h2, ...
    'h_2 Response with Measurement Noise');

%% ============================================================
% TEST 5: Noise + disturbance, h_ref = 0.2
% ============================================================

rng(42);

res_noise_dist = simulate_our_NN(net, t_dist, h_ref_dist, ...
    c1, c2, H_max, V_max, dt, ...
    true, noise_std, noise_sample_time, ...
    true, disturbance_time, disturbance_jump_h2, ...
    h1_initial, h2_initial);

metrics_noise_dist = compute_metrics(t_dist, h_ref_dist, res_noise_dist.h2, res_noise_dist.u);
metrics_noise_dist.RecoveryTime2 = compute_recovery_time(t_dist, h_ref_dist, res_noise_dist.h2, disturbance_time, 0.02);

print_metric_table('NOISE + DISTURBANCE TEST: h_ref = 0.2', metrics_noise_dist);

plot_single_result(t_dist, h_ref_dist, res_noise_dist, ...
    'Noise + Disturbance Test: h_{ref}=0.2 m', ...
    H_max, V_max, true, disturbance_time);

plot_noise_measurements(t_dist, res_noise_dist, ...
    'Noise + Disturbance Measurement Test');

plot_h2_only_with_disturbance(t_dist, h_ref_dist, res_noise_dist.h2, ...
    'h_2 Response with Noise and Disturbance', ...
    disturbance_time);

disp('============================================================');
disp('All benchmark tests completed.');
disp('Figures are open in MATLAB. Save manually if needed.');
disp('============================================================');

%% ============================================================
% LOCAL FUNCTION: generate custom staircase
% ============================================================

function h_ref_stair = generate_custom_staircase(t, h2_ref_max)

    h_ref_stair = zeros(size(t));

    for k = 1:numel(t)

        if t(k) < 80
            h_ref_stair(k) = 0.02;

        elseif t(k) < 160
            h_ref_stair(k) = 0.10;

        elseif t(k) < 240
            h_ref_stair(k) = 0.20;

        elseif t(k) < 320
            h_ref_stair(k) = 0.28;

        elseif t(k) < 400
            h_ref_stair(k) = 0.15;

        else
            h_ref_stair(k) = 0.05;
        end
    end

    h_ref_stair = max(0, min(h_ref_stair, h2_ref_max));
end

%% ============================================================
% LOCAL FUNCTION: simulate OUR NN controller
% ============================================================

function res = simulate_our_NN(net, t, h_ref, ...
    c1, c2, H_max, V_max, dt, ...
    use_noise, noise_std, noise_sample_time, ...
    use_disturbance, disturbance_time, disturbance_jump_h2, ...
    h1_initial, h2_initial)

    N = numel(t);

    h1 = zeros(1, N);
    h2 = zeros(1, N);
    u  = zeros(1, N);

    h1_meas = zeros(1, N);
    h2_meas = zeros(1, N);

    h1(1) = h1_initial;
    h2(1) = h2_initial;

    disturbance_applied = false;

    % Band-limited white noise equivalent:
    % Noise is updated every noise_sample_time seconds and held constant.
    noise_h1_current = 0;
    noise_h2_current = 0;
    next_noise_update_time = 0;

    for k = 1:N-1

        % Update noise sample every noise_sample_time seconds
        if use_noise && t(k) >= next_noise_update_time - 1e-12
            noise_h1_current = noise_std * randn();
            noise_h2_current = noise_std * randn();
            next_noise_update_time = next_noise_update_time + noise_sample_time;
        end

        % Measurement
        if use_noise
            h1_m = h1(k) + noise_h1_current;
            h2_m = h2(k) + noise_h2_current;
        else
            h1_m = h1(k);
            h2_m = h2(k);
        end

        h1_m = max(0, min(h1_m, H_max));
        h2_m = max(0, min(h2_m, H_max));

        h1_meas(k) = h1_m;
        h2_meas(k) = h2_m;

        href_k = h_ref(k);

        % NN controller
        u_raw = net([h1_m; h2_m; href_k]);
        u(k) = max(0, min(u_raw, V_max));

        % Nonlinear coupled-tank plant
        diff_h = h1(k) - h2(k);
        sign_val = sign(diff_h);
        sqrt_diff = sqrt(abs(diff_h));

        dh1 = -c1 * sign_val * sqrt_diff + c2 * u(k);
        dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(max(h2(k), 0));

        h1(k+1) = h1(k) + dt * dh1;
        h2(k+1) = h2(k) + dt * dh2;

        h1(k+1) = max(0, min(h1(k+1), H_max));
        h2(k+1) = max(0, min(h2(k+1), H_max));

        % Disturbance: sudden positive h2 jump
        if use_disturbance && ~disturbance_applied
            if t(k) < disturbance_time && t(k+1) >= disturbance_time
                h2(k+1) = max(0, min(h2(k+1) + disturbance_jump_h2, H_max));
                disturbance_applied = true;
            end
        end
    end

    u(end) = u(end-1);

    if use_noise
        h1_meas(end) = max(0, min(h1(end) + noise_h1_current, H_max));
        h2_meas(end) = max(0, min(h2(end) + noise_h2_current, H_max));
    else
        h1_meas(end) = h1(end);
        h2_meas(end) = h2(end);
    end

    res.h1 = h1;
    res.h2 = h2;
    res.u = u;
    res.h1_meas = h1_meas;
    res.h2_meas = h2_meas;
end

%% ============================================================
% LOCAL FUNCTION: compute metrics
% ============================================================

function metrics = compute_metrics(t, h_ref, h2, u)

    e = h_ref - h2;

    metrics.RMSE = sqrt(mean(e.^2));
    metrics.MAE  = mean(abs(e));
    metrics.IAE  = trapz(t, abs(e));
    metrics.ISE  = trapz(t, e.^2);

    metrics.FinalError = abs(h_ref(end) - h2(end));
    metrics.MaxH2 = max(h2);
    metrics.MinH2 = min(h2);
    metrics.MaxU = max(u);
    metrics.MinU = min(u);
    metrics.StdU = std(u);
    metrics.TVu = sum(abs(diff(u)));

    dt = mean(diff(t));
    du = [0, diff(u) ./ dt];
    metrics.MaxAbsDu = max(abs(du));

    % 2% settling time for final reference
    href_final = h_ref(end);
    tol = 0.02 * max(href_final, 1e-9);

    outside_idx = find(abs(e) > tol);

    if isempty(outside_idx)
        metrics.Settling2 = 0;
    elseif outside_idx(end) == numel(t)
        metrics.Settling2 = NaN;
    else
        metrics.Settling2 = t(outside_idx(end) + 1);
    end
end

%% ============================================================
% LOCAL FUNCTION: compute disturbance recovery time
% ============================================================

function recovery_time = compute_recovery_time(t, h_ref, h2, disturbance_time, settling_percent)

    idx_after = find(t >= disturbance_time);

    if isempty(idx_after)
        recovery_time = NaN;
        return;
    end

    t_after = t(idx_after);
    h2_after = h2(idx_after);
    h_ref_after = h_ref(idx_after);

    href_val = h_ref_after(end);
    tol = settling_percent * max(href_val, 1e-9);

    e_after = abs(h_ref_after - h2_after);

    outside_idx = find(e_after > tol);

    if isempty(outside_idx)
        recovery_time = 0;
    elseif outside_idx(end) == numel(t_after)
        recovery_time = NaN;
    else
        recovery_time = t_after(outside_idx(end)+1) - disturbance_time;
    end
end

%% ============================================================
% LOCAL FUNCTION: print metric table
% ============================================================

function print_metric_table(title_text, m)

    fprintf('\n================ %s ================\n', char(title_text));
    fprintf('RMSE h2 [m]             = %.6f\n', m.RMSE);
    fprintf('MAE h2 [m]              = %.6f\n', m.MAE);
    fprintf('IAE                     = %.6f\n', m.IAE);
    fprintf('ISE                     = %.6f\n', m.ISE);
    fprintf('Final error [m]         = %.6f\n', m.FinalError);
    fprintf('Max h2 [m]              = %.6f\n', m.MaxH2);
    fprintf('Min h2 [m]              = %.6f\n', m.MinH2);
    fprintf('Max u [V]               = %.6f\n', m.MaxU);
    fprintf('Min u [V]               = %.6f\n', m.MinU);
    fprintf('Std u                   = %.6f\n', m.StdU);
    fprintf('Total variation u       = %.6f\n', m.TVu);
    fprintf('Max |du/dt|             = %.6f\n', m.MaxAbsDu);
    fprintf('Settling time 2%% [s]    = %.6f\n', m.Settling2);

    if isfield(m, 'RecoveryTime2')
        fprintf('Recovery time 2%% [s]    = %.6f\n', m.RecoveryTime2);
    end

    fprintf('============================================================\n');
end

%% ============================================================
% LOCAL FUNCTION: custom staircase segment metrics
% ============================================================

function print_custom_staircase_segment_metrics(t, h_ref, h2)

    segments = [
        0,   80,  0.02;
        80,  160, 0.10;
        160, 240, 0.20;
        240, 320, 0.28;
        320, 400, 0.15;
        400, 480, 0.05
    ];

    fprintf('\n================ CUSTOM STAIRCASE SEGMENT METRICS ================\n');
    fprintf('%8s %8s %8s | %12s %12s %12s\n', ...
        't0', 'tf', 'href', 'final_h2', 'final_err', 'settle2');

    for i = 1:size(segments, 1)

        t0 = segments(i, 1);
        tf = segments(i, 2);
        href_i = segments(i, 3);

        idx = find(t >= t0 & t < tf);

        if isempty(idx)
            continue;
        end

        t_seg = t(idx);
        h2_seg = h2(idx);
        e_seg = href_i - h2_seg;

        final_h2 = h2_seg(end);
        final_err = abs(href_i - final_h2);

        tol = 0.02 * max(href_i, 1e-9);
        outside_idx = find(abs(e_seg) > tol);

        if isempty(outside_idx)
            settle2 = 0;
        elseif outside_idx(end) == numel(t_seg)
            settle2 = NaN;
        else
            settle2 = t_seg(outside_idx(end)+1) - t0;
        end

        fprintf('%8.1f %8.1f %8.3f | %12.6f %12.6f %12.2f\n', ...
            t0, tf, href_i, final_h2, final_err, settle2);
    end

    fprintf('=================================================================\n');
end

%% ============================================================
% LOCAL FUNCTION: main result plot
% ============================================================

function plot_single_result(t, h_ref, res, fig_title, H_max, V_max, show_disturbance, disturbance_time)

    fig_title_char = char(fig_title);

    e = h_ref - res.h2;

    dt = mean(diff(t));
    du = [0, diff(res.u) ./ dt];

    figure('Name', fig_title_char, ...
           'Color', 'w', ...
           'Position', [80, 80, 1150, 820]);

    tiledlayout(4,1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    stairs(t, h_ref, 'k--', 'LineWidth', 2.2); hold on;
    plot(t, res.h2, 'b', 'LineWidth', 2.0);

    if show_disturbance
        xline(disturbance_time, 'r--', 'Disturbance +0.03 m', ...
            'LineWidth', 1.4, 'LabelVerticalAlignment', 'bottom');
    end

    grid on; box on;
    ylabel('h_2 (m)');
    title(fig_title_char);
    legend('Reference h_{ref}', 'Tank 2 h_2', 'Location', 'best');

    nexttile;
    plot(t, res.h1, 'r--', 'LineWidth', 1.8); hold on;
    yline(H_max, 'k:', 'H_{max}=0.6 m', 'LineWidth', 1.2);

    if show_disturbance
        xline(disturbance_time, 'r--', 'Disturbance +0.03 m', ...
            'LineWidth', 1.4, 'LabelVerticalAlignment', 'bottom');
    end

    grid on; box on;
    ylabel('h_1 (m)');
    title('Tank 1 Level');

    nexttile;
    plot(t, e, 'Color', [0.10 0.45 0.10], 'LineWidth', 1.6); hold on;
    yline(0, 'k--', 'LineWidth', 1.0);

    if show_disturbance
        xline(disturbance_time, 'r--', 'Disturbance +0.03 m', ...
            'LineWidth', 1.4, 'LabelVerticalAlignment', 'bottom');
    end

    grid on; box on;
    ylabel('Error (m)');
    title('Tracking Error e(t)=h_{ref}(t)-h_2(t)');

    nexttile;
    plot(t, res.u, 'm', 'LineWidth', 1.6); hold on;
    yline(V_max, 'k:', '10 V saturation');
    yline(0, 'k:', '0 V');

    if show_disturbance
        xline(disturbance_time, 'r--', 'Disturbance +0.03 m', ...
            'LineWidth', 1.4, 'LabelVerticalAlignment', 'bottom');
    end

    grid on; box on;
    xlabel('Time (s)');
    ylabel('u_1 (V)');
    ylim([-0.5, 10.5]);
    title('NN Control Input');

    figure('Name', [fig_title_char, ' - Control Variation'], ...
           'Color', 'w', ...
           'Position', [120, 120, 1000, 480]);

    plot(t, abs(du), 'Color', [0.35 0.35 0.35], 'LineWidth', 1.3);

    if show_disturbance
        xline(disturbance_time, 'r--', 'Disturbance +0.03 m', ...
            'LineWidth', 1.4, 'LabelVerticalAlignment', 'bottom');
    end

    grid on; box on;
    xlabel('Time (s)');
    ylabel('|du/dt| (V/s)');
    title(['Control Signal Variation: ', fig_title_char]);
end

%% ============================================================
% LOCAL FUNCTION: h2-only plot
% ============================================================

function plot_h2_only(t, h_ref, h2, fig_title)

    figure('Name', char(fig_title), ...
           'Color', 'w', ...
           'Position', [100, 100, 1200, 550]);

    stairs(t, h_ref, 'b', 'LineWidth', 2.0); hold on;
    plot(t, h2, 'r', 'LineWidth', 2.0);

    grid on; box on;
    xlabel('Time');
    ylabel('Liquid Level');
    title(char(fig_title));
    legend('Step Input', 'h_2 Response', 'Location', 'best');
    ylim([0, 0.31]);
end

%% ============================================================
% LOCAL FUNCTION: h2-only plot with disturbance marker
% ============================================================

function plot_h2_only_with_disturbance(t, h_ref, h2, fig_title, disturbance_time)

    figure('Name', char(fig_title), ...
           'Color', 'w', ...
           'Position', [100, 100, 1200, 550]);

    stairs(t, h_ref, 'b', 'LineWidth', 2.0); hold on;
    plot(t, h2, 'r', 'LineWidth', 2.0);

    xline(disturbance_time, 'k--', 'Disturbance +0.03 m', ...
        'LineWidth', 1.4, 'LabelVerticalAlignment', 'bottom');

    grid on; box on;
    xlabel('Time');
    ylabel('Liquid Level');
    title(char(fig_title));
    legend('Step Input', 'h_2 Response', 'Disturbance', 'Location', 'best');
    ylim([0, 0.31]);
end

%% ============================================================
% LOCAL FUNCTION: noise measurement plot
% ============================================================

function plot_noise_measurements(t, res, fig_title)

    figure('Name', char(fig_title), ...
           'Color', 'w', ...
           'Position', [100, 100, 1150, 650]);

    tiledlayout(2,1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, res.h1, 'r', 'LineWidth', 2.0); hold on;
    plot(t, res.h1_meas, 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
    grid on; box on;
    ylabel('h_1 (m)');
    title('Tank 1: True and Noisy Measurement');
    legend('True h_1', 'Measured h_1 with noise', 'Location', 'best');

    nexttile;
    plot(t, res.h2, 'b', 'LineWidth', 2.0); hold on;
    plot(t, res.h2_meas, 'Color', [0.55 0.55 0.55], 'LineWidth', 0.9);
    grid on; box on;
    xlabel('Time (s)');
    ylabel('h_2 (m)');
    title('Tank 2: True and Noisy Measurement');
    legend('True h_2', 'Measured h_2 with noise', 'Location', 'best');
end