%% ============================================================
%  FILE: test_noise_rejection_plots.m
%  PURPOSE:
%     Post-training noise rejection test for the NN controller.
%     Compares:
%       Case 1: raw noisy measurements used directly by NN controller
%       Case 2: low-pass filtered measurements used by NN controller
%
%  REQUIREMENTS:
%     A trained neural controller named "net" must exist in workspace,
%     or a file named "trained_NN_controller.mat" must exist and contain net.
%
%  No external disturbance is applied.
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
% 1) Plant and physical parameters
% ============================================================

c1 = 1.035454e-2;
c2 = 1.143e-3;

H_max = 0.6;
V_max = 10;

%% ============================================================
% 2) Simulation settings
% ============================================================

dt = 0.1;
t_end = 500;
t = 0:dt:t_end;
N = numel(t);

h_ref = 0.25;      % feasible reference, comfortably below 0.3 m

%% ============================================================
% 3) Noise and filter settings
% ============================================================

rng(42);           % repeatable noise realization

noise_std_h1 = 0.002;    % 2 mm standard deviation
noise_std_h2 = 0.002;    % 2 mm standard deviation

alpha = 0.90;            % first-order low-pass filter coefficient

%% Pre-generate identical noise for fair comparison
noise_h1 = noise_std_h1 * randn(1, N);
noise_h2 = noise_std_h2 * randn(1, N);

%% ============================================================
% 4) CASE 1: Raw noisy measurement controller
% ============================================================

h1_raw = zeros(1, N);
h2_raw = zeros(1, N);
u_raw  = zeros(1, N);

h1_meas_raw = zeros(1, N);
h2_meas_raw = zeros(1, N);

h1_raw(1) = 0.0;
h2_raw(1) = 0.0;

for k = 1:N-1

    % Noisy measurements
    h1_m = h1_raw(k) + noise_h1(k);
    h2_m = h2_raw(k) + noise_h2(k);

    h1_m = max(0, min(h1_m, H_max));
    h2_m = max(0, min(h2_m, H_max));

    h1_meas_raw(k) = h1_m;
    h2_meas_raw(k) = h2_m;

    % NN controller using raw noisy measurements
    u_nn = net([h1_m; h2_m; h_ref]);
    u_raw(k) = max(0, min(u_nn, V_max));

    % True plant
    diff_h = h1_raw(k) - h2_raw(k);
    sign_val = sign(diff_h);
    sqrt_diff = sqrt(abs(diff_h));

    dh1 = -c1 * sign_val * sqrt_diff + c2 * u_raw(k);
    dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(max(h2_raw(k), 0));

    h1_raw(k+1) = h1_raw(k) + dt * dh1;
    h2_raw(k+1) = h2_raw(k) + dt * dh2;

    h1_raw(k+1) = max(0, min(h1_raw(k+1), H_max));
    h2_raw(k+1) = max(0, min(h2_raw(k+1), H_max));
end

u_raw(end) = u_raw(end-1);
h1_meas_raw(end) = max(0, min(h1_raw(end) + noise_h1(end), H_max));
h2_meas_raw(end) = max(0, min(h2_raw(end) + noise_h2(end), H_max));

%% ============================================================
% 5) CASE 2: Filtered measurement controller
% ============================================================

h1_filt = zeros(1, N);
h2_filt = zeros(1, N);
u_filt  = zeros(1, N);

h1_meas_filt = zeros(1, N);
h2_meas_filt = zeros(1, N);

h1_hat = zeros(1, N);
h2_hat = zeros(1, N);

h1_filt(1) = 0.0;
h2_filt(1) = 0.0;

h1_hat(1) = h1_filt(1);
h2_hat(1) = h2_filt(1);

for k = 1:N-1

    % Noisy measurements, same noise sequence as raw case
    h1_m = h1_filt(k) + noise_h1(k);
    h2_m = h2_filt(k) + noise_h2(k);

    h1_m = max(0, min(h1_m, H_max));
    h2_m = max(0, min(h2_m, H_max));

    h1_meas_filt(k) = h1_m;
    h2_meas_filt(k) = h2_m;

    % Low-pass filtering
    if k == 1
        h1_hat(k) = h1_m;
        h2_hat(k) = h2_m;
    else
        h1_hat(k) = alpha * h1_hat(k-1) + (1-alpha) * h1_m;
        h2_hat(k) = alpha * h2_hat(k-1) + (1-alpha) * h2_m;
    end

    % NN controller using filtered states
    u_nn = net([h1_hat(k); h2_hat(k); h_ref]);
    u_filt(k) = max(0, min(u_nn, V_max));

    % True plant
    diff_h = h1_filt(k) - h2_filt(k);
    sign_val = sign(diff_h);
    sqrt_diff = sqrt(abs(diff_h));

    dh1 = -c1 * sign_val * sqrt_diff + c2 * u_filt(k);
    dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(max(h2_filt(k), 0));

    h1_filt(k+1) = h1_filt(k) + dt * dh1;
    h2_filt(k+1) = h2_filt(k) + dt * dh2;

    h1_filt(k+1) = max(0, min(h1_filt(k+1), H_max));
    h2_filt(k+1) = max(0, min(h2_filt(k+1), H_max));
end

u_filt(end) = u_filt(end-1);
h1_hat(end) = h1_hat(end-1);
h2_hat(end) = h2_hat(end-1);
h1_meas_filt(end) = max(0, min(h1_filt(end) + noise_h1(end), H_max));
h2_meas_filt(end) = max(0, min(h2_filt(end) + noise_h2(end), H_max));

%% ============================================================
% 6) Metrics
% ============================================================

e_raw  = h_ref - h2_raw;
e_filt = h_ref - h2_filt;

RMSE_raw  = sqrt(mean(e_raw.^2));
RMSE_filt = sqrt(mean(e_filt.^2));

MAE_raw  = mean(abs(e_raw));
MAE_filt = mean(abs(e_filt));

IAE_raw  = trapz(t, abs(e_raw));
IAE_filt = trapz(t, abs(e_filt));

ISE_raw  = trapz(t, e_raw.^2);
ISE_filt = trapz(t, e_filt.^2);

du_raw  = [0, diff(u_raw)/dt];
du_filt = [0, diff(u_filt)/dt];

TVu_raw  = sum(abs(diff(u_raw)));
TVu_filt = sum(abs(diff(u_filt)));

std_u_raw  = std(u_raw);
std_u_filt = std(u_filt);

final_err_raw  = abs(h_ref - h2_raw(end));
final_err_filt = abs(h_ref - h2_filt(end));

reduction_TVu = 100 * (TVu_raw - TVu_filt) / TVu_raw;

fprintf('\n================ NOISE REJECTION TEST RESULTS ================\n');
fprintf('Reference h_ref              = %.3f m\n', h_ref);
fprintf('Noise std h1,h2              = %.4f m, %.4f m\n', noise_std_h1, noise_std_h2);
fprintf('Filter alpha                 = %.2f\n', alpha);
fprintf('--------------------------------------------------------------\n');
fprintf('%28s %14s %14s\n', 'Metric', 'Raw noisy', 'Filtered');
fprintf('%28s %14.6f %14.6f\n', 'RMSE h2', RMSE_raw, RMSE_filt);
fprintf('%28s %14.6f %14.6f\n', 'MAE h2', MAE_raw, MAE_filt);
fprintf('%28s %14.6f %14.6f\n', 'IAE', IAE_raw, IAE_filt);
fprintf('%28s %14.6f %14.6f\n', 'ISE', ISE_raw, ISE_filt);
fprintf('%28s %14.6f %14.6f\n', 'Final error', final_err_raw, final_err_filt);
fprintf('%28s %14.6f %14.6f\n', 'Max control u', max(u_raw), max(u_filt));
fprintf('%28s %14.6f %14.6f\n', 'Std control u', std_u_raw, std_u_filt);
fprintf('%28s %14.6f %14.6f\n', 'Total variation u', TVu_raw, TVu_filt);
fprintf('%28s %13.2f%%\n', 'TVu reduction', reduction_TVu);
fprintf('==============================================================\n');

%% Steady-state noise metrics after transient
t_ss_start = 250;
idx_ss = find(t >= t_ss_start);

RMSE_raw_ss  = sqrt(mean(e_raw(idx_ss).^2));
RMSE_filt_ss = sqrt(mean(e_filt(idx_ss).^2));

std_h2_raw_ss  = std(h2_raw(idx_ss));
std_h2_filt_ss = std(h2_filt(idx_ss));

std_u_raw_ss  = std(u_raw(idx_ss));
std_u_filt_ss = std(u_filt(idx_ss));

TVu_raw_ss  = sum(abs(diff(u_raw(idx_ss))));
TVu_filt_ss = sum(abs(diff(u_filt(idx_ss))));

fprintf('\n========== STEADY-STATE NOISE REJECTION METRICS ==========\n');
fprintf('Steady-state interval starts at t = %.1f s\n', t_ss_start);
fprintf('%28s %14s %14s\n', 'Metric', 'Raw noisy', 'Filtered');
fprintf('%28s %14.8f %14.8f\n', 'RMSE h2 ss', RMSE_raw_ss, RMSE_filt_ss);
fprintf('%28s %14.8f %14.8f\n', 'Std h2 ss', std_h2_raw_ss, std_h2_filt_ss);
fprintf('%28s %14.8f %14.8f\n', 'Std control u ss', std_u_raw_ss, std_u_filt_ss);
fprintf('%28s %14.8f %14.8f\n', 'TV control u ss', TVu_raw_ss, TVu_filt_ss);
fprintf('==========================================================\n');

%% ============================================================
% 7) Plot settings
% ============================================================

out_dir = "figures_noise_rejection";
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultLineLineWidth', 1.8);

%% ============================================================
% 8) Figure 1: Tracking comparison
% ============================================================

fig1 = figure('Name', 'Noise Rejection - Tracking Comparison', ...
              'Color', 'w', ...
              'Position', [80, 80, 1100, 560]);

plot(t, h_ref*ones(size(t)), 'k--', 'LineWidth', 2.3); hold on;
plot(t, h2_raw, 'b', 'LineWidth', 2.0);
plot(t, h2_filt, 'r', 'LineWidth', 2.0);
grid on; box on;
xlabel('Time (s)');
ylabel('Tank 2 Level h_2 (m)');
title('Tracking Under 2 mm Sensor Noise');
legend('Reference', 'Raw noisy measurement control', ...
       'Filtered measurement control', 'Location', 'best');
ylim([0, max(0.30, max([h2_raw h2_filt])+0.02)]);

exportgraphics(fig1, fullfile(out_dir, 'noise_tracking_comparison.png'), 'Resolution', 300);

%% ============================================================
% 9) Figure 2: Noisy measurement and filtered estimate
% ============================================================

fig2 = figure('Name', 'Noise Rejection - Measurement Filtering', ...
              'Color', 'w', ...
              'Position', [100, 100, 1100, 650]);

tl = tiledlayout(2,1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, h2_raw, 'b', 'LineWidth', 2.0); hold on;
plot(t, h2_meas_raw, 'Color', [0.55 0.55 0.55], 'LineWidth', 0.8);
grid on; box on;
ylabel('h_2 (m)');
title('Raw Case: True h_2 and Noisy h_2 Measurement');
legend('True h_2', 'Noisy measurement', 'Location', 'best');

nexttile;
plot(t, h2_filt, 'r', 'LineWidth', 2.0); hold on;
plot(t, h2_meas_filt, 'Color', [0.65 0.65 0.65], 'LineWidth', 0.8);
plot(t, h2_hat, 'k--', 'LineWidth', 1.6);
grid on; box on;
xlabel('Time (s)');
ylabel('h_2 (m)');
title('Filtered Case: True, Noisy, and Filtered h_2');
legend('True h_2', 'Noisy measurement', 'Filtered estimate \hat{h}_2', ...
       'Location', 'best');

exportgraphics(fig2, fullfile(out_dir, 'noise_measurement_filtering.png'), 'Resolution', 300);

%% ============================================================
% 10) Figure 3: Control signal comparison
% ============================================================

fig3 = figure('Name', 'Noise Rejection - Control Signal', ...
              'Color', 'w', ...
              'Position', [120, 120, 1100, 560]);

plot(t, u_raw, 'b', 'LineWidth', 1.4); hold on;
plot(t, u_filt, 'r', 'LineWidth', 1.8);
yline(V_max, 'k:', '10 V saturation', 'LineWidth', 1.4);
grid on; box on;
xlabel('Time (s)');
ylabel('Pump Voltage u_1 (V)');
title('Control Signal Under Measurement Noise');
legend('Raw noisy measurement control', 'Filtered measurement control', ...
       'Location', 'best');
ylim([-0.5 10.5]);

exportgraphics(fig3, fullfile(out_dir, 'noise_control_signal.png'), 'Resolution', 300);

%% ============================================================
% 11) Figure 4: Control variation |du/dt|
% ============================================================

fig4 = figure('Name', 'Noise Rejection - Control Variation', ...
              'Color', 'w', ...
              'Position', [140, 140, 1100, 560]);

plot(t, abs(du_raw), 'b', 'LineWidth', 1.0); hold on;
plot(t, abs(du_filt), 'r', 'LineWidth', 1.2);
grid on; box on;
xlabel('Time (s)');
ylabel('|du/dt| (V/s)');
title('Control Signal Variation Under Measurement Noise');
legend('Raw noisy measurement control', 'Filtered measurement control', ...
       'Location', 'best');

exportgraphics(fig4, fullfile(out_dir, 'noise_control_variation.png'), 'Resolution', 300);

%% ============================================================
% 12) Figure 5: Metric comparison bar chart
% ============================================================

fig5 = figure('Name', 'Noise Rejection - Metric Comparison', ...
              'Color', 'w', ...
              'Position', [160, 160, 900, 560]);

metric_names = categorical({'Std(u)', 'TV(u)/1000', 'RMSE h_2', 'Final Error'});
metric_names = reordercats(metric_names, {'Std(u)', 'TV(u)/1000', 'RMSE h_2', 'Final Error'});

raw_vals = [std_u_raw, TVu_raw/1000, RMSE_raw, final_err_raw];
filt_vals = [std_u_filt, TVu_filt/1000, RMSE_filt, final_err_filt];

bar(metric_names, [raw_vals(:), filt_vals(:)]);
grid on; box on;
ylabel('Metric Value');
title('Noise Rejection Metrics');
legend('Raw noisy', 'Filtered', 'Location', 'best');

exportgraphics(fig5, fullfile(out_dir, 'noise_metric_comparison.png'), 'Resolution', 300);

%% ============================================================
% 13) Save simulation data
% ============================================================

save('noise_rejection_results.mat', ...
     't', 'h_ref', ...
     'h1_raw', 'h2_raw', 'u_raw', 'h1_meas_raw', 'h2_meas_raw', ...
     'h1_filt', 'h2_filt', 'u_filt', 'h1_meas_filt', 'h2_meas_filt', ...
     'h1_hat', 'h2_hat', ...
     'noise_std_h1', 'noise_std_h2', 'alpha', ...
     'RMSE_raw', 'RMSE_filt', 'MAE_raw', 'MAE_filt', ...
     'IAE_raw', 'IAE_filt', 'ISE_raw', 'ISE_filt', ...
     'std_u_raw', 'std_u_filt', 'TVu_raw', 'TVu_filt', ...
     'RMSE_raw_ss', 'RMSE_filt_ss', ...
     'std_h2_raw_ss', 'std_h2_filt_ss', ...
     'std_u_raw_ss', 'std_u_filt_ss', ...
     'TVu_raw_ss', 'TVu_filt_ss');

disp('Noise rejection test completed.');
disp(['Figures saved to folder: ', char(out_dir)]);
disp('Simulation data saved to: noise_rejection_results.mat');
