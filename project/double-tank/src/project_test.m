%% ============================================================
%  FILE: test_time_varying_step_plots.m
%  PURPOSE:
%     Post-training time-varying step reference test for the NN controller.
%     No measurement noise, no disturbance.
%
%  REQUIREMENTS:
%     A trained neural controller named "net" must exist in workspace,
%     or a file named "trained_NN_controller.mat" must exist and contain net.
%
%  Controller input:
%     [h1; h2; h_ref]
%
%  Controller output:
%     u1 pump voltage [V]
%
%  Physical limits:
%     h1,h2 in [0, 0.6] m
%     u1 in [0, 10] V
%     practical/mathematical h2 reference limit: 0.3 m
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

H_max = 0.6;       % tank height limit [m]
V_max = 10;        % pump voltage limit [V]

h2_ref_max = 0.3;  % practical steady-state h2 limit from h1*=2h2*, h1<=0.6

%% ============================================================
% 2) Simulation settings
% ============================================================

dt = 0.1;
t_end = 800;
t = 0:dt:t_end;
N = numel(t);

%% ============================================================
% 3) Time-varying feasible step reference schedule
%    All values are <= 0.3 m.
%    The sequence includes upward and downward steps.
% ============================================================

segments = [
    0,   120, 0.08;
    120, 250, 0.18;
    250, 420, 0.28;
    420, 640, 0.12;
    640, 800, 0.25;
   
];
h_ref = zeros(1, N);

for i = 1:size(segments, 1)
    idx = t >= segments(i,1) & t < segments(i,2);
    h_ref(idx) = segments(i,3);
end

h_ref(t >= segments(end,1)) = segments(end,3);

% Safety clipping
h_ref = max(0, min(h_ref, h2_ref_max));

%% ============================================================
% 4) Closed-loop simulation
% ============================================================

h1 = zeros(1, N);
h2 = zeros(1, N);
u  = zeros(1, N);

% Initial levels
h1(1) = 0.0;
h2(1) = 0.0;

for k = 1:N-1

    % Current reference
    href_k = h_ref(k);

    % NN controller
    u_nn = net([h1(k); h2(k); href_k]);

    % Saturation
    u(k) = max(0, min(u_nn, V_max));

    % Nonlinear coupled-tank plant
    diff_h = h1(k) - h2(k);
    sign_val = sign(diff_h);
    sqrt_diff = sqrt(abs(diff_h));

    dh1 = -c1 * sign_val * sqrt_diff + c2 * u(k);
    dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(max(h2(k), 0));

    % Euler integration
    h1(k+1) = h1(k) + dt * dh1;
    h2(k+1) = h2(k) + dt * dh2;

    % Physical clipping
    h1(k+1) = max(0, min(h1(k+1), H_max));
    h2(k+1) = max(0, min(h2(k+1), H_max));
end

u(end) = u(end-1);

%% ============================================================
% 5) Performance metrics
% ============================================================

e = h_ref - h2;

RMSE = sqrt(mean(e.^2));
MAE  = mean(abs(e));
IAE  = trapz(t, abs(e));
ISE  = trapz(t, e.^2);

du = [0, diff(u)/dt];
TVu = sum(abs(diff(u)));

fprintf('\n================ TIME-VARYING STEP TEST ================\n');
fprintf('Global RMSE              = %.6f m\n', RMSE);
fprintf('Global MAE               = %.6f m\n', MAE);
fprintf('Global IAE               = %.6f\n', IAE);
fprintf('Global ISE               = %.6f\n', ISE);
fprintf('Maximum h1               = %.6f m\n', max(h1));
fprintf('Maximum h2               = %.6f m\n', max(h2));
fprintf('Maximum control u        = %.6f V\n', max(u));
fprintf('Minimum control u        = %.6f V\n', min(u));
fprintf('Total variation of u     = %.6f\n', TVu);
fprintf('h2 reference limit       = %.3f m\n', h2_ref_max);
fprintf('=========================================================\n');

%% Segment-wise metrics, including settling time
settling_percent = 0.02;

fprintf('\n================ SEGMENT-WISE PERFORMANCE ================\n');
fprintf('%10s %10s %10s %14s %14s %14s %14s %14s\n', ...
    't_start', 't_end', 'href', 'final_h2', 'final_error', ...
    'settle_abs', 'IAE_seg', 'max_u');

segment_results = zeros(size(segments,1), 7);

for i = 1:size(segments, 1)

    t_start = segments(i, 1);
    t_stop  = segments(i, 2);
    href_i  = segments(i, 3);

    idx = find(t >= t_start & t < t_stop);

    if isempty(idx)
        continue;
    end

    t_seg = t(idx);
    h2_seg = h2(idx);
    u_seg = u(idx);
    e_seg = href_i - h2_seg;

    final_h2_seg = h2_seg(end);
    final_error_seg = abs(href_i - final_h2_seg);
    IAE_seg = trapz(t_seg, abs(e_seg));
    max_u_seg = max(u_seg);

    % Settling time inside the segment
    tol = settling_percent * max(href_i, 1e-9);
    outside_idx = find(abs(e_seg) > tol);

    if isempty(outside_idx)
        settle_abs = 0;
    elseif outside_idx(end) == numel(t_seg)
        settle_abs = NaN;
    else
        settle_abs = t_seg(outside_idx(end)+1) - t_start;
    end

    segment_results(i,:) = [t_start, t_stop, href_i, final_h2_seg, ...
                            final_error_seg, settle_abs, max_u_seg];

    fprintf('%10.1f %10.1f %10.3f %14.6f %14.6f %14.2f %14.6f %14.6f\n', ...
        t_start, t_stop, href_i, final_h2_seg, final_error_seg, ...
        settle_abs, IAE_seg, max_u_seg);
end

fprintf('==========================================================\n');

%% ============================================================
% 6) Plot settings
% ============================================================

out_dir = "figures_step_test";
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultLineLineWidth', 1.8);

%% ============================================================
% 7) Figure 1: Main closed-loop response
% ============================================================

fig1 = figure('Name', 'Time-Varying Step Response - Main', ...
              'Color', 'w', ...
              'Position', [80, 80, 1100, 780]);

tl = tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
stairs(t, h_ref, 'k--', 'LineWidth', 2.2); hold on;
plot(t, h2, 'b', 'LineWidth', 2.0);
plot(t, h1, 'r--', 'LineWidth', 1.5);
yline(h2_ref_max, 'm:', 'h_2 feasible limit = 0.3 m', ...
    'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left');
yline(H_max, 'k:', 'H_{max}=0.6 m', ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment', 'left');
grid on; box on;
ylabel('Level (m)');
title('Closed-Loop Response to Time-Varying Step References');
legend('Reference h_{ref}', 'Tank 2 h_2', 'Tank 1 h_1', ...
       'Location', 'best');

nexttile;
plot(t, e, 'Color', [0.10 0.45 0.10], 'LineWidth', 1.6);
yline(0, 'k--', 'LineWidth', 1.0);
grid on; box on;
ylabel('Tracking Error (m)');
title('Tracking Error e(t)=h_{ref}(t)-h_2(t)');

nexttile;
plot(t, u, 'm', 'LineWidth', 1.7); hold on;
yline(V_max, 'k:', 'u_{max}=10 V', 'LineWidth', 1.3);
yline(0, 'k:', 'u_{min}=0 V', 'LineWidth', 1.3);
grid on; box on;
xlabel('Time (s)');
ylabel('Pump Voltage (V)');
ylim([-0.5 10.5]);
title('NN Controller Output');

exportgraphics(fig1, fullfile(out_dir, 'step_test_main_response.png'), 'Resolution', 300);

%% ============================================================
% 8) Figure 2: Cleaner output-only tracking plot
% ============================================================

fig2 = figure('Name', 'Time-Varying Step Tracking - h2 Only', ...
              'Color', 'w', ...
              'Position', [120, 120, 1100, 520]);

stairs(t, h_ref, 'k--', 'LineWidth', 2.4); hold on;
plot(t, h2, 'b', 'LineWidth', 2.2);
yline(h2_ref_max, 'r:', '0.3 m feasible limit', ...
    'LineWidth', 1.8, 'LabelHorizontalAlignment', 'left');
for i = 2:size(segments, 1)
    xline(segments(i,1), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);
end
grid on; box on;
xlabel('Time (s)');
ylabel('Tank 2 Level h_2 (m)');
title('Tank 2 Tracking for Feasible Time-Varying Step Inputs');
legend('Reference h_{ref}', 'Tank 2 h_2', 'Location', 'best');
ylim([0, 0.34]);

exportgraphics(fig2, fullfile(out_dir, 'step_test_h2_tracking.png'), 'Resolution', 300);

%% ============================================================
% 9) Figure 3: Control effort and rate
% ============================================================

fig3 = figure('Name', 'Control Effort and Control Rate', ...
              'Color', 'w', ...
              'Position', [150, 150, 1100, 650]);

tl2 = tiledlayout(2,1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, u, 'm', 'LineWidth', 1.8); hold on;
yline(V_max, 'k:', '10 V saturation', 'LineWidth', 1.4);
grid on; box on;
ylabel('u_1 (V)');
title('Control Effort');
ylim([-0.5 10.5]);

nexttile;
plot(t, abs(du), 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
grid on; box on;
xlabel('Time (s)');
ylabel('|du/dt| (V/s)');
title('Control Signal Variation');

exportgraphics(fig3, fullfile(out_dir, 'step_test_control_effort.png'), 'Resolution', 300);

%% ============================================================
% 10) Figure 4: Phase relation h1 vs h2
% ============================================================

fig4 = figure('Name', 'State Trajectory h1-h2', ...
              'Color', 'w', ...
              'Position', [180, 180, 700, 620]);

plot(h2, h1, 'b', 'LineWidth', 2.0); hold on;
plot([0 h2_ref_max], [0 2*h2_ref_max], 'k--', 'LineWidth', 1.8);
xline(h2_ref_max, 'r:', 'h_2=0.3 m', 'LineWidth', 1.4);
yline(H_max, 'r:', 'h_1=0.6 m', 'LineWidth', 1.4);
grid on; box on;
xlabel('h_2 (m)');
ylabel('h_1 (m)');
title('State Trajectory and Steady-State Relation h_1=2h_2');
legend('Closed-loop trajectory', 'Ideal steady-state line h_1=2h_2', ...
       'Location', 'best');
xlim([0, 0.34]);
ylim([0, 0.64]);

exportgraphics(fig4, fullfile(out_dir, 'step_test_state_trajectory.png'), 'Resolution', 300);

%% ============================================================
% 11) Save simulation data
% ============================================================

save('step_test_results.mat', ...
     't', 'h1', 'h2', 'h_ref', 'u', 'e', ...
     'segments', 'segment_results', ...
     'RMSE', 'MAE', 'IAE', 'ISE', 'TVu');

disp('Step test completed.');
disp(['Figures saved to folder: ', char(out_dir)]);
disp('Simulation data saved to: step_test_results.mat');
