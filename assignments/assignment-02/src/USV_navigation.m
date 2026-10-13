% =========================================================================
% KON426E - Intelligent Control Systems
% Assignment 2 - Question 3 Skeleton Code: USV Waypoint Navigation
%
% Skeleton Author: Res. Asst. Murat Furkan Mansur
% Version 2: 01.05.2026
% =========================================================================

clear; clc; close all;

%% 1. Simulation Setup

% Define a path of waypoints [x, y].
base_WPs = [ 0,  0;
             8,  6;
             8, 12;
            -2, 15;
            -5,  5];

num_WPs = size(base_WPs, 1);
wp_idx = 2;

% Waypoint scaling and acceptance radius.
wp_scale = 0.35;
WPs = base_WPs * wp_scale;

acceptance_radius = 0.15; % meters

% Initial conditions.
x = WPs(1, 1);
y = WPs(1, 2);
theta = pi/4; % Initial heading angle in radians.

% Simulation parameters.
dt = 0.1;
T_max = 100;
N = round(T_max / dt);

% Controller parameter.
Kh = 1.0; % 

% History arrays for plotting.
x_hist = zeros(1, N);
y_hist = zeros(1, N);
theta_hist = zeros(1, N);
v_hist = zeros(1, N);
omega_hist = zeros(1, N);
d_hist = zeros(1, N);
err_hist = zeros(1, N);
wp_idx_hist = zeros(1, N);

%% 2. Load Fuzzy Inference System

% Uncomment this line after designing and saving your FIS.
% The expected input order is:
%   Input 1: Distance to waypoint, d
%   Input 2: Absolute heading error, |delta_theta|
%
fis = readfis('USV_Speed_Controller.fis');

%% 3. Simulation Loop

for k = 1:N

    % Get current target waypoint.
    target_x = WPs(wp_idx, 1);
    target_y = WPs(wp_idx, 2);

    % Calculate distance to target waypoint.
    dx = target_x - x;
    dy = target_y - y;
    d = sqrt(dx^2 + dy^2);

    % Check if waypoint is reached.
    if d < acceptance_radius
        if wp_idx < num_WPs
            wp_idx = wp_idx + 1;

            target_x = WPs(wp_idx, 1);
            target_y = WPs(wp_idx, 2);

            dx = target_x - x;
            dy = target_y - y;
            d = sqrt(dx^2 + dy^2);
        else
            disp('Final waypoint reached.');

            % Store the final state before stopping.
            x_hist(k) = x;
            y_hist(k) = y;
            theta_hist(k) = theta;
            v_hist(k) = 0;
            omega_hist(k) = 0;
            d_hist(k) = d;
            err_hist(k) = 0;
            wp_idx_hist(k) = wp_idx;

            N = k;
            break;
        end
    end

    % Calculate desired heading and heading error.
    theta_d = atan2(dy, dx);
    delta_theta = theta_d - theta;

    % Wrap heading error to [-pi, pi].
    delta_theta = atan2(sin(delta_theta), cos(delta_theta));
    abs_delta_theta = abs(delta_theta);

    % =====================================================================
    % CONTROLLER SECTION: Calculate v and omega
    % =====================================================================
    %v = 0;
    %omega = 0;

    % ---------------------------------------------------------------------
    % SCENARIO A: Naive constant-speed controller
    % ---------------------------------------------------------------------
    % Uncomment this line for the baseline simulation:
    %
    % Naive constant-speed controller
    %v = 2.0;

    % Low-level proportional steering controller
    %omega = Kh * delta_theta;

    % ---------------------------------------------------------------------
    % SCENARIO B: Fuzzy supervisory speed controller
    % ---------------------------------------------------------------------
    % Uncomment this line after loading your FIS:
    %
    v = evalfis(fis, [d, abs_delta_theta]);

    % ---------------------------------------------------------------------
    % Low-level proportional steering controller
    % ---------------------------------------------------------------------
    omega = Kh * delta_theta;
    

    % =================================================
    % Simulation
    % =================================================
    v = max(0, min(2.0, v));

    % Update kinematics using Euler integration.
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;

    % Wrap theta to [-pi, pi] for numerical cleanliness.
    theta = atan2(sin(theta), cos(theta));

    % Store data for plotting.
    x_hist(k) = x;
    y_hist(k) = y;
    theta_hist(k) = theta;
    v_hist(k) = v;
    omega_hist(k) = omega;
    d_hist(k) = d;
    err_hist(k) = abs_delta_theta;
    wp_idx_hist(k) = wp_idx;
end

%% 4. Trim History Arrays

x_hist = x_hist(1:N);
y_hist = y_hist(1:N);
theta_hist = theta_hist(1:N);
v_hist = v_hist(1:N);
omega_hist = omega_hist(1:N);
d_hist = d_hist(1:N);
err_hist = err_hist(1:N);
wp_idx_hist = wp_idx_hist(1:N);

time = (0:N-1) * dt;

%% 5. Plot 2D Trajectory

figure(1); hold on; grid on;

h_wp = plot(WPs(:, 1), WPs(:, 2), 'ro--', ...
    'MarkerSize', 8, 'LineWidth', 1.5);

h_path = plot(x_hist, y_hist, 'b-', ...
    'LineWidth', 2);

% Draw acceptance radius circles.
th = linspace(0, 2*pi, 100);
h_circle = [];

for i = 1:num_WPs
    x_circle = acceptance_radius * cos(th) + WPs(i, 1);
    y_circle = acceptance_radius * sin(th) + WPs(i, 2);

    h_tmp = plot(x_circle, y_circle, 'k:', 'LineWidth', 1);

    if i == 1
        h_circle = h_tmp;
    end
end

title('USV 2D Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');

legend([h_wp, h_path, h_circle], ...
       {'Waypoints', 'USV Path', 'Acceptance Radius'}, ...
       'Location', 'best');

axis equal;

%% 6. Plots

figure(2);

subplot(4, 1, 1);
plot(time, v_hist, 'LineWidth', 1.5);
grid on;
ylabel('v (m/s)');
title('USV Signals over Time');

subplot(4, 1, 2);
plot(time, omega_hist, 'LineWidth', 1.5);
grid on;
ylabel('\omega (rad/s)');

subplot(4, 1, 3);
plot(time, d_hist, 'LineWidth', 1.5);
grid on;
ylabel('d (m)');

subplot(4, 1, 4);
plot(time, err_hist, 'LineWidth', 1.5);
grid on;
ylabel('|\Delta\theta| (rad)');
xlabel('Time (s)');

%%
fprintf('\n========== FUZZY USV RESULT ==========\n');
fprintf('Final waypoint index reached: %d / %d\n', wp_idx, num_WPs);
fprintf('Final distance to active waypoint: %.4f m\n', d_hist(end));
fprintf('Acceptance radius: %.4f m\n', acceptance_radius);
fprintf('Simulation end time: %.2f s\n', time(end));
fprintf('=========================================\n');