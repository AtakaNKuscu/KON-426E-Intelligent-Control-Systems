%% KON426E - Intelligent Control Systems
% Assignment 2 - Question 2 Part 1
% Fixed-Gain PI Control of a Nonlinear Spherical Tank using ode45

clear; clc; close all;

%% Plant parameters

R = 1.5;          % Tank radius [m]
Cv = 0.2;         % Outlet coefficient
h_max = 3.0;      % Maximum tank height [m]

%% Simulation parameters

Tend = 200;
tspan = [0 Tend];

%% Fixed PI gains
% Tuned primarily for h = 0.0 m to h = 0.5 m

Kp = 15.0;
Ki = 0.30;

fprintf('Fixed PI gains:\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);

%% Scenario 1: h = 0.0 m to h_ref = 0.5 m

h0_1 = 0.0;
href_1 = 0.5;

[t1, h1, u1, e1] = simulate_tank_PI_ode45(h0_1, href_1, Kp, Ki, R, Cv, h_max, tspan);

%% Scenario 2: h = 1.0 m to h_ref = 1.5 m

h0_2 = 1.0;
href_2 = 1.5;

[t2, h2, u2, e2] = simulate_tank_PI_ode45(h0_2, href_2, Kp, Ki, R, Cv, h_max, tspan);

%% Performance metrics

metrics1 = calculate_step_metrics(t1, h1, href_1);
metrics2 = calculate_step_metrics(t2, h2, href_2);

fprintf('\n========== Scenario 1: 0.0 m to 0.5 m ==========\n');
disp(metrics1);

fprintf('\n========== Scenario 2: 1.0 m to 1.5 m ==========\n');
disp(metrics2);

%% Plot height responses separately

figure;
plot(t1, h1, 'LineWidth', 1.6);
hold on;
plot(t1, href_1 * ones(size(t1)), 'k--', 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('Fluid height h(t) [m]');
title('Fixed PI Response: Step from 0.0 m to 0.5 m');
legend('h(t)', 'h_{ref}', 'Location', 'best');

figure;
plot(t2, h2, 'LineWidth', 1.6);
hold on;
plot(t2, href_2 * ones(size(t2)), 'k--', 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('Fluid height h(t) [m]');
title('Fixed PI Response: Step from 1.0 m to 1.5 m');
legend('h(t)', 'h_{ref}', 'Location', 'best');

%% Combined response plot

figure;
subplot(2,1,1);
plot(t1, h1, 'LineWidth', 1.6);
hold on;
plot(t1, href_1 * ones(size(t1)), 'k--', 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('h(t) [m]');
title('Scenario 1: Fixed PI, h = 0.0 m to 0.5 m');
legend('Tank level', 'Reference', 'Location', 'best');

subplot(2,1,2);
plot(t2, h2, 'LineWidth', 1.6);
hold on;
plot(t2, href_2 * ones(size(t2)), 'k--', 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('h(t) [m]');
title('Scenario 2: Fixed PI, h = 1.0 m to 1.5 m');
legend('Tank level', 'Reference', 'Location', 'best');

%% Control signal plot

figure;
subplot(2,1,1);
plot(t1, u1, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('u(t)');
title('Control Signal: Scenario 1');

subplot(2,1,2);
plot(t2, u2, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('u(t)');
title('Control Signal: Scenario 2');

%% Cross-sectional area plot

h_vec = linspace(0, h_max, 500);
A_vec = pi .* (2 * R .* h_vec - h_vec.^2);

figure;
plot(h_vec, A_vec, 'LineWidth', 1.6);
grid on;
xlabel('Fluid height h [m]');
ylabel('Cross-sectional area A(h) [m^2]');
title('Nonlinear Cross-sectional Area of the Spherical Tank');

%% ========================================================================
% Local functions
% ========================================================================

function [t, h, u_hist, e_hist] = simulate_tank_PI_ode45(h0, href, Kp, Ki, R, Cv, h_max, tspan)

    % State vector:
    % x(1) = h
    % x(2) = integral of error

    x0 = [h0; 0];

    opts = odeset('RelTol', 1e-7, ...
                  'AbsTol', 1e-9, ...
                  'MaxStep', 0.05);

    [t, x] = ode45(@(t,x) tank_PI_ode(t, x, href, Kp, Ki, R, Cv, h_max), tspan, x0, opts);

    h = x(:,1);
    I = x(:,2);

    % Numerical cleanup for plotting
    h = max(0, min(h, h_max));

    e_hist = href - h;
    u_hist = Kp .* e_hist + Ki .* I;
    u_hist = max(0, min(u_hist, 1.0));
end

function dxdt = tank_PI_ode(~, x, href, Kp, Ki, R, Cv, h_max)

    h = x(1);
    I = x(2);

    % Physical limits for computation
    h = max(0, min(h, h_max));

    e = href - h;

    % PI control
    u_unsat = Kp * e + Ki * I;

    % Inflow saturation
    u_min = 0.0;
    u_max = 1.0;
    u = max(u_min, min(u_unsat, u_max));

    % Anti-windup logic
    if abs(u - u_unsat) < 1e-12
        Idot = e;
    elseif u >= u_max && e < 0
        Idot = e;
    elseif u <= u_min && e > 0
        Idot = e;
    else
        Idot = 0;
    end

    % Tank dynamics
    hdot = spherical_tank_dynamics(h, u, R, Cv, h_max);

    % Prevent numerical motion outside physical limits
    if h <= 0 && hdot < 0
        hdot = 0;
    elseif h >= h_max && hdot > 0
        hdot = 0;
    end

    dxdt = [hdot; Idot];
end

function dhdt = spherical_tank_dynamics(h, u, R, Cv, h_max)

    % Numerical regularization:
    % A(0) = 0 causes a singular ODE. h_eff is used only for evaluating
    % area and outflow near the empty-tank boundary.
    h_eps = 1e-3;

    h_eff = max(h, h_eps);
    h_eff = min(h_eff, h_max - h_eps);

    A = pi * (2 * R * h_eff - h_eff^2);
    q_out = Cv * sqrt(h_eff);

    dhdt = (u - q_out) / A;
end

function metrics = calculate_step_metrics(t, h, href)

    final_value = h(end);
    steady_state_error = abs(href - final_value);

    initial_value = h(1);
    step_amplitude = href - initial_value;

    overshoot = max(0, (max(h) - href) / max(abs(step_amplitude), 1e-6) * 100);

    tolerance = 0.02 * abs(step_amplitude);

    if tolerance < 1e-6
        tolerance = 0.01;
    end

    idx_settle = find(abs(h - href) > tolerance, 1, 'last');

    if isempty(idx_settle)
        settling_time = 0;
    elseif idx_settle == length(t)
        settling_time = NaN;
    else
        settling_time = t(idx_settle + 1);
    end

    y10 = initial_value + 0.10 * step_amplitude;
    y90 = initial_value + 0.90 * step_amplitude;

    idx10 = find(h >= y10, 1, 'first');
    idx90 = find(h >= y90, 1, 'first');

    if isempty(idx10) || isempty(idx90)
        rise_time = NaN;
    else
        rise_time = t(idx90) - t(idx10);
    end

    metrics = table(final_value, steady_state_error, overshoot, rise_time, settling_time);
end