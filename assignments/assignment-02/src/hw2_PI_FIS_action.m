%% KON426E - Intelligent Control Systems
% Assignment 2 - Question 2 Part 3
% Fuzzy Gain-Scheduling PI Control of Spherical Tank

clear; clc; close all;

%% Plant parameters

R = 1.5;
Cv = 0.2;
h_max = 3.0;

%% Load FIS

fis = readfis('FGS_PI_Controller.fis');

%% Simulation parameters

dt = 0.01;
Tend = 600;
time = 0:dt:Tend;
N = length(time);

%% Reference signal: changes every 200 seconds

href = zeros(1, N);

for k = 1:N
    t = time(k);

    if t < 200
        href(k) = 0.5;
    elseif t < 400
        href(k) = 1.5;
    else
        href(k) = 2.5;
    end
end

%% Fuzzy gain bounds
% Suggested by the assignment

Kp_min = 1;
Kp_max = 20;

Ki_min = 0.01;
Ki_max = 2.0;

%% Scaling factors for fuzzy inputs

Ne = 1.0;      % error scaling
NdE = 0.2;     % derivative of error scaling

%% Initial conditions

h = 0.0;
integral_e = 0;
e_prev = href(1) - h;

%% Storage arrays

h_hist = zeros(1, N);
u_hist = zeros(1, N);
e_hist = zeros(1, N);
E_norm_hist = zeros(1, N);
dE_norm_hist = zeros(1, N);
Kp_hist = zeros(1, N);
Ki_hist = zeros(1, N);
Kp_norm_hist = zeros(1, N);
Ki_norm_hist = zeros(1, N);

%% Actuator limits

u_min = 0.0;
u_max = 1.0;

%% Simulation loop

for k = 1:N

    e = href(k) - h;
    de = (e - e_prev) / dt;

    % Normalize fuzzy inputs
    E_norm = Ne * e;
    dE_norm = NdE * de;

    % Saturate into [-1, 1]
    E_norm = max(-1, min(1, E_norm));
    dE_norm = max(-1, min(1, dE_norm));

    % Evaluate FIS
    fis_out = evalfis(fis, [E_norm dE_norm]);

    Kp_norm = fis_out(1);
    Ki_norm = fis_out(2);

    % Map normalized fuzzy outputs to actual gain ranges
    Kp = Kp_min + (Kp_max - Kp_min) * Kp_norm;
    Ki = Ki_min + (Ki_max - Ki_min) * Ki_norm;

    % PI controller with candidate integral update
    integral_candidate = integral_e + e * dt;

    u_unsat = Kp * e + Ki * integral_candidate;
    u = min(max(u_unsat, u_min), u_max);

    % Anti-windup
    if abs(u - u_unsat) < 1e-12
        integral_e = integral_candidate;
    elseif u >= u_max && e < 0
        integral_e = integral_candidate;
    elseif u <= u_min && e > 0
        integral_e = integral_candidate;
    end

    % Store values before integration
    h_hist(k) = h;
    u_hist(k) = u;
    e_hist(k) = e;
    E_norm_hist(k) = E_norm;
    dE_norm_hist(k) = dE_norm;
    Kp_hist(k) = Kp;
    Ki_hist(k) = Ki;
    Kp_norm_hist(k) = Kp_norm;
    Ki_norm_hist(k) = Ki_norm;

    % RK4 integration
    k1 = tank_dynamics(h, u, R, Cv, h_max);

    h2 = min(max(h + 0.5 * dt * k1, 0), h_max);
    k2 = tank_dynamics(h2, u, R, Cv, h_max);

    h3 = min(max(h + 0.5 * dt * k2, 0), h_max);
    k3 = tank_dynamics(h3, u, R, Cv, h_max);

    h4 = min(max(h + dt * k3, 0), h_max);
    k4 = tank_dynamics(h4, u, R, Cv, h_max);

    h = h + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);

    % Physical saturation
    h = max(0, min(h, h_max));

    e_prev = e;
end

%% Plot height tracking response

figure;
plot(time, href, 'k--', 'LineWidth', 1.5);
hold on;
plot(time, h_hist, 'b-', 'LineWidth', 1.6);
grid on;
xlabel('Time (s)');
ylabel('Fluid height h(t) [m]');
title('FGS-PI Control of Spherical Tank');
legend('Reference h_{ref}', 'Tank level h(t)', 'Location', 'best');

%% Plot dynamic gain scheduling

figure;
subplot(2,1,1);
plot(time, Kp_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('K_p(t)');
title('Scheduled Proportional Gain');

subplot(2,1,2);
plot(time, Ki_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('K_i(t)');
title('Scheduled Integral Gain');

%% Optional plot: control input and normalized fuzzy inputs

figure;
subplot(3,1,1);
plot(time, u_hist, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('u(t)');
title('Control Input');

subplot(3,1,2);
plot(time, E_norm_hist, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('E');
title('Normalized Error');

subplot(3,1,3);
plot(time, dE_norm_hist, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('\Delta E');
title('Normalized Error Derivative');

%% Print basic tracking errors for each interval

idx1 = time < 200;
idx2 = time >= 200 & time < 400;
idx3 = time >= 400;

rmse1 = sqrt(mean((href(idx1) - h_hist(idx1)).^2));
rmse2 = sqrt(mean((href(idx2) - h_hist(idx2)).^2));
rmse3 = sqrt(mean((href(idx3) - h_hist(idx3)).^2));

fprintf('\n========== FGS-PI TRACKING RMSE ==========\n');
fprintf('Interval 1, href = 0.5 m: RMSE = %.6f\n', rmse1);
fprintf('Interval 2, href = 1.5 m: RMSE = %.6f\n', rmse2);
fprintf('Interval 3, href = 2.5 m: RMSE = %.6f\n', rmse3);
fprintf('==========================================\n');


%% Steady-state RMSE after excluding first 30 seconds of each interval

idx1_ss = time >= 30  & time < 200;
idx2_ss = time >= 230 & time < 400;
idx3_ss = time >= 430 & time <= 600;

rmse1_ss = sqrt(mean((href(idx1_ss) - h_hist(idx1_ss)).^2));
rmse2_ss = sqrt(mean((href(idx2_ss) - h_hist(idx2_ss)).^2));
rmse3_ss = sqrt(mean((href(idx3_ss) - h_hist(idx3_ss)).^2));

fprintf('\n====== FGS-PI STEADY-STATE RMSE, TRANSIENT EXCLUDED ======\n');
fprintf('Interval 1, href = 0.5 m: RMSE = %.6f\n', rmse1_ss);
fprintf('Interval 2, href = 1.5 m: RMSE = %.6f\n', rmse2_ss);
fprintf('Interval 3, href = 2.5 m: RMSE = %.6f\n', rmse3_ss);
fprintf('==========================================================\n');

%% ========================================================================
% Local function
% ========================================================================

function dhdt = tank_dynamics(h, u, R, Cv, h_max)

    % Numerical regularization near h = 0 and h = h_max
    h_eps = 1e-3;

    h_eff = max(h, h_eps);
    h_eff = min(h_eff, h_max - h_eps);

    A = pi * (2 * R * h_eff - h_eff^2);
    q_out = Cv * sqrt(h_eff);

    dhdt = (u - q_out) / A;
end