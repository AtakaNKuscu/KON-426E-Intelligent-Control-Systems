clear; clc; close all;

%% Simulation settings
N = 80;
k = 1:N;

%% Reference signal
r = zeros(1, N);
r(1:20) = 0.5;
r(21:50) = 1.0;
r(51:N) = 0.8;

%% Run fixed PID
[y_pid, u_pid, e_pid] = simulate_fixed_pid(r, N);

%% Run adaptive PID-like
[y_ad, u_ad, e_ad, wP_hist, wI_hist, wD_hist] = simulate_adaptive_pid(r, N);

%% Performance measures
IAE_pid = sum(abs(e_pid));
ISE_pid = sum(e_pid.^2);

IAE_ad = sum(abs(e_ad));
ISE_ad = sum(e_ad.^2);

fprintf('Fixed PID:     IAE = %.6f, ISE = %.6f\n', IAE_pid, ISE_pid);
fprintf('Adaptive PID:  IAE = %.6f, ISE = %.6f\n', IAE_ad, ISE_ad);

%% Plot fixed PID
figure;
plot(1:N, r, 'k--', 'LineWidth', 1.5); hold on;
plot(1:N, y_pid(1:N), 'b', 'LineWidth', 1.5);
xlabel('k'); ylabel('Output');
legend('Reference r(k)', 'Output y(k)');
title('Fixed PID: Output Tracking');
grid on;

figure;
stairs(1:N, u_pid, 'LineWidth', 1.5);
xlabel('k'); ylabel('u(k)');
title('Fixed PID: Control Signal');
grid on;

figure;
plot(1:N, e_pid, 'LineWidth', 1.5);
xlabel('k'); ylabel('e(k)');
title('Fixed PID: Tracking Error');
grid on;

%% Plot adaptive PID-like
figure;
plot(1:N, r, 'k--', 'LineWidth', 1.5); hold on;
plot(1:N, y_ad(1:N), 'r', 'LineWidth', 1.5);
xlabel('k'); ylabel('Output');
legend('Reference r(k)', 'Output y(k)');
title('Adaptive PID-like: Output Tracking');
grid on;

figure;
stairs(1:N, u_ad, 'LineWidth', 1.5);
xlabel('k'); ylabel('u(k)');
title('Adaptive PID-like: Control Signal');
grid on;

figure;
plot(1:N, e_ad, 'LineWidth', 1.5);
xlabel('k'); ylabel('e(k)');
title('Adaptive PID-like: Tracking Error');
grid on;

figure;
plot(1:N, wP_hist, 'LineWidth', 1.5); hold on;
plot(1:N, wI_hist, 'LineWidth', 1.5);
plot(1:N, wD_hist, 'LineWidth', 1.5);
xlabel('k'); ylabel('Weights');
legend('w_P(k)', 'w_I(k)', 'w_D(k)');
title('Adaptive PID-like: Weight Evolution');
grid on;

%% Fixed PID
function [y, u, e] = simulate_fixed_pid(r, N)

    % Initial PID gains (first trial)
    Kp = 1.0;
    Ki = 0.05;
    Kd = 0.10;

    y = zeros(1, N+1);   % y(1) corresponds to y(0)
    u = zeros(1, N);
    e = zeros(1, N);

    Iterm = 0;
    e_prev = 0;

    for k = 1:N
        % tracking error
        e(k) = r(k) - y(k);

        % PID terms
        Iterm = Iterm + e(k);
        Dterm = e(k) - e_prev;

        % control
        u(k) = Kp * e(k) + Ki * Iterm + Kd * Dterm;

        % plant update
        y(k+1) = y(k)/(1 + 0.1*y(k)^2) + u(k);

        % update previous error
        e_prev = e(k);
    end
end

%% Adaptive PID
function [y, u, e, wP_hist, wI_hist, wD_hist] = simulate_adaptive_pid(r, N)

    % Given parameters
    eta = 0.03;

    wP = 0.8;
    wI = 0.05;
    wD = 0.10;

    y = zeros(1, N+1);   % y(1) = y(0)
    u = zeros(1, N);
    e = zeros(1, N);

    wP_hist = zeros(1, N);
    wI_hist = zeros(1, N);
    wD_hist = zeros(1, N);

    xI = 0;
    e_prev = 0;

    for k = 1:N
        % error
        e(k) = r(k) - y(k);

        % PID-like channels
        xP = e(k);
        xI = xI + e(k);
        xD = e(k) - e_prev;

        % control signal
        u(k) = wP * xP + wI * xI + wD * xD;

        % plant update
        y(k+1) = y(k)/(1 + 0.1*y(k)^2) + u(k);

        % save weights
        wP_hist(k) = wP;
        wI_hist(k) = wI;
        wD_hist(k) = wD;

        % normalized adaptive update
        denom = 1 + xP^2 + xI^2 + xD^2;

        wP = wP + eta * (e(k) * xP) / denom;
        wI = wI + eta * (e(k) * xI) / denom;
        wD = wD + eta * (e(k) * xD) / denom;

        % clipping
        wP = min(max(wP, 0), 3.0);
        wI = min(max(wI, 0), 1.0);
        wD = min(max(wD, 0), 1.0);

        % update previous error
        e_prev = e(k);
    end
end