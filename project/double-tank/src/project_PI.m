clear; clc; close all;

%% Nonlinear Double Tank System with PI Controller

Tend = 300;              % simulation time [s]
x0 = [0.05; 0.05; 0];    % states: h1, h2, integral of error

r = 0.25;                % reference level for h2 [m]

% PI gains - first trial
Kp = 80;
Ki = 0.8;

[t, x] = ode45(@(t,x) closedLoopDoubleTankPI(t, x, r, Kp, Ki), [0 Tend], x0);

h1 = x(:,1);
h2 = x(:,2);
Ie = x(:,3);

% Reconstruct control input and errors
e = r - h2;
u = zeros(size(t));

for k = 1:length(t)
    u_unsat = Kp * e(k) + Ki * Ie(k);
    u(k) = min(max(u_unsat, 0), 10);
end

de = [0; diff(e)./diff(t)];

%% Plot output tracking
figure;
plot(t, h2, 'LineWidth', 1.8); hold on;
plot(t, r*ones(size(t)), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
legend('h_2(t)', 'Reference');
title('PI Control: Step Tracking Response');

%% Plot both tank levels
figure;
plot(t, h1, 'LineWidth', 1.5); hold on;
plot(t, h2, 'LineWidth', 1.5);
plot(t, r*ones(size(t)), '--', 'LineWidth', 1.3);
grid on;
xlabel('Time [s]');
ylabel('Water Level [m]');
legend('h_1(t)', 'h_2(t)', 'Reference');
title('PI Control: Tank Levels');

%% Plot tracking error
figure;
plot(t, e, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Error e(t) [m]');
title('PI Control: Tracking Error');

%% Plot change of error
figure;
plot(t, de, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Change of Error de/dt [m/s]');
title('PI Control: Change of Error');

%% Plot control input
figure;
plot(t, u, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Control Input u(t) [V]');
title('PI Control: Pump Voltage');
ylim([-0.5 10.5]);

%% Performance metrics
final_error = abs(r - h2(end));
ISE = trapz(t, e.^2);
IAE = trapz(t, abs(e));
max_u = max(u);

fprintf('\n===== PI CONTROLLER PERFORMANCE =====\n');
fprintf('Reference h2        = %.4f m\n', r);
fprintf('Final h2            = %.4f m\n', h2(end));
fprintf('Final absolute error= %.6f m\n', final_error);
fprintf('ISE                 = %.6f\n', ISE);
fprintf('IAE                 = %.6f\n', IAE);
fprintf('Maximum control u   = %.4f V\n', max_u);
fprintf('=====================================\n');

%% Closed-loop nonlinear system
function dx = closedLoopDoubleTankPI(~, x, r, Kp, Ki)

    h1 = max(x(1), 0);
    h2 = max(x(2), 0);
    Ie = x(3);

    % Error
    e = r - h2;

    % PI controller
    u_unsat = Kp * e + Ki * Ie;

    % Pump voltage saturation
    u = min(max(u_unsat, 0), 10);

    % Nonlinear double tank parameters
    k = 1.035454e-2;
    ku = 1.143e-3;

    % Nonlinear flows
    flow12 = k * sign(h1 - h2) * sqrt(abs(h1 - h2));
    flowOut = k * sqrt(h2);

    % State equations
    dh1 = -flow12 + ku * u;
    dh2 =  flow12 - flowOut;

    % Integral of error
    dIe = e;

    dx = [dh1; dh2; dIe];
end