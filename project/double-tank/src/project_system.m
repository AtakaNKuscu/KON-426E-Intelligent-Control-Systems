clear; clc; close all;

%% Nonlinear Double Tank System Test
% States:
% x(1) = h1
% x(2) = h2
%
% Input:
% u = pump voltage [V], limited between 0 and 10 V

Tend = 200;
x0 = [0.05; 0.05];     % initial water levels [m]
u_const = 5;           % constant pump voltage [V]

[t, x] = ode45(@(t,x) doubleTankODE(t, x, u_const), [0 Tend], x0);

h1 = x(:,1);
h2 = x(:,2);

figure;
plot(t, h1, 'LineWidth', 1.5); hold on;
plot(t, h2, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Water Level [m]');
legend('h_1', 'h_2');
title('Open-loop Nonlinear Double Tank Response');

function dx = doubleTankODE(~, x, u)

    h1 = max(x(1), 0);
    h2 = max(x(2), 0);

    % Input saturation
    u = min(max(u, 0), 10);

    k = 1.035454e-2;
    ku = 1.143e-3;

    flow12 = k * sign(h1 - h2) * sqrt(abs(h1 - h2));
    flowOut = k * sqrt(h2);

    dh1 = -flow12 + ku * u;
    dh2 =  flow12 - flowOut;

    dx = [dh1; dh2];
end