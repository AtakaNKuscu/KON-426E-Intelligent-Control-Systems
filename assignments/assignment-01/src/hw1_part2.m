clear; clc; close all;

%% System data
A = [1 0.2;
     0 0.9];
B = [0;
     0.2];

Q = [10 0;
     0 1];

Qf = [20 0;
      0 2];

r = 1.0;
H = 5;

x0 = [2; 0];
Nsim = 15;

%% Storage
x_hist = zeros(2, Nsim+1);
u_hist = zeros(1, Nsim);

x_hist(:,1) = x0;

%% Optimization settings
options = optimoptions('fminunc', ...
    'Algorithm', 'quasi-newton', ...
    'Display', 'off', ...
    'MaxIterations', 200);

U_init = zeros(H,1);   % initial guess

%% Receding horizon control loop
for k = 1:Nsim
    xk = x_hist(:,k);

    cost_handle = @(U) mpc_cost(U, xk, A, B, Q, Qf, r, H);

    U_opt = fminunc(cost_handle, U_init, options);

    uk = U_opt(1);
    u_hist(k) = uk;

    x_next = A*xk + B*uk;
    x_hist(:,k+1) = x_next;

    % warm start for next step
    U_init = [U_opt(2:end); U_opt(end)];
end

%% Time axis
k_vec = 0:Nsim;
u_vec = 0:Nsim-1;

%% Plot states
figure;
plot(k_vec, x_hist(1,:), 'LineWidth', 1.5); hold on;
plot(k_vec, x_hist(2,:), 'LineWidth', 1.5);
xlabel('k');
ylabel('States');
legend('x_1(k)', 'x_2(k)');
title('State Trajectories');
grid on;

%% Plot control
figure;
stairs(u_vec, u_hist, 'LineWidth', 1.5);
xlabel('k');
ylabel('u(k)');
title('Applied Control Input');
grid on;

%%
function J = mpc_cost(U, xk, A, B, Q, Qf, r, H)
    x = xk;
    J = 0;

    for i = 1:H
        u = U(i);

        % stage cost
        J = J + x' * Q * x + r * u^2;

        % state update
        x = A * x + B * u;
    end

    % terminal cost
    J = J + x' * Qf * x;
end