clear; clc; close all;

%% Load data
data = load('signal.mat');

t = data.t(:);   % make it column vector
y = data.y(:);   % make it column vector

N = length(t);
N_train = floor(0.8 * N);

%% Train-test split
t_train = t(1:N_train);
y_train = y(1:N_train);

t_test = t(N_train+1:end);
y_test = y(N_train+1:end);

%% Model function
model = @(theta, t) theta(1) * sin(theta(2) * t + theta(3)) + theta(4);

%% Cost function on training set only
cost_fun = @(theta) 0.5 * sum((y_train - model(theta, t_train)).^2);

%% Optional: visualize split
figure;
plot(t, y, 'b', 'LineWidth', 1.2); hold on;
xline(t_train(end), '--r', 'LineWidth', 1.5);
legend('Noisy signal', 'Train/Test boundary');
xlabel('t');
ylabel('y');
title('Signal and Train/Test Split');
grid on;


%%
function yhat = sinusoid_model(theta, t)
    A = theta(1);
    w = theta(2);
    phi = theta(3);
    c = theta(4);

    yhat = A * sin(w * t + phi) + c;
end

%%
function J = cost_function(theta, t, y)
    yhat = sinusoid_model(theta, t);
    e = y - yhat;
    J = 0.5 * sum(e.^2);
end

%%
function grad = gradient_function(theta, t, y)
    A = theta(1);
    w = theta(2);
    phi = theta(3);

    s = sin(w * t + phi);
    cvec = cos(w * t + phi);

    yhat = A * s + theta(4);
    e = y - yhat;

    dJ_dA   = -sum(e .* s);
    dJ_dw   = -sum(e .* (A * cvec .* t));
    dJ_dphi = -sum(e .* (A * cvec));
    dJ_dc   = -sum(e);

    grad = [dJ_dA; dJ_dw; dJ_dphi; dJ_dc];
end

%%
function H = hessian_function(theta, t, y)
    A = theta(1);
    w = theta(2);
    phi = theta(3);
    c = theta(4);

    s = sin(w * t + phi);
    cvec = cos(w * t + phi);

    yhat = A * s + c;
    e = y - yhat;

    H = zeros(4,4);

    for i = 1:length(t)
        ti = t(i);
        si = s(i);
        ci = cvec(i);
        ei = e(i);

        g = [si;
             A * ci * ti;
             A * ci;
             1];

        second_model = [0,         ci*ti,          ci,      0;
                        ci*ti,    -A*si*ti^2,    -A*si*ti, 0;
                        ci,       -A*si*ti,      -A*si,    0;
                        0,         0,             0,       0];

        H = H + (g * g.') - ei * second_model;
    end
end

%%
function [theta, cost_history] = gradient_descent(theta0, t, y, alpha, max_iter, tol)
    theta = theta0;
    cost_history = zeros(max_iter, 1);

    for k = 1:max_iter
        J = cost_function(theta, t, y);
        grad = gradient_function(theta, t, y);

        cost_history(k) = J;

        % Update rule
        theta_new = theta - alpha * grad;

        % Stopping criterion
        if norm(grad) < tol
            theta = theta_new;
            cost_history = cost_history(1:k);
            fprintf('GD converged at iteration %d\n', k);
            return;
        end

        theta = theta_new;
    end

    fprintf('GD reached maximum iterations.\n');
end


%% Initial guess
A0 = (max(y_train) - min(y_train)) / 2;
w0 = 1.0;
phi0 = 0.0;
c0 = mean(y_train);

theta0 = [A0; w0; phi0; c0];

%% GD parameters
alpha = 1e-4;
max_iter = 5000;
tol = 1e-6;

%% Run GD
[theta_gd, cost_gd] = gradient_descent(theta0, t_train, y_train, alpha, max_iter, tol);

%% Display result
disp('Estimated parameters with GD:');
disp(theta_gd);

%% Plot training cost
figure;
plot(cost_gd, 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('Training Cost J(\theta)');
title('Gradient Descent: Training Cost vs Iteration');
grid on;

%% Plot fit on training and test data
yhat_train_gd = sinusoid_model(theta_gd, t_train);
yhat_test_gd  = sinusoid_model(theta_gd, t_test);

figure;
plot(t_train, y_train, 'b.', 'DisplayName', 'Training Data'); hold on;
plot(t_train, yhat_train_gd, 'r-', 'LineWidth', 1.5, 'DisplayName', 'GD Fit (Train)');
plot(t_test, y_test, 'g.', 'DisplayName', 'Test Data');
plot(t_test, yhat_test_gd, 'k--', 'LineWidth', 1.5, 'DisplayName', 'GD Prediction (Test)');
xlabel('t');
ylabel('y');
title('Gradient Descent Fit');
legend;
grid on;

%%
mse_gd = mean((y_test - yhat_test_gd).^2);
fprintf('GD Test MSE = %.6f\n', mse_gd);

%%
function [theta, cost_history] = gradient_descent_momentum(theta0, t, y, alpha, beta, max_iter, tol)
    theta = theta0;
    v = zeros(size(theta0));
    cost_history = zeros(max_iter, 1);

    for k = 1:max_iter
        J = cost_function(theta, t, y);
        grad = gradient_function(theta, t, y);

        cost_history(k) = J;

        % Momentum update
        v = beta * v - alpha * grad;
        theta_new = theta + v;

        % Stopping criterion
        if norm(grad) < tol
            theta = theta_new;
            cost_history = cost_history(1:k);
            fprintf('Momentum GD converged at iteration %d\n', k);
            return;
        end

        theta = theta_new;
    end

    fprintf('Momentum GD reached maximum iterations.\n');
end

%%
%% Momentum parameters
alpha_m = 1e-4;
beta = 0.9;
max_iter = 5000;
tol = 1e-6;

%% Run Momentum GD
[theta_mom, cost_mom] = gradient_descent_momentum(theta0, t_train, y_train, alpha_m, beta, max_iter, tol);

%% Display result
disp('Estimated parameters with Momentum GD:');
disp(theta_mom);

%% Plot training cost
figure;
plot(cost_mom, 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('Training Cost J(\theta)');
title('Gradient Descent with Momentum: Training Cost vs Iteration');
grid on;

%% Predictions
yhat_train_mom = sinusoid_model(theta_mom, t_train);
yhat_test_mom  = sinusoid_model(theta_mom, t_test);

%% Test MSE
mse_mom = mean((y_test - yhat_test_mom).^2);
fprintf('Momentum GD Test MSE = %.6f\n', mse_mom);

%% Plot fit
figure;
plot(t_train, y_train, 'b.', 'DisplayName', 'Training Data'); hold on;
plot(t_train, yhat_train_mom, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Momentum Fit (Train)');
plot(t_test, y_test, 'g.', 'DisplayName', 'Test Data');
plot(t_test, yhat_test_mom, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Momentum Prediction (Test)');
xlabel('t');
ylabel('y');
title('Gradient Descent with Momentum Fit');
legend;
grid on;

%%
figure;
plot(cost_gd, 'LineWidth', 1.5, 'DisplayName', 'Gradient Descent'); hold on;
plot(cost_mom, 'LineWidth', 1.5, 'DisplayName', 'Momentum GD');
xlabel('Iteration');
ylabel('Training Cost J(\theta)');
title('Training Cost Comparison: GD vs Momentum GD');
legend;
grid on;

%%
function [theta, cost_history] = newton_method(theta0, t, y, max_iter, tol, lambda)
    theta = theta0;
    cost_history = zeros(max_iter, 1);

    for k = 1:max_iter
        J = cost_function(theta, t, y);
        grad = gradient_function(theta, t, y);
        H = hessian_function(theta, t, y);

        cost_history(k) = J;

        % Regularized Hessian
        H_reg = H + lambda * eye(length(theta));

        % Newton step
        p = H_reg \ grad;

        % Update
        theta_new = theta - p;

        % Stopping criterion
        if norm(grad) < tol
            theta = theta_new;
            cost_history = cost_history(1:k);
            fprintf('Newton converged at iteration %d\n', k);
            return;
        end

        theta = theta_new;
    end

    fprintf('Newton reached maximum iterations.\n');
end

%% Newton parameters
max_iter_newton = 500;
tol_newton = 1e-6;
lambda = 1e-3;

%% Use the same initial guess
[theta_newton, cost_newton] = newton_method(theta0, t_train, y_train, max_iter_newton, tol_newton, lambda);

%% Display result
disp('Estimated parameters with Newton Method:');
disp(theta_newton);

%% Plot training cost
figure;
plot(cost_newton, 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('Training Cost J(\theta)');
title('Newton Method: Training Cost vs Iteration');
grid on;

%% Predictions
yhat_train_newton = sinusoid_model(theta_newton, t_train);
yhat_test_newton  = sinusoid_model(theta_newton, t_test);

%% Test MSE
mse_newton = mean((y_test - yhat_test_newton).^2);
fprintf('Newton Test MSE = %.6f\n', mse_newton);

%% Plot fit
figure;
plot(t_train, y_train, 'b.', 'DisplayName', 'Training Data'); hold on;
plot(t_train, yhat_train_newton, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Newton Fit (Train)');
plot(t_test, y_test, 'g.', 'DisplayName', 'Test Data');
plot(t_test, yhat_test_newton, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Newton Prediction (Test)');
xlabel('t');
ylabel('y');
title('Newton Method Fit');
legend;
grid on;

%%
figure;
plot(cost_gd, 'LineWidth', 1.5, 'DisplayName', 'Gradient Descent'); hold on;
plot(cost_mom, 'LineWidth', 1.5, 'DisplayName', 'Momentum GD');
plot(cost_newton, 'LineWidth', 1.5, 'DisplayName', 'Newton Method');
xlabel('Iteration');
ylabel('Training Cost J(\theta)');
title('Training Cost Comparison');
legend;
grid on;