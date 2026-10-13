%% KON426E - Intelligent Control Systems
% Assignment 2 - Question 1
% Neural System Identification using NARX Network

clear; clc; close all;

%% Student ID and System Creation

studentID = "040210520";
sys = System1(studentID);

%% ================================================================
% PART 1: Data Generation and Excitation
% ================================================================

Ntrain = 3000;
rng(42);

utrain = -1.5 + 3.0 * rand(Ntrain, 1);
ytrain = sys.simulate(utrain, true);

figure;
subplot(2,1,1);
plot(1:200, utrain(1:200), 'LineWidth', 1.2);
grid on;
xlabel('Time step k');
ylabel('u(k)');
title('Training Input: Uniform Random Excitation');

subplot(2,1,2);
plot(1:200, ytrain(1:200), 'LineWidth', 1.2);
grid on;
xlabel('Time step k');
ylabel('y(k)');
title('System Output for Training Input');

%% Convert to NARX time-series format

Utrain_seq = con2seq(utrain');
Ytrain_seq = con2seq(ytrain');

%% ================================================================
% PART 2: NARX Architecture and Training
% ================================================================

delay_candidates = 1:10;
hiddenLayerSize = [20 10];

% Validation input used for delay selection
Ntest = 500;
k = (1:Ntest)';
utest = sin(2*pi*k/25) + 0.5*sin(2*pi*k/10);

sys.reset();
ytest_true = sys.simulate(utest, true);

Utest_seq = con2seq(utest');
Ytest_seq = con2seq(ytest_true');

bestRMSE = inf;
bestDelay = NaN;
bestNet = [];
bestTr = [];

rmse_delay = zeros(length(delay_candidates), 1);

for i = 1:length(delay_candidates)

    d = delay_candidates(i);

    inputDelays = 0:d;
    feedbackDelays = 1:d;

    rng(1);

    net = narxnet(inputDelays, feedbackDelays, hiddenLayerSize, ...
                  'open', 'trainlm');

    % Network transfer functions
    net.layers{1}.transferFcn = 'tansig';
    net.layers{2}.transferFcn = 'tansig';
    net.layers{3}.transferFcn = 'purelin';

    % Training/validation/test split
    net.divideFcn = 'dividerand';
    net.divideParam.trainRatio = 0.70;
    net.divideParam.valRatio   = 0.15;
    net.divideParam.testRatio  = 0.15;

    % Training options
    net.trainParam.epochs = 500;
    net.trainParam.goal = 1e-8;
    net.trainParam.min_grad = 1e-10;
    net.trainParam.max_fail = 25;
    net.trainParam.showWindow = false;

    % Prepare and train
    [Xs, Xi, Ai, Ts] = preparets(net, Utrain_seq, {}, Ytrain_seq);
    [net, tr] = train(net, Xs, Ts, Xi, Ai);

    % One-step-ahead validation
    [Xv, Xiv, Aiv, Tv] = preparets(net, Utest_seq, {}, Ytest_seq);
    Yv_pred = net(Xv, Xiv, Aiv);

    y_true_vec = cell2mat(Tv);
    y_pred_vec = cell2mat(Yv_pred);

    rmse = sqrt(mean((y_true_vec - y_pred_vec).^2));
    rmse_delay(i) = rmse;

    fprintf('Delay order = %d, Validation RMSE = %.6f\n', d, rmse);

    if rmse < bestRMSE
        bestRMSE = rmse;
        bestDelay = d;
        bestNet = net;
        bestTr = tr;
    end
end

fprintf('\nBest delay order = %d\n', bestDelay);
fprintf('Best one-step validation RMSE = %.6f\n', bestRMSE);

net_best = bestNet;

%% Plot delay selection result

figure;
plot(delay_candidates, rmse_delay, 'o-', 'LineWidth', 1.5);
grid on;
xlabel('Delay order');
ylabel('Validation RMSE');
title('NARX Delay Order Selection');

%% Plot training performance

figure;
plotperform(bestTr);
title(sprintf('Training Performance of Best NARX Network, Delay = %d', bestDelay));

%% Optional clean MSE curve for report

figure;
semilogy(bestTr.perf, 'LineWidth', 1.5);
hold on;
semilogy(bestTr.vperf, 'LineWidth', 1.5);
semilogy(bestTr.tperf, 'LineWidth', 1.5);
grid on;
xlabel('Epoch');
ylabel('Mean Squared Error');
title(sprintf('Training, Validation and Test MSE Curves, Delay = %d', bestDelay));
legend('Training MSE', 'Validation MSE', 'Test MSE', 'Location', 'best');

%% ================================================================
% PART 3: Validation on Operational Profile
% ================================================================

sys.reset();
ytest_true = sys.simulate(utest, true);

Utest_seq = con2seq(utest');
Ytest_seq = con2seq(ytest_true');

%% One-step-ahead prediction

[Xv, Xiv, Aiv, Tv] = preparets(net_best, Utest_seq, {}, Ytest_seq);
Yv_pred = net_best(Xv, Xiv, Aiv);

y_true_osa = cell2mat(Tv);
y_pred_osa = cell2mat(Yv_pred);

rmse_osa = sqrt(mean((y_true_osa - y_pred_osa).^2));
time_osa = bestDelay + 1 : Ntest;

figure;
plot(time_osa, y_true_osa, 'k-', 'LineWidth', 1.6);
hold on;
plot(time_osa, y_pred_osa, 'r--', 'LineWidth', 1.4);
grid on;
xlabel('Time step k');
ylabel('Output');
title(sprintf('Operational Validation: True Output vs NARX Prediction, RMSE = %.6f', rmse_osa));
legend('True Plant Output', 'NARX Predicted Output', 'Location', 'best');

%% Closed-loop / free-run prediction

net_closed = closeloop(net_best);
net_closed.name = 'Closed-loop NARX Network';

[Xc, Xic, Aic, Tc] = preparets(net_closed, Utest_seq, {}, Ytest_seq);
Yc_pred = net_closed(Xc, Xic, Aic);

y_true_cl = cell2mat(Tc);
y_pred_cl = cell2mat(Yc_pred);

rmse_cl = sqrt(mean((y_true_cl - y_pred_cl).^2));
time_cl = bestDelay + 1 : Ntest;

figure;
plot(time_cl, y_true_cl, 'k-', 'LineWidth', 1.6);
hold on;
plot(time_cl, y_pred_cl, 'b--', 'LineWidth', 1.4);
grid on;
xlabel('Time step k');
ylabel('Output');
title(sprintf('Closed-loop NARX Validation, RMSE = %.6f', rmse_cl));
legend('True Plant Output', 'Closed-loop NARX Output', 'Location', 'best');

%% Combined validation plot

figure;
subplot(3,1,1);
plot(1:Ntest, utest, 'LineWidth', 1.4);
grid on;
xlabel('Time step k');
ylabel('u_{test}(k)');
title('Operational Validation Input');

subplot(3,1,2);
plot(time_osa, y_true_osa, 'k-', 'LineWidth', 1.5);
hold on;
plot(time_osa, y_pred_osa, 'r--', 'LineWidth', 1.3);
grid on;
xlabel('Time step k');
ylabel('Output');
title(sprintf('One-step-ahead Prediction, RMSE = %.6f', rmse_osa));
legend('True', 'Predicted');

subplot(3,1,3);
plot(time_cl, y_true_cl, 'k-', 'LineWidth', 1.5);
hold on;
plot(time_cl, y_pred_cl, 'b--', 'LineWidth', 1.3);
grid on;
xlabel('Time step k');
ylabel('Output');
title(sprintf('Closed-loop Prediction, RMSE = %.6f', rmse_cl));
legend('True', 'Predicted');

%% Final printed results

fprintf('\n========== FINAL VALIDATION RESULTS ==========\n');
fprintf('Best delay order                = %d\n', bestDelay);
fprintf('One-step-ahead validation RMSE  = %.6f\n', rmse_osa);
fprintf('Closed-loop validation RMSE     = %.6f\n', rmse_cl);
fprintf('==============================================\n');