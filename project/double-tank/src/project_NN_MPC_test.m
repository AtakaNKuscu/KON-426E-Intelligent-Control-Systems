clear; clc; close all;
rng(10);

%% ================================================================
%  Test Saved Output-Feedback NN-based MPC Controller
%
%  This script does NOT retrain the network.
%  It loads the saved trained controller and tests it under:
%       1) Different step references
%       2) Different initial conditions
%       3) Measurement noise
%       4) Outflow disturbance
%       5) Pump efficiency loss
%       6) Combined disturbance
%
%  Final NN controller:
%       u(k) = NN(h2, r, e, de, Ie, uPrev)
%
%  h1 is NOT given to the NN controller.
% ================================================================

%% ---------------- Load trained model ----------------
load('Output_Feedback_NN_based_MPC_DoubleTank_Final.mat','results');

net = results.net;

%% ---------------- Plant constants ----------------
k  = 1.035454e-2;
ku = 1.143e-3;

hMax = 0.6;

%% ---------------- Simulation settings ----------------
dt = results.dt;
Tend = results.Tend;
Nsim = round(Tend/dt) + 1;
t = (0:Nsim-1)'*dt;

fprintf('\n===== SAVED MODEL LOADED =====\n');
fprintf('dt   = %.3f s\n', dt);
fprintf('Tend = %.3f s\n', Tend);
fprintf('Number of samples = %d\n', Nsim);
fprintf('==============================\n');

%% ================================================================
%  TEST 1 — Different step references, same initial condition
% ================================================================

x0 = [0.05; 0.05];

referenceList = [0.18, 0.20, 0.25, 0.30, 0.33];

refResults = struct();

fprintf('\n===== TEST 1: DIFFERENT STEP REFERENCES =====\n');

figure; hold on; grid on;

for i = 1:length(referenceList)

    r = referenceList(i);

    [h1,h2,h2Meas,u,e,de,Ie] = simulateSavedOFNNController( ...
        net, r, x0, Nsim, dt, k, ku, hMax, 'nominal');

    metrics = computeMetrics(t,h2,u,r);

    refResults(i).r = r;
    refResults(i).h1 = h1;
    refResults(i).h2 = h2;
    refResults(i).h2Meas = h2Meas;
    refResults(i).u = u;
    refResults(i).e = e;
    refResults(i).de = de;
    refResults(i).Ie = Ie;
    refResults(i).metrics = metrics;

    plot(t,h2,'LineWidth',1.5);

    fprintf('\n--- Reference r = %.3f m ---\n', r);
    printMetrics(metrics,r,h2,u);
end

for i = 1:length(referenceList)
    plot(t,referenceList(i)*ones(size(t)),'--','LineWidth',1.0);
end

xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
title('Saved Output-Feedback NN Controller: Different Step References');
legend('h_2 r=0.18','h_2 r=0.20','h_2 r=0.25','h_2 r=0.30','h_2 r=0.33', ...
       'ref 0.18','ref 0.20','ref 0.25','ref 0.30','ref 0.33', ...
       'Location','best');

%% ================================================================
%  TEST 2 — Different initial conditions, same reference
% ================================================================

rMain = 0.25;

initialConditionList = [
    0.03 0.03;
    0.05 0.05;
    0.08 0.04;
    0.10 0.06;
    0.15 0.10
];

icResults = struct();

fprintf('\n===== TEST 2: DIFFERENT INITIAL CONDITIONS =====\n');

figure; hold on; grid on;

for i = 1:size(initialConditionList,1)

    x0_i = initialConditionList(i,:)';

    [h1,h2,h2Meas,u,e,de,Ie] = simulateSavedOFNNController( ...
        net, rMain, x0_i, Nsim, dt, k, ku, hMax, 'nominal');

    metrics = computeMetrics(t,h2,u,rMain);

    icResults(i).x0 = x0_i;
    icResults(i).h1 = h1;
    icResults(i).h2 = h2;
    icResults(i).h2Meas = h2Meas;
    icResults(i).u = u;
    icResults(i).e = e;
    icResults(i).de = de;
    icResults(i).Ie = Ie;
    icResults(i).metrics = metrics;

    plot(t,h2,'LineWidth',1.5);

    fprintf('\n--- Initial condition x0 = [%.3f %.3f] ---\n', x0_i(1), x0_i(2));
    printMetrics(metrics,rMain,h2,u);
end

plot(t,rMain*ones(size(t)),'k--','LineWidth',1.8);

xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
title('Saved Output-Feedback NN Controller: Different Initial Conditions');
legend('x0=[0.03,0.03]','x0=[0.05,0.05]','x0=[0.08,0.04]', ...
       'x0=[0.10,0.06]','x0=[0.15,0.10]','Reference', ...
       'Location','best');

%% ================================================================
%  TEST 3 — Noise and disturbance handling
% ================================================================

x0 = [0.05; 0.05];
rMain = 0.25;

scenarioList = {
    'nominal'
    'measurement_noise'
    'outflow_disturbance'
    'pump_loss'
    'combined_disturbance'
};

scenarioNames = {
    'Nominal'
    'Measurement Noise'
    'Outflow Disturbance'
    'Pump Efficiency Loss'
    'Combined Disturbance'
};

scenarioResults = struct();

fprintf('\n===== TEST 3: NOISE AND DISTURBANCE HANDLING =====\n');

figure; hold on; grid on;

for i = 1:length(scenarioList)

    scenario = scenarioList{i};

    [h1,h2,h2Meas,u,e,de,Ie] = simulateSavedOFNNController( ...
        net, rMain, x0, Nsim, dt, k, ku, hMax, scenario);

    metrics = computeMetrics(t,h2,u,rMain);

    scenarioResults(i).scenario = scenario;
    scenarioResults(i).name = scenarioNames{i};
    scenarioResults(i).h1 = h1;
    scenarioResults(i).h2 = h2;
    scenarioResults(i).h2Meas = h2Meas;
    scenarioResults(i).u = u;
    scenarioResults(i).e = e;
    scenarioResults(i).de = de;
    scenarioResults(i).Ie = Ie;
    scenarioResults(i).metrics = metrics;

    plot(t,h2,'LineWidth',1.5);

    fprintf('\n--- Scenario: %s ---\n', scenarioNames{i});
    printMetrics(metrics,rMain,h2,u);
end

plot(t,rMain*ones(size(t)),'k--','LineWidth',1.8);

xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
title('Saved Output-Feedback NN Controller: Noise and Disturbance Handling');
legend('Nominal','Measurement Noise','Outflow Disturbance', ...
       'Pump Efficiency Loss','Combined Disturbance','Reference', ...
       'Location','best');

%% Control input comparison under scenarios
figure; hold on; grid on;

for i = 1:length(scenarioList)
    plot(t,scenarioResults(i).u,'LineWidth',1.4);
end

xlabel('Time [s]');
ylabel('Control Input u(t) [V]');
ylim([-0.5 10.5]);
title('Control Input under Noise and Disturbance Scenarios');
legend('Nominal','Measurement Noise','Outflow Disturbance', ...
       'Pump Efficiency Loss','Combined Disturbance', ...
       'Location','best');

%% True vs measured h2 for measurement noise
noiseIdx = find(strcmp(scenarioList,'measurement_noise'),1);

figure;
plot(t,scenarioResults(noiseIdx).h2,'LineWidth',1.6); hold on;
plot(t,scenarioResults(noiseIdx).h2Meas,'.','MarkerSize',5);
plot(t,rMain*ones(size(t)),'k--','LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Tank 2 Level [m]');
legend('True h_2','Measured h_2 with noise','Reference','Location','best');
title('Measurement Noise Scenario: True vs Measured Output');

%% Combined disturbance detailed plots
combinedIdx = find(strcmp(scenarioList,'combined_disturbance'),1);

figure;
plot(t,scenarioResults(combinedIdx).e,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Tracking Error e(t) [m]');
title('Combined Disturbance Scenario: Tracking Error');

figure;
plot(t,scenarioResults(combinedIdx).de,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Change of Error \Delta e/\Delta t [m/s]');
title('Combined Disturbance Scenario: Change of Error');

%% ================================================================
%  TEST 4 — Step reference change over time
% ================================================================

% This tests tracking control for changing reference:
% r(t): 0.20 m -> 0.25 m -> 0.30 m

rProfile = zeros(Nsim,1);

for kk = 1:Nsim
    currentTime = t(kk);

    if currentTime < 90
        rProfile(kk) = 0.20;
    elseif currentTime < 180
        rProfile(kk) = 0.25;
    else
        rProfile(kk) = 0.30;
    end
end

[h1_prof,h2_prof,h2m_prof,u_prof,e_prof,de_prof,Ie_prof] = ...
    simulateSavedOFNNControllerReferenceProfile( ...
        net, rProfile, x0, Nsim, dt, k, ku, hMax, 'nominal');

fprintf('\n===== TEST 4: TIME-VARYING STEP REFERENCE PROFILE =====\n');
fprintf('This test uses r(t): 0.20 m -> 0.25 m -> 0.30 m\n');

figure;
plot(t,h2_prof,'LineWidth',1.8); hold on;
plot(t,rProfile,'k--','LineWidth',1.6);
grid on;
xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
legend('h_2(t)','r(t)','Location','best');
title('Saved Output-Feedback NN Controller: Step Reference Profile');

figure;
plot(t,u_prof,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Control Input u(t) [V]');
ylim([-0.5 10.5]);
title('Control Input for Step Reference Profile');

figure;
plot(t,e_prof,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Tracking Error e(t) [m]');
title('Tracking Error for Step Reference Profile');

%% ================================================================
%  Save all test results
% ================================================================

testOutput.refResults = refResults;
testOutput.icResults = icResults;
testOutput.scenarioResults = scenarioResults;
testOutput.rProfile = rProfile;
testOutput.profile.h1 = h1_prof;
testOutput.profile.h2 = h2_prof;
testOutput.profile.h2Meas = h2m_prof;
testOutput.profile.u = u_prof;
testOutput.profile.e = e_prof;
testOutput.profile.de = de_prof;
testOutput.profile.Ie = Ie_prof;
testOutput.t = t;
testOutput.dt = dt;
testOutput.Tend = Tend;

save('Saved_OF_NN_MPC_Extended_Tests.mat','testOutput');

fprintf('\nAll extended tests saved as Saved_OF_NN_MPC_Extended_Tests.mat\n');

%% ================================================================
%  Local functions
% ================================================================

function [h1Arr,h2Arr,h2MeasArr,uArr,eArr,deArr,IeArr] = ...
    simulateSavedOFNNController(net, r, x0, Nsim, dt, k, ku, hMax, scenario)

    h1Arr = zeros(Nsim,1);
    h2Arr = zeros(Nsim,1);
    h2MeasArr = zeros(Nsim,1);
    uArr  = zeros(Nsim,1);
    eArr  = zeros(Nsim,1);
    deArr = zeros(Nsim,1);
    IeArr = zeros(Nsim,1);

    x = x0;
    Ie = 0;
    ePrev = r - x(2);
    uPrev = steadyStateInput(r,k,ku);

    for kk = 1:Nsim

        currentTime = (kk-1)*dt;

        h1 = x(1);
        h2 = x(2);

        h2Meas = getMeasuredOutput(h2, hMax, scenario);

        e_meas = r - h2Meas;
        de_meas = (e_meas - ePrev)/dt;

        phi = makeFeaturesOF(h2Meas,r,e_meas,de_meas,Ie,uPrev);

        uNorm = net(phi);
        u = 10*uNorm;

        u = min(max(u,0),10);

        e_true = r - h2;

        h1Arr(kk) = h1;
        h2Arr(kk) = h2;
        h2MeasArr(kk) = h2Meas;
        uArr(kk) = u;
        eArr(kk) = e_true;
        deArr(kk) = de_meas;
        IeArr(kk) = Ie;

        x = plantStepScenario(x,u,dt,k,ku,hMax,scenario,currentTime);

        Ie = Ie + dt*e_meas;
        ePrev = e_meas;
        uPrev = u;
    end
end


function [h1Arr,h2Arr,h2MeasArr,uArr,eArr,deArr,IeArr] = ...
    simulateSavedOFNNControllerReferenceProfile( ...
        net, rProfile, x0, Nsim, dt, k, ku, hMax, scenario)

    h1Arr = zeros(Nsim,1);
    h2Arr = zeros(Nsim,1);
    h2MeasArr = zeros(Nsim,1);
    uArr  = zeros(Nsim,1);
    eArr  = zeros(Nsim,1);
    deArr = zeros(Nsim,1);
    IeArr = zeros(Nsim,1);

    x = x0;
    Ie = 0;
    r0 = rProfile(1);
    ePrev = r0 - x(2);
    uPrev = steadyStateInput(r0,k,ku);

    for kk = 1:Nsim

        currentTime = (kk-1)*dt;
        r = rProfile(kk);

        h1 = x(1);
        h2 = x(2);

        h2Meas = getMeasuredOutput(h2, hMax, scenario);

        e_meas = r - h2Meas;
        de_meas = (e_meas - ePrev)/dt;

        phi = makeFeaturesOF(h2Meas,r,e_meas,de_meas,Ie,uPrev);

        uNorm = net(phi);
        u = 10*uNorm;
        u = min(max(u,0),10);

        e_true = r - h2;

        h1Arr(kk) = h1;
        h2Arr(kk) = h2;
        h2MeasArr(kk) = h2Meas;
        uArr(kk) = u;
        eArr(kk) = e_true;
        deArr(kk) = de_meas;
        IeArr(kk) = Ie;

        x = plantStepScenario(x,u,dt,k,ku,hMax,scenario,currentTime);

        Ie = Ie + dt*e_meas;
        ePrev = e_meas;
        uPrev = u;
    end
end


function h2Meas = getMeasuredOutput(h2, hMax, scenario)

    noiseStd = 0;

    if strcmp(scenario,'measurement_noise')
        noiseStd = 0.002;    % 2 mm sensor noise
    elseif strcmp(scenario,'combined_disturbance')
        noiseStd = 0.002;    % 2 mm sensor noise
    end

    h2Meas = h2 + noiseStd*randn;
    h2Meas = min(max(h2Meas,0),hMax);
end


function xNext = plantStepScenario(x, u, dt, k, ku, hMax, scenario, currentTime)

    h1 = max(x(1),0);
    h2 = max(x(2),0);

    u = min(max(u,0),10);

    outflowMultiplier = 1.0;
    pumpMultiplier = 1.0;

    if strcmp(scenario,'outflow_disturbance')
        if currentTime >= 120
            outflowMultiplier = 1.20;
        end

    elseif strcmp(scenario,'pump_loss')
        if currentTime >= 120
            pumpMultiplier = 0.85;
        end

    elseif strcmp(scenario,'combined_disturbance')
        if currentTime >= 120
            outflowMultiplier = 1.20;
            pumpMultiplier = 0.85;
        end
    end

    flow12 = k * sign(h1 - h2) * sqrt(abs(h1 - h2));
    flowOut = outflowMultiplier * k * sqrt(max(h2,0));

    dh1 = -flow12 + pumpMultiplier*ku*u;
    dh2 =  flow12 - flowOut;

    h1n = h1 + dt*dh1;
    h2n = h2 + dt*dh2;

    h1n = min(max(h1n,0),hMax);
    h2n = min(max(h2n,0),hMax);

    xNext = [h1n; h2n];
end


function phi = makeFeaturesOF(h2,r,e,de,Ie,uPrev)

    phi = [
        h2/0.6;
        r/0.6;
        e/0.3;
        de/0.02;
        Ie/40;
        uPrev/10
    ];
end


function uSS = steadyStateInput(r,k,ku)

    r = max(r,1e-6);
    uSS = k*sqrt(r)/ku;
    uSS = min(max(uSS,0),10);
end


function metrics = computeMetrics(t,h2,u,r)

    e = r - h2;

    metrics.finalError = abs(r - h2(end));
    metrics.ISE = trapz(t,e.^2);
    metrics.IAE = trapz(t,abs(e));
    metrics.overshoot = max(0,max(h2)-r);
    metrics.maxU = max(u);
    metrics.minU = min(u);

    h2_initial = h2(1);
    target90 = h2_initial + 0.9*(r - h2_initial);

    idxRise = find(h2 >= target90,1,'first');
    if isempty(idxRise)
        metrics.riseTime90 = NaN;
    else
        metrics.riseTime90 = t(idxRise);
    end

    band = 0.02*r;
    metrics.settlingTime = NaN;

    for i = 1:length(t)
        if all(abs(h2(i:end)-r) <= band)
            metrics.settlingTime = t(i);
            break;
        end
    end
end


function printMetrics(metrics,r,h2,u)

    fprintf('Reference h2          = %.4f m\n', r);
    fprintf('Final h2              = %.4f m\n', h2(end));
    fprintf('Final absolute error  = %.6f m\n', metrics.finalError);
    fprintf('Overshoot             = %.6f m\n', metrics.overshoot);
    fprintf('ISE                   = %.6f\n', metrics.ISE);
    fprintf('IAE                   = %.6f\n', metrics.IAE);
    fprintf('90%% rise time          = %.2f s\n', metrics.riseTime90);
    fprintf('2%% settling time       = %.2f s\n', metrics.settlingTime);
    fprintf('Maximum control u     = %.4f V\n', max(u));
    fprintf('Minimum control u     = %.4f V\n', min(u));
end