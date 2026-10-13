clear; clc; close all;
rng(7);

%% ================================================================
%  Robust Full-State Neural Approximate Nonlinear MPC Controller
%  for Nonlinear Double Tank System
%
%  Final NN controller:
%
%       u(k) = sat( u_ff(r) + 5*NN(h1,h2,r,e,de,Ie,uPrev) )
%
%  The NN learns the correction around physics-based steady-state
%  feedforward input:
%
%       u_ff(r) = k*sqrt(r)/ku
%
%  Training:
%       Offline nonlinear MPC teacher generates data under randomized:
%           - references
%           - initial conditions
%           - outflow disturbance multipliers
%           - pump efficiency multipliers
%
%  Final tests:
%       - different references
%       - different initial conditions
%       - measurement noise
%       - outflow disturbance
%       - pump efficiency loss
%       - combined disturbance
%       - changing reference profile
% ================================================================

%% ---------------- Plant constants ----------------
k  = 1.035454e-2;
ku = 1.143e-3;

hMax = 0.6;
uMin = 0;
uMax = 10;

%% ---------------- Simulation settings ----------------
dt = 2.0;
Tend = 300;
Nsim = round(Tend/dt) + 1;
t = (0:Nsim-1)'*dt;

%% ---------------- Robust NMPC teacher settings ----------------
Np = 36;              % 36*2 = 72 s prediction horizon
Nc = 8;               % control horizon

mpc.Np = Np;
mpc.Nc = Nc;
mpc.dt = dt;
mpc.k = k;
mpc.ku = ku;
mpc.uMin = uMin;
mpc.uMax = uMax;
mpc.hMax = hMax;

% Cost weights
mpc.Q     = 900;       % h2 tracking
mpc.QT    = 12000;     % terminal h2 tracking
mpc.Qh1   = 90;        % h1 steady-state guidance
mpc.QTh1  = 1500;      % terminal h1 guidance
mpc.Qlate = 7000;      % late tracking
mpc.Qrise = 5000;      % rise-time penalty
mpc.R     = 0.0004;    % input effort around uSS
mpc.Rd    = 0.030;     % input smoothness
mpc.Qos   = 700;       % overshoot penalty

mpc.riseStartIndex = round(75/dt) + 1;
mpc.overshootTolerance = 0.002;

%% ================================================================
%  PART 1 — Randomized robust NMPC teacher dataset
% ================================================================

generateNewDataset = true;
datasetFile = 'Robust_FullState_NN_MPC_Dataset.mat';

% For first trial use 12-16 episodes.
% For final report use 30-50 episodes if time allows.
numEpisodes = 16;

if generateNewDataset

    fprintf('\n===== GENERATING ROBUST FULL-STATE NMPC DATASET =====\n');
    fprintf('Number of randomized training episodes = %d\n', numEpisodes);

    Xdata = [];
    Ydata = [];

    episodeData = struct();

    for ep = 1:numEpisodes

        % Random reference inside useful operating region
        r = 0.16 + (0.34 - 0.16)*rand;

        % Random physically plausible initial conditions
        h2_0 = 0.02 + (0.18 - 0.02)*rand;
        h1_0 = h2_0 + 0.25*rand;
        h1_0 = min(h1_0, 0.40);

        x = [h1_0; h2_0];

        % Random plant multipliers for robust training
        outflowSet = [0.90 1.00 1.10 1.20];
        pumpSet    = [0.85 0.90 1.00 1.05];

        outflowMultiplier = outflowSet(randi(numel(outflowSet)));
        pumpMultiplier    = pumpSet(randi(numel(pumpSet)));

        % Optional measurement noise during feature generation
        noiseSet = [0.000 0.001 0.002];
        noiseStd = noiseSet(randi(numel(noiseSet)));

        fprintf('\n--- Episode %d/%d ---\n', ep, numEpisodes);
        fprintf('r = %.4f, x0 = [%.4f %.4f], outflowMult = %.2f, pumpMult = %.2f, noiseStd = %.4f\n', ...
            r, h1_0, h2_0, outflowMultiplier, pumpMultiplier, noiseStd);

        Ie = 0;
        ePrev = r - x(2);
        uPrev = steadyStateInput(r,k,ku);
        uSS = steadyStateInput(r,k,ku);

        nHigh = ceil(0.45*Nc);
        Uguess = [10*ones(nHigh,1); uSS*ones(Nc-nHigh,1)];

        h1_log = zeros(Nsim,1);
        h2_log = zeros(Nsim,1);
        u_log  = zeros(Nsim,1);
        r_log  = r*ones(Nsim,1);
        e_log  = zeros(Nsim,1);
        de_log = zeros(Nsim,1);

        for kk = 1:Nsim

            h1 = x(1);
            h2 = x(2);

            % Noisy measurement only for features.
            % Teacher still uses true current state for prediction.
            h2Meas = h2 + noiseStd*randn;
            h2Meas = min(max(h2Meas,0),hMax);

            e = r - h2Meas;
            de = (e - ePrev)/dt;

            % Solve disturbance-aware NMPC teacher.
            % The teacher uses the episode's perturbed plant model.
            [uMPC, Uopt] = solveNMPC(x, r, uPrev, Uguess, mpc, ...
                outflowMultiplier, pumpMultiplier);

            % Physics-based feedforward
            uFF = steadyStateInput(r,k,ku);

            % Target is correction around feedforward
            y = (uMPC - uFF)/5;
            y = min(max(y,-1),1);

            % Full-state robust feature vector
            phi = makeFeaturesFS(h1,h2Meas,r,e,de,Ie,uPrev);

            Xdata = [Xdata, phi];
            Ydata = [Ydata, y];

            h1_log(kk) = h1;
            h2_log(kk) = h2;
            u_log(kk)  = uMPC;
            e_log(kk)  = r - h2;
            de_log(kk) = de;

            % Simulate perturbed plant
            x = plantStepMultiplier(x,uMPC,dt,k,ku,hMax,outflowMultiplier,pumpMultiplier);

            Ie = Ie + dt*e;
            ePrev = e;
            uPrev = uMPC;

            Uguess = [Uopt(2:end); Uopt(end)];

            if mod(kk,40) == 0
                fprintf('Step %3d/%3d | h2 = %.4f | uMPC = %.4f\n', ...
                    kk, Nsim, h2, uMPC);
            end
        end

        episodeData(ep).r = r;
        episodeData(ep).x0 = [h1_0; h2_0];
        episodeData(ep).outflowMultiplier = outflowMultiplier;
        episodeData(ep).pumpMultiplier = pumpMultiplier;
        episodeData(ep).noiseStd = noiseStd;
        episodeData(ep).t = t;
        episodeData(ep).h1 = h1_log;
        episodeData(ep).h2 = h2_log;
        episodeData(ep).u = u_log;
        episodeData(ep).e = e_log;
        episodeData(ep).de = de_log;
    end

    fprintf('\nTotal training samples = %d\n', size(Xdata,2));

    save(datasetFile,'Xdata','Ydata','episodeData','mpc','numEpisodes');
    fprintf('Dataset saved as %s\n', datasetFile);

else

    load(datasetFile,'Xdata','Ydata','episodeData','mpc','numEpisodes');
    fprintf('\nLoaded dataset: %s\n', datasetFile);

end

%% ================================================================
%  PART 2 — Train robust full-state NN controller
% ================================================================

fprintf('\n===== TRAINING ROBUST FULL-STATE NN CONTROLLER =====\n');

% Input:
%   [h1, h2, r, e, de, Ie, uPrev]
%
% Target:
%   y = (uMPC - uFF)/5

% Bayesian regularization generally improves generalization.
% If trainbr is too slow, replace with:
%   net = fitnet([24 16 8],'trainlm');
%   net.performParam.regularization = 0.1;

hiddenLayers = [24 16 8];
net = fitnet(hiddenLayers,'trainbr');

net.trainParam.epochs = 800;
net.trainParam.goal = 1e-7;
net.trainParam.min_grad = 1e-9;
net.trainParam.showWindow = true;

% For trainbr, MATLAB may ignore validation division internally.
net.divideParam.trainRatio = 0.80;
net.divideParam.valRatio   = 0.10;
net.divideParam.testRatio  = 0.10;

[net,tr] = train(net,Xdata,Ydata);

Ypred = net(Xdata);
trainMSE = mean((Ydata - Ypred).^2);

fprintf('\nTraining MSE on complete dataset = %.10f\n', trainMSE);

figure;
plotperform(tr);
title('Robust Full-State NN Controller Training Performance');

figure;
plot(Ydata,'LineWidth',1.2); hold on;
plot(Ypred,'--','LineWidth',1.2);
grid on;
xlabel('Sample');
ylabel('Normalized Correction y');
legend('Teacher target','NN prediction','Location','best');
title('NN Approximation of Feedforward-Corrected NMPC Policy');

figure;
plot(steadyTargetToVoltage(Ydata, Xdata, k, ku),'LineWidth',1.2); hold on;
plot(steadyTargetToVoltage(Ypred, Xdata, k, ku),'--','LineWidth',1.2);
grid on;
xlabel('Sample');
ylabel('Approximate Control Input [V]');
legend('Teacher-equivalent','NN-equivalent','Location','best');
title('Control Signal Approximation Quality');

%% ================================================================
%  PART 3 — Extended closed-loop tests
% ================================================================

fprintf('\n===== EXTENDED TESTS: ROBUST FULL-STATE NN CONTROLLER =====\n');

x0Main = [0.05;0.05];
rMain = 0.25;

%% Test A: Different references
referenceList = [0.18 0.20 0.25 0.30 0.33];
refResults = struct();

fprintf('\n===== TEST A: DIFFERENT REFERENCES =====\n');

figure; hold on; grid on;

for i = 1:numel(referenceList)

    r = referenceList(i);

    [h1,h2,h2m,u,e,de,Ie] = simulateFSNNController( ...
        net,r,x0Main,Nsim,dt,k,ku,hMax,'nominal');

    metrics = computeMetrics(t,h2,u,r);

    refResults(i).r = r;
    refResults(i).h1 = h1;
    refResults(i).h2 = h2;
    refResults(i).h2Meas = h2m;
    refResults(i).u = u;
    refResults(i).e = e;
    refResults(i).de = de;
    refResults(i).Ie = Ie;
    refResults(i).metrics = metrics;

    plot(t,h2,'LineWidth',1.5);

    fprintf('\n--- Reference r = %.3f m ---\n', r);
    printMetrics(metrics,r,h2,u);
end

for i = 1:numel(referenceList)
    plot(t,referenceList(i)*ones(size(t)),'--','LineWidth',1.0);
end

xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
title('Robust Full-State NN Controller: Different References');
legend('h2 r=0.18','h2 r=0.20','h2 r=0.25','h2 r=0.30','h2 r=0.33', ...
       'ref 0.18','ref 0.20','ref 0.25','ref 0.30','ref 0.33', ...
       'Location','best');

%% Test B: Different initial conditions
initialList = [
    0.03 0.03;
    0.05 0.05;
    0.08 0.04;
    0.10 0.06;
    0.15 0.10
];

icResults = struct();

fprintf('\n===== TEST B: DIFFERENT INITIAL CONDITIONS =====\n');

figure; hold on; grid on;

for i = 1:size(initialList,1)

    x0 = initialList(i,:)';

    [h1,h2,h2m,u,e,de,Ie] = simulateFSNNController( ...
        net,rMain,x0,Nsim,dt,k,ku,hMax,'nominal');

    metrics = computeMetrics(t,h2,u,rMain);

    icResults(i).x0 = x0;
    icResults(i).h1 = h1;
    icResults(i).h2 = h2;
    icResults(i).h2Meas = h2m;
    icResults(i).u = u;
    icResults(i).e = e;
    icResults(i).de = de;
    icResults(i).Ie = Ie;
    icResults(i).metrics = metrics;

    plot(t,h2,'LineWidth',1.5);

    fprintf('\n--- Initial x0 = [%.3f %.3f] ---\n', x0(1), x0(2));
    printMetrics(metrics,rMain,h2,u);
end

plot(t,rMain*ones(size(t)),'k--','LineWidth',1.8);
xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
title('Robust Full-State NN Controller: Different Initial Conditions');
legend('x0=[0.03,0.03]','x0=[0.05,0.05]','x0=[0.08,0.04]', ...
       'x0=[0.10,0.06]','x0=[0.15,0.10]','Reference', ...
       'Location','best');

%% Test C: Noise and disturbance handling
scenarioList = {'nominal','measurement_noise','outflow_disturbance','pump_loss','combined_disturbance'};
scenarioNames = {'Nominal','Measurement Noise','Outflow Disturbance','Pump Efficiency Loss','Combined Disturbance'};

scenarioResults = struct();

fprintf('\n===== TEST C: NOISE AND DISTURBANCE HANDLING =====\n');

figure; hold on; grid on;

for i = 1:numel(scenarioList)

    scenario = scenarioList{i};

    [h1,h2,h2m,u,e,de,Ie] = simulateFSNNController( ...
        net,rMain,x0Main,Nsim,dt,k,ku,hMax,scenario);

    metrics = computeMetrics(t,h2,u,rMain);

    scenarioResults(i).scenario = scenario;
    scenarioResults(i).name = scenarioNames{i};
    scenarioResults(i).h1 = h1;
    scenarioResults(i).h2 = h2;
    scenarioResults(i).h2Meas = h2m;
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
title('Robust Full-State NN Controller: Disturbance Handling');
legend('Nominal','Measurement Noise','Outflow Disturbance', ...
       'Pump Efficiency Loss','Combined Disturbance','Reference', ...
       'Location','best');

figure; hold on; grid on;
for i = 1:numel(scenarioList)
    plot(t,scenarioResults(i).u,'LineWidth',1.3);
end
xlabel('Time [s]');
ylabel('Control Input u(t) [V]');
ylim([-0.5 10.5]);
title('Control Inputs under Disturbance Scenarios');
legend('Nominal','Measurement Noise','Outflow Disturbance', ...
       'Pump Efficiency Loss','Combined Disturbance','Location','best');

%% Test D: Changing reference profile with fair timing
TendProfile = 540;
Nprof = round(TendProfile/dt)+1;
tProf = (0:Nprof-1)'*dt;

rProfile = zeros(Nprof,1);

for kk = 1:Nprof
    currentTime = tProf(kk);

    if currentTime < 180
        rProfile(kk) = 0.20;
    elseif currentTime < 360
        rProfile(kk) = 0.25;
    else
        rProfile(kk) = 0.30;
    end
end

[h1p,h2p,h2mp,up,ep,dep,Iep] = simulateFSNNControllerProfile( ...
    net,rProfile,x0Main,Nprof,dt,k,ku,hMax,'nominal');

fprintf('\n===== TEST D: CHANGING REFERENCE PROFILE =====\n');
fprintf('Reference profile: 0.20 m -> 0.25 m -> 0.30 m, with 180 s intervals.\n');

figure;
plot(tProf,h2p,'LineWidth',1.8); hold on;
plot(tProf,rProfile,'k--','LineWidth',1.6);
grid on;
xlabel('Time [s]');
ylabel('Tank 2 Level h_2 [m]');
legend('h_2(t)','r(t)','Location','best');
title('Robust Full-State NN Controller: Changing Reference Tracking');

figure;
plot(tProf,up,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Control Input u(t) [V]');
ylim([-0.5 10.5]);
title('Control Input for Changing Reference Tracking');

figure;
plot(tProf,ep,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Tracking Error e(t) [m]');
title('Tracking Error for Changing Reference Tracking');

%% Save final controller and tests
results.net = net;
results.tr = tr;
results.trainMSE = trainMSE;
results.Xdata = Xdata;
results.Ydata = Ydata;
results.Ypred = Ypred;
results.episodeData = episodeData;
results.mpc = mpc;
results.dt = dt;
results.Tend = Tend;
results.t = t;
results.refResults = refResults;
results.icResults = icResults;
results.scenarioResults = scenarioResults;
results.profile.t = tProf;
results.profile.r = rProfile;
results.profile.h1 = h1p;
results.profile.h2 = h2p;
results.profile.u = up;
results.profile.e = ep;
results.profile.de = dep;
results.profile.Ie = Iep;

save('Robust_FullState_NN_based_MPC_Final.mat','results');

fprintf('\nSaved final robust full-state NN controller: Robust_FullState_NN_based_MPC_Final.mat\n');

%% ================================================================
%  Local functions
% ================================================================

function phi = makeFeaturesFS(h1,h2,r,e,de,Ie,uPrev)

    phi = [
        h1/0.6;
        h2/0.6;
        r/0.6;
        e/0.35;
        de/0.03;
        Ie/60;
        uPrev/10
    ];
end


function [u, yNN] = nnControlFS(net,h1,h2,r,e,de,Ie,uPrev,k,ku)

    phi = makeFeaturesFS(h1,h2,r,e,de,Ie,uPrev);

    yNN = net(phi);
    yNN = min(max(yNN,-1),1);

    uFF = steadyStateInput(r,k,ku);

    u = uFF + 5*yNN;
    u = min(max(u,0),10);
end


function [h1Arr,h2Arr,h2MeasArr,uArr,eArr,deArr,IeArr] = simulateFSNNController( ...
    net,r,x0,Nsim,dt,k,ku,hMax,scenario)

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

        h2Meas = getMeasuredOutput(h2,hMax,scenario);

        e = r - h2Meas;
        de = (e - ePrev)/dt;

        [u,~] = nnControlFS(net,h1,h2Meas,r,e,de,Ie,uPrev,k,ku);

        h1Arr(kk) = h1;
        h2Arr(kk) = h2;
        h2MeasArr(kk) = h2Meas;
        uArr(kk) = u;
        eArr(kk) = r - h2;
        deArr(kk) = de;
        IeArr(kk) = Ie;

        [outflowMultiplier,pumpMultiplier] = scenarioMultipliers(scenario,currentTime);

        x = plantStepMultiplier(x,u,dt,k,ku,hMax,outflowMultiplier,pumpMultiplier);

        Ie = Ie + dt*e;
        ePrev = e;
        uPrev = u;
    end
end


function [h1Arr,h2Arr,h2MeasArr,uArr,eArr,deArr,IeArr] = simulateFSNNControllerProfile( ...
    net,rProfile,x0,Nsim,dt,k,ku,hMax,scenario)

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

        h2Meas = getMeasuredOutput(h2,hMax,scenario);

        e = r - h2Meas;
        de = (e - ePrev)/dt;

        [u,~] = nnControlFS(net,h1,h2Meas,r,e,de,Ie,uPrev,k,ku);

        h1Arr(kk) = h1;
        h2Arr(kk) = h2;
        h2MeasArr(kk) = h2Meas;
        uArr(kk) = u;
        eArr(kk) = r - h2;
        deArr(kk) = de;
        IeArr(kk) = Ie;

        [outflowMultiplier,pumpMultiplier] = scenarioMultipliers(scenario,currentTime);

        x = plantStepMultiplier(x,u,dt,k,ku,hMax,outflowMultiplier,pumpMultiplier);

        Ie = Ie + dt*e;
        ePrev = e;
        uPrev = u;
    end
end


function h2Meas = getMeasuredOutput(h2,hMax,scenario)

    noiseStd = 0;

    if strcmp(scenario,'measurement_noise')
        noiseStd = 0.002;
    elseif strcmp(scenario,'combined_disturbance')
        noiseStd = 0.002;
    end

    h2Meas = h2 + noiseStd*randn;
    h2Meas = min(max(h2Meas,0),hMax);
end


function [outflowMultiplier,pumpMultiplier] = scenarioMultipliers(scenario,currentTime)

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
end


function xNext = plantStepMultiplier(x,u,dt,k,ku,hMax,outflowMultiplier,pumpMultiplier)

    h1 = max(x(1),0);
    h2 = max(x(2),0);

    u = min(max(u,0),10);

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


function [uMPC,Uopt] = solveNMPC(x0,r,uPrev,Uguess,mpc,outflowMultiplier,pumpMultiplier)

    Nc = mpc.Nc;

    lb = mpc.uMin*ones(Nc,1);
    ub = mpc.uMax*ones(Nc,1);

    if isempty(Uguess)
        uSS = steadyStateInput(r,mpc.k,mpc.ku);
        nHigh = ceil(0.45*Nc);
        Uguess = [10*ones(nHigh,1); uSS*ones(Nc-nHigh,1)];
    end

    options = optimoptions('fmincon', ...
        'Display','off', ...
        'Algorithm','sqp', ...
        'MaxIterations',45, ...
        'MaxFunctionEvaluations',900, ...
        'OptimalityTolerance',8e-4, ...
        'StepTolerance',1e-5);

    costFun = @(U) nmpcCostRobust(U,x0,r,uPrev,mpc,outflowMultiplier,pumpMultiplier);

    try
        Uopt = fmincon(costFun,Uguess,[],[],[],[],lb,ub,[],options);
    catch
        warning('fmincon failed. Using previous guess.');
        Uopt = Uguess;
    end

    uMPC = Uopt(1);
    uMPC = min(max(uMPC,mpc.uMin),mpc.uMax);
end


function J = nmpcCostRobust(U,x0,r,uPrev,mpc,outflowMultiplier,pumpMultiplier)

    x = x0;
    J = 0;

    Np = mpc.Np;
    Nc = mpc.Nc;

    uLast = uPrev;
    uSS = steadyStateInput(r,mpc.k,mpc.ku);
    h1SS = 2*r;

    lateStartIndex = round(0.60*Np);

    for i = 1:Np

        if i <= Nc
            u = U(i);
        else
            u = U(end);
        end

        x = plantStepMultiplier(x,u,mpc.dt,mpc.k,mpc.ku,mpc.hMax, ...
            outflowMultiplier,pumpMultiplier);

        h1 = x(1);
        h2 = x(2);

        e = r - h2;

        % Main h2 tracking
        J = J + mpc.Q*e^2;

        % h1 guidance around physical steady-state relation h1 = 2r
        J = J + mpc.Qh1*(h1 - h1SS)^2;

        % Input effort around uSS
        J = J + mpc.R*(u - uSS)^2;

        % Smooth input
        J = J + mpc.Rd*(u - uLast)^2;

        % Late tracking
        if i > lateStartIndex
            J = J + mpc.Qlate*e^2;
        end

        % Rise-time penalty
        if i > mpc.riseStartIndex
            belowReference = max(r - h2,0);
            J = J + mpc.Qrise*belowReference^2;
        end

        % Overshoot penalty
        overshoot = max(h2 - r - mpc.overshootTolerance,0);
        J = J + mpc.Qos*overshoot^2;

        uLast = u;
    end

    terminalErrorH2 = r - x(2);
    terminalErrorH1 = h1SS - x(1);

    J = J + mpc.QT*terminalErrorH2^2;
    J = J + mpc.QTh1*terminalErrorH1^2;
end


function uSS = steadyStateInput(r,k,ku)

    r = max(r,1e-6);
    uSS = k*sqrt(r)/ku;
    uSS = min(max(uSS,0),10);
end


function uApprox = steadyTargetToVoltage(yNorm,Xdata,k,ku)

    % Xdata row 3 contains r/0.6
    rVec = Xdata(3,:)*0.6;

    uFF = zeros(size(rVec));

    for i = 1:numel(rVec)
        uFF(i) = steadyStateInput(rVec(i),k,ku);
    end

    uApprox = uFF + 5*yNorm;
    uApprox = min(max(uApprox,0),10);
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