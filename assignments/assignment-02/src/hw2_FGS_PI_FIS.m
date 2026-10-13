%% KON426E - Intelligent Control Systems
% Assignment 2 - Question 2 Part 2
% Construction of Fuzzy Gain-Scheduling PI Controller

clear; clc; close all;

%% Create Mamdani FIS

fis = mamfis('Name', 'FGS_PI_Controller');

% Inference settings required by the assignment
fis.AndMethod = 'min';
fis.OrMethod = 'max';
fis.ImplicationMethod = 'min';
fis.AggregationMethod = 'max';
fis.DefuzzificationMethod = 'centroid';

%% ========================================================================
% Input 1: Normalized error E in [-1, 1]
% ========================================================================

fis = addInput(fis, [-1 1], 'Name', 'E');

fis = addMF(fis, 'E', 'trimf', [-1.5 -1.0 -0.5], 'Name', 'NB');
fis = addMF(fis, 'E', 'trimf', [-1.0 -0.5  0.0], 'Name', 'NS');
fis = addMF(fis, 'E', 'trimf', [-0.5  0.0  0.5], 'Name', 'Z');
fis = addMF(fis, 'E', 'trimf', [ 0.0  0.5  1.0], 'Name', 'PS');
fis = addMF(fis, 'E', 'trimf', [ 0.5  1.0  1.5], 'Name', 'PB');

%% ========================================================================
% Input 2: Normalized derivative of error dE in [-1, 1]
% ========================================================================

fis = addInput(fis, [-1 1], 'Name', 'dE');

fis = addMF(fis, 'dE', 'trimf', [-1.5 -1.0 -0.5], 'Name', 'NB');
fis = addMF(fis, 'dE', 'trimf', [-1.0 -0.5  0.0], 'Name', 'NS');
fis = addMF(fis, 'dE', 'trimf', [-0.5  0.0  0.5], 'Name', 'Z');
fis = addMF(fis, 'dE', 'trimf', [ 0.0  0.5  1.0], 'Name', 'PS');
fis = addMF(fis, 'dE', 'trimf', [ 0.5  1.0  1.5], 'Name', 'PB');

%% ========================================================================
% Output 1: Normalized proportional gain Kp' in [0, 1]
% ========================================================================

fis = addOutput(fis, [0 1], 'Name', 'Kp_norm');

fis = addMF(fis, 'Kp_norm', 'trimf', [-0.25 0.00 0.25], 'Name', 'S');
fis = addMF(fis, 'Kp_norm', 'trimf', [ 0.00 0.25 0.50], 'Name', 'MS');
fis = addMF(fis, 'Kp_norm', 'trimf', [ 0.25 0.50 0.75], 'Name', 'M');
fis = addMF(fis, 'Kp_norm', 'trimf', [ 0.50 0.75 1.00], 'Name', 'MB');
fis = addMF(fis, 'Kp_norm', 'trimf', [ 0.75 1.00 1.25], 'Name', 'B');

%% ========================================================================
% Output 2: Normalized integral gain Ki' in [0, 1]
% ========================================================================

fis = addOutput(fis, [0 1], 'Name', 'Ki_norm');

fis = addMF(fis, 'Ki_norm', 'trimf', [-0.25 0.00 0.25], 'Name', 'S');
fis = addMF(fis, 'Ki_norm', 'trimf', [ 0.00 0.25 0.50], 'Name', 'MS');
fis = addMF(fis, 'Ki_norm', 'trimf', [ 0.25 0.50 0.75], 'Name', 'M');
fis = addMF(fis, 'Ki_norm', 'trimf', [ 0.50 0.75 1.00], 'Name', 'MB');
fis = addMF(fis, 'Ki_norm', 'trimf', [ 0.75 1.00 1.25], 'Name', 'B');

%% ========================================================================
% Rule Base
% ========================================================================
% MF index convention:
% Inputs E and dE:
% 1 = NB, 2 = NS, 3 = Z, 4 = PS, 5 = PB
%
% Outputs Kp_norm and Ki_norm:
% 1 = S, 2 = MS, 3 = M, 4 = MB, 5 = B
%
% Rule format:
% [E_index dE_index Kp_index Ki_index weight AND_or_OR]
%
% Important:
% The table in the assignment is organized as rows = dE and columns = E.
% addRule uses input order [E dE], therefore each rule is entered as:
% IF E is column_label AND dE is row_label THEN ...

% Kp rule table, rows dE = NB NS Z PS PB, columns E = NB NS Z PS PB
Kp_table = [
    5 5 4 3 1;
    5 4 3 1 3;
    4 3 1 3 4;
    3 1 3 4 5;
    1 3 4 5 5
];

% Ki rule table, rows dE = NB NS Z PS PB, columns E = NB NS Z PS PB
Ki_table = [
    1 1 1 3 5;
    1 1 3 5 3;
    1 3 5 3 1;
    3 5 3 1 1;
    5 3 1 1 1
];

ruleList = [];

for dE_idx = 1:5
    for E_idx = 1:5

        Kp_idx = Kp_table(dE_idx, E_idx);
        Ki_idx = Ki_table(dE_idx, E_idx);

        weight = 1;
        AND_connection = 1;

        ruleList = [ruleList;
                    E_idx dE_idx Kp_idx Ki_idx weight AND_connection];
    end
end

fis = addRule(fis, ruleList);

%% Display rules

disp('FGS-PI Fuzzy Rule Base:');
showrule(fis)

%% Save FIS

writeFIS(fis, 'FGS_PI_Controller.fis');
disp('FIS saved as FGS_PI_Controller.fis');

%% ========================================================================
% Membership function plots
% ========================================================================

figure;
plotmf(fis, 'input', 1);
grid on;
title('Membership Functions for Normalized Error E');

figure;
plotmf(fis, 'input', 2);
grid on;
title('Membership Functions for Normalized Error Derivative \DeltaE');

figure;
plotmf(fis, 'output', 1);
grid on;
title('Membership Functions for Normalized Proportional Gain K''_p');

figure;
plotmf(fis, 'output', 2);
grid on;
title('Membership Functions for Normalized Integral Gain K''_i');

%% ========================================================================
% 3D Surface Plots
% ========================================================================

figure;
gensurf(fis, [1 2], 1);
xlabel('E');
ylabel('\DeltaE');
zlabel('K''_p');
title('FIS Surface for Normalized Proportional Gain K''_p');
grid on;

figure;
gensurf(fis, [1 2], 2);
xlabel('E');
ylabel('\DeltaE');
zlabel('K''_i');
title('FIS Surface for Normalized Integral Gain K''_i');
grid on;