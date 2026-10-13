%% KON426E - Intelligent Control Systems
% Assignment 2 - Question 3 Part 2
% Mamdani FIS for USV Supervisory Speed Control

clear; clc; close all;

%% Create Mamdani FIS

fis = mamfis('Name', 'USV_Speed_Controller');

fis.AndMethod = 'min';
fis.OrMethod = 'max';
fis.ImplicationMethod = 'min';
fis.AggregationMethod = 'max';
fis.DefuzzificationMethod = 'centroid';

%% ========================================================================
% Input 1: Distance to waypoint d in [0, 20] m
% ========================================================================

fis = addInput(fis, [0 20], 'Name', 'Distance');

fis = addMF(fis, 'Distance', 'trimf', [-2 0 2],      'Name', 'Near');
fis = addMF(fis, 'Distance', 'trimf', [0.5 3 6],     'Name', 'Medium');
fis = addMF(fis, 'Distance', 'trimf', [4 8 12],      'Name', 'Far');
fis = addMF(fis, 'Distance', 'trimf', [10 20 30],    'Name', 'VeryFar');

%% ========================================================================
% Input 2: Absolute heading error |Delta theta| in [0, pi] rad
% ========================================================================

fis = addInput(fis, [0 pi], 'Name', 'HeadingError');

fis = addMF(fis, 'HeadingError', 'trimf', [-0.2 0 0.45],       'Name', 'Small');
fis = addMF(fis, 'HeadingError', 'trimf', [0.25 0.8 1.35],     'Name', 'Medium');
fis = addMF(fis, 'HeadingError', 'trimf', [1.1 1.75 2.4],      'Name', 'Large');
fis = addMF(fis, 'HeadingError', 'trimf', [2.1 pi 3.6],        'Name', 'VeryLarge');

%% ========================================================================
% Output: Linear speed v in [0, 2] m/s
% ========================================================================

fis = addOutput(fis, [0 2], 'Name', 'Speed');

fis = addMF(fis, 'Speed', 'trimf', [-0.2 0.0 0.3],    'Name', 'VerySlow');
fis = addMF(fis, 'Speed', 'trimf', [0.15 0.45 0.8],   'Name', 'Slow');
fis = addMF(fis, 'Speed', 'trimf', [0.6 1.0 1.4],     'Name', 'Medium');
fis = addMF(fis, 'Speed', 'trimf', [1.2 1.6 2.0],     'Name', 'Fast');
fis = addMF(fis, 'Speed', 'trimf', [1.7 2.0 2.3],     'Name', 'VeryFast');

%% ========================================================================
% Rule Base
% ========================================================================
% Input indices:
% Distance:     1 Near, 2 Medium, 3 Far, 4 VeryFar
% HeadingError: 1 Small, 2 Medium, 3 Large, 4 VeryLarge
%
% Output Speed:
% 1 VerySlow, 2 Slow, 3 Medium, 4 Fast, 5 VeryFast
%
% Rule format:
% [Distance_index HeadingError_index Speed_index weight AND_or_OR]

ruleList = [
    % Distance Near
    1 1 2 1 1;   % Near and Small heading error      -> Slow
    1 2 2 1 1;   % Near and Medium heading error     -> Slow
    1 3 1 1 1;   % Near and Large heading error      -> VerySlow
    1 4 1 1 1;   % Near and VeryLarge heading error  -> VerySlow

    % Distance Medium
    2 1 4 1 1;   % Medium and Small heading error     -> Fast
    2 2 3 1 1;   % Medium and Medium heading error    -> Medium
    2 3 2 1 1;   % Medium and Large heading error     -> Slow
    2 4 1 1 1;   % Medium and VeryLarge heading error -> VerySlow

    % Distance Far
    3 1 5 1 1;   % Far and Small heading error        -> VeryFast
    3 2 4 1 1;   % Far and Medium heading error       -> Fast
    3 3 3 1 1;   % Far and Large heading error        -> Medium
    3 4 2 1 1;   % Far and VeryLarge heading error    -> Slow

    % Distance VeryFar
    4 1 5 1 1;   % VeryFar and Small heading error     -> VeryFast
    4 2 5 1 1;   % VeryFar and Medium heading error    -> VeryFast
    4 3 4 1 1;   % VeryFar and Large heading error     -> Fast
    4 4 3 1 1;   % VeryFar and VeryLarge heading error -> Medium
];

fis = addRule(fis, ruleList);

%% Display rules

disp('USV Speed Controller Rule Base:');
showrule(fis)

%% Save FIS

writeFIS(fis, 'USV_Speed_Controller.fis');
disp('FIS saved as USV_Speed_Controller.fis');

%% Plot membership functions

figure;
plotmf(fis, 'input', 1);
grid on;
title('Membership Functions for Distance to Waypoint');

figure;
plotmf(fis, 'input', 2);
grid on;
title('Membership Functions for Absolute Heading Error');

figure;
plotmf(fis, 'output', 1);
grid on;
title('Membership Functions for Linear Speed');

%% Plot FIS surface

figure;
gensurf(fis);
xlabel('Distance to waypoint d [m]');
ylabel('|\Delta\theta| [rad]');
zlabel('Speed v [m/s]');
title('Fuzzy Supervisory Speed Control Surface');
grid on;