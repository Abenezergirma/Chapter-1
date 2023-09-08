% This script is used to generate the reachSet.mat file that is used to plot
% the simulation traces and the reachable sets in my papers  
clear all; close all; clc;

% Define constants
north   = 10;
east    = 10;
height  = 10;
chi     = 0.3;
gamma   = 0.2;
phi     = 0.2;
Va      = 15;

% Initial states setup
initialStates = [north, east, height, chi, gamma, phi, Va];

% Start timing
tic

% Initialize model
model = DryVR(0.1,300); % [x, y, z] position of the vehicle

% Generate traces
traces = GenerateTraces(model, 'fixedwing_vector', initialStates);

% Post-process and reformat traces
t = 0:0.01:10;
% Extend and reorder the traces matrix
traces(:,:,8) = repmat(t,1,1,1000);

% Slicing, permuting, and sampling every 50 from start to 1000, while ignoring the first 4 columns
Traces = permute(traces(:,:,[8,1:7]), [2,1,3]);
Traces = Traces(1:50:1000, 5:end, :);

% Swap the first and tenth slices
Traces([1 10],:,:) = Traces([10 1],:,:);


% Compute initial radii, discrepancy parameters, and reach tube
initialRadii = computeInitialRadii(model, Traces);
discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);

% End timing and save results
toc
save('reachSet')
