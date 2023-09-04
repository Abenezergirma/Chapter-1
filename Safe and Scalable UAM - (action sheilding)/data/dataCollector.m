% This script collects the data used to build the tables 
clear all; close all; clc;
folderPath = '/home/abenezertaye/Desktop/MATLAB/FastMDP/Fall 2022 versions/AIAA Journal/Safe and Scalable UAM - updated v1.1 (action sheilding)/data/Experiments/2 agents/';

%initialize arrays for data storage

experimentData = [];

for experiment = 1:25
    % run the experiment
    disp('Currently running experiment --- ' + string(experiment) )
    [totalNMACs,stepTimer] = main;

    % collect data for building the table

    allNMAC = totalNMACs;
    Mean = mean(stepTimer);
    Median = median(stepTimer);
    Std = std(stepTimer);
    Throughput = sum(stepTimer);

    experimentData = [experimentData;[allNMAC,Mean,Median,Std,Throughput] ];
    % save the workspace for future reference 
    filePath = strcat(folderPath,'Experiment ',num2str(experiment));
    save(filePath);
end
