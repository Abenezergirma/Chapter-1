function [allNMAC, Mean, Median, Std, Throughput] = main()

% Initialization
clear; close all; clc;
addContainingDirAndSubDir();
parameters();

% Set parameters
totalAgents = 32;
teamActions = fixedWing_Actions;
teamLimits = buildLimits;
scenarioSelector = struct('circle','circle', 'random','random');
initialStates = scenarioGenerator(totalAgents,'circle');
goals = circshift(initialStates, totalAgents/2);
DryVR_obj = DryVR(0.1, 500);
totalNMACs = 0;

% Instantiate drone list
droneList = instantiateDrones(totalAgents, goals, initialStates);

%% Main loop
numLeft = size(droneList, 2);
stepTimer = [];
j = 0;

while numLeft > 0
    tic;
    for k = 1:numel(droneList)
        ownship = droneList{k};
        if ownship.dead
            continue;
        end
        
        [positivePeaks, negativePeaks, droneList] = buildPeaks(ownship, droneList, teamActions, DryVR_obj);
        ownship = projectAndUpdate(ownship, teamActions, teamLimits, positivePeaks, negativePeaks);
        
        % Check goal condition
        if norm(ownship.currentStates(1:3) - ownship.goal) < 30
            ownship.dead = true;
            disp(['ownship ', num2str(ownship.aircraftID), ' made it to goal, removed']);
            numLeft = numLeft - 1;
        end
        droneList{k} = ownship;
    end

    % Monitor the game status
    [terminal, NMACs, droneList] = terminalDetection(droneList);
    totalNMACs = totalNMACs + NMACs;
    stepTimer = [stepTimer, toc];
    j = j + 1;
    displayGameStatus(j, toc, totalNMACs, numLeft);

    if terminal
        break;
    end
end

% Display results
allNMAC = totalNMACs;
Mean = mean(stepTimer);
Median = median(stepTimer);
Std = std(stepTimer);
Throughput = sum(stepTimer);

save('droneList32.mat', 'droneList');
save('simulate32');

end

function droneList = instantiateDrones(totalAgents, goals, initialStates)
    droneList = cell(1, totalAgents);
    baseLatitude = 10;
    baseAltitude = 10;

    for i = 1: totalAgents
        drone = Ownship(i, 1, baseLatitude, baseAltitude, goals(i, 1:3));
        drone = updateAircraftStates(drone, initialStates(i, :));
        droneList{i} = drone;
    end
end

function ownship = projectAndUpdate(ownship, teamActions, teamLimits, positivePeaks, negativePeaks)
    [futureStates, oneStepStates, futureActions] = neighboringStates(ownship, teamActions, teamLimits);
    totalValues = valueFunction(ownship, futureStates, positivePeaks, negativePeaks);
    ownship = selectBestAction(ownship, totalValues, futureActions, futureStates, oneStepStates);
    ownship = updateAircraftStates(ownship, ownship.nextStates);
    ownship.Traces = 0;
end

function displayGameStatus(step, computationTime, NMACs, numLeft)
    disp(['step ', num2str(step), ', computation time is ', num2str(computationTime), ' sec']);
    fprintf('NMACs so far, %d\n', NMACs);
    fprintf('Num aircraft left, %d\n', numLeft);
end
