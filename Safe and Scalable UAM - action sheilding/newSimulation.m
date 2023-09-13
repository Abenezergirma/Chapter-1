% This script simulates the aircrafts with full past trajectories 

clear; close all; clc;
load('simulate32.mat');

numDrones = length(droneList);

% Extract paths and compute maximum length
cellstore = cellfun(@(x) x.traveledPath(:,1:3), droneList, 'UniformOutput', false);
maxlength = max(cellfun(@length, cellstore));

% Preallocate arrays
xTraj = zeros(numDrones, maxlength);
yTraj = zeros(numDrones, maxlength);
zTraj = zeros(numDrones, maxlength);
bestTrajectory = zeros(numDrones, 10, 3, maxlength); 

% Populate trajectories
for i = 1:numDrones
    len = length(droneList{i}.traveledPath(:,1));
    xTraj(i,1:len) = droneList{i}.traveledPath(:,1);
    yTraj(i,1:len) = droneList{i}.traveledPath(:,2);
    zTraj(i,1:len) = droneList{i}.traveledPath(:,3);
    
    % Padding with the last element
    xTraj(i,xTraj(i,:)==0) = droneList{i}.traveledPath(end,1);
    yTraj(i,yTraj(i,:)==0) = droneList{i}.traveledPath(end,2);
    zTraj(i,zTraj(i,:)==0) = droneList{i}.traveledPath(end,3);
    
    % Organizing best trajectories
    bestTraject = droneList{i}.bestTrajectory;
    bestTrajectory(i,:,:,1:size(bestTraject,1)/10) = permute(reshape(bestTraject, 10, size(bestTraject,1)/10, 3), [1 3 2]);
end

% Plotting
figure('units','pixels','position',[0 0 1920 1080]); 
hold on;
for vehicle = 1:numDrones
    aircraftHandle(vehicle) = plot3(xTraj(vehicle,1),yTraj(vehicle,1),zTraj(vehicle,1),'o','MarkerFaceColor','red');
    pathHandle(vehicle) = plot3(xTraj(vehicle,1), yTraj(vehicle,1), zTraj(vehicle,1), 'LineWidth',1.2,'Color','blue');
    bestTrajHandle(vehicle) = plot3(xTraj(vehicle,1), yTraj(vehicle,1), zTraj(vehicle,1), 'LineWidth',1.5, 'Color','black');
    plot3(droneList{vehicle}.goal(1),droneList{vehicle}.goal(2),droneList{vehicle}.goal(3),'-s','LineWidth',5,'Color','black');
end

% Set plot limits and labels
xlim([min(xTraj,[],'all') - 1000, max(xTraj,[],'all') + 5000 ]);
ylim([min(yTraj,[],'all') - 1000, max(yTraj,[],'all') + 5000 ]);
zlim([min(zTraj,[],'all') - 500, max(zTraj,[],'all') + 500]);
xlabel('x in meters');
ylabel('y in meters');
zlabel('z in meters');
view(3);
grid on;

stepCounter = title(sprintf('step = %.2f', 1));
% text(-25000,-20000,-1500,'Navigation with ' + string(numDrones) +' agents','FontSize',14);

% Video capture
wobj = VideoWriter('test1.mp4');
wobj.FrameRate = 30;
open(wobj);

set(gcf, 'renderer', 'zbuffer');
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

% Dynamic Camera Initial Configuration
initAzim = 45; % starting azimuthal angle
azimRate = 0.05; % how much the angle changes per step
initElev = 30;
elevRate = 0.01;


for p = 1:maxlength
    for aircraft = 1:numDrones
        set(stepCounter, 'String', sprintf('step = %.2f',p));
        set(aircraftHandle(aircraft), 'XData', xTraj(aircraft,p), 'YData', yTraj(aircraft,p), 'ZData', zTraj(aircraft,p));
        
        % Update paths
        pathHandle(aircraft).XData = [pathHandle(aircraft).XData, xTraj(aircraft,p)];
        pathHandle(aircraft).YData = [pathHandle(aircraft).YData, yTraj(aircraft,p)];
        pathHandle(aircraft).ZData = [pathHandle(aircraft).ZData, zTraj(aircraft,p)];
        
        % Update best trajectories
        set(bestTrajHandle(aircraft), 'XData', bestTrajectory(aircraft,1:3,1,p), 'YData', bestTrajectory(aircraft,1:3,2,p), 'ZData', bestTrajectory(aircraft,1:3,3,p));
    end

    % Dynamic Camera View
    currAzim = initAzim + azimRate * p;
    currElev = initElev + elevRate * p;
    view(currAzim, currElev); % fixed elevation, rotating azimuth

    drawnow;
    frame = getframe(gcf);
    writeVideo(wobj, frame);
end

close(wobj);
