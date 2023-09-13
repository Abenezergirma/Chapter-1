clear; close all; clc;

load('MDP_32_debug.mat')

%to get the array with max length
for path=1:length(droneList)
    cellstore{path} = droneList{path}.traveledPath(:,1:3);
end
maxlength = max(cellfun(@length,cellstore));

xTraj = zeros(length(droneList), maxlength);
yTraj = zeros(length(droneList), maxlength);
zTraj = zeros(length(droneList), maxlength);
bestTrajectory =  zeros(length(droneList), 10, 3, maxlength); %(agents, 10futuresteps, (x,y,z), maxstep)

for st = 1:maxlength
    plot3(squeeze(bestTrajectory(3,:,1,st)), squeeze(bestTrajectory(3,:,2,st)), squeeze(bestTrajectory(3,:,3,st)))
    hold on
end

for i = 1:length(droneList)
    % paths
    xTraj(i,1:length(droneList{i}.traveledPath(:,1))) = droneList{i}.traveledPath(:,1);
    yTraj(i,1:length(droneList{i}.traveledPath(:,2))) = droneList{i}.traveledPath(:,2)';
    zTraj(i,1:length(droneList{i}.traveledPath(:,3))) = droneList{i}.traveledPath(:,3)';
    % best trajectories
    bestTraject = droneList{i}.bestTrajectory;
    bestTraject = permute(reshape(bestTraject, 10, length(bestTraject)/10, 3), [1 3 2]);
    bestTrajectory(i,:,:,1:length(bestTraject)) = bestTraject;

    % concatenate the last parts with the last element
    xTraj(i,xTraj(i,:)==0) = droneList{i}.traveledPath(end,1);
    yTraj(i,yTraj(i,:)==0) = droneList{i}.traveledPath(end,2);
    zTraj(i,zTraj(i,:)==0) = droneList{i}.traveledPath(end,3);

%     bestTrajectory(i,squeeze(bestTrajectory(i,:,:,:)==0)) = ...
        reshape(repmat(droneList{i}.bestTrajectory(end,:), 10*(maxlength - length(droneList{i}.traveledPath(:,1))), 1), 1, []);

end


%%
% hFigure = figure;

% define Path, aircraft, and future Traj handlers and plot the goal states
for vehicle = 1:length(droneList)
    aircraftHandle(vehicle) = plot3(xTraj(vehicle,1),yTraj(vehicle,1),zTraj(vehicle,1),'o','MarkerFaceColor','red');
    hold on
    pathHandle(vehicle) = plot3([xTraj(vehicle,1)], [yTraj(vehicle,1)], [zTraj(vehicle,1)], 'LineWidth',1.2,'Color','blue');


    bestTrajHandle(vehicle) = plot3([xTraj(vehicle,1)], [yTraj(vehicle,1)], [zTraj(vehicle,1)], 'LineWidth',1.5, 'Color','black');

    plot3(droneList{vehicle}.goal(1),droneList{vehicle}.goal(2),droneList{vehicle}.goal(3),'-s','LineWidth',5,'Color','black')

%     plot3(xTraj(1,1),yTraj(1,1),zTraj(1,1),'-s','LineWidth',5,'Color','black')
end

xlim([min(xTraj,[],'all') - 1000, max(xTraj,[],'all') + 5000 ])
ylim([min(yTraj,[],'all') - 1000, max(yTraj,[],'all') + 5000 ])
zlim([min(zTraj,[],'all') - 500, max(zTraj,[],'all') + 500])
ylabel('y in meters')
xlabel('x in meters')
zlabel('z in meters')
% title('Navigation with '+string(totalAgents) +' agents')
stepCounter = title(sprintf('step = %.2f', 1));

view(3)%(90,0)%(2)
grid on
text(-25000,-20000,-1500,'Navigation with '+string(totalAgents) +' agents','FontSize',14)
% get figure size
pos = get(gcf, 'Position');
width = pos(3)+10; height = pos(4)-7;

% preallocate data (for storing frame data)
numberOfFrames = maxlength;

% Set up the movie structure.
% Preallocate movie, which will be an array of structures.
% First get a cell array with all the frames.
allTheFrames = cell(numberOfFrames,1);
vidHeight = 344;%420;
vidWidth = 446;%560;
allTheFrames(:) = {zeros(vidHeight, vidWidth, 3, 'uint8')};
% Next get a cell array with all the colormaps.
allTheColorMaps = cell(numberOfFrames,1);
allTheColorMaps(:) = {zeros(256, 3)};
% Now combine these to make the array of structures.
myMovie = struct('cdata', allTheFrames, 'colormap', allTheColorMaps);
% Create a VideoWriter object to write the video out to a new, different file.
% writerObj = VideoWriter('problem_3.avi');
% open(writerObj);
% Need to change from the default renderer to zbuffer to get it to work right.
% openGL doesn't work and Painters is way too slow.
set(gcf, 'renderer', 'zbuffer');
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

% mov = zeros(height, width, 1, length(xTraj), 'uint32');
wobj = VideoWriter('test1.mp4');
wobj.FrameRate = 60;                  % frames per second (video speed)
open(wobj);                           % open file
% mkdir('test')


figure('units','pixels','position',[0 0 1920 1080]) 

for p = 1:maxlength
    for aircraft = 1:length(droneList)
        %display the time and NMACs also
        set(stepCounter, 'String', sprintf('step = %.2f',p));

        % plot the aircrafts
        set(aircraftHandle(aircraft), 'XData', xTraj(aircraft,p), 'YData', yTraj(aircraft,p), 'ZData', zTraj(aircraft,p))

        %plot the paths
        % replaced aircraft with 1
        pathHandle(aircraft).XData = [pathHandle(aircraft).XData, xTraj(aircraft,p)];
        pathHandle(aircraft).YData = [pathHandle(aircraft).YData, yTraj(aircraft,p)];
        pathHandle(aircraft).ZData = [pathHandle(aircraft).ZData, zTraj(aircraft,p)];

        %plot the best next trajectories
        set(bestTrajHandle(aircraft), 'XData', bestTrajectory(aircraft,:,1,p), 'YData', bestTrajectory(aircraft,:,2,p) ...
            , 'ZData', bestTrajectory(aircraft,:,3,p))

        %         drawnow


%         hFig = figure;                       % Bring up new figure
% %         imshow(hFig,'Border','tight') % The axes will fill up the entire figure as much as possible without changing aspect ratio.
%         hFig.WindowState = 'maximized';      % Maximize the figure to your whole screen.
%         hFigure.WindowState = 'minimized';

        thisFrame = getframe(gcf);              % This is as large as you can get.
        myMovie(p) = thisFrame;           % save frame into video


        %         f = getframe(gcf);
        %
        %         if p == 1
        %             [mov(:,:,1,p), map] = rgb2ind(f.cdata, 256, 'nodither');
        %         else
        %             mov(:,:,1,p) = rgb2ind(f.cdata, map, 'nodither');
        %         end

    end
end

%%
startingFolder = pwd;
defaultFileName = {'*.avi';'*.mp4';'*.mj2'}; %fullfile(startingFolder, '*.avi');
[baseFileName, folder] = uiputfile(defaultFileName, 'Specify a file');
fullFileName = fullfile(folder, baseFileName);
% Create a video writer object with that file name.
% The VideoWriter object must have a profile input argument, otherwise you get jpg.
% Determine the format the user specified:
[folder, baseFileName, ext] = fileparts(fullFileName);
switch lower(ext)
		case '.jp2'
			profile = 'Archival';
		case '.mp4'
			profile = 'MPEG-4';
		otherwise
			% Either avi or some other invalid extension.
			profile = 'Uncompressed AVI';
	end
writerObj = VideoWriter(fullFileName, profile);
	open(writerObj);
	% Write out all the frames.
	numberOfFrames = length(myMovie);
	for frameNumber = 1 : numberOfFrames 
	   writeVideo(writerObj, myMovie(frameNumber));
	end
	close(writerObj);
% %% convert .avi to mp4
% % video file
% if isunix % for linux
%     pathVideoAVI = '~/aaa.avi'; % filename, used later to generate mp4
% elseif ispc % fow windows
%     pathVideoAVI = 'd:\someVideo.avi'; % filename, used later to generate mp4
% end
% 
% % convert AVI to MP4
% pathVideoMP4 = regexprep(pathVideoAVI,'\.avi','.mp4'); % generate mp4 filename
% if isunix % for linux
%     [new_,vid] = system(sprintf('ffmpeg -i %s -y -an -c:v libx264 -crf 0 -preset slow %s',pathVideoAVI,pathVideoMP4)); % for this to work, you should have installed ffmpeg and have it available on PATH
% elseif ispc % for windows
%     [~,~] = system(sprintf('ffmpeg.exe -i %s -y -an -c:v libx264 -crf 0 -preset slow %s',pathVideoAVI,pathVideoMP4)); % for this to work, you should have installed ffmpeg and have it available on PATH
% end


% Create animated GIF
% imwrite(mov, map, 'animation_8.gif', 'DelayTime', 1/60, 'LoopCount', inf);

%%

% plot(droneList{9}.traveledPath(:,4))
%
%
% plot(squeeze(droneList{1}.pastControls))

