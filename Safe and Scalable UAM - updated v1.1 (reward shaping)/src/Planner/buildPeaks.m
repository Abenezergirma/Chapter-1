function [positivePeaks, negativePeaks, droneList] = buildPeaks(ownship, droneList,actions, DryVR_obj)
% builds a set of reward sources for a certain ownship

ownX = ownship.position(1);

ownY = ownship.position(2);

ownZ = ownship.position(3);

negativePeaks = [];

normalActions = actions;

for i = 1:numel(droneList) % Loop through the list of created drone objects
    drone = droneList{i};
    if drone.aircraftID == ownship.aircraftID %the case when the ownship ID and intruder ID is the same
        continue
    end
    if drone.dead % Skip the drone if its out of the game
        continue
    end

    [intruderX, intruderY, intruderZ] = deal(drone.position(1), drone.position(2), drone.position(3)); % intruder pos

    distance = sqrt( (ownX - intruderX)^2 + (ownY - intruderY)^2 + (ownZ - intruderZ)^2 );

    if distance < 2000
        %compute reachable set here and build the peaks inside the set
        %         intruderTraj = forwardSimulate(drone, actions, 0.1, 500);



%         if drone.hit == true
%             drone.Traces = 0;
% 
%             actions(:,3) = 24;
%             actions = unique(actions, 'rows');
%             % else
%             %     actions = actions([floor(linspace(1,1000,50))],:);
%         end
% 
%         if drone.Traces == 0
%         drone = forwardSimulate(drone, actions, 0.1, 500);
% 
%         intruderTraj = drone.Traces;
%         else
%             intruderTraj = drone.Traces;
%         end
% 
%         actions = normalActions;
% 
% %         intruderTraj = fixedWing_discrete(drone, actions, 0.1, 500);
% 
% 
%         [x_lower_point,x_lower_index] = min(intruderTraj(end,:,1));
%         [x_higher_point,x_higher_index] = max(intruderTraj(end,:,1));
% 
%         [y_lower_point,y_lower_index] = min(intruderTraj(end,:,2));
%         [y_higher_point,y_higher_index] = max(intruderTraj(end,:,2));
% 
%         [z_lower_point,z_lower_index] = min(intruderTraj(end,:,3));
%         [z_higher_point,z_higher_index] = max(intruderTraj(end,:,3));
% 
%         Xmin = intruderTraj(:,x_lower_index,1);
%         Xmax = intruderTraj(:,x_higher_index,1);
%         Ymin = intruderTraj(:,y_lower_index,2);
%         Ymax = intruderTraj(:,y_higher_index,2);
%         Zmin = intruderTraj(:,z_lower_index,3);
%         Zmax = intruderTraj(:,z_higher_index,3);

% 
%         plot(intruderTraj(:,:,1))
%         hold on
%         plot(intruderTraj(:,x_lower_index,1),"o")
%         plot(intruderTraj(:,x_higher_index,1),"o")



%         model = DryVR(0.1,300); % [x, y, z] pos of the vehicle
% 
%         initialStates = drone.currentStates;
%         traces = GenerateTraces(model, 'random_action', initialStates);
%         Traces = traces(:,5:end,:);
% 
% 
%         initialRadii = computeInitialRadii(model,Traces);
%         discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
%         reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);
        drone = computeReachTube(drone, DryVR_obj);
        reachTube = drone.reachSet;

        xStateBounds = reachTube(:,:,2);
        yStateBounds = reachTube(:,:,3);
        zStateBounds = reachTube(:,:,4);
% 
        segment = 1:2:length(xStateBounds);

        Xmin = xStateBounds(segment,1);
        Xmax = xStateBounds(segment,2);
        Ymin = yStateBounds(segment,1);
        Ymax = yStateBounds(segment,2);
        Zmin = zStateBounds(segment,1);
        Zmax = zStateBounds(segment,2);

        xRange = (Xmax - Xmin);
        yRange = (Ymax - Ymin);
        zRange = (Zmax - Zmin);
        center = [Xmin + xRange/2, Ymin + yRange/2, Zmin + zRange/2];
        radius = sqrt(xRange.^2 + yRange.^2 + zRange.^2) * 0.5;

        reward = linspace(500,1500,length(xRange))';


        negativePeaks = [negativePeaks; [reward, center, repmat(0.97, length(xRange),1) ,radius*1.5 ]];

    end
    droneList{i} = drone;

end

[goalX, goalY, goalZ] = deal(ownship.goal(1), ownship.goal(2), ownship.goal(3));

positivePeaks = [200, goalX, goalY, goalZ, 0.999, inf];

end