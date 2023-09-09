classdef Ownship
    %This class implements a 6-DOF guidance model of fixed wing
    % aircraft. The model is adopted from "Beard, R. W., and McLain, T. W., 
    % Small unmanned aircraft: Theory and practice, Princeton University Press, 2012.
    % https://doi.org/10.1515/9781400840601

    % This model is first implemented in my AIAA JGCD paper as part of
    % my first chapter of my PhD dissertation 

    % The main role of this model in this project is to desribe the
    % behavior of the aircraft in the trajectory planning stage of the
    % framework

    % Important properties:
    %      aircraftActions - Action set of the aircrft used to train the
    %      MDP Agent
    %      currentStates - 12 states of the aircraft at time t
    %      nextStates - Projected best states of the aircraft at t+1 
    %      traveledPath - Past [x, y, z] positions of aircraft 
    %      bestTrajectory - Projected best [x, y, z] of the aircraft 

    % Important methods:
    %      fixedwingDynamics - a discrete model of the aircraft
    %  
    %      forwardSimulate - simulates the future states of the aircraft 
    %      fixedwingActions - generates an array that contains the control set
    %      selectBestAction - picks the best action and state for t+1 
    properties %Uppercase -- TODO

        % general Aircraft properties
        %         L = 0.9; %Lift acceleration
        g = 9.81
        timeStep = 0.1;
        numSteps = 500;
        %         aicraftLimits;
        c;

        % Aircraft states
        currentStates = zeros(1,8);
        x = zeros(1,1);
        y = zeros(1,1);
        z = zeros(1,1);
        psi = zeros(1,1);
        gamma = zeros(1,1);
        phi = zeros(1,1);
        alpha_= zeros(1,1);
        xDot = zeros(1,1);
        yDot = zeros(1,1);
        zDot = zeros(1,1);
        V = zeros(1,1);
        position = zeros(1,3);
        velocity = zeros(1,3);
        nextStates = zeros(1,8);

        traveledPath; % stores the path traveled by the aircraft so far
        pastControls; % stores the control actions applied by the aircraft so far
        bestTrajectory;% stores the best trajectories picked by the aircraft so far
        values;
        bestVal;


        %control actions
        Va_c = zeros(1,1);
        controlActions;
        gamma_c;
        phi_c;

        %Vehicle identifiers
        aircraftID;
        aircraftTeam;
        dead = false;
        baseLatitude;
        baseAltitude;
        goal;

        %vehicle performace parameters
        hit = false;
        hitCounter;
        Traces = 0;
        reachSet = 0;

    end

    methods(Static)
        function futureTraj = fixedWingDynamics(ownship, actions, timestep, numSteps)
            % A function that returns a set of next states with state constraints
            % applied

            %Note: this function is vectorzed to accept an array of control actions

            b_gamma = 0.5;
            b_Va = 0.5;
            b_phi = 0.5;
            g = 9.81;
            % aircraftParamters;

            currentState = ownship.currentStates;
            % indexer = [1,2,3,4,5,7,8];
            % currentState= currentState(indexer);

            [north, east, height, chi, gamma, phi, Va] = deal(currentState(:,1),currentState(:,2),...
                currentState(:,3),currentState(:,4),currentState(:,5),currentState(:,6),currentState(:,7));

            [gamma_c, phi_c, Va_c] = deal(actions(:,1), actions(:,2), actions(:,3));

            futureTraj = zeros(numSteps, length(actions(:,1)), length(currentState(1,:))); %Preallocated memory for speed

            for i = 1:numSteps

                % get the parameters assuming no wind
                Vg = Va; % assuming no wind
                psi = chi; % assuming no wind

                % updating gamma air
                gamma_a = asin(Vg .*sin(gamma) ./Va);


                % updating the airspeed
                Va_Dot = b_Va * (Va_c - Va);
                Va = Va + Va_Dot * timestep;

                % updating gamma
                gamma_Dot = b_gamma .* (gamma_c - gamma);
                gamma = gamma + gamma_Dot * timestep;

                % updating phi
                phi_Dot = b_phi .* (phi_c - phi);
                phi = phi + phi_Dot * timestep;

                % updating chi
                chi_Dot = (g ./Vg) .* tan(phi) .* cos(chi - psi);
                chi = chi + chi_Dot * timestep;

                % updating the velocities
                northRate = Va .* cos(psi) .* cos(gamma_a);
                eastRate = Va .* sin(psi) .* cos(gamma_a);
                heightRate = Va .* sin(gamma_a);

                % updating the positions
                north = north + northRate * timestep;
                east = east + eastRate * timestep;
                height = height + heightRate * timestep;


                nextState = [north, east, height, chi, gamma, phi, Va];
                futureTraj(i,:,:) = nextState;
            end
        end

        function fixedWingActions = fixedWingActions(varargin)
            % Builds an array that stores the action space of each team
            % Fixedwing --
            
            gamma_c = linspace(-pi/10,pi/10,10);
        if ischar(varargin)
            gamma_c = [linspace(-pi,-pi/10,5), linspace(pi/10,pi,5)];
        end


            phi_c = gamma_c; 

            Va_c = 25:5:75; % this is in m/s check this vector from the paper

            % construct the joint action space that comprises the three action spaces
            fixedWingActions = {gamma_c, phi_c, Va_c};
            actionsD = fixedWingActions;
            [actionsD{:}] = ndgrid(fixedWingActions{:});
            teamActions = cell2mat(cellfun(@(m)m(:),actionsD,'uni',0));

            fixedWingActions = teamActions;

        end

    end

    methods
        function obj = Ownship(aircraftID, aircraftTeam, baseLatitude, baseAltitude, goal)
            % A method that initializes each aircraft in the game
            obj.aircraftID = aircraftID;
            obj.aircraftTeam = aircraftTeam;
            obj.baseLatitude = baseLatitude;
            obj.baseAltitude = baseAltitude;
            obj.goal = goal;
            %             obj.aicraftLimits = buildLimits;
            obj.aircraftActions = Ownship.fixedWingActions;

        end

        function [obj] = updateAircraftStates(obj,  currentStates)
            % A method that updates the states of the aircraft
            obj.x = currentStates(1);
            obj.y = currentStates(2);
            obj.z = currentStates(3);
            obj.psi = currentStates(4);
            obj.gamma = currentStates(5);
            %             obj.alpha_ = currentStates(6);
            obj.phi = currentStates(6);
            obj.V = currentStates(7);
            obj.position = [obj.x, obj.y, obj.z];
            obj.currentStates = currentStates;

        end

        function [obj] = updateControlActions(obj,  bestActions)
            % A method that updates the control actions based on the
            % selected best actions
            obj.gamma_c = bestActions(1);
            obj.phi_c= bestActions(2);
            obj.Va_c = bestActions(3);
            obj.controlActions = bestActions;
        end

        function [obj] = selectBestAction(obj, totalValues, futureActions, futureStates, oneStepStates)
            % A method that selects the optimal action
            bestValue = max(totalValues, [],'all');
            [bestRow, bestColumn, ~] = find(totalValues==bestValue);
            randomIndexRow = abs(randi(length(bestRow))); % randomly pick one index with max value
            randomIndexCol = abs(randi(length(bestColumn)));
            bestActions = futureActions(bestRow(randomIndexRow), bestColumn(randomIndexCol),:);

            % select the best next state
            bestStep = oneStepStates(bestRow(randomIndexRow), bestColumn(randomIndexCol),:);

            %select the best future trajectory for the next 10 sec
            bestTraj = squeeze(futureStates(:, bestColumn(randomIndexCol),1:3)) ;

            % update the states and control actions of the aircraft
            obj = obj.updateControlActions(bestActions);
            obj.nextStates = squeeze(bestStep)';

            %store the control actions and path traveled so far
            obj.traveledPath = [obj.traveledPath; obj.nextStates];
            obj.pastControls = [obj.pastControls; obj.controlActions];
            obj.bestTrajectory = [obj.bestTrajectory; bestTraj];
%             obj.values = [obj.values;totalValues];
            obj.bestVal = [obj.bestVal;bestValue];

        end

        function obj = forwardSimulate(obj, controlActions,timeStep, numSteps)
            obj.Traces = Ownship.fixedWingDynamics(obj, controlActions,timeStep, numSteps);
        end

        function obj = computeReachTube(obj, DryVR_obj)
            if obj.Traces == 0 

                if obj.hit == true
                    actions = obj.aircraftActions;
                    actions(:,3) = 24;
                    control_Actions = unique(actions, 'rows');
                else
                    control_Actions = obj.aircraftActions;
                
                end
                obj = forwardSimulate(obj, control_Actions,obj.timeStep, obj.numSteps);
            end
            traces = obj.Traces(3:end,floor(linspace(1,length(obj.Traces(1,:,1)),DryVR_obj.numTraces)),:);
            t = 0:0.1:50-0.3;
         
            traces(:,:,8) = repmat(t,1,1,DryVR_obj.numTraces); %DO this before the simulation
            traces = permute(traces(:,:,[8,1:7]), [2,1,3]);
            initialRadii = computeInitialRadii(DryVR_obj,traces);
            discrepancyParameters = computeDiscrepancyParameters(DryVR_obj, traces, initialRadii);
            reachTube = getReachtube(DryVR_obj, traces, initialRadii, discrepancyParameters);
            obj.reachSet = reachTube;
        end

    end
end