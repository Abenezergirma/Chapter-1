clear 

north = 10;
east = 10;
height = 10;
chi = 0.3;
gamma = 0.349;
phi = -0.2;
Va = 70;
initial_states = [north, east, height, chi, gamma, phi, Va];

actions = fixedWing_Actions();
% % 
traces = fixedWing_discrete(initial_states, actions,0.1,50 );

plot3(squeeze(traces(:,:,1)),squeeze(traces(:,:,2)),squeeze(traces(:,:,3)))

% plot(trajs(:,:,))
tic
model = DryVR(0.1,300); % [x, y, z] pos of the vehicle

initialStates = initial_states;
Traces = GenerateTraces(model, 'random_action', initialStates);


[x_lower_point,x_lower_index] = min(Traces(:,end,2));
[x_higher_point,x_higher_index] = max(Traces(:,end,2));

[y_lower_point,y_lower_index] = min(Traces(:,end,3));
[y_higher_point,y_higher_index] = max(Traces(:,end,3));

[z_lower_point,z_lower_index] = min(Traces(:,end,4));
[z_higher_point,z_higher_index] = max(Traces(:,end,4));

center_point_x = (x_lower_point+x_higher_point)*0.5;
[minValue,closestIndex_x] = min(abs(Traces(:,end,2) - center_point_x));

center_point_y = (y_lower_point+y_higher_point)*0.5;
[minValue,closestIndex_y] = min(abs(Traces(:,end,3) - center_point_y));

center_point_z = (z_lower_point+z_higher_point)*0.5;
[minValue,closestIndex_z] = min(abs(Traces(:,end,4) - center_point_z));

points = squeeze(Traces(:,end,2:4));
center_point = [center_point_x,center_point_y,center_point_z];
[index, distance] = dsearchn(points,center_point);
plot3(Traces(:,:,2)', Traces(:,:,3)',Traces(:,:,4)')
hold on
plot3(squeeze(Traces(index,:,2)),squeeze(Traces(index,:,3)),squeeze(Traces(index,:,4)),"o")

% plot(traces(:,:,3)')
% hold on
% plot(traces(closestIndex_y,:,3),"o")
% 
% 
% 
% 
% Xmin = intruderTraj(:,x_lower_index,1);
% Xmax = intruderTraj(:,x_higher_index,1);
% Ymin = intruderTraj(:,y_lower_index,2);
% Ymax = intruderTraj(:,y_higher_index,2);
% Zmin = intruderTraj(:,z_lower_index,3);
% Zmax = intruderTraj(:,z_higher_index,3);

% t = 0:0.01:10;
% B = traces;
% B(:,:,8) = repmat(t,1,1,1000); %DO this before the simulation
% Traces = B(:,:,[8,1:7]);
% traces = permute(Traces, [2,1,3]);
% Traces = traces(1:50:1000,5:end,:);
% first = Traces(1,:,:);
% mid = Traces(10,:,:);
% Traces(1,:,:) = mid;
% Traces(10,:,:) = first;
Traces = Traces(:,2:end,:);

% index = floor(linspace(1,1000,100));
% 
% plot3(squeeze(traces(index,index,2)),squeeze(traces(index,index,3)),squeeze(traces(index,index,4)),"o",color='red')
% Traces = Traces;
initialRadii = computeInitialRadii(model,Traces);
discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);
toc
save('output')
%%

% Simulate using ode45 
controlActions = fixedWing_Actions;
vectorizedStates = initial_states.*ones(length(controlActions),1);
[t1, Traces] = ode45(@(t,y) DryVR.FixedWing_vectorized(t,y,controlActions),0:0.01:5,vectorizedStates);
Traces = permute(reshape(Traces, length(t1), length(controlActions), length(initial_states)), [1, 2, 3]);
%% plot and compare the above two methods 
state = 7;
figure(1);plot(Traces(:,:,state))    

figure(2);plot(trajs(:,:,state))
%% 
% indexer = [1,2,3,4,5,7,8];
% initialStates(indexer);
%Pick the aircraft type from here 
aircraftType = struct('default', {'fwdProjectFast'}, 'simplified', {'simplified'}, 'fixedwing', {'fixedwing'}, 'multirotor',{'multirotor'});

initialStates = [100.97398, 100.69597, 100.16267, 0.00000, 0.00000, 4.74448, 0.00000,  24.01000];

model = DryVR(0.01,500); % [x, y, z] pos of the vehicle 
tic
[t,Traces] = GenerateTraces(model, 'random_action', initial_states);
% The following code is for the case when all actionsets are used 
B = Traces;
B(:,:,8) = repmat(t,1,1575); %DO this before the simulation 
Traces = B(:,:,[8,1:7]);
Traces = permute(Traces, [2,1,3]);
Traces = Traces(1:63:1575,5:end,:);
first = Traces(1,:,:);
mid = Traces(13,:,:);
Traces(1,:,:) = mid;
Traces(13,:,:) = first;

initialRadii = computeInitialRadii(model,Traces);
% muPoints = computeMuPoints(model, Traces, initialRadii);
discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);
toc

save('output')
