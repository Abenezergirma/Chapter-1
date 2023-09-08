clear 

north = 10;
east = 10;
height = 10;
chi = 0.3;
gamma = 0.2;
phi = 0.2;
Va = 15;
initialStates = [north, east, height, chi, gamma, phi, Va];

tic
model = DryVR(0.1,300); % [x, y, z] pos of the vehicle

% initialStates = initialStates;
traces = GenerateTraces(model, 'fixedwing_vector', initialStates);
t = 0:0.01:5;
B = traces;
B(:,:,8) = repmat(t,1,1,1000); %DO this before the simulation
Traces = B(:,:,[8,1:7]);
traces = permute(Traces, [2,1,3]);
Traces = traces(1:50:1000,5:end,:);
first = Traces(1,:,:);
mid = Traces(10,:,:);
Traces(1,:,:) = mid;
Traces(10,:,:) = first;
% Traces = traces(:,3:end,:);


initialRadii = computeInitialRadii(model,Traces);
discrepancyParameters = computeDiscrepancyParameters(model, Traces, initialRadii);
reachTube = getReachtube(model, Traces, initialRadii, discrepancyParameters);
toc
save('output')