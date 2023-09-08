function futureTraj = No_fixedWing_discrete(currentState, actions, timestep, numSteps)
% A function that returns a set of next states with state constraints
% applied 

%Note: this function is vectorzed to accept an array of control actions

b_gamma = 0.5;
b_Va = 0.5;
b_phi = 0.5;
g = 9.81;
% aircraftParamters;

% currentState = ownship.currentStates;

[north, east, height, chi, gamma, phi, Va] = deal(currentState(:,1),currentState(:,2),...
    currentState(:,3),currentState(:,4),currentState(:,5),currentState(:,6),currentState(:,7));

[gamma_c, phi_c, Va_c] = deal(actions(:,1), actions(:,2), actions(:,3));

futureTraj = zeros( numSteps, length(actions(:,1)), length(currentState(1,:))); %Preallocated memory for speed

for i = 1:numSteps

    % get the parameters assuming no wind
    Vg = Va; % assuming no wind
    psi = chi; % assuming no wind 

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

    % updating gamma air
    gamma_a = asin(Vg .*sin(gamma) ./Va);

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