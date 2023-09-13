function [futureStates, oneStepStates, futureActions] = neighboringStates(ownship, actions, limits)
% A function that returns next one step and few future states of the
% aircraft
% Input: 
% Output: 
% Usage: 
% 
if ownship.hit == true
    ownship.Traces = 0;
    
    actions(:,3) = 24;
    actions = unique(actions, 'rows');
% else
%     actions = actions([floor(linspace(1,1000,50))],:);
end



if ownship.Traces == 0
    ownship = forwardSimulate(ownship, actions, 0.1, 300);

    futureTraj = ownship.Traces;
else
    futureTraj = ownship.Traces;
end


oneStepStates = futureTraj(11,:,:); %eventually put an assertion block to check the size 
% plot(squeeze(futureStates(:,1:5,2)))

futIndex = floor(linspace(1,length(futureTraj(:,1,1)),10));

futureStates = futureTraj(futIndex,:,:); %eventually put an assertion block to check the size 

%reshaping these two arrays for next operations
futureActions = permute(repmat(actions,1,1,10), [3 1 2]) ;

oneStepStates = permute(repmat(oneStepStates,10,1,1), [1 2 3]);

end


