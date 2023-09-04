function fixedWingActions = fixedWing_Actions()
% Builds an array that stores the action space of each team
% Fixedwing --
% 
% gamma_c = linspace(-pi/10,pi/10,10);
% if ischar(varargin)
    gamma_c = [linspace(-pi,-pi/10,5), linspace(pi/10,pi,5)];
% end
% gamma_c = linspace(-pi/10,pi/10,10);

phi_c = gamma_c; %force vector in g's through nose of airplane

Va_c = 25:5:75; % this is in m/s check this vector from the paper

% construct the joint action space that comprises the three action spaces
fixedWingActions = {gamma_c, phi_c, Va_c};
actionsD = fixedWingActions;
[actionsD{:}] = ndgrid(fixedWingActions{:});
teamActions = cell2mat(cellfun(@(m)m(:),actionsD,'uni',0));

fixedWingActions = teamActions;

end

% new_gamma = [linspace(-pi,-pi/10,5), linspace(pi/10,pi,5)];
