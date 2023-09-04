function dydt = FixedWing_vectorized(t,y,actions)

b_gamma = 0.5;
b_Va = 0.5;
b_phi = 0.5;
g = 9.81;

%%% control actions %%%
% actions = fixedWingActions;

n = length(actions);

y = reshape(y, n, []);

[gamma_c, phi_c, Va_c] = deal(actions(:,1), actions(:,2), actions(:,3));

[~,~,~, chi, gamma, phi, Va] = deal(y(:,1), y(:,2), y(:,3), y(:,4), y(:,5), y(:,6), y(:,7));

Vg = Va; % assuming no wind

gamma_a = asin(Vg .*sin(gamma) ./Va);

psi = chi;

northRate = Va .* cos(psi) .* cos(gamma_a);

eastRate = Va .* sin(psi) .* cos(gamma_a);

heightRate = Va .* sin(gamma_a);

chiRate = (g ./Vg) .* tan(phi) .* cos(chi - psi);

gammaRate = b_gamma .* (gamma_c - gamma);

VaRate = b_Va .* (Va_c - Va);

phiRate = b_phi .* (phi_c - phi);

dydt = [northRate; eastRate; heightRate; chiRate; gammaRate; phiRate; VaRate];

end

