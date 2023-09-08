% function [] = visualizeStateConstraints()
clear; close all; clc;

load('MDP_32_debug.mat')

% Plot State constraints
figure(2)
VMin = 24; % m/s
VMax = 75; % m/s
psidotMin = - 30 * DEG2RAD; % rad/s
psidotMax =   30 * DEG2RAD; % rad/s
alphaMin = - 5 * DEG2RAD; % rad
alphaMax =  15 * DEG2RAD; % rad
phiMin = -pi/10 ; % rad
phiMax =  pi/10;  % rad
gammaMin = -pi/10 ;  % rad
gammaMax =  pi/10;   % rad

droneNO = 12; %12, 6

psi = droneList{droneNO}.traveledPath(:,4);
psiDot = diff(psi);
% psiDot = [psiDot; psiDot(end)];


gamma = droneList{droneNO}.traveledPath(:,5);

% alpha_ = droneList{4}.traveledPath(:,6);

phi = droneList{droneNO}.traveledPath(:,6);

V = droneList{droneNO}.traveledPath(:,7);
% define handlers for each state

% state psiDot
sub1 = subplot(2,2,1);
% ani1 = animatedline('LineWidth',1.2,'Color','blue');
axis([0,  length(psiDot), min(psiDot) , max(psiDot)])
plot(psiDot,'LineWidth',1.2,'Color','blue');
hold on
PsidotMax = psidotMax*ones(length(psiDot),1);
PsidotMin = psidotMin*ones(length(psiDot),1);
plot(PsidotMax,'--','Color',"#0072BD",'LineWidth',1.2);
plot(PsidotMin,'--','Color','red','LineWidth',1.2);
axis([0,  length(psiDot), psidotMin*1.25, psidotMax*1.25])
grid on
ylabel('$\dot{\chi}$', 'Interpreter','latex')
xlabel('steps')
legend('State','Max Limit','Min Limit', 'Location','northeast')

% state gamma
sub2 = subplot(2,2,2);
% ani2 = animatedline('LineWidth',1.2,'Color','blue');
axis([0,  length(gamma), min(gamma)*1.25, max(gamma)*1.25])
plot(gamma,'LineWidth',1.2,'Color','blue');
hold on
GammaMax = gammaMax*ones(length(gamma),1);
GammaMin = gammaMin*ones(length(gamma),1);
plot(GammaMax,'--','Color',"#0072BD",'LineWidth',1.2);
plot(GammaMin,'--','Color','red','LineWidth',1.2);
axis([0,  length(gamma), gammaMin*1.25, gammaMax*1.25])
grid on
ylabel('\gamma','Interpreter','tex')
xlabel('steps')


% % state alpha_
% sub3 = subplot(3,2,3);
% ani3 = animatedline('LineWidth',1.2,'Color','blue');
% hold on
% AlphaMax = alphaMax*ones(length(alpha_),1);
% AlphaMin = alphaMin*ones(length(alpha_),1);
% plot(AlphaMax,'--')
% plot(AlphaMin,'--')
% axis([0,  length(alpha_), alphaMin*1.25, alphaMax*1.25])
% grid on
% ylabel('\alpha','Interpreter','tex')
% xlabel('steps')


% state phi
sub4 = subplot(2,2,3);
% ani4 = animatedline('LineWidth',1.2,'Color','blue');
axis([0,  length(phi), min(phi), max(phi)])
plot(phi,'LineWidth',1.2,'Color','blue');
hold on
PhiMax = phiMax*ones(length(phi),1);
PhiMin = phiMin*ones(length(phi),1);
plot(PhiMax,'--','Color',"#0072BD",'LineWidth',1.2);
plot(PhiMin,'--','Color','red','LineWidth',1.2);
axis([0,  length(phi), phiMin*1.25, phiMax*1.25])
grid on
ylabel('\phi','Interpreter','tex')
xlabel('steps')


% state V
sub5 = subplot(2,2,4);
% ani5 = animatedline('LineWidth',1.2,'Color','blue');
axis([0,  length(V), min(V), max(V)])
plot(V,'LineWidth',1.2,'Color','blue');
hold on
PhiMax = VMax*ones(length(V),1);
PhiMin = VMin*ones(length(V),1);
plot(PhiMax,'--','Color',"#0072BD",'LineWidth',1.2);
plot(PhiMin,'--','Color','red','LineWidth',1.2);
axis([0,  length(V), VMin*0.75, VMax*1.25])
grid on
ylabel('V','Interpreter','tex')
xlabel('steps')

print -depsc stateConst.eps
%%
% get figure size
pos = get(gcf, 'Position');
width = pos(3); height = pos(4);

% preallocate data (for storing frame data)
mov = zeros(height, width, 1, length(psiDot), 'uint8');

stepCounter = sgtitle(sprintf('step = %.2f', 1));
for k = 1:length(psiDot)
    set(stepCounter, 'String', sprintf('step = %.2f',k));

    addpoints(ani1,k,psiDot(k))
    addpoints(ani2,k,gamma(k))
%     addpoints(ani3,k,alpha_(k))
    addpoints(ani4,k,phi(k))
    addpoints(ani5,k,V(k))

    drawnow
    pause(0.01)

% uncomment the following lines to record the simulation
%     f = getframe(gcf);
% 
%     if k == 1
%         [mov(:,:,1,k), map] = rgb2ind(f.cdata, 256, 'nodither');
%     else
%         mov(:,:,1,k) = rgb2ind(f.cdata, map, 'nodither');
%     end
end

% Create animated GIF
% imwrite(mov, map, 'stateConstraint.gif', 'DelayTime', 1/60, 'LoopCount', inf);

% end