% script to plot the bar chart comparison between the three tables in AIAA
% journal 

NMAC = [0,0,0
     0,0,0
     0,0,0
     1.36,0.68,0.24
     9.96,2.12,1.48];
X = categorical({'2','4','8','16','32'});
X = reordercats(X,{'2','4','8','16','32'});
figure(1)
h = bar(X,NMAC);
set(h, {'DisplayName'}, {'Baseline','Action sheilding','Reward shaping'}')
% set(gca,'FontSize',13)
% Legend will show names for each color
lgd = legend(Location="northwest");
lgd.FontSize = 14;
lgd.FontName = 'Times New Roman';
ax = gca;
ax.FontSize = 14; 
ylabel('LOS', 'FontSize',14)
xlabel('Number of agents', 'FontSize',14)
tit = title('LOS performance comparison');
tit.FontName = 'Times New Roman';
tit.FontSize = 14;
print -depsc NMAC_bar.eps


compTime = [0.07,0,0
     0.13,0,0.23
     0.46,0.87,0.87
     3.42,2.64,3.00
     8.69,7.24,7.37];
X = categorical({'2','4','8','16','32'});
X = reordercats(X,{'2','4','8','16','32'});
figure(2)
H = bar(X,compTime);
set(H, {'DisplayName'}, {'Baseline','Action sheilding','Reward shaping'}')
% set(gca,'FontSize',13)
% Legend will show names for each color
lgd = legend(Location="northwest");
lgd.FontSize = 14;
lgd.FontName = 'Times New Roman';
ax = gca;
ax.FontSize = 14; 
ylabel('Computation time [sec]', 'FontSize',14)
xlabel('Number of agents', 'FontSize',14)
tit = title('Computation time comparison');
tit.FontName = 'Times New Roman';
tit.FontSize = 14;
print -depsc CompTIme_bar.eps