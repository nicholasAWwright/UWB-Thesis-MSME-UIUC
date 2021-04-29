clear all
close all

%% Analyze rosbag data files for VICON windows/linux time sync testing

%Driving the new j8 around the CERL parking lot with a trimble prism tracking on top
Bag = rosbag('2021-01-29-16-25-14.bag');

%% Topic extraction
data = select(Bag,'Topic','/trimble/SX10/data');
dataRaw = select(Bag,'Topic','/trimble/SX10/dataRaw');

%% Message extraction
[ts0,cols0] = timeseries(data,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
t = ts0.time;
x = ts0.data(:,1);
y = ts0.data(:,2);
z = ts0.data(:,3);

[ts2,cols2] = timeseries(dataRaw,'Point.X','Point.Y','Point.Z');
t2 = ts2.time;
x2 = ts2.data(:,1);
y2 = ts2.data(:,2);
z2 = ts2.data(:,3);

%% Data analysis
close all
% set(gcf, 'Position',  [400, 250, 1800, 950])
%%plots

% 
% plot3(x,y,z,'.')
% axis equal
% plot(x2,y2,'.')
% plot(t2,y2,'.')
% plot(x,y,'.')

% % Adjust zero offset to match the picture
% x0 = 260;
% y0 = 540;
% 
% % Scale points to match 
% xscale = 6.5;
% yscale = 6.5;

% Rotate data to fit picture
theta = 0.025;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
v = [x';y'];
s = (R*v)';
xrot = s(:,1);
yrot = s(:,2);



%% Plot
close all
% Figure properties
width = 24; %figure size in inches
height = 12.8;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

%Scale the image axes to fit the data
xmin = -40;
xmax = 290;
ymin = -83;
ymax = 100;

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
data = imread('Satellite_Imagery_CERL.png') ;
image([xmin xmax], [ymin ymax],data);
hold on;
plot(0,0,'+','MarkerSize',10,'LineWidth',2)
hold on
plot(xrot,-yrot,'.')

axis equal
grid on
set(gca,'Units','normalized','Position',[.16 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Position Measurements of the J8 Robot using Trimble Total Station'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$East[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$North[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'SX-10 Origin','Measured Prism Position'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
text('VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs/SX-10_Outside_2';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png



