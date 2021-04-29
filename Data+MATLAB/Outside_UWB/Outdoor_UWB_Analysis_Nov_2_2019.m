clear all
close all

%% Analyze rosbag data files for VICON windows/linux time sync testing

Bag = rosbag('2019-11-02-18-56-29.bag');

%% Topic extraction
UWB0 = select(Bag,'Topic','/uwb/point0/decaPoint');
UWB2 = select(Bag,'Topic','/uwb/point2/decaPoint');
UWBpose = select(Bag,'Topic','/uwb/orient/decaPose');

%% Message extraction
[ts0,cols0] = timeseries(UWB0,'Point.X','Point.Y','Point.Z');
tUWB0 = ts0.time;
xUWB0 = ts0.data(:,1);
yUWB0 = ts0.data(:,2);
zUWB0 = ts0.data(:,3);

[ts2,cols2] = timeseries(UWB2,'Point.X','Point.Y','Point.Z');
tUWB2 = ts2.time;
xUWB2 = ts2.data(:,1);
yUWB2 = ts2.data(:,2);
zUWB2 = ts2.data(:,3);

[ts00,cols00] = timeseries(UWBpose,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
tUWBpose = ts00.time;
xUWBpose = ts00.data(:,1);
yUWBpose = ts00.data(:,2);
zUWBpose = ts00.data(:,3);
qx_UWBpose = ts00.data(:,4);
qy_UWBpose = ts00.data(:,5);
qz_UWBpose = ts00.data(:,6);
qw_UWBpose = ts00.data(:,7);
quat_UWBpose = [qx_UWBpose, qy_UWBpose ,qz_UWBpose, qw_UWBpose];
[yaw_UWBpose_rad, pitch_UWBpose_rad, roll_UWBpose_rad] = quat2angle(quat_UWBpose,'ZYX'); %returns in radians
angle_UWBpose_rad = [yaw_UWBpose_rad, pitch_UWBpose_rad, roll_UWBpose_rad];
angle_UWBpose_deg = rad2deg(angle_UWBpose_rad);
uUWBpose = cos(angle_UWBpose_rad(:,3));
vUWBpose = sin(angle_UWBpose_rad(:,3));


%% Anchor Locations
xAnchor = [-19.466,23.303,21.680,-23.973];
yAnchor = [17.873,13.906,-14.846,-13.657];
zAnchor = [2.110,1.888,1.655,1.852];
aLabels = {'UWB0','UWB1','UWB2','UWB3'};

%% Pose plot
close all
% Figure properties
width = 8; %figure size in inches
height = 6;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
quiver(xUWBpose,yUWBpose,uUWBpose,vUWBpose,0.25)
hold on
plot(xUWBpose,yUWBpose,'.')
plot(xAnchor,yAnchor,'g*')
xlim([-30 35])
ylim([-25 25])
xticks([-30:5:35])
yticks([-25:5:25])


axis equal
grid on
set(gca,'Units','normalized','Position',[.14 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'UWB Pose Measurements of the Jackal Robot Outside at Maximum Antenna Range'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'UWB Heading','UWB pose','UWB Anchors'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
text(xAnchor,yAnchor,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs/UWB_pose_outside';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png


%% Points plot
close all
% Figure properties
width = 8; %figure size in inches
height = 6;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(xUWB0,yUWB0,'.')
hold on
plot(xUWB2,yUWB2,'.')
hold on
plot(xAnchor,yAnchor,'g*')
xlim([-30 35])
ylim([-25 25])
xticks([-30:5:35])
yticks([-25:5:25])


axis equal
grid on
set(gca,'Units','normalized','Position',[.14 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'UWB Point Measurements of the Jackal Robot Outside at Maximum Antenna Range'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'UWB tag0 position','UWB tag2 position','UWB Anchors'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
text(xAnchor,yAnchor,aLabels,'VerticalAlignment','top','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs/UWB_points_outside';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png


