clear all
close all

%% Analyze rosbag data files for VICON windows/linux time sync testing
Bag = rosbag('2021-01-29-21-04-09-Derivative.bag');

%% Topic extraction
% VICONpose = select(Bag,'Topic','/uwb/dataFiltered/vicon');
rawTrimble = select(Bag,'Topic','/trimble/SX10/data');
rawUWBpoint0 = select(Bag,'Topic','/uwb/point0/decaPoint');
rawUWBpoint2 = select(Bag,'Topic','/uwb/point2/decaPoint');
rawUWBpose = select(Bag,'Topic','/uwb/orient/decaPose');
filteredTrimble = select(Bag,'Topic','/uwb/dataFiltered/trimble');
filteredUWBpoint = select(Bag,'Topic','/uwb/dataFiltered/uwb');

%% Message extraction
[ts00,cols00] = timeseries(rawUWBpose,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
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
angle_offset_deg = 45; % UWB point coordinates were not correctly entered at data acquisition
angle_offset_rad = deg2rad(angle_offset_deg);
angle_UWBpose_rad(:,3) = angle_UWBpose_rad(:,3) + angle_offset_rad;
uUWBpose = cos(angle_UWBpose_rad(:,3));
vUWBpose = sin(angle_UWBpose_rad(:,3));

[ts01,cols01] = timeseries(rawUWBpoint0,'Point.X','Point.Y','Point.Z');
tUWB_point0 = ts01.time;
xUWB_point0 = ts01.data(:,1);
yUWB_point0 = ts01.data(:,2);
zUWB_point0 = ts01.data(:,3);

[ts02,cols02] = timeseries(rawUWBpoint2,'Point.X','Point.Y','Point.Z');
tUWB_point2 = ts02.time;
xUWB_point2 = ts02.data(:,1);
yUWB_point2 = ts02.data(:,2);
zUWB_point2 = ts02.data(:,3);

[ts0,cols0] = timeseries(rawTrimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
tTrimble_raw = ts0.time;
xTrimble_raw = ts0.data(:,1);
yTrimble_raw = ts0.data(:,2);
zTrimble_raw = ts0.data(:,3);

[ts1,cols1] = timeseries(filteredUWBpoint,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z');
tUWB_filtered = ts1.time;
xUWB_filtered = ts1.data(:,1);
yUWB_filtered = ts1.data(:,2);
zUWB_filtered = ts1.data(:,3);

[ts2,cols2] = timeseries(filteredTrimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
tTrimble_filtered = ts2.time;
xTrimble_filtered = ts2.data(:,1);
yTrimble_filtered = ts2.data(:,2);
zTrimble_filtered = ts2.data(:,3);

%% Point cloud matching
% data has been filtered through an approximate time filter in order to
% match up points 1:1 for direct comparisons

% absor() requires row vectors
% absor() returns transform of first data set onto the second
UWB_data_filtered = [xUWB_filtered';yUWB_filtered'];
trimble_data_filtered = [xTrimble_filtered';yTrimble_filtered'];
[regParams,Bfit,ErrorStats]= absor(trimble_data_filtered,UWB_data_filtered,'doScale',0,'doTrans',1);

trimble_data_raw = [xTrimble_raw';yTrimble_raw'];
trimble_data_raw_transformed = regParams.s*regParams.R*trimble_data_raw + regParams.t;
trimble_data_filtered_transformed = regParams.s*regParams.R*trimble_data_filtered + regParams.t;
xTrimble_data_filtered_transformed = trimble_data_filtered_transformed(1,:)';
yTrimble_data_filtered_transformed = trimble_data_filtered_transformed(2,:)';

%% Statistics
euclidean_error = sqrt((xTrimble_data_filtered_transformed - xUWB_filtered).^2 + (yTrimble_data_filtered_transformed - yUWB_filtered).^2);
e_mean = mean(euclidean_error);
e_std = std(euclidean_error);

%% UWB Anchor Data
A0x = -5.971; A0y = 009.522; A0z = 3.848;
A1x = 09.474; A1y = 009.213; A1z = 2.990;
A2x = 09.446; A2y = -10.187; A2z = 2.976;
A3x = -7.176; A3y = -13.766; A3z = 2.893;
Ax = [A0x A1x A2x A3x];
Ay = [A0y A1y A2y A3y];
Az = [A0z A1z A2z A3z];
aLabels = {'UWB0','UWB1','UWB2','UWB3'};

%% Plot Unfiltered Point + Trimble Data
close all
% Figure properties
width = 6; %figure size in inches
height = 7.5;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(trimble_data_raw_transformed(1,:),trimble_data_raw_transformed(2,:),'Color', 'blue')
hold on
% plot(xUWBpose,yUWBpose,'.','Color',[0.8500 0.3250 0.0980])
% hold on
plot(xUWB_point0,yUWB_point0,'.','Color',[0.9290 0.6940 0.1250])
hold on
plot(xUWB_point2,yUWB_point2,'.','Color',[0.6350 0.0780 0.1840])
hold on
plot(Ax,Ay,'g*')
hold on 
plot(regParams.t(1),regParams.t(2),'+','MarkerSize',10,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
xlim([-9 12])
ylim([-15 11])
xticks([-9:12])
yticks([-15:11])

start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', e_mean*100);
std_str = sprintf('Error Std.Dev = %0.1f cm', e_std*100);
str = start_str + "\n" + mean_str + "\n" + std_str;
str = compose(str);
str = splitlines(str);
dim = [.32 .285 .0 .0]; %annotation location
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');

axis equal
grid on
set(gca,'Units','normalized','Position',[.14 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Comparison of Trimble SX-10 and Decawave TREK1000 UWB','Trilateration using Decawave TWR algorithm: 110kbps, Channel 2'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'SX-10 Data','UWB Point0','UWB Point2','UWB Anchors','SX-10 Origin'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
text(Ax,Ay,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs_pose/Trimble_vs_UWB_points_Tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Plot Unfiltered Pose and Trimble Data
close all
% Figure properties
width = 6; %figure size in inches
height = 7.5;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(trimble_data_raw_transformed(1,:),trimble_data_raw_transformed(2,:),'Color', 'blue')
hold on
plot(xUWBpose,yUWBpose,'.','Color',[0.8500 0.3250 0.0980])
hold on
plot(Ax,Ay,'g*')
hold on 
plot(regParams.t(1),regParams.t(2),'+','MarkerSize',10,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
xlim([-9 12])
ylim([-15 11])
xticks([-9:12])
yticks([-15:11])

start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', e_mean*100);
std_str = sprintf('Error Std.Dev = %0.1f cm', e_std*100);
str = start_str + "\n" + mean_str + "\n" + std_str;
str = compose(str);
str = splitlines(str);
dim = [.32 .29 .0 .0]; %annotation location
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');

axis equal
grid on
set(gca,'Units','normalized','Position',[.14 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Dual-tag UWB versus Trimble SX-10 position in the tent',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'SX-10 Data','UWB Position','UWB Anchors','SX-10 Origin'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
text(Ax,Ay,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs_pose/Trimble_vs_UWB_pose_Tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% error histogram

width = 6; %figure size in inches
height = 4;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
edges = 0:0.01:0.7;
histogram(euclidean_error,edges)

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Histogram of Euclidean error between Trimble SX-10 and dual-tag UWB'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', e_mean*100);
stdev_str = sprintf('Error Std.Dev = %0.1f cm', e_std*100);
str = start_str + "\n" + mean_str + "\n" + stdev_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');


filename = 'figs_pose/Error_Histogram_Trimble_Tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Plot Angles + Pose + Trimble
close all
% Figure properties
width = 6; %figure size in inches
height = 7.5;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

%filter results for less cluttered visualization
filter = 2;
xUWBpose_k = xUWBpose(1:filter:end);
yUWBpose_k = yUWBpose(1:filter:end);
uUWBpose_k = uUWBpose(1:filter:end);
vUWBpose_k = vUWBpose(1:filter:end);
    
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
quiver(xUWBpose_k,yUWBpose_k,uUWBpose_k, vUWBpose_k, 0,'Color',[0.8500 0.3250 0.0980]);
hold on
plot(xUWBpose_k,yUWBpose_k,'.','Color',[0.8500 0.3250 0.0980])
hold on
plot(trimble_data_raw_transformed(1,:),trimble_data_raw_transformed(2,:),'Color', 'blue')
hold on
plot(Ax,Ay,'g*')
hold on 
plot(regParams.t(1),regParams.t(2),'+','MarkerSize',10,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
xlim([-9 12])
ylim([-15 11])
xticks([-9:12])
yticks([-15:11])

start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', e_mean*100);
std_str = sprintf('Error Std.Dev = %0.1f cm', e_std*100);
str = start_str + "\n" + mean_str + "\n" + std_str;
str = compose(str);
str = splitlines(str);
dim = [.32 .29 .0 .0]; %annotation location
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');

axis equal
grid on
set(gca,'Units','normalized','Position',[.14 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Dual-tag UWB versus Trimble SX-10 position in the tent','Heading is calculated using dual-tag UWB'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'UWB Heading','UWB Pose','SX-10 Data','UWB Anchors','SX-10 Origin'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
text(Ax,Ay,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs_pose/Trimble_vs_UWB_Angles_Tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png



