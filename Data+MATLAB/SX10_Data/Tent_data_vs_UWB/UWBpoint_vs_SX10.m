clear all
close all

%% Analyze rosbag data files for VICON windows/linux time sync testing
rawBag = rosbag('2021-01-29-20-33-40.bag');
filteredBag = rosbag('2021-01-29-20-33-40-Derivative.bag');

%% Topic extraction
% VICONpose = select(Bag,'Topic','/uwb/dataFiltered/vicon');
rawTrimble = select(rawBag,'Topic','/trimble/SX10/data');
rawUWBpoint = select(rawBag,'Topic','/uwb/point0/decaPoint');
filteredTrimble = select(filteredBag,'Topic','/uwb/dataFiltered/trimble');
filteredUWBpoint = select(filteredBag,'Topic','/uwb/dataFiltered/uwb');

% %% Message extraction
% [ts0,cols0] = timeseries(VICONpose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% tVICON = ts0.time;
% xVICON = ts0.data(:,1);
% yVICON = ts0.data(:,2);
% zVICON = ts0.data(:,3);

%%
[ts4,cols4] = timeseries(rawUWBpoint,'Point.X','Point.Y','Point.Z');
tUWB_raw = ts4.time;
xUWB_raw = ts4.data(:,1);
yUWB_raw = ts4.data(:,2);
zUWB_raw = ts4.data(:,3);

[ts0,cols0] = timeseries(rawTrimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
tTrimble_raw = ts0.time;
xTrimble_raw = ts0.data(:,1);
yTrimble_raw = ts0.data(:,2);
zTrimble_raw = ts0.data(:,3);

[ts1,cols1] = timeseries(filteredUWBpoint,'Point.X','Point.Y','Point.Z');
tUWB_filtered = ts1.time;
xUWB_filtered = ts1.data(:,1);
yUWB_filtered = ts1.data(:,2);
zUWB_filtered = ts1.data(:,3);

[ts2,cols2] = timeseries(filteredTrimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
tTrimble_filtered = ts2.time;
xTrimble_filtered = ts2.data(:,1);
yTrimble_filtered = ts2.data(:,2);
zTrimble_filtered = ts2.data(:,3);

%% Offset UWB tag data by 1 frame to account for Decawave stated message delay
% % timestep  0 1 2 3.......n n+1
% % viconstep 0 1 2 3.......n n+1
% % UWB step -1 0 1 2...n-1 n
% % need to take 1 off the beginning of UWB and 1 off the end of everything else
% 
% tUWB_raw(1) = [];
% xUWB_raw(1) = [];
% yUWB_raw(1) = [];
% zUWB_raw(1) = [];
% 
% tTrimble_raw(end) = [];
% xTrimble_raw(end) = [];
% yTrimble_raw(end) = [];
% zTrimble_raw(end) = [];
% 
% tUWB_filtered(1) = [];
% xUWB_filtered(1) = [];
% yUWB_filtered(1) = [];
% zUWB_filtered(1) = [];
% 
% tTrimble_filtered(end) = [];
% xTrimble_filtered(end) = [];
% yTrimble_filtered(end) = [];
% zTrimble_filtered(end) = [];

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

%% Plot Filtered Data
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
plot(xUWB_raw,yUWB_raw,'.','Color',[0.8500 0.3250 0.0980])
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
title('Single-tag UWB versus Trimble SX-10 position in the tent',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'SX-10 Data','UWB Position','UWB Anchors','SX-10 Origin'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
text(Ax,Ay,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs_point/Trimble_vs_UWB_Tent';
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
title({'Histogram of Euclidean error between Trimble SX-10 and single-tag UWB'},...
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


filename = 'figs_point/Error_Histogram_Trimble_Tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png


