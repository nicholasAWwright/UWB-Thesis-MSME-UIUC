% clear all
% close all
%
% %% Analyze rosbag data files for VICON windows/linux time sync testing
% % rawBag = rosbag('2021-01-29-21-04-09.bag');
% Bag = rosbag('2021-02-25-20-56-49_derivative.bag');
% 
% %% Topic extraction
% VICONpose = select(Bag,'Topic','/vicon/j8/pose');
% rawTrimble = select(Bag,'Topic','/trimble/SX10/data');
% rawUWBpoint = select(Bag,'Topic','/uwb/point0/decaPoint');
% trimble2vicon = select(Bag,'Topic','/uwb/filtered/trimble_vicon');
% vicon2trimble = select(Bag,'Topic','/uwb/filtered/vicon_trimble');
% UWBpoint2vicon = select(Bag,'Topic','/uwb/filtered/uwbPoint2vicon');
% vicon2UWBpoint = select(Bag,'Topic','/uwb/filtered/vicon2uwbPoint');
% 
% %% Message extraction (takes a while)
% [ts00,cols00] = timeseries(VICONpose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z','Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');
% t_vicon = ts00.time;
% x_vicon = ts00.data(:,1);
% y_vicon = ts00.data(:,2);
% z_vicon = ts00.data(:,3);
% qx_vicon = ts00.data(:,4);
% qy_vicon = ts00.data(:,5);
% qz_vicon = ts00.data(:,6);
% qw_vicon = ts00.data(:,7);
% 
% [ts01,cols01] = timeseries(rawTrimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% t_trimble_raw = ts01.time;
% x_trimble_raw = ts01.data(:,1);
% y_trimble_raw = ts01.data(:,2);
% z_trimble_raw = ts01.data(:,3);
% 
% [ts0,cols0] = timeseries(rawUWBpoint,'Point.X','Point.Y','Point.Z');
% tUWB = ts0.time;
% xUWB = ts0.data(:,1);
% yUWB = ts0.data(:,2);
% zUWB = ts0.data(:,3);
% 
% [ts1,cols1] = timeseries(trimble2vicon,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% t_trimble2vicon = ts1.time;
% x_trimble2vicon = ts1.data(:,1);
% y_trimble2vicon = ts1.data(:,2);
% z_trimble2vicon = ts1.data(:,3);
% 
% [ts2,cols2] = timeseries(vicon2trimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% t_vicon2trimble = ts2.time;
% x_vicon2trimble = ts2.data(:,1);
% y_vicon2trimble = ts2.data(:,2);
% z_vicon2trimble = ts2.data(:,3);
% 
% [ts3,cols3] = timeseries(UWBpoint2vicon,'Point.X','Point.Y','Point.Z');
% t_uwb2vicon = ts3.time;
% x_uwb2vicon = ts3.data(:,1);
% y_uwb2vicon = ts3.data(:,2);
% z_uwb2vicon = ts3.data(:,3);
% 
% [ts4,cols4] = timeseries(vicon2UWBpoint,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z','Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');
% t_vicon2uwb = ts4.time;
% x_vicon2uwb = ts4.data(:,1);
% y_vicon2uwb = ts4.data(:,2);
% z_vicon2uwb = ts4.data(:,3);
% qx_vicon2uwb = ts4.data(:,4);
% qy_vicon2uwb = ts4.data(:,5);
% qz_vicon2uwb = ts4.data(:,6);
% qw_vicon2uwb = ts4.data(:,7);

%% Open saved .mat data to speed up the program
clear all
close all
currentFolder = pwd;
load('UWBpoint_and_SX10_vs_VICON_dynamic.mat');


%% Offset UWB tag data by 1 frame to account for Decawave stated message delay
% timestep  0 1 2 3.......n n+1
% viconstep 0 1 2 3.......n n+1
% UWB step -1 0 1 2...n-1 n
% need to take 1 off the beginning of UWB and 1 off the end of everything else

t_vicon(end) = [];
x_vicon(end) = [];
y_vicon(end) = [];
z_vicon(end) = [];
qx_vicon(end) = [];
qy_vicon(end) = [];
qz_vicon(end) = [];
qw_vicon(end) = [];

tUWB(1) = [];
xUWB(1) = [];
yUWB(1) = [];
zUWB(1) = [];

t_uwb2vicon(1) = [];
x_uwb2vicon(1) = [];
y_uwb2vicon(1) = [];
z_uwb2vicon(1) = [];

t_vicon2uwb(end) = [];
x_vicon2uwb(end) = [];
y_vicon2uwb(end) = [];
z_vicon2uwb(end) = [];
qx_vicon2uwb(end) = [];
qy_vicon2uwb(end) = [];
qz_vicon2uwb(end) = [];
qw_vicon2uwb(end) = [];

%% Angle stuff
quat_vicon = [qx_vicon, qy_vicon ,qz_vicon, qw_vicon];
[yaw_vicon_rad, pitch_vicon_rad, roll_vicon_rad] = quat2angle(quat_vicon,'ZYX'); %returns in radians
angle_vicon_rad = [yaw_vicon_rad, pitch_vicon_rad, roll_vicon_rad];
angle_vicon_deg = rad2deg(angle_vicon_rad);
u_vicon = cos(angle_vicon_rad(:,3)); % for quiver plots
v_vicon = sin(angle_vicon_rad(:,3));

quat_vicon2uwb = [qx_vicon2uwb, qy_vicon2uwb ,qz_vicon2uwb, qw_vicon2uwb];
[yaw_vicon2uwb_rad, pitch_vicon2uwb_rad, roll_vicon2uwb_rad] = quat2angle(quat_vicon2uwb,'ZYX'); %returns in radians
angle_vicon2uwb_rad = [yaw_vicon2uwb_rad, pitch_vicon2uwb_rad, roll_vicon2uwb_rad];
angle_vicon2uwb_deg = rad2deg(angle_vicon2uwb_rad);
u_vicon2uwb = cos(angle_vicon2uwb_rad(:,3));
v_vicon2uwb = sin(angle_vicon2uwb_rad(:,3));

%% Point cloud matching
% data has been filtered through an approximate time filter in order to
% match up points 1:1 for direct comparisons

% absor() requires row vectors
vicon = [x_vicon';y_vicon'];
vicon2uwb = [x_vicon2uwb';y_vicon2uwb'];
uwb2vicon = [x_uwb2vicon';y_uwb2vicon'];
vicon2trimble = [x_vicon2trimble';y_vicon2trimble'];
trimble2vicon = [x_trimble2vicon';y_trimble2vicon'];
vicon2trimble3 = [x_vicon2trimble';y_vicon2trimble';z_vicon2trimble'];
trimble2vicon3 = [x_trimble2vicon';y_trimble2vicon';z_trimble2vicon'];

% absor() returns transform of first data set onto the second
% fit both uwb and trimble datasets onto the vicon dataset as the groundtruth
[regParams_uwb2vicon,Bfit_uwb2vicon,ErrorStats_uwb2vicon]= absor(uwb2vicon,vicon2uwb,'doScale',0,'doTrans',1);
[regParams_trimble2vicon,Bfit_trimble2vicon,ErrorStats_trimble2vicon]= absor(trimble2vicon,vicon2trimble,'doScale',0,'doTrans',1);
[regParams_trimble2vicon3,Bfit_trimble2vicon3,ErrorStats_trimble2vicon3]= absor(trimble2vicon3,vicon2trimble3,'doScale',0,'doTrans',1);
[regParams_vicon2trimble3,Bfit_vicon2trimble3,ErrorStats_vicon2trimble3]= absor(vicon2trimble3,trimble2vicon3,'doScale',0,'doTrans',1);

uwb2vicon_transformed = regParams_uwb2vicon.s*regParams_uwb2vicon.R*uwb2vicon + regParams_uwb2vicon.t;
trimble2vicon_transformed = regParams_trimble2vicon.s*regParams_trimble2vicon.R*trimble2vicon + regParams_trimble2vicon.t;
trimble2vicon_transformed3 = regParams_trimble2vicon3.s*regParams_trimble2vicon3.R*trimble2vicon3 + regParams_trimble2vicon3.t;
vicon2trimble_transformed3 = regParams_vicon2trimble3.s*regParams_vicon2trimble3.R*vicon2trimble3 + regParams_vicon2trimble3.t;

%% Statistics
error_euclidean_uwb = sqrt((vicon2uwb(1,:) - uwb2vicon_transformed(1,:)).^2 + (vicon2uwb(2,:) - uwb2vicon_transformed(2,:)).^2);
error_mean_uwb = mean(error_euclidean_uwb);
error_std_uwb = std(error_euclidean_uwb);

error_trimble_x = trimble2vicon_transformed(1,:) - vicon2trimble(1,:);
error_mean_trimble_x = mean(error_trimble_x);
error_std_trimble_x = std(error_trimble_x);

error_trimble_y = trimble2vicon_transformed(2,:) - vicon2trimble(2,:);
error_mean_trimble_y = mean(error_trimble_y);
error_std_trimble_y = std(error_trimble_y);

error_trimble_z = trimble2vicon_transformed3(3,:) - vicon2trimble3(3,:);
error_mean_trimble_z = mean(error_trimble_z);
error_std_trimble_z = std(error_trimble_z);

error_euclidean_trimble = sqrt(error_trimble_x.^2 + error_trimble_y.^2);
error_mean_trimble = mean(error_euclidean_trimble);
error_std_trimble = std(error_euclidean_trimble);

%% Camera Poses (easier to parse in excel)
cam1qw	=	-0.005302896596655520	;	cam1qx	=	0.854780511714594000	;	cam1qy	=	-0.518946778880493000	;	cam1qz	=	0.004049292553442290	;	cam1x	=	85.4783214339778	;	cam1y	=	5739.3453303362300	;	cam1z	=	2157.4193748505100	;
cam2qw	=	0.643448207792986000	;	cam2qx	=	-0.606607893261426000	;	cam2qy	=	0.324976032278750000	;	cam2qz	=	0.335248931639296000	;	cam2x	=	-5324.3864737643500	;	cam2y	=	150.6426768282600	;	cam2z	=	2164.7409848795400	;
cam3qw	=	0.003405222315421470	;	cam3qx	=	0.952354875666912000	;	cam3qy	=	-0.304972910158748000	;	cam3qz	=	-0.000345432695766672	;	cam3x	=	3210.8776188786000	;	cam3y	=	5715.3319889504500	;	cam3z	=	2430.9378259227800	;
cam4qw	=	0.664097225965818000	;	cam4qx	=	-0.601856335699760000	;	cam4qy	=	0.280901131756527000	;	cam4qz	=	0.343275952872415000	;	cam4x	=	-5233.5645090130300	;	cam4y	=	-3102.3008186546600	;	cam4z	=	2469.8957168595700	;
cam5qw	=	0.602547766274727000	;	cam5qx	=	0.620484650325414000	;	cam5qy	=	-0.366097746999928000	;	cam5qz	=	0.343376510130588000	;	cam5x	=	5407.5852329686200	;	cam5y	=	376.7305148477740	;	cam5z	=	2143.4027290359400	;
cam6qw	=	0.893218471519558000	;	cam6qx	=	0.049130721579919600	;	cam6qy	=	-0.080985193458388200	;	cam6qz	=	0.439531947387001000	;	cam6x	=	3260.7976337421600	;	cam6y	=	-5182.7361886173400	;	cam6z	=	2455.5093098749800	;
cam7qw	=	-0.003203919981773300	;	cam7qx	=	0.889901346082022000	;	cam7qy	=	-0.455953863873216000	;	cam7qz	=	-0.013092102857875000	;	cam7x	=	-3203.7268978970300	;	cam7y	=	5691.0230311339200	;	cam7z	=	2436.7828751465100	;
cam8qw	=	0.672551222478594000	;	cam8qx	=	0.667360372828551000	;	cam8qy	=	-0.227337030898368000	;	cam8qz	=	0.224995245067538000	;	cam8x	=	5383.0872193147000	;	cam8y	=	-3090.1916711383900	;	cam8z	=	2375.5813524964500	;
cam9qw	=	0.620658613463301000	;	cam9qx	=	0.642849555161677000	;	cam9qy	=	-0.343126741625550000	;	cam9qz	=	0.289467397376079000	;	cam9x	=	5406.8316106079900	;	cam9y	=	3414.1269697815300	;	cam9z	=	2429.8422294872900	;
cam10qw	=	0.948192215279505000	;	cam10qx	=	0.004160232260453840	;	cam10qy	=	0.000897455709222263	;	cam10qz	=	0.317668710961803000	;	cam10x	=	-3166.3341520362000	;	cam10y	=	-5212.8734475048300	;	cam10z	=	2446.4898015268600	;
cam11qw	=	-0.647985533388807000	;	cam11qx	=	0.700986204475137000	;	cam11qy	=	-0.208974049483789000	;	cam11qz	=	-0.212280324798865000	;	cam11x	=	-5247.3438830317500	;	cam11y	=	3625.0835218258500	;	cam11z	=	2458.9488485566300	;
cam12qw	=	0.873430890924892000	;	cam12qx	=	-0.006947442284229040	;	cam12qy	=	-0.022430687015633800	;	cam12qz	=	0.486381615713282000	;	cam12x	=	105.2848250776710	;	cam12y	=	-5200.8766246558700	;	cam12z	=	2138.9916931298600	;

xCAM = [cam1x cam2x cam3x cam4x cam5x cam6x cam7x cam8x cam9x cam10x cam11x cam12x]/1000;
yCAM = [cam1y cam2y cam3y cam4y cam5y cam6y cam7y cam8y cam9y cam10y cam11y cam12y]/1000;
zCAM = [cam1z cam2z cam3z cam4z cam5z cam6z cam7z cam8z cam9z cam10z cam11z cam12z]/1000;

%% Anchor Postions (transformed by same absor() amount)
A0x = 09.474; A0y = 009.213; A0z = 2.990;
A1x = 09.446; A1y = -10.187; A1z = 2.976;
A2x = -7.176; A2y = -13.766; A2z = 2.893;
A3x = -5.971; A3y = 009.522; A3z = 3.848;
Ax = [A0x A1x A2x A3x];
Ay = [A0y A1y A2y A3y];
Az = [A0z A1z A2z A3z];
A = [Ax;Ay];
A_transformed = regParams_uwb2vicon.s*regParams_uwb2vicon.R*A + regParams_uwb2vicon.t;
Ax = A_transformed(1,:);
Ay = A_transformed(2,:);
aLabels = {'UWB0','UWB1','UWB2','UWB3'};
camLabels = {'VICON1','VICON2','VICON3','VICON4','VICON5','VICON6','VICON7','VICON8','VICON9','VICON10','VICON11','VICON12'};

% Figure settings
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

%% Vicon vs UWP Point
% Figure properties
width = 6; %figure size in inches
height = 8;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(vicon(1,:),vicon(2,:),uwb2vicon_transformed(1,:),uwb2vicon_transformed(2,:),'.')
hold on
plot(Ax,Ay,'g*',xCAM,yCAM,'mo')
xlim([-10 10])
ylim([-20 7])
xticks([-10:10])
yticks([-20:7])

start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', error_mean_uwb*100);
std_str = sprintf('Error Std.Dev = %0.1f cm', error_std_uwb*100);
str = start_str + "\n" + mean_str + "\n" + std_str;
str = compose(str);
str = splitlines(str);
dim = [.405 .37 .0 .0]; %annotation location
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');

axis equal
grid on
set(gca,'Units','normalized','Position',[.16 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Single-tag UWB versus Vicon ground truth position in the tent',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'VICON position','UWB position','UWB Anchors','VICON Cameras'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
text(Ax,Ay,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
     'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)

ax = gca;
filename = 'figs_point/Vicon_vs_UWB_Tent_single';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Error histogram - UWB Point

width = 6; %figure size in inches
height = 4;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
edges = 0:0.01:0.7;
histogram(error_euclidean_uwb,edges)

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Histogram of Euclidean error between Vicon and single-tag UWB'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', error_mean_uwb*100);
stdev_str = sprintf('Error Std.Dev = %0.1f cm', error_std_uwb*100);
str = start_str + "\n" + mean_str + "\n" + stdev_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');


filename = 'figs_point/Error_Histogram_Tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png


%% Plot Trimble vs VICON Positions

% Figure properties
width = 8; %figure size in inches
height = 6;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(vicon(1,:),vicon(2,:),trimble2vicon_transformed(1,:),trimble2vicon_transformed(2,:),'.')
hold on
plot(regParams_trimble2vicon.t(1),regParams_trimble2vicon.t(2),'+','MarkerSize',10,'LineWidth',2,'Color',[0.4940 0.1840 0.5560])
hold on
plot(xCAM,yCAM,'mo')
xlim([-10 6])
ylim([-7 7])
xticks([-10:6])
yticks([-7:7])

start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', error_mean_trimble*100);
std_str = sprintf('Error Std.Dev = %0.1f cm', error_std_trimble*100);
str = start_str + "\n" + mean_str + "\n" + std_str;
str = compose(str);
str = splitlines(str);
dim = [.215 .7 .0 .0]; %annotation location
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');

axis equal
grid on
set(gca,'Units','normalized','Position',[.16 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Comparison of VICON and Trimble SX-10 Total Station'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'VICON position','SX-10 position','SX-10 Origin','VICON Cameras'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthWest')

ax = gca;
filename = 'figs_trimble/Vicon_vs_Trimble_Tent_points';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png


%% Error histogram - Trimble Euclidean

width = 6; %figure size in inches
height = 4;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
edges = 0:.01:0.18;
histogram(error_euclidean_trimble,edges)

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Histogram of Euclidean error between Vicon and Trimble SX-10'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)

dim = [.57 .8 .0 .0]; %annotation
start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', 100*error_mean_trimble);
stdev_str = sprintf('Error Std.Dev = %0.1f cm', 100*error_std_trimble);
str = start_str + "\n" + mean_str + "\n" + stdev_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');


filename = 'figs_trimble/Trimble_Error_Histogram_Euclidean';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Error histogram - Trimble X

width = 6; %figure size in inches
height = 4;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% edges = -0.15:.01:0.15;
% histogram(error_trimble_x,edges)
histfit(error_trimble_x)
xlim([-0.16 0.16])
% xticks(-0.15:.03:0.15)

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Histogram of X-coordinate difference error between Vicon and Trimble SX-10'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Difference Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)

dim = [.6 .8 .0 .0]; %annotation
start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', 100*error_mean_trimble_x);
stdev_str = sprintf('Error Std.Dev = %0.1f cm', 100*error_std_trimble_x);
str = start_str + "\n" + mean_str + "\n" + stdev_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');


filename = 'figs_trimble/Trimble_Error_Histogram_X';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Error histogram - Trimble Y

width = 6; %figure size in inches
height = 4;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% edges = -0.15:.01:0.15;
% histogram(error_trimble_y,edges)
histfit(error_trimble_y)
xlim([-0.16 0.16])

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Histogram of Y-coordinate difference error between Vicon and Trimble SX-10'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Difference Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)

dim = [.6 .8 .0 .0]; %annotation
start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', 100*error_mean_trimble_y);
stdev_str = sprintf('Error Std.Dev = %0.1f cm', 100*error_std_trimble_y);
str = start_str + "\n" + mean_str + "\n" + stdev_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');


filename = 'figs_trimble/Trimble_Error_Histogram_Y';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Error histogram - Trimble Z

width = 6; %figure size in inches
height = 4;
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% edges = -0.15:.01:0.15;
% histogram(error_trimble_z)
histfit(error_trimble_z)
xlim([-0.16 0.16])

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Histogram of Z-coordinate difference error between Vicon and Trimble SX-10'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Difference Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)

dim = [.6 .8 .0 .0]; %annotation
start_str = sprintf("Euclidean Metric:");
mean_str = sprintf('Error Mean = %0.1f cm', 100*error_mean_trimble_z);
stdev_str = sprintf('Error Std.Dev = %0.1f cm', 100*error_std_trimble_z);
str = start_str + "\n" + mean_str + "\n" + stdev_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on',...
           'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'BackgroundColor','white');


filename = 'figs_trimble/Trimble_Error_Histogram_Z';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% Vicon2Trimble Analysis
% close all
% figure
% plot(trimble2vicon(1,:),trimble2vicon(2,:),vicon2trimble_transformed3(1,:),vicon2trimble_transformed3(2,:),'.')
% [vicon2trimble_theta,vicon2trimble_phi,vicon2trimble_r] = cart2sph(vicon2trimble_transformed3(1,:),vicon2trimble_transformed3(2,:),vicon2trimble_transformed3(3,:));
% 
% error_trimble_theta = x_trimble_raw' - vicon2trimble_r;
% error_trimble_phi = y_trimble_raw' - vicon2trimble_phi;
% error_trimble_r =  z_trimble_raw' - vicon2trimble_theta;
% figure
% histogram(error_trimble_theta)
% figure
% histogram(error_trimble_phi)
% figure
% histogram(error_trimble_r)

