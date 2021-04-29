clear all
close all

%% Analyze rosbag data files for VICON windows/linux time sync testing
% rawBag = rosbag('2021-01-29-21-04-09.bag');
Bag = rosbag('2021-02-25-20-06-59.bag');

%% Topic extraction
VICONpose = select(Bag,'Topic','/vicon/j8/pose');
rawTrimble = select(Bag,'Topic','/trimble/SX10/data');
rawUWBpose = select(Bag,'Topic','/uwb/orient/decaPose');
trimble2vicon = select(Bag,'Topic','/uwb/filtered/trimble_vicon');
vicon2trimble = select(Bag,'Topic','/uwb/filtered/vicon_trimble');
UWBpose2vicon = select(Bag,'Topic','/uwb/dataFiltered/uwb_vicon');
vicon2UWBpose = select(Bag,'Topic','/uwb/dataFiltered/vicon_uwb');

%% Message extraction (takes a while)
[ts00,cols00] = timeseries(VICONpose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z','Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');
t_vicon = ts00.time;
x_vicon = ts00.data(:,1);
y_vicon = ts00.data(:,2);
z_vicon = ts00.data(:,3);
qx_vicon = ts00.data(:,4);
qy_vicon = ts00.data(:,5);
qz_vicon = ts00.data(:,6);
qw_vicon = ts00.data(:,7);
quat_vicon = [qx_vicon, qy_vicon ,qz_vicon, qw_vicon];
[yaw_vicon_rad, pitch_vicon_rad, roll_vicon_rad] = quat2angle(quat_vicon,'ZYX'); %returns in radians
angle_vicon_rad = [yaw_vicon_rad, pitch_vicon_rad, roll_vicon_rad];
angle_vicon_deg = rad2deg(angle_vicon_rad);
u_vicon = cos(angle_vicon_rad(:,3)); % for quiver plots
v_vicon = sin(angle_vicon_rad(:,3));

[ts01,cols01] = timeseries(rawTrimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
t_trimble_raw = ts01.time;
x_trimble_raw = ts01.data(:,1);
y_trimble_raw = ts01.data(:,2);
z_trimble_raw = ts01.data(:,3);

[ts02,cols02] = timeseries(rawUWBpose,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
t_UWBpose = ts02.time;
x_UWBpose = ts02.data(:,1);
y_UWBpose = ts02.data(:,2);
z_UWBpose = ts02.data(:,3);
qx_UWBpose = ts02.data(:,4);
qy_UWBpose = ts02.data(:,5);
qz_UWBpose = ts02.data(:,6);
qw_UWBpose = ts02.data(:,7);
quat_UWBpose = [qx_UWBpose, qy_UWBpose ,qz_UWBpose, qw_UWBpose];
[yaw_UWBpose_rad, pitch_UWBpose_rad, roll_UWBpose_rad] = quat2angle(quat_UWBpose,'ZYX'); %returns in radians
angle_UWBpose_rad = [yaw_UWBpose_rad, pitch_UWBpose_rad, roll_UWBpose_rad];
angle_UWBpose_deg = rad2deg(angle_UWBpose_rad);
u_UWBpose = cos(angle_UWBpose_rad(:,3));
v_UWBpose = sin(angle_UWBpose_rad(:,3));

[ts1,cols1] = timeseries(trimble2vicon,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
t_trimble2vicon = ts1.time;
x_trimble2vicon = ts1.data(:,1);
y_trimble2vicon = ts1.data(:,2);
z_trimble2vicon = ts1.data(:,3);

[ts2,cols2] = timeseries(vicon2trimble,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
t_vicon2trimble = ts2.time;
x_vicon2trimble = ts2.data(:,1);
y_vicon2trimble = ts2.data(:,2);
z_vicon2trimble = ts2.data(:,3);

[ts3,cols3] = timeseries(UWBpose2vicon,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
t_uwb2vicon = ts3.time;
x_uwb2vicon = ts3.data(:,1);
y_uwb2vicon = ts3.data(:,2);
z_uwb2vicon = ts3.data(:,3);
qx_uwb2vicon = ts3.data(:,4);
qy_uwb2vicon = ts3.data(:,5);
qz_uwb2vicon = ts3.data(:,6);
qw_uwb2vicon = ts3.data(:,7);
quat_uwb2vicon = [qx_uwb2vicon, qy_uwb2vicon ,qz_uwb2vicon, qw_uwb2vicon];
[yaw_uwb2vicon_rad, pitch_uwb2vicon_rad, roll_uwb2vicon_rad] = quat2angle(quat_uwb2vicon,'ZYX'); %returns in radians
angle_uwb2vicon_rad = [yaw_uwb2vicon_rad, pitch_uwb2vicon_rad, roll_uwb2vicon_rad];
angle_uwb2vicon_deg = rad2deg(angle_uwb2vicon_rad);
u_uwb2vicon = cos(angle_uwb2vicon_rad(:,3));
v_uwb2vicon = sin(angle_uwb2vicon_rad(:,3));


[ts4,cols4] = timeseries(vicon2UWBpose,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z','Pose.Orientation.X','Pose.Orientation.Y','Pose.Orientation.Z','Pose.Orientation.W');
t_vicon2uwb = ts4.time;
x_vicon2uwb = ts4.data(:,1);
y_vicon2uwb = ts4.data(:,2);
z_vicon2uwb = ts4.data(:,3);
qx_vicon2uwb = ts4.data(:,4);
qy_vicon2uwb = ts4.data(:,5);
qz_vicon2uwb = ts4.data(:,6);
qw_vicon2uwb = ts4.data(:,7);
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

% absor() returns transform of first data set onto the second
% fit both uwb and trimble datasets onto the vicon dataset as the
% groundtruth
[regParams_uwb2vicon,Bfit_uwb2vicon,ErrorStats_uwb2vicon]= absor(uwb2vicon,vicon2uwb,'doScale',0,'doTrans',1);
[regParams_trimble2vicon,Bfit_trimble2vicon,ErrorStats_trimble2vicon]= absor(trimble2vicon,vicon2trimble,'doScale',0,'doTrans',1);

uwb2vicon_transformed = regParams_uwb2vicon.s*regParams_uwb2vicon.R*uwb2vicon + regParams_uwb2vicon.t;
trimble2vicon_transformed = regParams_trimble2vicon.s*regParams_trimble2vicon.R*trimble2vicon + regParams_trimble2vicon.t;


%% Check Plots

% plot(x_vicon,y_vicon,'.')
% hold on
% plot(x_UWBpose,y_UWBpose,'.')
% 
% plot(x_vicon,y_vicon,'.')
% hold on
% plot(x_trimble_raw,y_trimble_raw,'.')
% 
% plot(x_vicon2uwb,y_vicon2uwb,'.')
% hold on
% plot(x_uwb2vicon,y_uwb2vicon,'.')
% 
% plot(x_vicon2trimble,y_vicon2trimble,'.')
% hold on
% plot(x_trimble2vicon,y_trimble2vicon,'.')


%% Plot Filtered Data
close all

figure
plot(vicon(1,:),vicon(2,:),'.','MarkerSize',5)
hold on
plot(uwb2vicon_transformed(1,:),uwb2vicon_transformed(2,:),'.','MarkerSize',15)
grid on
legend('vicon','UWBpose')
axis equal

figure
plot(vicon(1,:),vicon(2,:),'.','MarkerSize',5)
hold on
plot(trimble2vicon_transformed(1,:),trimble2vicon_transformed(2,:),'.','MarkerSize',15)
grid on
legend('vicon','trimble')
axis equal

figure
plot(vicon(1,:),vicon(2,:),'.','MarkerSize',5)
hold on
plot(uwb2vicon_transformed(1,:),uwb2vicon_transformed(2,:),'.','MarkerSize',15)
hold on
plot(trimble2vicon_transformed(1,:),trimble2vicon_transformed(2,:),'.','MarkerSize',15)
grid on
legend('vicon','uwb','trimble')
axis equal

%% plot headings
close all

figure
quiver(x_vicon2uwb,y_vicon2uwb,u_vicon2uwb, v_vicon2uwb, 0);
axis equal


%% Statistics
error_euclidean_uwb = sqrt((vicon2uwb(1,:) - uwb2vicon_transformed(1,:)).^2 + (vicon2uwb(2,:) - uwb2vicon_transformed(2,:)).^2);
error_mean_uwb = mean(error_euclidean_uwb);
error_std_uwb = std(error_euclidean_uwb);

error_euclidean_trimble = sqrt((vicon2trimble(1,:) - trimble2vicon_transformed(1,:)).^2 + (vicon2trimble(2,:) - trimble2vicon_transformed(2,:)).^2);
error_mean_trimble = mean(error_euclidean_trimble);
error_std_trimble = std(error_euclidean_trimble);

%% Camera Poses
% cam1qw=0.858624074795118;    cam1qx=0.0063588920558668;  cam1qy= -0.0114107661882249; cam1qz= 0.512439320397106;   cam1x=87.7535238137889;  cam1y= -3724.00665640508;  cam1z= 1463.43424722973;
% cam2qw=0.00885166539731656;  cam2qx= 0.855427117479611;  cam2qy= -0.517810048989183;  cam2qz= 0.00623280563129276; cam2x=170.398962209271;  cam2y= 4146.66078525202;   cam2z= 1456.20177307327;
% cam3qw=0.493452019149796;    cam3qx= 0.713510656409674;  cam3qy= -0.40800968965731;   cam3qz= 0.284492075693808;   cam3x=2909.74472661161;  cam3y= 2037.95587071347;   cam3z= 1751.56500219415;
% cam4qw=0.604190045885996;    cam4qx= 0.610812369167507;  cam4qy= -0.368511780918908;  cam4qz= 0.35505169405064;    cam4x=2884.79209455769;  cam4y= 122.656054945728;   cam4z= 1465.78036853942;
% cam5qw=-0.323247996509163;   cam5qx= 0.781260911280642;  cam5qy= -0.495003477133873;  cam5qz= -0.200283995574108;  cam5x=-2825.77981358908; cam5y= 3920.82301442049;   cam5z= 1750.00346240647;
% cam6qw=0.604847542431433;    cam6qx= -0.604769090000534; cam6qy= 0.362206994186371;   cam6qz= 0.370432033654024;   cam6x=-2832.70531483627; cam6y= 221.880447099351;   cam6z= 1467.67556690142;
% cam7qw=0.796178585758678;    cam7qx= 0.319269887936899;  cam7qy= -0.186221236826518;  cam7qz= 0.479049109372801;   cam7x=2861.2097582114;   cam7y= -3514.58636363406;  cam7z= 1758.05856684576;
% cam8qw=-0.469250884029611;   cam8qx= 0.74293151669288;   cam8qy= -0.413429021625054;  cam8qz= -0.238605979430516;  cam8x=-2828.90876570326; cam8y= 2182.04109924523;   cam8z= 1745.28833173837;
% cam9qw=0.287371029684088;    cam9qx= 0.792887902284394;  cam9qy= -0.503840862645741;  cam9qz= 0.186791463503438;   cam9x=2725.82218275622;  cam9y= 4150.95296838581;   cam9z= 1757.89497675724;
% cam10qw=0.798378770822862;   cam10qx= -0.33064762239193; cam10qy= 0.20008984669996;   cam10qz= 0.461765677972635;  cam10x=-2874.59451811957;cam10y= -3483.86802295117; cam10z= 1745.29275074808;
% cam11qw=0.722815892135711;   cam11qx= -0.474123061956425;cam11qy= 0.276594271126765;  cam11qz= 0.419809620395928;  cam11x=-2843.62501343957;cam11y= -1740.13654548375; cam11z= 1748.18417549821;
% cam12qw=0.739844738748693;   cam12qx= 0.472881291048359; cam12qy= -0.252570710105516; cam12qz= 0.406473964134373;  cam12x=2865.72534253268; cam12y= -1611.80047754914; cam12z= 1749.59725450252;
% 
% xCAM = [cam1x cam2x cam3x cam4x cam5x cam6x cam7x cam8x cam9x cam10x cam11x cam12x]/1000;
% yCAM = [cam1y cam2y cam3y cam4y cam5y cam6y cam7y cam8y cam9y cam10y cam11y cam12y]/1000;
% zCAM = [cam1z cam2z cam3z cam4z cam5z cam6z cam7z cam8z cam9z cam10z cam11z cam12z]/1000;

%% Plotting
% % Figure properties
% width = 6; %figure size in inches
% height = 8;
% x0 = 10; %position on screen
% y0 = 4;
% titleFontSize = 12;
% axisFontSize = 9;
% legendFontSize = 9;
% Font = 'Times';
% 
% 
% close all
% %Anchor Postions
% A0x = -3.112; A0y = 04.195; A0z = 2.169;
% A1x = 03.223; A1y = 04.424; A1z = 2.176;
% A2x = 03.087; A2y = -6.883; A2z = 2.070;
% A3x = -3.170; A3y = -3.857; A3z = 2.592;
% Ax = [A0x A1x A2x A3x];
% Ay = [A0y A1y A2y A3y];
% Az = [A0z A1z A2z A3z];
% aLabels = {'UWB0','UWB1','UWB2','UWB3'};
% camLabels = {'VICON1','VICON2','VICON3','VICON4','VICON5','VICON6','VICON7','VICON8','VICON9','VICON10','VICON11','VICON12'};
% 
% % plot(tVICON,xVICON)
% % plot(xVICON,yVICON)
% % plot(xUWB,yUWB)
% % plot3(xVICON,yVICON,zVICON,xUWB,yUWB,zUWB)
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot(xVICON,yVICON,xUWB,yUWB,'.')
% hold on
% plot(Ax,Ay,'g*',xCAM,yCAM,'mo')
% xlim([-4 4])
% ylim([-7 5])
% 
% start_str = sprintf("Euclidean Metric:");
% mean_str = sprintf('Error Mean = %0.1f cm', e_mean*100);
% std_str = sprintf('Error StDev = %0.1f cm', e_std*100);
% str = start_str + "\n" + mean_str + "\n" + std_str;
% str = compose(str);
% str = splitlines(str);
% dim = [.405 .37 .0 .0]; %annotation location
% annotation('textbox',dim,'String',str,'FitBoxToText','on',...
%            'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font);
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'Comparison of VICON and UWB in the USACE CERL ECR.','Trilateration using Decawave TWR algorithm: 110kbps, Channel 2.'},...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'VICON position','UWB tag0 position','UWB Anchors','VICON Cameras'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
% text(Ax,Ay,aLabels,'VerticalAlignment','bottom','HorizontalAlignment','left',...
%      'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% 
% 
% % 3D pdf!
% ax = gca;
% filename = 'ECR-TREK1000-Data';
% print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint









