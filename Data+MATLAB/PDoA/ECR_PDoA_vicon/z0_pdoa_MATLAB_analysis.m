% %% Data extraction from bag file (Just do this once and then save workspace)
% clear all
% close all
% 
% % Import rosbag data file
% Bag = rosbag('2020-03-17-17-55-04_pdoa-vicon-3D-center-and-around.bag');
% 
% % Topic extraction
% EXPdata = select(Bag,'Topic','/uwb/pdoaVICON/dataFiltered');
% 
% % Message extraction (need to define custom ROS message for MATLAB: https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html)
% [ts,cols] = timeseries(EXPdata,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% % This takes a relatively long time
% 
% %Populate MATLAB variables from timeseries data
% t = ts.time;
% dist_horz_pdoa = ts.data(:,1);
% pdoa_horz = ts.data(:,2);
% dist_vert_pdoa = ts.data(:,3);
% pdoa_vert = ts.data(:,4);
% xVICON = ts.data(:,5);
% yVICON = ts.data(:,6);
% zVICON = ts.data(:,7);
% vicon = [xVICON,yVICON,zVICON];

%% Data manipulation
clear all
close all

currentFolder = pwd;
load('z1_vicon_floor_experiment1.mat');

pdoa_node_midl = [0,0,0.8163]; %location of the PDOA node in vicon arena, halfway between horz and vert
pdoa_node_ofst = [0,0,0.0255]; %Half the distance between each pdoa node center
pdoa_node_horz = pdoa_node_midl - pdoa_node_ofst;
pdoa_node_vert = pdoa_node_midl + pdoa_node_ofst;

zVICON = zVICON - pdoa_node_midl(3); %transform VICON frame origin to be coincident with PDoA frame origin by adjusting Z

angles_horz_rad = atan2(yVICON,xVICON); %angle in the vicon XY-plane
angles_horz_deg = rad2deg(angles_horz_rad);
angles_horz_rad_adj = wrapToPi(angles_horz_rad - pi()/2); %adjusted to be +/- from PDOA node zero angle during experiment rather than vicon zero angle
angles_horz_deg_adj = wrapTo180(angles_horz_deg - 90);

angles_vert_rad = -atan2( zVICON, sign(yVICON) .* sqrt(xVICON.^2 + yVICON.^2) ); %negative because of physical antenna orientation opposite to standard convention
angles_vert_deg = rad2deg(angles_vert_rad);
angles_vert_pos_rad = -atan2( zVICON, sqrt(xVICON.^2 + yVICON.^2) ); %all vertical angles constrained to be positive
angles_vert_pos_deg = rad2deg(angles_vert_pos_rad);

dist_pdoa = (dist_horz_pdoa + dist_vert_pdoa) / 2; %average distance from PDoA shield
dist_horz_vicon = sqrt(xVICON.^2 + yVICON.^2); %horz distance from PDOA shield
dist_vert_vicon = sqrt(yVICON.^2 + zVICON.^2); %vert distance from PDOA shield
dist_elev_vicon = sqrt( xVICON.^2 + sqrt(yVICON.^2 + zVICON.^2) ); %elevation angle distance from PDOA shield
dist_vicon = sqrt(xVICON.^2 + yVICON.^2 + zVICON.^2); %distance from PDOA shield

data_raw = [pdoa_horz,dist_horz_pdoa,pdoa_vert,dist_vert_pdoa,xVICON,yVICON,zVICON]; %raw data collected in experiment with transformed VICON frame origin to be coincident with PDoA frame origin by adjusting Z
data_aug = [data_raw,angles_horz_deg_adj, angles_horz_rad_adj, dist_horz_vicon, angles_vert_deg, angles_vert_rad, dist_vert_vicon, dist_vicon]; %added $AoA_{horz}$ [deg] and distance from node to act as filter variables

% dataset filtering
run('z2_location_segmenting.m') %removes data corresponding to stray vicon points and creates variable data_good = data_aug - stray_vicon_points

% averages of data_aug variables at each separate location in space of data collection
pdoa_horz_avg = pointStats(:,1);
dist_horz_pdoa_avg = pointStats(:,2);
pdoa_vert_avg = pointStats(:,3);
dist_vert_pdoa_avg = pointStats(:,4);
xVICON_avg = pointStats(:,5);
yVICON_avg = pointStats(:,6);
zVICON_avg = pointStats(:,7);
angles_horz_deg_adj_avg = pointStats(:,8);
angles_horz_rad_adj_avg = pointStats(:,9);
dist_horz_vicon_avg = pointStats(:,10);
angles_vert_deg_avg = pointStats(:,11);
angles_vert_rad_avg = pointStats(:,12);
dist_vert_vicon_avg = pointStats(:,13);
dist_vicon_avg = pointStats(:,14);
dist_pdoa_avg = (dist_horz_pdoa_avg + dist_vert_pdoa_avg) / 2;

%initialize matrices
data5 = data_good;
data15 = data_good;
data35 = data_good;
data50 = data_good;
data60 = data_good;
data75 = data_good;
data90 = data_good;
data75plus = data_good;

pointAvg5 = pointStats;
pointAvg15 = pointStats;
pointAvg35 = pointStats;
pointAvg50 = pointStats;
pointAvg60 = pointStats;
pointAvg75 = pointStats;
pointAvg90 = pointStats;
pointAvg75plus = pointStats;

% full data logical tests
data5_test = abs(data_good(:,8)) > 5; % +/- 5 degrees or more
data15_test = abs(data_good(:,8)) > 15; % +/- 15 degrees or more
data35_test = abs(data_good(:,8)) > 35; % +/- 35 degrees or more
data50_test = abs(data_good(:,8)) > 50; % +/- 50 degrees or more
data60_test = abs(data_good(:,8)) > 60; % +/- 60 degrees or more
data75_test = abs(data_good(:,8)) > 75; % +/- 75 degrees or more
data90_test = abs(data_good(:,8)) > 90; % +/- 90 degrees or more
data75plus_test = abs(data_good(:,8)) < 75; % data +/- 75 degrees or greater

% average logical tests
data5_avg_test = abs(pointStats(:,8)) > 5; % +/- 5 degrees or more
data15_avg_test = abs(pointStats(:,8)) > 15; % +/- 15 degrees or more
data35_avg_test = abs(pointStats(:,8)) > 35; % +/- 35 degrees or more
data50_avg_test = abs(pointStats(:,8)) > 50; % +/- 50 degrees or more
data60_avg_test = abs(pointStats(:,8)) > 60; % +/- 60 degrees or more
data75_avg_test = abs(pointStats(:,8)) > 75; % +/- 75 degrees or more
data90_avg_test = abs(pointStats(:,8)) > 90; % +/- 90 degrees or more
data75plus_avg_test = abs(pointStats(:,8)) < 75; % data +/- 75 degrees or greater

%filter out all logical trues to leave the 
data5(data5_test,:) = [];
data15(data15_test,:) = [];
data35(data35_test,:) = [];
data50(data50_test,:) = [];
data60(data60_test,:) = [];
data75(data75_test,:) = [];
data90(data90_test,:) = [];
data75plus(data75plus_test,:) = [];

pointAvg5(data5_avg_test,:) = [];
pointAvg15(data15_avg_test,:) = [];
pointAvg35(data35_avg_test,:) = [];
pointAvg50(data50_avg_test,:) = [];
pointAvg60(data60_avg_test,:) = [];
pointAvg75(data75_avg_test,:) = [];
pointAvg90(data90_avg_test,:) = [];
pointAvg75plus(data75plus_avg_test,:) = [];

% MATLAB neural network variables
input5 = data5(:,1:4);
output5 = data5(:,5:7);
output5_2d = data5(:,5:6);
output5_z = data5(:,7);

input15 = data15(:,1:4);
output15 = data15(:,5:7);
output15_2d = data15(:,5:6);
output15_z = data15(:,7);

input35 = data35(:,1:4);
output35 = data35(:,5:7);
output35_2d = data35(:,5:7);

input50 = data50(:,1:4);
output50 = data50(:,5:7);

input60 = data60(:,1:4);
output60 = data60(:,5:7);
output60_2d = data60(:,5:6);
output60_z = data60(:,7);

input75 = data75(:,1:4);
output75 = data75(:,5:7);

input90 = data90(:,1:4);
output90 = data90(:,5:7);

input75plus = data75plus(:,1:4);
output75plus = data75plus(:,5:7);

input_full = data_raw(:,1:4);
output_full = data_raw(:,5:7);

% filtering 50in data
pdoa_vert_50in = pdoa_vert; 
pdoa_horz_50in = pdoa_horz;
angles_vert_deg_50in = angles_vert_deg;
angles_horz_deg_adj_50in = angles_horz_deg_adj;
dist_pdoa_50in = dist_pdoa;
dist_vicon_50in = dist_vicon;

pdoa_vert_50in(data50_test) = [];
pdoa_horz_50in(data50_test) = [];
angles_vert_deg_50in(data50_test) = [];
angles_horz_deg_adj_50in(data50_test) = [];
dist_pdoa_50in(data50_test) = [];
dist_vicon_50in(data50_test) = [];

% 50in averages
pdoa_horz_avg_50in = pointStats(:,1);
pdoa_vert_avg_50in = pointStats(:,3);
angles_horz_deg_adj_avg_50in = pointStats(:,8);
angles_vert_deg_avg_50in = pointStats(:,11);
dist_pdoa_avg_50in = (pointStats(:,2) + pointStats(:,4)) / 2;

pdoa_horz_avg_50in(data50_avg_test) = [];
pdoa_vert_avg_50in(data50_avg_test) = [];
angles_horz_deg_adj_avg_50in(data50_avg_test) = [];
angles_vert_deg_avg_50in(data50_avg_test) = [];
dist_pdoa_avg_50in(data50_avg_test) = [];

% filtering 60in data
pdoa_vert_60in = pdoa_vert; 
pdoa_horz_60in = pdoa_horz;
angles_vert_deg_60in = angles_vert_deg;
angles_horz_deg_adj_60in = angles_horz_deg_adj;
dist_pdoa_60in = dist_pdoa;
dist_vicon_60in = dist_vicon;

pdoa_vert_60in(data60_test) = [];
pdoa_horz_60in(data60_test) = [];
angles_vert_deg_60in(data60_test) = [];
angles_horz_deg_adj_60in(data60_test) = [];
dist_pdoa_60in(data60_test) = [];
dist_vicon_60in(data60_test) = [];

% 60in averages
pdoa_horz_avg_60in = pointStats(:,1);
pdoa_vert_avg_60in = pointStats(:,3);
angles_horz_deg_adj_avg_60in = pointStats(:,8);
angles_vert_deg_avg_60in = pointStats(:,11);
dist_pdoa_avg_60in = (pointStats(:,2) + pointStats(:,4)) / 2;
dist_vicon_avg_60in = pointStats(:,14);

pdoa_horz_avg_60in(data60_avg_test) = [];
pdoa_vert_avg_60in(data60_avg_test) = [];
angles_horz_deg_adj_avg_60in(data60_avg_test) = [];
angles_vert_deg_avg_60in(data60_avg_test) = [];
dist_pdoa_avg_60in(data60_avg_test) = [];
dist_vicon_avg_60in(data60_avg_test) = [];

% filtering 60out data
pdoa_vert_60out = pdoa_vert; 
pdoa_horz_60out = pdoa_horz;
angles_vert_deg_60out = angles_vert_deg;
angles_horz_deg_adj_60out = angles_horz_deg_adj;
dist_pdoa_60out = dist_pdoa;

pdoa_vert_60out(~data60_test) = [];
pdoa_horz_60out(~data60_test) = [];
angles_vert_deg_60out(~data60_test) = [];
angles_horz_deg_adj_60out(~data60_test) = [];
dist_pdoa_60out(~data60_test) = [];

% filtering 75in data
pdoa_vert_75in = pdoa_vert; 
pdoa_horz_75in = pdoa_horz;
angles_vert_deg_75in = angles_vert_deg;
angles_horz_deg_adj_75in = angles_horz_deg_adj;
dist_pdoa_75in = dist_pdoa;
dist_vicon_75in = dist_vicon;

pdoa_vert_75in(data75_test) = [];
pdoa_horz_75in(data75_test) = [];
angles_vert_deg_75in(data75_test) = [];
angles_horz_deg_adj_75in(data75_test) = [];
dist_pdoa_75in(data75_test) = [];
dist_vicon_75in(data75_test) = [];

% 75in averages
pdoa_horz_avg_75in = pointStats(:,1);
pdoa_vert_avg_75in = pointStats(:,3);
angles_horz_deg_adj_avg_75in = pointStats(:,8);
angles_vert_deg_avg_75in = pointStats(:,11);
dist_pdoa_avg_75in = (pointStats(:,2) + pointStats(:,4)) / 2;

pdoa_horz_avg_75in(data75_avg_test) = [];
pdoa_vert_avg_75in(data75_avg_test) = [];
angles_horz_deg_adj_avg_75in(data75_avg_test) = [];
angles_vert_deg_avg_75in(data75_avg_test) = [];
dist_pdoa_avg_75in(data75_avg_test) = [];

% filtering 75out data
pdoa_vert_75out = pdoa_vert; 
pdoa_horz_75out = pdoa_horz;
angles_vert_deg_75out = angles_vert_deg;
angles_horz_deg_75out = angles_horz_deg_adj;
dist_pdoa_75out = dist_pdoa;

pdoa_vert_75out(~data75_test) = [];
pdoa_horz_75out(~data75_test) = [];
angles_vert_deg_75out(~data75_test) = [];
angles_horz_deg_75out(~data75_test) = [];
dist_pdoa_75out(~data75_test) = [];

% filtering 90in data
pdoa_vert_90in = pdoa_vert; 
pdoa_horz_90in = pdoa_horz;
angles_vert_deg_90in = angles_vert_deg;
angles_horz_deg_adj_90in = angles_horz_deg_adj;
dist_pdoa_90in = dist_pdoa;
dist_vicon_90in = dist_vicon;

pdoa_vert_90in(data90_test) = [];
pdoa_horz_90in(data90_test) = [];
angles_vert_deg_90in(data90_test) = [];
angles_horz_deg_adj_90in(data90_test) = [];
dist_pdoa_90in(data90_test) = [];
dist_vicon_90in(data90_test) = [];

% 90in averages
pdoa_horz_avg_90in = pointStats(:,1);
pdoa_vert_avg_90in = pointStats(:,3);
angles_horz_deg_adj_avg_90in = pointStats(:,8);
angles_vert_deg_avg_90in = pointStats(:,11);
dist_pdoa_avg_90in = (pointStats(:,2) + pointStats(:,4)) / 2;

pdoa_horz_avg_90in(data90_avg_test) = [];
pdoa_vert_avg_90in(data90_avg_test) = [];
angles_horz_deg_adj_avg_90in(data90_avg_test) = [];
angles_vert_deg_avg_90in(data90_avg_test) = [];
dist_pdoa_avg_90in(data90_avg_test) = [];

% playing with points

% average input
viconPoints = pointStats(:,5:7);
output = ANN_60_50n1_normal(pointStats(:,1:4)')';
% plot3(output(:,1),output(:,2),output(:,3),'.','MarkerSize',35);
% hold on
% plot3(pointStats(:,5),pointStats(:,6),pointStats(:,7),'.','MarkerSize',35);
% axis equal
% Error metric as Euclidean distance
errorMag = sqrt( (viconPoints(:,1) - output(:,1)).^2 + (viconPoints(:,2) - output(:,2)).^2 + (viconPoints(:,3) - output(:,3)).^2 ); 
interleave = reshape([viconPoints(:) output(:)]',2*size(viconPoints,1), []);

% raw point output
output_ANN60_raw = ANN_60_50n1_normal(input60')'; %equivalent to ANN_60_50n(mean(point001(:,1:4))); for all points
% average output
output_ANN60 = ANN_60_50n1_normal(pointStats(:,1:4)')'; %equivalent to ANN_60_50n(mean(point001(:,1:4))); for all points

output_60in = output_ANN60;
output_60out = output_ANN60;
viconPoints_60in = viconPoints;
viconPoints_60out = viconPoints; 

listing_60 = ones(length(output_ANN60),1);
% out of 60deg points
listing_60(29:30,:) = zeros();
listing_60(34:41,:) = zeros();
listing_60(42:69,:) = zeros();
listing_60(70:77,:) = zeros();
listing_60(81:82,:) = zeros();
listing_60(139:140,:) = zeros();
listing_60(144:151,:) = zeros();
listing_60(152:179,:) = zeros();
listing_60(180:187,:) = zeros();
listing_60(191:192,:) = zeros();

inFlag60 = listing_60(:) == 0;
outFlag60 = listing_60(:) ~= 0;

output_60in(inFlag60,:) = [];
output_60out(outFlag60,:) = [];
viconPoints_60in(inFlag60,:) = [];
viconPoints_60out(outFlag60,:) = [];

% interleave model output and ground truth in order to plot mapping lines between the two
interleave60 = reshape([viconPoints(:) output_ANN60(:)]',2*size(viconPoints,1), []);
interleave_60in = reshape([viconPoints_60in(:) output_60in(:)]',2*size(viconPoints_60in,1), []);
interleave_60out = reshape([viconPoints_60out(:) output_60out(:)]',2*size(viconPoints_60out,1), []);

% Error metric as Euclidean distance
errorMag_60 = sqrt( (viconPoints(:,1) - output_ANN60(:,1)).^2 + (viconPoints(:,2) - output_ANN60(:,2)).^2 + (viconPoints(:,3) - output_ANN60(:,3)).^2 );
errorMag_60in = errorMag_60;
errorMag_60out = errorMag_60;
errorMag_60in(inFlag60,:) = [];
errorMag_60out(outFlag60,:) = [];

% histogram(errorMag)
% histogram(errorMag_60out)
% hold on

% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% histogram(errorMag_60in)
% 
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'Error histogram for fit of averaged $\pm$60deg dataset using an ANN'},...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% % legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
% %         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
% filename = 'figs/Error_ANN_60';
% print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(filename,'-dpng','-opengl'); %print figure as eps for the infinite zoom

output_ANN75 = ANN_75_50n1_normal(pointStats(:,1:4)')'; %equivalent to ANN_60_50n(mean(point001(:,1:4))); for all points

output_75in = output_ANN75;
output_75out = output_ANN75;
viconPoints_75in = viconPoints;
viconPoints_75out = viconPoints; 

listing_75 = ones(length(output_ANN75),1);
% out of 75deg points
listing_75(36:41,:) = zeros();
listing_75(42:69,:) = zeros();
listing_75(70:75,:) = zeros();
listing_75(146:151,:) = zeros();
listing_75(152:179,:) = zeros();
listing_75(180:185,:) = zeros();


inFlag75 = listing_75(:) == 0;
outFlag75 = listing_75(:) ~= 0;

output_75in(inFlag75,:) = [];
output_75out(outFlag75,:) = [];
viconPoints_75in(inFlag75,:) = [];
viconPoints_75out(outFlag75,:) = [];

% interleave model output and ground truth in order to plot mapping lines between the two
interleave75 = reshape([viconPoints(:) output_ANN75(:)]',2*size(viconPoints,1), []);
interleave_75in = reshape([viconPoints_75in(:) output_75in(:)]',2*size(viconPoints_75in,1), []);
interleave_75out = reshape([viconPoints_75out(:) output_75out(:)]',2*size(viconPoints_75out,1), []);

% Error metric as Euclidean distance
errorMag_75 = sqrt( (viconPoints(:,1) - output_ANN75(:,1)).^2 + (viconPoints(:,2) - output_ANN75(:,2)).^2 + (viconPoints(:,3) - output_ANN75(:,3)).^2 );
errorMag_75in = errorMag_75;
errorMag_75out = errorMag_75;
errorMag_75in(inFlag75,:) = [];
errorMag_75out(outFlag75,:) = [];

% histogram(errorMag)
% histogram(errorMag_75out)
% hold on
% figure, histogram(errorMag_75in)

% Curve fitting models
[fitresult60_horz_planar, gof60_horz_planar] = fit_angles_horz_60in_planar(pdoa_horz_60in, pdoa_vert_60in, angles_horz_deg_adj_60in);
[fitresult60_vert_planar, gof60_vert_planar] = fit_angles_vert_60in_planar(pdoa_horz_60in, pdoa_vert_60in, angles_vert_deg_60in);
[fitresult75_horz_planar, gof75_horz_planar] = fit_angles_horz_75in_planar(pdoa_horz_75in, pdoa_vert_75in, angles_horz_deg_adj_75in);
[fitresult75_vert_planar, gof75_vert_planar] = fit_angles_vert_75in_planar(pdoa_horz_75in, pdoa_vert_75in, angles_vert_deg_75in);
[fitresult60_horz_surf3, gof60_horz_surf3] = fit_angles_horz_60in_surf3(pdoa_horz_60in, pdoa_vert_60in, angles_horz_deg_adj_60in); %Function to use for ROS implementation
[fitresult60_vert_surf3, gof60_vert_surf3] = fit_angles_vert_60in_surf3(pdoa_horz_60in, pdoa_vert_60in, angles_vert_deg_60in); %Function to use for ROS implementation
coeffHorz = coeffvalues(fitresult60_horz_surf3); %horz coefficients to use in ROS
coeffVert = coeffvalues(fitresult60_vert_surf3); %vert coefficients to use in ROS
meanHorz = mean(pdoa_horz_60in); %horz normalization values for ROS
stdHorz = std(pdoa_horz_60in); %horz normalization values for ROS
meanVert = mean(pdoa_vert_60in); %vert normalization values for ROS
stdVert = std(pdoa_vert_60in); %vert normalization values for ROS
[fitresult75_horz_surf3, gof75_horz_surf3] = fit_angles_horz_75in_surf3(pdoa_horz_75in, pdoa_vert_75in, angles_horz_deg_adj_75in);
[fitresult75_vert_surf3, gof75_vert_surf3] = fit_angles_vert_75in_surf3(pdoa_horz_75in, pdoa_vert_75in, angles_vert_deg_75in);
[fitHorz60_linear,gofHorz60_linear,outputHorz60_linear] = fit(pdoa_horz_60in/1000,angles_horz_deg_adj_60in,'poly1');
[fitHorz60_cubic,gofHorz60_cubic,outputHorz60_cubic] = fit(pdoa_horz_60in/1000,angles_horz_deg_adj_60in,'poly3');
[fitVert60_linear,gofVert60_linear,outputVert60_linear] = fit(pdoa_vert_60in/1000,angles_vert_deg_60in,'poly1');
[fitVert60_cubic,gofVert60_cubic,outputVert60_cubic] = fit(pdoa_vert_60in/1000,angles_vert_deg_60in,'poly3');
[fitHorz75_linear,gofHorz75_linear,outputHorz75_linear] = fit(pdoa_horz_75in/1000,angles_horz_deg_adj_75in,'poly1');
[fitHorz75_cubic,gofHorz75_cubic,outputHorz75_cubic] = fit(pdoa_horz_75in/1000,angles_horz_deg_adj_75in,'poly3');
[fitVert75_linear,gofVert75_linear,outputVert75_linear] = fit(pdoa_vert_75in/1000,angles_vert_deg_75in,'poly1');
[fitVert75_cubic,gofVert75_cubic,outputVert75_cubic] = fit(pdoa_vert_75in/1000,angles_vert_deg_75in,'poly3');


%% Figure properties
width = 8; %figure size in inches
height = 8;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

savePlots = 0; % set to non-zero value to save all plots on run

%% Raw distance vs ground truth distance
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

[fitDist75,gofDist75,outputDist75] = fit(dist_vicon_75in,dist_pdoa_75in/1000,'poly1');
plot(dist_vicon,dist_pdoa/1000,'.')
hold on
plot(fitDist75,dist_vicon_75in,dist_pdoa_75in/1000,'.')
axis equal
xlim([0 4])
ylim([0 4])
title('Raw PDoA distance averaged between both nodes vs ground truth VICON distance')
xlabel('ground truth VICON distance [m]')
ylabel('raw PDoA distance [m]')
legend('Data collected outside of +/-75deg horz','Data collected inside of +/-75deg horz','Linear fit of inside data')


%% Plot data collection points in 2d space
% close all
% %from +Z
% subplot(1,3,1)
% plot(pdoa_node_midl(1),pdoa_node_midl(2),'*','MarkerSize',10,'LineWidth',1)
% hold on
% plot(xVICON,yVICON,'.','MarkerSize',10)
% xlabel('x [m]')
% ylabel('y [m]')
% 
% %from +X
% subplot(1,3,2)
% plot(pdoa_node_midl(2),0,'*','MarkerSize',10,'LineWidth',1)
% hold on
% plot(yVICON,zVICON,'.','MarkerSize',10)
% xlabel('y [m]')
% ylabel('z [m]')
% 
% %from +Y
% subplot(1,3,3)
% plot(pdoa_node_midl(1),0,'*','MarkerSize',10,'LineWidth',1)
% hold on
% plot(-xVICON,zVICON,'.','MarkerSize',10)
% xlabel('x [m]')
% ylabel('z [m]')

%% Plot data collection points in 3d space
close all

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
% plot3(xVICON,yVICON,zVICON,'.','MarkerSize',20)
plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ground truth data point location relative to PDoA shield location in the VICON frame',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'PDoA Shield Center','VICON data points'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots
% 3D pdf!
ax = gca;
filename = 'figs/Vicon_DataPoints_Cartesian3D';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)
end

%% Spherical reconstruction to ensure calcs are correct
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% [x,y,z] = sph2cart(angles_horz_rad_adj,-angles_vert_rad,dist_vicon);
% plot3(x,y,z,'.','MarkerSize',10)

%% $AoA_{horz}$ [deg] polar plotting
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% polarplot(0,0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% polarplot(angles_horz_rad_adj,dist_horz_vicon,'.','MarkerSize',10) %changed perspective to match PDOA node angle 0 from Z+
% ax = gca;
% ax.ThetaLim = [-180 180];
% ax.ThetaZeroLocation = 'top';
% ax.ThetaDir = 'Counterclockwise';
% 
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'ground truth data point locations in the XY plane of the VICON frame','azimuth angle [deg] and distance [m] relative to PDoA shield location'},...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% legend({'PDoA Shield Center','VICON data points'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
% 
% if savePlots   
%     filename = 'figs/Vicon_DataPoints_Polar_Horz';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
% end

%% vert angle polar plotting
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% polarplot(0,0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% polarplot(angles_vert_rad,dist_vert_vicon,'.','MarkerSize',10) %changed perspective to match PDOA node angle 0 from X+
% ax = gca;
% ax.ThetaLim = [-180 180];
% ax.ThetaZeroLocation = 'right';
% ax.ThetaDir = 'Counterclockwise';
% ax.ThetaTick = [-180 -165 -150 -135 -120 -105 -90 -75 -60 -45 -30 -15 0 15 30 45 60 75 90 105 120 135 150 165 180];
% 
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'ground truth data point locations in the YZ plane of the VICON frame','elevation angle [deg] and distance [m] relative to PDoA shield location'},...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% legend({'PDoA Shield Center','VICON data points'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
% 
% if savePlots   
%     filename = 'figs/Vicon_DataPoints_Polar_Vert';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
% end

%% angle_deg_horz vs pdoa_horz full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_horz_deg_adj,pdoa_horz/1000,'.')
hold on
plot(angles_horz_deg_adj_avg,pdoa_horz_avg/1000,'.','MarkerSize',15)
xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Horizontal PDoA vs ground truth horizontal AoA, full dataset',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw data','Avg data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
if savePlots   
    filename = 'figs/PDOA_Horz_vs_Angle_Horz_Full';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% angle_deg_horz vs pdoa_horz full, avg
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot(angles_horz_deg_adj_avg,pdoa_horz_avg/1000,'.','MarkerSize',10)
% xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
% yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
% 
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Point averaged horizontal PDoA angle vs ground truth horizontal angle, full dataset',...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$AoA_{horz}$ [deg] [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'Avg data'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
% 
% if savePlots   
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
% end

%% pdoa_horz vs angle_deg_horz full, zoom 90
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_horz_deg_adj,pdoa_horz/1000,'.') % 30 deg spread at 0
hold on
plot(angles_horz_deg_adj_avg,pdoa_horz_avg/1000,'.','MarkerSize',15)

xticks([-90 -75 -60 -45 -30 -15 0 15 30 45 60 75 90])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
xlim([-100 100])
ylim([-200 200])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Raw horizontal PDoA vs ground truth horizontal AoA, in front of antenna',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw data','Avg data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')

if savePlots 
    filename = 'figs/PDOA_Horz_vs_Angle_Horz_90';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_horz vs angle_deg_horz, zoom 60
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

[fitHorz60,gofHorz60,outputHorz60] = fit(angles_horz_deg_adj_60in,pdoa_horz_60in/1000,'poly1');
plot(angles_horz_deg_adj_60in,pdoa_horz_60in/1000,'.')
hold on
plot(angles_horz_deg_adj_avg_60in,pdoa_horz_avg_60in/1000,'.','MarkerSize',15)
hold on
plot(fitHorz60)

xticks([-60 -45 -30 -15 0 15 30 45 60])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
xlim([-65 65])
ylim([-200 200])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Horizontal PDoA vs ground truth horizontal AoA, \pm60 degrees',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw data','Avg data','Linear fit'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots 
    filename = 'figs/PDOA_Horz_vs_Angle_Horz_60';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end


%% XY vs positive angle_deg_vert (shows vertical angle (on z-axis) that pdoa node would see if it were spun (about z-axis) to face each location upon data collection)
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(xVICON,yVICON,angles_vert_pos_deg,'.','MarkerSize',15)
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% 
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('Vertical Angle [deg]')
% 
% title('vert angle as a function XY position relative to the VICON frame')
% 
% if savePlots 
%     % 3D pdf!
%     ax = gca;
%     filename = 'figs/VertAngleCartesian3D';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     % fig2u3d(ax,filename)
% end

%% pdoa_vert vs angle_deg_vert full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg,pdoa_vert./1000,'.') 
hold on
plot(angles_vert_deg_avg,pdoa_vert_avg/1000,'.','MarkerSize',15)

xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Vertical PDoA vs ground truth vertical AoA, full dataset',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw data','Avg data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')

if savePlots     
    filename = 'figs/PDOA_Vert_vs_Angle_Vert_Full';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_vert, zoom
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg,pdoa_vert./1000,'.') 
hold on
plot(angles_vert_deg_avg,pdoa_vert_avg/1000,'.','MarkerSize',15)

xlim([-40 40])
ylim([-200 200])
xticks(-35:5:35)
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Vertical PDoA vs ground truth vertical AoA, in front of antenna',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw data','Avg data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')

if savePlots     
    filename = 'figs/PDOA_Vert_vs_Angle_Vert_Zoom';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_vert, zoom color coded
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg_90in,pdoa_vert_90in/1000,'.','Color',[0.9290 0.6940 0.1250]) 
hold on
plot(angles_vert_deg_75in,pdoa_vert_75in/1000,'.','Color',[0 0.4470 0.7410]) 
hold on
plot(angles_vert_deg_60in,pdoa_vert_60in/1000,'.','Color',[0.8500 0.3250 0.0980]) 
hold on
plot(angles_vert_deg_50in,pdoa_vert_50in/1000,'.','Color',[0.4940 0.1840 0.5560]) 
xlim([-40 40])
ylim([-180 180])
xticks([-35 -30 -25 -20 -15 -10 -5 0 5 10 15 20 25 30 35])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
xlabel('vert angle [deg]')
ylabel('$PDoA_{vert}$ [deg]')


set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth vertical AoA, in front of antenna','Segmented based on horizontal AoA to see mapping of out of range data back into range'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-75 $<$ $AoA_{horz}$ [deg] $<$ 75','-60 $<$ $AoA_{horz}$ [deg] $<$ 60','-50 $<$ $AoA_{horz}$ [deg] $<$ 50'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots     
    filename = 'figs/PDOA_Vert_vs_Angle_Vert_Zoom2Horz_raw';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_vert avg, zoom 
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg_avg_90in,pdoa_vert_avg_90in/1000,'.','MarkerSize',15,'Color',[0.9290 0.6940 0.1250]) 
hold on
plot(angles_vert_deg_avg_75in,pdoa_vert_avg_75in/1000,'.','MarkerSize',15,'Color',[0 0.4470 0.7410]) 
hold on
plot(angles_vert_deg_avg_60in,pdoa_vert_avg_60in/1000,'.','MarkerSize',15,'Color',[0.8500 0.3250 0.0980]) 
hold on
plot(angles_vert_deg_avg_50in,pdoa_vert_avg_50in/1000,'.','MarkerSize',15,'Color',[0.4940 0.1840 0.5560]) 
xlim([-40 40])
ylim([-180 180])
xticks([-35 -30 -25 -20 -15 -10 -5 0 5 10 15 20 25 30 35])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
xlabel('vert angle [deg]')
ylabel('$PDoA_{vert}$ [deg]')


set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth vertical AoA, in front of antenna','Segmented based on horizontal AoA to see mapping of out of range data back into range'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-75 $<$ $AoA_{horz}$ [deg] $<$ 75','-60 $<$ $AoA_{horz}$ [deg] $<$ 60','-50 $<$ $AoA_{horz}$ [deg] $<$ 50'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots     
    filename = 'figs/PDOA_Vert_vs_Angle_Vert_Zoom2Horz_avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_vert avg, zoom +/- 75 vs 60
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

[fitVert75,gofVert75,outputVert75] = fit(angles_vert_deg_avg_75in,pdoa_vert_avg_75in/1000,'poly1');
[fitVert60,gofVert60,outputVert60] = fit(angles_vert_deg_avg_60in,pdoa_vert_avg_60in/1000,'poly1');
plot(angles_vert_deg_avg_75in,pdoa_vert_avg_75in/1000,'.','MarkerSize',15) 
hold on
plot(angles_vert_deg_avg_60in,pdoa_vert_avg_60in/1000,'.','MarkerSize',15)
hold on
plot(fitVert75,'b')
hold on
plot(fitVert60)
 
xlim([-40 40])
ylim([-180 180])
xticks(-35:5:35)
yticks(-180:30:180)
xlabel('vert angle [deg]')
ylabel('$PDoA_{vert}$ [deg]')


set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth vertical AoA, in front of antenna','Segmented based on horizontal AoA to find best fit horizontal AoA region'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-75 $<$ $AoA_{horz}$ [deg] $<$ 75','-60 $<$ $AoA_{horz}$ [deg] $<$ 60','linear fit for $\pm$75 $AoA_{horz}$ [deg]','linear fit for $\pm$60 $AoA_{horz}$ [deg]'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots     
    filename = 'figs/PDOA_Vert_vs_Angle_Vert_Zoom2Horz_avg_zoom';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_vert, zoom 60
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg_60in,pdoa_vert_60in/1000,'.')
hold on
plot(angles_vert_deg_avg_60in,pdoa_vert_avg_60in/1000,'.','MarkerSize',15)
hold on
plot(fitVert60)

xlim([-65 65])
ylim([-200 200])
xticks(-60:15:60)
yticks([-180:30:180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Vertical PDoA vs ground truth vertical AoA, \pm60 degrees',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert} $[deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw data','Avg data','Linear fit'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots 
    filename = 'figs/PDOA_Vert_vs_Angle_Vert_60';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs pdoa_dist avg
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot(angles_vert_deg_avg,dist_pdoa_avg/1000,'.','MarkerSize',15) 
% hold on
% plot(angles_vert_deg_avg_75in,dist_pdoa_avg_75in/1000,'.','MarkerSize',15) 
% hold on
% plot(angles_vert_deg_avg_60in,dist_pdoa_avg_60in/1000,'.','MarkerSize',15) 
% hold on
% plot(angles_vert_deg_avg_50in,dist_pdoa_avg_50in/1000,'.','MarkerSize',15) 
% xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
% % yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
% xlabel('vert angle [deg]')
% ylabel('PDoA dist [m]')
% xlim([-45 45])
% % ylim([-180 180])
% title('Full vert angle vs raw dist pdoa dataset')
% legend('full','75','60','50')
% if savePlots 
%     filename = 'figs/AnglePDOACompareVertFull';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
% end

%% pdoa_horz vs angle_deg_vert, full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg,pdoa_horz/1000,'.','Color',[0.9290 0.6940 0.1250]) 
hold on
plot(angles_vert_deg_90in,pdoa_horz_90in/1000,'.','Color',[0 0.4470 0.7410])
hold on
plot(angles_vert_deg_60in,pdoa_horz_60in/1000,'.','Color',[0.8500 0.3250 0.0980]) 
xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Horizontal PDoA vs ground truth vertical AoA, full dataset','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-180 $<$ $AoA_{horz}$ [deg] $<$ 180','-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','East')

if savePlots    
    filename = 'figs/PDOA_Horz_vs_Angle_Vert_full';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_horz vs angle_deg_vert, full avg
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg_avg,pdoa_horz_avg/1000,'.','Color',[0.9290 0.6940 0.1250]) 
hold on
plot(angles_vert_deg_avg_90in,pdoa_horz_avg_90in/1000,'.','MarkerSize',15,'Color',[0 0.4470 0.7410])
hold on
plot(angles_vert_deg_avg_60in,pdoa_horz_avg_60in/1000,'.','MarkerSize',15,'Color',[0.8500 0.3250 0.0980]) 
xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Horizontal PDoA vs ground truth vertical AoA, averaged full dataset','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-180 $<$ $AoA_{horz}$ [deg] $<$ 180','-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    filename = 'figs/PDOA_Horz_vs_Angle_Vert_full_avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_horz vs angle_deg_vert, zoom
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg_90in,pdoa_horz_90in/1000,'.','Color',[0 0.4470 0.7410])
hold on
plot(angles_vert_deg_60in,pdoa_horz_60in/1000,'.','Color',[0.8500 0.3250 0.0980]) 
xlim([-40 40])
xticks([-35 -30 -25 -20 -15 -10 -5 0 5 10 15 20 25 30 35])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Horizontal PDoA vs ground truth vertical AoA, in front of antenna','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    filename = 'figs/PDOA_Horz_vs_Angle_Vert_zoom';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_horz vs angle_deg_vert, zoom avg
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_vert_deg_avg_90in,pdoa_horz_avg_90in/1000,'.','MarkerSize',15,'Color',[0 0.4470 0.7410])
hold on
plot(angles_vert_deg_avg_60in,pdoa_horz_avg_60in/1000,'.','MarkerSize',15,'Color',[0.8500 0.3250 0.0980]) 
xlim([-40 40])
xticks([-35 -30 -25 -20 -15 -10 -5 0 5 10 15 20 25 30 35])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Horizontal PDoA vs ground truth vertical AoA, averages in front of antenna','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    filename = 'figs/PDOA_Horz_vs_Angle_Vert_zoom_avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_horz, full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_horz_deg_adj,pdoa_vert/1000,'.','Color',[0.9290 0.6940 0.1250]) 
hold on
plot(angles_horz_deg_adj_90in,pdoa_vert_90in/1000,'.','Color',[0 0.4470 0.7410])
hold on
plot(angles_horz_deg_adj_60in,pdoa_vert_60in/1000,'.','Color',[0.8500 0.3250 0.0980])

xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth horizontal AoA, full dataset','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg] [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-180 $<$ $AoA_{horz}$ [deg] $<$ 180','-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
    
if savePlots
    filename = 'figs/PDOA_Vert_vs_Angle_Horz_full';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_horz, full avg
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_horz_deg_adj_avg,pdoa_vert_avg/1000,'.','MarkerSize',15,'Color',[0.9290 0.6940 0.1250]) 
hold on
plot(angles_horz_deg_adj_avg_90in,pdoa_vert_avg_90in/1000,'.','MarkerSize',15,'Color',[0 0.4470 0.7410])
hold on
plot(angles_horz_deg_adj_avg_60in,pdoa_vert_avg_60in/1000,'.','MarkerSize',15,'Color',[0.8500 0.3250 0.0980])

xticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth horizontal AoA, averaged full dataset','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg] [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-180 $<$ $AoA_{horz}$ [deg] $<$ 180','-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')
    
if savePlots    
    filename = 'figs/PDOA_Vert_vs_Angle_Horz_full_avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_horz, zoom
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_horz_deg_adj_90in,pdoa_vert_90in/1000,'.')
hold on
plot(angles_horz_deg_adj_60in,pdoa_vert_60in/1000,'.')
xlim([-80 80])
xticks([-75 -60 -45 -30 -15 0 15 30 45 60 75])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth horizontal AoA, full dataset','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg] [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    filename = 'figs/PDOA_Vert_vs_Angle_Horz_full_zoom';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% pdoa_vert vs angle_deg_horz, zoom avg
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot(angles_horz_deg_adj_avg_90in,pdoa_vert_avg_90in/1000,'.','MarkerSize',15,'Color',[0 0.4470 0.7410])
hold on
plot(angles_horz_deg_adj_avg_60in,pdoa_vert_avg_60in/1000,'.','MarkerSize',15,'Color',[0.8500 0.3250 0.0980])
xlim([-80 80])
ylim([-130 130])
xticks([-75 -60 -45 -30 -15 0 15 30 45 60 75])
yticks([-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA vs ground truth horizontal AoA, averages in front of antenna','Segmented based on horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$AoA_{horz}$ [deg] [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'-90 $<$ $AoA_{horz}$ [deg] $<$ 90','-60 $<$ $AoA_{horz}$ [deg] $<$ 60'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    filename = 'figs/PDOA_Vert_vs_Angle_Horz_full_avg_zoom';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Cool 3D plot of raw data FULL
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(pdoa_horz_60out./1000,pdoa_vert_60out./1000,dist_pdoa_60out./1000,'.','MarkerSize',10)
hold on
plot3(pdoa_horz_60in./1000,pdoa_vert_60in./1000,dist_pdoa_60in./1000,'.','MarkerSize',10)
hold on
plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Independent raw data outputs from PDoA shield',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'avg PDoA distance [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'full dataset','$\pm$60deg horz dataset','PDoA Shield Center'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')

if savePlots
    % 3D pdf!
    ax = gca;
    filename = 'figs/Raw_Data_3D_full';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end

%% Cool 3D plot of raw data ONLY WITHIN +/-60 deg horz
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% plot3(pdoa_horz_60in./1000,pdoa_vert_60in./1000,dist_pdoa_60in./1000,'.','MarkerSize',10)
% xlim([-180 180])
% ylim([-180 180])
% xticks([-180 -120 -60 0 60 120 180])
% yticks([-180 -120 -60 0 60 120 180])
% 
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Independent raw data outputs from PDoA shield within the range $\pm$60deg horizontal angle',...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'avg PDoA distance [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'PDoA Shield Center','$\pm$60deg horz dataset'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
% 
% if savePlots
%     % 3D pdf!
%     ax = gca;
%     filename = 'figs/Raw_Data_3D_60in';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
%     % fig2u3d(ax,filename)
% end


%% Cool 3D plot of raw data FULL, avg
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(pdoa_horz_60out./1000,pdoa_vert_60out./1000,dist_pdoa_60out./1000,'.','MarkerSize',10)
% hold on
% plot3(pdoa_horz_60in./1000,pdoa_vert_60in./1000,dist_pdoa_60in./1000,'.','MarkerSize',10)
% hold on
% plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)
% 
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Independent raw data outputs from PDoA shield',...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'avg PDoA distance [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'full dataset','$\pm$60deg horz dataset','PDoA Shield Center'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
% 
% if savePlots
%     % 3D pdf!
%     ax = gca;
%     filename = 'figs/Raw_Data_3D_full_avg';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
%     % fig2u3d(ax,filename)
% end

%% Cool 3D plot of raw data ONLY WITHIN +/-60 deg horz, avg
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)
hold on
plot3(pdoa_horz_avg_60in./1000,pdoa_vert_avg_60in./1000,dist_pdoa_avg_60in./1000,'.','MarkerSize',15)
xlim([-180 180])
ylim([-180 180])
xticks([-180 -120 -60 0 60 120 180])
yticks([-180 -120 -60 0 60 120 180])

set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Independent averaged raw data outputs from PDoA shield','within the range $\pm$60deg horizontal angle'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$PDoA_{horz}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$ [deg]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'avg PDoA distance [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'PDoA Shield Center','$\pm$60deg horz dataset'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')

if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/Raw_Data_3D_60in_avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end


%% pdoa_horz as a function of both angles
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% [fitSurf60,gofSurf60,outputSurf60] = fit([angles_horz_deg_adj_60in angles_vert_deg_60in],pdoa_vert_60in./1000,'poly11');
% % plot(fitVert60,angles_vert_deg_60in,pdoa_vert_60in./1000,'.') % 40 degree spread at 0
% plot3(angles_horz_deg_adj, angles_vert_deg,pdoa_horz/1000,'.')
% xlabel('$AoA_{horz}$ [deg] [deg]')
% ylabel('vert angle [deg]')
% zlabel('$PDoA_{horz}$ [deg]')


%% pdoa_vert as a function of both angles
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(angles_horz_deg_adj, angles_vert_deg,pdoa_vert/1000,'.')
% xlabel('$AoA_{horz}$ [deg] [deg]')
% ylabel('vert angle [deg]')
% zlabel('$PDoA_{vert}$ [deg]')

%% Polar plotting color coded (this crashes my computers) (run from cmd: "matlab -softwareopenglmesa" to plot maybe without crashing)
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% polarplot(data75plus(:,9),data75plus(:,10),'.','MarkerSize',15)
% hold on
% polarplot(data75(:,9),data75(:,10),'.','MarkerSize',15)
% hold on
% polarplot(data60(:,9),data60(:,10),'.','MarkerSize',15)
% hold on
% polarplot(data35(:,9),data35(:,10),'.','MarkerSize',15)
% hold on
% polarplot(data15(:,9),data15(:,10),'.','MarkerSize',15)
% hold on
% polarplot(data5(:,9),data5(:,10),'.','MarkerSize',15)
% ax = gca;
% ax.ThetaLim = [-180 180];
% ax.ThetaZeroLocation = 'top';
% ax.ThetaDir = 'Counterclockwise';
% legend('$AoA_{horz}$ [deg] > 75 deg','$AoA_{horz}$ [deg] < 75 deg','$AoA_{horz}$ [deg] < 65 deg','$AoA_{horz}$ [deg] < 35 deg','$AoA_{horz}$ [deg] < 15 deg','$AoA_{horz}$ [deg] < 5 deg','location','southoutside')
% title('Dataset segmenting based on $AoA_{horz}$ [deg] for ANN tests')
% 
% if savePlots
%     filename = 'figs/PolarDataSegmentation';
%     print(filename,'-depsc'); %print figure as eps for the infinite zoom
%     print(filename,'-dpng'); %print figure as png for quick inspection
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
% end

%% ANN size tests with in range data (+/-60 deg horz) 
close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% outputNN60_5n = ANN_60_5n(input60);
% plot3(outputNN60_5n(:,1),outputNN60_5n(:,2),outputNN60_5n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_10n = ANN_60_10n(input60);
% plot3(outputNN60_10n(:,1),outputNN60_10n(:,2),outputNN60_10n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% title('In range data mapping between ground truth position and ANN+/-60Horz10n fit of raw PDoA data')
% legend('ANN fit','ground truth VICON','PDoA shield','location','southoutside')
% % % 3D pdf!
% % ax = gca;
% % filename = 'figs/ANN10mapping60in';
% % print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% % print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
% % print(filename,'-dpng','-opengl'); %print figure as png
% % % fig2u3d(ax,filename)

% outputNN60_15n = ANN_60_15n(input60);
% plot3(outputNN60_15n(:,1),outputNN60_15n(:,2),outputNN60_15n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_20n = ANN_60_20n(input60);
% plot3(outputNN60_20n(:,1),outputNN60_20n(:,2),outputNN60_20n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_25n = ANN_60_25n(input60);
% plot3(outputNN60_25n(:,1),outputNN60_25n(:,2),outputNN60_25n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_30n = ANN_60_30n(input60);
% plot3(outputNN60_30n(:,1),outputNN60_30n(:,2),outputNN60_30n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_50n = ANN_60_5n_normal(input60')';
% plot3(outputNN60_50n(:,1),outputNN60_50n(:,2),outputNN60_50n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_50n1 = ANN_60_50n1(input60);
% plot3(outputNN60_50n1(:,1),outputNN60_50n1(:,2),outputNN60_50n1(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% axis equal
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% title('In range data mapping between ground truth position and ANN+/-60Horz50n fit of raw PDoA data')
% legend('ANN fit','ground truth VICON','PDoA shield','location','southoutside')
% % % 3D pdf!
% % ax = gca;
% % filename = 'figs/ANNmapping60in';
% % print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% % print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
% % print(filename,'-dpng','-opengl'); %print figure as png
% % % fig2u3d(ax,filename)


% outputNNfull_50n = ANN_full_50n(input_full);
% plot3(outputNNfull_50n(:,1),outputNNfull_50n(:,2),outputNNfull_50n(:,3),'.');
% hold on
% plot3(output_full(:,1),output_full(:,2),output_full(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% axis equal
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% title('Full data mapping between ground truth position and ANNfull50n fit of raw PDoA data')
% legend('ANN fit','ground truth VICON','PDoA shield','location','southoutside')
% % % 3D pdf!
% % ax = gca;
% % filename = 'figs/ANNmappingFULL50n';
% % print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% % print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
% % print(filename,'-dpng','-opengl'); %print figure as png
% % % fig2u3d(ax,filename)


% outputNNfull_10n = ANN_full_10n(input_full);
% plot3(outputNNfull_10n(:,1),outputNNfull_10n(:,2),outputNNfull_10n(:,3),'.');
% hold on
% plot3(output_full(:,1),output_full(:,2),output_full(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% axis equal
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% title('Full data mapping between ground truth position and ANNfull10n fit of raw PDoA data')
% legend('ANN fit','ground truth VICON','PDoA shield','location','southoutside')
% % % 3D pdf!
% % ax = gca;
% % filename = 'figs/ANNmappingFULL10n';
% % print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% % print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
% % print(filename,'-dpng','-opengl'); %print figure as png
% % % fig2u3d(ax,filename)

% outputNN60_100n = ANN_60_100n(input60);
% plot3(outputNN60_100n(:,1),outputNN60_100n(:,2),outputNN60_100n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal

% outputNN60_200n = ANN_60_100n(input60);
% plot3(outputNN60_200n(:,1),outputNN60_200n(:,2),outputNN60_200n(:,3),'.');
% hold on
% plot3(output60(:,1),output60(:,2),output60(:,3),'.','MarkerSize',35);
% axis equal
%% statistical fingerprints for out of range?

UWBstats = [pointStats(:,1:4),pointStats(:,8:11)]; %mean and std.dev of UWB data
UWB_data_out_range = [UWBstats(42:69,:) ; UWBstats(152:179,:)];
UWB_data_in_range = [UWBstats(1:41,:) ; UWBstats(70:151,:) ; UWBstats(180:220,:)];
UWB_data_out_range_avg = mean(UWB_data_out_range);
UWB_data_in_range_avg = mean(UWB_data_in_range);
UWB_data_out_range_range = range(UWB_data_out_range);
UWB_data_in_range_range = range(UWB_data_in_range);

% %PDoA std.dev plots
% plot(UWB_data_in_range(:,6))
% hold on
% plot(UWB_data_in_range(:,8))
% hold on
% plot(UWB_data_out_range(:,6),'r')
% hold on
% plot(UWB_data_out_range(:,8),'r')
% 
% %dist std.dev plots
% plot(UWB_data_in_range(:,5))
% hold on
% plot(UWB_data_in_range(:,7))
% hold on
% len_in = length(UWB_data_in_range);
% plot(1:len_in,ones(len_in)*UWB_data_in_range_avg(5))
% hold on
% plot(1:len_in,ones(len_in)*UWB_data_in_range_avg(7))
% hold on
% plot(UWB_data_out_range(:,5))
% hold on
% plot(UWB_data_out_range(:,7))
% hold on
% len_out = length(UWB_data_out_range);
% plot(1:len_out,ones(len_out)*UWB_data_out_range_avg(5))
% hold on
% plot(1:len_out,ones(len_out)*UWB_data_out_range_avg(7))
% legend('in-dist-horz-std.D','in-dist-vert-std.D','out-dist-horz-std.D','out-dist-vert-std.D')

%can use std.dev of dist from PDoA to determine if points are in range or
%not. Also std.dev of PDoA, but it is less telling



%% NN fit, full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot3(output(:,1),output(:,2),output(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave)-1
    plot3(interleave(k:k+1,1),interleave(k:k+1,2),interleave(k:k+1,3),'k.-','LineWidth',errorMag((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ANN positional fit of full raw dataset vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/ANN_Fit_Full_Avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end

%% NN fit, inside +/- 60 deg horz
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot3(output_60in(:,1),output_60in(:,2),output_60in(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave_60in)-1
    plot3(interleave_60in(k:k+1,1),interleave_60in(k:k+1,2),interleave_60in(k:k+1,3),'k.-','LineWidth',50*errorMag_60in((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'ANN positional fit of $\pm$60deg dataset vs ground truth locations','location averaged'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/ANN_Fit_60in_Avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end


%% NN fit, inside +/- 60 deg horz
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot3(output_ANN60_raw(:,1),output_ANN60_raw(:,2),output_ANN60_raw(:,3),'.');
hold on
plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ANN positional fit of $\pm$60deg dataset vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/ANN_Fit_60in';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end


%% NN fit, outside +/- 60 deg horz
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(output_60out(:,1),output_60out(:,2),output_60out(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints_60out(:,1),viconPoints_60out(:,2),viconPoints_60out(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave_60out)-1
    plot3(interleave_60out(k:k+1,1),interleave_60out(k:k+1,2),interleave_60out(k:k+1,3),'k.-','LineWidth',errorMag_60out((k+1)/2));
    hold on
end
axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'ANN positional fit of full dataset','without $\pm$60deg dataset vs ground truth locations'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/ANN_Fit_60out_Avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end

%% NN fit, +/- 75 deg horz
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(output_75in(:,1),output_75in(:,2),output_75in(:,3),'.','MarkerSize',35);
% hold on
% plot3(viconPoints_75in(:,1),viconPoints_75in(:,2),viconPoints_75in(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave_75in)-1
%     plot3(interleave_75in(k:k+1,1),interleave_75in(k:k+1,2),interleave_75in(k:k+1,3),'k.-','LineWidth',50*errorMag_75in((k+1)/2));
%     hold on
% end
% % axis equal
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% title('In range data mapping average between ground truth position and ANN+/-75Horz50n fit of raw PDoA data')
% legend('ANN fit','ground truth VICON','PDoA shield','location','southoutside')
% % % 3D pdf!
% % ax = gca;
% % filename = 'figs/ANNmapping75inAverage';
% % print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% % print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
% % print(filename,'-dpng','-opengl'); %print figure as png
% % % fig2u3d(ax,filename)

%% 60in planar surface
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit60_horz_planar = deg2rad( fitresult60_horz_planar(pdoa_horz_60in,pdoa_vert_60in) + 90 ); % transform back into vicon frame with a 90deg rotation
fit60_vert_planar = deg2rad( -fitresult60_vert_planar(pdoa_horz_60in,pdoa_vert_60in) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit60_planar,yFit60_planar,zFit60_planar] = sph2cart(fit60_horz_planar,fit60_vert_planar,dist_pdoa_60in/1000);
plot3(xFit60_planar,yFit60_planar,zFit60_planar,'.')
hold on
plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Surface fit of $\pm$60deg dataset using planar regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/Surface_Fit_60_Planar';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end

%% 60in planar surface, avg

fit_avg_60_horz_planar = deg2rad( fitresult60_horz_planar(pdoa_horz_avg_60in,pdoa_vert_avg_60in) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_avg_60_vert_planar = deg2rad( -fitresult60_vert_planar(pdoa_horz_avg_60in,pdoa_vert_avg_60in) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_avg_60_planar,yFit_avg_60_planar,zFit_avg_60_planar] = sph2cart(fit_avg_60_horz_planar,fit_avg_60_vert_planar,dist_pdoa_avg_60in/1000);
fit_avg_60_planar = [xFit_avg_60_planar,yFit_avg_60_planar,zFit_avg_60_planar];

% Error metric as Euclidean distance
errorMag_fit_avg_60_planar = sqrt( (viconPoints_60in(:,1) - xFit_avg_60_planar) .^2 + (viconPoints_60in(:,2) - yFit_avg_60_planar) .^2 + (viconPoints_60in(:,3) - zFit_avg_60_planar) .^2 );
interleave_fit_avg_60_planar = reshape([viconPoints_60in(:) fit_avg_60_planar(:)]',2*size(viconPoints_60in,1), []);

close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(xFit_avg_60_planar,yFit_avg_60_planar,zFit_avg_60_planar,'.','MarkerSize',35)
hold on
plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave_fit_avg_60_planar)-1
    plot3(interleave_fit_avg_60_planar(k:k+1,1),interleave_fit_avg_60_planar(k:k+1,2),interleave_fit_avg_60_planar(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_60_planar((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Surface fit of averaged $\pm$60deg dataset using planar regression vs ground truth locations','location averaged'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/Surface_Fit_60_Planar_avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end

% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% histogram(errorMag_fit_avg_60_planar)
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'Error histogram for surface fit of averaged $\pm$60deg dataset using planar regression'},...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% % legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
% %         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
% filename = 'figs/Error_Histogram_Surface1';
% print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(filename,'-dpng','-opengl'); %print figure as eps for the infinite zoom

%% 60in poly33 surface
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit60_horz_surf3 = deg2rad( fitresult60_horz_surf3(pdoa_horz_60in,pdoa_vert_60in) + 90 ); % transform back into vicon frame with a 90deg rotation
fit60_vert_surf3 = deg2rad( -fitresult60_vert_surf3(pdoa_horz_60in,pdoa_vert_60in) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit60_surf3,yFit60_surf3,zFit60_surf3] = sph2cart(fit60_horz_surf3,fit60_vert_surf3,dist_pdoa_60in/1000);
plot3(xFit60_surf3,yFit60_surf3,zFit60_surf3,'.')
hold on
plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Surface fit of $\pm$60deg dataset using poly33 regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'poly33 regression','VICON data points','PDoA Shield Center'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/Surface_Fit_60_Cubic';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end


%% 60in cubic surface, avg
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_avg_60_horz_surf3 = deg2rad( fitresult60_horz_surf3(pdoa_horz_avg_60in,pdoa_vert_avg_60in) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_avg_60_vert_surf3 = deg2rad( -fitresult60_vert_surf3(pdoa_horz_avg_60in,pdoa_vert_avg_60in) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_avg_60_surf3,yFit_avg_60_surf3,zFit_avg_60_surf3] = sph2cart(fit_avg_60_horz_surf3,fit_avg_60_vert_surf3,dist_pdoa_avg_60in/1000);
fit_avg_60_surf3 = [xFit_avg_60_surf3,yFit_avg_60_surf3,zFit_avg_60_surf3];

% Error metric as Euclidean distance
errorMag_fit_avg_60_surf3 = sqrt( (viconPoints_60in(:,1) - xFit_avg_60_surf3) .^2 + (viconPoints_60in(:,2) - yFit_avg_60_surf3) .^2 + (viconPoints_60in(:,3) - zFit_avg_60_surf3) .^2 );
interleave_fit_avg_60_surf3 = reshape([viconPoints_60in(:) fit_avg_60_surf3(:)]',2*size(viconPoints_60in,1), []);

close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(xFit_avg_60_surf3,yFit_avg_60_surf3,zFit_avg_60_surf3,'.','MarkerSize',35)
hold on
plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave_fit_avg_60_surf3)-1
    plot3(interleave_fit_avg_60_surf3(k:k+1,1),interleave_fit_avg_60_surf3(k:k+1,2),interleave_fit_avg_60_surf3(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_60_surf3((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Surface fit of averaged $\pm$60deg dataset using poly33 regression vs ground truth locations','location averaged'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
if savePlots    
    % 3D pdf!
    ax = gca;
    filename = 'figs/Surface_Fit_60_Cubic_Avg';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
    % fig2u3d(ax,filename)
end

% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% histogram(errorMag_fit_avg_60_surf3)
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'Error histogram for surface fit of averaged $\pm$60deg dataset using poly33 regression'},...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% % legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
% %         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
% filename = 'figs/Error_Histogram_Surface3';
% print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(filename,'-dpng','-opengl'); %print figure as eps for the infinite zoom


%%
% close all
% % 75in cubic surface
% fit75_horz_surf3 = deg2rad( fitresult75_horz_surf3(pdoa_horz_75in,pdoa_vert_75in) + 90 ); % transform back into vicon frame with a 90deg rotation
% fit75_vert_surf3 = deg2rad( -fitresult75_vert_surf3(pdoa_horz_75in,pdoa_vert_75in) ); % transform back to standard elevation angle by multiplying by -1
% 
% % [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
% [xFit75_surf3,yFit75_surf3,zFit75_surf3] = sph2cart(fit75_horz_surf3,fit75_vert_surf3,dist_pdoa_75in/1000);
% plot3(xFit75_surf3,yFit75_surf3,zFit75_surf3,'.')
% hold on
% plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
% 
% close all
% % 75in planar surface
% fit75_horz_planar = deg2rad( fitresult75_horz_planar(pdoa_horz_75in,pdoa_vert_75in) + 90 ); % transform back into vicon frame with a 90deg rotation
% fit75_vert_planar = deg2rad( -fitresult75_vert_planar(pdoa_horz_75in,pdoa_vert_75in) ); % transform back to standard elevation angle by multiplying by -1
% 
% % [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
% [xFit75_planar,yFit75_planar,zFit75_planar] = sph2cart(fit75_horz_planar,fit75_vert_planar,dist_pdoa_75in/1000);
% plot3(xFit75_planar,yFit75_planar,zFit75_planar,'.')
% hold on
% plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
% ylim([0 3.5])
% 
% close all
% % _avg_75in cubic surface
% fit_avg_75_horz_surf3 = deg2rad( fitresult75_horz_surf3(pdoa_horz_avg_75in,pdoa_vert_avg_75in) + 90 ); % transform back into vicon frame with a 90deg rotation
% fit_avg_75_vert_surf3 = deg2rad( -fitresult75_vert_surf3(pdoa_horz_avg_75in,pdoa_vert_avg_75in) ); % transform back to standard elevation angle by multiplying by -1
% 
% % [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
% [xFit_avg_75_surf3,yFit_avg_75_surf3,zFit_avg_75_surf3] = sph2cart(fit_avg_75_horz_surf3,fit_avg_75_vert_surf3,dist_pdoa_avg_75in/1000);
% fit_avg_75_surf3 = [xFit_avg_75_surf3,yFit_avg_75_surf3,zFit_avg_75_surf3];
% 
% % Error metric as Euclidean distance
% errorMag_fit_avg_75_surf3 = sqrt( (viconPoints_75in(:,1) - xFit_avg_75_surf3) .^2 + (viconPoints_75in(:,2) - yFit_avg_75_surf3) .^2 + (viconPoints_75in(:,3) - zFit_avg_75_surf3) .^2 );
% figure, histogram(errorMag_fit_avg_75_surf3)
% interleave_fit_avg_75_surf3 = reshape([viconPoints_75in(:) fit_avg_75_surf3(:)]',2*size(viconPoints_75in,1), []);
% 
% figure
% plot3(xFit_avg_75_surf3,yFit_avg_75_surf3,zFit_avg_75_surf3,'.','MarkerSize',35)
% hold on
% plot3(viconPoints_75in(:,1),viconPoints_75in(:,2),viconPoints_75in(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave_fit_avg_75_surf3)-1
%     plot3(interleave_fit_avg_75_surf3(k:k+1,1),interleave_fit_avg_75_surf3(k:k+1,2),interleave_fit_avg_75_surf3(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_75_surf3((k+1)/2));
%     hold on
% end

% close all
% % _avg_75in planar surface
% fit_avg_75_horz_planar = deg2rad( fitresult75_horz_planar(pdoa_horz_avg_75in,pdoa_vert_avg_75in) + 90 ); % transform back into vicon frame with a 90deg rotation
% fit_avg_75_vert_planar = deg2rad( -fitresult75_vert_planar(pdoa_horz_avg_75in,pdoa_vert_avg_75in) ); % transform back to standard elevation angle by multiplying by -1
% 
% % [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
% [xFit_avg_75_planar,yFit_avg_75_planar,zFit_avg_75_planar] = sph2cart(fit_avg_75_horz_planar,fit_avg_75_vert_planar,dist_pdoa_avg_75in/1000);
% fit_avg_75_planar = [xFit_avg_75_planar,yFit_avg_75_planar,zFit_avg_75_planar];
% 
% % Error metric as Euclidean distance
% errorMag_fit_avg_75_planar = sqrt( (viconPoints_75in(:,1) - xFit_avg_75_planar) .^2 + (viconPoints_75in(:,2) - yFit_avg_75_planar) .^2 + (viconPoints_75in(:,3) - zFit_avg_75_planar) .^2 );
% figure, histogram(errorMag_fit_avg_75_planar)
% interleave_fit_avg_75_planar = reshape([viconPoints_75in(:) fit_avg_75_planar(:)]',2*size(viconPoints_75in,1), []);
% 
% figure
% plot3(xFit_avg_75_planar,yFit_avg_75_planar,zFit_avg_75_planar,'.','MarkerSize',35)
% hold on
% plot3(viconPoints_75in(:,1),viconPoints_75in(:,2),viconPoints_75in(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave_fit_avg_75_planar)-1
%     plot3(interleave_fit_avg_75_planar(k:k+1,1),interleave_fit_avg_75_planar(k:k+1,2),interleave_fit_avg_75_planar(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_75_planar((k+1)/2));
%     hold on
% end

%% Naive Independence Assumption
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% fit60_horz_avg_cubic = fitHorz60_cubic(pdoa_horz_avg_60in/1000);
% plot(angles_horz_deg_adj_60in,pdoa_horz_60in/1000,'.')
% hold on
% plot(angles_horz_deg_adj_avg_60in,pdoa_horz_avg_60in/1000,'.','MarkerSize',15)
% hold on
% plot(fit60_horz_avg_cubic,pdoa_horz_avg_60in/1000,'.','MarkerSize',15)
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% fit60_vert_avg_cubic = fitVert60_cubic(pdoa_vert_avg_60in/1000);
% plot(angles_vert_deg_60in,pdoa_vert_60in/1000,'.')
% hold on
% plot(angles_vert_deg_avg_60in,pdoa_vert_avg_60in/1000,'.','MarkerSize',15)
% hold on
% plot(fit60_vert_avg_cubic,pdoa_vert_avg_60in/1000,'.','MarkerSize',15)
% 
% fit60_horz_avg_linear = deg2rad( fitHorz60_linear(pdoa_horz_avg_60in/1000) + 90 ); % transform back into vicon frame with a 90deg rotation
% fit60_vert_avg_linear = deg2rad( -fitVert60_linear(pdoa_vert_avg_60in/1000) ); % transform back to standard elevation angle by multiplying by -1
% 
% [xFit60_avg_linear,yFit60_avg_linear,zFit60_avg_linear] = sph2cart(fit60_horz_avg_linear,fit60_vert_avg_linear,dist_pdoa_avg_60in/1000);
% fit_avg_60_linear = [xFit60_avg_linear,yFit60_avg_linear,zFit60_avg_linear];
% 
% % Error metric as Euclidean distance
% errorMag_fit_avg_60_linear = sqrt( (viconPoints_60in(:,1) - xFit60_avg_linear) .^2 + (viconPoints_60in(:,2) - yFit60_avg_linear) .^2 + (viconPoints_60in(:,3) - zFit60_avg_linear) .^2 );
% interleave_fit_avg_60_linear = reshape([viconPoints_60in(:) fit_avg_60_linear(:)]',2*size(viconPoints_60in,1), []);
% 
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% plot3(xFit60_avg_linear,yFit60_avg_linear,zFit60_avg_linear,'.','MarkerSize',35)
% hold on
% plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave_fit_avg_60_linear)-1
%     plot3(interleave_fit_avg_60_linear(k:k+1,1),interleave_fit_avg_60_linear(k:k+1,2),interleave_fit_avg_60_linear(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_60_linear((k+1)/2));
%     hold on
% end
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Surface fit of averaged $\pm$60deg dataset using poly33 regression vs ground truth locations',...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'linear regression','VICON data points','PDoA Shield Center','Error'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
% 
% if savePlots    
%     % 3D pdf!
%     ax = gca;
%     filename = 'figs/Line_Fit_60_Linear_avg';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
%     % fig2u3d(ax,filename)
% end
% 
% 
% 
fit60_horz_avg_cubic = deg2rad( fitHorz60_cubic(pdoa_horz_avg_60in/1000) + 90 ); % transform back into vicon frame with a 90deg rotation
fit60_vert_avg_cubic = deg2rad( -fitVert60_cubic(pdoa_vert_avg_60in/1000) ); % transform back to standard elevation angle by multiplying by -1

[xFit60_avg_cubic,yFit60_avg_cubic,zFit60_avg_cubic] = sph2cart(fit60_horz_avg_cubic,fit60_vert_avg_cubic,dist_pdoa_avg_60in/1000);
fit_avg_60_cubic = [xFit60_avg_cubic,yFit60_avg_cubic,zFit60_avg_cubic];

% Error metric as Euclidean distance
errorMag_fit_avg_60_cubic = sqrt( (viconPoints_60in(:,1) - xFit60_avg_cubic) .^2 + (viconPoints_60in(:,2) - yFit60_avg_cubic) .^2 + (viconPoints_60in(:,3) - zFit60_avg_cubic) .^2 );
interleave_fit_avg_60_cubic = reshape([viconPoints_60in(:) fit_avg_60_cubic(:)]',2*size(viconPoints_60in,1), []);
% 
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% plot3(xFit60_avg_cubic,yFit60_avg_cubic,zFit60_avg_cubic,'.','MarkerSize',35)
% hold on
% plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave_fit_avg_60_cubic)-1
%     plot3(interleave_fit_avg_60_cubic(k:k+1,1),interleave_fit_avg_60_cubic(k:k+1,2),interleave_fit_avg_60_cubic(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_60_cubic((k+1)/2));
%     hold on
% end
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Surface fit of averaged $\pm$60deg dataset using poly33 regression vs ground truth locations',...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'cubic regression','VICON data points','PDoA Shield Center','Error'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
%     
% if savePlots    
%     % 3D pdf!
%     ax = gca;
%     filename = 'figs/Line_Fit_60_Cubic_avg';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
%     % fig2u3d(ax,filename)
% end
% 
% 
% 
% 
% fit60_horz_avg_ANN = deg2rad( ANN_horz_500n1_normal(pdoa_horz_avg_60in')' +90 ); % transform back into vicon frame with a 90deg rotation
% fit60_vert_avg_ANN = deg2rad( -ANN_vert_500n1_normal(pdoa_vert_avg_60in')' ); % transform back to standard elevation angle by multiplying by -1
% 
% [xFit60_avg_ANN,yFit60_avg_ANN,zFit60_avg_ANN] = sph2cart(fit60_horz_avg_ANN,fit60_vert_avg_ANN,dist_pdoa_avg_60in/1000);
% fit_avg_60_ANN = [xFit60_avg_ANN,yFit60_avg_ANN,zFit60_avg_ANN];
% 
% % Error metric as Euclidean distance
% errorMag_fit_avg_60_ANN = sqrt( (viconPoints_60in(:,1) - xFit60_avg_ANN) .^2 + (viconPoints_60in(:,2) - yFit60_avg_ANN) .^2 + (viconPoints_60in(:,3) - zFit60_avg_ANN) .^2 );
% interleave_fit_avg_60_ANN = reshape([viconPoints_60in(:) fit_avg_60_ANN(:)]',2*size(viconPoints_60in,1), []);
% 
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% plot3(xFit60_avg_ANN,yFit60_avg_ANN,zFit60_avg_ANN,'.','MarkerSize',35)
% hold on
% plot3(viconPoints_60in(:,1),viconPoints_60in(:,2),viconPoints_60in(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave_fit_avg_60_ANN)-1
%     plot3(interleave_fit_avg_60_ANN(k:k+1,1),interleave_fit_avg_60_ANN(k:k+1,2),interleave_fit_avg_60_ANN(k:k+1,3),'k.-','LineWidth',10*errorMag_fit_avg_60_ANN((k+1)/2));
%     hold on
% end
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Surface fit of averaged $\pm$60deg dataset using poly33 regression vs ground truth locations',...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'linear regression','VICON data points','PDoA Shield Center','Error'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
%     
% if savePlots    
%     % 3D pdf!
%     ax = gca;
%     filename = 'figs/Line_Fit_60_Linear_avg';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
%     % fig2u3d(ax,filename)
% end
% 

%% Histogram comparisons
% close all
% % figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% errorMag_fit_avg_60_linear_mean = mean(errorMag_fit_avg_60_linear);
% errorMag_fit_avg_60_cubic_mean = mean(errorMag_fit_avg_60_cubic);
% errorMag_fit_avg_60_twoD_mean = mean(errorMag_fit_avg_60_ANN);
% errorMag_fit_avg_60_planar_mean = mean(errorMag_fit_avg_60_planar);
% errorMag_fit_avg_60_surf3_mean = mean(errorMag_fit_avg_60_surf3);
% errorMag_60in_mean = mean(errorMag_60in);
% 
% edges = 0:0.01:0.7;
% 
% histogram(errorMag_fit_avg_60_linear,edges)
% hold on
% histogram(errorMag_fit_avg_60_cubic,edges)
% hold on
% histogram(errorMag_fit_avg_60_ANN,edges)
% hold on
% histogram(errorMag_fit_avg_60_planar,edges)
% hold on
% histogram(errorMag_fit_avg_60_surf3,edges)
% hold on
% histogram(errorMag_60in,edges)
% 
% % subplot(3,2,1)
% % histogram(errorMag_fit_avg_60_linear,edges)
% % subplot(3,2,2)
% % histogram(errorMag_fit_avg_60_cubic,edges)
% % subplot(3,2,3)
% % histogram(errorMag_fit_avg_60_ANN,edges)
% % subplot(3,2,4)
% % histogram(errorMag_fit_avg_60_planar,edges)
% % subplot(3,2,5)
% % histogram(errorMag_fit_avg_60_surf3,edges)
% % subplot(3,2,6)
% % histogram(errorMag_60in,edges)
% 
% 
% 
% set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title({'Error histogram comparison between four separate regression methods'},...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'linear regression',...
%         'cubic regression',...
%         'independent regression',...
%         'planar regression',...
%         'poly33 regression',...
%         'ANN regression'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
%     
% 
% dim = [.45 .6 .0 .0]; %annotation
% start_str = sprintf("Average Euclidean Errors:");
% linear_str = sprintf('linear regression = %0.1f cm', errorMag_fit_avg_60_linear_mean*100);
% cubic_str = sprintf('cubic regression = %0.1f cm', errorMag_fit_avg_60_cubic_mean*100);
% twoD_str = sprintf('2D independent regression = %0.1f cm', errorMag_fit_avg_60_twoD_mean*100);
% planar_str = sprintf('planar regression = %0.1f cm', errorMag_fit_avg_60_planar_mean*100);
% surf3_str = sprintf('poly33 regression = %0.1f cm', errorMag_fit_avg_60_surf3_mean*100);
% ANN_str = sprintf('ANN regression = %0.1f cm', errorMag_60in_mean*100);
% str = start_str + "\n" + linear_str + "\n" + cubic_str + "\n" + twoD_str + "\n" + planar_str + "\n" + surf3_str + "\n" + ANN_str;
% str = compose(str);
% str = splitlines(str);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
% 
% if savePlots
%     filename = 'figs/Error_Histogram_Comparison';
%     print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
%     print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
%     print(filename,'-dpng','-opengl'); %print figure as png
% end

%% Histogram comparisons (3 in thesis)
close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

errorMag_fit_avg_60_planar_mean = mean(errorMag_fit_avg_60_planar);
errorMag_fit_avg_60_surf3_mean = mean(errorMag_fit_avg_60_surf3);
errorMag_60in_mean = mean(errorMag_60in);

edges = 0:0.01:0.7;
histogram(errorMag_fit_avg_60_surf3,edges,'FaceColor',[0.4660 0.6740 0.1880])
hold on
histogram(errorMag_fit_avg_60_planar,edges,'FaceColor',[0.8500 0.3250 0.0980])
hold on
histogram(errorMag_60in,edges,'FaceColor',[0 0.4470 0.7410])

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Error histogram comparison between three regression methods'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend('poly33','Planar','ANN')  

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Mean Euclidean Error:");
planar_str = sprintf('planar regression = %0.1f cm', errorMag_fit_avg_60_planar_mean*100);
surf3_str = sprintf('poly33 regression = %0.1f cm', errorMag_fit_avg_60_surf3_mean*100);
ANN_str = sprintf('ANN regression = %0.1f cm', errorMag_60in_mean*100);
str = start_str + "\n" + surf3_str + "\n" + planar_str + "\n" + ANN_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

if savePlots
    filename = 'figs/Error_Histogram_Comparison_thesis';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Histogram comparisons (poly33 vs independent cubics)
close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

errorMag_fit_avg_60_twoD_mean = mean(errorMag_fit_avg_60_cubic);
errorMag_fit_avg_60_surf3_mean = mean(errorMag_fit_avg_60_surf3);

edges = 0:0.01:0.7;

histogram(errorMag_fit_avg_60_surf3,edges)
hold on
histogram(errorMag_fit_avg_60_cubic,edges)

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Error histogram comparison between two theoretical assumptions'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Two Cubic Fits','poly33 Surface Fit'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
    

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Mean Euclidean Error:");
twoD_str = sprintf('Two Cubic Fits = %0.1f cm', errorMag_fit_avg_60_twoD_mean*100);
surf_str = sprintf('poly33 Surface Fit = %0.1f cm', errorMag_fit_avg_60_surf3_mean*100);
str = start_str + "\n" + twoD_str + "\n" + surf_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

if savePlots
    filename = 'figs/Error_Histogram_Comparison_cubic_vs_surface';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Plot Surface Fits
close all
% AoA Horz - Planar
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(fitresult60_horz_planar,[pdoa_horz_60in, pdoa_vert_60in], angles_horz_deg_adj_60in);
set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Planar Surface Fit for Horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$PDoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$AoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw Data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
    
if savePlots
    filename = 'figs/Surface_Fit_Horz_Planar';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end
    
% AoA Vert - Planar
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(fitresult60_vert_planar,[pdoa_horz_60in, pdoa_vert_60in], angles_vert_deg_60in);

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Planar Surface Fit for Vertical AoA'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$PDoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$AoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw Data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')

if savePlots    
    filename = 'figs/Surface_Fit_Vert_Planar';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end




% AoA Horz - Poly33
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(fitresult60_horz_surf3,[pdoa_horz_60in, pdoa_vert_60in], angles_horz_deg_adj_60in); %Function to use for ROS implementation

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Poly33 Surface Fit for Horizontal AoA'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$PDoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$AoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw Data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')
    
if savePlots
    filename = 'figs/Surface_Fit_Horz_Poly33';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end


% AoA Vert - Poly33
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
plot(fitresult60_vert_surf3,[pdoa_horz_60in, pdoa_vert_60in], angles_vert_deg_60in); %Function to use for ROS implementation

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Poly33 Surface Fit for Vertical AoA'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$PDoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$PDoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$AoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Raw Data'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','NorthEast')

if savePlots
    filename = 'figs/Surface_Fit_Vert_Poly33';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Assumed data generalization (conquered by symmetry!)
% close all
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(zVICON_avg,yVICON_avg,xVICON_avg,'.','MarkerSize',20)
% hold on
% plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('ground truth data point location relative to PDoA shield location in the VICON frame',...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'PDoA Shield Center','VICON data points'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')


%% Elliptical cone assumption real data
close all
% https://math.stackexchange.com/questions/2206137/a-right-elliptical-cone-is-4m-high-and-has-an-elliptical-base-with-half-axes-len
% https://www.mathworks.com/matlabcentral/answers/349988-how-to-create-a-3d-cone
% https://mathworld.wolfram.com/EllipticCone.html
% A cone with elliptical cross section. The parametric equations for an elliptic cone of height h, semimajor axis a, and semiminor axis b are
% x	=	a( (h-u)/h )cos(v)	
% y	=	b( (h-u)/h )sin(v)	
% z	=	u,	
% where v in [0,2pi) and u in [0,h].

% tan(semimajor angle) = a/h =>  a = h*tan(semimajor angle)
% tan(semiminor angle) = b/h =>  b = h*tan(semiminor angle)

deg_SM = 60; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
deg_sm = 60; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
rad_SM = deg2rad(deg_SM); %semimajor angle in radians
rad_sm = deg2rad(deg_sm); %semiminor angle in radian
h = 3; %experimental range in meters
a = h*tan(rad_SM);
b = h*tan(rad_sm);

u = linspace(0,h) ;
v = linspace(0,2*pi) ;
[U,V] = meshgrid(u,v) ;
x = (a/h)*(h-U).*cos(V) ;
y = (b/h)*(h-U).*sin(V) ;
z = U - h; %subtract h to place vertex at origin

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)
hold on
mesh(x,-z,y) %switch y and z to point cone along the y-axis

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Assumed Valid Modelling Region',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'PDoA Shield Center','VICON data points','Assumed Valid Region'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
az = -150.2172;
el = 25.4011;
view([az el])

if savePlots    
    filename = 'figs/Cone_assumed';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Elliptical cone assumption range vs assumed data
% close all
% % https://math.stackexchange.com/questions/2206137/a-right-elliptical-cone-is-4m-high-and-has-an-elliptical-base-with-half-axes-len
% % https://www.mathworks.com/matlabcentral/answers/349988-how-to-create-a-3d-cone
% % https://mathworld.wolfram.com/EllipticCone.html
% % A cone with elliptical cross section. The parametric equations for an elliptic cone of height h, semimajor axis a, and semiminor axis b are
% % x	=	a( (h-u)/h )cos(v)	
% % y	=	b( (h-u)/h )sin(v)	
% % z	=	u,	
% % where v in [0,2pi) and u in [0,h].
% 
% % tan(semimajor angle) = a/h =>  a = h*tan(semimajor angle)
% % tan(semiminor angle) = b/h =>  b = h*tan(semiminor angle)
% 
% deg_SM = 85; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
% deg_sm = 60; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
% rad_SM = deg2rad(deg_SM); %semimajor angle in radians
% rad_sm = deg2rad(deg_sm); %semiminor angle in radian
% h = 3.1; %experimental range in meters
% a = h*tan(rad_SM);
% b = h*tan(rad_sm);
% a1 = a;
% b1 = b;
% a2 = b;
% b2 = a;
% 
% u = linspace(0,h) ;
% v = linspace(0,2*pi) ;
% [U1,V1] = meshgrid(u,v) ;
% [U2,V2] = meshgrid(u,v) ;
% x1 = (a1/h)*(h-U1).*cos(V1) ;
% y1 = (b1/h)*(h-U1).*sin(V1) ;
% x2 = (a2/h)*(h-U1).*cos(V1) ;
% y2 = (b2/h)*(h-U1).*sin(V1) ;
% z = U1 - h; %subtract h to place vertex at origin
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(zVICON_avg,yVICON_avg,xVICON_avg,'.','MarkerSize',20)
% hold on
% plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% mesh(x1,-z,y1) %switch y and z to point cone along the y-axis
% hold on
% mesh(x2,-z,y2) %switch y and z to point cone along the y-axis
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('ground truth data point location relative to PDoA shield location in the VICON frame',...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'PDoA Shield Center','VICON data points'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
%% Right cone assumption range vs assumed data
% close all
% % https://math.stackexchange.com/questions/2206137/a-right-elliptical-cone-is-4m-high-and-has-an-elliptical-base-with-half-axes-len
% % https://www.mathworks.com/matlabcentral/answers/349988-how-to-create-a-3d-cone
% % https://mathworld.wolfram.com/EllipticCone.html
% % A cone with elliptical cross section. The parametric equations for an elliptic cone of height h, semimajor axis a, and semiminor axis b are
% % x	=	a( (h-u)/h )cos(v)	
% % y	=	b( (h-u)/h )sin(v)	
% % z	=	u,	
% % where v in [0,2pi) and u in [0,h].
% 
% % tan(semimajor angle) = a/h =>  a = h*tan(semimajor angle)
% % tan(semiminor angle) = b/h =>  b = h*tan(semiminor angle)
% 
% deg_SM = 60; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
% deg_sm = 60; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
% rad_SM = deg2rad(deg_SM); %semimajor angle in radians
% rad_sm = deg2rad(deg_sm); %semiminor angle in radian
% h = 3.1; %experimental range in meters
% a = h*tan(rad_SM);
% b = h*tan(rad_sm);
% 
% u = linspace(0,h) ;
% v = linspace(0,2*pi) ;
% [U,V] = meshgrid(u,v) ;
% x = (a/h)*(h-U).*cos(V) ;
% y = (b/h)*(h-U).*sin(V) ;
% z = U - h; %subtract h to place vertex at origin
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% plot3(zVICON_avg,yVICON_avg,xVICON_avg,'.','MarkerSize',20)
% hold on
% plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)
% hold on
% plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% mesh(x,-z,y) %switch y and z to point cone along the y-axis
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('ground truth data point location relative to PDoA shield location in the VICON frame',...
%       'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'PDoA Shield Center','VICON data points'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')

%% Elliptical cone measured range vs real data (the angluar range that I verified experimentally to work is a function of distance)
close all
% https://math.stackexchange.com/questions/2206137/a-right-elliptical-cone-is-4m-high-and-has-an-elliptical-base-with-half-axes-len
% https://www.mathworks.com/matlabcentral/answers/349988-how-to-create-a-3d-cone
% https://mathworld.wolfram.com/EllipticCone.html
% A cone with elliptical cross section. The parametric equations for an elliptic cone of height h, semimajor axis a, and semiminor axis b are
% x	=	a( (h-u)/h )cos(v)	
% y	=	b( (h-u)/h )sin(v)	
% z	=	u,	
% where v in [0,2pi) and u in [0,h].

% tan(semimajor angle) = a/h =>  a = h*tan(semimajor angle)
% tan(semiminor angle) = b/h =>  b = h*tan(semiminor angle)

deg_SM = 30; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
deg_sm = 8; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
rad_SM = deg2rad(deg_SM); %semimajor angle in radians
rad_sm = deg2rad(deg_sm); %semiminor angle in radian
h = 3.1; %experimental range in meters
a = h*tan(rad_SM);
b = h*tan(rad_sm);

u = linspace(0,h) ;
v = linspace(0,2*pi) ;
[U,V] = meshgrid(u,v) ;
x = (a/h)*(h-U).*cos(V) ;
y = (b/h)*(h-U).*sin(V) ;
z = U - h; %subtract h to place vertex at origin

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)
hold on
mesh(x,-z,y) %switch y and z to point cone along the y-axis

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Fully Tested Valid Modelling Region',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'PDoA Shield Center','VICON data points','Tested Valid Region'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
az = -150.9729;
el = 28.9222;
view([az el])   
    
if savePlots       
    filename = 'figs/Cone_tested';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Elliptical cone comparison between assumed and measured
close all
% https://math.stackexchange.com/questions/2206137/a-right-elliptical-cone-is-4m-high-and-has-an-elliptical-base-with-half-axes-len
% https://www.mathworks.com/matlabcentral/answers/349988-how-to-create-a-3d-cone
% https://mathworld.wolfram.com/EllipticCone.html
% A cone with elliptical cross section. The parametric equations for an elliptic cone of height h, semimajor axis a, and semiminor axis b are
% x	=	a( (h-u)/h )cos(v)	
% y	=	b( (h-u)/h )sin(v)	
% z	=	u,	
% where v in [0,2pi) and u in [0,h].

% tan(semimajor angle) = a/h =>  a = h*tan(semimajor angle)
% tan(semiminor angle) = b/h =>  b = h*tan(semiminor angle)


deg_SM = 60; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
deg_sm = 60; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
rad_SM = deg2rad(deg_SM); %semimajor angle in radians
rad_sm = deg2rad(deg_sm); %semiminor angle in radian
h = 3; %experimental range in meters
a = h*tan(rad_SM);
b = h*tan(rad_sm);

u = linspace(0,h) ;
v = linspace(0,2*pi) ;
[U,V] = meshgrid(u,v) ;
x1 = (a/h)*(h-U).*cos(V) ;
y1 = (b/h)*(h-U).*sin(V) ;
z1 = U - h; %subtract h to place vertex at origin


deg_SM = 30; %semimajor angle in degrees by experiment: measured extreme range is +/- 30deg @ 3m
deg_sm = 8; %semiminor angle in degrees by experiment: measured extreme range is +/-  7deg @ 3m
rad_SM = deg2rad(deg_SM); %semimajor angle in radians
rad_sm = deg2rad(deg_sm); %semiminor angle in radian
h = 3.1; %experimental range in meters
a = h*tan(rad_SM);
b = h*tan(rad_sm);

u = linspace(0,h) ;
v = linspace(0,2*pi) ;
[U,V] = meshgrid(u,v) ;
x2 = (a/h)*(h-U).*cos(V) ;
y2 = (b/h)*(h-U).*sin(V) ;
z2 = U - h; %subtract h to place vertex at origin

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
plot3(xVICON_avg,yVICON_avg,zVICON_avg,'.','MarkerSize',20)
hold on
mesh(x1,-z1,y1) %switch y and z to point cone along the y-axis
hold on
mesh(x2,-z2,y2,'EdgeColor',[0 0.4470 0.7410]) %switch y and z to point cone along the y-axis

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Comparision between Assumed and Fully Tested Valid Modeling Regions',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'PDoA Shield Center','VICON data points','Assumed Valid Region','Tested Valid Region'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
az = -158.7058;
el = 16.2034;
view([az el])

if savePlots       
    filename = 'figs/Cone_compare';
    print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
    print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
    print(filename,'-dpng','-opengl'); %print figure as png
end

%% Neural Network Generation and Training
% close all
% % Solve an Input-Output Fitting problem with a Neural Network
% % Script generated by Neural Fitting app
% % Created 13-May-2020 18:38:00
% %
% % This script assumes these variables are defined:
% %
% %   input60 - input data.
% %   output60 - target data.
% 
% % Have to transpose my data to work with this NN format
% name = 'ANN_vert_500n1_normal';
% hiddenLayerSize = 500;
% x = pdoa_vert_60in';
% t = angles_vert_deg_60in';
% 
% % Choose a Training Function
% % For a list of all training functions type: help nntrain
% % 'trainlm' is usually fastest.
% % 'trainbr' takes longer but may be better for challenging problems.
% % 'trainscg' uses less memory. Suitable in low memory situations.
% trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.
% 
% % Create a Fitting Network
% 
% net = fitnet(hiddenLayerSize,trainFcn);
% 
% % Choose Input and Output Pre/Post-Processing Functions
% % For a list of all processing functions type: help nnprocess
% net.input.processFcns = {'removeconstantrows','mapminmax'};
% net.output.processFcns = {'removeconstantrows','mapminmax'};
% 
% % Setup Division of Data for Training, Validation, Testing
% % For a list of all data division functions type: help nndivision
% net.divideFcn = 'dividerand';  % Divide data randomly
% net.divideMode = 'sample';  % Divide up every sample
% net.divideParam.trainRatio = 70/100;
% net.divideParam.valRatio = 15/100;
% net.divideParam.testRatio = 15/100;
% 
% % Choose a Performance Function
% % For a list of all performance functions type: help nnperformance
% net.performFcn = 'mse';  % Mean Squared Error
% 
% % Custom performance functions
% % https://www.mathworks.com/matlabcentral/answers/64319-neural-network-how-to-use-a-custom-performance-function
% 
% % 'normalization' can be set to 'none' (the default); 'standard', which normalizes 
% % errors between -2 and 2, corresponding to normalizing outputs and targets 
% % between -1 and 1; and 'percent', which normalizes errors between -1 and 1. 
% % This feature is useful for networks with multi-element outputs. It ensures 
% % that the relative accuracy of output elements with differing target value 
% % ranges are treated as equally important, instead ofprioritizing the 
% % relative accuracy of the output element with the largest target value range.
% % *above is from MATLAB 'doc mse'
% % https://www.mathworks.com/matlabcentral/answers/431858-what-formula-is-used-when-setting-net-performparam-nor-malization-standard-in-combination-with-m
% % https://www.mathworks.com/matlabcentral/answers/348564-how-can-i-set-the-normalization-of-the-performance-parameter-in-training-a-neural-network
% net.performParam.normalization = 'standard';
% 
% % Choose Plot Functions
% % For a list of all plot functions type: help nnplot
% net.plotFcns = {'plotperform','plottrainstate','ploterrhist', ...
%     'plotregression', 'plotfit'};
% 
% % Train the Network
% [net,tr] = train(net,x,t);
% 
% % Test the Network
% y = net(x);
% e = gsubtract(t,y);
% performance = perform(net,t,y);
% 
% % Recalculate Training, Validation and Test Performance
% tr.trainTargets = t .* tr.trainMask{1};
% tr.valTargets = t .* tr.valMask{1};
% tr.testTargets = t .* tr.testMask{1};
% 
% tr.trainError = gsubtract(tr.trainTargets,y);
% tr.valError = gsubtract(tr.valTargets,y);
% tr.testError = gsubtract(tr.testTargets,y);
% 
% % View the Network
% % view(net)
% save(strcat('nets/',name,'_net.mat'),'net')
% save(strcat('nets/',name,'_tr.mat'),'tr')
% 
% % Plots
% % Uncomment these lines to enable various plots.
% figure, plotperform(tr)
% print(strcat('nets/',name,'_performance'),'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(strcat('nets/',name,'_performance'),'-dpng','-opengl'); %print figure as png for quick inspection
% 
% figure, plottrainstate(tr)
% print(strcat('nets/',name,'_trainState'),'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(strcat('nets/',name,'_trainState'),'-dpng','-opengl'); %print figure as png for quick inspection
% 
% figure, ploterrhist(tr.trainError,'Training',tr.valError,'Validation',tr.testError,'Test')
% print(strcat('nets/',name,'_errorHist'),'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(strcat('nets/',name,'_errorHist'),'-dpng','-opengl'); %print figure as png for quick inspection
% 
% figure, plotregression(tr.trainTargets,y,'Training',tr.valTargets,y,'Validation',tr.testTargets,y,'Test',t,y,'All')
% print(strcat('nets/',name,'_regression'),'-depsc','-painters'); %print figure as eps for the infinite zoom
% print(strcat('nets/',name,'_regression'),'-dpng','-opengl'); %print figure as png for quick inspection
% 
% % figure, plotfit(net,x,t)
% 
% % Deployment
% % Change the (false) values to (true) to enable the following code blocks.
% % See the help for each generation function for more information.
% if (true)
%     % Generate MATLAB function for neural network for application
%     % deployment in MATLAB scripts or with MATLAB Compiler and Builder
%     % tools, or simply to examine the calculations your trained neural
%     % network performs.
%     genFunction(net,name);
%     genFunction(net,strcat('nets/',name,'_compiler'));
% %     y = myNeuralNetworkFunction(x);
% end
% if (true)
%     % Generate a matrix-only MATLAB function for neural network code
%     % generation with MATLAB Coder tools.
%     genFunction(net,strcat('nets/',name,'_coder'),'MatrixOnly','yes');
%     y = myNeuralNetworkFunction(x);
% end
% % if (false)
% %     % Generate a Simulink diagram for simulation or deployment with.
% %     % Simulink Coder tools.
% %     gensim(net);
% % end
