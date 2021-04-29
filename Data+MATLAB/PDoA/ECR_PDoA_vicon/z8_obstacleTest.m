% clear all
% close all
% 
% % Import rosbag data file
% Bag = rosbag('2020-06-26-12-53-26.bag'); %whiteboard between PDoA shield and tag
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
% 
% pdoa_node_midl = [0,0,0.8163]; %location of the PDOA node in vicon arena, halfway between horz and vert
% 
% data_raw = [pdoa_horz,dist_horz_pdoa,pdoa_vert,dist_vert_pdoa,xVICON,yVICON,zVICON-pdoa_node_midl(3)];

%% Data manipulation
clear all
close all

currentFolder = pwd;
load('z7_fitModels.mat');
load('z9_obstacleTest.mat');
run('z10_obstacle_segmenting.m');
dist_pdoa = (pointStats(:,2) + pointStats(:,4)) / 2;

viconPoints = pointStats(:,5:7);

% Figure properties
width = 8; %figure size in inches
height = 8;
x0 = 10; %position on screen
y0 = 7;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

%% ANN fit , full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

output_ANN = ANN_60_50n1_normal(pointStats(:,1:4)')';

% Verified that rotating the resulting point clud does not make a better
% fit => wrong total station orientation is not the issue
% % % Coordinate transform
% % [regParams,output_ANN,ErrorStats]= absor(output_ANN',viconPoints','doScale',0,'doTrans',0);
% % output_ANN = output_ANN';

interleave_ANN = reshape([viconPoints(:) output_ANN(:)]',2*size(viconPoints,1), []);
errorMag_ANN = sqrt( (viconPoints(:,1) - output_ANN(:,1)).^2 + (viconPoints(:,2) - output_ANN(:,2)).^2 + (viconPoints(:,3) - output_ANN(:,3)).^2 ); 

plot3(output_ANN(:,1),output_ANN(:,2),output_ANN(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave_ANN)-1
    plot3(interleave_ANN(k:k+1,1),interleave_ANN(k:k+1,2),interleave_ANN(k:k+1,3),'k.-','LineWidth',1*errorMag_ANN((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ANN positional fit of full raw dataset vs groundtruth locations',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
% 3D pdf!
ax = gca;
filename = 'figs/ANN_Obstacle';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
fig2u3d(ax,filename)


% Planar Surface fit 
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_planar = deg2rad( fitresult60_horz_planar(pointStats(:,1),pointStats(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_planar = deg2rad( -fitresult60_vert_planar(pointStats(:,1),pointStats(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_planar,yFit_planar,zFit_planar] = sph2cart(fit_horz_planar,fit_vert_planar,dist_pdoa/1000);
fit_planar = [xFit_planar,yFit_planar,zFit_planar];

errorMag_planar = sqrt( (viconPoints(:,1) - xFit_planar) .^2 + (viconPoints(:,2) - yFit_planar) .^2 + (viconPoints(:,3) - zFit_planar) .^2 );
interleav_planar = reshape([viconPoints(:) fit_planar(:)]',2*size(viconPoints,1), []);

plot3(xFit_planar,yFit_planar,zFit_planar,'.','MarkerSize',35)
hold on
plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav_planar)-1
    plot3(interleav_planar(k:k+1,1),interleav_planar(k:k+1,2),interleav_planar(k:k+1,3),'k.-','LineWidth',1*errorMag_planar((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Surface fit of $\pm$60deg dataset using planar regression vs groundtruth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs/Planar_Obstacle';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
fig2u3d(ax,filename)


% surf3 Surface fit 
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_surf3 = deg2rad( fitresult60_horz_surf3(pointStats(:,1),pointStats(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_surf3 = deg2rad( -fitresult60_vert_surf3(pointStats(:,1),pointStats(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_surf3,yFit_surf3,zFit_surf3] = sph2cart(fit_horz_surf3,fit_vert_surf3,dist_pdoa/1000);
fit_surf3 = [xFit_surf3,yFit_surf3,zFit_surf3];

errorMag_surf3 = sqrt( (viconPoints(:,1) - xFit_surf3) .^2 + (viconPoints(:,2) - yFit_surf3) .^2 + (viconPoints(:,3) - zFit_surf3) .^2 );
interleav_surf3 = reshape([viconPoints(:) fit_surf3(:)]',2*size(viconPoints,1), []);

plot3(xFit_surf3,yFit_surf3,zFit_surf3,'.','MarkerSize',35)
hold on
plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav_surf3)-1
    plot3(interleav_surf3(k:k+1,1),interleav_surf3(k:k+1,2),interleav_surf3(k:k+1,3),'k.-','LineWidth',1*errorMag_surf3((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Surface fit of $\pm$60deg dataset using planar regression vs groundtruth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs/surf3_obstacle';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
fig2u3d(ax,filename)

% histogram 
close all

edges = 0:0.05:1;
histogram(errorMag_ANN,edges)
hold on
histogram(errorMag_planar,edges)
hold on
histogram(errorMag_surf3,edges)
legend('ANN','Planar','surf3')

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Average Euclidean Errors:");
ANN_str = sprintf('ANN regression = %0.1f cm', mean(errorMag_ANN)*100);
planar_str = sprintf('planar regression = %0.1f cm', mean(errorMag_planar)*100);
surf3_str = sprintf('surf3 regression = %0.1f cm', mean(errorMag_surf3)*100);
str = start_str + "\n" + ANN_str + "\n" + planar_str + "\n" + surf3_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');
