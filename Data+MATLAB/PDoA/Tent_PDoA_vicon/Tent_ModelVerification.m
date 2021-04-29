% clear all
% close all
% 
% % Import rosbag data file
% Bag0 = rosbag('2021-03-05-20-37-20.bag'); %PDoA_shield location of horz antenna in vicon space
% Bag1 = rosbag('2021-03-05-21-14-47.bag'); %Total station at 90deg from vicon X+
% 
% % Topic extraction
% shieldLoc_data = select(Bag0,'Topic','/vicon/pdoa_shield/pose');
% Locator2vicon_data = select(Bag1,'Topic','/uwb/pdoaLocatorFiltered/pdoaLocator2vicon');
% vicon2Locator_data = select(Bag1,'Topic','/uwb/pdoaLocatorFiltered/vicon2pdoaLocator');
% pdoa_vicon_data = select(Bag1,'Topic','/uwb/pdoaVICON/dataFiltered');
% 
% % Message extraction (need to define custom ROS message for MATLAB: https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html)
% [ts1,cols1] = timeseries(shieldLoc_data,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% [ts2,cols2] = timeseries(vicon2Locator_data,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% [ts3,cols3] = timeseries(Locator2vicon_data,'Point.X','Point.Y','Point.Z');
% [ts4,cols4] = timeseries(pdoa_vicon_data,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% 
% %Populate MATLAB variables from timeseries data
% pdoa_shield_x = ts1.data(:,1);
% pdoa_shield_y = ts1.data(:,2);
% pdoa_shield_z = ts1.data(:,3);
% 
% vicon2Locator_x = ts2.data(:,1);
% vicon2Locator_y = ts2.data(:,2);
% vicon2Locator_z = ts2.data(:,3);
% Locator2vicon_x = ts3.data(:,1);
% Locator2vicon_y = ts3.data(:,2);
% Locator2vicon_z = ts3.data(:,3);
% 
% pdoa2vicon_distHorz = ts4.data(:,1);
% pdoa2vicon_pdoaHorz = ts4.data(:,2);
% pdoa2vicon_distVert = ts4.data(:,3);
% pdoa2vicon_pdoaVert = ts4.data(:,4);
% vicon2pdoa_x = ts4.data(:,5);
% vicon2pdoa_y = ts4.data(:,6);
% vicon2pdoa_z = ts4.data(:,7);
% 
% t = ts4.time;
% dist_horz_pdoa = ts4.data(:,1);
% pdoa_horz = ts4.data(:,2);
% dist_vert_pdoa = ts4.data(:,3);
% pdoa_vert = ts4.data(:,4);
% xVICON = ts4.data(:,5);
% yVICON = ts4.data(:,6);
% zVICON = ts4.data(:,7);
% 
% pdoa_shield_horz = [mean(pdoa_shield_x),mean(pdoa_shield_y),mean(pdoa_shield_z)];
% pdoa_node_ofst = [0,0,0.0255]; %Half the distance between each pdoa node center
% % ended up being just 0.5mm different from the ECR measurement
% pdoa_node_z = [0,0,pdoa_shield_horz(3) + pdoa_node_ofst(3)]; %location of the PDOA node in vicon arena, halfway between horz and vert
% 
% Locator2vicon = [Locator2vicon_x, Locator2vicon_y, Locator2vicon_z];
% vicon2Locator  = [vicon2Locator_x, vicon2Locator_y, vicon2Locator_z - pdoa_node_z(3)];
% 
% data_raw = [pdoa_horz,dist_horz_pdoa,pdoa_vert,dist_vert_pdoa,xVICON,yVICON,zVICON - pdoa_node_z(3)];

%% Data manipulation (first try)
% clear all
% close all
% currentFolder = pwd;
% load('Tent_ModelVerification_data.mat');
% 
% errorMag = sqrt( (Locator2vicon(:,1) - vicon2Locator(:,1)).^2 + (Locator2vicon(:,2) - vicon2Locator(:,2)).^2 + (Locator2vicon(:,3) - vicon2Locator(:,3)).^2 );
% errorMag_avg = mean(errorMag);
% errorMag_std = std(errorMag);
% 
% % Figure properties
% width = 8; %figure size in inches
% height = 8;
% x0 = 8; %position on screen
% y0 = 4;
% titleFontSize = 12;
% axisFontSize = 9;
% legendFontSize = 9;
% Font = 'Times';

% %% Error plots (first try)
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% plot(Locator2vicon - vicon2Locator);
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% plot(errorMag);
% 
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% edges1 = 0:0.1:1.5;
% histogram(errorMag)
% 
% dim = [.45 .6 .0 .0]; %annotation
% start_str = sprintf("Euclidean Error:");
% avg_str = sprintf('average = %0.1f cm', errorMag_avg*100);
% std_str = sprintf('std.dev = %0.1f cm', errorMag_std*100);
% str = start_str + "\n" + avg_str + "\n" + std_str;
% str = compose(str);
% str = splitlines(str);
% annotation('textbox',dim,'String',str,'FitBoxToText','on');
% 
% %% Plots (first try)
% 
% % plot3(Locator2vicon(:,1),Locator2vicon(:,2),Locator2vicon(:,3),'.')
% % hold on
% % plot3(vicon2Locator(:,1),vicon2Locator(:,2),vicon2Locator(:,3),'.')
% 
% close all
% figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
% 
% interleave = reshape([vicon2Locator(:) Locator2vicon(:)]',2*size(vicon2Locator,1), []);
% 
% plot3(Locator2vicon(:,1),Locator2vicon(:,2),Locator2vicon(:,3),'.','MarkerSize',35);
% hold on
% plot3(vicon2Locator(:,1),vicon2Locator(:,2),vicon2Locator(:,3),'.','MarkerSize',35);
% hold on
% plot3(pdoa_shield_horz(1),pdoa_shield_horz(2),0,'m*','MarkerSize',15,'LineWidth',1)
% hold on
% for k = 1:2:length(interleave)-1
%     plot3(interleave(k:k+1,1),interleave(k:k+1,2),interleave(k:k+1,3),'k.-','LineWidth',10*errorMag((k+1)/2));
%     hold on
% end
% 
% axis equal
% set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
%         'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
% title('Surface fit of averaged $\pm$60deg dataset using 3x3 polynomial regression vs ground truth locations',...
%       'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
% xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({'Locator data points','VICON data points','PDoA Shield Center','Error'},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')


%% Data manipulation
clear all
close all

currentFolder = pwd;
load('Tent_ModelVerification_data.mat');
load('z7_fitModels.mat');
run('z_tent_segmenting.m');
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

%% ANN fit, full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

output_ANN = ANN_60_50n1_normal(pointStats(:,1:4)')';

% Verified that rotating the resulting point clud does not make a better
% fit => wrong total station orientation is not the issue
% % % Coordinate transform
% % [regParams,output1_ANN,ErrorStats]= absor(output1_ANN',viconPoints1','doScale',0,'doTrans',0);
% % output1_ANN = output1_ANN';

interleave_ANN = reshape([viconPoints(:) output_ANN(:)]',2*size(viconPoints,1), []);
errorMag_ANN = sqrt( (viconPoints(:,1) - output_ANN(:,1)).^2 + (viconPoints(:,2) - output_ANN(:,2)).^2 + (viconPoints(:,3) - output_ANN(:,3)).^2 ); 

plot3(output_ANN(:,1),output_ANN(:,2),output_ANN(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints(:,1),viconPoints(:,2),viconPoints(:,3),'.','MarkerSize',35);
hold on
plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave_ANN)-1
    plot3(interleave_ANN(k:k+1,1),interleave_ANN(k:k+1,2),interleave_ANN(k:k+1,3),'k.-','LineWidth',1*errorMag_ANN((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Tent verification: ANN positional fit vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
% 3D pdf!
ax = gca;
filename = 'figs/ANN_tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


%% Planar Surface fit
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
plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav_planar)-1
    plot3(interleav_planar(k:k+1,1),interleav_planar(k:k+1,2),interleav_planar(k:k+1,3),'k.-','LineWidth',10*errorMag_planar((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Tent verification: Surface fit using planar regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs/Planar_tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


%% surf3 Surface fit
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
plot3(0,0,0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav_surf3)-1
    plot3(interleav_surf3(k:k+1,1),interleav_surf3(k:k+1,2),interleav_surf3(k:k+1,3),'k.-','LineWidth',10*errorMag_surf3((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('Tent verification: Surface fit using poly33 regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs/surf3_tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)

%% histogram
close all

edges = 0:0.05:1;
histogram(errorMag_surf3,edges,'FaceColor',[0.4660 0.6740 0.1880])
hold on
histogram(errorMag_planar,edges,'FaceColor',[0.8500 0.3250 0.0980])
hold on
histogram(errorMag_ANN,edges,'FaceColor',[0 0.4470 0.7410])
legend('3x3','Planar','ANN')

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Tent verification: Error histogram comparison'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend('poly33','Planar','ANN')  

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Mean Euclidean Error:");
ANN_str = sprintf('ANN regression = %0.1f cm', mean(errorMag_ANN)*100);
planar_str = sprintf('Planar regression = %0.1f cm', mean(errorMag_planar)*100);
surf3_str = sprintf('poly33 regression = %0.1f cm', mean(errorMag_surf3)*100);
str = start_str + "\n" + surf3_str + "\n" + planar_str + "\n" + ANN_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

filename = 'figs/histogram_tent';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png


