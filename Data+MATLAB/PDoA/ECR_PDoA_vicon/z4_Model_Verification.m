% clear all
% close all
% 
% % Import rosbag data file
% Bag1 = rosbag('2020-05-21-09-53-56.bag'); %points in front of the PDoA shield, within +/-60 horizontal angle
% Bag2 = rosbag('2020-05-21-11-41-31.bag'); %rotated the PDoA shield 180deg to look on the other side of the vicon arena and moved the PDoA tag over there as well
% Bag3 = rosbag('2020-05-21-12-38-30.bag'); %PDoA shield rotated -90deg from initial experiment
% 
% % Topic extraction
% EXPdata1 = select(Bag1,'Topic','/uwb/pdoaVICON/dataFiltered');
% EXPdata2 = select(Bag2,'Topic','/uwb/pdoaVICON/dataFiltered');
% EXPdata3 = select(Bag3,'Topic','/uwb/pdoaVICON/dataFiltered');
% 
% % Message extraction (need to define custom ROS message for MATLAB: https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html)
% [ts1,cols1] = timeseries(EXPdata1,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% [ts2,cols2] = timeseries(EXPdata2,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% [ts3,cols3] = timeseries(EXPdata3,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% % This takes a relatively long time
% 
% %Populate MATLAB variables from timeseries data
% t1 = ts1.time;
% dist_horz_pdoa1 = ts1.data(:,1);
% pdoa_horz1 = ts1.data(:,2);
% dist_vert_pdoa1 = ts1.data(:,3);
% pdoa_vert1 = ts1.data(:,4);
% xVICON1 = ts1.data(:,5);
% yVICON1 = ts1.data(:,6);
% zVICON1 = ts1.data(:,7);
% 
% t2 = ts2.time;
% dist_horz_pdoa2 = ts2.data(:,1);
% pdoa_horz2 = ts2.data(:,2);
% dist_vert_pdoa2 = ts2.data(:,3);
% pdoa_vert2 = ts2.data(:,4);
% xVICON2 = ts2.data(:,5);
% yVICON2 = ts2.data(:,6);
% zVICON2 = ts2.data(:,7);
% 
% t3 = ts3.time;
% dist_horz_pdoa3 = ts3.data(:,1);
% pdoa_horz3 = ts3.data(:,2);
% dist_vert_pdoa3 = ts3.data(:,3);
% pdoa_vert3 = ts3.data(:,4);
% xVICON3 = ts3.data(:,5);
% yVICON3 = ts3.data(:,6);
% zVICON3 = ts3.data(:,7);
% 
% pdoa_node_midl = [0,0,0.8163]; %location of the PDOA node in vicon arena, halfway between horz and vert
% 
% data_raw1 = [pdoa_horz1,dist_horz_pdoa1,pdoa_vert1,dist_vert_pdoa1,xVICON1,yVICON1,zVICON1-pdoa_node_midl(3)];
% data_raw2 = [pdoa_horz2,dist_horz_pdoa2,pdoa_vert2,dist_vert_pdoa2,xVICON2,yVICON2,zVICON2-pdoa_node_midl(3)];
% data_raw3 = [pdoa_horz3,dist_horz_pdoa3,pdoa_vert3,dist_vert_pdoa3,xVICON3,yVICON3,zVICON3-pdoa_node_midl(3)];

%% Data manipulation
clear all
close all

currentFolder = pwd;
load('z5_verification_data.mat');
load('z7_fitModels.mat');
run('z6_verification_segmenting.m');
dist_pdoa1 = (pointStats1(:,2) + pointStats1(:,4)) / 2;
dist_pdoa2 = (pointStats2(:,2) + pointStats2(:,4)) / 2;
dist_pdoa3 = (pointStats3(:,2) + pointStats3(:,4)) / 2;

viconPoints1 = pointStats1(:,5:7);
viconPoints2 = pointStats2(:,5:7);
viconPoints3 = pointStats3(:,5:7);

% Figure properties
width = 8; %figure size in inches
height = 8;
x0 = 10; %position on screen
y0 = 7;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

%% ANN fit 1, full
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

output1_ANN = ANN_60_50n1_normal(pointStats1(:,1:4)')';

% Verified that rotating the resulting point clud does not make a better
% fit => wrong total station orientation is not the issue
% % % Coordinate transform
% % [regParams,output1_ANN,ErrorStats]= absor(output1_ANN',viconPoints1','doScale',0,'doTrans',0);
% % output1_ANN = output1_ANN';

interleave1_ANN = reshape([viconPoints1(:) output1_ANN(:)]',2*size(viconPoints1,1), []);
errorMag1_ANN = sqrt( (viconPoints1(:,1) - output1_ANN(:,1)).^2 + (viconPoints1(:,2) - output1_ANN(:,2)).^2 + (viconPoints1(:,3) - output1_ANN(:,3)).^2 ); 

plot3(output1_ANN(:,1),output1_ANN(:,2),output1_ANN(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints1(:,1),viconPoints1(:,2),viconPoints1(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave1_ANN)-1
    plot3(interleave1_ANN(k:k+1,1),interleave1_ANN(k:k+1,2),interleave1_ANN(k:k+1,3),'k.-','LineWidth',10*errorMag1_ANN((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 1: ANN positional fit vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/ANN_Verification1';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


% Planar Surface fit 1
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_planar1 = deg2rad( fitresult60_horz_planar(pointStats1(:,1),pointStats1(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_planar1 = deg2rad( -fitresult60_vert_planar(pointStats1(:,1),pointStats1(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_planar1,yFit_planar1,zFit_planar1] = sph2cart(fit_horz_planar1,fit_vert_planar1,dist_pdoa1/1000);
fit_planar1 = [xFit_planar1,yFit_planar1,zFit_planar1];

errorMag1_planar = sqrt( (viconPoints1(:,1) - xFit_planar1) .^2 + (viconPoints1(:,2) - yFit_planar1) .^2 + (viconPoints1(:,3) - zFit_planar1) .^2 );
interleav1_planar = reshape([viconPoints1(:) fit_planar1(:)]',2*size(viconPoints1,1), []);

plot3(xFit_planar1,yFit_planar1,zFit_planar1,'.','MarkerSize',35)
hold on
plot3(viconPoints1(:,1),viconPoints1(:,2),viconPoints1(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav1_planar)-1
    plot3(interleav1_planar(k:k+1,1),interleav1_planar(k:k+1,2),interleav1_planar(k:k+1,3),'k.-','LineWidth',10*errorMag1_planar((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 1: Surface fit using planar regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/Planar_Verification1';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


% surf3 Surface fit 1
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_surf31 = deg2rad( fitresult60_horz_surf3(pointStats1(:,1),pointStats1(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_surf31 = deg2rad( -fitresult60_vert_surf3(pointStats1(:,1),pointStats1(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_surf31,yFit_surf31,zFit_surf31] = sph2cart(fit_horz_surf31,fit_vert_surf31,dist_pdoa1/1000);
fit_surf31 = [xFit_surf31,yFit_surf31,zFit_surf31];

errorMag1_surf3 = sqrt( (viconPoints1(:,1) - xFit_surf31) .^2 + (viconPoints1(:,2) - yFit_surf31) .^2 + (viconPoints1(:,3) - zFit_surf31) .^2 );
interleav1_surf3 = reshape([viconPoints1(:) fit_surf31(:)]',2*size(viconPoints1,1), []);

plot3(xFit_surf31,yFit_surf31,zFit_surf31,'.','MarkerSize',35)
hold on
plot3(viconPoints1(:,1),viconPoints1(:,2),viconPoints1(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav1_surf3)-1
    plot3(interleav1_surf3(k:k+1,1),interleav1_surf3(k:k+1,2),interleav1_surf3(k:k+1,3),'k.-','LineWidth',10*errorMag1_surf3((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 1: Surface fit using poly33 regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/surf3_Verification1';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)

% histogram 1
close all

edges = 0:0.05:1;
histogram(errorMag1_surf3,edges,'FaceColor',[0.4660 0.6740 0.1880])
hold on
histogram(errorMag1_planar,edges,'FaceColor',[0.8500 0.3250 0.0980])
hold on
histogram(errorMag1_ANN,edges,'FaceColor',[0 0.4470 0.7410])
legend('poly33','Planar','ANN')

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'ECR Verification 1: Error histogram comparison'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend('poly33','Planar','ANN')  

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Mean Euclidean Error:");
ANN_str = sprintf('ANN regression = %0.1f cm', mean(errorMag1_ANN)*100);
planar_str = sprintf('planar regression = %0.1f cm', mean(errorMag1_planar)*100);
surf3_str = sprintf('poly33 regression = %0.1f cm', mean(errorMag1_surf3)*100);
str = start_str + "\n" + surf3_str + "\n" + planar_str + "\n" + ANN_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

filename = 'figs_verify/histogram1';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png

%% ANN fit 2
close all

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

theta2 = pi; %rotate by 180deg in z
R2 = [cos(theta2) -sin(theta2) 0; ...
     sin(theta2)  cos(theta2) 0; ...
              0           0  1];

output2_ANN = (R2*ANN_60_50n1_normal(pointStats2(:,1:4)'))';
interleave2_ANN = reshape([viconPoints2(:) output2_ANN(:)]',2*size(viconPoints2,1), []);
errorMag2_ANN = sqrt( (viconPoints2(:,1) - output2_ANN(:,1)).^2 + (viconPoints2(:,2) - output2_ANN(:,2)).^2 + (viconPoints2(:,3) - output2_ANN(:,3)).^2 ); 

plot3(output2_ANN(:,1),output2_ANN(:,2),output2_ANN(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints2(:,1),viconPoints2(:,2),viconPoints2(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave2_ANN)-1
    plot3(interleave2_ANN(k:k+1,1),interleave2_ANN(k:k+1,2),interleave2_ANN(k:k+1,3),'k.-','LineWidth',10*errorMag2_ANN((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 2: ANN positional fit vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/ANN_Verification2';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


% Planar Surface fit 2
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_planar2 = deg2rad( fitresult60_horz_planar(pointStats2(:,1),pointStats2(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_planar2 = deg2rad( -fitresult60_vert_planar(pointStats2(:,1),pointStats2(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_planar2,yFit_planar2,zFit_planar2] = sph2cart(fit_horz_planar2,fit_vert_planar2,dist_pdoa2/1000);
fit_planar2 = (R2*[xFit_planar2,yFit_planar2,zFit_planar2]')';

errorMag2_planar = sqrt( (viconPoints2(:,1) - fit_planar2(:,1)) .^2 + (viconPoints2(:,2) - fit_planar2(:,2)) .^2 + (viconPoints2(:,3) - fit_planar2(:,3)) .^2 );
interleav2_planar = reshape([viconPoints2(:) fit_planar2(:)]',2*size(viconPoints2,1), []);

plot3(fit_planar2(:,1),fit_planar2(:,2),fit_planar2(:,3),'.','MarkerSize',35)
hold on
plot3(viconPoints2(:,1),viconPoints2(:,2),viconPoints2(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav2_planar)-1
    plot3(interleav2_planar(k:k+1,1),interleav2_planar(k:k+1,2),interleav2_planar(k:k+1,3),'k.-','LineWidth',10*errorMag2_planar((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 2: Surface fit using planar regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/Planar_Verification2';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


% surf3 Surface fit 2
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_surf32 = deg2rad( fitresult60_horz_surf3(pointStats2(:,1),pointStats2(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_surf32 = deg2rad( -fitresult60_vert_surf3(pointStats2(:,1),pointStats2(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_surf32,yFit_surf32,zFit_surf32] = sph2cart(fit_horz_surf32,fit_vert_surf32,dist_pdoa2/1000);
fit_surf32 = (R2*[xFit_surf32,yFit_surf32,zFit_surf32]')';

errorMag2_surf3 = sqrt( (viconPoints2(:,1) - fit_surf32(:,1)) .^2 + (viconPoints2(:,2) - fit_surf32(:,2)) .^2 + (viconPoints2(:,3) - fit_surf32(:,3)) .^2 );
interleav2_surf3 = reshape([viconPoints2(:) fit_surf32(:)]',2*size(viconPoints2,1), []);

plot3(fit_surf32(:,1),fit_surf32(:,2),fit_surf32(:,3),'.','MarkerSize',35)
hold on
plot3(viconPoints2(:,1),viconPoints2(:,2),viconPoints2(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav2_surf3)-1
    plot3(interleav2_surf3(k:k+1,1),interleav2_surf3(k:k+1,2),interleav2_surf3(k:k+1,3),'k.-','LineWidth',10*errorMag2_surf3((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 2: Surface fit using poly33 regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/surf3_Verification2';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)

% histogram 2
close all

edges = 0:0.05:1;
histogram(errorMag2_surf3,edges,'FaceColor',[0.4660 0.6740 0.1880])
hold on
histogram(errorMag2_planar,edges,'FaceColor',[0.8500 0.3250 0.0980])
hold on
histogram(errorMag2_ANN,edges,'FaceColor',[0 0.4470 0.7410])
legend('poly33','Planar','ANN')

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'ECR Verification 2: Error histogram comparison'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend('poly33','Planar','ANN')  

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Mean Euclidean Error:");
ANN_str = sprintf('ANN regression = %0.1f cm', mean(errorMag2_ANN)*100);
planar_str = sprintf('planar regression = %0.1f cm', mean(errorMag2_planar)*100);
surf3_str = sprintf('poly33 regression = %0.1f cm', mean(errorMag2_surf3)*100);
str = start_str + "\n" + surf3_str + "\n" + planar_str + "\n" + ANN_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

filename = 'figs_verify/histogram2';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png

%% ANN fit 3
close all

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

theta3 = -pi/2; %rotate by 90deg in z
R3 = [cos(theta3) -sin(theta3) 0; ...
      sin(theta3)  cos(theta3) 0; ...
                0           0  1];

output3_ANN = (R3*ANN_60_50n1_normal(pointStats3(:,1:4)'))';
interleave3_ANN = reshape([viconPoints3(:) output3_ANN(:)]',2*size(viconPoints3,1), []);
errorMag3_ANN = sqrt( (viconPoints3(:,1) - output3_ANN(:,1)).^2 + (viconPoints3(:,2) - output3_ANN(:,2)).^2 + (viconPoints3(:,3) - output3_ANN(:,3)).^2 ); 

plot3(output3_ANN(:,1),output3_ANN(:,2),output3_ANN(:,3),'.','MarkerSize',35);
hold on
plot3(viconPoints3(:,1),viconPoints3(:,2),viconPoints3(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleave3_ANN)-1
    plot3(interleave3_ANN(k:k+1,1),interleave3_ANN(k:k+1,2),interleave3_ANN(k:k+1,3),'k.-','LineWidth',10*errorMag3_ANN((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 3: ANN positional fit vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'ANN fit','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthOutside')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/ANN_Verification3';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


% Planar Surface fit 3
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_planar3 = deg2rad( fitresult60_horz_planar(pointStats3(:,1),pointStats3(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_planar3 = deg2rad( -fitresult60_vert_planar(pointStats3(:,1),pointStats3(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_planar3,yFit_planar3,zFit_planar3] = sph2cart(fit_horz_planar3,fit_vert_planar3,dist_pdoa3/1000);
fit_planar3 = (R3*[xFit_planar3,yFit_planar3,zFit_planar3]')';

errorMag3_planar = sqrt( (viconPoints3(:,1) - fit_planar3(:,1)) .^2 + (viconPoints3(:,2) - fit_planar3(:,2)) .^2 + (viconPoints3(:,3) - fit_planar3(:,3)) .^2 );
interleav3_planar = reshape([viconPoints3(:) fit_planar3(:)]',2*size(viconPoints3,1), []);

plot3(fit_planar3(:,1),fit_planar3(:,2),fit_planar3(:,3),'.','MarkerSize',35)
hold on
plot3(viconPoints3(:,1),viconPoints3(:,2),viconPoints3(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav3_planar)-1
    plot3(interleav3_planar(k:k+1,1),interleav3_planar(k:k+1,2),interleav3_planar(k:k+1,3),'k.-','LineWidth',10*errorMag3_planar((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 3: Surface fit using planar regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'Planar regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/Planar_Verification3';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)


% surf3 Surface fit 3
close all
figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');

fit_horz_surf33 = deg2rad( fitresult60_horz_surf3(pointStats3(:,1),pointStats3(:,3)) + 90 ); % transform back into vicon frame with a 90deg rotation
fit_vert_surf33 = deg2rad( -fitresult60_vert_surf3(pointStats3(:,1),pointStats3(:,3)) ); % transform back to standard elevation angle by multiplying by -1

% [x,y,z] = sph2cart(azimuth,elevation,r) transforms corresponding elements of the spherical coordinate arrays azimuth, elevation, and r to Cartesian, or xyz, coordinates.
[xFit_surf33,yFit_surf33,zFit_surf33] = sph2cart(fit_horz_surf33,fit_vert_surf33,dist_pdoa3/1000);
fit_surf33 = (R3*[xFit_surf33,yFit_surf33,zFit_surf33]')';

errorMag3_surf3 = sqrt( (viconPoints3(:,1) - fit_surf33(:,1)) .^2 + (viconPoints3(:,2) - fit_surf33(:,2)) .^2 + (viconPoints3(:,3) - fit_surf33(:,3)) .^2 );
interleav3_surf3 = reshape([viconPoints3(:) fit_surf33(:)]',2*size(viconPoints3,1), []);

plot3(fit_surf33(:,1),fit_surf33(:,2),fit_surf33(:,3),'.','MarkerSize',35)
hold on
plot3(viconPoints3(:,1),viconPoints3(:,2),viconPoints3(:,3),'.','MarkerSize',35);
hold on
plot3(pdoa_node_midl(1),pdoa_node_midl(2),0,'m*','MarkerSize',15,'LineWidth',1)
hold on
for k = 1:2:length(interleav3_surf3)-1
    plot3(interleav3_surf3(k:k+1,1),interleav3_surf3(k:k+1,2),interleav3_surf3(k:k+1,3),'k.-','LineWidth',10*errorMag3_surf3((k+1)/2));
    hold on
end

axis equal
set(gca,'Units','normalized','Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title('ECR Verification 3: Surface fit using poly33 regression vs ground truth locations',...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'$x[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend({'poly33 regression','VICON data points','PDoA Shield Center','Error'},...
        'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','SouthEast')
    
% 3D pdf!
ax = gca;
filename = 'figs_verify/surf3_Verification3';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png
% fig2u3d(ax,filename)

% histogram 3
close all

edges = 0:0.05:1;
histogram(errorMag3_surf3,edges,'FaceColor',[0.4660 0.6740 0.1880])
hold on
histogram(errorMag3_planar,edges,'FaceColor',[0.8500 0.3250 0.0980])
hold on
histogram(errorMag3_ANN,edges,'FaceColor',[0 0.4470 0.7410])
legend('poly33','Planar','ANN')

set(gca,'Units','normalized',...'Position',[.15 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'ECR Verification 3: Error histogram comparison'},...
      'FontUnits','points','FontWeight','normal','interpreter','latex','FontSize',titleFontSize,'FontName',Font)
xlabel({'Euclidean Error [m]'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'Count'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
legend('poly33','Planar','ANN')  

dim = [.45 .6 .0 .0]; %annotation
start_str = sprintf("Mean Euclidean Error:");
ANN_str = sprintf('ANN regression = %0.1f cm', mean(errorMag3_ANN)*100);
planar_str = sprintf('planar regression = %0.1f cm', mean(errorMag3_planar)*100);
surf3_str = sprintf('poly33 regression = %0.1f cm', mean(errorMag3_surf3)*100);
str = start_str + "\n" + surf3_str + "\n" + planar_str + "\n" + ANN_str;
str = compose(str);
str = splitlines(str);
annotation('textbox',dim,'String',str,'FitBoxToText','on');

filename = 'figs_verify/histogram3';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint/word
print(filename,'-dpng','-opengl'); %print figure as png