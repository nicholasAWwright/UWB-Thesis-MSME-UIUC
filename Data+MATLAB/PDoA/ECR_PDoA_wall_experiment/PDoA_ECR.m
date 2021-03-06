% Nicholas AW Wright
% 2020 February 28
% PDoA on the ECR wall experiment
% Y reversed from TS measurements to match standard convention

clear all
close all

%% Import data from spreadsheet
% Script for importing data from the following spreadsheet:
%
%    Workbook: U:\REINO\PDoA_Data\PDoA_Data_Compiled_11-13-2019.xlsx
%    Worksheet: Digestible
%
% Auto-generated by MATLAB on 28-Feb-2020 14:42:57

% Setup the Import Options and import the data
opts = spreadsheetImportOptions("NumVariables", 14);

% Specify sheet and range
opts.Sheet = "Digestible";
opts.DataRange = "A2:N76";

% Specify column names and types
opts.VariableNames = ["AvgD_horzmm", "StdDD_horzmm", "AvgPDoA_horzmDegrees", "StdDPDoA_horzmDegrees", "AvgD_vertmm", "StdDD_vertmm", "AvgPDoA_vertmDegrees", "StdDPDoA_vertmDegrees", "Northingm", "Eastingm", "Elevationm", "Xmm", "Ymm", "Zmm"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ImportErrorRule = "omitrow";
opts.MissingRule = "omitrow";

% Specify variable properties
opts = setvaropts(opts, ["AvgD_horzmm", "StdDD_horzmm", "AvgPDoA_horzmDegrees", "StdDPDoA_horzmDegrees", "AvgD_vertmm", "StdDD_vertmm", "AvgPDoA_vertmDegrees", "StdDPDoA_vertmDegrees", "Northingm", "Eastingm", "Elevationm", "Xmm", "Ymm", "Zmm"], "TreatAsMissing", '');

% Import the data
PDOAdata = readtable("Z:\Dropbox\RIENO\THESIS\Data\PDoA_Data\ECR_wall_experiment\Data\PDoA_Data_Compiled_11-13-2019.xlsx", opts, "UseExcel", false);


% Clear temporary variables
clear opts

%% Variable assignment
PDOAmatrix = PDOAdata{:,:}; %bring the data into a matrix

D_horz_avg = PDOAmatrix(:,1)/1000; %m
D_horz_sdv = PDOAmatrix(:,2)/1000; %m
PDOA_horz_avg = PDOAmatrix(:,3)/1000; %Deg
PDOA_horz_sdv = PDOAmatrix(:,4)/1000; %Deg
D_vert_avg = PDOAmatrix(:,5)/1000; %m
D_vert_sdv = PDOAmatrix(:,6)/1000; %m
PDOA_vert_avg = PDOAmatrix(:,7)/1000; %Deg
PDOA_vert_sdv = PDOAmatrix(:,8)/1000; %Deg
X = PDOAmatrix(:,12)/1000; %m
Y = PDOAmatrix(:,13)/1000; %m
Z = PDOAmatrix(:,14)/1000; %m
D = sqrt(X.^2+Y.^2+Z.^2); %Distance measured by TS

%% Plots

% plot(Y,D_horz_avg,'.')
% plot(Y,D_vert_avg,'.')
% plot(Y,PDOA_horz_avg,'.')
% plot(Y,PDOA_vert_avg,'.')
% plot(PDOA_horz_avg,Z,'.')clc

% plot(Z,PDOA_vert_avg,'.')
%% Measurement locations
plot3(X,Y,Z,'.')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal

%% UWB Horz Distance at measurement locations
plot3(Y,Z,D_horz_avg,'.',Y,Z,D,'.')
xlabel('Y')
ylabel('Z')
zlabel('Horizontal Distance')

%% Difference between UWB Horz Distance and TS distance
plot3(Y,Z,D_horz_avg-D,'.')
xlabel('Y')
ylabel('Z')
zlabel('Horizontal Distance')

%% UWB Horz Distance at measurement locations
[yi,zi] = meshgrid(-4:0.1:4, -4:0.1:4);
Di = griddata(Y,Z,D_horz_avg,yi,zi);
surf(yi,zi,Di)
xlabel('Y')
ylabel('Z')
zlabel('Horizontal Distance')

%%  Difference between UWB Horz Distance and TS distance
[yi,zi] = meshgrid(-4:0.05:4, -4:0.05:4);
Di = griddata(Y,Z,D_horz_avg-D,yi,zi);
surf(yi,zi,Di)
xlabel('Y')
ylabel('Z')
zlabel('Horizontal Distance')
shading interp

%% UWB Horz PDOA at measurement locations

close all
% Figure properties
width = 6; %figure size in inches
height = 8;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
[yi,zi] = meshgrid(-4:0.1:4, -4:0.1:4);
Di = griddata(Y,Z,PDOA_horz_avg,yi,zi);
surf(yi,zi,Di)
% shading flat
zlim([-180 180])
zticks(-180:30:180)
% axis equal
grid on
set(gca,'Units','normalized','Position',[.16 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Horizontal PDoA Constrained to a Plane'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$Y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$Z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$PDoA_{horz}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({''},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')

ax = gca;
filename = 'figs/PDoA_Horz';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png

%% UWB Vert PDOA at measurement locations
close all
% Figure properties
width = 6; %figure size in inches
height = 8;
x0 = 10; %position on screen
y0 = 4;
titleFontSize = 12;
axisFontSize = 9;
legendFontSize = 9;
Font = 'Times';

figure('Units','inches','Position',[x0 y0 width height],'PaperPositionMode','auto');
[yi,zi] = meshgrid(-4:0.1:4, -4:0.1:4);
Di = griddata(Y,Z,PDOA_vert_avg,yi,zi);
surf(yi,zi,Di)
% shading flat
zlim([-180 180])
zticks(-180:30:180)
% axis equal
grid on
set(gca,'Units','normalized','Position',[.16 .2 .75 .7],...
        'FontUnits','points','FontWeight','normal','FontSize',axisFontSize,'FontName',Font)
title({'Vertical PDoA Constrained to a Plane'},...
      'FontUnits','points','FontWeight','normal','FontSize',titleFontSize,'FontName',Font)
xlabel({'$Y[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
ylabel({'$Z[m]$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
zlabel({'$PDoA_{vert}$'},'FontUnits','points','interpreter','latex','FontSize',axisFontSize,'FontName',Font)
% legend({''},...
%         'Units','points','interpreter','latex','FontSize',legendFontSize,'FontName',Font,'Location','South')

ax = gca;
filename = 'figs/PDoA_Vert';
print(filename,'-depsc','-painters'); %print figure as eps for the infinite zoom
print(filename,'-dsvg','-painters'); %print figure as svg for powerpoint
print(filename,'-dpng','-opengl'); %print figure as png


%% TS distance vs UWB distance
plot(D,D-D_horz_avg,'.',D,D-D_vert_avg,'.')


