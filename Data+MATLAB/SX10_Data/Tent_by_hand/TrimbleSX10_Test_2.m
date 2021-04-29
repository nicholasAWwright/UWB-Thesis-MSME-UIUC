clear all
close all

%% Analyze rosbag data files for VICON windows/linux time sync testing

%Driving the new j8 around the tent with a trimble prism tracking on top
Bag = rosbag('2021-01-28-16-41-29.bag');

%% Topic extraction
data = select(Bag,'Topic','/trimble/SX10/data');
dataRaw = select(Bag,'Topic','/trimble/SX10/dataRaw');

%% Message extraction
[ts0,cols0] = timeseries(data,'Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
t0 = ts0.time;
x0 = ts0.data(:,1);
y0 = ts0.data(:,2);
z0 = ts0.data(:,3);
%%
[ts2,cols2] = timeseries(dataRaw,'Point.X','Point.Y','Point.Z');
t2 = ts2.time;
x2 = ts2.data(:,1);
y2 = ts2.data(:,2);
z2 = ts2.data(:,3);

%% Data analysis

%%plots

plot(x0,y0)
% plot3(x0,y0,z0'.')
% plot(x2,y2,'.')

% plot(x0,y0,'.',x2,y2,'.')
% text(xAnchor,yAnchor,aLabels,'VerticalAlignment','top','HorizontalAlignment','left')
% % plot(xUWBpose,yUWBpose,'.')
% title ('Position Measurements of the Jackal Robot using UWB outside at CERL (ENU frame)')
% legend ('UWB tag0 position','UWB tag2 position')
% xlabel ('X [m]')
% ylabel ('Y [m]')




