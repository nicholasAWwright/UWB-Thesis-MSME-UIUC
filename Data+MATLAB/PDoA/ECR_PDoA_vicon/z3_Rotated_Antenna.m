clear all
close all

% Import rosbag data file
Bag1 = rosbag('2020-03-18-11-34-10.bag');
Bag2 = rosbag('2020-03-18-11-46-15.bag');

% Topic extraction
EXPdata1 = select(Bag1,'Topic','/uwb/pdoaVICON/dataFiltered');
EXPdata2 = select(Bag2,'Topic','/uwb/pdoaVICON/dataFiltered');

% Message extraction (need to define custom ROS message for MATLAB: https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html)
[ts1,cols1] = timeseries(EXPdata1,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
[ts2,cols2] = timeseries(EXPdata2,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
% This takes a relatively long time

%Populate MATLAB variables from timeseries data
t1 = ts1.time;
dist_horz_pdoa1 = ts1.data(:,1);
pdoa_horz1 = ts1.data(:,2);
dist_vert_pdoa1 = ts1.data(:,3);
pdoa_vert1 = ts1.data(:,4);
xVICON1 = ts1.data(:,5);
yVICON1 = ts1.data(:,6);
zVICON1 = ts1.data(:,7);

%Populate MATLAB variables from timeseries data
t2 = ts2.time;
dist_horz_pdoa2 = ts2.data(:,1);
pdoa_horz2 = ts2.data(:,2);
dist_vert_pdoa2 = ts2.data(:,3);
pdoa_vert2 = ts2.data(:,4);
xVICON2 = ts2.data(:,5);
yVICON2 = ts2.data(:,6);
zVICON2 = ts2.data(:,7);

pdoa_node_midl = [0,0,0.8163]; %location of the PDOA node in vicon arena, halfway between horz and vert

data_raw1 = [pdoa_horz1,dist_horz_pdoa1,pdoa_vert1,dist_vert_pdoa1,xVICON1,yVICON1,zVICON1-pdoa_node_midl(3)];
data_raw2 = [pdoa_horz2,dist_horz_pdoa2,pdoa_vert2,dist_vert_pdoa2,xVICON2,yVICON2,zVICON2-pdoa_node_midl(3)];

%% plot

% output1 = ANN_60_50n(data_raw1(:,1:4));
% plot3(output1(:,1),output1(:,2),output1(:,3),'.');
% hold on
% plot3(data_raw1(:,5),data_raw1(:,6),data_raw1(:,7),'.','MarkerSize',35);
% axis equal

output2 = ANN_60_50n1_normal(data_raw2(:,1:4)')';
plot3(output2(:,1),output2(:,2),output2(:,3),'.');
hold on
plot3(data_raw2(:,5),data_raw2(:,6),data_raw2(:,7),'.','MarkerSize',35);
axis equal