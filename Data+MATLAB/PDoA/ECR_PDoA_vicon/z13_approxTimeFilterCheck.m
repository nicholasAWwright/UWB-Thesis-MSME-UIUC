% Nicholas AW Wright
% March 4, 2021
% This is a simple check to make sure that changing parameters for the
% approx time filter did not alter data collection. New filter was run with
% old bags and a new filtered topic was generated for comparison. Looks
% like the max error introduced in the second run was less than a
% millimeter and overall should have no effect on the previously collected
% data. Can always run the old filter on new data too just to be certain

clear all
close all

% Import rosbag data file
Bag1 = rosbag('2020-03-18-11-34-10_derived.bag');
Bag2 = rosbag('2020-03-18-11-46-15_derived.bag');

% Topic extraction
data1 = select(Bag1,'Topic','/uwb/pdoaVICON/dataFiltered');
data1_check = select(Bag1,'Topic','/uwb/pdoaVICON/dataFiltered_new');
data2 = select(Bag2,'Topic','/uwb/pdoaVICON/dataFiltered');
data2_check = select(Bag2,'Topic','/uwb/pdoaVICON/dataFiltered_new');

% Message extraction (need to define custom ROS message for MATLAB: https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html)
[ts1,cols1] = timeseries(data1,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
[ts2,cols2] = timeseries(data1_check,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
[ts3,cols3] = timeseries(data2,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');
[ts4,cols4] = timeseries(data2_check,'DistHorz','PdoaHorz','DistVert','PdoaVert','Pose.Position.X','Pose.Position.Y','Pose.Position.Z');

% This takes a relatively long time

%Populate MATLAB variables from timeseries data
t_1 = ts1.time;
dist_horz_pdoa_1 = ts1.data(:,1);
pdoa_horz_1 = ts1.data(:,2);
dist_vert_pdoa_1 = ts1.data(:,3);
pdoa_vert_1 = ts1.data(:,4);
xVICON_1 = ts1.data(:,5);
yVICON_1 = ts1.data(:,6);
zVICON_1 = ts1.data(:,7);
vicon_1 = [xVICON_1,yVICON_1,zVICON_1];

t_1_check = ts2.time;
dist_horz_pdoa_1_check = ts2.data(:,1);
pdoa_horz_1_check = ts2.data(:,2);
dist_vert_pdoa_1_check = ts2.data(:,3);
pdoa_vert_1_check = ts2.data(:,4);
xVICON_1_check = ts2.data(:,5);
yVICON_1_check = ts2.data(:,6);
zVICON_1_check = ts2.data(:,7);
vicon_1_check = [xVICON_1_check,yVICON_1_check,zVICON_1_check];

t_2 = ts3.time;
dist_horz_pdoa_2 = ts3.data(:,1);
pdoa_horz_2 = ts3.data(:,2);
dist_vert_pdoa_2 = ts3.data(:,3);
pdoa_vert_2 = ts3.data(:,4);
xVICON_2 = ts3.data(:,5);
yVICON_2 = ts3.data(:,6);
zVICON_2 = ts3.data(:,7);
vicon_2 = [xVICON_2,yVICON_2,zVICON_2];

t_2_check = ts4.time;
dist_horz_pdoa_2_check = ts4.data(:,1);
pdoa_horz_2_check = ts4.data(:,2);
dist_vert_pdoa_2_check = ts4.data(:,3);
pdoa_vert_2_check = ts4.data(:,4);
xVICON_2_check = ts4.data(:,5);
yVICON_2_check = ts4.data(:,6);
zVICON_2_check = ts4.data(:,7);
vicon_2_check = [xVICON_2_check,yVICON_2_check,zVICON_2_check];


%% Data comparisons

dist_horz_check = dist_horz_pdoa_2 - dist_horz_pdoa_2_check;
dist_vert_check = dist_vert_pdoa_2 - dist_vert_pdoa_2_check;
pdoa_horz_check = pdoa_horz_2 - pdoa_horz_2_check;
pdoa_vert_check = pdoa_vert_2 - pdoa_vert_2_check;
t_check = t_2 - t_2_check;
vicon_check = vicon_2 - vicon_2_check;

error_max_dist_horz_check = max(dist_horz_check);
error_max_dist_vert_check = max(dist_vert_check);
error_max_pdoa_horz_check = max(pdoa_horz_check);
error_max_pdoa_vert_check = max(pdoa_vert_check);
error_max_t_check = max(t_check);
error_max_vicon_check = max(vicon_check);
error_sorted_vicon_check = sort(vicon_check, 'descend'); % sort the vicon errors from large to small
error_euclidean_vicon = sqrt( vicon_check(:,1).^2 + vicon_check(:,2).^2 + vicon_check(:,3).^2 );
error_euclidean_max = max(error_euclidean_vicon);
error_euclidean_sorted = sort(error_euclidean_vicon, 'descend');


