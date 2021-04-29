clear all
close all

% xDoc = xmlread(fullfile('U:\REINO\LatestCalibration.xml'));
% allListitems = xDoc.getElementsByTagName('KeyFrame');

sampleXMLfile = 'VICON_LatestCalibration.xml';
type(sampleXMLfile)

DOMnode = xmlread(sampleXMLfile);