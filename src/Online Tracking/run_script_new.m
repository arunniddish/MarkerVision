%==========================================================================================================================
% Script Name: run_script.m
% Author: Arun Niddish Mahendran
% Last modified date: 2024-MM-DD
% Description: Briefly describe what this script does.
% Inputs:
% 2) "vid_name"     : Video's name with the extension to be
%                     tracked.
%
% 3) "output_vid_filename" : Specify any name with the extension
%                            to save the animation of tracked
%                            file
%
% 4) "params.number_of_markers" : Number of markers on the body
%                                 to be tracked
%
% Outputs:
% 1) "tracking_data" : Centroid data (x,y,z) of the respective markers on the robot,
%                      global rotation and translation matrix; body
%                      rotation and translation matrix.
% Dependencies:
% 1) "OnlineTracking.m" : Class for performing online tracking.
%
% 2) "parfeval(@getFrameFromCamera,...)" : Parallel processing for capturing frames using the function @getFrameFromCamera
% 
% 3) "camera_properties_marker_1920_Blue.m" : Function to set camera properties for easily segmenting blue markers at 1920x1080 resolution.
%
% 4) "createMaskhdblue.m" : Function to create binary mask for segmenting blue markers at 1920x1080 resolution.
%
% The data are in the following order:
% 1 to 3 [1x3] Marker 1 [x,y,z]
% 4 to 6 [1x3] Marker 2 [x,y,z]
% 7 to 9 [1x3] Marker 3 [x,y,z]
% 10 to 12 [1x3] Marker 4 [x,y,z]
% 13 to 21 [1x9] Rotation matrix Intermediate frames [Reshaped from 3x3 to 1x9]
% 22 to 24 [1x3] Translation Matrix Intermediate frames [[x,y,z]Reshaped from 3x1 to 1x3]
% 25 to 33 [1x9] Rotation matrix Global frame [Reshaped from 3x3 to 1x9]
% 34 to 36 [1x3] Translation Matrix Global frame [[x,y,z]Reshaped from 3x1 to 1x3]
% 37 [1x1] Timestamp
%============================================================================================================================

clear all;
clc;

%[] Video file information
vid_name = 'demo_video.mp4'; % Raw video data after configuring camera properties
output_vid_filename = 'demo_video_gcf.mp4'; % Final tracked video

% Initalize object to write tracked video frames
params.vwrite_tracked = VideoWriter(output_vid_filename,'MPEG-4');
open(params.vwrite_tracked);

% Tracking parameters
params.number_of_markers = 4;

% Choose overlay image (Yes/No - 1/0)
params.overlay = 0;

%[] Initialize camera and camera properties
choose_cam = menu("Initialize camera",'Yes','No');
% Camera for tracking
if choose_cam == 1
    cam = webcam(1);
    cam.Resolution = '1920x1080';
    preview(cam);
end
choose_cam_prop = menu("Adjust Camera properties",'Yes','No');
if choose_cam_prop == 1
    cam = functions.camera_properties_marker_1920_Blue(cam);
    cam.Resolution = '1920x1080';
    pause(3);
end

input('Press <Enter> to start');

% Check 1st frame for marker visibility
if choose_cam == 1
    first_frame = snapshot(cam);
    first_frame = functions.createMaskhdblue(first_frame);
    first_frame = bwareaopen(first_frame,25);
    first_frame = imfill(first_frame, 'holes');
    [labeledImage, numberOfRegions] = bwlabel(first_frame);
    stats = regionprops(labeledImage, 'BoundingBox','Centroid');
    count = 0;
    for rb = 1:numberOfRegions
        count = count + 1;
        cent(count,:) = stats(rb).Centroid;
        cent(count,2) = 1080 - cent(count,2) ;  % Correction for y-axis.
        BB(count,:) = stats(rb).BoundingBox;
    end

    figure;
    imshow(first_frame);
    hold on;

    % Centroid
    plot(cent(:,1),1080 - cent(:,2),'r*','MarkerSize',10,'LineWidth',2);
    % Bounding box
    for ii = 1:size(BB,1)
        h = rectangle('Position',BB(ii,:),'EdgeColor','y','LineWidth',3);
    end

    caption = sprintf('Check marker visibility. %d areas are segmented. Press any key to continue',numberOfRegions);
    title(caption);

    if numberOfRegions < params.number_of_markers
        disp('Not all markers are visible and segmented.!! Adjust camera position and properties');
        error('Exiting script');
    end

    if numberOfRegions > params.number_of_markers
        disp('More markers are visible than expected.!! Adjust camera position and properties');
        error('Exiting script');
    end

    if numberOfRegions == params.number_of_markers
        disp('All markers are visible and segmented. Proceeding with tracking if you think these are the intended markers.');
    end
    disp('Press any key to continue');
    pause;
end

close all;
clear cam;

pause(2);

%[] Initilaize object for online tracking
addpath(pwd);
tracker_obj = onlinetracking.OnlineTracking(params);
PrevPt = [];
P0 = [];
robot_centroid = [];
theta_curr = [];
tracking_data = [];

% Distortion parameter
% Uncomment to use it but create your own. Check paper Sec III-A for how to create one.
% This is not mandatory for the tracking framework to run. This just corrects for distortion which
% are not that significant.
% distortion = load('cameraParams.mat');

%[] Initialize Parallel Processing --> parfeval-(f)
parpool('local',1); % It is better if number of pools opened is equal to number of asynchronous function running

%[] Initialize --> Pollable Data Queue-(P)
P = parallel.pool.PollableDataQueue;

%f_1 = parfeval(@getFrameFromCamera,0,P,L,distortion,vid_name);  % Uncomment and use this if you have distortion parameters. Also uncomment in @getFrameFromCamera
f_1 = parfeval(@getFrameFromCamera, 0, P, vid_name);

pause(2);

% input('Press <Enter> to start');

idx = 0;

while true

    if P.QueueLength > 0

        cam_data = poll(P);
        newim = cam_data.img;
        ts = cam_data.time_stamp;
        idx = idx + 1;
        [output_data] = tracker_obj.tracking(newim, PrevPt, P0, robot_centroid, theta_curr, idx);
        tracking_data(idx,:) = [output_data.tracking_data(idx,:) ts];
        PrevPt = reshape(output_data.tracking_data(idx,1:12),[3,4])';
        robot_centroid = output_data.robot_centroid;
        theta_curr = output_data.theta_curr;
        check(idx,:) = theta_curr;

        if idx == 1
            P0 = output_data.P0;
        end

    end

end
