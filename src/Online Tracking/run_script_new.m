clear all;
clc;

%[] Video file information
vid_name = 'demo_video.mp4'; % Raw video data after configuring camera properties 
output_vid_filename = 'demo_video_gcf.mp4'; % Final tracked video 

% Instantiate a video objects for this video.
params.vwrite = VideoWriter(output_vid_filename,'MPEG-4');
open(params.vwrite);

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
    cam = camera_properties_marker_1920_Blue(cam);
    cam.Resolution = '1920x1080';
	pause(2);
end

% Distortion parameter
% Uncomment to use it but create your own. Check paper Sec III-A for how to create one.
% This is not mandatory for the tracking framework to run. This just corrects for distortion which
% are not that significant.
% distortion = load('cameraParams.mat');  

% Initilaize object for online tracking
tracker_obj = OnlineTracking(params);
PrevPt = [];
P0 = [];
robo_centroid = [];

input('Press <Enter> to start');

%[] Initialize --> Pollable Data Queue-(P)
P = parallel.pool.PollableDataQueue;
L = parallel.pool.PollableDataQueue;
Q = parallel.pool.PollableDataQueue;


%[] Initialize --> parfeval-(f)
parpool('local',1);
%freq = 25;
%f = parfeval(@getFrameFromCamera,0,P,L,distortion,vid_name);  % Uncomment and use this if you have distortion parameters. Also uncomment in @getFrameFromCamera
f = parfeval(@getFrameFromCamera,0,P,L,vid_name);
pause(1.5);

idx = 0;

while true

	if P.QueueLength > 0
	
		cam_data = poll(P);
        newim = cam_data.img;
        ts = cam_data.time_stamp;
        poll_data = poll_data + 1;
        idx = idx + 1;
        [output_data] = tracker_obj.tracking(newim,PrevPt,P0,robo_centroid,trajectory_position,idx);
        tracking_data(idx,:) = [output_data.tracking_data(idx,:) ts];
        PrevPt = reshape(output_data.tracking_data(idx,1:12),[3,4])';
        robo_centroid = output_data.robo_centroid;
		
		if idx == 1
            P0 = output_data.P0;
        end
        
    end
plot_data.img = newim;
plot_data.centroids = robo_centroid; 
plot_data.marker_centroid = PrevPt; 
send(Q, plot_data);
end






