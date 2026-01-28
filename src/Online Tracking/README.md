# README

This provides a brief description of all the files inside the **Online Tracking**

1) *getFrameFromCamera.m* : This is MATLAB code which will execute asynchronously for capturing frame. 

2) *run_script_new.m* : This matlab code is run for initializing the online tracking. 

Inside the folder **functions**,

3) *colorMasks_####.m* : HSV chromatic mask generated using Color Thresholder app. 

4) *camera_properties_marker_1920_Blue.m* : Adjusted camera properties (brightness, saturation, contrast etc.,). 

5) *cameraParams.mat* : Correction for lens distortion.

6) *rigid_transform_3D.m* : Arun's algorithm

Inside the folder **+onlinetracking**,

7) *OfflineTracking.m* : This is the class containing all the algorithm for performing online tracking.
