# README

This provides a brief description of all the files inside the **Offline Tracking**

1) *video_capture.m* : This matlab code is run for video acquistion for the tracking. 

2) *run_script.m* : This matlab code is run for initializing the offline tracking. 

Inside the folder **functions**,

3) *createMask_1.m* : HSV chromatic mask generated using Color Thresholder app. This mask is specifically made for video *demo_1.mp4* (in folder **demos**). 

4) *createMask_2.m* : HSV chromatic mask generated using Color Thresholder app. This mask is specifically made for *demo_2.mp4* (in folder **demos**).
  
5) *createMask_3.m* : HSV chromatic mask generated using Color Thresholder app. This mask is specifically made for *demo_3.mp4* (in folder **demos**).

6) *cam_properties_marker_1920_Blue.m* : Adjusted camera properties (brightness, saturation, contrast etc.,). 

7) *cameraParams.mat* : Correction for lens distortion.

8) *rigid_transform_3D.m* : Arun's algorithm

Inside the folder **+offlinetracking**,

9) *OfflineTracking.m* : This is the class containing all the algorithm for performing offline tracking.
