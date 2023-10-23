# Kalman Filter Multi Object Tracker

## Build the project 

To build the project: 
```
mkdir build
cd build
cmake ..
make -j2
```
Download the dataset with the point cloud from here (https://drive.google.com/file/d/1TWfV1odleih6d0SG7q2Sbs1jbZ4oSPkP/view?usp=share_link) and place the ```log``` folder in the used build folder. 


Run the project by executing
```
./main
```

NOTE: the program will NOT work properly, some implementation is missing.

## Goals: 

This code implements a Kalman Filter to track the pedestrians on the LiDAR point cloud using the given clusters.
The handling of the point cloud and the clustering is given, as well as the tracker implementation and the viewer. 
Only some parts are missing. 
You are not required to implement anything, we do not expect for the solution should to be delivered. 
However, this project will be the starting point for the technical interview. 

- Files to complete: 
    - KalmanFilter.cpp: implement the Kalman Filter predict and update and initialize the covariance matrix
    - Tracker.cpp: implement the following components: 
        - Implement the initialization initialization (initialize the variables)
        - Implement the track logic
        - Implement the removal tracklets logic
        - Implement the data association to associate clusters with tracklets (i.e. tracked objects).
- Files to ignore: everything related to the viewer or cloud processing (e.g. Rendered.cpp or CloudManager.cpp). The clustering is given and no contribution it expected on that.
- Suggest possible improvements to enhance the solution.

Note: the 'v' key turns off the LiDAR clusters (hence the software just uses the Kalman Filter estimations). Could be helpful to validate the implementation.

## Evaluation metrics (over 15 points extra 5 optional points): 

KF tracker compile and work (0-10 points)
    - The score will be assigned according to the performance of the tracker (tracking performance, timing performance, code quality, and comments) 
    - The more the originality of the code the better the degree will be

Optional EKF assignment (0-5 points)
    - Please see the readme
    
Implement any cool functionality! (0-5 points)
-   Return the id and length of the path of the track that has traveled the longest path
-   Define an area and count the persons that has entered in that area
-   Define an area and return the ID of the person who has been in that area the longest
-   Feel free to propose your idea!
# KF_assignment
