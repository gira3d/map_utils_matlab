^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package map_utils_matlab
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2024-03-02)
------------------
* updates license
* Contributors: Wennie Tabib

0.1.0 (2020-06-17)
------------------
* Merge branch 'feature/sliding_grid' into 'develop'
  Feature/sliding grid
  See merge request wtabib/map_utils_matlab!1
* Updates log_odds_hit and log_odds_miss values
* Updates disjunctiveUnion functions to setDifference
* decrease max clamp so that it entropy doesn't become NaN
* add getRayPoints function which returns voxels along a ray
* get unknown point cloud and map entropy
* disjunctive union and intersection functions
* remove CXX_STANDARD 17 and correct name of library
* compile only if MATLAB is found on system
* get parameters from matlab pointer to map_utils::grid3d::Grid3D
* add bbx_t matlab bindings and functions to interact with sliding grid3d
* pass width, height, and depth as arguments to the occupancy grid
* updated parameters using Kshitij's aaai paper
* update FindMATLAB.cmake to automatically generalize across matlab versions and
  operating systems.
* Nate stores quaternions as <w,x,y,z> and ROS does <x,y,z,w>.
* remove print in mex file
* initial commit
* Contributors: Wennie Tabib
