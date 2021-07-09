# lib_planning


## Dependencies

This package requires of the following system libraries and packages

* [cmake](https://www.cmake.org "CMake's Homepage"), a cross-platform build system.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), C++ template library for linear algebra

## Compilation and installation

Installation:

```shell
git clone https://github.com/AUROVA-LAB/lib_planning
cd lib_planning/
mkdir build
cd build
cmake .. 
make
sudo make install
```

## How to include it

To use this library in an other library or application, it is necessary add in the CMakeLists.txt file:

``` find_package(planning REQUIRED) ```

And Eigen's dependence is necessary:

``` 
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
```

And link the libraries to the program

``` 
TARGET_LINK_LIBRARIES(<executable name> planning) 
```

## How to use it (Global Path Planning)

#### Include:

```c++
#include <planning/graph.h>
```

#### Structs:

##### Pose:

```c++
struct Pose{ 
  vector<double> coordinates; 
  vector<vector<double> > matrix; 
};
```

  - coordinates: Vector with coordinates x,y,z,yaw

  - matrix: Covariance matrix (4x4)


##### StNodes:

```c++
struct StNodes {
  long id;
  vector<double> coordinates;
  vector<vector<double> > matrix;
  vector<long> nodesConnected;
};
```
  - id: An identifier of the node

  - coordinates: Vector with coordinates x,y,z,yaw

  - matrix: Covariance matrix (4x4)

  - nodesConnected: identifiers of connected nodes


#### Constructors: 

```c++	
Graph(string url, vector<vector<double> > matrix, Util::Distances typeDistance, double radiusDistance) 
```

Load an OpenStreetMap xml
  
  - url: A string where the file .xml is
  (The file must have been saved with latitude longitude coordinates)
  
  - matrix: Covariance matrix
  
  - typeDistance: Can be mahalanobis or euclidean. It is a enum and to use it you have to use Util::Mahalanobis or Util::Euclidean.
  
  - radiusDistance: This value indicates the distance to consider an intermediate position of the trajectory as reached
  
  
```c++	
Graph(Util::Distances typeDistance, double radiusDistance);
```

Does not load any data

  - typeDistance: Can be mahalanobis or euclidean. It is a enum and to use it you have to use Util::Mahalanobis or Util::Euclidean.
  
  - radiusDistance: This value indicates the distance to consider an intermediate position of the trajectory as reached

  
#### Method getNextPose:

```c++ 
Pose getNextPose(Pose myPosition, Pose endGoal)	
```	

  - myPosition: Pose where the robot is
  
  - endGoal: Final goal
  
  return next pose of the trajectory


#### Method getPathPoses:

```c++ 
  vector<Pose> getPathPoses(Pose myPosition, Pose endGoal);
```	

  - myPosition: Pose where the robot is
  
  - endGoal: Final goal
  
return a vector with the next poses of the trajectory


#### Method xmlWrite:

```c++ 
  void xmlWrite(string fileName, int precision);
```	
Write an xml with the information from the graph. The coordinates will be saved with UTM coordinates.

  - fileName: Name of the xml file to write
  
  - precision: Number of decimals for each coordinate to save. 
  ( If the precision is 1, it will save the number with a decimal. If the precision is 15, it will save the number with 15 decimals )  

  
#### Method xmlReadUTM:

```c++ 
  void xmlReadUTM(string url);
```	
Load the information from the file.

  - fileName: Name of the xml file to read
  (The file must have been saved with UTM coordinates)


#### Method loadStructGraph:

```c++ 
  void loadStructGraph(vector<StNodes> st_nodes);
```	

Load the information from the StNodes struct.


  - st_nodes: Vector of StNodes struct


#### Method getStructGraph:

```c++ 
  vector<StNodes> getStructGraph();
```	

 return a vector of StNodes struct


#### Method newMatrix:

```c++ 
vector<vector<double> > static newMatrix(double x, double y, double z, double yaw) 
```

  - x,y,z,yaw: Diagonal values in the matrix (4x4)
  


#### Method setAStarAlgorithm:

```c++ 
  void setAStarAlgorithm();
```	

Set Dijkstra as the algorithm to use


#### Method getPlanningGraph:

```c++ 
  void setDijkstraAlgorithm();
```	

Set Dijkstra as the algorithm to use

## How to use it (Local Path Planning)

#### Include:

```c++
#include <planning/local_planning.h>
```

#### Structs:

##### Configuration for lidar sensor:

```c++
struct SensorConfiguration
{
  float max_elevation_angle;
  float min_elevation_angle;

  float max_azimuth_angle;
  float min_azimuth_angle;

  float grid_azimuth_angular_resolution;
  float grid_elevation_angular_resolution;

  // To calculate these values:
  // 1 + (max_azimuth_angle - min_azimuth_angle) / grid_azimuth_angular_resolution;
  int num_of_azimuth_cells;
  int num_of_elevation_cells;
};
```

##### Configuration for ground filter as obstacle detection:

```c++
struct FilteringConfiguration
{
  float max_range; // Points above the threshold are discarded
  float min_range; // Points below this threshold will get discarded
                   // (to filter reflections in the Ego-vehicle).

                   // canonical parameters of plane
  double a; // plane intersection in x axe
  double b; // plane intersection in y axe
  double c; // plane intersection in z axe
  double variance;
  double radious;
  double var_factor;

  float ground_in_sim;

  bool is_simulation;
};
```

##### Configuration for ackermann control:

```c++
struct AckermannControl
{
  float max_angle;
  float delta_angle;
  float v_min;
  float v_max;
  float v_length;
  float delta_arc;
  float max_arc;
  float kp;
  float margin; //from base_link

  float steering;
  float velocity;
};
```
##### 2D pose struct:

```c++
struct Pose2D
{
  float x;
  float y;
  float yaw;
};
```
#### Constructor: 

```c++	
LocalPlanning(); 
```

#### Method groundSegmentation:

```c++ 
void groundSegmentation(pcl::PointCloud<pcl::PointXYZ> &input_cloud,
      local_planning_lib::SensorConfiguration lidar_configuration,
      local_planning_lib::FilteringConfiguration filtering_configuration,
      pcl::PointCloud<pcl::PointXYZ> &ground_cloud,
      pcl::PointCloud<pcl::PointXYZ> &obstacles_cloud,
      pcl::PointCloud<pcl::PointXYZ> &free_map);	
```	

#### Method localGoalCalculation:

```c++ 
void localGoalCalculation(pcl::PointXYZ global_goal,
      pcl::PointCloud<pcl::PointXYZ> obstacles_cloud,
      pcl::PointCloud<pcl::PointXYZ> free_map,
      pcl::PointCloud<pcl::PointXYZ> &local_path);	
```	

#### Method controlActionCalculation:

```c++ 
void controlActionCalculation(pcl::PointXYZ local_goal,
      local_planning_lib::Pose2D base_in_lidarf,
      pcl::PointCloud<pcl::PointXYZ> obstacles_cloud,
      pcl::PointCloud<pcl::PointXYZ> &collision_risk,
      pcl::PointCloud<pcl::PointXYZ> &collision_actions,
      pcl::PointCloud<pcl::PointXYZ> &free_actions,
      local_planning_lib::AckermannControl &ackermann_control);	
```
