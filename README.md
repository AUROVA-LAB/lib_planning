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

### How to use it

#### Struct:

```c++
struct Pose{ 
  vector<double> coordinates; 
  vector<vector<double> > matrix; 
};
```


coordinates: Vector with coordinates x,y,z,yaw

matrix: Covariance matrix (4x4)


#### Constructor: 

```c++	
Graph(string url, vector<vector<double> > matrix,string typeDistance, double radiusDistance) 
```
  
  url: A string where the file .xml is
  
  matrix: Covariance matrix
  
  typeDistance: Can be "M" or "E". "M" means mahalanobis distance and "E" means Euclidean distance
  
  radiusDistance: This value indicates the distance to consider an intermediate position of the trajectory as reached
  
#### Method getNextPose:

```c++ 
Pose getNextPose(Pose myPosition, Pose endGoal)	
```	

  myPosition: Pose where the robot is
  
  endGoal: Final goal
  
  return next pose of the trajectory


#### Method newMatrix:

```c++ 
vector<vector<double> > static newMatrix(double x, double y, double z, double yaw) 
```

  x,y,z,yaw: Diagonal values in the matrix (4x4)
