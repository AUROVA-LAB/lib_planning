#ifndef INCLUDES_GRAPH_H_
#define INCLUDES_GRAPH_H_

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <bits/stdc++.h>
#include <string>

#define IMPOSSIBLE_RANGE_VALUE -1.0
#define PI 3.141592

using namespace std;

namespace local_planning_lib
{
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

  float sensor_height;
};

struct FilteringConfiguration
{
  float max_range; // Points above the threshold are discarded
  float min_range; // Points below this threshold will get discarded
                   // (to filter reflections in the Ego-vehicle).
  float min_dot_product_for_ground;             // Only used in ground filtering
  float max_ground_elevation_angle_change_in_degrees; // Only used in ground filtering

  // canonical parameters of plane
  double a; // plane intersection in x axe
  double b; // plane intersection in y axe
  double c; // plane intersection in z axe
  double variance;
  double radious;
  double var_factor;
};

struct AckermannControl
{
  float max_angle;
  float delta_angle;
  float v_min;
  float v_max;
  float v_length;
  float delta_time;

  float steering;
  float direction;
};

struct Pose2D
{
  float x;
  float y;
  float yaw;
};

}

class LocalPlanning
{
public:

  LocalPlanning();
  virtual ~LocalPlanning();

  void sphericalInDegrees2Cartesian(float range, float azimuth, float elevation,
      float &x, float &y, float &z);

  void cartesian2SphericalInDegrees(float x, float y, float z, float &range,
      float &azimuth, float &elevation);

  void initializeSphericalGrid(float **grid,
      local_planning_lib::SensorConfiguration lidar_configuration);

  void pointCloud2SphericalGrid(pcl::PointCloud<pcl::PointXYZ> &input_cloud,
      local_planning_lib::SensorConfiguration lidar_configuration,
      local_planning_lib::FilteringConfiguration filtering_configuration,
      float **output_range_grid);

  void sphericalGridToPointCloud(float **spherical_grid,
      local_planning_lib::SensorConfiguration lidar_configuration,
      pcl::PointCloud<pcl::PointXYZ> &output_cloud);

  void groundSegmentation(pcl::PointCloud<pcl::PointXYZ> &input_cloud,
      local_planning_lib::SensorConfiguration lidar_configuration,
      local_planning_lib::FilteringConfiguration filtering_configuration,
      pcl::PointCloud<pcl::PointXYZ> &output_cloud,
      pcl::PointCloud<pcl::PointXYZ> &obstacles_cloud,
      pcl::PointCloud<pcl::PointXYZ> &limits_cloud);

  void localGoalCalculation(pcl::PointXYZ global_goal,
      pcl::PointCloud<pcl::PointXYZ> obstacles_cloud,
      pcl::PointCloud<pcl::PointXYZ> limits_cloud, pcl::PointXYZ &local_goal);

  void controlActionCalculation(pcl::PointXYZ local_goal,
      local_planning_lib::Pose2D base_in_lidarf,
      pcl::PointCloud<pcl::PointXYZ> obstacles_cloud,
      pcl::PointCloud<pcl::PointXYZ> &collision_risk,
      local_planning_lib::AckermannControl &ackermann_control);

};

#endif
