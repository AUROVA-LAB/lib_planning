#ifndef INCLUDES_GRAPH_H_
#define INCLUDES_GRAPH_H_

#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#define IMPOSSIBLE_RANGE_VALUE -1.0

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
  float min_dot_product_for_ground;                   // Only used in ground filtering
  float max_ground_elevation_angle_change_in_degrees; // Only used in ground filtering
};
}


class LocalPlanning
{
public:

  LocalPlanning();
  virtual ~LocalPlanning();
  
  void sphericalInDegrees2Cartesian(float range, float azimuth, float elevation,
                                   float& x, float& y, float& z);
  
  void cartesian2SphericalInDegrees(float x, float y, float z, float& range,
                                    float& azimuth, float& elevation);
                                                 
  void initializeSphericalGrid(float** grid, 
                               local_planning_lib::SensorConfiguration lidar_configuration);

  void pointCloud2SphericalGrid(pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                                local_planning_lib::SensorConfiguration lidar_configuration,
                                local_planning_lib::FilteringConfiguration filtering_configuration, 
                                float** output_range_grid);
                                
  void sphericalGridToPointCloud(float** spherical_grid, 
                                 local_planning_lib::SensorConfiguration lidar_configuration,
                                 pcl::PointCloud<pcl::PointXYZ>& output_cloud);
  
  void freeSpaceMap(pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                    local_planning_lib::SensorConfiguration lidar_configuration,
                    local_planning_lib::FilteringConfiguration filtering_configuration,
                    pcl::PointCloud<pcl::PointXYZ>& output_cloud);
                    
  void freeSpaceMap(float **input_grid, 
                    local_planning_lib::SensorConfiguration lidar_configuration,
                    local_planning_lib::FilteringConfiguration filtering_configuration,
                    float** output_grid);

};

#endif
