#include "../includes/local_planning.h"

LocalPlanning::LocalPlanning()
{
}

LocalPlanning::~LocalPlanning()
{
}

void LocalPlanning::sphericalInDegrees2Cartesian(float range, float azimuth,
    float elevation, float &x, float &y, float &z)
{
  float sin_elevation, cos_elevation, sin_azimuth, cos_azimuth;

  sin_elevation = sin(elevation * M_PI / 180.0);
  cos_elevation = cos(elevation * M_PI / 180.0);
  sin_azimuth = sin(azimuth * M_PI / 180.0);
  cos_azimuth = cos(azimuth * M_PI / 180.0);

  x = range * sin_elevation * cos_azimuth;
  y = range * sin_elevation * sin_azimuth;
  z = range * cos_elevation;
}

void LocalPlanning::cartesian2SphericalInDegrees(float x, float y, float z,
    float &range, float &azimuth, float &elevation)
{
  range = sqrt((x * x) + (y * y) + (z * z));

  azimuth = atan2(y, x) * 180.0 / M_PI;
  elevation = atan2(sqrt((x * x) + (y * y)), z) * 180.0 / M_PI;

  if (azimuth < 0)
    azimuth += 360.0;
  if (azimuth >= 360)
    azimuth -= 360;

  if (elevation < 0)
    elevation += 360.0;
  if (elevation >= 360)
    elevation -= 360;

  return;
}

void LocalPlanning::initializeSphericalGrid(float **grid,
    local_planning_lib::SensorConfiguration lidar_configuration)
{
  // Initializing the grid with impossible values
  for (register int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i)
  {
    for (register int j = 0; j < lidar_configuration.num_of_elevation_cells;
        ++j)
    {
      grid[i][j] = IMPOSSIBLE_RANGE_VALUE;
      //std::cout << "point col = " << i << "    row = " << j << std::endl; //Debug
    }
  }
}

void LocalPlanning::pointCloud2SphericalGrid(
    pcl::PointCloud<pcl::PointXYZ> &input_cloud,
    local_planning_lib::SensorConfiguration lidar_configuration,
    local_planning_lib::FilteringConfiguration filtering_configuration,
    float **output_range_grid)
{
  float range = 0.0;
  float elevation = 0.0;
  float azimuth = 0.0;

  initializeSphericalGrid(output_range_grid, lidar_configuration);

  //Filling the grids
  int row, col;
  for (size_t i = 0; i < input_cloud.points.size(); ++i)
  {
    pcl::PointXYZ point = input_cloud.points[i];

    cartesian2SphericalInDegrees(point.x, point.y, point.z, range, azimuth,
        elevation);

    //Filtering points of our own vehicle and out of desired FOV
    if (range >= filtering_configuration.min_range
        && range <= filtering_configuration.max_range
        && azimuth <= lidar_configuration.max_azimuth_angle
        && azimuth >= lidar_configuration.min_azimuth_angle
        && elevation <= lidar_configuration.max_elevation_angle
        && elevation >= lidar_configuration.min_elevation_angle)
    {
      row = (int) round(
          (azimuth - lidar_configuration.min_azimuth_angle)
              / lidar_configuration.grid_azimuth_angular_resolution);
      col = (int) round(
          (elevation - lidar_configuration.min_elevation_angle)
              / lidar_configuration.grid_elevation_angular_resolution);

      if (row < 0 || row >= lidar_configuration.num_of_azimuth_cells || col < 0
          || col >= lidar_configuration.num_of_elevation_cells)
      {
        // Debuggin!
      } else
      {
        // We are interested in the closest points to our car, so we keep the minimum value
        if (output_range_grid[row][col] == -1.0
            || (range < output_range_grid[row][col]))
        {
          output_range_grid[row][col] = range;
        }
      }
    }
  }
}

void LocalPlanning::sphericalGridToPointCloud(float **spherical_grid,
    local_planning_lib::SensorConfiguration lidar_configuration,
    pcl::PointCloud<pcl::PointXYZ> &output_cloud)
{
  pcl::PointXYZ point;
  output_cloud.points.clear();

  float range = 0.0;
  float azimuth = 0.0;
  float elevation = 0.0;

  for (register int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i)
  {
    for (register int j = 0; j < lidar_configuration.num_of_elevation_cells;
        ++j)
    {
      if ((int) round(spherical_grid[i][j]) != -1)
      {
        azimuth = ((float) i
            * lidar_configuration.grid_azimuth_angular_resolution)
            + lidar_configuration.min_azimuth_angle;

        elevation = ((float) j
            * lidar_configuration.grid_elevation_angular_resolution)
            + lidar_configuration.min_elevation_angle;

        range = spherical_grid[i][j];

        sphericalInDegrees2Cartesian(range, azimuth, elevation, point.x,
            point.y, point.z);

        output_cloud.points.push_back(point);
      }
    }
  }
}

void LocalPlanning::freeSpaceMap(float **input_grid,
    local_planning_lib::SensorConfiguration lidar_configuration,
    local_planning_lib::FilteringConfiguration filtering_configuration,
    float **output_grid, pcl::PointCloud<pcl::PointXYZ> &perimeter_cloud)
{

  pcl::PointXYZ point;
  perimeter_cloud.points.clear();

  float range;
  float elevation;
  float azimuth;

  float x_a;
  float y_a;
  float z_a;
  int index_a;
  float mod_ab;

  float x_b;
  float y_b;
  float z_b;

  float x_c;
  float y_c;
  float z_c;
  int index_c;
  float mod_bc;

  float dot = 0.0;

  initializeSphericalGrid(output_grid, lidar_configuration);

  // Erasing non obstacle points
  for (register int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i) // We will search in every vertical slice
  {
    azimuth = ((float) i * lidar_configuration.grid_azimuth_angular_resolution)
        + lidar_configuration.min_azimuth_angle;

    // We will check beginning from the lowest points (180 degrees is looking down, 90 front and 0 degrees up)
    register int j = lidar_configuration.num_of_elevation_cells - 1;
    while (j >= 0)
    {
      if (input_grid[i][j] != -1.0) // First point found
      {
        range = input_grid[i][j]; // We extract the x and y coordinates
        elevation = ((float) j
            * lidar_configuration.grid_elevation_angular_resolution)
            + lidar_configuration.min_elevation_angle;

        sphericalInDegrees2Cartesian(range, azimuth, elevation, x_a, y_a, z_a);

        index_a = j;
        float observed_elevation_angle_in_degrees = atan2(
            z_a + lidar_configuration.sensor_height,
            sqrt((x_a * x_a) + (y_a * y_a))) * 180.0 / M_PI;

        if (observed_elevation_angle_in_degrees
            < filtering_configuration.max_ground_elevation_angle_change_in_degrees) // If this point could be ground
        {
          j--;  // Then  we point to the next cell

          while (j >= 0 && input_grid[i][j] == -1) // and keep moving until we find a new point
          {
            j--;
          }
          if (j >= 0) // If we don't have finish the vector, we have found a second point
          {
            range = input_grid[i][j]; // We extract the x and y coordinates for this second point

            elevation = ((float) j
                * lidar_configuration.grid_elevation_angular_resolution)
                + lidar_configuration.min_elevation_angle;

            sphericalInDegrees2Cartesian(range, azimuth, elevation, x_b, y_b,
                z_b);

            j--;  // Then  we point to the next cell

            while (j >= 0 && input_grid[i][j] == -1) // and keep moving until we find a new point
            {
              j--;
            }
            if (j >= 0) // If we don't have finish the vector, then we have found a third point
            {
              range = input_grid[i][j]; // We extract the x and y coordinates for this third point

              elevation = ((float) j
                  * lidar_configuration.grid_elevation_angular_resolution)
                  + lidar_configuration.min_elevation_angle;

              sphericalInDegrees2Cartesian(range, azimuth, elevation, x_c, y_c,
                  z_c);

              index_c = j;

              mod_ab = sqrt(
                  pow((x_b - x_a), 2) + pow((y_b - y_a), 2)
                      + pow((z_b - z_a), 2));

              mod_bc = sqrt(
                  pow((x_c - x_b), 2) + pow((y_c - y_b), 2)
                      + pow((z_c - z_b), 2));

              dot = (x_b - x_a) * (x_c - x_b) + (y_b - y_a) * (y_c - y_b)
                  + (z_b - z_a) * (z_c - z_b);
              dot = dot / (mod_ab * mod_bc);

              if (fabs(dot)
                  > filtering_configuration.min_dot_product_for_ground) // if ground measurement
              {
                //ROS_INFO_STREAM(std::endl << "BC modulus = " << mod_bc);
              } else
              {
                output_grid[i][index_c] = input_grid[i][index_c];
                point.x = x_c;
                point.y = y_c;
                point.z = 0.0;
              }
            } // End of third point found
          } // End of second point found
        } else
        {
          output_grid[i][index_a] = input_grid[i][index_a];
          point.x = x_a;
          point.y = y_a;
          point.z = 0.0;
        }

        j = index_a;

      } // End of first point found
      j--;
    } // End of vertical search
  } // End of horizontal search

  for (register int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i) // We will search in every vertical slice
  {
    azimuth = ((float) i * lidar_configuration.grid_azimuth_angular_resolution)
        + lidar_configuration.min_azimuth_angle;

    // We will check beginning from the lowest points (180 degrees is looking down, 90 front and 0 degrees up)
    register int j = lidar_configuration.num_of_elevation_cells - 1;
    while (j >= 0)
    {
      if (output_grid[i][j] != -1.0)
      {
        range = input_grid[i][j]; // We extract the x and y coordinates
        elevation = ((float) j
            * lidar_configuration.grid_elevation_angular_resolution)
            + lidar_configuration.min_elevation_angle;

        sphericalInDegrees2Cartesian(range, azimuth, elevation, x_a, y_a, z_a);

        point.x = x_a;
        point.y = y_a;
        point.z = z_a;

        perimeter_cloud.points.push_back(point);

        j = -1;
      }
      j--;
    }
  }
  return;
}

void LocalPlanning::freeSpaceMap(pcl::PointCloud<pcl::PointXYZ> &input_cloud,
    local_planning_lib::SensorConfiguration lidar_configuration,
    local_planning_lib::FilteringConfiguration filtering_configuration,
    pcl::PointCloud<pcl::PointXYZ> &output_cloud,
    pcl::PointCloud<pcl::PointXYZ> &perimeter_cloud)
{
  float **input_grid; // To store the poincloud in an ordered matrix using spherical coordinates

  float **output_grid; // This matrix is the output of the ground filtering step
                       // it will contains (hopefully) only obstacle points and is
                       // implemented using the dot product, inspired in the work of
                       // Petrovskaya and Thrun 2009
                       // "Model based vehicle detection and tracking for autonomous urban driving"

  bool error = false;                     // To check problems in dynamic memory

  //std::cout << std::endl << "Starting memory reserve";
  try
  {
    input_grid = new float*[lidar_configuration.num_of_azimuth_cells];
    output_grid = new float*[lidar_configuration.num_of_azimuth_cells];

    for (int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i)
    {
      input_grid[i] = new float[lidar_configuration.num_of_elevation_cells];
      output_grid[i] = new float[lidar_configuration.num_of_elevation_cells];
    }
  } catch (...)
  {
    std::cout << std::endl
        << "Runtime error in CGeometric_Pointcloud_Processing::groundFiltering!!!";
    error = true;
  }
  //std::cout << std::endl << "Reserve done";

  if (!error)
  {
    pointCloud2SphericalGrid(input_cloud, lidar_configuration,
        filtering_configuration, input_grid);

    freeSpaceMap(input_grid, lidar_configuration, filtering_configuration,
        output_grid, perimeter_cloud);

    sphericalGridToPointCloud(output_grid, lidar_configuration, output_cloud);

    //std::cout << std::endl << "Starting freeing";
    for (int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i)
    {
      delete[] input_grid[i];
      delete[] output_grid[i];
    }
    //std::cout << std::endl << "First freeing done";

    delete[] input_grid;
    delete[] output_grid;
    //std::cout << std::endl << "Second freeing done";
  }

  return;
}

void LocalPlanning::groundSegmentation(
    pcl::PointCloud<pcl::PointXYZ> &input_cloud,
    local_planning_lib::SensorConfiguration lidar_configuration,
    local_planning_lib::FilteringConfiguration filtering_configuration,
    pcl::PointCloud<pcl::PointXYZ> &output_cloud,
    pcl::PointCloud<pcl::PointXYZ> &obstacles_cloud,
    pcl::PointCloud<pcl::PointXYZ> &limits_cloud)
{

  output_cloud.points.clear();
  obstacles_cloud.points.clear();
  limits_cloud.points.clear();
  pcl::PointXYZ point;
  float plane_ec;
  float distance;
  float var_factor;

  ///////////////////////////////////////////////////////////////////////////
  // naive objects segmentation
  for (size_t i = 0; i < input_cloud.points.size(); ++i)
  {
    point.x = input_cloud.points[i].x;
    point.y = input_cloud.points[i].y;
    point.z = input_cloud.points[i].z;

    distance = sqrt(point.x * point.x + point.y * point.y);

    var_factor = distance * filtering_configuration.var_factor;

    if (filtering_configuration.a != 0.0 && filtering_configuration.b != 0.0
        && filtering_configuration.c != 0.0
        && point.z < filtering_configuration.variance * var_factor
        && distance < filtering_configuration.radious)
    {
      plane_ec = point.x / filtering_configuration.a
          + point.y / filtering_configuration.b
          + point.z / filtering_configuration.c;

      plane_ec = plane_ec - 1;

      if (abs(plane_ec) > filtering_configuration.variance * var_factor)
      {
        output_cloud.points.push_back(point);
      }
    }

  }

  ///////////////////////////////////////////////////////////////////////////
  // objects perimeter calculation
  float range = filtering_configuration.radious;
  float azimuth, x, y, z;
  float elevation = 90.0;
  float **obstacles_grid;
  bool error = false;
  bool no_obstacle = false;
  try
  {
    obstacles_grid = new float*[lidar_configuration.num_of_azimuth_cells];
    for (int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i)
    {
      obstacles_grid[i] = new float[lidar_configuration.num_of_elevation_cells];
    }
  } catch (...)
  {
    std::cout << std::endl
        << "Runtime error in CGeometric_Pointcloud_Processing::groundFiltering!!!";
    error = true;
  }

  if (!error)
  {
    pointCloud2SphericalGrid(output_cloud, lidar_configuration,
        filtering_configuration, obstacles_grid);

    for (register int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i) // We will search in every vertical slice
    {

      if (no_obstacle)
      {
        range = filtering_configuration.radious;
        sphericalInDegrees2Cartesian(range, azimuth, 90.0, x, y, z);
        point.x = x;
        point.y = y;
        point.z = 0.0;
        limits_cloud.points.push_back(point);
      }

      azimuth =
          ((float) i * lidar_configuration.grid_azimuth_angular_resolution)
              + lidar_configuration.min_azimuth_angle;

      // We will check beginning from the lowest points (180 degrees is looking down, 90 front and 0 degrees up)
      register int j = lidar_configuration.num_of_elevation_cells - 1;
      no_obstacle = true;
      while (j >= 0)
      {
        if (obstacles_grid[i][j] != -1.0)
        {
          range = obstacles_grid[i][j]; // We extract the x and y coordinates
          elevation = ((float) j
              * lidar_configuration.grid_elevation_angular_resolution)
              + lidar_configuration.min_elevation_angle;

          sphericalInDegrees2Cartesian(range, azimuth, elevation, x, y, z);

          point.x = x;
          point.y = y;
          point.z = 0.0;

          obstacles_cloud.points.push_back(point);
          no_obstacle = false;

          j = -1;
        }
        j--;
      }
    }

    for (int i = 0; i < lidar_configuration.num_of_azimuth_cells; ++i)
    {
      delete[] obstacles_grid[i];
    }
    delete[] obstacles_grid;
  }

  return;
}

void LocalPlanning::localGoalCalculation(pcl::PointXYZ global_goal,
    pcl::PointCloud<pcl::PointXYZ> obstacles_cloud,
    pcl::PointCloud<pcl::PointXYZ> limits_cloud, pcl::PointXYZ &local_goal)
{
  local_goal.x = 0.0;
  local_goal.y = 0.0;
  local_goal.z = 0.0;
  float min_distance = 10000.0;
  float distance;
  float x, y;

  for (int i = 0; i < limits_cloud.points.size(); i++){
    x = limits_cloud.points[i].x - global_goal.x;
    y = limits_cloud.points[i].y - global_goal.y;
    distance = sqrt(x*x + y*y);

    if (distance < min_distance){
      min_distance = distance;
      local_goal.x = limits_cloud.points[i].x;
      local_goal.y = limits_cloud.points[i].y;
    }
  }
  return;
}

