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
  // 1) Naive objects segmentation
  if (filtering_configuration.is_simulation)
  {
    for (size_t i = 0; i < input_cloud.points.size(); ++i)
    {
      point.x = input_cloud.points[i].x;
      point.y = input_cloud.points[i].y;
      point.z = input_cloud.points[i].z;

      distance = sqrt(point.x * point.x + point.y * point.y);

      if (point.z < 0.0 && point.z > filtering_configuration.ground_in_sim
          && distance < filtering_configuration.radious)
      {
        output_cloud.points.push_back(point);
      }
    }
  } else
  {
    for (size_t i = 0; i < input_cloud.points.size(); ++i)
    {
      point.x = input_cloud.points[i].x;
      point.y = input_cloud.points[i].y;
      point.z = input_cloud.points[i].z;

      distance = sqrt(point.x * point.x + point.y * point.y);

      var_factor = distance * filtering_configuration.var_factor;

      if (filtering_configuration.a != 0.0 && filtering_configuration.b != 0.0
          && filtering_configuration.c != 0.0
          && point.z < filtering_configuration.variance * var_factor // LiDAR horizontal mounting assumption
          && distance < filtering_configuration.radious
          && distance > filtering_configuration.lidar_protect)
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
  }

  ///////////////////////////////////////////////////////////////////////////
  // 2) Objects perimeter calculation
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
    pcl::PointCloud<pcl::PointXYZ> limits_cloud,
    pcl::PointCloud<pcl::PointXYZ> &local_path)
{
  local_path.points.clear();

  pcl::PointXYZ local_goal;
  local_goal.x = 0.0;
  local_goal.y = 0.0;
  local_goal.z = 0.0;
  float distance_a;
  float distance_r;
  float force_a;
  float force_r;
  float force;
  float x, y;

  // TODO: get from param
  float wa = 100.0;
  float wr = 1.0;
  float ar = 0.8;
  float aa = 0.3;

  ///////////////////////////////////////////////////////////////////////////////////
  //// 1) PATH POINT IN RADIOUS LIMIT
  float min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000;
  float min_distance = 10000.0;
  for (int i = 0; i < limits_cloud.points.size(); i += 2)
  {
    // goal weight calculation
    x = limits_cloud.points[i].x - global_goal.x;
    y = limits_cloud.points[i].y - global_goal.y;
    distance_a = sqrt(x * x + y * y);
    if (distance_a < 0.1)
      distance_a = 0.1;

    force_a = wa / pow(distance_a, aa);

    // min obstacle weight calculation
    min_distance = 10000.0;
    for (int j = 0; j < obstacles_cloud.points.size(); j += 2)
    {
      x = limits_cloud.points[i].x - obstacles_cloud.points[j].x;
      y = limits_cloud.points[i].y - obstacles_cloud.points[j].y;
      distance_r = sqrt(x * x + y * y);
      if (distance_r < 0.1)
        distance_r = 0.1;

      if (distance_r < min_distance)
      {
        min_distance = distance_r;
        force_r = wr / pow(distance_r, ar);
      }
    }

    force = force_r - force_a;

    if (force < min_force)
    {
      min_force = force;
      local_goal.x = limits_cloud.points[i].x;
      local_goal.y = limits_cloud.points[i].y;
    }
  }
  local_path.points.push_back(local_goal);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  //// 2) PATH POINT IN INNER RINGS
  // TODO: get from param
  float wa2 = 3.0;
  float wr2 = 1.0;
  float ar2 = 0.8;
  float aa2 = 0.3;
  float radious = 6.0;
  float delta_rad = radious / 3.0;
  float azimuth, elevation, range;
  float x_in, y_in, z_in;
  for (float rad = radious; rad >= delta_rad; rad -= delta_rad)
  {
    min_force = wr / pow(0.1, ar) - wa / pow(100.0, aa) + 1000;
    min_distance = 10000.0;
    for (int i = 0; i < limits_cloud.points.size(); i += 2)
    {
      x = limits_cloud.points[i].x;
      y = limits_cloud.points[i].y;
      cartesian2SphericalInDegrees(x, y, 0.0, range, azimuth, elevation);
      sphericalInDegrees2Cartesian(rad, azimuth, 90.0, x_in, y_in, z_in);

      // goal obstacle weight calculation
      x = x_in - local_path.points[0].x;
      y = y_in - local_path.points[0].y;
      distance_a = sqrt(x * x + y * y);
      if (distance_a < 0.1)
        distance_a = 0.1;

      force_a = wa2 / pow(distance_a, aa2);

      // min obstacle weight calculation
      min_distance = 10000.0;
      for (int j = 0; j < obstacles_cloud.points.size(); j += 2)
      {
        x = x_in - obstacles_cloud.points[j].x;
        y = y_in - obstacles_cloud.points[j].y;
        distance_r = sqrt(x * x + y * y);
        if (distance_r < 0.1)
          distance_r = 0.1;

        if (distance_r < min_distance)
        {
          min_distance = distance_r;
          force_r = wr2 / pow(distance_r, ar2);
        }
      }

      force = force_r - force_a;

      // point selection
      if (force < min_force)
      {
        min_force = force;
        local_goal.x = x_in;
        local_goal.y = y_in;
      }
    }
    local_path.points.push_back(local_goal);
  }
  ///////////////////////////////////////////////////////////////////////////////////

  return;
}

void LocalPlanning::controlActionCalculation(pcl::PointXYZ local_goal,
    pcl::PointXYZ semilocal_goal,
    local_planning_lib::Pose2D base_in_lidarf,
    pcl::PointCloud<pcl::PointXYZ> obstacles_cloud,
    pcl::PointCloud<pcl::PointXYZ> &collision_risk,
    pcl::PointCloud<pcl::PointXYZ> &collision_actions,
    pcl::PointCloud<pcl::PointXYZ> &free_actions,
    local_planning_lib::AckermannControl &ackermann_control)
{
  pcl::PointXYZ point;
  pcl::PointXYZ point_front;
  static pcl::PointCloud<pcl::PointXYZ> action_arc;
  std::vector<float> steering_options;
  std::vector<float> direction_options;
  std::vector<float> error_values;
  bool flag_collision_risk = false;

  collision_risk.points.clear();
  collision_actions.points.clear();
  free_actions.points.clear();

  ackermann_control.steering = 0.0;
  ackermann_control.velocity = 0.0;

  //////////////////////////////////////////////////////////////////////////////////////////
  //// 1) CHECK POSIBLE CONTROL ACTIONS
  for (float st = -1 * ackermann_control.max_angle;
      st <= ackermann_control.max_angle; st += ackermann_control.delta_angle)
  {

    //// CHECK REAR-FRONT ACTIONS
    for (float dir = -1.0; dir <= 1.0; dir += 2.0)
    {
      float steering_radians = st * M_PI / 180.0;
      flag_collision_risk = false;
      action_arc.points.clear();

      //// CALCULATE TRAJECTORY ARC FROM STEERING
      for (float arc = ackermann_control.delta_arc;
          arc <= ackermann_control.max_arc; arc += ackermann_control.delta_arc)
      {
        if (steering_radians != 0.0)
        {
          float r = ackermann_control.v_length / tan(steering_radians);

          float x_center = base_in_lidarf.x - r * sin(base_in_lidarf.yaw);
          float y_center = base_in_lidarf.y - r * cos(base_in_lidarf.yaw);

          float w_current = base_in_lidarf.yaw + (arc * dir) / r;
          float x_current = r * sin(w_current) + x_center;
          float y_current = r * cos(w_current) + y_center;

          point.x = x_current;
          point.y = y_current;
          point.z = w_current;
          action_arc.points.push_back(point);
        } else
        {
          float x_current, y_current, z_current;
          sphericalInDegrees2Cartesian(arc * dir,
              (base_in_lidarf.yaw * 180.0 / M_PI), 90.0, x_current, y_current,
              z_current);

          point.x = base_in_lidarf.x + x_current;
          point.y = base_in_lidarf.y + y_current;
          point.z = base_in_lidarf.yaw;
          action_arc.points.push_back(point);
        }

        float x, y, z;
        sphericalInDegrees2Cartesian(ackermann_control.delta_arc,
            (-point.z * 180.0 / M_PI), 90.0, x, y, z);

        point_front.x = point.x + x;
        point_front.y = point.y + y;
        point_front.z = point.z;

        //// CHECK IF COLLISION RISK
        for (int i = 0; i < obstacles_cloud.points.size(); i++)
        {
          if (obstacles_cloud.points[i].x < point.x + ackermann_control.margin
              && obstacles_cloud.points[i].x
                  > point.x - ackermann_control.margin
              && obstacles_cloud.points[i].y
                  < point.y + ackermann_control.margin
              && obstacles_cloud.points[i].y
                  > point.y - ackermann_control.margin)
          {
            collision_risk.points.push_back(obstacles_cloud.points[i]);
            flag_collision_risk = true;
          }
        }
      }

      if (!flag_collision_risk)
      {
        //// ERROR FUNCTION (TO MINIMIZE) FOR FREE ACTIONS
        float d_front = sqrt(
            pow(local_goal.x - point_front.x, 2)
                + pow(local_goal.y - point_front.y, 2));
        float d_rear = sqrt(
            pow(local_goal.x - point.x, 2) + pow(local_goal.y - point.y, 2));
        float e_ang = d_front - d_rear;
        float e_dist = d_rear;

        // TODO?: get from params?
        float min_x = -1.0;
        float k_dist;
        float k_ang;
        if (local_goal.x < min_x)
        {
          k_dist = 0.1;
          k_ang = 2.0;
        } else
        {
          k_dist = 0.5;
          k_ang = 1.0;
        }

        float error = e_ang * k_ang + e_dist * k_dist;

        steering_options.push_back(-steering_radians);
        direction_options.push_back(dir);
        error_values.push_back(error);

        for (int k = 0; k < action_arc.points.size(); k++)
        {
          action_arc.points[k].z = 0.0;
          free_actions.points.push_back(action_arc.points[k]);
        }
        //free_actions.points.push_back(point_front);
      } else
      {
        for (int k = 0; k < action_arc.points.size(); k++)
        {
          action_arc.points[k].z = 0.0;
          collision_actions.points.push_back(action_arc.points[k]);
        }
        //collision_actions.points.push_back(point_front);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  //// 2) FIND BEST CONTROL ACTION
  float min_error = 1000;
  float k_sp = (ackermann_control.v_max - ackermann_control.v_min)
      / ackermann_control.max_angle;
  for (int i = 0; i < steering_options.size(); i++)
  {
    if (error_values[i] < min_error)
    {
      ackermann_control.steering = steering_options[i];
      ackermann_control.velocity = (ackermann_control.v_max - abs(ackermann_control.steering) * k_sp) * direction_options[i];

      if (ackermann_control.carrot_ctrl){
        float goal_distance = sqrt(pow(semilocal_goal.x, 2) + pow(semilocal_goal.y, 2));

        std::cout << "goal_distance = " << goal_distance << std::endl; //Debug
        std::cout << "carrot_distance = " << ackermann_control.carrot_distance << std::endl; //Debug
        std::cout << "carrot_stop_distance = " << ackermann_control.carrot_stop_distance << std::endl; //Debug

        if (goal_distance < (ackermann_control.carrot_distance+ackermann_control.carrot_stop_distance)){
          ackermann_control.velocity *= (goal_distance-ackermann_control.carrot_stop_distance) / ackermann_control.carrot_distance;
        }
        if (goal_distance < ackermann_control.carrot_stop_distance){ 
          ackermann_control.velocity = 0.0; 
          ackermann_control.steering=0.0;
        }
      }
      
      min_error = error_values[i];
    }
  }

  // DEBUG!!!
  //ackermann_control.steering = 0.0;
  //ackermann_control.velocity = 0.0;

  return;
}

