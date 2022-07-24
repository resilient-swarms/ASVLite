#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid_controller.h"
#include "simulation.h"
#include "asv.h"
#include "wave.h"
#include "geometry.h"
#include "constants.h"
#include "errors.h"

struct Controller
{
  // Inputs
  // ------
  struct Asv* asv;
  double kp_heading;
  double ki_heading;
  double kd_heading;
  double kp_position;
  double ki_position;
  double kd_position;
   
  // Intermediate calculation variables
  // ----------------------------------
  double error_heading;
  double error_int_heading;
  double error_diff_heading;
  double error_position;
  double error_int_position;
  double error_diff_position;
  char* error_msg;
};

struct Controller* controller_new(struct Asv* asv)
{
  struct Controller* controller = (struct Controller*)malloc(sizeof(struct Controller));
  if(controller)
  {
    controller->asv                 = asv;
    controller->kp_heading          = 0.0;
    controller->ki_heading          = 0.0;
    controller->kd_heading          = 0.0;
    controller->kp_position         = 0.0;
    controller->ki_position         = 0.0;
    controller->kd_position         = 0.0;
    controller->error_heading       = 0.0;
    controller->error_int_heading   = 0.0;
    controller->error_diff_heading  = 0.0;
    controller->error_position      = 0.0;
    controller->error_int_position  = 0.0;
    controller->error_diff_position = 0.0;
    controller->error_msg           = NULL;
  }
  return controller;
}

void controller_delete(struct Controller* controller)
{
  if(controller)
  {
    free(controller->error_msg);
    free(controller);
    controller = NULL;
  }
}

const char* controller_get_error_msg(const struct Controller* controller)
{
  if(controller)
  {
    return controller->error_msg;
  }
  return NULL;
}

void controller_set_gains_position(struct Controller* controller, 
                                      double p, double i, double d)
{
  clear_error_msg(controller->error_msg);
  if(controller)
  {
    controller->kp_position = p;
    controller->ki_position = i;
    controller->kd_position = d;
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}

void controller_set_gains_heading(struct Controller* controller, 
                                      double p, double i, double d)
{
  clear_error_msg(controller->error_msg);
  if(controller)
  {
    controller->kp_heading = p;
    controller->ki_heading = i;
    controller->kd_heading = d;
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}

void controller_set_thrust(struct Controller* controller, union Coordinates_3D way_point)
{
  clear_error_msg(controller->error_msg);
  if(controller)
  {
    union Coordinates_3D p1 = asv_get_position_origin(controller->asv);
    union Coordinates_3D p2 = asv_get_position_cog(controller->asv);
    union Coordinates_3D p3 = way_point;

    const double limit_error_magnitude = PI; 
    
    // Calculate the heading error in radian.
    // Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
    double m1 = (p2.keys.y == p1.keys.y)? __DBL_MAX__ : (p2.keys.x - p1.keys.x)/(p2.keys.y - p1.keys.y);
    double m2 = (p3.keys.y == p1.keys.y)? __DBL_MAX__ : (p3.keys.x - p1.keys.x)/(p3.keys.y - p1.keys.y);
    double error_heading = atan((m2-m1)/(1+ m1*m2)); // radians
    // Calculate the integral heading error.
    double gamma_heading_error = 0.9; // Should be in the range (0,1).
                                      // Value = 1, implies the past error is never forgotten.
                                      // Value = 0, implies the past error is always ignored. 
                                      // Value between 0 and 1 implies the past errors gradually decreases. 
                                      // Value > 0 implies past errors are magnified.  
    controller->error_int_heading = error_heading + gamma_heading_error * controller->error_int_heading;
    // Calculate the differential heading error.
    controller->error_diff_heading = error_heading - controller->error_heading;
    controller->error_heading = error_heading; 

    // Equation of a line passing through the origin and perpendicular to the longitudinal axis of the asv:
    // Slope of the longitudinal axis of the asv in the global frame
    m1 = (p2.keys.x - p1.keys.x != 0.0)? (p2.keys.y - p1.keys.y)/(p2.keys.x - p1.keys.x) : __DBL_MAX__;
    // Slope of line perpendicular to the longitudinal axis
    m2 = (m1 != 0.0)? -1.0/m1 : __DBL_MAX__;
    // Equation of line with slope m is y = mx + c
    // Compute c for line passing through p1 and slope m2
    double c = p1.keys.y - m2*p1.keys.y;

    // Calculate the position error
    double error_position = sqrt(pow(p3.keys.x - p1.keys.x, 2.0) + pow(p3.keys.y - p1.keys.y, 2.0));
    // Set -ve magnitude for error_position if waypoint is behind the asv.
    // Check if waypoint is behind the vehicle.
    // A point is above a line if y - mx -c is +ve, 
    // A point is below a line if y - mx -c is -ve, and
    // A point is on the line if y - mx -c is 0
    // Check if the both the cog and waypoint are on either sides of the line 
    if((p3.keys.y - m2*p3.keys.x - c)*(p2.keys.y - m2*p2.keys.x - c) < 0.0)
    {
      error_position = -error_position;
    }
    // Calculate the integral error for position.
    double gamma_position_error = 0.9; // Should be in the range (0,1).
                                      // Value = 1, implies the past error is never forgotten.
                                      // Value = 0, implies the past error is always ignored. 
                                      // Value between 0 and 1 implies the past errors gradually decreases. 
                                      // Value > 0 implies past errors are magnified. 
    controller->error_int_position = error_position + gamma_position_error * controller->error_int_position;
    // Calculate the differential error for position.
    controller->error_diff_position = error_position - controller->error_position;
    controller->error_position = error_position;
  
    // Calculate thruster thrust.
    double max_thrust = 5.0; // SMARTY platform thruster has a maximum capacity of 5N. 

    double heading_thrust = 
      controller->kp_heading * controller->error_heading      + 
      controller->ki_heading * controller->error_int_heading  + 
      controller->kd_heading * controller->error_diff_heading; 
    // Do not use more than 20% of thruster capacity for heading correction.
    // heading_thrust = (fabs(heading_thrust) > 0.2 * max_thrust)? max_thrust * 0.2 : heading_thrust;

    double position_thrust= 
      controller->kp_position * controller->error_position      +
      controller->ki_position * controller->error_int_position  + 
      controller->kd_position * controller->error_diff_position;
    
    double thrust_ps = position_thrust + heading_thrust; // left side thrust
    double thrust_sb = position_thrust - heading_thrust; // right side thrust

    double max_value = (fabs(thrust_ps) > fabs(thrust_sb))? fabs(thrust_ps) : fabs(thrust_sb);
    if(max_value > max_thrust)
    {
      double ratio = max_thrust / max_value;
      thrust_ps = thrust_ps * ratio;
      thrust_sb = thrust_sb * ratio;
    } 

    // Set thruster thrust on each of the 4 thrusters.
    union Coordinates_3D orientation_fore_thrusters = {0.0, PI, 0.0};
    union Coordinates_3D orientation_aft_thrusters  = {0.0, 0.0, 0.0};
    struct Thruster** thrusters = asv_get_thrusters(controller->asv);
    // Thruster configuration
    //
    // Thust direction is towards aft
    //  |              |
    //  v              v
    //
    // Fore PS        Fore SB
    // [0] -----------[1] 
    //  +              +
    //  |              |
    //  |              |
    //  +              +
    // [2] -----------[3] 
    // Aft PS         Aft SB
    // 
    //  ^              ^
    //  |              |
    // Thrust direction is towards fore.     
    //    
    if(thrust_ps >= 0.0)
    {
      thruster_set_thrust(thrusters[2], orientation_aft_thrusters, thrust_ps);
      thruster_set_thrust(thrusters[0], orientation_fore_thrusters, 0.0);
    }
    else
    {
      thruster_set_thrust(thrusters[2], orientation_aft_thrusters, 0.0);
      thruster_set_thrust(thrusters[0], orientation_fore_thrusters, thrust_ps);
    }
    if(thrust_sb >= 0.0)
    {
      thruster_set_thrust(thrusters[3], orientation_aft_thrusters, thrust_sb);
      thruster_set_thrust(thrusters[1], orientation_fore_thrusters, 0.0);
    }
    else
    {
      thruster_set_thrust(thrusters[3], orientation_aft_thrusters, 0.0);
      thruster_set_thrust(thrusters[1], orientation_fore_thrusters, thrust_sb);
    }
  }
  else
  {
    set_error_msg(controller->error_msg, error_null_pointer);
  }
}

static double simulate_for_tunning(struct Asv* asv, double* k_position, double* k_heading)
{
  double error = -1.0;
  if(asv)
  {
    double min_significant_wave_height = 1.0; // m
    double max_significant_wave_height = 1.0; // m
    double delta_significant_wave_height = 1.0; // m
    int count_significant_wave_heights = (max_significant_wave_height - min_significant_wave_height)/delta_significant_wave_height + 1;
    double delta_asv_heading = PI/4.0;
    int count_asv_headings = (2.0*PI)/delta_asv_heading;
    int count_asvs = count_significant_wave_heights * count_asv_headings;
    int count_thrusters = asv_get_count_thrusters(asv);
    struct Asv** asvs = (struct Asv**)malloc(sizeof(struct Asv*) * count_asvs);
    union Coordinates_3D start_point;
    start_point.keys.x = 1000.0;
    start_point.keys.y = 1000.0;
    start_point.keys.z = 0.0;
    union Coordinates_3D waypoint;
    waypoint.keys.x = 1000.0;
    waypoint.keys.y = 5000.0;
    waypoint.keys.z = 0.0;

    int j = 0;
    for(double significant_wave_height = min_significant_wave_height; 
        significant_wave_height <= max_significant_wave_height; 
        significant_wave_height += delta_significant_wave_height)
    {
      for(double asv_heading = 0.0; asv_heading < 2.0*PI; asv_heading += PI/4.0)
      {
        // Create the sea surface.
        double wave_heading = 0.0;
        int rand_seed = 1;
        int count_wave_spectral_directions  = 5;
        int count_wave_spectral_frequencies = 15;
        struct Wave* wave = wave_new(significant_wave_height, wave_heading, rand_seed, count_wave_spectral_directions, count_wave_spectral_frequencies);
        // Create the thrusters for the ASV by copying data from the existing ASV. 
        struct Thruster** thrusters = asv_get_thrusters(asv);
        struct Thruster** new_thrusters = (struct Thruster**)malloc(sizeof(struct Thruster*)*count_thrusters);
        for(int i = 0; i < count_thrusters; ++i)
        {
          union Coordinates_3D position = thruster_get_position(thrusters[i]);
          new_thrusters[i] = thruster_new(position);
        }
        // Create ASV
        union Coordinates_3D start_attitude;
        start_attitude.keys.x = 0;
        start_attitude.keys.y = 0;
        start_attitude.keys.z = asv_heading;        
        struct Asv_specification spec = asv_get_spec(asv);
        struct Asv* new_asv = asv_new(spec, wave, start_point, start_attitude);
        asv_set_thrusters(new_asv, new_thrusters, count_thrusters);
        free(new_thrusters);
        asvs[j++] = new_asv;
      }
    }

    // Create simulation
    struct Simulation* simulation = simulation_new();
    bool time_sync = false;
    simulation_set_input_using_asvs(simulation, asvs, count_asvs, time_sync);
    // Set the waypoints for all asvs
    for(int i = 0; i < count_asvs; ++i)
    {
      int count_waypoints = 1;
      simulation_set_waypoints_for_asv(simulation, asvs[i], &waypoint, count_waypoints);
      simulation_set_controller(simulation, k_position, k_heading);
    }
    
    // Run simulation for a set period of time.
    double max_time = 200.0; // seconds
    simulation_run_upto_time(simulation, max_time);

    // Compute error
    double sum_error = 0.0;
    for(int i = 0; i < count_asvs; ++i)
    {
      // Calculate the sum of offsets of the asv position from the straight line joining 
      // start and end point.
      struct Buffer* buffer = simulation_get_buffer(simulation, asvs[i]);
      long buffer_length = simulation_get_buffer_length(simulation, asvs[i]);
      double sum_offsets = 0.0;
      for(int j = 0; j < buffer_length; ++j)
      {
        union Coordinates_3D p1 = start_point;
        union Coordinates_3D p2 = waypoint;
        union Coordinates_3D p0 = buffer_get_asv_position_at(buffer, j);
        // Distance between a point p0 to a line joining p1 and p2 is:
        // distance = abs((x2-x1)(y1-y0) - (x1-x0)(y2-y1))/sqrt((x2-x1)^2 + (y2-y1)^2) 
        double numerator = fabs((p2.keys.x - p1.keys.x)*(p1.keys.y - p0.keys.y) - (p1.keys.x - p0.keys.x)*(p2.keys.y - p1.keys.y));
        double denominator = sqrt((p2.keys.x-p1.keys.x)*(p2.keys.x-p1.keys.x) + (p2.keys.y-p1.keys.y)*(p2.keys.y-p1.keys.y));
        double offset = numerator/denominator;
        sum_offsets += offset;
      }
      // Calculate the distance of the asv from the waypoint
      union Coordinates_3D p1 = waypoint;
      union Coordinates_3D p2 = asv_get_position_cog(asvs[i]);
      double distance = sqrt((p1.keys.x-p2.keys.x)*(p1.keys.x-p2.keys.x) + (p1.keys.y-p2.keys.y)*(p1.keys.y-p2.keys.y));
      // Total error = sum of offsets + distance to waypoint
      sum_error += distance + sum_offsets;
      // sum_error += distance;
    }
    error = sum_error/count_asvs;

    // Clean memory
    for(int i = 0; i < count_asvs; ++i)
    {
      struct Thruster** thrusters = asv_get_thrusters(asvs[i]);
      for(int j = 0; j < count_thrusters; ++j)
      {
        thruster_delete(thrusters[j]);
      }
    }
    simulation_delete(simulation);
    free(asvs);
  }
  return error;
}

static int compute_and_set_average_costs(double** costs, int index, double* average_costs, double* k)
{
  // Find average cost for each case of k-delta, k, k+delta
  for(int i = 0; i < 3; ++i)
  {
    double sum_cost = 0.0;
    double count = 0.0; 
    for(int j = 0; j < pow(3, 6); ++j)
    {
      if(costs[j][index] == k[i])
      {
        sum_cost += costs[j][6];
        count += 1.0;
      }
    }
    average_costs[i] = sum_cost/count;
  }
  // Find the min of the 3 averages
  double min = __DBL_MAX__;
  int min_index = -1;
  for(int i = 0; i < 3; ++i)
  {
    if(average_costs[i] < min)
    {
      min = average_costs[i];
      min_index = i;
    }
  }
  return min_index;
}

void controller_tune(struct Controller* controller)
{
  clear_error_msg(controller->error_msg);
  // Open file to write
  char* file = "./tunning";
  FILE *fp = fopen(file, "w");
  if (fp == 0)
  {
    fprintf(stderr, "ERROR: cannot open file \"%s\".\n", file);
    exit(1);
  }
  fprintf(fp,  
            "position_p "
            "position_i "
            "position_d "
            "heading_p "
            "heading_i "
            "heading_d "
            "cost ");
  // Initialise gain terms
  double k_position[3] = {1.0, 1.0, 1.0};
  double k_heading[3]  = {1.0, 1.0, 1.0};
  double delta = 0.5;
  int count_iterations = 100;
  for(int i = 0; i < count_iterations; ++i)
  {
    double p_position[3] = {k_position[0]-delta, k_position[0], k_position[0]+delta}; 
    double i_position[3] = {k_position[1]-delta, k_position[1], k_position[1]+delta};
    double d_position[3] = {k_position[2]-delta, k_position[2], k_position[2]+delta};
    double p_heading[3] = {k_heading[0]-delta, k_heading[0], k_heading[0]+delta}; 
    double i_heading[3] = {k_heading[1]-delta, k_heading[1], k_heading[1]+delta};
    double d_heading[3] = {k_heading[2]-delta, k_heading[2], k_heading[2]+delta};
    double** costs = (double**)malloc(sizeof(double*)* pow(3, 6)); // This is a table of 7 columns and 3^6 rows.
    int i = 0;
    for(int p_p = 0; p_p < 3; ++p_p)
    {
      for(int p_i = 0; p_i < 3; ++p_i)
      {
        for(int p_d = 0; p_d < 3; ++p_d)
        {
          for(int h_p = 0; h_p < 3; ++h_p)
          {
            for(int h_i = 0; h_i < 3; ++h_i)
            {
              for(int h_d = 0; h_d < 3; ++h_d)
              {
                double* row = (double*)malloc(sizeof(double)*7);
                row[0] = p_position[p_p];
                row[1] = i_position[p_i];
                row[2] = d_position[p_d];
                row[3] = p_heading[h_p];
                row[4] = i_heading[h_i];
                row[5] = d_heading[h_d];
                row[6] = simulate_for_tunning(controller->asv, row, row+3);
                costs[i++] = row;
              }
            }
          }
        }
      }
    }

    // Write to file 
    double average_cost_for_current_ks = -1.0;
    for(int i = 0; i < pow(3, 6); ++i)
    {
      if(costs[i][0] == k_position[0] && 
         costs[i][1] == k_position[1] &&
         costs[i][2] == k_position[2] &&
         costs[i][3] == k_heading[0] && 
         costs[i][4] == k_heading[1] &&
         costs[i][5] == k_heading[2])
      {
        average_cost_for_current_ks = costs[i][6];
      }
    }
    fprintf(fp, "\n%f %f %f %f %f %f %f", 
            k_position[0],
            k_position[1],
            k_position[2],
            k_heading[0],
            k_heading[1],
            k_heading[2],
            average_cost_for_current_ks);
    fprintf(stdout, "\n%f %f %f %f %f %f %f", 
            k_position[0],
            k_position[1],
            k_position[2],
            k_heading[0],
            k_heading[1],
            k_heading[2],
            average_cost_for_current_ks);

    // (1) Set the new gain terms using method of average 
    // ------
    // double average_costs_p_position[3]; // [k-delta, k, k+delta]
    // double average_costs_i_position[3];
    // double average_costs_d_position[3];
    // double average_costs_p_heading[3]; 
    // double average_costs_i_heading[3];
    // double average_costs_d_heading[3];
    // // Find average cost for each case
    // int min_index_position_p = compute_and_set_average_costs(costs, 0, average_costs_p_position, p_position);
    // int min_index_position_i = compute_and_set_average_costs(costs, 1, average_costs_i_position, i_position);
    // int min_index_position_d = compute_and_set_average_costs(costs, 2, average_costs_d_position, d_position);
    // int min_index_heading_p = compute_and_set_average_costs(costs, 3, average_costs_p_heading, p_heading);
    // int min_index_heading_i = compute_and_set_average_costs(costs, 4, average_costs_i_heading, i_heading);
    // int min_index_heading_d = compute_and_set_average_costs(costs, 5, average_costs_d_heading, d_heading);
    // Set the new gain terms
    // k_position[0] = p_position[min_index_position_p];
    // k_position[1] = i_position[min_index_position_i];
    // k_position[2] = d_position[min_index_position_d];
    // k_heading[0]  = p_heading[min_index_heading_p];
    // k_heading[1]  = i_heading[min_index_heading_i];
    // k_heading[2]  = d_heading[min_index_heading_d];
    // ------

    // (2) Set the new gain terms using method of min cost
    // ------
    double min_cost = __DBL_MAX__;
    int min_index = -1;
    for(int i = 0; i < pow(3,6); ++i)
    {
      if(costs[i][6] < min_cost)
      {
        min_cost = costs[i][6];
        min_index = i;
      }
    }
    // Set the new gain terms
    k_position[0] = costs[min_index][0];
    k_position[1] = costs[min_index][1];
    k_position[2] = costs[min_index][2];
    k_heading[0]  = costs[min_index][3];
    k_heading[1]  = costs[min_index][4];
    k_heading[2]  = costs[min_index][5];
    // ------

    // Clean memory
    for(int i = 0; i < pow(3, 6); ++i)
    {
      free(costs[i]);
    }
    free(costs);
  }
  // Set the controller
  controller->kp_position = k_position[0];
  controller->ki_position = k_position[1];
  controller->kd_position = k_position[2];
  controller->kp_heading = k_heading[0];
  controller->ki_heading = k_heading[1];
  controller->kd_heading = k_heading[2];
  fclose(fp);
}

union Coordinates_3D controller_get_gains_position(struct Controller* controller)
{
  union Coordinates_3D k;
  k.keys.x = controller->kp_position;
  k.keys.y = controller->ki_position;
  k.keys.z = controller->kd_position;
  return k;
}

union Coordinates_3D controller_get_gains_heading(struct Controller* controller)
{
  union Coordinates_3D k;
  k.keys.x = controller->kp_heading;
  k.keys.y = controller->ki_heading;
  k.keys.z = controller->kd_heading;
  return k;
}