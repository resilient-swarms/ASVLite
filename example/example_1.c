#include <stdio.h>
#include <constants.h>
#include <geometry.h>
#include <sea_surface.h>
#include <asv.h>

int main(int argc, char** argv)
{ 
  // Initialise the sea state
  double wave_ht = 1.2; // significant wave height (m) for the simulated sea.
  double wave_heading = 20; // predominant wave heading direction measure in
                            // deg with respect to North direction.
  int rand_seed = 3;
  int count_component_waves = 21; // The number of regular component waves in the irregular sea surface. 
  struct Sea_surface* sea_surface = sea_surface_new(wave_ht, wave_heading * PI/180.0, rand_seed, count_component_waves); 

  // Input ASV specification
  struct Asv_specification asv_spec = {
                                          0.3, // L_wl, m
                                          0.3, // B_wl, m
                                          0.21,// D, m
                                          0.11,// T, m
                                          2.0, // max_speed, m/s 
                                          0.007, // disp, m3  
                                          0.08,  // r_roll, m 
                                          0.08,  // r_pitch, m
                                          0.106, // r_yaw, m
                                          {0.15, 0.0, -0.2}}; // COG, {m, m, m} 
  
  // Initialise the ASV
  union Coordinates_3D asv_position = {100, 100, 0};
  union Coordinates_3D asv_attitude = {0, 0, 0};
  struct Asv* asv = asv_new(asv_spec, sea_surface, asv_position, asv_attitude);

  // Initialise thrusters
  union Coordinates_3D position = {0.0, 0.0, 0.0};
  struct Thruster* thruster = thruster_new(position);
  int count_thrusters = 1;
  struct Thruster* thrusters[1] = {thruster};

  // Set the thrusters on the ASV
  asv_set_thrusters(asv, thrusters, count_thrusters);
  
  // Simulate
  double time_step_size = 40; // milli-sec, step size of each simulation step
  for(int t = 0; t < 100; ++t)
  {
    double time = t * time_step_size/ 1000; // sec

    // Set the propeller thrust and direction
    union Coordinates_3D thrust_direction = {0.0, 0.0, 0.0};
    double thrust_magnitude = 0.25; // N
    thruster_set_thrust(thruster, thrust_direction, thrust_magnitude);

    // Compute the new position and attitude of the vehicle
    asv_compute_dynamics(asv, time_step_size);

    // Get the position of the vehicle
    union Coordinates_3D new_position = asv_get_position_cog(asv);
    double x_coordinate = new_position.keys.x; // extract x coordinate
    double y_coordinate = new_position.keys.y; // extract y coordinate
    double z_coordinate = new_position.keys.z; // extract z coordinate
    printf("position = (%f, %f, %f)\n", x_coordinate, y_coordinate, z_coordinate);
    
    // Get the wave elevation at the location of the vehicle.
    double wave_elevation = sea_surface_get_elevation(sea_surface, new_position, time);
  }

  return 0;
}