#include <stdio.h>
#include <string.h>
#include <time.h>
#include "world.h"
#include "asv.h"
#include "constants.h"
#include "pid_controller.h"

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    fprintf(stderr, "Error. Usage: %s input_file.xml.\n", argv[0]);
    return 1;
  }

  struct World world;
  world_init(&world, argv[1]);

  // Open output file to print results
  char* in_file = argv[1];
  char out_file[120];
  if(strlen(in_file) > 114)
  {
    fprintf(stderr, "Error. Filename too long. Cannot create output file.\n");
    return 1;
  }
  for(int i = 0; i<strlen(in_file)-4; ++i)
  {
    out_file[i] = in_file[i];
  }
  strcpy(out_file+strlen(in_file)-4, "_out.txt");
  FILE* fp;
  if(!(fp = fopen(out_file, "w")))
  {
    fprintf(stderr, "Error. Cannot open output file %s.\n", out_file);
    return 1;
  }

  // Initialise the PID controller
  struct PID_controller controller;
  pid_controller_init(&controller);
  struct Point way_point = (struct Point){0.0, 100.0, 0};
  pid_controller_set_way_point(&controller, way_point);
  controller.kp_heading = 1.0 * 0.01;
  controller.ki_heading = 1.0 * 0.01;
  controller.kd_heading = 1.0 * 0.01;
  controller.kp_position = 1.0* 0.01;
  controller.ki_position = 1.0* 0.01;
  controller.kd_position = 1.0* 0.01;

  // Start simulation
  fprintf(stdout, "Star simulation: \n");
  
  double frame_length = 10.0; // time duration of each frame in milli-seconds 
  double duration = 1200.0; // time duration of animation in seconds.
  fprintf(stdout, "--> frame duration = %f milli_seconds. \n", frame_length);
  fprintf(stdout, "--> simulation duration = %f seconds. \n", duration);
  
  fprintf(fp, "#[01]time(sec)  "
               "[02]wave_elevation(m)  " 
               "[03]cog_x(m)  "
               "[04]cog_y(m)  "
               "[05]cog_z(m)  "
               "[06]heel(deg)  "
               "[07]trim(deg)  "
               "[08]heading(deg) " 
               "[09]thrust_fore_ps(N) "
               "[10]thrust_fore_sb(N) "
               "[11]thrust_aft_ps(N)  "
               "[12]thrust_aft_sb(N)  "
               "\n");
  clock_t start, end;
  for(double t = 0.0; t < duration; t += (frame_length/1000.0))
  {
    // Start clock to measure time for each simulation step.
    start = clock();

    // Get the wave elevation if wave is simulated.
    double wave_elevation = 0.0;
    if(world.wave)
    {  
      wave_elevation = wave_get_elevation(world.wave, 
                                          &world.asv.cog_position, 
                                          t);
    }

    // Set the propeller thrust and orientation.
    struct Attitude propeller_orientation = (struct Attitude){0.0,0.0,0.0};
    pid_controller_set_current_state(&controller, 
                                   world.asv.cog_position,
                                   world.asv.attitude);
    pid_controller_set_thrust(&controller);
    asv_propeller_set_thrust(&world.asv.propellers[0], 
                             controller.thrust_fore_ps,
                             propeller_orientation);
    asv_propeller_set_thrust(&world.asv.propellers[1], 
                             controller.thrust_fore_sb,
                             propeller_orientation);
    asv_propeller_set_thrust(&world.asv.propellers[2], 
                             controller.thrust_aft_ps,
                             propeller_orientation);
    asv_propeller_set_thrust(&world.asv.propellers[3], 
                             controller.thrust_aft_sb,
                             propeller_orientation);

    // Get the asv dynamics for the current time step.
    world_set_frame(&world, t);

    // Print the results.
    fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f \n", 
            t, 
            wave_elevation,
            world.asv.cog_position.x, 
            world.asv.cog_position.y, 
            world.asv.cog_position.z -(world.asv.spec.cog.z - world.asv.spec.T), 
            world.asv.attitude.heel * 180.0/PI, 
            world.asv.attitude.trim * 180.0/PI, 
            world.asv.attitude.heading * 180.0/PI,
            controller.thrust_fore_ps, 
            controller.thrust_fore_sb,
            controller.thrust_aft_ps,
            controller.thrust_aft_sb); 

    // Stop clock.
    end = clock();
  }
  fprintf(stdout, "--> time taken per simulation cycle = %f milli-sec. \n", 
          ((double)(end - start)) / CLOCKS_PER_SEC * 1000);
  fprintf(stdout, "--> simulation data written to file %s. \n", 
          out_file);
  fclose(fp);
  
  fprintf(stdout, "End simulation. \n");

  world_clean(&world);

  return 0;
}
