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
  const int count_way_points = 5;
  struct Point way_points[count_way_points] = 
                               {(struct Point){1000.0, 0.0,    0.0},
                                (struct Point){2000.0, 0.0,    0.0},
                                (struct Point){2000.0, 1000.0, 0.0},
                                (struct Point){1000.0, 1000.0, 0.0},
                                (struct Point){1000.0, 0.0,    0.0}};

  // Initialise simulation time.
  double time_step_size = 10.0; // time for each frame in milli-seconds 
  double run_time = 0.0; // initialise time

  // PID controller set gain terms
  double p_position = 1.0 * time_step_size/1000.0;
  double i_position = 0.0 * time_step_size/1000.0;
  double d_position = 0.0 * time_step_size/1000.0;
  pid_controller_set_gains_position(&controller, 
                                    p_position, i_position, d_position);
  double p_heading = 1.0 * time_step_size/1000.0;
  double i_heading = 0.0 * time_step_size/1000.0;
  double d_heading = 0.0 * time_step_size/1000.0;
  pid_controller_set_gains_heading(&controller, 
                                   p_heading, i_heading, d_heading);
  
  // Start simulation
  fprintf(stdout, "Star simulation: \n");
  
  fprintf(stdout, "--> time step size = %f milli_seconds. \n", time_step_size);
  
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
  for(int i = 0; i < count_way_points; run_time+=(time_step_size/1000.0))
  {
    pid_controller_set_way_point(&controller, way_points[i]);

    // Start clock to measure time for each simulation step.
    start = clock();

    // Get the wave elevation if wave is simulated.
    double wave_elevation = 0.0;
    if(world.wave)
    {  
      wave_elevation = wave_get_elevation(world.wave, 
                                          &world.asv.cog_position, 
                                          run_time);
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
    world_set_frame(&world, run_time);

    // Print the results.
    fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f \n", 
            run_time, 
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

    // If reached a way-point then move on to next
    double x1 = world.asv.cog_position.x;
    double y1 = world.asv.cog_position.y;
    double x2 = way_points[i].x;
    double y2 = way_points[i].y;
    double error = sqrt(pow(x2-x1, 2.0) + pow(y2-y1, 2.0));
    double margin = 0.5; // acceptable error margin in m.
    if(error <= margin)
    {
      fprintf(stdout, "--> reached way-point[%i] (%f m, %f m, %f m). \n", 
              i, way_points[i].x, way_points[i].y, way_points[i].z);
      ++i;
    }
  }

  fprintf(stdout, "--> simulation duration = %f seconds. \n", run_time/1000.0);
  fprintf(stdout, "--> time taken per simulation cycle = %f milli-sec. \n", 
          ((double)(end - start)) / CLOCKS_PER_SEC * 1000);
  fprintf(stdout, "--> simulation data written to file %s. \n", 
          out_file);
  fclose(fp);
  
  fprintf(stdout, "End simulation. \n");

  world_clean(&world);

  return 0;
}
