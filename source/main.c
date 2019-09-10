#include <stdio.h>
#include <string.h>
#include <time.h>
#include "world.h"
#include "asv.h"
#include "constants.h"
#include "pid_controller.h"

#define BUFFER_SIZE 50000

/**
 * A simple struct to record the simulated data for each time step of 
 * simulation.
 */
struct Simulation_data
{
  double time; // sec.
  double wave_elevation; // Wave elevation at the position of the vehicle, m.
  double cog_x;   // m.
  double cog_y;   // m.
  double cog_z;   // m.
  double heel;    // deg.
  double trim;    // deg. 
  double heading; // deg.
  double thrust_fore_ps; // N.
  double thrust_fore_sb; // N.
  double thrust_aft_ps;  // N.
  double thrust_aft_sb;  // N.
}simulation_data[BUFFER_SIZE];

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    fprintf(stderr, "Error. Usage: %s input_file.xml.\n", argv[0]);
    return 1;
  }

  // Init world
  struct World world;
  world_init(&world, argv[1]);

  // Init clock.
  double time_step_size = 10.0; // time step size in milli-seconds 
  double run_time = 0.0; // initialise time
  int t = 0; // counter for time steps
  clock_t start, end;

  // Propeller orientations are fixed.
  struct Attitude propeller_orientation = (struct Attitude){0.0,0.0,0.0};

  // Set destination
  struct Point destination = (struct Point){100.0, 100.0, 0.0};
  
  // Start simulation
  for(double h = 0.0; h <= 15; h=h+0.5)
  {
    // Reset time for each simulation
    run_time = 0.0; 
    t = 0; 

    // Reset the world
    if(h != 0.0)
    {
      if(world.wave == NULL)
      {
        world.wave = (struct Wave*)malloc(sizeof(struct Wave));
      }
      wave_init_with_sig_wave_ht(world.wave, h, 0.0);
      asv_init(&world.asv, 
               world.asv.spec, 
               world.wave, 
               world.wind, 
               world.current);
    }

    fprintf(stdout, 
      "\nStar simulation for significant wave height of {%f} m. \n", h);
    fprintf(stdout, "--> time step size = %f milli_seconds. \n", 
            time_step_size);
    
    // Start clock to measure time.
    start = clock();
    while(world.asv.cog_position.x < destination.x)
    {
      // Get the wave elevation if wave is simulated.
      double wave_elevation = 0.0;
      if(world.wave)
      {  
        wave_elevation = wave_get_elevation(world.wave, 
                                            &world.asv.cog_position, 
                                            run_time);
      }

      // Set the propeller thrust and orientation.
      asv_propeller_set_thrust(&world.asv.propellers[0], 
                               5.0,
                               propeller_orientation);
      asv_propeller_set_thrust(&world.asv.propellers[1], 
                               5.0,
                               propeller_orientation);
      asv_propeller_set_thrust(&world.asv.propellers[2], 
                               5.0,
                               propeller_orientation);
      asv_propeller_set_thrust(&world.asv.propellers[3], 
                               5.0,
                               propeller_orientation);

      // Get the asv dynamics for the current time step.
      world_set_frame(&world, run_time);

      // Record the simulated data.
      if(t < BUFFER_SIZE)
      {
        run_time = (t*time_step_size)/1000.0; // seconds
        simulation_data[t].time = run_time;
        simulation_data[t].wave_elevation = wave_elevation;
        simulation_data[t].cog_x = world.asv.cog_position.x;
        simulation_data[t].cog_y = world.asv.cog_position.y;
        simulation_data[t].cog_z = world.asv.cog_position.z -
                                  (world.asv.spec.cog.z - world.asv.spec.T);
        simulation_data[t].heel = world.asv.attitude.heel * 180.0/PI;
        simulation_data[t].trim = world.asv.attitude.trim * 180.0/PI;
        simulation_data[t].heel = world.asv.attitude.heading * 180.0/PI;
        ++t;
      }
      else
      {
        fprintf(stderr, "Error: no buffer space for simulated data. \n");
        break;
        //return 0;
      }     
    }
    // Stop clock.
    end = clock();

    // Record the results.
    // Open output text file. 
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
    sprintf(out_file+strlen(in_file)-4, "_%.2f.txt", h);
    FILE* fp;
    if(!(fp = fopen(out_file, "w")))
    {
      fprintf(stderr, "Error. Cannot open output file %s.\n", out_file);
      return 1;
    }
    // Print the results to the file
    fprintf(fp, "# task duration = %f seconds.\n", run_time);
    fprintf(fp, "# time taken for simulation = %f sec. \n", 
            ((double)(end - start)) / CLOCKS_PER_SEC);
    
    fprintf(fp, "#[01]time(sec)  "
               "[02]wave_elevation(m)  " 
               "[03]cog_x(m)  "
               "[04]cog_y(m)  "
               "[05]cog_z(m)  "
               "[06]heel(deg)  "
               "[07]trim(deg)  "
               "[08]heading(deg) " 
               "\n");
    for(int i = 0; i < t; ++i)
    {
      fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f \n", 
              simulation_data[i].time,
              simulation_data[i].wave_elevation,
              simulation_data[i].cog_x, 
              simulation_data[i].cog_y, 
              simulation_data[i].cog_z, 
              simulation_data[i].heel, 
              simulation_data[i].trim, 
              simulation_data[i].heading);
    }

    fprintf(stdout, "--> task duration = %f seconds. \n", run_time);
    fprintf(stdout, "--> time taken for simulation = %f sec. \n", 
            ((double)(end - start)) / CLOCKS_PER_SEC);
    fprintf(stdout, "--> simulation data written to file %s. \n", 
            out_file);
    fclose(fp);
  }
  
  world_clean(&world);
  fprintf(stdout, "End simulation. \n");

  return 0;
}
