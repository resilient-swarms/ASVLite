#include <stdio.h>
#include <string.h>
#include <time.h>
#include "asv.h"

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
};

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    fprintf(stderr, "Error. Usage: %s output_file_prefix.\n", argv[0]);
    return 1;
  }

  // vehicle specification
  struct Asv_specification asv_spec;
  asv_spec.L_wl      = 0.3;
  asv_spec.B_wl      = 0.3;
  asv_spec.D         = 0.3;
  asv_spec.T         = 0.1;
  asv_spec.max_speed = 2.0;
  asv_spec.disp      = 0.007;
  asv_spec.r_roll    = 0.08;
  asv_spec.r_pitch   = 0.08;
  asv_spec.r_yaw     = 0.106;
  asv_spec.cog.x     = 0.15;
  asv_spec.cog.y     = 0.0;
  asv_spec.cog.z     = 0.15;

  // set vehicle propellers
  struct Asv_propeller propellers[4];
  asv_propeller_init(&(propellers[0]), (struct Point){1.585, -0.085, -0.125});
  asv_propeller_init(&(propellers[1]), (struct Point){1.585, +0.085, -0.125});
  asv_propeller_init(&(propellers[2]), (struct Point){1.415, -0.085, -0.125});
  asv_propeller_init(&(propellers[3]), (struct Point){1.415, +0.085, -0.125});

  // vehicle to move to destination point
  struct Point destination = (struct Point){0.0, 10.0, 0};

  // start simultion
  for(double h = 0.0; h <= 15.0; h=h+0.5)
  {
    // Open file to print result
    char out_file[120];
    sprintf(out_file, "%s_%.2f.txt", argv[1], h);
    FILE* fp;
    if(!(fp = fopen(out_file, "w")))
    {
      fprintf(stderr, "Error. Cannot open output file %s.\n", out_file);
      return 1;
    }

    // Buffer to store simulation data before writing to file.
    struct Simulation_data buffer[BUFFER_SIZE];

    // init vehicle
    struct Asv asv;
    asv_init(&asv, asv_spec);
    asv_set_propeller(&asv, propellers[0]);
    asv_set_propeller(&asv, propellers[1]);
    asv_set_propeller(&asv, propellers[2]);
    asv_set_propeller(&asv, propellers[3]);

    // init wave
    struct Wave wave;
    double wave_heading = PI;
    if(h != 0.0)
    {
      wave_init(&wave, h, wave_heading);
      asv_set_wave(&asv, &wave);
    }

    // start clock
    double time = 0.0;
    int t = 0;
    double time_step_size = 10.0; //milli-sec
    clock_t start, end;
    start = clock();

    while(asv.cog_position.y <= destination.y)
    {
      // time
      time = t*time_step_size/1000.0; //sec

      // set propeller thrust
      asv_propeller_set_thrust(&(asv.propellers[0]), 
        5.0, (struct Attitude){0.0,0.0,0.0});
      asv_propeller_set_thrust(&(asv.propellers[1]), 
        5.0, (struct Attitude){0.0,0.0,0.0});
      asv_propeller_set_thrust(&(asv.propellers[2]), 
        5.0, (struct Attitude){0.0,0.0,0.0});
      asv_propeller_set_thrust(&(asv.propellers[3]), 
        5.0, (struct Attitude){0.0,0.0,0.0});

      // compute new position and attitude
      asv_set_dynamics(&asv, time);

      // compute wave elevation
      double wave_elevation = 0.0;
      if(h != 0.0)
      {
        wave_elevation = wave_get_elevation(&wave, &asv.cog_position, time);
      }

      // save the buffer
      buffer[t].time = time;
      buffer[t].wave_elevation = wave_elevation;
      buffer[t].cog_x = asv.cog_position.x;
      buffer[t].cog_y = asv.cog_position.y;
      buffer[t].cog_z = asv.cog_position.z - (asv.spec.cog.z - asv.spec.T);
      buffer[t].heel = asv.attitude.heel * 180.0/PI;
      buffer[t].trim = asv.attitude.trim * 180.0/PI;
      buffer[t].heel = asv.attitude.heading * 180.0/PI;

      // increment time
      ++t;
    }

    // end clock
    end = clock();
    double simulation_time = (end - start) / CLOCKS_PER_SEC;

    // Display on screen
    fprintf(stdout, "# significant wave height = %f m.\n", h);
    fprintf(stdout, "# task duration = %f seconds.\n", time);
    fprintf(stdout, "# time taken for simulation = %f sec. \n", 
            ((double)(end - start)) / CLOCKS_PER_SEC);

    // write buffer to file and close the file.
    fprintf(fp, "# significant wave height = %f m.\n", h);
    fprintf(fp, "# task duration = %f seconds.\n", time);
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
    for(int i = 0; i <= t; ++i)
    {
      fprintf(fp, "%f %f %f %f %f %f %f %f \n", 
              buffer[i].time,
              buffer[i].wave_elevation,
              buffer[i].cog_x, 
              buffer[i].cog_y, 
              buffer[i].cog_z, 
              buffer[i].heel, 
              buffer[i].trim, 
              buffer[i].heading);
    }

    fclose(fp);
  }

  return 0;
}
