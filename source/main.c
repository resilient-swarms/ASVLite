#include <stdio.h>
#include <string.h>
#include <time.h>
#include "world.h"
#include "asv.h"

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

  // Start simulation
  fprintf(stdout, "Star simulation: \n");
  
  double frame_length = 10.0; // time duration of each frame in milli-seconds 
  double duration = 1200.0; // time duration of animation.
  fprintf(stdout, "--> frame duration = %f milli_seconds. \n", frame_length);
  fprintf(stdout, "--> simulation duration = %f seconds. \n", duration);
  
  fprintf(fp, "#[1]time  "
               "[2]wave_elevation  " 
               "[3]cog_x  "
               "[4]cog_y  "
               "[5]cog_z  "
               "[6]heel  "
               "[7]trim  "
               "[8]heading \n");
  clock_t start, end;
  for(double t = 0.0; t < duration; t += (frame_length/1000.0))
  {
    start = clock();
    double wave_elevation = 0.0;
    if(world.wave)
    {  
      wave_elevation = wave_get_elevation(world.wave, 
                                          &world.asv.cog_position, 
                                          t);
    }
    world_set_frame(&world, t);
    fprintf(fp, "%f %f %f %f %f %f %f %f \n", 
            t, 
            wave_elevation,
            world.asv.cog_position.x, 
            world.asv.cog_position.y, 
            world.asv.cog_position.z, 
            world.asv.attitude.heel, 
            world.asv.attitude.trim, 
            world.asv.attitude.heading); 
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
