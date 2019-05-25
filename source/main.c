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
  fprintf(stdout, "START SIMULATION: \n");
  
  double frame_length = 10.0; // time duration of each frame in milli-seconds 
  double duration = 120.0; // time duration of animation.
  fprintf(fp, "time cog_x cog_y cog_z heel trim heading \n");
  clock_t start, end;
  for(double t = 0.0; t < duration; t += (frame_length/1000.0))
  {
    start = clock();
    world_set_frame(&world, t);
    fprintf(fp, "%f %f %f %f %f %f %f \n", 
            t, 
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
  
  fprintf(stdout, "END. \n");

  world_clean(&world);

  return 0;
}
