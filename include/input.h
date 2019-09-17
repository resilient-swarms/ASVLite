#ifndef INPUT_H
#define INPUT_H

#include "asv.h"

struct Waypoints
{
  int count;
  struct Point points[COUNT_WAYPOINTS_MAX];
};

/**
 * Function to read the input file and set the ASV's input values. 
 * @param file is the path to the input toml file. 
 * @param asv is the asv object for which the input values are to be set. 
 * @param waypoints through which the vehicle has to navigate. 
 * @return clock step size if provided in the input file else 0.0.
 */
double set_input(char* file, struct Asv* asv, struct Waypoints* waypoints);

#endif