#ifndef IO_H
#define IO_H

#include "asv.h"

/**
 * A simle structure to store the waypoints. 
 */
struct Waypoints
{
  int count;
  struct Point points[COUNT_WAYPOINTS_MAX];
};

/**
 * A simple struct to record the simulated data for each time step of 
 * simulation.
 */
struct
{ 
  double sig_wave_ht; // m
  double wave_heading; // deg
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
} buffer[OUTPUT_BUFFER_SIZE];

/**
 * Function to read the input file and set the ASV's input values. 
 * @param file is the path to the input toml file. 
 * @param asv is the asv object for which the input values are to be set. 
 * @param waypoints through which the vehicle has to navigate. 
 */
void set_input(char* file, struct Asv* asv, struct Waypoints* waypoints);

/**
 * Function to write the simulated data to file. 
 * @param file is the path to the output file. 
 * @param buffer_length is the number of lines in the buffer.
 */
void write_output(char* file, 
                  int buffer_length,
                  double wave_ht, 
                  double wave_heading, 
                  double task_duration,
                  double simulation_time);

#endif 