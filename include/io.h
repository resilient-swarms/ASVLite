#ifndef IO_H
#define IO_H

#include "asv.h"
#include <pthread.h>

/**
 * A simple structure to store the waypoints. 
 */
struct Waypoints
{
  int count;
  struct Dimensions points[COUNT_WAYPOINTS_MAX];
};

/**
 * A simple struct to record the simulated data for each time step of 
 * simulation.
 */
struct Buffer
{ 
  double sig_wave_ht; // m
  double wave_heading; // deg
  long random_number_seed;
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
  double surge_velocity; // m/s.
  double surge_acceleration; // m/s2. 
  double F_surge; // N
  double F_sway; //N
};

/**
 * Linked list to store simulation data related to each asv.
 */
struct Simulation_data
{
  // Each simulation runs on its own thread
  pthread_t thread;
  // Inputs and outputs
  char id[32];
  struct Asv* asv; 
  struct Waypoints* waypoints;
  struct Buffer* buffer;
  // Data related to current time step in the simulation
  long current_time_index;
  int current_waypoint_index;
  // Linkes list pointers
  struct Simulation_data* previous; // previous in the linked list.
  struct Simulation_data* next; // next in the linked list.
};

/** Initialise a new node for the linked list.
 */
struct Simulation_data* simulation_data_new_node();

/**
 * Function to read the input file and set the ASV's input values. 
 * @param simulation_data first item of the linked list.
 * @param file is the path to the input toml file.  
 * @param wave_ht wave height in meter.
 * @param wave_heading in deg.
 * @param rand_seed seed for random number generator. 
 */
void simulation_data_set_input(struct Simulation_data* simulation_data,
                               char* file,  
                               double wave_ht, 
                               double wave_heading, 
                               long rand_seed);

/**
 * Function to write the simulated data to file. 
 * @param simulation_data first item of the liked list.
 * @param out is the path to the output director or file.  
 * @param simulation_time in sec.
 */
void simulation_data_write_output(struct Simulation_data* simulation_data,
                                  char* out, 
                                  double simulation_time);

#endif 
