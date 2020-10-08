#ifndef SIMULATION_H
#define SIMULATION_H

#include "asv.h"
#include <pthread.h>

/**
 * Structure to store an array of waypoints. 
 */
struct Waypoints
{
  int count;
  struct Dimensions points[COUNT_WAYPOINTS_MAX];
};

/**
 * Structure to record the ASV dynamics at each time step of 
 * the simulation.
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
 * Struct Simulation is a linked list to store simulation data related to each asv.
 */
struct Simulation
{
  // Each simulation runs on its own thread
  pthread_t thread;
  // Inputs and outputs
  char id[32];
  struct Wave* wave;
  struct Asv* asv; 
  struct Waypoints* waypoints;
  struct Buffer* buffer;
  // Data related to current time step in the simulation
  long current_time_index;
  int current_waypoint_index;
  // Linkes list pointers
  struct Simulation* previous; // previous in the linked list.
  struct Simulation* next; // next in the linked list.
};

/** Initialise a new node for the linked list.
 */
struct Simulation* simulation_new_node();

/** Free the heap memory.
 * @param node is the first node in the linked list Simulatation_data.
 */
void simulation_clean(struct Simulation* node);

/**
 * Function to read the input file and set the ASV's input values. 
 * @param node is the first node in the linked list Simulatation_data.
 * @param file is the path to the input toml file.  
 * @param wave_ht wave height in meter.
 * @param wave_heading in deg.
 * @param rand_seed seed for random number generator. 
 */
void simulation_set_input(struct Simulation* node,
                               char* file,  
                               double wave_ht, 
                               double wave_heading, 
                               long rand_seed);

/**
 * Function to write the simulated data to file. 
 * @param node is the first node in the linked list Simulatation_data.
 * @param out is the path to the output director or file. If the linked list contain only one node
 * then out is the name of the ouput file, else out is the name of the directory which will contain 
 * output files corresponding to each node.
 * @param simulation_time in sec.
 */
void simulation_write_output(struct Simulation* node,
                                  char* out, 
                                  double simulation_time);

/**
 * Simulate vehicle dynamics for each time step. This function runs 
 * simultion of each ASV in a independent thread and does not synchronize
 * the simulation for each time step between ASVs. This function is faster
 * compared to the alternative simulate_with_time_sync().
 */
void simulation_run_without_time_sync(struct Simulation* first_node);

/**
 * Simulate vehicle dynamics for each time step. This function runs 
 * simultion of each ASV in a independent thread and also synchronize
 * the simulation for each time step between ASVs. This function is slower
 * compared to the alternative simulate_without_time_sync() because the function
 * waits and joins the threads at each time step.
 */
void simulation_run_with_time_sync(struct Simulation* first_node);

/**
 * For all nodes simulate one time step.
 */
void simulation_with_time_sync_for_time_step(struct Simulation* first_node, long timer_count, bool* buffer_exceeded, bool* has_all_reached_final_waypoint);

/**
 * Visualisation data from input file. Function to get the sea surface edge length in meter.
 */
double get_sea_surface_edge_length();

/** 
 * Visualisation data from input file. Function to get the number of mesh cells along 
 * one edge of the sea surface.
 */
int get_count_mesh_cells_along_edge();

#endif // SIMULATION_H
