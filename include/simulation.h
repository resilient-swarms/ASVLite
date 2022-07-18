#ifndef SIMULATION_H
#define SIMULATION_H

#include "asv.h"
#include "pid_controller.h"
#include <pthread.h>

struct Buffer;
struct Simulation;

/** 
 * Initialise a linked list of simulation nodes.
 */
struct Simulation* simulation_new();

/** 
 * Free the heap memory.
 * @param node is the first node in the linked list Simulatation_data.
 */
void simulation_delete(struct Simulation* node);

/**
 * Function to read the input file and set the ASV's input values. 
 * @param node is the first node in the linked list Simulatation_data.
 * @param file is the path to the input toml file.  
 * @param wave_ht wave height in meter.
 * @param wave_heading in deg.
 * @param rand_seed seed for random number generator.
 * @param with_time_sync is true when simulations of all asvs are to run synchronous; else false.   
 */
void simulation_set_input(struct Simulation* node,
                          char* file,  
                          double wave_ht, 
                          double wave_heading, 
                          long rand_seed,
                          bool with_time_sync);

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
 * simultion of each ASV in a independent thread and also synchronize
 * the simulation for each time step between ASVs. This function is slower
 * compared to the alternative simulate_without_time_sync() because the function
 * waits and joins the threads at each time step.
 */
void simulation_run(struct Simulation* first_node);

/**
 * Visualisation data from input file. Function to get the sea surface edge length in meter.
 */
double get_sea_surface_edge_length();

/** 
 * Visualisation data from input file. Function to get the number of mesh cells along 
 * one edge of the sea surface.
 */
int get_count_mesh_cells_along_edge();

/**
 * Visualisation data from input file. Function to get the bottom left corner of the simulated sea surface.
 */
union Coordinates_3D get_sea_surface_position();

#endif // SIMULATION_H
