#ifndef SIMULATION_H
#define SIMULATION_H

#include <stdbool.h>

/**
 * An instance of Simulation should only be created by calling the function simulation_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to simulation_new() should be paired with a call to simulation_delete() 
 * to avoid memory leaks. 
 */
struct Simulation;

/** 
 * Initialise a simulation.
 */
struct Simulation* simulation_new();

/** 
 * Free the heap memory.
 * @param simulation is a non-null pointer to an instance of Simulation to be deallocated.
 */
void simulation_delete(struct Simulation* simulation);

/**
 * Function to read the input file and set the ASV's input values. 
 * @param file is the path to the input toml file with asv specs.  
 * @param wave_ht wave height in meter.
 * @param wave_heading in deg.
 * @param rand_seed seed for random number generator.
 * @param with_time_sync is true when simulations of all asvs are to run synchronous; else false.   
 */
void simulation_set_input(struct Simulation* simulation,
                          char* file,  
                          double wave_ht, 
                          double wave_heading, 
                          long rand_seed,
                          bool with_time_sync);

/**
 * Function to write the simulated data to file. 
 * @param out is the path to the output director or file. If the linked list contain only one simulation
 * then out is the name of the ouput file, else out is the name of the directory which will contain 
 * output files corresponding to each simulation.
 * @param simulation_time in sec.
 */
void simulation_write_output(struct Simulation* simulation,
                             char* out, 
                             double simulation_time);


/**
 * Simulate vehicle dynamics for each time step. 
 */
void simulation_run(struct Simulation* simulation);

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
