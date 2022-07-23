#ifndef SIMULATION_H
#define SIMULATION_H

#include <stdbool.h>
#include "geometry.h"

/**
 * @file
 * An instance of Simulation should only be created by calling the function simulation_new(). 
 * This function allocates and initialises a block of memory on the stack, and 
 * therefore all calls to simulation_new() should be paired with a call to simulation_delete() 
 * to avoid memory leaks. 
 */
struct Simulation;
struct Asv;

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
void simulation_set_input_using_file(struct Simulation* simulation,
                                     char* file,  
                                     double wave_ht, 
                                     double wave_heading, 
                                     long rand_seed,
                                     bool with_time_sync);

/**
 * Function to init simulation using an instance of Asv. 
 * @param asvs array to pointers of asvs used for initialising simulation.
 * @param count_asvs is the size of the asvs array.  
 * @param with_time_sync is true when simulations of all asvs are to run synchronous; else false.   
 */
void simulation_set_input_using_asvs(struct Simulation* simulation,
                                    struct Asv** asvs,  
                                    int count_asvs,
                                    bool with_time_sync);

/**
 * Set a new array of waypoints for an asv. 
 * @param asv for which the new set of waypoints are to be set.
 * @param waypoints is the array of new waypoints for the asv.
 * @param count_waypoints is the size of the array waypoints.
 */
void simulation_set_waypoints_for_asv(struct Simulation* simulation,
                                      struct Asv* asv, 
                                      union Coordinates_3D* waypoints,
                                      int count_waypoints);

/**
 * Set the controller used for all asvs in the simulation.
 * @param gain_position is an array of size 3 with gain terms for position in the order - proportional, integral, differential.
 * @param gain_heading is an array of size 3 with gain terms for heading in the order - proportional, integral, differential.
 */
void simulation_set_controller(struct Simulation* simulation, double* gain_position, double* gain_heading);

/**
 * Tune the controllers for the asvs.
 */
void simulation_tune_controller(struct Simulation* simulation);

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
 * Simulate vehicle dynamics for each time step till the vehicle reaches the last waypoint. 
 */
void simulation_run_upto_waypoint(struct Simulation* simulation);

/**
 * Simulate vehicle dynamics for each time step for a fixed time. 
 * @param max_time is the time to stop simulation.
 */
void simulation_run_upto_time(struct Simulation* simulation, double max_time);

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

/**
 * Get the buffer associated with the simulation of an asv. 
 */
struct Buffer* simulation_get_buffer(struct Simulation* simulation, struct Asv* asv);

/**
 * Get the buffer length associated with the simulation of an asv. 
 */
struct Buffer* simulation_get_buffer_length(struct Simulation* simulation, struct Asv* asv);

/**
 * Get the position of the asv recorded in the buffer at given index.
 */
union Coordinates_3D buffer_get_asv_position_at(struct Buffer* buffer, int index);

#endif // SIMULATION_H
