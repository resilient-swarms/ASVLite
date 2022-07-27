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
 * Simulate the next time step. 
 */
void simulation_run_a_timestep(struct Simulation* simulation);

/**
 * Get the buffer size associated with the simulation of an asv. Buffer length <= buffer size.
 */
long simulation_get_buffer_size();

/**
 * Get the buffer length associated with the simulation of an asv. 
 */
long simulation_get_buffer_length(struct Simulation* simulation, struct Asv* asv);

/**
 * Get the buffer associated with the simulation of an asv. 
 */
struct Buffer* simulation_get_buffer(struct Simulation* simulation, struct Asv* asv);

/**
 * Function to get the total number of asvs simulated. 
 */
int simulation_get_count_asvs(struct Simulation* simulation);

/**
 * Get the current waypoint for an asv.
 */
union Coordinates_3D simulation_get_waypoint(struct Simulation* simulation, struct Asv* asv);

/**
 * Get the number of waypoints for the asv.
 */
int simulation_get_count_waypoints(struct Simulation* simulation, struct Asv* asv);

/**
 * Function to get all waypoints for an asv.
 */
union Coordinates_3D* simulation_get_waypoints(struct Simulation* simulation, struct Asv* asv);

/**
 * Function to get all asvs simulated.
 * @param asvs is the return array with pointers to the asvs simulated. The function assumes that 
 * the caller has allocated sufficient size for asvs to contain all the pointers to be placed in it.
 * @return the number of pointers placed into the argument asvs. 
 */
int simulation_get_asvs(struct Simulation* simulation, struct Asv** asvs);

/**
 * Get the position of the asv recorded in the buffer at given index.
 */
union Coordinates_3D buffer_get_asv_position_at(struct Buffer* buffer, int index);

#endif // SIMULATION_H
