#ifndef ASV_H
#define ASV_H

#include <stdbool.h>
#include "constants.h"
#include "geometry.h"
#include "wave.h"

/**
 * Index for DoF for asv dynamics.
 */
enum i_dof{surge, sway, heave, roll, pitch, yaw}; 
/**
 * Index for axis for linear motion:
 */
enum i_axis{x, y, z};
/** 
 * Index for floating attitude of ASV:
 */
enum i_attitude{heel, trim, heading}; 

/**
 * Struct to hold all the inputs for the propeller.
 */
struct Asv_propeller
{
  struct Dimensions position; //!< Input variable. Position of propeller force 
                              //!< vector in ASV's body-fixed frame.
  struct Dimensions orientation; //!< Input variable. Orientation of the force 
                                 //!< vector of the propeller in body-fixed 
                                 //!< frame.
  double thrust; //!< Magnitude of propeller force in Newton.
};

/**
 * Struct to hold some of the input value of the vehicle. All 
 * variables in this struct are inputs and should be set before calling 
 * asv_init().
 * Coordinate system: Body-centric frame. The origin of the frame is on the
 * waterline at the aft end centre line.
 */
struct Asv_specification
{
  double L_wl;      //!< Input variable. Length waterline in m.
  double B_wl;      //!< Input variable. Breadth waterline in m.
  double D;         //!< Input variable. Depth of the ASV in m.
  double T;         //!< Input variable. Draught of the ASV in m.
  double max_speed; //!< Input variable. Maximum operational speed of the ASV in 
                    //!< m/s.
  double disp;   //!< Input variable. Displacement of the ASV in m3.
  double r_roll; //!< Input variable. roll radius of gyration.
  double r_pitch;//!< Input variable. pitch radius of gyration.
  double r_yaw;  //!< Input variable. yaw radius of gyration.
  struct Dimensions cog; //!< Input variable. Centre of gravity in body-fixed 
                         //!< frame.
};

/**
 * Struct to contain both inputs and outputs of ASV dynamics. All input
 * variables must be set before calling asv_compute_dynamics().
 */
struct Asv_dynamics
{
  // Input
  double time_step_size; //!< Input variable. Time step size in seconds.
  double time; //!< Input variable. Time since start of simulation in seconds.
  
  // Future work: Set these as over writeable inputs
  double M[COUNT_DOF]; //!< Output variable. Mass + added mass in Kg.
  double C[COUNT_DOF]; //!< Output variable. Drag force coefficients.
  double K[COUNT_DOF]; //!< Output variable. Stiffness.
  
  // Output
  double X[COUNT_DOF]; //!< Output variable. Deflection in body-fixed frame.
  double V[COUNT_DOF]; //!< Output variable. Velocity of ASV in body-fixed frame.
  double A[COUNT_DOF]; //!< Output variable. Acceleration of ASV in body-fixed 
                       //!< frame.
  
  double F[COUNT_DOF]; //!< Output variable. Net force.
  double F_wave[COUNT_DOF]; //!< Output variable. Wave force.
  double F_propeller[COUNT_DOF]; //!< Output variable. Propeller force.
  double F_drag[COUNT_DOF]; //!< Output variable. Quadratic force force.
  double F_restoring[COUNT_DOF]; //!< Output variable. Hydrostatic restoring 
                                 //!< force.

  double P_unit_wave[COUNT_ASV_SPECTRAL_FREQUENCIES][2]; //!< Output variable. 
    //!< 2D array with wave pressure amplitude. Index 0 is wave freq and index 
    //!< 1 gives the corresponding wave pressure amplitude. 
  double P_unit_regular_wave; //!< Output variable. Pressure amplitude when 
                              //!< wave_type is set as regular_wave.
  double P_unit_wave_freq_min;//!< Output variable. Minimum wave frequency 
                              //!< considered in array P_unit_wave.
  double P_unit_wave_freq_max;//!< Output variable. Maximum wave frequency 
                              //!< considered in array P_unit_wave.
};

/**
 * Stuct to contain both input and output of ASV motion in waves. 
 */
struct Asv
{
  // Input
  struct Asv_specification spec; //!< Input variable. ASV specification.
  int count_propellers; //!< Input variable. Number of propellers attached to 
                        //!< ASV. Should not be greater than
                        //!< COUNT_PROPELLERS_MAX defined in file constants.h.
  struct Asv_propeller propellers[COUNT_PROPELLERS_MAX]; //!< Input variable. 
                                                         //!< ASV propeller
                                                         //!< instances. 
  struct Wave* wave; //!< Input variable. Irregular wave instance. 

  // Initial used for input but later contains results. 
  struct Dimensions origin_position; //!< Initially set as input but later 
                                     //!< contains output. Position of the 
                                     //!< body-fixed frame in the global frame 
                                     //!< for the current time step.
  struct Dimensions attitude; //!< Initially set as input but later contains 
                              //!< output. The heel and trim are in body-fixed 
                              //!< frame and the heading is in global frame.
  
  // Output
  struct Asv_dynamics dynamics; //!< Output variable. ASV dynamics variables. 
  struct Dimensions cog_position; //!< Output variable. Position of the centre 
                             //!< of gravity of the ASV in the global frame for 
                             //!< the current time step.
};

/**
 * Function to initialise a model of ASV after setting the vehicle spec. 
 * **Note:** This function should be called only after setting all input values.
 * @param asv is the object to be initialised.
 * @param wave for the asv. Set to NULL for still water simulation.
 */
void asv_init(struct Asv* asv, struct Wave* wave);

/**
 * Function to set the position and attitude of the ASV for the given time step.
 * @param asv is the pointer to the asv object for which the position is to be
 * computed. 
 * @param time is the time for which the position is to be computed. 
 */
void asv_compute_dynamics(struct Asv* asv, double time);

/**
 * Function to initialise a new sea state.
 * @param asv is the object to be initialised.
 * @param wave for the asv. Set to NULL for still water simulation.
 */
void asv_set_sea_state(struct Asv* asv, struct Wave* wave);

/**
 * Similar to function asv_compute_dynamics but should be used only for a wave glider. 
 * This function to set the position and attitude of the ASV for the given time step, 
 * and also computes the wave thrust force generated by the underwater glider.
 * @param asv is the pointer to the asv object for which the position is to be
 * computed. 
 * @param rudder_angle is the angle of the rudder with respect to X axis of the ASV. 
 * Rudder angle must within (-90, 90), and is positive when aft end of the rudder points to starboard side. 
 * @param time is the time for which the position is to be computed.
 */
void wave_glider_compute_dynamics(struct Asv* asv, double rudder_angle, double time);

#endif // ASV_H
