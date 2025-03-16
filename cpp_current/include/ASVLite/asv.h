#pragma once

#include "geometry.h"
#include "sea_surface.h"
#include <Eigen/Dense>
#include <vector>


namespace ASVLite {

    struct AsvSpecification {
        const double L_wl; // Length at waterline in m.
        const double B_wl; // Breadth at waterline in m.
        const double D;    // Depth of the ASV in m.
        const double T;    // Draught of the ASV in m.
    };

    // Structure to hold the rigid body dynamics variables. 
    struct AsvDynamics {
        double time = 0.0; // sec
        const double time_step_size = 40; // milli-sec

        // Position and attitude of the vehicle for the current time.
        // The reference origin is located at the midpoint of the vehicle's still-waterline position.
        Geometry::Coordinates3D position;
        Geometry::Coordinates3D attitude;
        double submersion_depth; // m

        Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero(); // Mass + added mass in Kg. 
        Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero(); // Drag force coefficients. 
        Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero(); // Stiffness. 

        Eigen::Matrix<double, 6, 1> X = Eigen::Matrix<double, 6, 1>::Zero(); // Deflection in body-fixed frame. 
        Eigen::Matrix<double, 6, 1> V = Eigen::Matrix<double, 6, 1>::Zero(); // Velocity of ASV in body-fixed frame. 
        Eigen::Matrix<double, 6, 1> A = Eigen::Matrix<double, 6, 1>::Zero(); // Acceleration of ASV in body-fixed frame. 
        
        Eigen::Matrix<double, 6, 1> F           = Eigen::Matrix<double, 6, 1>::Zero(); // Net force. 
        Eigen::Matrix<double, 6, 1> F_wave      = Eigen::Matrix<double, 6, 1>::Zero(); // Wave force. 
        Eigen::Matrix<double, 6, 1> F_thrust    = Eigen::Matrix<double, 6, 1>::Zero(); // Thruster force. 
        Eigen::Matrix<double, 6, 1> F_drag      = Eigen::Matrix<double, 6, 1>::Zero(); // Quadratic force.
        Eigen::Matrix<double, 6, 1> F_restoring = Eigen::Matrix<double, 6, 1>::Zero(); // Hydrostatic restoring force.  
    };


    class Asv {
        public:
            // Constructor.
            // param spec of the ASV. 
            // param sea_surface is the irregular sea surface for the asv. 
            // param position of the asv on the sea surface. 
            // param attitude of the asv.
            Asv(const AsvSpecification& spec, const SeaSurface* sea_surface, const Geometry::Coordinates3D& position, const Geometry::Coordinates3D& attitude);

            // Advances the simulation by one time step based on the applied thrust.
            // param thrust_position represents the point of thrust application on the vehicle in body-coordinates.
            // param thrust_magnitude in vector representing the thrust magnitude and direction.
            void step_simulation(const Geometry::Coordinates3D& thrust_position, const Geometry::Coordinates3D& thrust_magnitude);

            // Function to modify the current sea state to a new sea state. 
            void set_sea_state(const SeaSurface* sea_surface);

            // Function to modify the ocean current to a new state. 
            void set_ocean_current(const std::pair<double, double>& ocean_current) {
                this->ocean_current = ocean_current;
            }

            // Set true to halt surge and sway motions. All the remainig 4 dof are not ignored.
            void set_surge_sway_halt(bool set_halt) {
                halt_surge_and_sway = set_halt;
            }

            // Get the current sea state.
            const SeaSurface* get_sea_surface() const {
                return sea_surface;
            }

            // Get current position of the vehicle
            Geometry::Coordinates3D get_position() const {
                return dynamics.position;
            }

            // Get current attitude of the vehicle. 
            Geometry::Coordinates3D get_attitude() const {
                return dynamics.attitude;
            }

            // Get the depth of the vehicle's lowest point relative to the waterline.  
            // Depth is expected to be negative when the vehicle is submerged.  
            // A positive depth indicates that the vehicle is above the waterline (out of the water).
            double get_submersion_depth() const {
                return dynamics.submersion_depth;
            }

            // Time since start of simulation in sec. 
            double get_time() const {
                return dynamics.time;
            }

            // Get time step size used in simulation in milli-sec. 
            double get_time_step_size() const {
                return dynamics.time_step_size;
            }

            // Get wave force magnitude, N, for current time as a vector.
            Geometry::RigidBodyDOF get_wave_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_wave(i);
                }
                return force;
            }

            // Get drag force magnitude, N, for current time as a vector.
            Geometry::RigidBodyDOF get_drag_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_drag(i);
                }
                return force;
            }

            // Get the restoring force magnitude, N, for the current time as a vector.
            Geometry::RigidBodyDOF get_restoring_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_restoring(i);
                }
                return force;
            }

            // Get the propulsive thrust, N, for the current time as a vector.
            Geometry::RigidBodyDOF get_propulsive_thrust() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_thrust(i);
                }
                return force;
            }

            // Get the net force, in N, acting on the vehicle for the current time as a vector.
            Geometry::RigidBodyDOF get_net_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F(i);
                }
                return force;
            }

            // Get acceleration, in m/s2.
            Geometry::RigidBodyDOF get_acceleration() const {
                Geometry::RigidBodyDOF acceleration;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    acceleration.array[i] = dynamics.A(i);
                }
                return acceleration;
            }

            // Get currnt vehicle velocity. 
            Geometry::RigidBodyDOF get_velocity() const {
                Geometry::RigidBodyDOF velocity;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    velocity.array[i] = dynamics.V(i);
                }
                return velocity;
            }

            // Get mass. 
            Geometry::RigidBodyDOF get_mass() const {
                Geometry::RigidBodyDOF mass;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    mass.array[i] = dynamics.M(i,i);
                }
                return mass;
            }

            // Get vechicle specification.
            AsvSpecification get_spec() const {
                return spec;
            }

        private:
            double get_submerged_volume(const double submersion_depth) const; // submerged depth should be -ve.
            double get_added_mass_coeff() const;
            double get_drag_coefficient_parallel_flow(const double l, const double d) const; // l --> dimension along the flow, d --> dimension perpendicular to the flow.
            double get_drag_coefficient_prependicular_flow(const double b, const double h) const; // b --> longer edge, h --> short edge.
            void set_mass();
            void set_drag_coefficient();
            void set_stiffness();
            void set_wave_force();
            void set_thrust(const Geometry::Coordinates3D& thrust_position, const Geometry::Coordinates3D& thrust_magnitude);
            void set_drag_force();
            void set_restoring_force();
            void set_net_force();
            void set_acceleration();
            void set_velocity();
            void set_deflection();
            void set_pose();
        
        private:
            const AsvSpecification spec;                        // ASV specification.
            const SeaSurface* sea_surface;                      // Irregular sea_surface instance. 
            std::pair<double, double> ocean_current{0.0, 0.0};  // zonal and meridional velocities of ocean current in m/s; 
            bool halt_surge_and_sway {false};                   // Set variable to true to keep the ASV stationary.            
            AsvDynamics dynamics;                
    };


    // Function to get the position and magnitude of propulsive thrust generated by the subsurface gliders.
    // param wave_glider is a reference to the asv. 
    // param rudder_angle is the angle of the rudder with respect to X axis of the ASV. 
    // Rudder angle must within (-PI/2, PI/2). Angle is positive when the vehicle has to turn 
    // to starboard (ie. aft end of the rudder points to starboard side). 
    std::pair<Geometry::Coordinates3D, Geometry::Coordinates3D> get_wave_glider_thrust(const Asv& wave_glider, const double rudder_angle, const double significant_wave_ht); 
}