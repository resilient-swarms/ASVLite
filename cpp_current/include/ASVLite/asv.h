#pragma once

#include "geometry.h"
#include "sea_surface.h"
#include <Eigen/Dense>
#include <vector>
#include <btBulletDynamicsCommon.h>


namespace ASVLite {

    struct AsvSpecification {
        const double L_wl;                // Length at waterline in m.
        const double B_wl;                // Breadth at waterline in m.
        const double D;                   // Depth of the ASV in m.
        const double T;                   // Draught of the ASV in m.
    };


    struct AsvDynamics {
        // Bullet Constants
        double time = 0.0; // sec
        const double time_step_size = 40; // milli-sec
        const int max_substeps = 10;

        // Position and attitude of the vehicle for the current time.
        // The origin is placed at the bottom-mid position of the vehicle.
        Geometry::Coordinates3D position;
        Geometry::Coordinates3D attitude;
        double submersion_depth;

        Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero(); // Mass + added mass in Kg. 
        Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero(); // Drag force coefficients. 
        Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero(); // Stiffness. 

        Eigen::Matrix<double, 6, 1> X = Eigen::Matrix<double, 6, 1>::Zero(); // Deflection in body-fixed frame. 
        Eigen::Matrix<double, 6, 1> V = Eigen::Matrix<double, 6, 1>::Zero(); // Velocity of ASV in body-fixed frame. 
        Eigen::Matrix<double, 6, 1> A = Eigen::Matrix<double, 6, 1>::Zero(); // Acceleration of ASV in body-fixed frame. 
        
        Eigen::Matrix<double, 6, 1> F           = Eigen::Matrix<double, 6, 1>::Zero(); // Net force. 
        Eigen::Matrix<double, 6, 1> F_wave      = Eigen::Matrix<double, 6, 1>::Zero(); // Wave force. 
        Eigen::Matrix<double, 6, 1> F_thrust    = Eigen::Matrix<double, 6, 1>::Zero(); // Thruster force. 
        Eigen::Matrix<double, 6, 1> F_damping   = Eigen::Matrix<double, 6, 1>::Zero(); // Quadratic force force. 
        Eigen::Matrix<double, 6, 1> F_restoring = Eigen::Matrix<double, 6, 1>::Zero(); // Hydrostatic restoring force.  
    };


    class Asv {
        public:
            Asv(const AsvSpecification& spec, const SeaSurface* sea_surface, const Geometry::Coordinates3D& position, const Geometry::Coordinates3D& attitude);
            void step_simulation(const Geometry::Coordinates3D& thrust_position, const Geometry::Coordinates3D& thrust_magnitude);
            void set_sea_state(const SeaSurface* sea_surface);
            void set_ocean_current(const std::pair<double, double>& ocean_current) {
                this->ocean_current = ocean_current;
            }
            void set_surge_sway_halt(bool set_halt) {
                halt_surge_and_sway = set_halt;
            }
            const SeaSurface* get_sea_surface() const {
                return sea_surface;
            }
            Geometry::Coordinates3D get_position() const {
                return dynamics.position;
            }
            Geometry::Coordinates3D get_attitude() const {
                return dynamics.attitude;
            }
            double get_submersion_depth() const {
                return dynamics.submersion_depth;
            }
            double get_time() const {
                return dynamics.time;
            }
            double get_time_step_size() const {
                return dynamics.time_step_size;
            }
            Geometry::RigidBodyDOF get_wave_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_wave(i);
                }
                return force;
            }
            Geometry::RigidBodyDOF get_damping_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_damping(i);
                }
                return force;
            }
            Geometry::RigidBodyDOF get_restoring_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_restoring(i);
                }
                return force;
            }
            Geometry::RigidBodyDOF get_propulsive_thrust() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F_thrust(i);
                }
                return force;
            }
            Geometry::RigidBodyDOF get_net_force() const {
                Geometry::RigidBodyDOF force;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    force.array[i] = dynamics.F(i);
                }
                return force;
            }

            Geometry::RigidBodyDOF get_velocity() const {
                Geometry::RigidBodyDOF velocity;
                for(size_t i = 0; i < Geometry::COUNT_DOF; ++i){
                    velocity.array[i] = dynamics.V(i);
                }
                return velocity;
            }
            AsvSpecification get_spec() const {
                return spec;
            }

        private:
            void set_mass();
            void set_damping_coefficient();
            void set_stiffness();
            void set_wave_force();
            void set_thrust(const Geometry::Coordinates3D& thrust_position, const Geometry::Coordinates3D& thrust_magnitude);
            void set_damping_force();
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


    std::pair<Geometry::Coordinates3D, Geometry::Coordinates3D> get_wave_glider_thrust(const Asv& wave_glider, double rudder_angle, double tuning_factor_thrust); // rudder angle in range (-PI/2, PI/2). Angle is positive when the vehicle has to turn 
                                                                                                                                                                // to portside (ie. aft end of the rudder points to portside)
}