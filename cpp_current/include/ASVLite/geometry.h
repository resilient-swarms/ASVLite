#pragma once

namespace ASVLite {

    namespace Geometry {
    

        constexpr int COUNT_COORDINATES = 3; // Number of dimensions in 3D space
        union Coordinates3D {
            struct {
                double x;
                double y;
                double z;
            } keys;

            double array[COUNT_COORDINATES];
        };


        constexpr int COUNT_DOF = 6; // Number of degrees of freedom for a rigid body.
        union RigidBodyDOF {
            struct {
                double surge; 
                double sway;  
                double heave; 
                double roll;  
                double pitch; 
                double yaw;   
        } keys; 
        
        double array[COUNT_DOF]; 
        };


        /**
         * Normalise angle so that -PI < angle <= PI.
         * @param angle in radians.
         * @return normalised angle in radians. 
         */
        double normalise_angle_PI(const double angle);


        /**
         * Normalise angle so that 0 <= angle < 2PI.
         * @param angle in radians.
         * @return normalised angle in radians. 
         */
        double normalise_angle_2PI(const double angle);
    
    }

}