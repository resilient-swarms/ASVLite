#pragma once

namespace ASVLite {

    namespace Geometry {
    

        constexpr int COUNT_COORDINATES = 3; // Number of dimensions in 3D space
        // Cartesian coordinates for a three-dimensional space. 
        // The members can either be accessed as a struct using keys or as an array using index.
        union Coordinates3D {
            struct {
                double x; 
                double y; 
                double z; 
            } keys; // To access the coordinate values using the keys x, y, z.
            double array[COUNT_COORDINATES]; // To access the coordinate values using index 0, 1, 2.

            bool operator==(const Coordinates3D& rhs) {
                return (this->keys.x == rhs.keys.x) &&
                       (this->keys.y == rhs.keys.y) &&
                       (this->keys.z == rhs.keys.z);
            }
        };


        constexpr int COUNT_DOF = 6; // Number of degrees of freedom for a rigid body.
        // Six degrees of freedom for a rigid body in a three-dimensional space. 
        // The members can either be accessed as a struct using keys or as an array using index.
        union RigidBodyDOF {
            struct {
                double surge; 
                double sway;  
                double heave; 
                double roll;  
                double pitch; 
                double yaw;   
        } keys; // To access the DOF values using the keys surge, sway, heave, roll, pitch, yaw.
        double array[COUNT_DOF]; // To access the DOF values using index 0, 1, 2, 3, 4, 5.
        };


        // Normalise angle so that -PI < angle <= PI.
        // param angle in radians.
        // returns normalised angle in radians. 
        double normalise_angle_PI(const double angle);


        // Normalise angle so that 0 <= angle < 2PI.
        // param angle in radians.
        // returns normalised angle in radians. 
        double normalise_angle_2PI(const double angle);
    
    }

}