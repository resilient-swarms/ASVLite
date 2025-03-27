#pragma once

#include <cmath>

namespace ASVLite {

    namespace Geometry {

        /** @brief Number of dimensions in 3D space. */
        constexpr int COUNT_COORDINATES = 3;

        /**
         * @brief Cartesian coordinates in 3D space.
         * 
         * Allows access to x, y, z either via named members or as an array.
         */
        union Coordinates3D {
            /**
             * @brief Struct for key-based access to coordinate components.
             * 
             * Use keys.x, keys.y, keys.z to access individual coordinates.
             */
            struct {
                double x; ///< X-coordinate
                double y; ///< Y-coordinate
                double z; ///< Z-coordinate
            } keys;

            /**
             * @brief Array-based access to coordinate values.
             * 
             * Index 0: x, Index 1: y, Index 2: z.
             */
            double array[COUNT_COORDINATES];

            /**
             * @brief Equality operator to compare two 3D coordinates.
             * 
             * @param rhs The right-hand side Coordinates3D to compare.
             * @return true if all coordinate values match, false otherwise.
             */
            bool operator==(const Coordinates3D& rhs);
        };


        /** @brief Number of degrees of freedom for a rigid body in 3D space. */
        constexpr int COUNT_DOF = 6;

        /**
         * @brief Six degrees of freedom (DOF) for a rigid body in 3D space.
         * 
         * Allows access to translational (surge, sway, heave) and rotational (roll, pitch, yaw)
         * components either by named keys or by array index.
         */
        union RigidBodyDOF {
            /**
             * @brief Struct for key-based access to DOF components.
             * 
             * Use keys.surge, keys.sway, keys.heave, keys.roll, keys.pitch, keys.yaw.
             */
            struct {
                double surge;  ///< Surge (translation along x-axis)
                double sway;   ///< Sway (translation along y-axis)
                double heave;  ///< Heave (translation along z-axis)
                double roll;   ///< Roll (rotation about x-axis)
                double pitch;  ///< Pitch (rotation about y-axis)
                double yaw;    ///< Yaw (rotation about z-axis)
            } keys;

            /**
             * @brief Array-based access to DOF values.
             * 
             * Index 0: surge, 1: sway, 2: heave, 3: roll, 4: pitch, 5: yaw.
             */
            double array[COUNT_DOF];
        };


        /**
         * @brief Normalizes an angle to the range (-PI, PI].
         * 
         * @param angle Angle in radians.
         * @return double Normalized angle in radians.
         */
        inline double normalise_angle_PI(const double angle) {
            // Reduce the angle if greater than 2PI
            double value = fmod(angle, 2.0*M_PI);
            // Set to range (-PI, PI]
            if(value > M_PI)
            {
                value -= (2.0*M_PI);
            }
            if(value < -M_PI)
            {
                value += (2.0*M_PI);
            }
            return value;
        }


        /**
         * @brief Normalizes an angle to the range [0, 2PI).
         * 
         * @param angle Angle in radians.
         * @return double Normalized angle in radians.
         */
        inline double normalise_angle_2PI(const double angle) {
            // Reduce the angle if greater than 2PI
            double value = fmod(angle, 2.0*M_PI);
            // Set to range [0, PI)
            value = fmod(angle + 2.0*M_PI, 2.0*M_PI);
            return value;
        }
    
    }

}