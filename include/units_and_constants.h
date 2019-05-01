/**
 * This header file provides a wrapper for Boost.Units library for convenience.
 * Coding using Boost.Units can be verbose because of the nested namespaces 
 * used. This header file aims at making coding using Boost.Units more concise 
 * by providing shorter namespace aliases.
 * 
 * Example of creating an object of type length without using this headder:
 * boost::units::quantity<boost::units::si::length> L = 
 *                                                10 * boost::units::si::meter;
 * 
 * Using this header, the above line can be shortened to:
 * Quantity<Units::length> L = 10 * Units::meter;
 * 
 * This header file also defines some numerical constants which are commonly 
 * used.
 */

#ifndef UNITS_H
#define UNITS_H

#include<boost/units/systems/si/prefixes.hpp>
#include<boost/units/quantity.hpp>
#include<boost/units/systems/si/length.hpp>
#include<boost/units/systems/si/volume.hpp>
#include<boost/units/systems/si/mass.hpp>
#include<boost/units/systems/si/wavenumber.hpp>
#include<boost/units/systems/si/acceleration.hpp>
#include<boost/units/systems/si/angular_acceleration.hpp>
#include<boost/units/systems/si/velocity.hpp>
#include<boost/units/systems/si/angular_velocity.hpp>
#include<boost/units/systems/si/force.hpp>
#include<boost/units/systems/si/time.hpp>
#include<boost/units/systems/si/frequency.hpp>
#include<boost/units/systems/si/plane_angle.hpp>
#include<boost/units/systems/angle/degrees.hpp>
#include<boost/units/systems/si/dimensionless.hpp>
#include<boost/units/systems/si/prefixes.hpp>
#include<boost/units/cmath.hpp>
#include<cmath>
#include"quantity.h"

namespace asv_swarm
{
/**
 * Quantity<T>, an alias for boost::units::quantity<T>.
 */
template<typename T>
using Quantity = boost::units::quantity<T>;

/**
 * Units, an alias for boost::units::si.
 */
namespace Units = boost::units::si;

/**
 * SR, an alias for boost::units::static_rational.
 */
template<boost::units::integer_type N, boost::units::integer_type D>
using SR = boost::units::static_rational<N, D>;

/**
 * Numerical constants.
 */
namespace Constant
{
const Quantity<Units::acceleration> G = 
  9.81 * (Units::meter / Units::second/ Units::second); /* Acceleration due to 
                                                           gravity. */

const Quantity<Units::dimensionless> PI = M_PI;  /* Pi */

const Quantity<Units::density> RHO_SEA_WATER = 
  1025*Units::kilogram_per_meter_cube; /* Density of sea water. */

} // namespace Const
} // namespace asv_swarm

#endif // UNITS_H
