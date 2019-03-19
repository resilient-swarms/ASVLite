#ifndef QUANTITY_H
#define QUANTITY_H

#include <boost/units/derived_dimension.hpp>
#include <boost/units/physical_dimensions/length.hpp>
#include <boost/units/physical_dimensions/mass.hpp>
#include <boost/units/physical_dimensions/time.hpp>
#include <boost/units/systems/si/base.hpp>

/**
 * This file defines some quantities that are required but not provided by
 * boost units.
 */
namespace boost
{
namespace units
{
/**
 * Derived dimension for damping coefficient.
 */
typedef derived_dimension<mass_base_dimension,1,
                          time_base_dimension,-1>::type damping_coeff_dimension;

/**
 * Derived dimension for stiffness coefficient.
 */
typedef derived_dimension<mass_base_dimension,1,
                        time_base_dimension,-2>::type stiffness_coeff_dimension;


/**
 * Derived dimension for density.
 */
typedef derived_dimension<mass_base_dimension,1,
                          length_base_dimension,-3>::type density_dimension;



namespace si 
{
typedef unit<damping_coeff_dimension, si::system> damping_coefficient;
BOOST_UNITS_STATIC_CONSTANT(newton_sec_per_meter, damping_coefficient);

typedef unit<stiffness_coeff_dimension, si::system> stiffness_coefficient;
BOOST_UNITS_STATIC_CONSTANT(newton_per_meter, stiffness_coefficient);

typedef unit<density_dimension, si::system> density;
BOOST_UNITS_STATIC_CONSTANT(kilogram_per_meter_cube, density);


} // namespace si
} // namespace units
} // namespace boost

#endif // QUANTITY_H
