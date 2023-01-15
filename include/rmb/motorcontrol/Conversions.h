
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace rmb {
namespace MotorControlConversions {

// A place for these commonly used non-standard units that avoids namespace
// polution.
using ConversionUnit =
    units::compound_unit<units::meters, units::inverse<units::radians>>;
using ConversionUnit_t = units::unit_t<ConversionUnit>;
} // namespace MotorControlConversions
} // namespace rmb