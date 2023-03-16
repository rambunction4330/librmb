
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace rmb {
namespace MotorControlConversions {
using ConversionUnit =
    units::compound_unit<units::meters, units::inverse<units::radians>>;
using ConversionUnit_t = units::unit_t<ConversionUnit>;
} // namespace MotorControlConversions
} // namespace rmb