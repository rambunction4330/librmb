
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

namespace rmb {
class AngularEncoder {
public:
  /**
   * Common interface for returning the angular velocity of an encoder.
   *
   * @return The velocity of the encoder in radians per second.
   */
  virtual units::radians_per_second_t getVelocity() const = 0;

  /**
   * Common interface for returning the angular position of an encoder.
   *
   * @return The position of the encoder in radians.
   */
  virtual units::radian_t getPosition() const  = 0;

    /**
   * Common interface for zeroing the anguklar positon an encoder so the current
   * position is set to the offset.
   *
   * @param offset the offset from the current angular position at which to 
   *               set the zero position.
   */
  virtual void zeroPosition(units::radian_t offset = 0_rad) const = 0;
};
} // namespace rmb