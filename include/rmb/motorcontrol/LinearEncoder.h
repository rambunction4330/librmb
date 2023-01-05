
#pragma once

#include <units/length.h>
#include <units/velocity.h>

namespace rmb {
class LinearEncoder {
public:
  /**
   * Common interface for returning the linear velocity of an encoder.
   *
   * @return The velocity of the encoder in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() const = 0;

  /**
   * Common interface for returning the linear position of an encoder.
   *
   * @return The position of the encoder in meters.
   */
  virtual units::meter_t getPosition() const  = 0;

    /**
   * Common interface for zeroing the linear positon an encoder so the current
   * position is set to the offset.
   *
   * @param offset the offset from the current angular position at which to 
   *               set the zero position.
   */
  virtual void zeroPosition(units::meter_t offset = 0_m) const = 0;
};
} // namespace rmb