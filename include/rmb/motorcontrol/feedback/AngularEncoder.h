
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "rmb/motorcontrol/feedback/LinearEncoder.h"

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
  virtual units::radian_t getPosition() const = 0;

    /**
   * Common interface for zeroing the anguklar positon an encoder so the current
   * position is set to the offset.
   *
   * @param offset the offset from the current angular position at which to 
   *               set the zero position.
   */
  virtual void zeroPosition(units::radian_t offset = 0_rad) = 0;

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setEncoderInverted(bool isInverted) = 0;

  /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return The state of inversion, true is inverted.
   */
  virtual bool getEncoderInverted() const = 0;

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  /**
   * Generates a `AngularAsLinearEncoder` to measure the same mechanism as this
   * object, but with linear instead of angular units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they measure the same physical mechanism.
   * 
   * @param conversion conversion from linear to angular units.
   */
  AngularAsLinearEncoder asLinearEncoder(ConversionUnit_t conversion) {
    return AngularAsLinearEncoder(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class AngularAsLinearEncoder : public LinearEncoder {
public:
  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  AngularAsLinearEncoder(const AngularAsLinearEncoder&) = delete;
  AngularAsLinearEncoder(AngularAsLinearEncoder&&) = default;

  AngularAsLinearEncoder(AngularEncoder& angularEncoder, ConversionUnit_t conversionFactor) :
                         angular(angularEncoder), conversion(conversionFactor) {}

  units::meters_per_second_t getVelocity() const { return angular.getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular.getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular.zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { angular.setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return angular.getEncoderInverted(); }

private:
  AngularEncoder& angular;
  ConversionUnit_t conversion;
};
} // namespace rmb