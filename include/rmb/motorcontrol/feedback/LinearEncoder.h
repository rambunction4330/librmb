
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
  virtual void zeroPosition(units::meter_t offset = 0_m) = 0;

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setEncoderInverted(bool isInverted) = 0;

  /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  virtual bool getEncoderInverted() const = 0;

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  /**
   * Generates a `LinearAsAngularEncoder` to measure the same mechanism
   * as this controller, but with angular instead of linear units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they control the same physical mechanism.
   * 
   * @param conversion conversion from linear to angular units.
   */
  LinearAsAngularEncoder asAngularEncoder(ConversionUnit_t conversion) {
    return LinearAsAngularEncoder(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class LinearAsAngularEncoder : public AngularEncoder {
public:
  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  LinearAsAngularEncoder(const LinearAsAngularEncoder&) = delete;
  LinearAsAngularEncoder(LinearAsAngularEncoder&&) = default;

  LinearAsAngularEncoder(LinearEncoder& linearEncoder, ConversionUnit_t conversionFactor) :
                         linear(linearEncoder), conversion(conversionFactor) {}

  units::radians_per_second_t getVelocity() const { return linear.getVelocity() * conversion; }
  units::radian_t getPosition() const { return linear.getPosition() * conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear.zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { linear.setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return linear.getEncoderInverted(); }

private:
  LinearEncoder& linear;
  ConversionUnit_t conversion;
};
} // namespace rmb