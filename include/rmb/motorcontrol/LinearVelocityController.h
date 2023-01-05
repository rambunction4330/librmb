
#pragma once

#include <units/velocity.h>

#include "rmb/motorcontrol/AngularVelocityController.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's linear velocity used by wrappers of 
 * device specific APIs.
 */
class LinearVelocityController {
public:

  /**
   * Common interface for setting the target linear velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  virtual void setVelocity(units::meters_per_second_t velocity) = 0;

  /**
   * Common interface for getting the target linear velocity.
   * 
   * @return The target linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getTargetVelocity() const = 0;

  /**
   * Common interface for setting the maximum linear velocity.
   * 
   * @param max The maximum linear velocity in meters per second.
   */
  virtual void setMaxVelocity(units::meters_per_second_t max) = 0;

  /**
   * Common interface for getting the maximum linear velocity.
   * 
   * @return The maximum linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getMaxVelocity() const = 0;

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setInverted(bool isInverted) = 0;

  /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  virtual bool getInverted() const = 0;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the mechanism until `setVelocity` is called again.
   */
  virtual void stop() = 0;

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  /**
   * Generates a `AngularVelocityController` to controll the same mechanism
   * as this controller, but with angular instead of linear units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they control the same physical mechanism.
   * 
   * @param conversion conversion from linear to angular units.
   */
  LinearAsAngularVelocityController asAngularController(ConversionUnit_t conversion) {
    return LinearAsAngularVelocityController(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class LinearAsAngularVelocityController: public AngularVelocityController {
 public:

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  LinearAsAngularVelocityController(const LinearAsAngularVelocityController&) = default;
  LinearAsAngularVelocityController(LinearAsAngularVelocityController&&) = delete;

  LinearAsAngularVelocityController(LinearVelocityController& linearController, 
                                    ConversionUnit_t conversionFactor) :
                                    linear(linearController), conversion(conversionFactor) {}

  void setVelocity(units::radians_per_second_t velocity) { linear.setVelocity(velocity * conversion); }
  units::radians_per_second_t getTargetVelocity() const { return linear.getTargetVelocity() / conversion; }
  void setMaxVelocity(units::radians_per_second_t max) { linear.setMaxVelocity(max * conversion); }
  units::radians_per_second_t getMaxVelocity() const { return linear.getMaxVelocity() / conversion; }
  void setInverted(bool isInverted) { linear.setInverted(isInverted); }
  bool getInverted() const { return linear.getInverted(); }
  void disable() { linear.disable(); }
  void stop() { linear.stop(); }
 private:
  LinearVelocityController& linear;
  ConversionUnit_t conversion;
};

} // namespace rmb
