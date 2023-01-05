
#pragma once

#include <units/angular_velocity.h>

#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's angular velocity used by wrappers of 
 * device specific APIs.
 */
class AngularVelocityController {
public:

  /**
   * Common interface for setting the target angular velocity.
   * 
   * @param velocity The target angular velocity in radians per second.
   */
  virtual void setVelocity(units::radians_per_second_t velocity) = 0;

  /**
   * Common interface for getting the <b>target</b> angular velocity.
   * 
   * @return The <b>target</b> angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getTargetVelocity() const = 0;

  /**
   * Common interface for setting the maximum angular velocity.
   * 
   * @param max The maximum angular velocity in radians per second.
   */
  virtual void setMaxVelocity(units::radians_per_second_t max) = 0;

  /**
   * Common interface for getting the maximum angular velocity.
   * 
   * @return The maximum angular velocity in radianss per second.
   */
  virtual units::radians_per_second_t getMaxVelocity() const = 0;

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
   * Generates a `LinearVelocityController` to controll the same mechanism
   * as this controller, but with linear instead of angular units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they control the same physical mechanism.
   * 
   * @param conversion conversion factor from linear to angular units.
   */
  AngularAsLinearVelocityController asLinearController(ConversionUnit_t conversion) {
    return AngularAsLinearVelocityController(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class AngularAsLinearVelocityController: public LinearVelocityController {
 public:
  
  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  AngularAsLinearVelocityController(const AngularAsLinearVelocityController&) = default;
  AngularAsLinearVelocityController(AngularAsLinearVelocityController&&) = delete;

  AngularAsLinearVelocityController(AngularVelocityController& angularController, 
                                    ConversionUnit_t conversionFactor) :
                                    angular(angularController), conversion(conversionFactor) {}

  void setVelocity(units::meters_per_second_t velocity) { angular.setVelocity(velocity / conversion); }
  units::meters_per_second_t getTargetVelocity() const { return angular.getTargetVelocity() * conversion; }
  void setMaxVelocity(units::meters_per_second_t max) { angular.setMaxVelocity(max / conversion); }
  units::meters_per_second_t getMaxVelocity() const { return angular.getMaxVelocity() * conversion; }
  void setInverted(bool isInverted) { angular.setInverted(isInverted); }
  bool getInverted() const { return angular.getInverted(); }
  void disable() { angular.disable(); }
  void stop() { angular.stop(); }
 private:
  AngularVelocityController& angular;
  units::unit_t<units::compound_unit<units::radians, units::inverse<units::meters>>> conversion;
};

} // namespace rmb
