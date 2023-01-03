
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
   * Common interface for getting the <b>current</b> linear velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() const = 0;

  /**
   * Common interface for getting the <b>target</b> linear velocity.
   * 
   * @return The <b>target</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getTargetVelocity() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity error.
   * 
   * @return The diffrence between the actual and target linear velocitys.
   */
  virtual units::meters_per_second_t getError() const {
    return getVelocity() - getTargetVelocity();
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * velocity. No defualt implementation is given  since an equality check is 
   * meaningless as some non zero error will always exist.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  virtual units::meter_t getPosition() const = 0;

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
  LinearAsAngularVelocityController asAngular(ConversionUnit_t conversion) {
    return LinearAsAngularVelocityController(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class LinearAsAngularVelocityController: public AngularVelocityController {
 public:
  LinearAsAngularVelocityController(LinearVelocityController& linearController, 
                                    ConversionUnit_t conversionFactor) :
                                    linear(linearController), conversion(conversionFactor) {}

  void setAngularVelocity(units::radians_per_second_t velocity) { linear.setVelocity(velocity / conversion); }
  units::radians_per_second_t getAngularVelocity() const  { return linear.getVelocity() * conversion; }
  units::radians_per_second_t getTargetAngularVelocity() const { return linear.getTargetVelocity() * conversion; }
  bool atTarget() const { return linear.atTarget(); }
  units::radian_t getAngularPosition() const { return linear.getPosition() * conversion; }
  void setInverted(bool isInverted) { linear.setInverted(isInverted); }
  bool getInverted() const { return linear.getInverted(); }
  void disable() { linear.disable(); }
  void stop() { linear.stop(); }
 private:
  LinearVelocityController& linear;
  ConversionUnit_t conversion;
};

} // namespace rmb
