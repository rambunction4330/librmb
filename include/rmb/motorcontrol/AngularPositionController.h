
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include "rmb/motorcontrol/LinearPositionController.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's angular position used by wrappers of 
 * device specific APIs.
 */
class AngularPositionController {
public:

  /**
   * Common interface for setting the target angular position. 
   * 
   * @param position The target angular position in radians.
   */
  virtual void setAngularPosition(units::radian_t position) = 0;

  /**
   * Common interface for getting the <b>current</b> angular position.
   * 
   * @return the <b>current</b> angular position in radians.
   */
  virtual units::radian_t getAngularPosition() const = 0;

  /**
   * Common interface for getting the <b>target</b> angular position.
   * 
   * @return The <b>target</b> angular position in meters.
   */
  virtual units::radian_t getTargetAngularPosition() const = 0;

  /**
   * Common interface for getting the <b>current</b> angular velocity error. 
   * 
   * @return The diffrence between the actual angular velocity and target 
   *         angular velocity in radians.
   */
  units::radian_t getAngularError() const {
    return getAngularPosition() - getTargetAngularPosition();
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for getting the <b>current</b> angular veolcity.
   * 
   * @return the <b>current</b> angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getAngularVelocity() const = 0;

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
   * Common interface to stop the mechanism until `setPosition` is called again.
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
  AngularAsLinearPositionController asLinear(ConversionUnit_t conversion) {
    return AngularAsLinearPositionController(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class AngularAsLinearPositionController : public LinearPositionController {
public:
  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  AngularAsLinearPositionController(AngularPositionController& angularController, 
                                    ConversionUnit_t conversionFactor) :
                                    angular(angularController), conversion(conversionFactor) {}

  void setPosition(units::meter_t position) { angular.setAngularPosition(position / conversion); }
  units::meter_t getPosition() const { return angular.getAngularPosition() * conversion; }
  units::meter_t getTargetPosition() const { return angular.getTargetAngularPosition() * conversion; }
  bool atTarget() const { return angular.atTarget(); }
  units::meters_per_second_t getVelocity() const { return angular.getAngularVelocity() * conversion; }
  void setInverted(bool isInverted) { angular.setInverted(isInverted); }
  bool getInverted() const {angular.getInverted(); }
  void disable() { angular.disable(); }
  void stop() { angular.stop(); }

private:
  AngularPositionController& angular;
  ConversionUnit_t conversion;
};

} // namespace rmb
