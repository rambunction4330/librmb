
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
  virtual void setPosition(units::radian_t position) = 0;

  /**
   * Common interface for getting the <b>target</b> angular position.
   * 
   * @return The target angular position in radians.
   */
  virtual units::radian_t getTargetPosition() const = 0;

  /**
   * Common interface for setting the minimum angular position.
   * 
   * @param min The minimum angular position in radians.
   */
  virtual void setMinPosition(units::radian_t min) = 0;

  /**
   * Common interface for getting the minimum angular position.
   * 
   * @return The minimum angular position in radians.
   */
  virtual units::radian_t getMinPosition() const = 0;

  /**
   * Common interface for setting the maximum angular position.
   * 
   * @param max  The maximum angular position in radians.
   */
  virtual void setMaxPosition(units::radian_t max) = 0;

  /**
   * Common interface for getting the maximum angular position.
   * 
   * @return The maximum angular position in radians.
   */
  virtual units::radian_t getMaxPosition() const = 0;

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

  void setPosition(units::meter_t position) { angular.setPosition(position / conversion); }
  units::meter_t getTargetPosition() const { return angular.getTargetPosition() * conversion; }
  void setMinPosition(units::meter_t min) { angular.setMinPosition(min / conversion); }
  units::meter_t getMinPosition() const { return angular.getMinPosition() * conversion; }
  void setMaxPosition(units::meter_t max) { angular.setMaxPosition(max / conversion); }
  units::meter_t getMaxPosition() const { return angular.getMaxPosition() * conversion; }
  void setInverted(bool isInverted) { angular.setInverted(isInverted); }
  bool getInverted() const {angular.getInverted(); }
  void disable() { angular.disable(); }
  void stop() { angular.stop(); }

private:
  AngularPositionController& angular;
  ConversionUnit_t conversion;
};

} // namespace rmb
