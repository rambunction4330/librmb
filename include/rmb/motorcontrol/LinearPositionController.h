
#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include "rmb/motorcontrol/AngularPositionController.h"


namespace rmb {

/**
 * Interface for controlling a mechanism's linear position used by wrappers of 
 * device specific APIs.
 */
class LinearPositionController {
public:

  /**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  virtual void setPosition(units::meter_t position) = 0;

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  virtual units::meter_t getPosition() const = 0;

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  virtual units::meter_t getTargetPosition() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear position error.
   * 
   * @return The diffrence between the actual linear position and target linear 
   *         position in meters.
   */
  units::meter_t getError() const {
    return getPosition() - getTargetPosition();
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() const = 0;

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
  LinearAsAngularPositionController asAngular(ConversionUnit_t conversion) {
    return LinearAsAngularPositionController(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class LinearAsAngularPositionController : public AngularPositionController {
public:
  LinearAsAngularPositionController(LinearPositionController& linearController, 
                                    ConversionUnit_t conversionFactor) :
                                    linear(linearController), conversion(conversionFactor) {}

  void setAngularPosition(units::radian_t position) { linear.setPosition(position / conversion); }
  units::radian_t getAngularPosition() const { return linear.getPosition() * conversion; }
  units::radian_t getTargetAngularPosition() const { return linear.getTargetPosition() * conversion; }
  bool atTarget() const { return linear.atTarget(); }
  units::radians_per_second_t getAngularVelocity() const { return linear.getVelocity() * conversion; }
  void setInverted(bool isInverted) { linear.setInverted(isInverted); }
  bool getInverted() const {linear.getInverted(); }
  void disable() { linear.disable(); }
  void stop() { linear.stop(); }

private:
  LinearPositionController& linear;
  ConversionUnit_t conversion;
};

} // namespace rmb
