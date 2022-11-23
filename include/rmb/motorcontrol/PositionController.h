#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace rmb {

/**
 * Interface for controlling motor position used by wrappers of device specific 
 * APIs. The interface is meant to descibe what a motor should do, but not how 
 * should do it. Methods are given in both linear and angular units but controll 
 * the same motor, so changes to the target linear velocity also modify the 
 * target angular velocity and vice versa.
 * 
 * @see SparkMaxPositionController TalonSRXPositionController 
 *      FalconPositionController
 */
class PositionController {
public:

  /**
   * Common interface for setting the target linear position.
   * 
   * @param position The target linear position in meters.
   */
  virtual void setPosition(units::meter_t position) = 0;

  /**
   * Common interface for setting the target angular position.
   * 
   * @param position The target angular position in radians.
   */
  virtual void setAngularPosition(units::radian_t position) = 0;

  /**
   * Common interface for setting the target linear position as offset from the 
   * previouse target.
   * 
   * @param position The offset from the previouse target in meters.
   */
  virtual void setPositionOffset(units::meter_t position) = 0;

  /**
   * Common interface for setting the target angular position as offset from the 
   * previouse target.
   * 
   * @param position The offset from the previouse target in radians.
   */
  virtual void setAngularPositionOffset(units::radian_t position) = 0;

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  virtual units::meter_t getPosition() const = 0;

  /**
   * Common interface for getting the <b>current</b> angular position.
   * 
   * @return the <b>current</b> angular position in radians.
   */
  virtual units::radian_t getAngularPosition() const = 0;

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  virtual units::meter_t getTargetPosition() const = 0;

  /**
   * Common interface for getting the <b>target</b> angular position.
   * 
   * @return The <b>target</b> angular position in meters.
   */
  virtual units::radian_t getTargetAngularPosition() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear position error.
   * 
   * @return The diffrence between the actual linear position and target linear 
   *         position in meters.
   */
  virtual units::meter_t getError() = 0;

  /**
   * Common interface for getting the <b>current</b> angular velocity error.
   * 
   * @return The diffrence between the actual angular velocity and target 
   *         angular velocity in radians.
   */
  virtual units::radian_t getAngularError() = 0;

  /**
   * Common interface for returning whetther the motor is at the target position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() = 0;

  /**
   * Common interface for getting the current linear velocity.
   * 
   * @return The current linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() = 0;

  /**
   * Common interface for getting the current angular velocity.
   * 
   * @return The current angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getAngularVelocity() = 0;

  /**
   * Common interface for zeroing the linear position such that the 
   * <b>target</b> linear position is remapped to the given position. 
   * 
   * @param position The values the <b>target</b> linear position will be 
   *                 remapped to.
   */
  virtual void zeroPosition(units::meter_t position = 0_m) = 0;

  /**
   * Common interface for zeroing the angular position such that the 
   * <b>target</b> angular position is remapped to the given position. 
   * 
   * @param position The values the <b>target</b> angular position will be
   *                 remapped to.
   */
  virtual void zeroAngularPosition(units::radian_t position = 0_rad) = 0;

  /**
   * Common interface for setting the maximum linear position.
   * 
   * @param max The maximum linear position in meters.
   */
  virtual void setMaxPosition(units::meter_t max) = 0;

  /**
   * Common interface for setting the maximum angular position.
   * 
   * @param max The maximum angular position in radians.
   */
  virtual void setMaxAngularPosition(units::radian_t max) = 0;

  /**
   * Common interface for getting the maximum linear position.
   * 
   * @return The maximum linear position in meters.
   */
  virtual units::meter_t getMaxPosition() = 0;

  /**
   * Common interface for getting the maximum angular position.
   * 
   * @return The maximum angular position in radians.
   */
  virtual units::radian_t getMaxAngularPosition() = 0;

  /**
   * Common interface for setting the minimum linear position.
   * 
   * @param min The minimum linear position in meters.
   */
  virtual void setMinPosition(units::meter_t min) = 0;

  /**
   * Common interface for setting the minimum angular position.
   * 
   * @param min The minimum angular position in radians.
   */
  virtual void setMinAngularPosition(units::radian_t min) = 0;

  /**
   * Common interface for getting the minimum linear position.
   * 
   * @return The minimum linear position in meters.
   */
  virtual units::meter_t getMinPosition() = 0;

    /**
   * Common interface for sgetting the minimum angular position.
   * 
   * @return The minimum angular position in radians.
   */
  virtual units::radian_t getMinAngularPosition() = 0;

  /**
   * Common interface for setting the maximum linear velocity.
   * 
   * @param max The maximum linear velocity in meters per second.
   */
  virtual void setMaxVelocity(units::meters_per_second_t max) = 0;

  /**
   * Common interface for setting the maximum angular velocity.
   * 
   * @param max The maximum angular velocity in radians per second.
   */
  virtual void setMaxAngularVelocity(units::radians_per_second_t max) = 0;

  /**
   * Common interface for getting the maximum linear velocity.
   * 
   * @return The maximum linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getMaxVelocity() = 0;

  /**
   * Common interface for getting the maximum angular velocity.
   * 
   * @return The maximum angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getMaxAngularVelocity() = 0;

  /**
   * Common interface for setting the maximum linear acceleration.
   * 
   * @param max The maximum linear acceleration in meters per second squared.
   */
  virtual void setMaxAcceleration(units::meters_per_second_squared_t max) = 0;

  /**
   * Common interface for setting the maximum angular acceleration.
   * 
   * @param max The maximum angular acceleration in radians per second squared.
   */
  virtual void setMaxAngularAcceleration(units::radians_per_second_squared_t max) = 0;

  /**
   * Common interface for getting the maximum linear acceleration.
   * 
   * @return The maximum linear acceleration in meters per second squared.
   */
  virtual units::meters_per_second_squared_t getMaxAcceleration() = 0;

  /**
   * Common interface for getting the maximum angular acceleration.
   * 
   * @return The maximum angular acceleration in radians per second squared.
   */
  virtual units::radians_per_second_squared_t getMaxAngularAcceleration() = 0;

  /**
   * Common interface for setting the coversion factor from linear to angular units.
   * 
   * @param conversionFactor The conversion factor from linear to angular units (such as a wheel
   *                         diameter or gear ratio)
   */
  virtual void setConversion(units::compound_unit<units::radians, units::inverse<units::meters>> conversionFactor) = 0;

  /**
   * Common interface for getting the coversion factor from linear to angular units.
   * 
   * @return The conversion factor from linear to angular units (such as a wheel diameter or gear ratio)
   */
  virtual units::compound_unit<units::radians, units::inverse<units::meters>> getConversion() = 0;

    /**
   * Common interface for inverting direction of a motor.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setInverted(bool isInverted) = 0;

    /**
   * Common interface for returning the inversion state of a motor.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  virtual bool getInverted() const = 0;

  /**
   * Common interface for disabling a motor.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the motor until `SetPosition` is called again.
   */
  virtual void stopMotor() = 0;
};

} // namespace rmb
