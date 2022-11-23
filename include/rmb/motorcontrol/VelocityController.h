
#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc/motorcontrol/MotorController.h>

namespace rmb {

/**
 * Interface for controlling motor velocity used by wrappers of device specific 
 * APIs. The interface is meant to descibe what a motor should do, but not how 
 * should do it. Methods are given in both linear and angular units but controll 
 * the same motor, so changes to the target linear velocity also modify the 
 * target angular velocity and vice versa.
 * 
 * @see SparkMaxVelocityController TalonSRXVelocityController 
 *      FalconVelocityController
 */
class VelocityController {
public:

  /**
   * Common interface for setting the target linear velocity.
   * positon
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  virtual void setVelocity(units::meters_per_second_t velocity) = 0;

  /**
   * Common interface for setting the target angular velocity.
   * 
   * @param velocity The target angular velocity in radians per second.
   */
  virtual void setAngularVelocity(units::radians_per_second_t velocity) = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity of the
   * motor regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() = 0;

  /**
   * Common interface for getting the <b>current</b> angular velocity of the
   * motor regardless of target.
   * 
   * @return The <b>current</b> angular velocity in radians per second
   */
  virtual units::radians_per_second_t getAngularVelocity() = 0;

  /**
   * Common interface for getting the <b>target</b> linear velocity.
   * 
   * @return The <b>target</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getTargetVelocity() = 0;

  /**
   * Common interface for getting the <b>target</b> angular velocity.
   * 
   * @return The <b>target</b> angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getTargetAngularVelocity() = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity error.
   * 
   * @return The diffrence between the actual linear velocity and target linear velocity.
   */
  virtual units::meters_per_second_t getError() = 0;

  /**
   * Common interface for getting the <b>current</b> angular velocity error.
   * 
   * @return The diffrence between the actual angular velocity and target angular velocity.
   */
  virtual units::radians_per_second_t getAngularError() = 0;

  /**
   * Common interface for returning whetther the motor is at the target velocity.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() = 0;

  /**
   * Common interface for setting the maximum desired linear velocity.
   * 
   * @param maxVelocity The maximum linear velocity of a motor.
   */
  virtual void setMaxVelocity(units::radians_per_second_t maxVelocity) = 0;

  /**
   * Common interface for setting the maximum desired angular velocity.
   * 
   * @param maxVelocity The maximum angular velocity of a motor.
   */
  virtual void setMaxAngularVelocity(units::radians_per_second_t maxVelocity) = 0;

  /**
   * Common interface for getting the maximum desired linear velocity.
   * 
   * @return The maximum linear velocity of a motor.
   */
  virtual units::radians_per_second_t getMaxVelocity() = 0;

  /**
   * Common interface for getting the maximum desired angular velocity.
   * 
   * @return The maximum angular velocity of a motor.
   */
  virtual units::radians_per_second_t getMaxAngularVelocity() = 0;

  /**
   * Common interface for setting the maximum desired linear accleration.
   * 
   * @param maxAcceleration The maximum linear acceleration of a motor.
   */
  virtual void setMaxAcceleration(units::meters_per_second_squared_t maxAcceleration) = 0;

  /**
   * Common interface for setting the maximum desired angular accleration.
   * 
   * @param maxAcceleration The maximum angular acceleration of a motor.
   */
  virtual void setMaxAngularAcceleration(units::radians_per_second_squared_t maxAcceleration) = 0;

  /**
   * Common interface for getting the maximum desired linear accleration.
   * 
   * @return The maximum linear acceleration of a motor.
   */
  virtual units::meters_per_second_squared_t getMaxAcceleration() = 0;

  /**
   * Common interface for getting the maximum desired angular accleration.
   * 
   * @return The maximum angular acceleration of a motor.
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
   * Common interface to stop the motor until `setVelocity` is called again.
   */
  virtual void stopMotor() = 0;
};
} // namespace rmb
