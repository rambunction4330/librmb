
#pragma once

#include <functional>
#include <assert.h>

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace rmb {

/**
 * Interface for controlling a mechanisms velocity used by wrappers of device 
 * specific APIs. The interface is meant to descibe what a mechanism should do,
 * not how should do it. Methods are given in both linear and angular units but 
 * controll the same mechanism, so changes to the target linear velocity also 
 * modify the target angular velocity and vice versa. Linear methods have been 
 * given default implintations that rely on angular methods under the assumtion 
 * that the mechanism is powered by a motor. If this is not the case, 
 * re-implement or mark methods unimplemented as nessesary.
 * 
 * @see SparkMaxVelocityController TalonSRXVelocityController 
 *      FalconVelocityController FeedforwardVelocityController
 */
class VelocityController {
public:

  /**
   * Common interface for setting the target linear velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  virtual void setVelocity(units::meters_per_second_t velocity);

  /**
   * Common interface for setting the target angular velocity.
   * 
   * @param velocity The target angular velocity in radians per second.
   */
  virtual void setAngularVelocity(units::radians_per_second_t velocity) = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() const;

  /**
   * Common interface for getting the <b>current</b> angular velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> angular velocity in radians per second
   */
  virtual units::radians_per_second_t getAngularVelocity() const = 0;

  /**
   * Common interface for getting the <b>target</b> linear velocity.
   * 
   * @return The <b>target</b> linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getTargetVelocity() const;

  /**
   * Common interface for getting the <b>target</b> angular velocity.
   * 
   * @return The <b>target</b> angular velocity in radians per second.
   */
  virtual units::radians_per_second_t getTargetAngularVelocity() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity error.
   * 
   * @return The diffrence between the actual and target linear velocitys.
   */
  virtual units::meters_per_second_t getError() const;

  /**
   * Common interface for getting the <b>current</b> angular velocity error.
   * 
   * @return The diffrence between the actual and target angular velocitys.
   */
  virtual units::radians_per_second_t getAngularError() const;

  /**
   * Common interface for returning whether the mechanism is at the target 
   * velocity.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for setting the maximum desired linear velocity.
   * 
   * @param maxVelocity The maximum linear velocity of a mechanism.
   */
  virtual void setMaxVelocity(units::radians_per_second_t maxVelocity);

  /**
   * Common interface for setting the maximum desired angular velocity.
   * 
   * @param maxVelocity The maximum angular velocity of a mechanism.
   */
  virtual void setMaxAngularVelocity(units::radians_per_second_t maxVelocity) = 0;

  /**
   * Common interface for getting the maximum desired linear velocity.
   * 
   * @return The maximum linear velocity of a mechanism.
   */
  virtual units::radians_per_second_t getMaxVelocity() const;

  /**
   * Common interface for getting the maximum desired angular velocity.
   * 
   * @return The maximum angular velocity of a mechanism.
   */
  virtual units::radians_per_second_t getMaxAngularVelocity() const = 0;

  /**
   * Common interface for converting from linear to angular units. A functions
   * is used rather than a conversion factor since some conversions may be
   * non-linear. 
   * 
   * @param linear The linear distance in meters.
   * @return The angular distance in radians.
   */
  virtual units::radians_per_second_t linearToAngular(units::meters_per_second_t linear) const = 0;

  /**
   * Common interface for converting from angular to linear units. A functions
   * is used rather than a conversion factor since some conversions may be
   * non-linear. 
   * 
   * @param angle The linear distance in radians.
   * @return The linear distance in meters.
   */
  virtual units::meters_per_second_t angularToLinear(units::radians_per_second_t angle) const  = 0;

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
  virtual void stopMotor() = 0;
};
} // namespace rmb
