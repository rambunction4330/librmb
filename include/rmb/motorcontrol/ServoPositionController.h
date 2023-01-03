
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc/Servo.h>

#include "rmb/motorcontrol/AngularPositionController.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's linear position used by wrappers of 
 * device specific APIs.
 */
class ServoPositionController : public AngularPositionController {
public:

  ServoPositionController(const ServoPositionController&) = delete;
  ServoPositionController(ServoPositionController&&) = default;

  ServoPositionController(int channel) : servo(channel) {} 

  /**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  virtual void setAngularPosition(units::radian_t position) {
    return servo.SetAngle(units::degree_t(position).to<double>() * inversion);
  }

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  virtual units::radian_t getAngularPosition() const {
    return units::degree_t(servo.GetAngle() * inversion);
  }

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  virtual units::radian_t getTargetAngularPosition() const {
    return units::degree_t(servo.GetAngle() * inversion);
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const {
    return true;
  }

  /**
   * Common interface for getting the <b>current</b> linear velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  virtual units::radians_per_second_t getAngularVelocity() const {
    return units::radians_per_second_t(0.0);
  }

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setInverted(bool isInverted) {
    inversion = isInverted ? -1 : 1;
  }

    /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  virtual bool getInverted() const {
    return inversion == 1;
  }

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() {
    servo.SetDisabled();
  }

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() {
    servo.SetDisabled();
  }

private:
  frc::Servo servo;
  int inversion = 1;
};
} // namespace rmb
