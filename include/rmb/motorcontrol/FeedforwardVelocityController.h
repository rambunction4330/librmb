
#pragma once

#include <frc/motorcontrol/MotorController.h>

#include "rmb/motorcontrol/VelocityController.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {

/**
 * Interface for setting a motor controllers velocity using a feedfoward. 
 * <b>Beware<\b> that since there is no feedbakc device several functions will 
 * behave incorrectly. Additionaly, an `update` method may need to be added for
 * proper voltage compenstation.
 */
template <typename DistanceUnit>
class FeedforwardVelocityController : public VelocityController<DistanceUnit> {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;

  FeedforwardVelocityController(const FeedforwardVelocityController&) = delete;
  FeedforwardVelocityController(FeedforwardVelocityController&&) = default;

  FeedforwardVelocityController(std::unique_ptr<frc::MotorController> motorController, 
                                std::unique_ptr<rmb::Feedforward<DistanceUnit> feedforward) : 
                                motorController(motorController), feedforward(feedforward) {}

  /**
   * Sets the target velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  void setVelocity(Velocity_t velocity) {
    targetVelocity = velocity;
    motorController.SetVoltage(feedforward->calculate(targetVelocity));
  }

  /**
   * Since there is no feedbakc device this fucntion will always return the same as 
   * `getTarget Velocity`.
   */
  Velocity_t getVelocity() const { return targetVelocity; }

  /**
   * Gets the <b>target</b> velocity.
   * 
   * @return The <b>target</b> velocity in meters per second.
   */
  Velocity_t getTargetVelocity() const { return targetVelocity; }

  /**
   * Sicne there is no feedback devise this fucntion will always return true.
   */
  bool atTarget() const { return true; }

  /**
   * Since this motor controller has no feedback device, this function will 
   * always returns zero. Do not use it in cases where actual position is needed.
   * 
   * Considering adding position estimation in the future, though it would be 
   * so inaccurate and faulty that is likly useless.
   */
  Distance_t getPosition() const { return Distance_t(0); }

  /**
   *Inverterts the direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  void setInverted(bool isInverted) { motorController.SetInverted(isInverted); }

  /**
   * Returns the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  bool getInverted() const { motorController.GetInverted(); }

  /**
   * Disabls a mechanism.
   */
  void disable() { 
    targetVelocity = Velocity_t(0.0);
    motorController.Disable(); 
  }

  /**
   * Stops the mechanism until `setVelocity` is called again.
   */
  void stop() { 
    targetVelocity = Velocity_t(0.0);
    motorController.StopMotor(); 
  }

private:
  std::unique_ptr<frc::MotorController> motorController;
  std::unique_ptr<rmb::Feedforward<DistanceUnit>> feedforward;
  Velocity_t targetVelocity = Velocity_t(0.0);
};
} // namespace rmb
