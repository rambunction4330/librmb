
#pragma once

#include <frc/motorcontrol/MotorController.h>

#include "rmb/motorcontrol/LinearVelocityController.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"

namespace rmb {

/**
 * Interface for setting a motor controllers linear velocity using a feedfoward. 
 * <b>Beware<\b> that since there is no feedbakc device several functions will 
 * behave incorrectly. Additionaly, an `update` method may need to be added for
 * proper voltage compenstation.
 */
class LinearFeedforwardController : public LinearVelocityController{
public:

  LinearFeedforwardController(const LinearFeedforwardController&) = delete;
  LinearFeedforwardController(LinearFeedforwardController&&) = default;

  LinearFeedforwardController(std::unique_ptr<frc::MotorController>&& motorController, 
                              std::unique_ptr<rmb::Feedforward<units::meters>>&& feedforward) : 
                              motorController(std::move(motorController)), feedforward(std::move(feedforward)) {}

  /**
   * Sets the target velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  void setVelocity(units::meters_per_second_t velocity) {
    targetVelocity = velocity;
    motorController->SetVoltage(feedforward->calculate(targetVelocity));
  }

  /**
   * Since there is no feedbakc device this fucntion will always return the same as 
   * `getTarget Velocity`.
   */
  units::meters_per_second_t getVelocity() const { return targetVelocity; }

  /**
   * Gets the <b>target</b> velocity.
   * 
   * @return The <b>target</b> velocity in meters per second.
   */
  units::meters_per_second_t getTargetVelocity() const { return targetVelocity; }

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
  units::meter_t getPosition() const { return 0.0_m; }

  /**
   *Inverterts the direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  void setInverted(bool isInverted) { motorController->SetInverted(isInverted); }

  /**
   * Returns the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  bool getInverted() const { motorController->GetInverted(); }

  /**
   * Disabls a mechanism.
   */
  void disable() { 
    targetVelocity = 0.0_mps;
    motorController->Disable(); 
  }

  /**
   * Stops the mechanism until `setVelocity` is called again.
   */
  void stop() { 
    targetVelocity = 0.0_mps;
    motorController->StopMotor(); 
  }

private:
  std::unique_ptr<frc::MotorController> motorController;
  std::unique_ptr<rmb::Feedforward<units::meters>> feedforward;
  units::meters_per_second_t targetVelocity = 0.0_mps;
};
} // namespace rmb
