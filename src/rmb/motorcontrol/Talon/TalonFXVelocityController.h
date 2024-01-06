#pragma once

#include <optional>

#include "rmb/motorcontrol/AngularVelocityController.h"

#include "TalonFXPositionController.h"
#include "units/angular_velocity.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

namespace rmb {
namespace TalonFXVelocityControllerHelper {
struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;

  units::second_t rampRate = 1.0_s;
};

struct OpenLoopConfig {
  double minOutput = -1.0, maxOutput = 1.0;
  units::second_t rampRate = 1.0_s;
};

struct ProfileConfig {
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
};
} // namespace TalonFXVelocityControllerHelper

class TalonFXVelocityController : public AngularVelocityController {
public:
  struct CreateInfo {
    TalonFXPositionControllerHelper::MotorConfig config;
    TalonFXVelocityControllerHelper::PIDConfig pidConfig;
    TalonFXVelocityControllerHelper::ProfileConfig profileConfig;
    TalonFXPositionControllerHelper::FeedbackConfig feedbackConfig;
    TalonFXVelocityControllerHelper::OpenLoopConfig openLoopConfig;
    TalonFXPositionControllerHelper::CurrentLimits currentLimits;
    std::optional<TalonFXPositionControllerHelper::CANCoderConfig>
        canCoderConfig;
  };

  TalonFXVelocityController(const CreateInfo &createInfo);

  virtual ~TalonFXVelocityController() = default;

  //--------------------------------------------------
  // Methods Inherited from AngularVelocityController
  //--------------------------------------------------

  /**
   * Sets the target angular velocity.
   *
   * @param velocity The target angular velocity in radians per second.
   */
  void setVelocity(units::radians_per_second_t velocity) override;

  /**
   * Gets the target angular velocity.
   *
   * @return The target angular velocity in radians per second.
   */
  units::radians_per_second_t getTargetVelocity() const override;

  /**
   * Common interface for setting a mechanism's raw power output.
   */
  virtual void setPower(double power) override;

  /**
   * Retrieve the percentage [-1.0, 1.0] output of the motor
   */
  virtual double getPower() const override;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() override;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() override;

  //---------------------------------------
  // Methods Inherited from AngularEncoder
  //---------------------------------------

  /**
   * Gets the angular velocity of the motor.
   *
   * @return The velocity of the motor in radians per second.
   */
  units::radians_per_second_t getVelocity() const override;

  /**
   * Gets the angular position of an motor.
   *
   * @return The position of the motor in radians.
   */
  units::radian_t getPosition() const override;

  /**
   * Sets the encoder's reported position
   * @param position The position to reset the reference to. Defaults to 0
   */
  void setEncoderPosition(units::radian_t position = 0_rad) override;

  //----------------------------------------------------------
  // Methods Inherited from AngularvelocityFeedbackController
  //----------------------------------------------------------

  /**
   * Gets the motor's tolerance.
   *
   * @return the motor's tolerance in radians per second.
   */
  virtual units::radians_per_second_t getTolerance() const override {
    return 0.0_rad_per_s;
  }

  /**
   * Match duty cycle of the given controller
   * @param inverted Use the inverted form of the parent's duty cycle
   */
  void follow(const TalonFXVelocityController &parentController, bool inverted);

private:
  mutable ctre::phoenix6::hardware::TalonFX motorcontroller;

  units::radians_per_second_t tolerance = 0.0_tps;

  TalonFXVelocityControllerHelper::ProfileConfig profileConfig;

  const bool usingCANCoder;

  mutable std::optional<ctre::phoenix6::hardware::CANcoder> canCoder;
};

} // namespace rmb
