#pragma once

#include <initializer_list>

#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>

#include "rmb/motorcontrol/AngularVelocityController.h"

namespace rmb {

namespace SparkMaxVelocityControllerHelper {
struct MotorConfig {
  int id;
  rev::CANSparkMax::MotorType motorType =
      rev::CANSparkMax::MotorType::kBrushless;
  bool inverted = false;

  units::ampere_t currentLimit = 40.0_A;
  units::second_t openLoopRampRate = 0.5_s;
};

struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
  units::radians_per_second_t tolerance = 0.0_rad_per_s;
  double iZone = 0.0, iMaxAccumulator = 0.0;
  double maxOutput = 1.0, minOutput = -1.0;
};

struct ProfileConfig {
  bool useSmartMotion = false;
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
  rev::SparkMaxPIDController::AccelStrategy accelStrategy =
      rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal;

  units::second_t closedLoopRampRate = 0.5_s;
};

enum EncoderType { HallSensor, Quadrature, Alternate, Absolute };
enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

struct FeedbackConfig {
  double gearRatio = 1.0;
  EncoderType encoderType = HallSensor;
  int countPerRev = 42;
  LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
};
} // namespace SparkMaxVelocityControllerHelper

/**
 * A wrapper around the SparkMax motorcontroller so that it can complies with
 * the `AngularVelocityFeedbackController` interface.
 */
class SparkMaxVelocityController : public AngularVelocityController {
public:
  using MotorConfig = SparkMaxVelocityControllerHelper::MotorConfig;
  using PIDConfig = SparkMaxVelocityControllerHelper::PIDConfig;
  using ProfileConfig = SparkMaxVelocityControllerHelper::ProfileConfig;
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;
  using LimitSwitchConfig = SparkMaxVelocityControllerHelper::LimitSwitchConfig;
  using FeedbackConfig = SparkMaxVelocityControllerHelper::FeedbackConfig;

  struct CreateInfo {
    const MotorConfig motorConfig;
    const PIDConfig pidConfig = {};
    const ProfileConfig profileConfig = {};
    const FeedbackConfig feedbackConfig = {};
    std::initializer_list<const MotorConfig> followers;
  };

  SparkMaxVelocityController(SparkMaxVelocityController &&) = delete;
  SparkMaxVelocityController(const SparkMaxVelocityController &) = delete;

  SparkMaxVelocityController(const CreateInfo &createInfo);

  rev::CANSparkMax &getMotor();

  rev::SparkMaxPIDController &getPIDCOntroller();

  std::unique_ptr<rev::MotorFeedbackSensor> &getFeedbackSensor();

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
  virtual units::radians_per_second_t getTolerance() const override;

private:
  rev::CANSparkMax sparkMax;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;
  rev::CANSparkMax::ControlType controlType;

  rev::SparkMaxPIDController pidController;
  units::radians_per_second_t targetVelocity = 0.0_rpm;
  units::radians_per_second_t tolerance;

  std::unique_ptr<rev::MotorFeedbackSensor> encoder;
  EncoderType encoderType;
  double gearRatio;
};
} // namespace rmb
