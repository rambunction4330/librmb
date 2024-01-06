#pragma once

#include <limits>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/feedforward/SimpleFeedforward.h"

namespace rmb {

namespace SparkMaxPositionControllerHelper {
struct MotorConfig {
  int id;
  rev::CANSparkMax::MotorType motorType =
      rev::CANSparkMax::MotorType::kBrushless;
  bool inverted = false;

  units::ampere_t currentLimit = 40.0_A;
};

struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
  units::turn_t tolerance = 0.0_rad;
  double iZone = 0.0, iMaxAccumulator = 0.0;
  double maxOutput = 1.0, minOutput = -1.0;
};

struct Range {
  units::radian_t minPosition =
      -std::numeric_limits<units::radian_t>::infinity();
  units::radian_t maxPosition =
      std::numeric_limits<units::radian_t>::infinity();
  bool isContinuous = false;
};

struct ProfileConfig {
  bool useSmartMotion = false;
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
  rev::SparkMaxPIDController::AccelStrategy accelStrategy =
      rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal;
};

enum EncoderType { HallSensor, Quadrature, Alternate, Absolute };
enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

struct FeedbackConfig {
  double gearRatio = 1.0;
  EncoderType encoderType = HallSensor;
  int countPerRev = 42;
  LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
};
} // namespace SparkMaxPositionControllerHelper

/**
 * A wrapper around the SparkMax motorcontroller that allows for the user to set
 * and get the position of the motor accurately through PID functionallity
 */
class SparkMaxPositionController : public AngularPositionController {
public:
  using MotorConfig = SparkMaxPositionControllerHelper::MotorConfig;
  using PIDConfig = SparkMaxPositionControllerHelper::PIDConfig;
  using Range = SparkMaxPositionControllerHelper::Range;
  using ProfileConfig = SparkMaxPositionControllerHelper::ProfileConfig;
  using EncoderType = SparkMaxPositionControllerHelper::EncoderType;
  using LimitSwitchConfig = SparkMaxPositionControllerHelper::LimitSwitchConfig;
  using FeedbackConfig = SparkMaxPositionControllerHelper::FeedbackConfig;

  struct CreateInfo {
    const MotorConfig motorConfig;
    const PIDConfig pidConfig = {};
    const std::shared_ptr<Feedforward<units::radians>> feedforward =
        std::make_shared<SimpleFeedforward<units::radians>>();
    const Range range = {};
    const ProfileConfig profileConfig = {};
    const FeedbackConfig feedbackConfig = {};
    std::initializer_list<const MotorConfig> followers;
  };

  SparkMaxPositionController(SparkMaxPositionController &&) = delete;
  SparkMaxPositionController(const SparkMaxPositionController &) = delete;

  SparkMaxPositionController(const CreateInfo &createInfo);

  //--------------------
  // Controller Methods
  //--------------------

  /**
   * Setting the target position.
   *
   * @param position The target position in radians.
   */
  void setPosition(units::radian_t position) override;

  /**
   * Gets the target position.
   *
   * @return The target position in radians.
   */
  units::radian_t getTargetPosition() const override;

  /**
   * Common interface for setting a mechanism's raw power output.
   */
  virtual void setPower(double power) override;

  /**
   * Retrieve the percentage [0.0, 1.0] output of the motor
   */
  virtual double getPower() const override;

  /**
   * Gets the minimum position.
   *
   * @return The minimum position in radians.
   */
  units::radian_t getMinPosition() const override;

  /**
   * Gets the maximum position.
   *
   * @return The maximum position in radians.
   */
  units::radian_t getMaxPosition() const override;

  /**
   * Disables the motor.
   */
  void disable() override;

  /**
   * Stops the motor until `setPosition` is called again.
   */
  void stop() override;

  //-----------------
  // Encoder Methods
  //-----------------

  /**
   * Gets the velocity of the motor.
   *
   * @return The velocity of the motor in radians per second.
   */
  units::radians_per_second_t getVelocity() const override;

  /**
   * Gets the position of the motor.
   *
   * @return The position of the motor in radians.
   */
  units::radian_t getPosition() const override;

  /**
   * Set the position the motor reports
   *
   * @param position The new motor position
   */
  void setEncoderPosition(units::radian_t position = 0_rad) override;

  //-----------------------------
  // Feedbakc Controller Methods
  //-----------------------------

  /**
   * Gets a controllers tolerance
   *
   * @return tolerance in radians
   */
  units::radian_t getTolerance() const override;

private:
  rev::CANSparkMax sparkMax;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;
  rev::CANSparkMax::ControlType controlType;

  rev::SparkMaxPIDController pidController;
  units::radian_t targetPosition;
  units::radian_t tolerance;

  const std::shared_ptr<Feedforward<units::radians>> feedforward;

  units::radian_t minPose;
  units::radian_t maxPose;

  std::unique_ptr<rev::MotorFeedbackSensor> encoder;
  EncoderType encoderType;
  double gearRatio;
};
} // namespace rmb
