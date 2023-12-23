#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "rmb/motorcontrol/AngularPositionController.h"

#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/current.h"
#include "units/time.h"

#include <optional>

namespace rmb {
namespace TalonFXPositionControllerHelper {
struct MotorConfig {
  int id;
  bool inverted = false;
  bool brake = false;
  units::ampere_t currentLimit = 40_A; /*< Stator current limit*/
  double minOutput = -1.0,
         maxOutput = 1.0; /*< Applies to both open and closed loop*/
};

struct OpenLoopConfig {
  units::second_t rampRate = 0.0_s;
};

struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
  // units::turn_t tolerance = 0.0_rad; /*< Can't find a way to implement in the
  // new Pheonix V6 API*/ double closedLoopMaxPercentOutput = 1.0; /*< Can't
  // find a way to implement in the new Pheonix V6 API*/

  units::second_t rampRate = 0.0_s; /*< Amount of time it takes to go from 0 to
                                       full throttle. 0_s disables*/
};

struct Range {
  units::radian_t minPosition =
      -std::numeric_limits<units::radian_t>::infinity();
  units::radian_t maxPosition =
      std::numeric_limits<units::radian_t>::infinity();

  bool isContinuous =
      true; /*< If false, the encoder will keep track of overflow, meaning the
                position value of the internal encoder, say will go from 2047 to
                2048 to 2049 as you keep turning it. Otherwise it will go from
                2047 to 0*/
};

struct ProfileConfig {
  bool useSmartMotion = /*< Note to future users: Don't use smart motion!*/
      false;
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
};

enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

struct FeedbackConfig {
  double sensorToMechanismRatio = 1.0;
  LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
};

struct CANCoderConfig {
  int id;

  units::radian_t zeroPosition = 0.0_rad;
};

} // namespace TalonFXPositionControllerHelper
class TalonFXPositionController : public AngularPositionController {
public:
  struct CreateInfo {
    TalonFXPositionControllerHelper::MotorConfig config;
    TalonFXPositionControllerHelper::PIDConfig pidConfig;
    TalonFXPositionControllerHelper::Range range;
    TalonFXPositionControllerHelper::FeedbackConfig feedbackConfig;
    TalonFXPositionControllerHelper::OpenLoopConfig openLoopConfig;
    std::optional<TalonFXPositionControllerHelper::CANCoderConfig>
        canCoderConfig;
  };

  /**
   * Creates a TalonFX position TalonFXPositionController
   * @param createInfo CreateInfo struct used to initialize the position
   * controller
   */
  TalonFXPositionController(const CreateInfo &createInfo);

  virtual ~TalonFXPositionController() = default;

  /**
   * Sets a closed loop position setpoint on the TalonFX to the given position
   * @param position The position setpoint
   */
  void setPosition(units::radian_t position) override;

  /**
   * Sets open loop power on the motor
   * @param power The power target supplied to the motor. Must be in range
   * [0.0, 1.0]
   */
  void setPower(double power) override;

  /**
   * Queries the Phoenix API for the current set point of the motor
   */
  units::radian_t getTargetPosition() const override;

  /**
   * Get the minium position of the motor
   * @return The minimum position in a units::angle value
   */
  units::radian_t getMinPosition() const override;

  /**
   * Get the maximum position of the motor
   * @return The maximum position of the motor in a units::angle value
   */
  units::radian_t getMaxPosition() const override;

  /**
   * Disables the motor
   */
  void disable() override;

  /**
   * Stops the motor
   */
  void stop() override;

  /**
   * Get the current velocity of the motor
   * @return The velocity of the motor in a units::angular_velocity container
   */
  units::radians_per_second_t getVelocity() const override;

  /**
   * Get the current position of the motor
   * @return The current position of the motor as measured by the selected
   *         sensor
   */
  units::radian_t getPosition() const override;

  /**
   * Sets the reference position
   * @param offset The position to reset the reference to. Defaults to 0
   */
  void zeroPosition(units::radian_t offset = 0_rad) override;

  /**
   * Get the closed loop position tolerance.
   * @warn Tolerance is unimplemented and will return 0
   * \return The tolerance in a units::angle container
   */
  units::radian_t getTolerance() const override { return 0.0_tr; }

  /**
   * Match duty cycle of the given controller
   * @param inverted Use the inverted form of the parent's duty cycle
   */
  void follow(const TalonFXPositionController &parentController, bool inverted);

private:
  // mutable ctre::phoenix::motorcontrol::can::WPI_TalonFX motorcontroller;
  mutable ctre::phoenix6::hardware::TalonFX motorcontroller;

  // std::optional<ctre::phoenix::sensors::WPI_CANCoder> canCoder;
  mutable std::optional<ctre::phoenix6::hardware::CANcoder> canCoder;

  TalonFXPositionControllerHelper::Range range;

  float sensorToMechanismRatio = 0.0;

  units::radian_t offset = 0_rad;

  const bool usingCANCoder;
};
} // namespace rmb
