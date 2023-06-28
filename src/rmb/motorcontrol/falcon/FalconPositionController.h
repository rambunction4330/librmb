#pragma once

#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/WPI_CANCoder.h"

#include "units/base.h"
#include "units/current.h"
#include "units/time.h"
#include <optional>

namespace rmb {
namespace FalconPositionControllerHelper {
struct MotorConfig {
  int id;
  bool inverted = false;
  units::ampere_t currentLimit = 40_A;
};

struct OpenLoopConfig {
  double minOutput = -1.0, maxOutput = 1.0;
  units::second_t rampRate = 1.0_s;
};

struct PIDConfig {
  double p = 0.0, i = 0.0, d = 0.0, ff = 0.0;
  units::turn_t tolerance = 0.0_rad;
  double iZone = 0.0, iMaxAccumulator = 0.0;
  double closedLoopMaxPercentOutput = 1.0;

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
  bool useSmartMotion =
      false; /*< Note to future users: Don't use smart motion!*/
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
};

enum LimitSwitchConfig { Disabled, NormalyOpen, NormalyClosed };

struct FeedbackConfig {
  double gearRatio = 1.0;
  LimitSwitchConfig forwardSwitch = Disabled, reverseSwitch = Disabled;
};

struct CANCoderConfig {
  enum RemoteSlot { RemoteSlot0 = 0, RemoteSlot1 = 1 };

  bool useCANCoder = false;
  int id;

  int remoteSensorSlot; /*< Can be 0 or 1 depending on RemoteSlot0 or
                           RemoteSlot1*/
};

} // namespace FalconPositionControllerHelper
class FalconPositionController : public AngularPositionController {
public:
  //-------------Integrated Encoder
  // Units-----------------------------------------
  typedef units::unit<std::ratio<1, 2048>, units::turns> IntegratedEncoderTick;
  typedef units::unit_t<IntegratedEncoderTick> IntegratedEncoderTick_t;

  typedef units::compound_unit<IntegratedEncoderTick,
                               units::inverse<units::deciseconds>>
      RawVelocityUnit;
  typedef units::unit_t<RawVelocityUnit> RawIntegratedVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, IntegratedEncoderTick> RawPositionUnit;
  typedef units::unit_t<RawPositionUnit> RawIntegratedPositionUnit_t;

  //-------------CANCoder
  // Units---------------------------------------------------
  typedef units::unit<std::ratio<1, 4096>, units::turns> CANCoderTick;
  typedef units::unit_t<CANCoderTick> CANCoderTick_t;

  typedef units::compound_unit<CANCoderTick, units::inverse<units::deciseconds>>
      RawCANCoderVelocityUnit;
  typedef units::unit_t<RawCANCoderVelocityUnit> RawCANCoderVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, CANCoderTick> RawCANCoderPositionUnit;
  typedef units::unit_t<RawCANCoderPositionUnit> RawCANCoderPositionUnit_t;

  struct CreateInfo {
    FalconPositionControllerHelper::MotorConfig config;
    FalconPositionControllerHelper::PIDConfig pidConfig;
    FalconPositionControllerHelper::Range range;
    FalconPositionControllerHelper::FeedbackConfig feedbackConfig;
    FalconPositionControllerHelper::OpenLoopConfig openLoopConfig;
    FalconPositionControllerHelper::CANCoderConfig canCoderConfig;
  };

  /**
   * Creates a Falcon position FalconPositionController
   * @param createInfo CreateInfo struct used to initialize the position
   * controller
   */
  FalconPositionController(const CreateInfo &createInfo);

  /**
   * Sets a closed loop position setpoint on the falcon to the given position
   * @param position The position setpoint
   */
  void setPosition(units::radian_t position) override;

  /**
   * Sets open loop power on the motor
   * @param power The power target supplied to the motor. Must be in range
   * [0.0, 1.0]
   */
  void setPower(double power);

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
   * Get the closed loop position tolerance
   * @return The tolerance in a units::angle container
   */
  units::radian_t getTolerance() const override;

private:
  mutable ctre::phoenix::motorcontrol::can::WPI_TalonFX motorcontroller;

  std::optional<ctre::phoenix::sensors::WPI_CANCoder> canCoder;

  FalconPositionControllerHelper::Range range;

  float gearRatio = 0.0;

  units::radian_t offset = 0_rad;

  units::radian_t tolerance = 0.0_rad;

  const bool usingCANCoder;
};
} // namespace rmb
