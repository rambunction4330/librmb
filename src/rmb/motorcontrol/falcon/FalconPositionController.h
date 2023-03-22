#pragma once

#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/feedback/AngularPositionFeedbackController.h"
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

  units::second_t rampRate = 1.0_s;
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
class FalconPositionController : public AngularPositionFeedbackController {
public:
  //-------------Integrated Encoder
  //Units-----------------------------------------
  typedef units::unit<std::ratio<2048, 1>, units::turns> IntegratedEncoderTick;
  typedef units::unit_t<IntegratedEncoderTick> IntegratedEncoderTick_t;

  typedef units::compound_unit<IntegratedEncoderTick,
                               units::inverse<units::deciseconds>>
      RawVelocityUnit;
  typedef units::unit_t<RawVelocityUnit> RawIntegratedVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, IntegratedEncoderTick> RawPositionUnit;
  typedef units::unit_t<RawPositionUnit> RawIntegratedPositionUnit_t;

  //-------------CANCoder
  //Units---------------------------------------------------
  typedef units::unit<std::ratio<4096, 1>, units::turns> CANCoderTick;
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

  FalconPositionController(const CreateInfo &createInfo);

  void setPosition(units::radian_t position) override;

  units::radian_t getTargetPosition() const override;

  units::radian_t getMinPosition() const override;

  units::radian_t getMaxPosition() const override;

  void disable() override;

  void stop() override;

  units::radians_per_second_t getVelocity() const override;

  units::radian_t getPosition() const override;

  void zeroPosition(units::radian_t offset = 0_rad) override;

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
