#include "TalonFXPositionController.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"
#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/sensors/CANCoder.h"
#include "ctre/phoenix/sensors/SensorTimeBase.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "units/angle.h"
#include <iostream>

namespace ctre {
using namespace phoenix::motorcontrol::can;
}

namespace rmb {

TalonFXPositionController::TalonFXPositionController(
    const TalonFXPositionController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id), range(createInfo.range),
      usingCANCoder(createInfo.canCoderConfig.useCANCoder) {

  motorcontroller.ConfigFactoryDefault();

  motorcontroller.SetInverted(createInfo.config.inverted);

  setPIDConstants(createInfo.pidConfig);

  motorcontroller.ConfigForwardSoftLimitEnable(
      createInfo.feedbackConfig.forwardSwitch);

  ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currentConfig{};
  currentConfig.currentLimit = createInfo.config.currentLimit();
  currentConfig.enable = true;
  currentConfig.triggerThresholdTime = 0;
  currentConfig.triggerThresholdCurrent = 0.0;
  motorcontroller.ConfigStatorCurrentLimit(currentConfig);

  if (createInfo.canCoderConfig.useCANCoder) {
    canCoder.emplace(createInfo.canCoderConfig.id);
    canCoder->ConfigFactoryDefault();
    canCoder->ConfigAbsoluteSensorRange(
        ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);
    canCoder->ConfigFeedbackCoefficient(
        1.0, "ticks", ctre::phoenix::sensors::SensorTimeBase::PerSecond);

    FeedbackDevice device;
    switch (createInfo.canCoderConfig.remoteSensorSlot) {
    case 0:
      device = FeedbackDevice::RemoteSensor0;
      break;
    case 1:
      device = FeedbackDevice::RemoteSensor1;
      break;
    default:
      std::cerr << "invalid remote sensor ID! Must be 0 or 1!" << std::endl;
      return;
      break;
    }

    motorcontroller.ConfigRemoteFeedbackFilter(
        canCoder.value(), createInfo.canCoderConfig.remoteSensorSlot);
    motorcontroller.ConfigSelectedFeedbackSensor(device, 0, 0);
    motorcontroller.ConfigIntegratedSensorAbsoluteRange(
        ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);
    motorcontroller.ConfigSelectedFeedbackCoefficient(1.0f);
  }

  motorcontroller.ConfigFeedbackNotContinuous(!createInfo.range.isContinuous);

  if (usingCANCoder) {
    motorcontroller.ConfigAllowableClosedloopError(
        0, RawIntegratedPositionUnit_t(createInfo.pidConfig.tolerance)());
  } else {
    motorcontroller.ConfigAllowableClosedloopError(
        0, RawCANCoderPositionUnit_t(createInfo.pidConfig.tolerance)());
  }

  gearRatio = createInfo.feedbackConfig.gearRatio;
  tolerance = createInfo.pidConfig.tolerance;
}
// end of constructor

void TalonFXPositionController::setPosition(units::radian_t position) {
  units::radian_t targetPosition(position);

  if (targetPosition > range.maxPosition) {
    targetPosition = range.maxPosition;
  } else if (targetPosition < range.minPosition) {
    targetPosition = range.minPosition;
  }

  if (usingCANCoder) {
    motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                        (RawCANCoderPositionUnit_t(targetPosition))());
  } else {
    motorcontroller.Set(
        ctre::phoenix::motorcontrol::ControlMode::Position,
        (RawIntegratedPositionUnit_t(targetPosition) * gearRatio)());
  }
}

units::radian_t TalonFXPositionController::getTargetPosition() const {
  if (usingCANCoder) {
    return (RawCANCoderPositionUnit_t(motorcontroller.GetClosedLoopTarget()));
  } else {
    return (RawIntegratedPositionUnit_t(
               motorcontroller.GetClosedLoopTarget())) /
           gearRatio;
  }
}

units::radian_t TalonFXPositionController::getMinPosition() const {
  return range.minPosition;
}

units::radian_t TalonFXPositionController::getMaxPosition() const {
  return range.maxPosition;
}

void TalonFXPositionController::disable() { motorcontroller.Disable(); }

void TalonFXPositionController::stop() { motorcontroller.StopMotor(); }

units::radians_per_second_t TalonFXPositionController::getVelocity() const {
  if (usingCANCoder) {
    return RawCANCoderVelocityUnit_t(
        motorcontroller.GetSelectedSensorVelocity());
  } else {
    return RawIntegratedVelocityUnit_t(
        motorcontroller.GetSelectedSensorVelocity() / gearRatio);
  }
}

units::radian_t TalonFXPositionController::getPosition() const {
  if (usingCANCoder) {
    return RawCANCoderPositionUnit_t(
        motorcontroller.GetSelectedSensorPosition());
  } else {
    return RawIntegratedPositionUnit_t(
        motorcontroller.GetSelectedSensorPosition() / gearRatio);
  }
}

void TalonFXPositionController::zeroPosition(units::radian_t offset) {
  if (usingCANCoder) {
    canCoder->SetPosition(RawCANCoderPositionUnit_t(offset)());
  } else {
    motorcontroller.SetSelectedSensorPosition(
        RawIntegratedPositionUnit_t(offset * gearRatio)());
  }
}

units::radian_t TalonFXPositionController::getTolerance() const {
  return tolerance;
}

void TalonFXPositionController::setPower(double power) {
  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                      power);
}

void TalonFXPositionController::setPIDConstants(PIDConfig config) {

  motorcontroller.ConfigPeakOutputForward(config.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(config.minOutput);
  motorcontroller.ConfigOpenloopRamp(config.rampRate);
  motorcontroller.ConfigClosedloopRamp(config.rampRate);

  motorcontroller.Config_kD(0, config.d);
  motorcontroller.Config_kI(0, config.i);
  motorcontroller.Config_kP(0, config.p);
  motorcontroller.Config_kF(0, config.ff);

  motorcontroller.ConfigAllowableClosedloopError(
      0, 0); // RawIntegratedPositionUnit_t(createInfo.pidConfig.tolerance)());
  motorcontroller.Config_IntegralZone(0, config.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(0, config.iMaxAccumulator);
  motorcontroller.ConfigClosedLoopPeakOutput(0, config.maxOutput);
}

} // namespace rmb
