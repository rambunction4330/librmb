#include "FalconPositionController.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"
#include "units/angle.h"

namespace ctre {
using namespace phoenix::motorcontrol::can;
}

namespace rmb {

FalconPositionController::FalconPositionController(const FalconPositionController::CreateInfo& createInfo)
    : motorcontroller(createInfo.config.id), range(createInfo.range) {

  motorcontroller.SetInverted(createInfo.config.inverted);

  motorcontroller.ConfigPeakOutputForward(0, createInfo.openLoopConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(0, createInfo.openLoopConfig.minOutput);

  motorcontroller.Config_kD(0, createInfo.pidConfig.d);
  motorcontroller.Config_kI(0, createInfo.pidConfig.i);
  motorcontroller.Config_kP(0, createInfo.pidConfig.p);
  motorcontroller.Config_kF(0, createInfo.pidConfig.ff);
  motorcontroller.ConfigAllowableClosedloopError(
      0, RawPositionUnit_t(createInfo.pidConfig.tolerance)());
  motorcontroller.Config_IntegralZone(0, createInfo.pidConfig.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(0, createInfo.pidConfig.iMaxAccumulator);
  motorcontroller.ConfigClosedLoopPeakOutput(0, createInfo.pidConfig.closedLoopMaxPercentOutput);

  motorcontroller.ConfigForwardSoftLimitEnable(createInfo.feedbackConfig.forwardSwitch);

  ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currentConfig{}; 
  currentConfig.currentLimit = createInfo.config.currentLimit();
  currentConfig.enable = true;
  currentConfig.triggerThresholdTime = 0;
  currentConfig.triggerThresholdCurrent = 0.0;
  motorcontroller.ConfigStatorCurrentLimit(currentConfig);


  gearRatio = createInfo.feedbackConfig.gearRatio;
  tolerance = createInfo.pidConfig.tolerance;
}

void FalconPositionController::setPosition(units::radian_t position) {
  RawPositionUnit_t targetPosition(position);

  if (targetPosition > range.maxPosition) {
    targetPosition = range.maxPosition;
  } else if (targetPosition < range.minPosition) {
    targetPosition = range.minPosition;
  }

  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                      (targetPosition * gearRatio)());
}

units::radian_t FalconPositionController::getTargetPosition() const {
  return RawPositionUnit_t(motorcontroller.GetClosedLoopTarget()) / gearRatio;
}

units::radian_t FalconPositionController::getMinPosition() const {
  return range.minPosition;
}

units::radian_t FalconPositionController::getMaxPosition() const {
  return range.maxPosition;
}

void FalconPositionController::disable() { motorcontroller.Disable(); }

void FalconPositionController::stop() { motorcontroller.StopMotor(); }

units::radians_per_second_t FalconPositionController::getVelocity() const {
  return RawVelocityUnit_t(motorcontroller.GetSelectedSensorVelocity() /
                           gearRatio);
}

units::radian_t FalconPositionController::getPosition() const {
  return RawPositionUnit_t(motorcontroller.GetSelectedSensorPosition() /
                           gearRatio);
}

void FalconPositionController::zeroPosition(units::radian_t offset) {
  motorcontroller.SetSelectedSensorPosition(
      RawPositionUnit_t(offset * gearRatio)());
}

units::radian_t FalconPositionController::getTolerance() const {
  return tolerance;
}

} // namespace rmb
