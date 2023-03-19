#include "FalconVelocityController.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "units/angular_velocity.h"

namespace rmb {

FalconVelocityController::FalconVelocityController(
    const FalconVelocityController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id) {

  motorcontroller.SetInverted(createInfo.config.inverted);

  motorcontroller.ConfigPeakOutputForward(0,
                                          createInfo.openLoopConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(0,
                                          createInfo.openLoopConfig.minOutput);
  motorcontroller.ConfigOpenloopRamp(createInfo.openLoopConfig.rampRate());

  ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currentConfig{};
  currentConfig.currentLimit = createInfo.config.currentLimit();
  currentConfig.enable = true;
  currentConfig.triggerThresholdTime = 0;
  currentConfig.triggerThresholdCurrent = 0.0;
  motorcontroller.ConfigStatorCurrentLimit(currentConfig);

  motorcontroller.Config_kD(0, createInfo.pidConfig.d);
  motorcontroller.Config_kI(0, createInfo.pidConfig.i);
  motorcontroller.Config_kP(0, createInfo.pidConfig.p);
  motorcontroller.Config_kF(0, createInfo.pidConfig.ff);
  motorcontroller.ConfigAllowableClosedloopError(
      0, RawVelocityUnit_t(createInfo.pidConfig.tolerance)());
  motorcontroller.ConfigClosedLoopPeakOutput(
      0, createInfo.pidConfig.closedLoopMaxPercentOutput);
  motorcontroller.ConfigClosedloopRamp(createInfo.pidConfig.rampRate());

  motorcontroller.Config_IntegralZone(0, createInfo.pidConfig.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(
      0, createInfo.pidConfig.iMaxAccumulator);

  motorcontroller.ConfigForwardSoftLimitEnable(
      createInfo.feedbackConfig.forwardSwitch);

  gearRatio = createInfo.feedbackConfig.gearRatio;
  tolerance = createInfo.pidConfig.tolerance;
  this->profileConfig = createInfo.profileConfig;
}

void FalconVelocityController::setVelocity(
    units::radians_per_second_t velocity) {
  RawVelocityUnit_t targetVelocity(velocity);

  if (velocity > profileConfig.maxVelocity) {
    targetVelocity = profileConfig.maxVelocity;
  } else if (velocity < profileConfig.minVelocity) {
    targetVelocity = profileConfig.minVelocity;
  }

  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity,
                      (targetVelocity * gearRatio)());
}

units::radians_per_second_t
FalconVelocityController::getTargetVelocity() const {
  return RawVelocityUnit_t(motorcontroller.GetClosedLoopTarget() / gearRatio);
}

units::radians_per_second_t FalconVelocityController::getVelocity() const {
  return RawVelocityUnit_t(motorcontroller.GetSelectedSensorVelocity() /
                           gearRatio);
}

units::radians_per_second_t FalconVelocityController::getTolerance() const {
  return tolerance;
}

void FalconVelocityController::setPower(double power) {
  motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                      power);
}

void FalconVelocityController::disable() { motorcontroller.Disable(); }

void FalconVelocityController::stop() { motorcontroller.StopMotor(); }

units::radian_t FalconVelocityController::getPosition() const {
  return RawPositionUnit_t(motorcontroller.GetSelectedSensorPosition() /
                           gearRatio);
}

void FalconVelocityController::zeroPosition(units::radian_t offset) {
  motorcontroller.SetSelectedSensorPosition(
      RawPositionUnit_t(offset * gearRatio)());
}

} // namespace rmb
