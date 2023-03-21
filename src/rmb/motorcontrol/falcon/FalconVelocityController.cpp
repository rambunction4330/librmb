#include "FalconVelocityController.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "units/angular_velocity.h"
#include <iostream>

namespace rmb {

FalconVelocityController::FalconVelocityController(
    const FalconVelocityController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id),
      usingCANCoder(createInfo.canCoderConfig.useCANCoder) {

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
      0, RawInternalVelocityUnit_t(createInfo.pidConfig.tolerance)());
  motorcontroller.ConfigClosedLoopPeakOutput(
      0, createInfo.pidConfig.closedLoopMaxPercentOutput);
  motorcontroller.ConfigClosedloopRamp(createInfo.pidConfig.rampRate());

  motorcontroller.Config_IntegralZone(0, createInfo.pidConfig.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(
      0, createInfo.pidConfig.iMaxAccumulator);

  motorcontroller.ConfigForwardSoftLimitEnable(
      createInfo.feedbackConfig.forwardSwitch);

  if (createInfo.canCoderConfig.useCANCoder) {
    canCoder.emplace(createInfo.canCoderConfig.id);

    ctre::phoenix::motorcontrol::FeedbackDevice device;
    switch (createInfo.canCoderConfig.remoteSensorSlot) {
    case 0:
      device = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
      break;
    case 1:
      device = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor1;
      break;
    default:
      std::cerr << "invalid remote sensor ID! Must be 0 or 1!" << std::endl;
      return;
      break;
    }

    motorcontroller.ConfigRemoteFeedbackFilter(
        canCoder.value(), createInfo.canCoderConfig.remoteSensorSlot);
    motorcontroller.ConfigSelectedFeedbackSensor(device, 0, 0);
  }

  gearRatio = createInfo.feedbackConfig.gearRatio;
  tolerance = createInfo.pidConfig.tolerance;
  this->profileConfig = createInfo.profileConfig;
}

void FalconVelocityController::setVelocity(
    units::radians_per_second_t velocity) {
  units::radians_per_second_t targetVelocity(velocity);

  if (velocity > profileConfig.maxVelocity) {
    targetVelocity = profileConfig.maxVelocity;
  } else if (velocity < profileConfig.minVelocity) {
    targetVelocity = profileConfig.minVelocity;
  }

  if (usingCANCoder) {
    motorcontroller.Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity,
        (RawCANCoderVelocityUnit_t(targetVelocity * gearRatio))());
  } else {
    motorcontroller.Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity,
        (RawInternalVelocityUnit_t(targetVelocity * gearRatio))());
  }
}

units::radians_per_second_t
FalconVelocityController::getTargetVelocity() const {
  if (usingCANCoder) {
    return RawCANCoderVelocityUnit_t(motorcontroller.GetClosedLoopTarget() /
                                     gearRatio);
  } else {
    return RawInternalVelocityUnit_t(motorcontroller.GetClosedLoopTarget() /
                                     gearRatio);
  }
}

units::radians_per_second_t FalconVelocityController::getVelocity() const {
  if (usingCANCoder) {
    return RawCANCoderVelocityUnit_t(
        motorcontroller.GetSelectedSensorVelocity() / gearRatio);
  } else {
    return RawInternalVelocityUnit_t(
        motorcontroller.GetSelectedSensorVelocity() / gearRatio);
  }
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
  if (usingCANCoder) {
    return RawCANCoderPositionUnit_t(
        motorcontroller.GetSelectedSensorPosition() / gearRatio);
  } else {
    return RawInternalPositionUnit_t(
        motorcontroller.GetSelectedSensorPosition() / gearRatio);
  }
}

void FalconVelocityController::zeroPosition(units::radian_t offset) {
  if (usingCANCoder) {
    motorcontroller.SetSelectedSensorPosition(
        RawCANCoderPositionUnit_t(offset * gearRatio)());
  } else {
    motorcontroller.SetSelectedSensorPosition(
        RawInternalPositionUnit_t(offset * gearRatio)());
  }
}

} // namespace rmb
