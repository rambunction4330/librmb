#include "TalonFXPositionController.h"
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "units/angle.h"

#include <iostream>

namespace rmb {

TalonFXPositionController::TalonFXPositionController(
    const TalonFXPositionController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id), range(createInfo.range),
      usingCANCoder(createInfo.canCoderConfig.has_value()) {

  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfig{};

  talonFXConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue(createInfo.config.inverted);
  talonFXConfig.MotorOutput.PeakForwardDutyCycle = createInfo.config.maxOutput;
  talonFXConfig.MotorOutput.PeakReverseDutyCycle = createInfo.config.minOutput;
  // talonFXConfig.MotorOutput.DutyCycleNeutralDeadband; NOTE: use if you want
  // to demote low target percentage outputs to zero
  talonFXConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue(
          createInfo.config.brake
              ? ctre::phoenix6::signals::NeutralModeValue::Brake
              : ctre::phoenix6::signals::NeutralModeValue::Coast);

  talonFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
      createInfo.openLoopConfig.rampRate();
  talonFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
      createInfo.pidConfig.rampRate();

  talonFXConfig.Slot0.kP = createInfo.pidConfig.p;
  talonFXConfig.Slot0.kI = createInfo.pidConfig.i;
  talonFXConfig.Slot0.kD = createInfo.pidConfig.d;
  talonFXConfig.Slot0.kS = createInfo.pidConfig.ff;
  // Izone, maxAccumulator nonexistant in the v6 API "no use for them, so we
  // didn't implement"

  // Currently no way to set allowableClosedLoopError or closedLoopPeakOutput
  // TODO: print out warnings or implement them yourself somehow

  talonFXConfig.HardwareLimitSwitch.ForwardLimitType =
      ctre::phoenix6::signals::ForwardLimitTypeValue::NormallyOpen;
  talonFXConfig.HardwareLimitSwitch.ForwardLimitEnable =
      createInfo.feedbackConfig.forwardSwitch;
  if (createInfo.feedbackConfig.forwardSwitch) {
    std::cout << "Warning: forward limit switches are probably incomplete in "
                 "Librmb. If you are using this and find issues, "
                 "please open an issue on the github or try to fix it yourself."
              << std::endl;
  }

  // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10
  talonFXConfig.CurrentLimits.SupplyCurrentLimit =
      createInfo.currentLimits.supplyCurrentLimit();
  talonFXConfig.CurrentLimits.SupplyTimeThreshold =
      createInfo.currentLimits.supplyTimeThreshold(); // But wait for this time
  talonFXConfig.CurrentLimits.SupplyCurrentThreshold =
      createInfo.currentLimits
          .supplyCurrentThreshold(); // After exceed this current
  talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
      createInfo.currentLimits.supplyCurrentLimitEnable;
  talonFXConfig.CurrentLimits.StatorCurrentLimitEnable =
      createInfo.currentLimits.statorCurrentLimitEnable;
  talonFXConfig.CurrentLimits.StatorCurrentLimit =
      createInfo.currentLimits.statorCurrentLimit(); // Motor-usage current
                                                     // limit Prevent heat

  if (createInfo.canCoderConfig.has_value()) {
    canCoder.emplace(createInfo.canCoderConfig.value().id);

    ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig{};

    canCoderConfig.MagnetSensor.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue(
            ctre::phoenix6::signals::SensorDirectionValue::
                CounterClockwise_Positive);
    canCoderConfig.MagnetSensor.AbsoluteSensorRange =
        ctre::phoenix6::signals::AbsoluteSensorRangeValue(
            ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1);

    canCoderConfig.MagnetSensor.MagnetOffset =
        units::turn_t(createInfo.canCoderConfig.value().magnetOffset)();

    canCoder->GetConfigurator().Apply(canCoderConfig);

    // talonFXConfig.Feedback.RotorToSensorRatio; // This is for FusedCANCoder
    talonFXConfig.Feedback.WithRemoteCANcoder(canCoder.value());
  } else {
    talonFXConfig.Feedback.FeedbackSensorSource =
        ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  }

  // Often there is a gear ratio between the motor's rotor and the actual output
  // of the mechanism
  talonFXConfig.Feedback.SensorToMechanismRatio =
      createInfo.feedbackConfig.sensorToMechanismRatio;
  // talonFXConfig.Feedback.RotorToSensorRatio =// For fused CANCoders
  //     createInfo.feedbackConfig.sensorToMechanismRatio;
  // talonFXConfig.Feedback.SensorToMechanismRatio;
  // But we can't use this firmware feature because CTRE are capitalist pigs
  // and we (as of writing) don't feel like paying for v6 Pro

  talonFXConfig.ClosedLoopGeneral.ContinuousWrap =
      createInfo.range.continuousWrap;

  motorcontroller.GetConfigurator().Apply(talonFXConfig);

  sensorToMechanismRatio = createInfo.feedbackConfig.sensorToMechanismRatio;
  // tolerance = createInfo.pidConfig.tolerance;
}

void TalonFXPositionController::setPosition(units::radian_t position) {
  units::radian_t targetPosition(position);

  targetPosition =
      std::clamp(targetPosition, range.minPosition, range.maxPosition);

  ctre::phoenix6::controls::PositionDutyCycle request(targetPosition);

  motorcontroller.SetControl(request);
}

units::radian_t TalonFXPositionController::getTargetPosition() const {
  return units::turn_t(motorcontroller.GetClosedLoopReference().GetValue());
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
    return canCoder->GetVelocity().GetValue();
  } else {
    return motorcontroller.GetVelocity().GetValue();
  }
}

units::radian_t TalonFXPositionController::getPosition() const {
  if (usingCANCoder) {
    return canCoder->GetPosition().GetValue();
  } else {
    return motorcontroller.GetPosition().GetValue();
  }
}

void TalonFXPositionController::setEncoderPosition(units::radian_t position) {
  if (canCoder.has_value()) {
    canCoder->SetPosition(position);
  } else {
    motorcontroller.SetPosition(position);
  }
}

void TalonFXPositionController::setPower(double power) {
  motorcontroller.Set(power);
}

double TalonFXPositionController::getPower() const {
  return motorcontroller.Get();
}

void TalonFXPositionController::follow(
    const rmb::TalonFXPositionController &parent, bool invert) {
  motorcontroller.SetControl(ctre::phoenix6::controls::Follower(
      parent.motorcontroller.GetDeviceID(), invert));
}

} // namespace rmb
