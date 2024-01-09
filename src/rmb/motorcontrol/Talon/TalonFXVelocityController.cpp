#include "TalonFXVelocityController.h"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"
#include "units/angular_velocity.h"

#include <iostream>

namespace rmb {

TalonFXVelocityController::TalonFXVelocityController(
    const TalonFXVelocityController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id, "rio"),
      usingCANCoder(createInfo.canCoderConfig.has_value()) {
  auto &configurator = motorcontroller.GetConfigurator();

  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfig{};

  talonFXConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue(createInfo.config.inverted);
  // TODO: make sure these fields (maxOutput, minOutput) are moved to a
  // different struct because they are for both closed and open loop
  // control in the new API
  talonFXConfig.MotorOutput.PeakForwardDutyCycle =
      createInfo.openLoopConfig.maxOutput;
  talonFXConfig.MotorOutput.PeakReverseDutyCycle =
      createInfo.openLoopConfig.minOutput;
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
  // of the mechanism.
  talonFXConfig.Feedback.SensorToMechanismRatio =
      createInfo.feedbackConfig.sensorToMechanismRatio;
  // talonFXConfig.Feedback.SensorToMechanismRatio; // For fused CANCoders
  // But we can't use this firmware feature because CTRE are capitalist pigs
  // and we (as of writing) don't feel like paying for v6 Pro

  configurator.Apply(talonFXConfig);

  this->profileConfig = createInfo.profileConfig;
}

void TalonFXVelocityController::setVelocity(
    units::radians_per_second_t velocity) {
  units::radians_per_second_t targetVelocity(velocity);

  if (velocity > profileConfig.maxVelocity) {
    targetVelocity = profileConfig.maxVelocity;
  } else if (velocity < profileConfig.minVelocity) {
    targetVelocity = profileConfig.minVelocity;
  }

  // units::millisecond_t start = frc::Timer::GetFPGATimestamp();
  motorcontroller.SetControl(
      ctre::phoenix6::controls::VelocityDutyCycle(velocity));
}

units::radians_per_second_t
TalonFXVelocityController::getTargetVelocity() const {
  return units::turns_per_second_t(
      motorcontroller.GetClosedLoopReference().GetValue());
}

units::radians_per_second_t TalonFXVelocityController::getVelocity() const {
  if (usingCANCoder) {
    return canCoder->GetVelocity().GetValue();
  } else {
    return motorcontroller.GetVelocity().GetValue();
  }
}

void TalonFXVelocityController::setPower(double power) {
  motorcontroller.SetControl(ctre::phoenix6::controls::DutyCycleOut(power));
  // motorcontroller.Set(power);

  // std::cout << "power: " << power;
  // std::cout << "also power: " << getPower() << std::endl;
}

double TalonFXVelocityController::getPower() const {
  return motorcontroller.Get();
}

void TalonFXVelocityController::disable() { motorcontroller.Disable(); }

void TalonFXVelocityController::stop() { motorcontroller.StopMotor(); }

units::radian_t TalonFXVelocityController::getPosition() const {
  if (usingCANCoder) {
    return canCoder->GetPosition().GetValue();
  } else {
    return motorcontroller.GetPosition().GetValue();
  }
}

void TalonFXVelocityController::setEncoderPosition(units::radian_t position) {
  if (canCoder.has_value()) {
    canCoder->SetPosition(position);
  } else {
    motorcontroller.SetPosition(position);
  }
}

void TalonFXVelocityController::follow(
    const rmb::TalonFXVelocityController &parent, bool invert) {
  motorcontroller.SetControl(ctre::phoenix6::controls::Follower(
      parent.motorcontroller.GetDeviceID(), invert));
}
} // namespace rmb
