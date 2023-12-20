#include "TalonFXVelocityController.h"
#include "units/angular_velocity.h"
#include <iostream>

namespace rmb {

TalonFXVelocityController::TalonFXVelocityController(
    const TalonFXVelocityController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id, "rio"),
      usingCANCoder(createInfo.canCoderConfig.useCANCoder) {

  /*motorcontroller.ConfigFactoryDefault();

  motorcontroller.SetInverted(createInfo.config.inverted);

  motorcontroller.ConfigPeakOutputForward(createInfo.openLoopConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(createInfo.openLoopConfig.minOutput);
  motorcontroller.ConfigOpenloopRamp(createInfo.openLoopConfig.rampRate());

  ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration currentConfig{};
  currentConfig.currentLimit = createInfo.config.currentLimit();
  currentConfig.enable = true;
  currentConfig.triggerThresholdTime = 0;
  currentConfig.triggerThresholdCurrent = 0.0;
  motorcontroller.ConfigStatorCurrentLimit(currentConfig);

  motorcontroller.SetNeutralMode(
      ctre::phoenix::motorcontrol::NeutralMode::Brake);

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
  }*/
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
      40.0; // Allow infinite current
  talonFXConfig.CurrentLimits.SupplyTimeThreshold =
      0.5; // But wait for this time
  talonFXConfig.CurrentLimits.SupplyCurrentThreshold =
      50.0; // After exceed this current
  talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  talonFXConfig.CurrentLimits.StatorCurrentLimit =
      createInfo.config.currentLimit(); // Motor-usage current limit
                                        // Prevent heat

  if (createInfo.canCoderConfig.useCANCoder) {
    canCoder.emplace(createInfo.canCoderConfig.id);

    ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig{};

    canCoderConfig.MagnetSensor.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue(
            ctre::phoenix6::signals::SensorDirectionValue::
                CounterClockwise_Positive);
    canCoderConfig.MagnetSensor.AbsoluteSensorRange =
        ctre::phoenix6::signals::AbsoluteSensorRangeValue(
            ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1);

    canCoderConfig.MagnetSensor.MagnetOffset =
        units::turn_t(createInfo.canCoderConfig.zeroPosition)();

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

  gearRatio = createInfo.feedbackConfig.sensorToMechanismRatio;
  tolerance = createInfo.pidConfig.tolerance;
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

  /*if (usingCANCoder) {
    motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity,
                        (RawCANCoderVelocityUnit_t(targetVelocity))());
  } else {
    motorcontroller.Set(
        ctre::phoenix::motorcontrol::ControlMode::Velocity,
        (RawInternalVelocityUnit_t(targetVelocity * gearRatio))());
  }*/

  // units::millisecond_t start = frc::Timer::GetFPGATimestamp();
  motorcontroller.SetControl(
      ctre::phoenix6::controls::VelocityDutyCycle(velocity));
  // std::cout << "velocity duty cycle request: " << ((units::millisecond_t)
  // frc::Timer::GetFPGATimestamp() - start)() << std::endl;
}

units::radians_per_second_t
TalonFXVelocityController::getTargetVelocity() const {
  // if (usingCANCoder) {
  //   return RawCANCoderVelocityUnit_t(motorcontroller.GetClosedLoopTarget());
  // } else {
  //   return RawInternalVelocityUnit_t(motorcontroller.GetClosedLoopTarget() /
  //                                    gearRatio);
  // }
  return units::turns_per_second_t(
      motorcontroller.GetClosedLoopReference().GetValue());
}

units::radians_per_second_t TalonFXVelocityController::getVelocity() const {
  // if (usingCANCoder) {
  //   return RawCANCoderVelocityUnit_t(
  //       motorcontroller.GetSelectedSensorVelocity());
  // } else {
  //   return RawInternalVelocityUnit_t(
  //       motorcontroller.GetSelectedSensorVelocity() / gearRatio);
  // }

  if (usingCANCoder) {
    return canCoder->GetVelocity().GetValue();
  } else {
    return motorcontroller.GetVelocity().GetValue();
  }
}

units::radians_per_second_t TalonFXVelocityController::getTolerance() const {
  return tolerance;
}

void TalonFXVelocityController::setPower(double power) {
  motorcontroller.Set(power);
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

void TalonFXVelocityController::zeroPosition(units::radian_t offset) {
  // if (usingCANCoder) {
  //   canCoder->SetPosition(RawCANCoderPositionUnit_t(offset)());
  // } else {
  //   motorcontroller.SetSelectedSensorPosition(
  //       RawInternalPositionUnit_t(offset * gearRatio)());
  // }
  if (usingCANCoder) {
    ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig{};

    canCoder->GetConfigurator().Refresh(canCoderConfig);
    canCoderConfig.MagnetSensor.MagnetOffset = ((units::turn_t)offset)();

    canCoder->GetConfigurator().Apply(canCoderConfig);
  } else {
    ctre::phoenix6::configs::FeedbackConfigs config{};
    motorcontroller.GetConfigurator().Refresh(config);

    config.FeedbackRotorOffset = ((units::turn_t)offset)();

    motorcontroller.GetConfigurator().Apply(config);
  }
}
} // namespace rmb
// namespace rmb
