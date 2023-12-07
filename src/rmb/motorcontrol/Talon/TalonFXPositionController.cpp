#include "TalonFXPositionController.h"
// #include "ctre/phoenix/motorcontrol/ControlMode.h"
/*#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"
#include "ctre/phoenix/sensors/CANCoder.h"
#include "ctre/phoenix/sensors/SensorTimeBase.h"*/
#include "ctre/phoenix6/configs/Configs.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "units/angle.h"
#include <iostream>

namespace rmb {

TalonFXPositionController::TalonFXPositionController(
    const TalonFXPositionController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id, "rio"), range(createInfo.range),
      usingCANCoder(createInfo.canCoderConfig.useCANCoder) {

  /*motorcontroller.ConfigFactoryDefault();

  motorcontroller.SetInverted(createInfo.config.inverted);

  motorcontroller.ConfigPeakOutputForward(createInfo.openLoopConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(createInfo.openLoopConfig.minOutput);
  motorcontroller.ConfigOpenloopRamp(createInfo.openLoopConfig.rampRate());
  motorcontroller.ConfigClosedloopRamp(createInfo.pidConfig.rampRate());

  motorcontroller.SetNeutralMode(
      ctre::phoenix::motorcontrol::NeutralMode::Brake);

  motorcontroller.Config_kD(0, createInfo.pidConfig.d);
  motorcontroller.Config_kI(0, createInfo.pidConfig.i);
  motorcontroller.Config_kP(0, createInfo.pidConfig.p);
  motorcontroller.Config_kF(0, createInfo.pidConfig.ff);
  motorcontroller.ConfigAllowableClosedloopError(
      0, 0); // RawIntegratedPositionUnit_t(createInfo.pidConfig.tolerance)());
  motorcontroller.Config_IntegralZone(0, createInfo.pidConfig.iZone);
  motorcontroller.ConfigMaxIntegralAccumulator(
      0, createInfo.pidConfig.iMaxAccumulator);
  motorcontroller.ConfigClosedLoopPeakOutput(
      0, createInfo.pidConfig.closedLoopMaxPercentOutput);

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
    canCoder->ConfigSensorInitializationStrategy(
        ctre::phoenix::sensors::SensorInitializationStrategy::
            BootToAbsolutePosition);
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
    motorcontroller.ConfigIntegratedSensorInitializationStrategy(
        ctre::phoenix::sensors::SensorInitializationStrategy::
            BootToAbsolutePosition);

    zeroPosition(RawCANCoderPositionUnit_t(canCoder->GetAbsolutePosition()) -
                 createInfo.canCoderConfig.zeroPosition);
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
  tolerance = createInfo.pidConfig.tolerance;*/

  auto &configurator = motorcontroller.GetConfigurator();

  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfig{};

  talonFXConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue(createInfo.config.inverted);
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
    // Leave offset to offset function

    canCoder->GetConfigurator().Apply(canCoderConfig, 0.0_s);

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

  talonFXConfig.ClosedLoopGeneral.ContinuousWrap =
      createInfo.range.isContinuous;

  configurator.Apply(talonFXConfig, 0.0_s);

  sensorToMechanismRatio = createInfo.feedbackConfig.sensorToMechanismRatio;
  tolerance = createInfo.pidConfig.tolerance;
}

void TalonFXPositionController::setPosition(units::radian_t position) {
  units::radian_t targetPosition(position);

  if (targetPosition > range.maxPosition) {
    targetPosition = range.maxPosition;
  } else if (targetPosition < range.minPosition) {
    targetPosition = range.minPosition;
  }

  /*if (usingCANCoder) {
    motorcontroller.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                        (RawCANCoderPositionUnit_t(targetPosition))());
  } else {
    motorcontroller.Set(
        ctre::phoenix::motorcontrol::ControlMode::Position,
        (RawIntegratedPositionUnit_t(targetPosition) * gearRatio)());
  }*/
  ctre::phoenix6::controls::PositionDutyCycle request(targetPosition);

  motorcontroller.SetControl(request);
}

units::radian_t TalonFXPositionController::getTargetPosition() const {
  /* if (usingCANCoder) {
    return (RawCANCoderPositionUnit_t(motorcontroller.GetClosedLoopTarget()));
  } else {
    return (RawIntegratedPositionUnit_t(
               motorcontroller.GetClosedLoopTarget())) /
           gearRatio;
  }*/
  if (motorcontroller.GetAppliedControl()->GetControlInfo()["Name"] ==
      "PositionDutyCycle") {
    // Theoretically, this doesn't 100% guarantee that the type behind the
    // pointer is controls::PositionDutyCycle* because CTRE might have screwed
    // up something in their codebase or don't handle faults correctly somewhere
    // but we're going to trust them anyways.
    // Practice safe sex, wear condoms. They don't work 100% of the time,
    // but the are better than nothing. Let this piece of code be your role
    // model

    return ((ctre::phoenix6::controls::PositionDutyCycle *)motorcontroller
                .GetAppliedControl()
                .get())
        ->Position;
  } else {
    std::cout << "(" << __FILE__ << ":" << __LINE__
              << ") Could not retrieve PositionDutyCycle" << std::endl;
    return 0.0_rad;
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
  /*if (usingCANCoder) {
    return RawCANCoderVelocityUnit_t(
        motorcontroller.GetSelectedSensorVelocity());
  } else {
    return RawIntegratedVelocityUnit_t(
        motorcontroller.GetSelectedSensorVelocity() / gearRatio);
  }*/

  if (usingCANCoder) {
    return canCoder->GetVelocity().GetValue();
  } else {
    return motorcontroller.GetVelocity().GetValue();
  }
}

units::radian_t TalonFXPositionController::getPosition() const {
  // if (usingCANCoder) {
  //   return RawCANCoderPositionUnit_t(
  //       motorcontroller.GetSelectedSensorPosition());
  // } else {
  //   return RawIntegratedPositionUnit_t(
  //       motorcontroller.GetSelectedSensorPosition() / gearRatio);
  // }
  if (usingCANCoder) {
    return canCoder->GetPosition().GetValue();
  } else {
    return motorcontroller.GetPosition().GetValue();
  }
}

void TalonFXPositionController::zeroPosition(units::radian_t offset) {
  // if (usingCANCoder) {
  //   canCoder->SetPosition(RawCANCoderPositionUnit_t(offset)());
  // } else {
  //   motorcontroller.SetSelectedSensorPosition(
  //       RawIntegratedPositionUnit_t(offset * gearRatio)());
  // }
  //

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

units::radian_t TalonFXPositionController::getTolerance() const {
  return tolerance;
}

void TalonFXPositionController::setPower(double power) {
  motorcontroller.Set(power);
}

} // namespace rmb
