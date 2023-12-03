#include "TalonFXPositionController.h"
// #include "ctre/phoenix/motorcontrol/ControlMode.h"
/*#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"
#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"
#include "ctre/phoenix/sensors/CANCoder.h"
#include "ctre/phoenix/sensors/SensorTimeBase.h"*/
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "units/angle.h"
#include <iostream>

namespace rmb {

TalonFXPositionController::TalonFXPositionController(
    const TalonFXPositionController::CreateInfo &createInfo)
    : motorcontroller(createInfo.config.id), range(createInfo.range),
      usingCANCoder(createInfo.canCoderConfig.useCANCoder) {

  /*motorcontroller.ConfigFactoryDefault();

  motorcontroller.SetInverted(createInfo.config.inverted);

  motorcontroller.ConfigPeakOutputForward(createInfo.openLoopConfig.maxOutput);
  motorcontroller.ConfigPeakOutputReverse(createInfo.openLoopConfig.minOutput);
  motorcontroller.ConfigOpenloopRamp(createInfo.openLoopConfig.rampRate());
  motorcontroller.ConfigClosedloopRamp(createInfo.pidConfig.rampRate());

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

    // talonFXConfig.Feedback.RotorToSensorRatio; // This is for FusedCANCoder
    talonFXConfig.Feedback.WithRemoteCANcoder(canCoder.value());
  }

  // Often there is a gear ratio between the motor's rotor and the actual output of
  // the mechanism.
  talonFXConfig.Feedback.SensorToMechanismRatio =
      createInfo.feedbackConfig.gearRatio;
  talonFXConfig.Feedback.FeedbackRotorOffset;
}

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

} // namespace rmb
