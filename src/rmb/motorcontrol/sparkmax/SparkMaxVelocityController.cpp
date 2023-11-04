#include "rmb/motorcontrol/sparkmax/SparkMaxVelocityController.h"

#include <units/angle.h>
#include <units/length.h>

namespace rmb {

SparkMaxVelocityController::SparkMaxVelocityController(
    const SparkMaxVelocityController::CreateInfo &createInfo)
    : sparkMax(createInfo.motorConfig.id, createInfo.motorConfig.motorType),
      pidController(sparkMax.GetPIDController()),
      tolerance(createInfo.pidConfig.tolerance),
      encoderType(createInfo.feedbackConfig.encoderType),
      gearRatio(createInfo.feedbackConfig.gearRatio) {

  // Restore defaults to ensure a consistent and clean slate.
  sparkMax.RestoreFactoryDefaults();
  sparkMax.SetSmartCurrentLimit(
      static_cast<unsigned int>(createInfo.motorConfig.currentLimit() + 0.5));
  sparkMax.SetOpenLoopRampRate(createInfo.profileConfig.closedLoopRampRate());
  sparkMax.SetClosedLoopRampRate(createInfo.motorConfig.openLoopRampRate());

  // Motor Configuration
  sparkMax.SetInverted(createInfo.motorConfig.inverted);

  // PID Configuration
  setPIDConstants(createInfo.pidConfig);
  // Motion Profiling Configuration
  controlType = rev::CANSparkMax::ControlType::kVelocity;
  if (createInfo.profileConfig.useSmartMotion) {
    controlType = rev::CANSparkMax::ControlType::kSmartVelocity;
    pidController.SetSmartMotionMaxVelocity(
        units::revolutions_per_minute_t(createInfo.profileConfig.maxVelocity)
            .to<double>() *
        gearRatio);
    pidController.SetSmartMotionMaxAccel(
        units::revolutions_per_minute_per_second_t(
            createInfo.profileConfig.maxAcceleration)
            .to<double>() *
        gearRatio);
    pidController.SetSmartMotionAccelStrategy(
        createInfo.profileConfig.accelStrategy);
  }

  // Encoder Configuation
  switch (encoderType) {
  case EncoderType::HallSensor:
    encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(
        sparkMax.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor,
                            createInfo.feedbackConfig.countPerRev));
    break;
  case EncoderType::Quadrature:
    encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(
        sparkMax.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kQuadrature,
                            createInfo.feedbackConfig.countPerRev));
    break;
  case EncoderType::Alternate:
    encoder = std::make_unique<rev::SparkMaxAlternateEncoder>(
        sparkMax.GetAlternateEncoder(createInfo.feedbackConfig.countPerRev));
    break;
  case EncoderType::Absolute:
    encoder = std::make_unique<rev::SparkMaxAbsoluteEncoder>(
        sparkMax.GetAbsoluteEncoder(
            rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle));
    break;
  }

  pidController.SetFeedbackDevice(*encoder);

  // Limit Switch Configuaration
  switch (createInfo.feedbackConfig.forwardSwitch) {
  case LimitSwitchConfig::Disabled:
    sparkMax
        .GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
        .EnableLimitSwitch(false);
    break;
  case LimitSwitchConfig::NormalyOpen:
    sparkMax
        .GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
        .EnableLimitSwitch(true);
    break;
  case LimitSwitchConfig::NormalyClosed:
    sparkMax
        .GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed)
        .EnableLimitSwitch(true);
    break;
  }

  switch (createInfo.feedbackConfig.reverseSwitch) {
  case LimitSwitchConfig::Disabled:
    sparkMax
        .GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
        .EnableLimitSwitch(false);
    break;
  case LimitSwitchConfig::NormalyOpen:
    sparkMax
        .GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)
        .EnableLimitSwitch(true);
    break;
  case LimitSwitchConfig::NormalyClosed:
    sparkMax
        .GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed)
        .EnableLimitSwitch(true);
    break;
  }

  // Follower Congiguration
  for (auto &follower : createInfo.followers) {
    followers.emplace_back(
        std::make_unique<rev::CANSparkMax>(follower.id, follower.motorType));
    followers.back()->Follow(sparkMax, follower.inverted);
  }
}

void SparkMaxVelocityController::setVelocity(
    units::radians_per_second_t velocity) {
  targetVelocity = velocity;
  pidController.SetReference(
      units::revolutions_per_minute_t(targetVelocity).to<double>() * gearRatio,
      controlType);
}

units::radians_per_second_t
SparkMaxVelocityController::getTargetVelocity() const {
  return targetVelocity;
}

void SparkMaxVelocityController::setPower(double power) {
  targetVelocity = 0.0_rad_per_s;
  sparkMax.Set(power);
}

void SparkMaxVelocityController::disable() {
  targetVelocity = 0.0_rad_per_s;
  sparkMax.Disable();
}

void SparkMaxVelocityController::stop() {
  targetVelocity = 0.0_rad_per_s;
  sparkMax.StopMotor();
}

units::radians_per_second_t SparkMaxVelocityController::getVelocity() const {
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::SparkMaxRelativeEncoder *rel =
        static_cast<rev::SparkMaxRelativeEncoder *>(encoder.get());
    return units::revolutions_per_minute_t(rel->GetVelocity() / gearRatio);
  }
  case EncoderType::Alternate: {
    rev::SparkMaxAlternateEncoder *alt =
        static_cast<rev::SparkMaxAlternateEncoder *>(encoder.get());
    return units::revolutions_per_minute_t(alt->GetVelocity() / gearRatio);
  }
  case EncoderType::Absolute: {
    rev::AbsoluteEncoder *ab =
        static_cast<rev::AbsoluteEncoder *>(encoder.get());
    return units::revolutions_per_minute_t(ab->GetVelocity() / gearRatio);
  }
  }

  return 0_rpm;
}

units::radian_t SparkMaxVelocityController::getPosition() const {
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::SparkMaxRelativeEncoder *rel =
        static_cast<rev::SparkMaxRelativeEncoder *>(encoder.get());
    return units::turn_t(rel->GetPosition() / gearRatio);
  }
  case EncoderType::Alternate: {
    rev::SparkMaxAlternateEncoder *alt =
        static_cast<rev::SparkMaxAlternateEncoder *>(encoder.get());
    return units::turn_t(alt->GetPosition() / gearRatio);
  }
  case EncoderType::Absolute: {
    rev::SparkMaxAbsoluteEncoder *ab =
        static_cast<rev::SparkMaxAbsoluteEncoder *>(encoder.get());
    return units::turn_t(ab->GetPosition() / gearRatio);
  }
  }

  return 0_rad;
}

void SparkMaxVelocityController::zeroPosition(units::radian_t offset) {
  using EncoderType = SparkMaxVelocityControllerHelper::EncoderType;

  switch (encoderType) {
  case EncoderType::HallSensor:
  case EncoderType::Quadrature: {
    rev::SparkMaxRelativeEncoder *rel =
        static_cast<rev::SparkMaxRelativeEncoder *>(encoder.get());
    rel->SetPosition(units::turn_t(offset).to<double>() * gearRatio);
    break;
  }
  case EncoderType::Alternate: {
    rev::SparkMaxAlternateEncoder *rel =
        static_cast<rev::SparkMaxAlternateEncoder *>(encoder.get());
    rel->SetPosition(units::turn_t(offset).to<double>() * gearRatio);
    break;
  }
  case EncoderType::Absolute: {
    rev::SparkMaxAbsoluteEncoder *ab =
        static_cast<rev::SparkMaxAbsoluteEncoder *>(encoder.get());
    ab->SetZeroOffset(units::turn_t(offset).to<double>() * gearRatio);
  }
  }
}

units::radians_per_second_t SparkMaxVelocityController::getTolerance() const {
  return tolerance;
}

void SparkMaxVelocityController::setPIDConstants(PIDConfig config) {
  pidController.SetP(config.p);
  pidController.SetI(config.i);
  pidController.SetD(config.d);
  pidController.SetFF(config.ff);
  pidController.SetIZone(config.iZone);
  pidController.SetIMaxAccum(config.iMaxAccumulator);
  pidController.SetOutputRange(config.minOutput, config.maxOutput);
}
} // namespace rmb
