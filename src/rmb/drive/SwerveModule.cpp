#include "rmb/drive/SwerveModule.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "units/angle.h"
#include "units/velocity.h"
#include "wpi/sendable/SendableBuilder.h"
#include "wpi/sendable/SendableRegistry.h"

#include <functional>
#include <iostream>

namespace rmb {

SwerveModulePower SwerveModulePower::Optimize(const SwerveModulePower &desired,
                                              const frc::Rotation2d &current) {
  if (units::math::abs((desired.angle - current).Degrees()) > 90_deg) {
    return {-desired.power, desired.angle + frc::Rotation2d(180_deg)};
  }
  return {desired.power, desired.angle};
}

SwerveModule::SwerveModule(
    std::unique_ptr<LinearVelocityController> velocityController,
    std::unique_ptr<AngularPositionController> angularController,
    const frc::Translation2d &moduleTranslation, bool breakMode)
    : angularController(std::move(angularController)),
      velocityController(std::move(velocityController)),
      moduleTranslation(moduleTranslation), breakMode(breakMode) {}

void SwerveModule::setState(const units::meters_per_second_t &velocity,
                            const frc::Rotation2d &angle) {
  setState({velocity, angle});
}

void SwerveModule::setState(const frc::SwerveModuleState &state) {
  auto optomized = frc::SwerveModuleState::Optimize(state, getState().angle);
  if (breakMode && state.speed == 0.0_mps) {
    std::cout << "stop module!" << std::endl;
    velocityController->stop();
  } else {
    velocityController->setVelocity(optomized.speed);
  }
  angularController->setPosition(optomized.angle.Radians());
}

void SwerveModule::smartdashboardDisplayTargetState(
    const std::string &name) const {
  frc::SmartDashboard::PutNumber(
      name + "_tgt_direction",
      ((units::degree_t)(angularController->getTargetPosition()))());

  frc::SmartDashboard::PutNumber(
      name + "_tgt_vel", ((units::meters_per_second_t)(
                             velocityController->getTargetVelocity()))());
}

frc::SwerveModuleState SwerveModule::getState() const {
  return {velocityController->getVelocity(),
          frc::Rotation2d(angularController->getPosition())};
}

frc::SwerveModulePosition SwerveModule::getPosition() const {
  return {velocityController->getPosition(),
          frc::Rotation2d(angularController->getPosition())};
}

frc::SwerveModuleState SwerveModule::getTargetState() const {
  return {velocityController->getTargetVelocity(),
          frc::Rotation2d(angularController->getTargetPosition())};
}

void SwerveModule::setPower(double power, const frc::Rotation2d &angle) {
  velocityController->setPower(power);
  angularController->setPosition(angle.Radians());
}

void SwerveModule::setPower(const SwerveModulePower &power) {
  velocityController->setPower(power.power);
  angularController->setPosition(power.angle.Radians());
}

const frc::Translation2d &SwerveModule::getModuleTranslation() const {
  return moduleTranslation;
}

void SwerveModule::InitSendable(wpi::SendableBuilder &builder) {
  builder.AddDoubleProperty(
      "angle", [this] { return this->getAngle(); }, [](double) {});
}

} // namespace rmb
