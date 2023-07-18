#include "rmb/drive/SwerveModule.h"
#include "frc/kinematics/SwerveModuleState.h"

namespace rmb {

SwerveModulePower SwerveModulePower::Optimize(const SwerveModulePower &desired,
                                              const frc::Rotation2d &current) {
  if (units::math::abs((desired.angle - current).Degrees()) > 90_deg) {
    return {-desired.power, desired.angle + frc::Rotation2d(180_deg)};
  }
  return {desired.power, desired.angle};
}

SwerveModule::SwerveModule(
    std::unique_ptr<AngularPositionController> angularController,
    std::unique_ptr<LinearVelocityController> velocityController,
    const frc::Translation2d &moduleTranslation)
    : angularController(std::move(angularController)),
      velocityController(std::move(velocityController)),
      moduleTranslation(moduleTranslation) {}

void SwerveModule::setState(const units::meters_per_second_t &velocity,
                            const frc::Rotation2d &angle) {
  setState({velocity, angle});
}

void SwerveModule::setState(const frc::SwerveModuleState &state) {
  auto optomized = frc::SwerveModuleState::Optimize(state, getState().angle);
  velocityController->setVelocity(optomized.speed);
  angularController->setPosition(optomized.angle.Radians());
}

frc::SwerveModuleState SwerveModule::getState() const {
  return {velocityController->getVelocity(),
          frc::Rotation2d(angularController->getPosition())};
}

frc::SwerveModulePosition SwerveModule::getPosition() const {
  return {velocityController->getPosition(),
          frc::Rotation2d(angularController->getPosition())};
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





} // namespace rmb