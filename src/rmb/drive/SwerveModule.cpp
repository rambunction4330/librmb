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
  if (desired.power == 0.0) {
    return {0.0, current};
  } else if (units::math::abs((desired.angle - current).Degrees()) > 90_deg) {
    return {-desired.power, desired.angle + frc::Rotation2d(180_deg)};
  } else {
    return {desired.power, desired.angle};
  }
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
  // units::millisecond_t start = frc::Timer::GetFPGATimestamp();
  auto optomized = frc::SwerveModuleState::Optimize(state, getState().angle);
  // std::cout << "optimize time: "
  //           << ((units::millisecond_t)frc::Timer::GetFPGATimestamp() -
  //           start)()
  //           << std::endl;
  velocityController->setVelocity(optomized.speed);
  angularController->setPosition(optomized.angle.Radians());
}

void SwerveModule::smartdashboardDisplayTargetState(
    const std::string &name) const {
  /*frc::SmartDashboard::PutNumber(
      name + "_tgt_direction",
      ((units::degree_t)(angularController->getTargetPosition()))());

  frc::SmartDashboard::PutNumber(
      name + "_tgt_vel", ((units::meters_per_second_t)(
                             velocityController->getTargetVelocity()))());*/
}

frc::SwerveModuleState SwerveModule::getState() const {
  // units::millisecond_t startGetVelocity = frc::Timer::GetFPGATimestamp();
  auto velocity = velocityController->getVelocity();
  // std::cout << "getVelocity time: "
  //           << ((units::millisecond_t)frc::Timer::GetFPGATimestamp() -
  //               startGetVelocity)()
  //           << std::endl;

  // units::millisecond_t start = frc::Timer::GetFPGATimestamp();
  auto rotation = frc::Rotation2d(angularController->getPosition());
  // std::cout << "getRotation time: "
  //           << ((units::millisecond_t)frc::Timer::GetFPGATimestamp() -
  //           start)()
  //           << std::endl;

  return {velocity, rotation};
}

frc::SwerveModulePosition SwerveModule::getPosition() const {
  return {velocityController->getPosition(),
          frc::Rotation2d(angularController->getPosition())};
}

frc::SwerveModuleState SwerveModule::getTargetState() const {
  return {velocityController->getTargetVelocity(),
          frc::Rotation2d(angularController->getTargetPosition())};
}

units::meters_per_second_t SwerveModule::getTargetVelocity() const {
  return velocityController->getTargetVelocity();
}

frc::Rotation2d SwerveModule::getTargetRotation() const {
  return frc::Rotation2d(angularController->getTargetPosition());
}

void SwerveModule::setPower(double power, const frc::Rotation2d &angle) {
  velocityController->setPower(power);
  angularController->setPosition(angle.Radians());
}

void SwerveModule::setPower(const SwerveModulePower &power) {
  velocityController->setPower(power.power);
  angularController->setPosition(power.angle.Radians());
}

SwerveModulePower SwerveModule::getPower() {
  return {.power = velocityController->getPower(),
          .angle = angularController->getTargetPosition()};
}

const frc::Translation2d &SwerveModule::getModuleTranslation() const {
  return moduleTranslation;
}

void SwerveModule::InitSendable(wpi::SendableBuilder &builder) {
  builder.AddDoubleProperty(
      "angle", [this] { return this->getAngle(); }, [](double) {});
}

void SwerveModule::stop() {
  velocityController->stop();
  angularController->stop();
}

} // namespace rmb
