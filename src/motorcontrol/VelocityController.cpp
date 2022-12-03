
#include <rmb/motorcontrol/VelocityController.h>

namespace rmb {
void VelocityController::setVelocity(units::meters_per_second_t vel) {
  setAngularVelocity(linearToAngular(vel));
}

units::meters_per_second_t VelocityController::getVelocity() const {
  return angularToLinear(getAngularVelocity());
}

units::meters_per_second_t VelocityController::getTargetVelocity() const {
  return angularToLinear(getTargetAngularVelocity());
}

units::meters_per_second_t VelocityController::getError() const {
  return getVelocity() - getTargetVelocity();
}

units::radians_per_second_t VelocityController::getAngularError() const {
  return getAngularVelocity() - getTargetAngularVelocity();
}

void VelocityController::setMaxVelocity(units::radians_per_second_t max) {
  setMaxAngularVelocity(linearToAngular(max));
}

units::radians_per_second_t VelocityController::getMaxVelocity() const {
  return angularToLinear(getMaxAngularVelocity());
}

} // namespace rmb