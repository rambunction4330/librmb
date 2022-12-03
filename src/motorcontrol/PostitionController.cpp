
#include <rmb/motorcontrol/PositionController.h>

namespace rmb {
void PositionController::setPosition(units::meter_t pos) {
  return setAngularPosition(linearToAngular(pos));
}

void PositionController::setPositionOffset(units::meter_t off) {
  return setPosition(getPosition() + off);
}

void PositionController::setAngularPositionOffset(units::radian_t off) {
  return setAngularPosition(getAngularPosition() + off);
}

units::meter_t PositionController::getPosition() const {
  return angularToLinear(getAngularPosition());
}

units::meter_t PositionController::getTargetPosition() const {
  return angularToLinear(getTargetAngularPosition());
}

units::meter_t PositionController::getError() const {
  return getPosition() - getTargetPosition();
}

units::radian_t PositionController::getAngularError() const {
  return getAngularPosition() - getTargetAngularPosition();
}

bool PositionController::zeroPosition(units::meter_t off) {
  units::radian_t angularPos = linearToAngular(getTargetPosition() + off);
  return zeroAngularPosition(angularPos - getAngularPosition());
}

bool PositionController::setMaxPosition(units::meter_t max) {
  setMaxAngularPosition(linearToAngular(max));
}

units::meter_t PositionController::getMaxPosition() const {
  return angularToLinear(getMaxAngularPosition());
}

bool PositionController::setMinPosition(units::meter_t min) {
  setMinAngularPosition(linearToAngular(min));
}

units::meter_t PositionController::getMinPosition() const {
  return angularToLinear(getMinAngularPosition());
}
} // namespace rmb