#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/LinearVelocityController.h"
#include "rmb/motorcontrol/LinearPositionController.h"

namespace rmb {

// AngularVelocityController
std::unique_ptr<LinearVelocityController> asLinear(std::unique_ptr<AngularVelocityController> angularController,
                                                   MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<AngularAsLinearVelocityController>(angularController, conversion);
}

class AngularAsLinearVelocityController: public LinearVelocityController {
public:

  AngularAsLinearVelocityController(std::unique_ptr<AngularVelocityController> angularController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    angular(std::move(angularController)), conversion(conversionFactor) {}

  void setVelocity(units::meters_per_second_t velocity) { angular->setVelocity(velocity / conversion); }
  units::meters_per_second_t getTargetVelocity() const { return angular->getTargetVelocity() * conversion; }
  void setMaxVelocity(units::meters_per_second_t max) { angular->setMaxVelocity(max / conversion); }
  units::meters_per_second_t getMaxVelocity() const { return angular->getMaxVelocity() * conversion; }
  void setInverted(bool isInverted) { angular->setInverted(isInverted); }
  bool getInverted() const { return angular->getInverted(); }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }
private:
  std::unique_ptr<AngularVelocityController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

// AngularPositionController
std::unique_ptr<LinearPositionController> asLinear(std::unique_ptr<AngularPositionController> angularController, 
                                                   MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<AngularAsLinearPositionController>(angularController, conversion);
}

class AngularAsLinearPositionController : public LinearPositionController {
public:

  AngularAsLinearPositionController(std::unique_ptr<AngularPositionController> angularController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    angular(std::move(angularController)), conversion(conversionFactor) {}

  void setPosition(units::meter_t position) { angular->setPosition(position / conversion); }
  units::meter_t getTargetPosition() const { return angular->getTargetPosition() * conversion; }
  void setMinPosition(units::meter_t min) { angular->setMinPosition(min / conversion); }
  units::meter_t getMinPosition() const { return angular->getMinPosition() * conversion; }
  void setMaxPosition(units::meter_t max) { angular->setMaxPosition(max / conversion); }
  units::meter_t getMaxPosition() const { return angular->getMaxPosition() * conversion; }
  void setInverted(bool isInverted) { angular->setInverted(isInverted); }
  bool getInverted() const { angular->getInverted(); }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }

private:
  std::unique_ptr<AngularPositionController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

// LinearVelocityController
std::unique_ptr<AngularVelocityController> asAngular(std::unique_ptr<LinearVelocityController> linearController,
                                            MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<LinearAsAngularVelocityController>(linearController, conversion);
}

class LinearAsAngularVelocityController: public AngularVelocityController {
 public:

  LinearAsAngularVelocityController(std::unique_ptr<LinearVelocityController> linearController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    linear(std::move(linearController)), conversion(conversionFactor) {}

  void setVelocity(units::radians_per_second_t velocity) { linear->setVelocity(velocity * conversion); }
  units::radians_per_second_t getTargetVelocity() const { return linear->getTargetVelocity() / conversion; }
  void setMaxVelocity(units::radians_per_second_t max) { linear->setMaxVelocity(max * conversion); }
  units::radians_per_second_t getMaxVelocity() const { return linear->getMaxVelocity() / conversion; }
  void setInverted(bool isInverted) { linear->setInverted(isInverted); }
  bool getInverted() const { return linear->getInverted(); }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }
 private:
  std::unique_ptr<LinearVelocityController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

// LinearPositionController
std::unique_ptr<AngularPositionController> asAngular(std::unique_ptr<LinearPositionController> linearController, 
                                                     MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<LinearAsAngularPositionController>(linearController, conversion);
}

class LinearAsAngularPositionController : public AngularPositionController {
public:

  LinearAsAngularPositionController(std::unique_ptr<LinearPositionController> linearController, 
                                    MotorControlConversions::ConversionUnit_t conversionFactor) :
                                    linear(std::move(linearController)), conversion(conversionFactor) {}

  void setPosition(units::radian_t position) { linear->setPosition(position * conversion); }
  units::radian_t getTargetPosition() const { return linear->getTargetPosition() / conversion; }
  void setMinPosition(units::radian_t min) { linear->setMinPosition(min * conversion); }
  units::radian_t getMinPosition() const { return linear->getMinPosition() / conversion; }
  void setMaxPosition(units::radian_t max) { linear->setMaxPosition(max * conversion); }
  units::radian_t getMaxPosition() const { return linear->getMaxPosition() / conversion; }
  void setInverted(bool isInverted) { linear->setInverted(isInverted); }
  bool getInverted() const { linear->getInverted(); }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }

private:
  std::unique_ptr<LinearPositionController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};
} // namespace rmb