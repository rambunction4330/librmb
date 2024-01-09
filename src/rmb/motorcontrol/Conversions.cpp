#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/LinearPositionController.h"
#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {

//---------------------------
// AngularVelocityController
//---------------------------

class AngularAsLinearVelocityController : public LinearVelocityController {
public:
  AngularAsLinearVelocityController(
      std::unique_ptr<AngularVelocityController> angularController,
      AngularVelocityController::ConversionUnit_t conversionFactor)
      : angular(std::move(angularController)), conversion(conversionFactor) {}

  void setVelocity(units::meters_per_second_t velocity) override {
    angular->setVelocity(velocity / conversion);
  }

  units::meters_per_second_t getTargetVelocity() const override {
    return angular->getTargetVelocity() * conversion;
  }

  void setPower(double power) override { angular->setPower(power); }

  double getPower() const override { return angular->getPower(); }

  void disable() override { angular->disable(); }

  void stop() override { angular->stop(); }

  units::meters_per_second_t getVelocity() const override {
    return angular->getVelocity() * conversion;
  }

  units::meter_t getPosition() const override {
    return angular->getPosition() * conversion;
  }

  void setEncoderPosition(units::meter_t position) override {
    angular->setEncoderPosition(position / conversion);
  }

  units::meters_per_second_t getTolerance() const override {
    return angular->getTolerance() * conversion;
  }

private:
  std::unique_ptr<AngularVelocityController> angular;
  AngularVelocityController::ConversionUnit_t conversion;
};

std::unique_ptr<LinearVelocityController>
asLinear(std::unique_ptr<AngularVelocityController> angularController,
         AngularVelocityController::ConversionUnit_t conversion) {

  return std::make_unique<AngularAsLinearVelocityController>(
      std::move(angularController), conversion);
}

//---------------------------
// AngularPositionController
//---------------------------

class AngularAsLinearPositionController : public LinearPositionController {
public:
  AngularAsLinearPositionController(
      std::unique_ptr<AngularPositionController> angularController,
      AngularPositionController::ConversionUnit_t conversionFactor)
      : angular(std::move(angularController)), conversion(conversionFactor) {}

  void setPosition(units::meter_t position) override {
    angular->setPosition(position / conversion);
  }

  units::meter_t getTargetPosition() const override {
    return angular->getTargetPosition() * conversion;
  }

  void setPower(double power) override { angular->setPower(power); }

  double getPower() const override { return angular->getPower(); }

  units::meter_t getMinPosition() const override {
    return angular->getMinPosition() * conversion;
  }

  units::meter_t getMaxPosition() const override {
    return angular->getMaxPosition() * conversion;
  }

  void disable() override { angular->disable(); }

  void stop() override { angular->stop(); }

  units::meters_per_second_t getVelocity() const override {
    return angular->getVelocity() * conversion;
  }

  units::meter_t getPosition() const override {
    return angular->getPosition() * conversion;
  }

  void setEncoderPosition(units::meter_t position) override {
    angular->setEncoderPosition(position / conversion);
  }

  units::meter_t getTolerance() const override {
    return angular->getTolerance() * conversion;
  }

private:
  std::unique_ptr<AngularPositionController> angular;
  AngularPositionController::ConversionUnit_t conversion;
};

std::unique_ptr<LinearPositionController>
asLinear(std::unique_ptr<AngularPositionController> angularController,
         AngularPositionController::ConversionUnit_t conversion) {

  return std::make_unique<AngularAsLinearPositionController>(
      std::move(angularController), conversion);
}

//--------------------------
// LinearVelocityController
//--------------------------

class LinearAsAngularVelocityController : public AngularVelocityController {
public:
  LinearAsAngularVelocityController(
      std::unique_ptr<LinearVelocityController> linearController,
      LinearVelocityController::ConversionUnit_t conversionFactor)
      : linear(std::move(linearController)), conversion(conversionFactor) {}

  void setVelocity(units::radians_per_second_t velocity) override {
    linear->setVelocity(velocity * conversion);
  }

  units::radians_per_second_t getTargetVelocity() const override {
    return linear->getTargetVelocity() / conversion;
  }

  void setPower(double power) override { linear->setPower(power); }

  double getPower() const override { return linear->getPower(); }

  void disable() override { linear->disable(); }

  void stop() override { linear->stop(); }

  units::radians_per_second_t getVelocity() const override {
    return linear->getVelocity() / conversion;
  }

  units::radian_t getPosition() const override {
    return linear->getPosition() / conversion;
  }

  void setEncoderPosition(units::radian_t position) override {
    linear->setEncoderPosition(position * conversion);
  }

  units::radians_per_second_t getTolerance() const override {
    return linear->getTolerance() / conversion;
  }

private:
  std::unique_ptr<LinearVelocityController> linear;
  LinearVelocityController::ConversionUnit_t conversion;
};

std::unique_ptr<AngularVelocityController>
asAngular(std::unique_ptr<LinearVelocityController> linearController,
          LinearVelocityController::ConversionUnit_t conversion) {

  return std::make_unique<LinearAsAngularVelocityController>(
      std::move(linearController), conversion);
}

//--------------------------
// LinearPositionController
//--------------------------

class LinearAsAngularPositionController : public AngularPositionController {
public:
  LinearAsAngularPositionController(
      std::unique_ptr<LinearPositionController> linearController,
      LinearPositionController::ConversionUnit_t conversionFactor)
      : linear(std::move(linearController)), conversion(conversionFactor) {}

  void setPosition(units::radian_t position) override {
    linear->setPosition(position * conversion);
  }

  units::radian_t getTargetPosition() const override {
    return linear->getTargetPosition() / conversion;
  }

  void setPower(double power) override { linear->setPower(power); }

  double getPower() const override { return linear->getPower(); }

  units::radian_t getMinPosition() const override {
    return linear->getMinPosition() / conversion;
  }

  units::radian_t getMaxPosition() const override {
    return linear->getMaxPosition() / conversion;
  }

  void disable() override { linear->disable(); }

  void stop() override { linear->stop(); }

  units::radians_per_second_t getVelocity() const override {
    return linear->getVelocity() / conversion;
  }

  units::radian_t getPosition() const override {
    return linear->getPosition() / conversion;
  }

  void setEncoderPosition(units::radian_t position) override {

    linear->setEncoderPosition(position * conversion);
  }

  units::radian_t getTolerance() const override {
    return linear->getTolerance() / conversion;
  }

private:
  std::unique_ptr<LinearPositionController> linear;
  LinearPositionController::ConversionUnit_t conversion;
};

std::unique_ptr<AngularPositionController>
asAngular(std::unique_ptr<LinearPositionController> linearController,
          LinearPositionController::ConversionUnit_t conversion) {

  return std::make_unique<LinearAsAngularPositionController>(
      std::move(linearController), conversion);
}
} // namespace rmb
