#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/feedback/AngularFeedbackVelocityController.h"
#include "rmb/motorcontrol/feedback/AngularFeedbackPositionController.h"
#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/feedback/LinearFeedbackVelocityController.h"
#include "rmb/motorcontrol/feedback/LinearFeedbackPositionController.h"

namespace rmb {
// AngularEncoder
std::unique_ptr<LinearEncoder> asLinear(std::unique_ptr<AngularEncoder> angularEncoder, 
                                        MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<AngularAsLinearEncoder>(angularEncoder, conversion);
}

class AngularAsLinearEncoder : public LinearEncoder {
public:

  AngularAsLinearEncoder(std::unique_ptr<AngularEncoder> angularEncoder, MotorControlConversions::ConversionUnit_t conversionFactor) :
                         angular(std::move(angularEncoder)), conversion(conversionFactor) {}

  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { angular->setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return angular->getEncoderInverted(); }

private:
  std::unique_ptr<AngularEncoder> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

// AngularFeedbackVelocityController
std::unique_ptr<LinearFeedbackVelocityController> asLinear(std::unique_ptr<AngularFeedbackVelocityController> angularController, 
                                                           MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<AngularAsLinearFeedbackVelocityController>(angularController, conversion);
}

class AngularAsLinearFeedbackVelocityController : public LinearFeedbackVelocityController {
public:

  AngularAsLinearFeedbackVelocityController(std::unique_ptr<AngularFeedbackVelocityController> angularController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            angular(std::move(angularController)), conversion(conversionFactor) {}

  // Encoder Methods
  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { angular->setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return angular->getEncoderInverted(); }

  // Controller Methods
  void setVelocity(units::meters_per_second_t position) { angular->setVelocity(position / conversion); }
  units::meters_per_second_t getTargetVelocity() const { return angular->getTargetVelocity() * conversion; }
  void setMaxVelocity(units::meters_per_second_t max) { angular->setMaxVelocity(max / conversion); }
  units::meters_per_second_t getMaxVelocity() const { return angular->getMaxVelocity() * conversion; }
  void setInverted(bool isInverted) { angular->setInverted(isInverted); }
  bool getInverted() const { angular->getInverted(); }
  void disable() { angular->disable(); }
  void stop() { angular->stop(); }

  // Encoder Methods
  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { angular->setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return angular->getEncoderInverted(); }

  // Feedback Methods
  bool atTarget() const { return angular->atTarget(); }

private:
  std::unique_ptr<AngularFeedbackVelocityController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

// AngularFeedbackPositionController
std::unique_ptr<LinearFeedbackPositionController> asLinear(std::unique_ptr<AngularFeedbackPositionController> angularController,
                                                           MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<AngularAsLinearFeedbackPositionController>(angularController, conversion);
}

class AngularAsLinearFeedbackPositionController : public LinearFeedbackPositionController {
public:

  AngularAsLinearFeedbackPositionController(std::unique_ptr<AngularFeedbackPositionController> angularController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            angular(std::move(angularController)), conversion(conversionFactor) {}

  // Controller Methods
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

  // Encoder Methods
  units::meters_per_second_t getVelocity() const { return angular->getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular->getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { angular->setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return angular->getEncoderInverted(); }

  // Feedback Methods
  bool atTarget() const { return angular->atTarget(); }

private:
  std::unique_ptr<AngularFeedbackPositionController> angular;
  MotorControlConversions::ConversionUnit_t conversion;
};

// LinearEncoder
std::unique_ptr<AngularEncoder> asAngular(std::unique_ptr<LinearEncoder> linearEncoder, 
                                          MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<LinearAsAngularEncoder>(linearEncoder, conversion);
}

class LinearAsAngularEncoder : public AngularEncoder {
public:

  LinearAsAngularEncoder(std::unique_ptr<LinearEncoder> linearEncoder, 
                         MotorControlConversions::ConversionUnit_t conversionFactor) :
                         linear(std::move(linearEncoder)), conversion(conversionFactor) {}

  units::radians_per_second_t getVelocity() const { return linear->getVelocity() * conversion; }
  units::radian_t getPosition() const { return linear->getPosition() * conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { linear->setEncoderInverted(isInverted); }
  bool getInverted() const { return linear->getEncoderInverted(); }

private:
  std::unique_ptr<LinearEncoder> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

// LinearFeedbackVelocityController
std::unique_ptr<AngularFeedbackVelocityController> asAngular(std::unique_ptr<LinearFeedbackVelocityController> linearController,
                                                            MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<LinearAsAngularFeedbackVelocityController>(linearController, conversion);
}

class LinearAsAngularFeedbackVelocityController : public AngularFeedbackVelocityController {
public:

  LinearAsAngularFeedbackVelocityController(std::unique_ptr<LinearFeedbackVelocityController> linearController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            linear(std::move(linearController)), conversion(conversionFactor) {}

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear->getVelocity() / conversion; }
  units::radian_t getPosition() const { return linear->getPosition() / conversion; }
  void zeroPosition(units::radian_t offset = 0_m) { linear->zeroPosition(offset * conversion); }
  void setEncoderInverted(bool isInverted) { linear->setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return linear->getEncoderInverted(); }

  // Controller Methods
  void setVelocity(units::radians_per_second_t position) { linear->setVelocity(position * conversion); }
  units::radians_per_second_t getTargetVelocity() const { return linear->getTargetVelocity() / conversion; }
  void setMaxVelocity(units::radians_per_second_t max) { linear->setMaxVelocity(max * conversion); }
  units::radians_per_second_t getMaxVelocity() const { return linear->getMaxVelocity() / conversion; }
  void setInverted(bool isInverted) { linear->setInverted(isInverted); }
  bool getInverted() const { linear->getInverted(); }
  void disable() { linear->disable(); }
  void stop() { linear->stop(); }

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear->getVelocity() * conversion; }
  units::radian_t getPosition() const { return linear->getPosition() * conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { linear->setEncoderInverted(isInverted); }
  bool getInverted() const { return linear->getEncoderInverted(); }

  // Feedback Methods
  bool atTarget() const { return linear->atTarget(); }

private:
  std::unique_ptr<LinearFeedbackVelocityController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

// LinearFeedbackPositionController
std::unique_ptr<AngularFeedbackPositionController> asAngular(std::unique_ptr<LinearFeedbackPositionController> linearController,
                                                             MotorControlConversions::ConversionUnit_t conversion) {
  return std::make_unique<LinearAsAngularFeedbackPositionController>(linearController, conversion);
}

class LinearAsAngularFeedbackPositionController : public AngularFeedbackPositionController {
public:

  LinearAsAngularFeedbackPositionController(std::unique_ptr<LinearFeedbackPositionController> linearController, 
                                            MotorControlConversions::ConversionUnit_t conversionFactor) :
                                            linear(std::move(linearController)), conversion(conversionFactor) {}

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear->getVelocity() / conversion; }
  units::radian_t getPosition() const { return linear->getPosition() / conversion; }
  void zeroPosition(units::radian_t offset = 0_m) { linear->zeroPosition(offset * conversion); }
  void setEncoderInverted(bool isInverted) { linear->setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return linear->getEncoderInverted(); }

  // Controller Methods
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

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear->getVelocity() * conversion; }
  units::radian_t getPosition() const { return linear->getPosition() * conversion; }
  void zeroPosition(units::radian_t offset = 0_rad) { linear->zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { linear->setEncoderInverted(isInverted); }
  bool getInverted() const { return linear->getEncoderInverted(); }

  // Feedback Methods
  bool atTarget() const { return linear->atTarget(); }

private:
  std::unique_ptr<LinearFeedbackPositionController> linear;
  MotorControlConversions::ConversionUnit_t conversion;
};

} // namespace rmb