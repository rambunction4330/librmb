
#pragma once

#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/feedback/LinearFeedbackVelocityController.h"

namespace rmb {

class AngularFeedbackVelocityController : 
public AngularVelocityController, public AngularEncoder {
public:
  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   * 
   * @return position error in radians per second.
   */
  units::radians_per_second_t getError() const {
    return getVelocity() - getTargetVelocity();
  }

  /**
   * Common interface for getting whether the mechanism has achived it's
   * target velocity. 
   * 
   * Note: Unfortunaty no default implimentation can be made since some 
   * non-zero error will always exist. When implemented an amount allowable 
   * error must be accounted for.
   * 
   * @return true is the controller has achived the target velocity.
   */
  virtual bool atTarget() const = 0;
};

/**
 * Generates a `AngularAsLinearEncoder` to measure the same mechanism as this
 * object, but with linear instead of angular units via a linear
 * conversion factor. Changes to one controller will effect the other since 
 * they measure the same physical mechanism.
 * 
 * @param conversion conversion from linear to angular units.
 */
std::unique_ptr<LinearFeedbackVelocityController> asLinear(std::unique_ptr<AngularFeedbackVelocityController> angularController, 
                                                           AngularAsLinearFeedbackVelocityController::ConversionUnit_t conversion) {
  return std::make_unique<AngularAsLinearFeedbackVelocityController>(angularController, conversion);
}

class AngularAsLinearFeedbackVelocityController : public LinearFeedbackVelocityController {
public:

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  AngularAsLinearFeedbackVelocityController(std::unique_ptr<AngularFeedbackVelocityController> angularController, 
                                            ConversionUnit_t conversionFactor) :
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
  ConversionUnit_t conversion;
};

} // namespace rmb
