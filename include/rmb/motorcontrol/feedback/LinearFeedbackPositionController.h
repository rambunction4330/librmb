
#pragma once

#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/LinearPositionController.h"
#include "rmb/motorcontrol/feedback/AngularFeedbackPositionController.h"

namespace rmb {

class LinearFeedbackPositionController : 
public LinearPositionController, public LinearEncoder {
public:

  /**
   * Common interface for getting the error between the position controllers
   * target position and the actual position measured by the encoder.
   * 
   * @return position error in meters.
   */
  units::meter_t getError() const {
    return getPosition() - getTargetPosition();
  }

  /**
   * Common interface for getting whether the mechanism has achived it's
   * target position. 
   * 
   * Note: Unfortunaty no default implimentation can be made since some 
   * non-zero error will always exist. When implemented an amount allowable 
   * error must be accounted for.
   * 
   * @return true is the controller has achived the target position.
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
std::unique_ptr<AngularFeedbackPositionController> asAngular(std::unique_ptr<LinearFeedbackPositionController> linearController,
                                                             LinearAsAngularFeedbackPositionController::ConversionUnit_t conversion) {
  return std::make_unique<LinearAsAngularFeedbackPositionController>(linearController, conversion);
}

class LinearAsAngularFeedbackPositionController : public AngularFeedbackPositionController {
public:

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  LinearAsAngularFeedbackPositionController(std::unique_ptr<LinearFeedbackPositionController> linearController, 
                                            ConversionUnit_t conversionFactor) :
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
  ConversionUnit_t conversion;
};
} // namespace rmb
