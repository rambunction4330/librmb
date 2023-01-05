
#pragma once

#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/feedback/LinearFeedbackPositonController.h"

namespace rmb {

class AngularFeedbackPositionController : 
public AngularPositionController, public AngularEncoder {
public:

  /**
   * Common interface for getting the error between the position controllers
   * target position and the actual position measured by the encoder.
   * 
   * @return position error in radians.
   */
  units::radian_t getError() const {
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

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  /**
   * Generates a `AngularAsLinearEncoder` to measure the same mechanism as this
   * object, but with linear instead of angular units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they measure the same physical mechanism.
   * 
   * @param conversion conversion from linear to angular units.
   */
  AngularAsLinearFeedbackPositionController asLinearFeedbackController(ConversionUnit_t conversion) {
    return AngularAsLinearFeedbackPositionController(*this, conversion);
  }
};

class AngularAsLinearFeedbackPositionController : public LinearFeedbackPositionController {
public:

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  AngularAsLinearFeedbackPositionController(const AngularAsLinearFeedbackPositionController&) = delete;
  AngularAsLinearFeedbackPositionController(AngularAsLinearFeedbackPositionController&&) = default;

  AngularAsLinearFeedbackPositionController(AngularFeedbackPositionController& angularController, 
                                            ConversionUnit_t conversionFactor) :
                                            angular(angularController), conversion(conversionFactor) {}

  // Encoder Methods
  units::meters_per_second_t getVelocity() const { return angular.getVelocity() * conversion; }
  units::meter_t getPosition() const { return angular.getPosition() * conversion; }
  void zeroPosition(units::meter_t offset = 0_m) { angular.zeroPosition(offset / conversion); }
  void setEncoderInverted(bool isInverted) { angular.setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return angular.getEncoderInverted(); }

  // Controller Methods
  void setPosition(units::meter_t position) { angular.setPosition(position / conversion); }
  units::meter_t getTargetPosition() const { return angular.getTargetPosition() * conversion; }
  void setMinPosition(units::meter_t min) { angular.setMinPosition(min / conversion); }
  units::meter_t getMinPosition() const { return angular.getMinPosition() * conversion; }
  void setMaxPosition(units::meter_t max) { angular.setMaxPosition(max / conversion); }
  units::meter_t getMaxPosition() const { return angular.getMaxPosition() * conversion; }
  void setInverted(bool isInverted) { angular.setInverted(isInverted); }
  bool getInverted() const {angular.getInverted(); }
  void disable() { angular.disable(); }
  void stop() { angular.stop(); }

  // Feedback Methods
  bool atTarget() const { return angular.atTarget(); }

private:
  AngularFeedbackPositionController& angular;
  ConversionUnit_t conversion;
};
} // namespace rmb
