
#pragma once

#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {

class LinearFeedbackVelocityController : 
public LinearVelocityController, public LinearEncoder {
public:

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   * 
   * @return position error in meters per second.
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
  LinearAsAngularFeedbackVelocityController asLinearFeedbackController(ConversionUnit_t conversion) {
    return LinearAsAngularFeedbackVelocityController(*this, conversion);
  }
};

class LinearAsAngularFeedbackVelocityController : public AngularFeedbackVelocityController {
public:

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  LinearAsAngularFeedbackVelocityController(const LinearAsAngularFeedbackVelocityController&) = delete;
  LinearAsAngularFeedbackVelocityController(LinearAsAngularFeedbackVelocityController&&) = default;

  LinearAsAngularFeedbackVelocityController(LinearFeedbackVelocityController& linearController, 
                                            ConversionUnit_t conversionFactor) :
                                            linear(linearController), conversion(conversionFactor) {}

  // Encoder Methods
  units::radians_per_second_t getVelocity() const { return linear.getVelocity() / conversion; }
  units::radian_t getPosition() const { return linear.getPosition() / conversion; }
  void zeroPosition(units::radian_t offset = 0_m) { linear.zeroPosition(offset * conversion); }
  void setEncoderInverted(bool isInverted) { linear.setEncoderInverted(isInverted); }
  bool getEncoderInverted() const { return linear.getEncoderInverted(); }

  // Controller Methods
  void setVelocity(units::radians_per_second_t position) { linear.setVelocity(position * conversion); }
  units::radians_per_second_t getTargetVelocity() const { return linear.getTargetVelocity() / conversion; }
  void setMaxVelocity(units::radians_per_second_t max) { linear.setMaxVelocity(max * conversion); }
  units::radians_per_second_t getMaxVelocity() const { return linear.getMaxVelocity() / conversion; }
  void setInverted(bool isInverted) { linear.setInverted(isInverted); }
  bool getInverted() const { linear.getInverted(); }
  void disable() { linear.disable(); }
  void stop() { linear.stop(); }

  // Feedback Methods
  bool atTarget() const { return linear.atTarget(); }

private:
  LinearFeedbackVelocityController& linear;
  ConversionUnit_t conversion;
};

} // namespace rmb
