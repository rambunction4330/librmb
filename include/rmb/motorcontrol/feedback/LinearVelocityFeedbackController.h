
#pragma once

#include "rmb/motorcontrol/feedback/LinearEncoder.h"
#include "rmb/motorcontrol/LinearVelocityController.h"
#include "rmb/motorcontrol/feedback/AngularVelocityFeedbackController.h"

namespace rmb {

class LinearVelocityFeedbackController : 
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
};

/**
 * Generates a `AngularAsLinearEncoder` to measure the same mechanism as this
 * object, but with linear instead of angular units via a linear
 * conversion factor. Changes to one controller will effect the other since 
 * they measure the same physical mechanism.
 * 
 * @param conversion conversion from linear to angular units.
 */
std::unique_ptr<AngularVelocityFeedbackController> asAngular(std::unique_ptr<LinearVelocityFeedbackController> linearController,
                                                             MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
