
#pragma once

#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/feedback/LinearVelocityFeedbackController.h"

namespace rmb {

class AngularVelocityFeedbackController : public AngularVelocityController,
                                          public AngularEncoder {
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
 * Generates a `LinearVelocityFeedbackController` to controller from an
 * `AngularVelocityFeedbackController` via a linear conversion factor. The new
 * controller takes ownership over the old one.
 *
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::unique_ptr<LinearVelocityFeedbackController>
asLinear(std::unique_ptr<AngularVelocityFeedbackController> angularController,
         MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
