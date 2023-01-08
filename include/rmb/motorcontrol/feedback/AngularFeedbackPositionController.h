
#pragma once

#include "rmb/motorcontrol/feedback/AngularEncoder.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/feedback/LinearFeedbackPositionController.h"

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
};

/**
 * Generates a `LinearFeedbackPositionController` to controller from an 
 * `AngularFeedbackPositionController` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::unique_ptr<LinearFeedbackPositionController> asLinear(std::unique_ptr<AngularFeedbackPositionController> angularController,
                                                           MotorControlConversions::ConversionUnit_t conversion);
} // namespace rmb
