
#pragma once

#include <units/velocity.h>

#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/Conversions.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's linear velocity used by wrappers of 
 * device specific APIs.
 */
class LinearVelocityController {
public:

  /**
   * Common interface for setting the target linear velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  virtual void setVelocity(units::meters_per_second_t velocity) = 0;

  /**
   * Common interface for getting the target linear velocity.
   * 
   * @return The target linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getTargetVelocity() const = 0;

  /**
   * Common interface for setting the maximum linear velocity.
   * 
   * @param max The maximum linear velocity in meters per second.
   */
  virtual void setMaxVelocity(units::meters_per_second_t max) = 0;

  /**
   * Common interface for getting the maximum linear velocity.
   * 
   * @return The maximum linear velocity in meters per second.
   */
  virtual units::meters_per_second_t getMaxVelocity() const = 0;

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  virtual void setInverted(bool isInverted) = 0;

  /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  virtual bool getInverted() const = 0;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the mechanism until `setVelocity` is called again.
   */
  virtual void stop() = 0;
};

/**
 * Generates a `AngularVelocityController` to controller from an 
 * `LinearVelocityController` via a linear conversion factor. The new 
 * controller takes ownership over the old one.
 * 
 * @param angularController origional controller the new one is generated from.
 * @param conversion conversion factor from linear to angular units.
 */
std::unique_ptr<AngularVelocityController> asAngular(std::unique_ptr<LinearVelocityController> linearController,
                                                     MotorControlConversions::ConversionUnit_t conversion);

} // namespace rmb
