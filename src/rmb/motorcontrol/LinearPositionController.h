
#pragma once

#include <memory>

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

namespace rmb {

class AngularPositionController;

/**
 * Interface for controlling a mechanism's linear position used by wrappers of
 * device specific APIs.
 */
class LinearPositionController {
public:
  //***************
  // Motor Control
  //***************

  /**
   * Common interface for setting the target linear position.
   *
   * @param position The target linear position in meters.
   */
  virtual void setPosition(units::meter_t position) = 0;

  /**
   * Common interface for getting the <b>target</b> linear position.
   *
   * @return The target linear position in meters.
   */
  virtual units::meter_t getTargetPosition() const = 0;

  /**
   * Common interface for getting the minimum linear position.
   *
   * @return The minimum linear position in meters.
   */
  virtual units::meter_t getMinPosition() const = 0;

  /**
   * Common interface for getting the maximum linear position.
   *
   * @return The maximum linear position in meters.
   */
  virtual units::meter_t getMaxPosition() const = 0;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() = 0;

  //**********
  // Feedback
  //**********

  /**
   * Common interface for returning the linear velocity of an encoder.
   *
   * @return The velocity of the encoder in meters per second.
   */
  virtual units::meters_per_second_t getVelocity() const = 0;

  /**
   * Common interface for returning the linear position of an encoder.
   *
   * @return The position of the encoder in meters.
   */
  virtual units::meter_t getPosition() const = 0;

  /**
   * Common interface for zeroing the linear positon an encoder so the current
   * position is set to the offset.
   *
   * @param offset the offset from the current angular position at which to
   *               set the zero position.
   */
  virtual void zeroPosition(units::meter_t offset = 0_m) = 0;

  /**
   * Common interface for getting a controllers tolerance
   */
  virtual units::meter_t getTolerance() const = 0;

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   *
   * @return position error in radians per second.
   */
  virtual units::meter_t getError() const {
    return getPosition() - getTargetPosition();
  }

  /**
   * Common interface for getting whether the mechanism has achived it's
   * target velocity.
   *
   * @return true is the controller has achived the target velocity.
   */
  virtual bool atTarget() const {
    return units::math::abs(getError()) < getTolerance();
  }

  using ConversionUnit =
      units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;
};

/**
 * Generates a `AngularVelocityController` from a `LinearVelocityController`
 * via a proportional conversion factor. The new controller takes ownership
 * over the old one so this function can only be called at construction.
 *
 * @param conversion conversion factor from linear to angular units such as a
 *                   wheel diameter.
 */
std::unique_ptr<AngularPositionController>
asAngular(std::unique_ptr<LinearPositionController> linearController,
          LinearPositionController::ConversionUnit_t conversion);

} // namespace rmb
