
#pragma once

#include <memory>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/math.h>

namespace rmb {

class LinearPositionController;

/**
 * Interface for controlling a mechanism's angular position used by wrappers of
 * device specific APIs.
 */
class AngularPositionController {
public:
  virtual ~AngularPositionController() = default;

  /**
   * Common interface for setting the target angular position.
   *
   * @param position The target angular position in radians.
   */

  virtual void setPosition(units::radian_t position) = 0;

  /**
   * Common interface for getting the <b>target</b> angular position.
   *
   * @return The target angular position in radians.
   */
  virtual units::radian_t getTargetPosition() const = 0;

  /**
   * Common interface for setting a mechanism's raw power output.
   */
  virtual void setPower(double power) = 0;

  /**
   * Retrieve the percentage [0.0, 1.0] output of the motor
   */
  virtual double getPower() const = 0;

  /**
   * Common interface for getting the minimum angular position.
   *
   * @return The minimum angular position in radians.
   */
  virtual units::radian_t getMinPosition() const = 0;

  /**
   * Common interface for getting the maximum angular position.
   *
   * @return The maximum angular position in radians.
   */
  virtual units::radian_t getMaxPosition() const = 0;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() = 0;

  /**
   * Common interface for returning the angular velocity of an encoder.
   *
   * @return The velocity of the encoder in radians per second.
   */
  virtual units::radians_per_second_t getVelocity() const = 0;

  /**
   * Common interface for returning the angular position of an encoder.
   *
   * @return The position of the encoder in radians.
   */
  virtual units::radian_t getPosition() const = 0;

  /**
   * Common interface for setting the position the encoder reports
   *
   * @param position the position the encoder should report
   */
  virtual void setEncoderPosition(units::radian_t position = 0_rad) = 0;

  /**
   * Common interface for getting a controllers tolerance
   */
  virtual units::radian_t getTolerance() const = 0;

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   *
   * @return position error in radians per second.
   */
  virtual units::radian_t getError() const {
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
 * Generates a `linearVelocityController` from a `AngularVelocityController`
 * via a proportional conversion factor. The new controller takes ownership
 * over the old one.
 *
 * @param angularController Controller in angular units form which to create a
 *                          linear controller
 * @param conversion        conversion factor from linear to angular units such
 *                          as a wheel diameter.
 */
std::unique_ptr<LinearPositionController>
asLinear(std::unique_ptr<AngularPositionController> angularController,
         AngularPositionController::ConversionUnit_t conversion);

} // namespace rmb
