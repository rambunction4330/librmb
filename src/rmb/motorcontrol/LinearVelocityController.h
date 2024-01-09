
#pragma once

#include <memory>

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

namespace rmb {

class AngularVelocityController;

/**
 * Interface for controlling a mechanism's linear velocity used by wrappers of
 * device specific APIs.
 */
class LinearVelocityController {
public:
  virtual ~LinearVelocityController() = default;

  //***************
  // Motor Control
  //***************

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
   * Common interface for setting a mechanism's raw power output.
   */
  virtual void setPower(double power) = 0;

  /**
   * Retrieve the percentage [0.0, 1.0] output of the motor
   */
  virtual double getPower() const = 0;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() = 0;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() = 0;

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
   * Sets the encoder's reported position
   * @param position The position to reset the reference to. Defaults to 0
   */
  virtual void setEncoderPosition(units::meter_t position = 0_m) = 0;

  /**
   * Common interface for getting a controllers tolerance
   */
  virtual units::meters_per_second_t getTolerance() const = 0;

  /**
   * Common interface for getting the error between the velocities controllers
   * target velocity and the actual velocity measured by the encoder.
   *
   * @return position error in radians per second.
   */
  virtual units::meters_per_second_t getError() const {
    return getVelocity() - getTargetVelocity();
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

  //*************
  // Conversions
  //*************

  using ConversionUnit =
      units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;
};

/**
 * Generates a `AngularVelocityController` from a `LinearVelocityController`
 * via a proportional conversion factor. The new controller takes ownership
 * over the old one.
 *
 * @param linearController Controller in linear units form which to create an
 *                         angular controller
 * @param conversion       conversion factor from linear to angular units such
 *                         as a wheel diameter.
 */
std::unique_ptr<AngularVelocityController>
asAngular(std::unique_ptr<LinearVelocityController> linearController,
          LinearVelocityController::ConversionUnit_t conversion);

} // namespace rmb
