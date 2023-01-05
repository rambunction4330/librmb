
#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include "rmb/motorcontrol/AngularPositionController.h"


namespace rmb {

/**
 * Interface for controlling a mechanism's linear position used by wrappers of 
 * device specific APIs.
 */
class LinearPositionController {
public:

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
   * Common interface for setting the minimum linear position.
   * 
   * @param min The minimum linear position in meters.
   */
  virtual void setMinPosition(units::meter_t min) = 0;

  /**
   * Common interface for getting the minimum linear position.
   * 
   * @return The minimum linear position in meters.
   */
  virtual units::meter_t getMinPosition() const = 0;

  /**
   * Common interface for setting the maximum linear position.
   * 
   * @param max  The maximum linear position in meters.
   */
  virtual void setMaxPosition(units::meter_t max) = 0;

  /**
   * Common interface for getting the maximum linear position.
   * 
   * @return The maximum linear position in meters.
   */
  virtual units::meter_t getMaxPosition() const = 0;

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
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() = 0;

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  /**
   * Generates a `AngularVelocityController` to controll the same mechanism
   * as this controller, but with angular instead of linear units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they control the same physical mechanism.
   * 
   * @param conversion conversion from linear to angular units.
   */
  LinearAsAngularPositionController asAngularController(ConversionUnit_t conversion) {
    return LinearAsAngularPositionController(*this, conversion);
  }
};

// Simple wrapper class to handle unit conversions
class LinearAsAngularPositionController : public AngularPositionController {
public:

  using ConversionUnit = units::compound_unit<units::meters, units::inverse<units::radians>>;
  using ConversionUnit_t = units::unit_t<ConversionUnit>;

  LinearAsAngularPositionController(const LinearAsAngularPositionController&) = default;
  LinearAsAngularPositionController(LinearAsAngularPositionController&&) = delete;

  LinearAsAngularPositionController(LinearPositionController& linearController, 
                                    ConversionUnit_t conversionFactor) :
                                    linear(linearController), conversion(conversionFactor) {}

  void setPosition(units::radian_t position) { linear.setPosition(position * conversion); }
  units::radian_t getTargetPosition() const { return linear.getTargetPosition() / conversion; }
  void setMinPosition(units::radian_t min) { linear.setMinPosition(min * conversion); }
  units::radian_t getMinPosition() const { return linear.getMinPosition() / conversion; }
  void setMaxPosition(units::radian_t max) { linear.setMaxPosition(max * conversion); }
  units::radian_t getMaxPosition() const { return linear.getMaxPosition() / conversion; }
  void setInverted(bool isInverted) { linear.setInverted(isInverted); }
  bool getInverted() const {linear.getInverted(); }
  void disable() { linear.disable(); }
  void stop() { linear.stop(); }

private:
  LinearPositionController& linear;
  ConversionUnit_t conversion;
};

} // namespace rmb
