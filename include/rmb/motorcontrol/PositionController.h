
#pragma once

#include <units/length.h>
#include <units/velocity.h>

#include "rmb/motorcontrol/AngularPositionController.h"


namespace rmb {

/**
 * Interface for controlling a mechanism's linear position used by wrappers of 
 * device specific APIs.
 */
template <typename DistanceUnit>
class PositionController {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;

  /**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  virtual void setPosition(Distance_t position) = 0;

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  virtual Distance_t getPosition() const = 0;

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  virtual Distance_t getTargetPosition() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear position error.
   * 
   * @return The diffrence between the actual linear position and target linear 
   *         position in meters.
   */
  Distance_t getError() const {
    return getPosition() - getTargetPosition();
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  virtual Velocity_t getVelocity() const = 0;

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

  /**
   * Generates a `AngularVelocityController` to controll the same mechanism
   * as this controller, but with angular instead of linear units via a linear
   * conversion factor. Changes to one controller will effect the other since 
   * they control the same physical mechanism.
   * 
   * @param conversion conversion from linear to angular units.
   */
  template <typename NewDistanceUnit>
  std::unique_ptr<PositionController<NewDistanceUnit>> getAngularUnits(units::unit_t<units::compound_unit<NewDistanceUnit, units::inverse<DistanceUnit>>> conversion) {
    return std::unique_ptr<PositionController<NewDistanceUnit>>(new PositionControllerConverter<DistanceUnit, NewDistanceUnit>(*this, conversion));
  }
};

// Simple wrapper class to handle unit conversions
template <typename OldDistanceUnit, typename NewDistanceUnit>
class PositionControllerConverter : public PositionController<NewDistanceUnit> {
public:
  PositionControllerConverter(PositionController<OldDistanceUnit>& oldController, 
                              units::unit_t<units::compound_unit<NewDistanceUnit, units::inverse<DistanceUnit>>> conversion) :
                              oldController(oldController), conversion(conversion) {}

  void setAngularPosition(Distance_t position) { oldController.setPosition(position / conversion); }
  Distance_t getAngularPosition() const { return oldController.getPosition() * conversion; }
  Distance_t getTargetAngularPosition() const { return oldController.getTargetPosition() * conversion; }
  bool atTarget() const { return oldController.atTarget(); }
  Velocity_t getAngularVelocity() const { return oldController.getVelocity() * conversion; }
  void setInverted(bool isInverted) { oldController.setInverted(isInverted); }
  bool getInverted() const { oldController.getInverted(); }
  void disable() { oldController.disable(); }
  void stop() { oldController.stop(); }

private:
  PositionController<OldDistanceUnit>& oldController;
  units::unit_t<units::compound_unit<NewDistanceUnit, units::inverse<DistanceUnit>>> conversion;
};

} // namespace rmb
