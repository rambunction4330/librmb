
#pragma once

#include <units/base.h>
#include <units/time.h>

#include "rmb/motorcontrol/AngularVelocityController.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's velocity used by wrappers of 
 * device specific APIs.
 */
template <typename DistanceUnit>
class VelocityController {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;

  /**
   * Common interface for setting the target velocity.
   * 
   * @param velocity The target velocity in user defined units.
   */
  virtual void setVelocity(Velocity_t velocity) = 0;

  /**
   * Common interface for getting the <b>current</b> velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> velocity in user defined units.
   */
  virtual Velocity_t getVelocity() const = 0;

  /**
   * Common interface for getting the <b>target</b> velocity.
   * 
   * @return The <b>target</b> velocity in user defined units.
   */
  virtual Velocity_t getTargetVelocity() const = 0;

  /**
   * Common interface for getting the <b>current</b> velocity error.
   * 
   * @return The diffrence between the actual and target velocitys.
   */
  virtual Velocity_t getError() const {
    return getVelocity() - getTargetVelocity();
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * velocity. No defualt implementation is given  since an equality check is 
   * meaningless as some non zero error will always exist.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for getting the <b>current</b> position.
   * 
   * @return The <b>current</b> position in user defined units.
   */
  virtual Distance_t getPosition() const = 0;

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

  /**
   * Generates a `AngularVelocityController` to controll the same mechanism
   * as this controller, but with a different tyle of units (almost certainly
   * angular ie. radians).
   * 
   * @param conversion conversion factor
   */
  template<typename NewDistanceUnit>
  std::unique_ptr<AngularVelocityController> getAngularUnits(units::unit_t<units::compound_unit<NewDistanceUnit, units::inverse<DistanceUnit>>> conversion) {
    return std::unique_ptr<VelocityController<NewDistanceUnit>>(new VelocityControllerConverter<DistanceUnit, NewDistanceUnit>(*this, conversion));
  }
};

// Simple wrapper class to handle unit conversions
template <typename OldDistanceUnit, typename NewDistanceUnit>
class VelocityControllerConverter: public VelocityController<NewDistanceUnit> {
 public:
  VelocityControllerConverter(LinearVelocityController& linearController, 
                              units::unit_t<units::compound_unit<NewDistanceUnit, units::inverse<DistanceUnit>>> conversion) :
                              linearController(linearController), conversion(conversion) {}

  void setAngularVelocity(units::radians_per_second_t velocity) { oldController.setVelocity(velocity / conversion); }
  units::radians_per_second_t getAngularVelocity() const  { return oldController.getVelocity() * conversion; }
  units::radians_per_second_t getTargetAngularVelocity() const { return oldController.getTargetVelocity() * conversion; }
  bool atTarget() const { return oldController.atTarget(); }
  units::radian_t getAngularPosition() const { return oldController.getPosition() * conversion; }
  void setInverted(bool isInverted) { oldController.setInverted(isInverted); }
  bool getInverted() const { return oldController.getInverted(); }
  void disable() { oldController.disable(); }
  void stop() { oldController.stop(); }
 private:
  VelocityController<OldDistanceUnit>& oldController;
  units::unit_t<units::compound_unit<NewDistanceUnit, units::inverse<DistanceUnit>>> conversion;
};
} // namespace rmb
