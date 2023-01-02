
#pragma once

#include <units/base.h>
#include <units/time.h>

#include "rmb/motorcontrol/VelocityController.h"

namespace rmb {

/**
 * Wraps an existing VelocityController to have a maximum ramp rate.
 */
template <typename DistanceUnit>
class ProfiledVelocityController : public VelocityController<DistanceUnit> {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;
  using AccelerationUnit = units::compound_unit<VelocityUnit, units::inverse<units::seconds>>
  using Acceleration_t = units::unit_t<AccelerationUnit>;

  ProfiledVelocityController(const ProfiledVelocityController&) = delete;
  ProfiledVelocityController(ProfiledVelocityController&&) = default;

  ProfiledVelocityController(std::unique_ptr<VelocityController<DistanceUnit>> controller, 
                             Acceleration_t maxAcceleration) : controller(controller), 
                             maxAcceleration(maxAcceleration), targetVelocity(0.0) {}

  /**
   * Sets the target linear velocity.
   * 
   * @param velocity The target linear velocity in meters per second.
   */
  void setVelocity(Velocity_t velocity) {
    targetVelocity = velocity;
    stopped = false;
  }

  /**
   * Gets the <b>current</b> velocity of the mechanism regardless of 
   * target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  Velocity_t getVelocity() const {
    controller->getVelocity();
  }

  /**
   * Gets the <b>target</b> velocity.
   * 
   * @return The <b>target</b> velocity in meters per second.
   */
  Velocity_t getTargetVelocity() const {
    return targetVelocity;
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * velocity. No defualt implementation is given  since an equality check is 
   * meaningless as some non zero error will always exist.
   * 
   * @return true if it at the target, and false if it is not.
   */
  bool atTarget() const {
    return controller->atTarget() && controller->getTargetVelocity() == targetVelocity;
  }

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  Distance_t getPosition() const {
    controller->getPosition();
  }

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  void setInverted(bool isInverted) {
    controller->getInverted();
  }

  /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  bool getInverted() const {
    controller->getInverted();
  }

  /**
   * Common interface for disabling a mechanism.
   */
  void disable() {
    stopped = true;
    targetVelocity = Velocity_t(0);
    controller->disable();
  }

  /**
   * Common interface to stop the mechanism until `setVelocity` is called again.
   */
  void stop() {
    stopped = true;
    targetVelocity = Velocity_t(0);
    controller->stop();
  }

  void update(units::second_t period = 20_ms) {
    if (stopped) { 
      controller->stop();
      return; 
    }

    Velocity_t velocityIncrement = maxAccleration * period;
    if (abs(getError()) < velocityIncrement) {
      controller->setVelocity(targetVelocity);
    }

    controller->setVelocity(getTargetVelocity() - std::copysign(velocityIncrement, getError()));
  }

private:
  std::unique_ptr<VelocityController<DistanceUnit>> controller;
  const Acceleration_t maxAcceleration;
  Velocity_t targetVelocity;
  bool stopped = true;
};

} // namespace rmb
