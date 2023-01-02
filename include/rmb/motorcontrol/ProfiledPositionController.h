
#pragma once

#include <frc/Servo.h>

#include "rmb/motorcontrol/PositionController.h"
#include "rmb/motorcontrol/ProfiledVelocityController.h"

namespace rmb {

/**
 * Interface for controlling a mechanism's linear position used by wrappers of 
 * device specific APIs.
 */
template <typename DistanceUnit>
class ProfiledPositionController : public PositionController<DistanceUnit> {
public:
  using Distance_t = units::unit_t<DistanceUnit>;
  using VelocityUnit = units::compound_unit<DistanceUnit, units::inverse<units::seconds>>;
  using Velocity_t = units::unit_t<VelocityUnit>;
  using AccelerationUnit = units::compound_unit<VelocityUnit, units::inverse<units::seconds>>;
  using Acceleration_t = units::unit_t<AccelerationUnit>;

  ProfiledPositionController(const ProfiledPositionController&) = delete;
  ProfiledPositionController(ProfiledPositionController&&) = default;

  ProfiledPositionController(std::unique_ptr<rmb::VelocityController> controller, 
                             Velocity_t maxVelocity, Acceleration_t maxAcceleration, Distance_t allowableError) :
                             velocityController(new ProfiledVelocityController(controller, maxAcceleration)),
                             maxAcceleration(maxAcceleration), maxVelocity(maxVelocity), allowableError(allowableError),
                             targetPosition(controller->getPosition()) {}

  /**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  void setPosition(Distance_t position) {
    targetPosition = position;
  }

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  Distance_t getPosition() const {
    velocityController->getPosition();
  }

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  Distance_t getTargetPosition() const {
    return targetPosition;
  }

  /**
   * Common interface for returning whether the mechanism is at the target 
   * position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  bool atTarget() const {
    return abs(getError()) < allowableError;
  }

  /**
   * Common interface for getting the <b>current</b> linear velocity of the
   * mechanism regardless of target.
   * 
   * @return The <b>current</b> linear velocity in meters per second.
   */
  Velocity_t getVelocity() const {
    velocityController->getVelocity();
  }

  /**
   * Common interface for inverting direction of a mechanism.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  void setInverted(bool isInverted) {
    velocityController->setInverted(isInverted);
  }

    /**
   * Common interface for returning the inversion state of a mechanism.
   *
   * @return isInverted The state of inversion, true is inverted.
   */
  bool getInverted() const {
    return velocityController->getInverted();
  }

  /**
   * Common interface for disabling a mechanism.
   */
  void disable() {
    velocityController->disable();
  }

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  void stop() {
    velocityController->stop();
  }

  void update(units::second_t period = 20_ms) {
    if (abs(getError() < allowableError) {
      velocityController->setVelocity(Velocity_t(0.0));
      return;
    }

    Distance_t decelerationDist = (getTargetVelocity() * getTargetVelocity()) / maxAcceleration;

    if (abs(getError()) < decelerationDist) {
      velocityController->setVelocity(Velocity_t(0.0));   
    } else {
      velocityController->setVelocity(-std::copysign(maxVelocity, getError()));
    }
  }

private:
  std::unique_ptr<rmb::ProfiledVelocityController> velocityController;
  Distance_t targetPosition;
  Velocity_t maxVelocity;
  Acceleration_t maxAcceleration;
  Distance_t allowableError;
};
} // namespace rmb
