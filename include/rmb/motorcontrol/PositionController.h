#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace rmb {

/**
 * Interface for controlling a mechanism's position used by wrappers of device 
 * specific APIs. The interface is meant to descibe what a mechanism should do, 
 * not how it should do it. Methods are given in both linear and angular 
 * units but controll the same mechanism, so changes to the target linear 
 * position also modify the target angular position and vice versa. Linear 
 * methods are given default implintations that rely on angular the methods 
 * under the assumtion that the mechanism is powered by a motor and that the 
 * conversion from angular to linear units id defined for all angles. If this 
 * is not the case, re-implement or mark methods as unimplemented as nessesary.
 * 
 * @see SparkMaxPositionController TalonSRXPositionController 
 *      FalconPositionController Servo
 */
class PositionController {
public:

  /**
   * Common interface for setting the target linear position. 
   * 
   * @param position The target linear position in meters.
   */
  virtual void setPosition(units::meter_t position);

  /**
   * Common interface for setting the target angular position. 
   * 
   * @param position The target angular position in radians.
   */
  virtual void setAngularPosition(units::radian_t position) = 0;

  /**
   * Common interface for setting the target linear position as an offset 
   * from the current target. 
   * 
   * @param position The offset from the previouse target in meters.
   */
  virtual void setPositionOffset(units::meter_t position);

  /**
   * Common interface for setting the target angular position as offset from 
   * the previouse target.
   * 
   * @param position The offset from the previouse target in radians.
   */
  virtual void setAngularPositionOffset(units::radian_t position);

  /**
   * Common interface for getting the <b>current</b> linear position.
   * 
   * @return The <b>current</b> linear position in meters
   */
  virtual units::meter_t getPosition() const;

  /**
   * Common interface for getting the <b>current</b> angular position.
   * 
   * @return the <b>current</b> angular position in radians.
   */
  virtual units::radian_t getAngularPosition() const = 0;

  /**
   * Common interface for getting the <b>target</b> linear position.
   * 
   * @return The <b>target</b> linear position in meters.
   */
  virtual units::meter_t getTargetPosition() const;

  /**
   * Common interface for getting the <b>target</b> angular position.
   * 
   * @return The <b>target</b> angular position in meters.
   */
  virtual units::radian_t getTargetAngularPosition() const = 0;

  /**
   * Common interface for getting the <b>current</b> linear position error.
   * 
   * @return The diffrence between the actual linear position and target linear 
   *         position in meters.
   */
  virtual units::meter_t getError() const;

  /**
   * Common interface for getting the <b>current</b> angular velocity error. 
   * 
   * @return The diffrence between the actual angular velocity and target 
   *         angular velocity in radians.
   */
  virtual units::radian_t getAngularError() const;

  /**
   * Common interface for returning whether the mechanism is at the target 
   * position.
   * 
   * @return true if it at the target, and false if it is not.
   */
  virtual bool atTarget() const = 0;

  /**
   * Common interface for zeroing the linear position such that the 
   * <b>target</b> linear position is remapped to the provided position. If the
   * provided offset would put the mechanism out of bounds, the next clossest 
   * offset used and the function returns false.
   * 
   * <p><b>Note/b> this function will also remapp min and max.</p>
   * 
   * @param position The values the <b>target</b> linear position will be 
   *                 remapped to.
   */
  virtual bool zeroPosition(units::meter_t offset = 0_m);

  /**
   * Common interface for zeroing the angular position such that the 
   * <b>target</b> angular position is remapped to the given position. If the
   * provided offset would put the mechanism out of bounds, the next clossest 
   * offset used and the function returns false. 
   * 
   * <p><b>Note</b> this function will also remapp min and max values.</p> 
   * 
   * @param position The values the <b>target</b> angular position will be
   *                 remapped to.
   */
  virtual bool zeroAngularPosition(units::radian_t offset = 0_rad) = 0;

  /**
   * Common interface for setting the maximum linear position. If the given
   * maximum is invalid the closest valid maximum is used and the function 
   * returns false.
   * 
   * @param max The maximum linear position in meters.
   */
  virtual bool setMaxPosition(units::meter_t max);

  /**
   * Common interface for setting the maximum angular position. If the given
   * maximum is invalid the closest valid maximum is used and the function 
   * returns false.
   * 
   * @param max The maximum angular position in radians.
   */
  virtual bool setMaxAngularPosition(units::radian_t max) = 0;

  /**
   * Common interface for getting the maximum linear position.
   * 
   * @return The maximum linear position in meters.
   */
  virtual units::meter_t getMaxPosition() const;

  /**
   * Common interface for getting the maximum angular position.
   * 
   * @return The maximum angular position in radians.
   */
  virtual units::radian_t getMaxAngularPosition() const = 0;

  /**
   * Common interface for setting the minimum linear position. If the given
   * minimum is invalid the closest valid minimum is used and the function 
   * returns false.
   * 
   * @param min The minimum linear position in meters.
   */
  virtual bool setMinPosition(units::meter_t min);

  /**
   * Common interface for setting the minimum angular position. If the given
   * minimum is invalid the closest valid minimum is used and the function 
   * returns false.
   * 
   * @param min The minimum angular position in radians.
   */
  virtual bool setMinAngularPosition(units::radian_t min) = 0;

  /**
   * Common interface for getting the minimum linear position.
   * 
   * @return The minimum linear position in meters.
   */
  virtual units::meter_t getMinPosition() const;

    /**
   * Common interface for sgetting the minimum angular position.
   * 
   * @return The minimum angular position in radians.
   */
  virtual units::radian_t getMinAngularPosition() const = 0;

  /**
   * Common interface for converting from linear to angular units. A functions
   * is used rather than a conversion factor since some conversions may be
   * non-linear. 
   * 
   * @param linear The linear distance in meters.
   * @return The angular distance in radians.
   */
  virtual units::radian_t linearToAngular(units::meter_t linear) const = 0;

  /**
   * Common interface for converting from angular to linear units. A functions
   * is used rather than a conversion factor since some conversions may be
   * non-linear. 
   * 
   * @param angle The linear distance in radians.
   * @return The linear distance in meters.
   */
  virtual units::meter_t angularToLinear(units::radian_t angle) const = 0;

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
  virtual void stopMotor() = 0;
};

} // namespace rmb
