#pragma once

#include <memory>

#include <units/angle.h>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "frc/drive/RobotDriveBase.h"
#include "frc/geometry/Rotation2d.h"
#include "rmb/motorcontrol/AngularPositionController.h"
#include "rmb/motorcontrol/LinearVelocityController.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableHelper.h"

namespace rmb {

/**
 * Open loop state of a swerve module.
 *
 * This is often useful for teleoperated driving since it tends to more
 * natural for a human driver to controll the power of a motor in open loop
 * operation rather than the velocity in a closed loop mode (closed loop
 * operation is, of course, not possible for angle control).
 */
struct SwerveModulePower {
  double power;          /* <- Power to drive swerve module at. */
  frc::Rotation2d angle; /* <- Angel to drive swerve module at. */

  /**
   * Minimize the change in heading the desired power would require by
   * potentially reversing the direction the wheel spins so the furthest a
   * wheel will ever rotate is 90 degrees.
   *
   * @param desiredPower The desired power.
   * @param currentAngle The current module angle.
   *
   * @return Optomized module power.
   */
  static SwerveModulePower Optimize(const SwerveModulePower &desiredPower,
                                    const frc::Rotation2d &currentAngle);
};

/**
 * Class managing the motion of a swerve module
 */
class SwerveModule : public wpi::Sendable,
                     public wpi::SendableHelper<SwerveModule> {
public:
  SwerveModule(const SwerveModule &) = delete;
  SwerveModule(SwerveModule &&) = default;

  /**
   * Constructs a SwerveModule object for controlling module states.
   *
   * @param angularController Controller of the angle of module.
   * @param angularController Controller of the velocioty of module.
   * @param moduleTranslation the position of teh modlue reletive to the
   *                          center of the robot for kinematics.
   */
  SwerveModule(std::unique_ptr<LinearVelocityController> velocityController,
               std::unique_ptr<AngularPositionController> angularController,
               const frc::Translation2d &moduleTranslation,
               bool breakMode = false);

  /**
   * Sets the desired state of the swerve module.
   *
   * @param velocity The desired speed of the swerve module.
   * @param angle    The desired angle of the swerve module.
   */
  void setState(const units::meters_per_second_t &velocity,
                const frc::Rotation2d &angle);

  void smartdashboardDisplayTargetState(const std::string &name) const;

  /**
   * Sets the desired state of the swerve module.
   *
   * @param state The desired state of the module.
   */
  void setState(const frc::SwerveModuleState &state);

  /**
   * Returns the current state of the module.
   */
  frc::SwerveModuleState getState() const;

  /**
   * Returns the current position of the module useful for more accurate
   * odometry.
   */
  frc::SwerveModulePosition getPosition() const;

  /**
   * @return The target state of the module. This is useful for debugging.
   */
  frc::SwerveModuleState getTargetState() const;

  units::meters_per_second_t getTargetVelocity() const;
  frc::Rotation2d getTargetRotation() const;

  /**
   * Sets the desired open loop powerof the swerve module. Tghis is helpful
   * for teleoperated driving as drivers tend power based control to be more
   * natural.
   *
   * @param power The desired power output of the swerve module.
   * @param angle The desired angle of the swerve module.
   */
  void setPower(double power, const frc::Rotation2d &angle);

  /**
   * Sets the desired open loop power of the swerve module. This is helpful
   * for teleoperated driving as drivers tend power based control to be more
   * natural.
   *
   * @param power The desired power output of the swerve module.
   */
  void setPower(const SwerveModulePower &power);

  void stop();

  SwerveModulePower getPower();

  /**
   * Returns the position fo the module reletive to the center of the robot
   * for kinematics.
   */
  const frc::Translation2d &getModuleTranslation() const;

  virtual void InitSendable(wpi::SendableBuilder &builder) override;

  double getAngle() {
    return ((units::angle::degree_t)angularController->getPosition())();
  }

private:
  /**
   * Controls the angle of the module.
   */
  std::unique_ptr<AngularPositionController> angularController;

  /**
   * Controls the velocity of the module.
   */
  std::unique_ptr<LinearVelocityController> velocityController;

  /**
   * The position of the module relative to the center of the robot for
   * kinematics.
   */
  frc::Translation2d moduleTranslation;

  bool breakMode;
};
} // namespace rmb
