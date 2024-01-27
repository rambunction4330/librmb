#pragma once

#include <initializer_list>
#include <memory>
#include <string>
#include <unordered_map>

#include <units/velocity.h>

#include <frc/controller/RamseteController.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/trajectory/Trajectory.h>

#include <frc2/command/CommandPtr.h>

#include <pathplanner/lib/path/PathPlannerTrajectory.h>

#include "pathplanner/lib/path/PathPlannerPath.h"
#include "rmb/drive/BaseDrive.h"
#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {

/**
 * Class to manage most aspects of a differential drivetrain from basic teleop
 * drive funtions to odometry and full path following for both WPILib and
 * PathPlanner trajectories.
 */
class DifferentialDrive : public BaseDrive {
public:
  DifferentialDrive(const DifferentialDrive &) = delete;
  DifferentialDrive(DifferentialDrive &&) = delete;

  /**
   * Constructs a DifferentialDrive object that automatically incorrperates
   * vision measurments over the network for odometry.
   *
   * @param left              Controls and monitors left side wheel speeds.
   * @param right             Controls and monitors right side wheel speeds.
   * @param gyro              Monitors the robots heading for odometry.
   * @param kinematics        Kinematic modle for converting from wheel states
   *                          to chassis states.
   * @param ramseteController Feedbakc controller for keeping the robot on path.
   * @param visionTable       Path to the NetworkTables table for listening
   *                          for vision updates.
   * @param initialPose       Starting position of the robot for odometry.
   */
  DifferentialDrive(std::unique_ptr<LinearVelocityController> left,
                    std::unique_ptr<LinearVelocityController> right,
                    std::shared_ptr<const frc::Gyro> gyro,
                    frc::DifferentialDriveKinematics kinematics,
                    frc::RamseteController ramseteController,
                    std::string visionTable,
                    const frc::Pose2d &initalPose = frc::Pose2d());

  /**
   * Constructs a DifferentialDrive object that **does not** sutomatically
   * incorrperates vision measurments over the network for odometry.
   *
   * @param left              Controls and monitors left side wheel speeds.
   * @param right             Controls and monitors right side wheel speeds.
   * @param gyro              Monitors the robots heading for odometry.
   * @param kinematics        Kinematic modle for converting from wheel states
   *                          to chassis states.
   * @param ramseteController Feedbakc controller for keeping the robot on path.
   * @param initialPose       Starting position of the robot for odometry.
   */
  DifferentialDrive(std::unique_ptr<LinearVelocityController> left,
                    std::unique_ptr<LinearVelocityController> right,
                    std::shared_ptr<const frc::Gyro> gyro,
                    frc::DifferentialDriveKinematics kinematics,
                    frc::RamseteController ramseteController,
                    const frc::Pose2d &initalPose = frc::Pose2d());

  //---------------
  // Drive Methods
  //---------------

  /**
   * Drives the robot according to the arcade algorithm which the xSpeed is
   * added ot both sides while rotation is addef ot the left and subtracted
   * from the right. This tends to be the most natural method for a human
   * driver, but is quite useless for autonomouse driving since  the speed of
   * the motors is not controlled, just the power output.
   *
   * @param xSpeed      Desired forward "speed" of the robot form -1.0 to +1.0.
   * @param zRotation   Desired turnign rate of the robot from -1.0 to +1.0.
   */
  void arcadeDrive(double xSpeed, double zRotation);

  /**
   * Drives the robot according to the curvature algorithm in which the turning
   * radius of the robot is independedn from the forward speed. This is most
   * useful for teleoperated driving as the speed of the motors is not
   * controlled, just  the power output of each motor. This is more natural
   * for human input, but far less accurate for autonomus driving.
   *
   * @param xSpeed      Desired forward "speed" of the robot form -1.0 to +1.0.
   * @param zRotation   Desired turnign rat eof teh robot from -1.0 to +1.0.
   * @param turnInPlace When true, the robot defaults back to an arcade drive
   *                    so teh robot can turn in place.
   */
  void curvatureDrive(double xSpeed, double zRotation, bool turnInPlace);

  /**
   * Drives the robot according to the tank algorithm where the power of the
   * motor on each side of the robot is directly controlled.This is most
   * useful for teleoperated driving as the speed of the motors is not
   * controlled, just  the power output of each motor. This is more natural
   * for human input, but far less accurate for autonomus driving.
   *
   * @param leftSpeed  Desired power or the left motor from -1.0 to +1.0.
   * @param rightSpeed Desired power or the right motor from -1.0 to +1.0.
   */
  void tankDrive(double leftSpeed, double rightSpeed);

  /**
   * Drives the wheels of the robot at the requested speeds
   *
   * @param leftVelocity  Requested speed of the left side wheels.
   * @param rightVelocity Requested speed of the right side wheels.
   */
  void driveWheelSpeeds(units::meters_per_second_t leftVelocity,
                        units::meters_per_second_t rightVelocity);

  /**
   * Drives the wheels of the robot at the requested speeds.
   *
   * @param wheelSpeeds Requested speeds of the robots wheels
   */
  void driveWheelSpeeds(frc::DifferentialDriveWheelSpeeds wheelSpeeds);

  /**
   * Returns the current speeds of the robot's wheels.
   */
  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() const;

  /**
   * Drives the robot via the speeds of the Chassis.
   *
   * @param chassisSpeeds Desired speeds of the robot Chassis.
   */
  void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) override;

  /**
   * Returns the speeds of the robot chassis.
   */
  frc::ChassisSpeeds getChassisSpeeds() const override;

  //------------------
  // Odometry Methods
  //------------------

  /**
   * Returns the current poition without modifying it.
   */
  frc::Pose2d getPose() const override;

  /**
   * Updates the current position of the robot using encoder and gyroscope
   * data.
   *
   * *Note:* Vision estimations are updated on a separate thread generated at
   * object construction.
   *
   * @return The updated position.
   */
  frc::Pose2d updatePose() override;

  /**
   * Resets the estimated robot poition.
   *
   * @return The position to reset the robots estimated position to.
   */
  void resetPose(const frc::Pose2d &pose = frc::Pose2d()) override;

  /**
   * Updates the current position of the robot using latency compensated vision
   * data.
   *
   * @param poseEstimate The estimated position of the robot from vision.
   * @param time         The time at which the data that produces this
   *                     estimate was captures. This is an absolute time with
   *                     with the zero eposh being the same as wpi::Now() and
   *                     nt::Now(). This is usually extracted from network
   *                     tables.
   */
  void addVisionMeasurments(const frc::Pose2d &poseEstimate,
                            units::second_t time) override;

  /**
   * Change accuratly vision data is expected to be.
   *
   * @param standardDevs The standar deviation of vision measurments. This is
   *                     by how much teh actual robot positioon varies from
   *                     the actual posiotion of the robot. A larger value
   *                     means less acurate data. These are in units of meters
   *                     and radians ordered X, Y, Theta.
   */
  void setVisionSTDevs(wpi::array<double, 3> standardDevs) override;

  //----------------------
  // Trajectory Following
  //----------------------

  /**
   * Returns wherther the drive train is holonomic, meanign can move in all
   * directions. This is nessesry for determining how to rezero a robot at the
   * beginning of a path.
   */
  bool isHolonomic() const override { return false; }

  /**
   * Generates a command to follow WPILib Trajectory.
   *
   * @param trajectory       The trajectory to follow.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory.
   */
  frc2::CommandPtr followWPILibTrajectory(
      frc::Trajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  /**
   * Generates a command to follow PathPlanner Trajectory.
   *
   * @param trajectory       The trajectory to follow.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory.
   */
  frc2::CommandPtr followPPPath(
      std::shared_ptr<pathplanner::PathPlannerPath> path,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

private:
  //-----------------
  // Drive Variables
  //-----------------

  /**
   * Controls and monitors the speeds of the wheels on the left side of the
   * robot. */
  std::unique_ptr<LinearVelocityController> left;

  /**
   * Controls and monitors the speeds of the wheels on the right side of the
   * robot.
   */
  std::unique_ptr<LinearVelocityController> right;

  /**
   * Gyroscope to monitor the heading of the robot.
   */
  std::shared_ptr<const frc::Gyro> gyro;

  /**
   * Kinematics to convert from wheel motion to chassis motion and visa versa.
   */
  frc::DifferentialDriveKinematics kinematics;

  /**
   * Feedback controller for following trajectories.
   */
  frc::RamseteController ramseteController;

  //-------------------
  // Odometry Variables
  //-------------------

  /**
   * Object to handle the math behind pose estimation.
   */
  frc::DifferentialDrivePoseEstimator poseEstimator;

  /**
   * Mutex to protect position estimations between vision threads.
   */
  mutable std::mutex visionThreadMutex;
};
} // namespace rmb
