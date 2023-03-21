
#pragma once

#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>

#include <units/velocity.h>

#include <frc/controller/RamseteController.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/trajectory/Trajectory.h>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>

#include <frc2/command/CommandPtr.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>

#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {
class DifferentialDrive {
public:
  DifferentialDrive(const DifferentialDrive &) = delete;
  DifferentialDrive(DifferentialDrive &&) = delete;

  DifferentialDrive(std::unique_ptr<LinearVelocityController> left,
                    std::unique_ptr<LinearVelocityController> right,
                    const frc::Gyro &gyro,
                    frc::DifferentialDriveKinematics kinematics,
                    frc::RamseteController ramseteController,
                    std::string visionTable,
                    const frc::Pose2d &initalPose = frc::Pose2d());

  DifferentialDrive(std::unique_ptr<LinearVelocityController> left,
                    std::unique_ptr<LinearVelocityController> right,
                    const frc::Gyro &gyro,
                    frc::DifferentialDriveKinematics kinematics,
                    frc::RamseteController ramseteController,
                    const frc::Pose2d &initalPose = frc::Pose2d());

  ~DifferentialDrive();

  //---------------
  // Drive Methods
  //---------------

  void arcadeDrive(double xSpeed, double zRotation);
  void curvatureDrive(double xSpeed, double zRotation, bool turnInPlace);
  void tankDrive(double leftSpeed, double rightSpeed);

  void driveWheelSpeeds(units::meters_per_second_t leftVelocity,
                        units::meters_per_second_t rightVelocity);
  void driveWheelSpeeds(frc::DifferentialDriveWheelSpeeds wheelSpeeds);
  frc::DifferentialDriveWheelSpeeds getWheelSpeeds() const;

  void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds);
  frc::ChassisSpeeds getChassisSpeeds() const;

  //------------------
  // Odometry Methods
  //------------------

  /**
   * Returns the current poition without modifying it.
   */
  frc::Pose2d getPose() const;

  /**
   * Updates the current position of the robot using encoder and gyroscope
   * data.
   *
   * *Note:* Vision estimations are updated on a separate thread generated at
   * object construction.
   *
   * @return The updated position.
   */
  frc::Pose2d updatePose();

  /**
   * Resets the estimated robot poition.
   *
   * @return The position to reset the robots estimated position to.
   */
  void resetPose(const frc::Pose2d &pose = frc::Pose2d());

  //----------------------
  // Trajectory Following
  //----------------------

  // TODO: Add Trajectory Following Methods

  frc2::CommandPtr followWPILibTrajectory(
      frc::Trajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr followWPILibTrajectoryGroup(
      std::vector<frc::Trajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr followPPTrajectory(
      pathplanner::PathPlannerTrajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr followPPTrajectoryGroup(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr followPPTrajectoryWithEvents(
      pathplanner::PathPlannerTrajectory trajectory,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> evenMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr followPPTrajectoryGroupWithEvents(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr fullPPAuto(
      pathplanner::PathPlannerTrajectory trajectory,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  frc2::CommandPtr fullPPAuto(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

private:
  //-----------------
  // Drive Variables
  //-----------------

  std::unique_ptr<LinearVelocityController> left, right;
  const frc::Gyro &gyro;
  frc::DifferentialDriveKinematics kinematics;
  frc::RamseteController ramseteController;

  //-------------------
  // Odometry Variables
  //-------------------

  /** Object to handle the math behind pose estimation. */
  frc::DifferentialDrivePoseEstimator poseEstimator;

  //---------------
  // Vision Thread
  //---------------

  /**
   * Network Tables subscriber for vision position predictions.
   *
   * Data is sent in a three entry double array ordered X, Y, Theta. X and Y
   * are in meters and Theta is in radians.
   */
  nt::DoubleArraySubscriber poseSubscriber;

  /**
   * Network Tables subscriber for the standard deviation of vision predictions.
   *
   * Data is sent in a three entry double array ordered X, Y, Theta. X and Y
   * are in meters and Theta is in radians. The standard deviations are a
   * measure of how much the reported position typically deviates from the true
   * position. Larger values denote less accuracy in vision data, while smaller
   * values denote higher accuracy.
   */
  nt::DoubleArraySubscriber stdDevSubscriber;

  /** Handle for keeping track of vision position callback for position. */
  NT_Listener poseListener;

  /** Handle for keeping track of vision position standard deviations callback
   * for position.
   */
  NT_Listener stdDevListener;

  /** mutex to protect position estimations between vision threads. */
  mutable std::mutex visionThreadMutex;
};
} // namespace rmb
