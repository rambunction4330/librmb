
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

#include "rmb/drive/BaseDrive.h"
#include "rmb/motorcontrol/LinearVelocityController.h"

namespace rmb {
class DifferentialDrive : public BaseDrive {
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

  void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) override;
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

  frc2::CommandPtr followWPILibTrajectory(
      frc::Trajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr followWPILibTrajectoryGroup(
      std::vector<frc::Trajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr followPPTrajectory(
      pathplanner::PathPlannerTrajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr followPPTrajectoryGroup(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr followPPTrajectoryWithEvents(
      pathplanner::PathPlannerTrajectory trajectory,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr followPPTrajectoryGroupWithEvents(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr fullPPAuto(
      pathplanner::PathPlannerTrajectory trajectory,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr fullPPAuto(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

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
};
} // namespace rmb
