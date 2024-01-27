#pragma once

#include <array>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>

#include <units/angle.h>
#include <units/velocity.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/Trajectory.h>

#include <pathplanner/lib/path/PathPlannerTrajectory.h>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc2/command/Commands.h"
#include "networktables/DoubleArrayTopic.h"
#include "pathplanner/lib/commands/FollowPathHolonomic.h"
#include "pathplanner/lib/path/PathConstraints.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "rmb/drive/BaseDrive.h"
#include "rmb/drive/SwerveModule.h"
#include "units/angular_velocity.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include "networktables/DoubleTopic.h"
#include "units/time.h"

#include <rmb/sensors/gyro.h>
#include <vector>

namespace rmb {

/**
 * Class to manage most aspects of a swerve drivetrain from basic teleop
 * drive funtions to odometry and full path following for both WPIL
 ib and
 * PathPlanner trajectories.
 *
 * @tparam NumModules Number fo swerve modules on the drivetrain.
 */
template <size_t NumModules> class SwerveDrive : public BaseDrive {
public:
  SwerveDrive(const SwerveDrive &) = delete;
  SwerveDrive(SwerveDrive &&) = delete;

  /**
   * Constructs a SwerveDrive object that automatically incorrperates
   * vision measurments over the network for odometry.
   *
   * @param modules             Array of swerve modules aprt of the drivetrain.
   * @param gyro                Monitors the robots heading for odometry.
   * @param holonomicController Feedbakc controller for keeping the robot on
   * path.
   * @param visionTable         Path to the NetworkTables table for listening
   *                            for vision updates.
   * @param maxModuleSpeed      Maximum speed any module can turn
   * @param initialPose         Starting position of the robot for odometry.
   *
   */
  SwerveDrive(std::array<SwerveModule, NumModules> modules,
              std::shared_ptr<const rmb::Gyro> gyro,
              frc::HolonomicDriveController holonomicController,
              std::string visionTable,
              units::meters_per_second_t maxModuleSpeed,
              const frc::Pose2d &initialPose = frc::Pose2d());

  /**
   * Constructs a SwerveDrive object that **does not** automatically
   * incorrperates vision measurments over the network for odometry.
   *
   * @param modules             Array of swerve modules aprt of the drivetrain.
   * @param gyro                Monitors the robots heading for odometry.
   * @param holonomicController Feedback controller for keeping the robot on
   *                            path.
   * @param maxModuleSpeed      Maximum speed any module can turn
   * @param initialPose         Starting position of the robot for odometry.
   */
  SwerveDrive(std::array<SwerveModule, NumModules> modules,
              std::shared_ptr<const rmb::Gyro> gyro,
              frc::HolonomicDriveController holonomicController,
              units::meters_per_second_t maxModuleSpeed,
              const frc::Pose2d &initialPose = frc::Pose2d());

  virtual ~SwerveDrive() = default;

  void driveCartesian(double xSpeed, double ySpeed, double zRotation,
                      bool fieldOriented);

  void drivePolar(double speed, const frc::Rotation2d &angle, double zRotation,
                  bool fieldOriented);

  void driveModulePowers(std::array<SwerveModulePower, NumModules> powers);

  void driveModuleStates(std::array<frc::SwerveModuleState, NumModules> states);

  std::array<frc::SwerveModuleState, NumModules> getModuleStates() const;

  std::array<frc::SwerveModulePosition, NumModules> getModulePositions() const;

  const std::array<rmb::SwerveModule, NumModules> &getModules() const {
    return modules;
  }

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

  std::array<frc::SwerveModuleState, NumModules> getTargetModuleStates() const;

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
   *             o





           nt::Now(). This is usually extracted from network
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
  bool isHolonomic() const override { return true; }

  // /**
  //  * Generates a command to follow WPILib Trajectory.
  //  *
  //  * @param trajectory       The trajectory to follow.
  //  * @param driveRequirments The subsystems required for driving the robot
  //  * (ie. the one that contains this class)
  //  *
  //  * @return The command to follow a trajectory.
  //  */
  frc2::CommandPtr followWPILibTrajectory(
      frc::Trajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  // /**
  //  * Generates a command to follow PathPlanner Trajectory.
  //  *
  //  * @param trajectory       The trajectory to follow.
  //  * @param driveRequirments The subsystems required for driving the robot
  //  *                         (ie. the one that contains this class)
  //  *
  //  * @return The command to follow a trajectory.
  //  */
  frc2::CommandPtr followPPPath(
      std::shared_ptr<pathplanner::PathPlannerPath> path,
      std::initializer_list<frc2::Subsystem *> driveRequirements) override;

  frc2::CommandPtr FollowGeneratedPPPath(
      frc::Pose2d targetPose, pathplanner::PathConstraints constraints,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  void updateNTDebugInfo(bool openLoopVelocity = false);

  void stop();

private:
  void recomputeOpenloopInverseKinematicsMatrix();

  //-----------------
  // Network Tables Debugging
  //-----------------

  // std::array<nt::DoublePublisher, NumModules> ntVelocityErrorTopics;
  // std::array<nt::DoublePublisher, NumModules> ntPositionErrorTopics;
  // std::array<nt::DoublePublisher, NumModules> ntPositionTopics;
  // std::array<nt::DoublePublisher, NumModules> ntVelocityTopics;

  nt::DoubleArrayPublisher ntPositionTopics;
  nt::DoubleArrayPublisher ntVelocityTopics;

  nt::DoubleArrayPublisher ntPositionErrorTopics;
  nt::DoubleArrayPublisher ntVelocityErrorTopics;

  nt::DoubleArrayPublisher ntTargetPositionTopics;
  nt::DoubleArrayPublisher ntTargetVelocityTopics;

  //-----------------
  // Drive Variables
  //-----------------

  /**
   * Array of swerve modules being used.
   */
  std::array<SwerveModule, NumModules> modules;

  /**
   * Gyroscope to monitor the heading of the robot.
   */
  std::shared_ptr<const rmb::Gyro> gyro;

  /**
   * Kinematics to convert from module motion to chassis motion and visa versa.
   */
  frc::SwerveDriveKinematics<NumModules> kinematics;

  /**
   * Inverse Kinematics matrix to convert chassis speeds in percentage outputs
   * to module states
   */
  Eigen::Matrix<float, 2 * NumModules, 3> openLoopInverseKinematics;

  /**
   * Controller to help drive follow a path.
   */
  frc::HolonomicDriveController holonomicController;

  //-------------------
  // Odometry Variables
  //-------------------

  /**
   * Object to handle the math behind pose estimation.
   */
  frc::SwerveDrivePoseEstimator<NumModules> poseEstimator;

  /**
   * Mutex to protect position estimations between vision threads.
   */
  mutable std::mutex visionThreadMutex;

  units::meters_per_second_t maxModuleSpeed;

  units::meter_t largestModuleDistance = 1.0_m;
};
} // namespace rmb

#include "SwerveDrive.inl"
