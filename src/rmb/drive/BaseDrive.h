#pragma once

#include <initializer_list>
#include <string>

#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/Trajectory.h>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>

namespace rmb {
class BaseDrive {
public:
  //---------------
  // Drive Methods
  //---------------

  /**
   * Drives the robot via the speeds of the Chassis.
   *
   * @param chassisSpeeds Desired speeds of the robot Chassis.
   */
  virtual void driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) = 0;

  /**
   * Returns the speeds of the robot chassis.
   */
  virtual frc::ChassisSpeeds getChassisSpeeds() const = 0;

  //------------------
  // Odometry Methods
  //------------------

  /**
   * Returns the current poition estimation without modifying it.
   */
  virtual frc::Pose2d getPose() const = 0;

  /**
   * Updates the current position of the robot using encoder and gyroscope
   * data.
   *
   * @return The updated position.
   */
  virtual frc::Pose2d updatePose() = 0;

  /**
   * Resets the estimated robot poition.
   *
   * @return The position to reset the robots estimated to.
   */
  virtual void resetPose(const frc::Pose2d &pose = frc::Pose2d()) = 0;

  //----------------------
  // Trajectory Following
  //----------------------

  /**
   * Generates a command to follow WPILib Trajectory.
   *
   * @param trajectory       The trajectory to follow.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory.
   */
  virtual frc2::CommandPtr followWPILibTrajectory(
      frc::Trajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirments) = 0;

  /**
   * Generates a command to follow a vector of WPILib Trajectories. This is
   * helpful if the robot should stop in the middle of a path or changes
   * direction.
   *
   * @param trajectorygroup  The vector of trajectories to follow.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a list of trajectories.
   */
  virtual frc2::CommandPtr followWPILibTrajectoryGroup(
      std::vector<frc::Trajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirments);

  /**
   * Generates a command to follow PathPlanner Trajectory.
   *
   * @param trajectory       The trajectory to follow.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory.
   */
  virtual frc2::CommandPtr followPPTrajectory(
      pathplanner::PathPlannerTrajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirments) = 0;

  /**
   * Generates a command to follow a vector of PathPlanner Trajectories. This
   * is helpful if the robot should stop in the middle of a path or changes
   * direction.
   *
   * @param trajectorygroup  The vector of trajectories to follow.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a list of trajectories.
   */
  virtual frc2::CommandPtr followPPTrajectoryGroup(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirments);

  /**
   * Generates a command to follow a PathPlanner Trajectories with events.
   * When each event is triggered, the robot will execute the corosponding
   * commmand in the event map while continuing to follow the path.
   *
   * @param trajectory       The trajectory to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectory.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory with events.
   */
  virtual frc2::CommandPtr followPPTrajectoryWithEvents(
      pathplanner::PathPlannerTrajectory trajectory,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirments);

  /**
   * Generates a command to follow a vector of PathPlanner Trajectories with
   * events. When each event is triggered, the robot will execute the
   * corosponding commmand in the event map while continuing to follow the
   * path.
   *
   * @param trajectoryGroup  The vector of trajectories to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectories.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a list of trajectories with events.
   */
  virtual frc2::CommandPtr followPPTrajectoryGroupWithEvents(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirments);

  /**
   * Generates a command to complete a full autonomouse routine generated by
   * PathPlanner. Beyond just a path with events, this includes resetting the
   * position at the start of the path and executing stop events.
   *
   * @param trajectory       The trajectory to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectories.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a full autonomus routine.
   */
  virtual frc2::CommandPtr fullPPAuto(
      pathplanner::PathPlannerTrajectory trajectory,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirments);

  /**
   * Generates a command to complete a full autonomouse routine generated by
   * PathPlanner. Beyond just a path with events, this includes resetting the
   * position at the start of the path and executing stop events.
   *
   * @param trajectoryGroup  The vector of trajectories to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectories.
   * @param driveRequirments The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a full autonomus routine.
   */
  virtual frc2::CommandPtr fullPPAuto(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirments) = 0;
};
} // namespace rmb