#pragma once

#include "pathplanner/lib/path/PathPlannerPath.h"
#include <initializer_list>
#include <memory>
#include <string>
#include <unordered_map>

#include <units/time.h>

#include <wpi/array.h>

#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/Trajectory.h>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>

#include <pathplanner/lib/path/PathPlannerTrajectory.h>

namespace rmb {

/**
 * Base interface for robot drive classes that can handle automatic updating
 * of vision based odometry and more complex path following.
 */
class BaseDrive {
protected:
  BaseDrive() = default;

  // TODO: Modifyable for compatability with LimeLight.

  /**
   * Constructs a base drive class capable of automatically listening for
   * vision based odometry over the network via NetworkTables.
   *
   * @param visionTable Path to NetworkTables table for listening for vision
   *                    based odometry updates. This table whould include two
   *                    DoubleArrayTopics. One titled `pose` with three entries
   *                    ordered X, Y, Theta in meters and radians. The other
   *                    should be names `stDev` with three entris again ordered
   *                    X, Y, Theta with units meters and radians. Always
   *                    update stDev before pose.
   */
  BaseDrive(std::string visionTable);

  virtual ~BaseDrive();

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
  virtual void addVisionMeasurments(const frc::Pose2d &poseEstimate,
                                    units::second_t time) = 0;

  /**
   * Change accuratly vision data is expected to be.
   *
   * @param standardDevs The standar deviation of vision measurments. This is
   *                     by how much teh actual robot positioon varies from
   *                     the actual posiotion of the robot. A larger value
   *                     means less acurate data. These are in units of meters
   *                     and radians ordered X, Y, Theta.
   */
  virtual void setVisionSTDevs(wpi::array<double, 3> standardDevs) = 0;

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
   * Returns wherther the drive train is holonomic, meanign can move in all
   * directions. This is nessesry for determining how to rezero a robot at the
   * beginning of a path.
   */
  virtual bool isHolonomic() const = 0;

  /**
   * Generates a command to follow WPILib Trajectory.
   *
   * @param trajectory       The trajectory to follow.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory.
   */
  virtual frc2::CommandPtr followWPILibTrajectory(
      frc::Trajectory trajectory,
      std::initializer_list<frc2::Subsystem *> driveRequirements) = 0;

  /**
   * Generates a command to follow a vector of WPILib Trajectories. This is
   * helpful if the robot should stop in the middle of a path or changes
   * direction.
   *
   * @param trajectorygroup  The vector of trajectories to follow.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a list of trajectories.
   */
  virtual frc2::CommandPtr followWPILibTrajectoryGroup(
      std::vector<frc::Trajectory> trajectoryGroup,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  /**
   * Generates a command to follow PathPlanner Trajectory.
   *
   * @param trajectory       The trajectory to follow.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory.
   */
  virtual frc2::CommandPtr
  followPPPath(std::shared_ptr<pathplanner::PathPlannerPath> path,
               std::initializer_list<frc2::Subsystem *> driveRequirements) = 0;

  /**
   * Generates a command to follow a vector of PathPlanner Trajectories. This
   * is helpful if the robot should stop in the middle of a path or changes
   * direction.
   *
   * @param trajectorygroup  The vector of trajectories to follow.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a list of trajectories.
   */
  //   virtual frc2::CommandPtr followPPTrajectoryGroup(
  //       std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
  //       std::initializer_list<frc2::Subsystem *> driveRequirements);

  /**
   * Generates a command to follow a PathPlanner Trajectories with events.
   * When each event is triggered, the robot will execute the corosponding
   * commmand in the event map while continuing to follow the path.
   *
   * @param trajectory       The trajectory to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectory.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a trajectory with events.
   */
  virtual frc2::CommandPtr followPPPathWithEvents(
      pathplanner::PathPlannerPath path,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  /**
   * Generates a command to follow a vector of PathPlanner Trajectories with
   * events. When each event is triggered, the robot will execute the
   * corosponding commmand in the event map while continuing to follow the
   * path.
   *
   * @param trajectoryGroup  The vector of trajectories to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectories.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a list of trajectories with events.
   */
  //   virtual frc2::CommandPtr followPPTrajectoryGroupWithEvents(
  //       std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
  //       std::unordered_map<std::string, std::shared_ptr<frc2::Command>>
  //       eventMap, std::initializer_list<frc2::Subsystem *>
  //       driveRequirements);

  /**
   * Generates a command to complete a full autonomouse routine generated by
   * PathPlanner. Beyond just a path with events, this includes resetting the
   * position at the start of the path and executing stop events.
   *
   * @param trajectory       The trajectory to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectories.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a full autonomus routine.
   */
  virtual frc2::CommandPtr fullPPAuto(
      pathplanner::PathPlannerPath path,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

  /**
   * Generates a command to complete a full autonomouse routine generated by
   * PathPlanner. Beyond just a path with events, this includes resetting the
   * position at the start of the path and executing stop events.
   *
   * @param trajectoryGroup  The vector of trajectories to follow.
   * @param evenMap          Used to map event names to thier corosponding
   *                         commands for execution during the trajectories.
   * @param driveRequirements The subsystems required for driving the robot
   *                         (ie. the one that contains this class)
   *
   * @return The command to follow a full autonomus routine.
   */
  virtual frc2::CommandPtr fullPPAuto(
      std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
      std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
      std::initializer_list<frc2::Subsystem *> driveRequirements);

protected:
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
};
} // namespace rmb
