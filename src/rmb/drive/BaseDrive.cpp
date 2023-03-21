#include "rmb/drive/BaseDrive.h"

#include <frc2/command/Commands.h>

#include <pathplanner/lib/commands/FollowPathWithEvents.h>

namespace rmb {
frc2::CommandPtr BaseDrive::followWPILibTrajectoryGroup(
    std::vector<frc::Trajectory> trajectoryGroup,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {
  
  std::vector<frc2::CommandPtr> followCommands;

  for (auto trajectory : trajectoryGroup) {
    followCommands.emplace_back(
        followWPILibTrajectory(trajectory, driveRequirments));
  }

  return frc2::cmd::Sequence(std::move(followCommands));
}

frc2::CommandPtr BaseDrive::followPPTrajectoryGroup(
    std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  std::vector<frc2::CommandPtr> followCommands;

  for (auto trajectory : trajectoryGroup) {
    followCommands.emplace_back(
        followPPTrajectory(trajectory, driveRequirments));
  }

  return frc2::cmd::Sequence(std::move(followCommands));
}

frc2::CommandPtr BaseDrive::followPPTrajectoryWithEvents(
    pathplanner::PathPlannerTrajectory trajectory,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  return pathplanner::FollowPathWithEvents(
              followPPTrajectory(trajectory, driveRequirments).Unwrap(),
              trajectory.getMarkers(), eventMap)
      .ToPtr();
}

frc2::CommandPtr BaseDrive::followPPTrajectoryGroupWithEvents(
    std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  std::vector<frc2::CommandPtr> followCommands;

  for (auto trajectory : trajectoryGroup) {
    followCommands.emplace_back(
        followPPTrajectoryWithEvents(trajectory, eventMap, driveRequirments));
  }

  return frc2::cmd::Sequence(std::move(followCommands));
}

frc2::CommandPtr BaseDrive::fullPPAuto(
    pathplanner::PathPlannerTrajectory trajectory,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup;
  trajectoryGroup.push_back(trajectory);
  return fullPPAuto(trajectoryGroup, eventMap, driveRequirments);
}
}