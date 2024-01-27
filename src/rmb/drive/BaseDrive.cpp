#include "rmb/drive/BaseDrive.h"
#include "frc2/command/CommandPtr.h"

#include <typeinfo>

#include <frc2/command/Commands.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <pathplanner/lib/controllers/PPRamseteController.h>

namespace rmb {
BaseDrive::BaseDrive(std::string visionTable) {
  // Get vision table.
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable(visionTable);

  // Get positon and standard deviation topics.
  poseSubscriber = table->GetDoubleArrayTopic("pose").Subscribe({});
  stdDevSubscriber = table->GetDoubleArrayTopic("stdDev").Subscribe({});

  // Create positon callback
  poseListener = inst.AddListener(
      poseSubscriber, nt::EventFlags::kValueAll,
      [this](const nt::Event &event) {
        // Get timestamped data.
        nt::TimestampedDoubleArray rawData = poseSubscriber.GetAtomic();

        // Check data format.
        if (rawData.value.size() != 3) {
          return;
        }

        // Extract positon and time.
        frc::Pose2d pose = {units::meter_t(rawData.value[0]),
                            units::meter_t(rawData.value[1]),
                            units::radian_t(rawData.value[2])};

        units::second_t time = units::microsecond_t(rawData.time);

        addVisionMeasurments(pose, time);
      });

  // Create standard deviation callback
  stdDevListener =
      inst.AddListener(stdDevSubscriber, nt::EventFlags::kValueAll,
                       [this](const nt::Event &event) {
                         // Get data from table.
                         std::vector<double> rawData = stdDevSubscriber.Get();

                         // Check data format
                         if (rawData.size() != 3) {
                           return;
                         }

                         setVisionSTDevs({rawData[0], rawData[1], rawData[2]});
                       });
}

BaseDrive::~BaseDrive() {
  // Remove listeners.
  nt::RemoveListener(poseListener);
  nt::RemoveListener(stdDevListener);
}

frc2::CommandPtr BaseDrive::followWPILibTrajectoryGroup(
    std::vector<frc::Trajectory> trajectoryGroup,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  std::vector<frc2::CommandPtr> followCommands;

  for (auto trajectory : trajectoryGroup) {
    followCommands.emplace_back(
        followWPILibTrajectory(trajectory, driveRequirements));
  }

  return frc2::cmd::Sequence(std::move(followCommands));
}

// frc2::CommandPtr BaseDrive::followPPTrajectoryGroup(
//     std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
//     std::initializer_list<frc2::Subsystem *> driveRequirements) {

//   std::vector<frc2::CommandPtr> followCommands;

//   for (auto path : trajectoryGroup) {
//     followCommands.emplace_back(
//         followPPTrajectory(path, driveRequirements));
//   }

//   return frc2::cmd::Sequence(std::move(followCommands));
// }

frc2::CommandPtr BaseDrive::followPPPathWithEvents(
    pathplanner::PathPlannerPath path,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  // return pathplanner::FollowPathWithEvents(
  //            followPPTrajectory(trajectory, driveRequirments).Unwrap(),
  //            trajectory.getMarkers(), eventMap)
  //     .ToPtr();

  return frc2::cmd::None();
}

// frc2::CommandPtr BaseDrive::followPPTrajectoryGroupWithEvents(
//     std::vector<pathplanner::PathPlannerPath> trajectoryGroup,
//     std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
//     std::initializer_list<frc2::Subsystem *> driveRequirements) {

//   std::vector<frc2::CommandPtr> followCommands;

//   for (auto trajectory : trajectoryGroup) {
//     followCommands.emplace_back(
//         followPPTrajectoryWithEvents(trajectory, eventMap,
//         driveRequirements));
//   }

//   return frc2::cmd::Sequence(std::move(followCommands));
// }

frc2::CommandPtr BaseDrive::fullPPAuto(
    pathplanner::PathPlannerPath path,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  // std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup;
  // trajectoryGroup.push_back(path);
  // return fullPPAuto(trajectoryGroup, eventMap, driveRequirements);
  return frc2::cmd::None();
}

frc2::CommandPtr BaseDrive::fullPPAuto(
    std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  if (trajectoryGroup.size() < 1) {
    return frc2::cmd::None();
  }

  // Dummy auto builder just used to generate stop commands so the underlying
  // the actual type does not matter.
  // pathplanner::RamseteAutoBuilder autoBuilder(
  //     []() { return frc::Pose2d(); }, [](auto) {}, {},
  //     frc::DifferentialDriveKinematics(0.0_m), [](auto, auto) {}, eventMap,
  //     {});
  //
  // std::vector<frc2::CommandPtr> commands;
  //
  // if (isHolonomic()) {
  //   commands.emplace_back(frc2::InstantCommand([this, trajectoryGroup]() {
  //     resetPose(trajectoryGroup.front().getInitialHolonomicPose());
  //   }));
  // } else {
  //   commands.emplace_back(frc2::InstantCommand([this, trajectoryGroup]() {
  //     resetPose(trajectoryGroup.front().getInitialPose());
  //   }));
  // }
  //
  // for (auto trajectory : trajectoryGroup) {
  //   commands.emplace_back(
  //       autoBuilder.stopEventGroup(trajectory.getStartStopEvent()));
  //   commands.emplace_back(
  //       followPPTrajectoryWithEvents(trajectory, eventMap,
  //       driveRequirements));
  // }
  //
  // commands.emplace_back(
  //     autoBuilder.stopEventGroup(trajectoryGroup.back().getEndStopEvent()));
  //
  // return frc2::cmd::Sequence(std::move(commands));

  return frc2::cmd::None();
}

} // namespace rmb
