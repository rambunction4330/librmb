#include "rmb/drive/DifferentialDrive.h"

#include <initializer_list>
#include <memory>
#include <unordered_map>

#include <frc/drive/DifferentialDrive.h>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/Subsystem.h>

#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/RamseteAutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <pathplanner/lib/commands/PPRamseteCommand.h>

namespace rmb {

DifferentialDrive::DifferentialDrive(
    std::unique_ptr<LinearVelocityController> left,
    std::unique_ptr<LinearVelocityController> right, const frc::Gyro &gyro,
    frc::DifferentialDriveKinematics kinematics,
    frc::RamseteController ramseteController, std::string visionTable,
    const frc::Pose2d &initalPose)
    : left(std::move(left)), right(std::move(right)), gyro(gyro),
      kinematics(kinematics), ramseteController(ramseteController),
      poseEstimator(kinematics, gyro.GetRotation2d(), left->getPosition(),
                    right->getPosition(), initalPose) {

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

        // Lock mutex for thread saftey and add vision.
        std::lock_guard<std::mutex> lock(visionThreadMutex);
        poseEstimator.AddVisionMeasurement(pose, time);
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

                         // Lock mutex for thread saftey and add vision.
                         std::lock_guard<std::mutex> lock(visionThreadMutex);
                         poseEstimator.SetVisionMeasurementStdDevs(
                             {rawData[0], rawData[1], rawData[2]});
                       });
}

DifferentialDrive::DifferentialDrive(
    std::unique_ptr<LinearVelocityController> left,
    std::unique_ptr<LinearVelocityController> right, const frc::Gyro &gyro,
    frc::DifferentialDriveKinematics kinematics,
    frc::RamseteController ramseteController, const frc::Pose2d &initalPose)
    : left(std::move(left)), right(std::move(right)), gyro(gyro),
      kinematics(kinematics), ramseteController(ramseteController),
      poseEstimator(kinematics, gyro.GetRotation2d(), left->getPosition(),
                    right->getPosition(), initalPose) {}

DifferentialDrive::~DifferentialDrive() {
  // Remove listeners.
  nt::RemoveListener(poseListener);
  nt::RemoveListener(stdDevListener);
}

void DifferentialDrive::arcadeDrive(double xSpeed, double zRotation) {
  auto wheelSpeeds =
      frc::DifferentialDrive::ArcadeDriveIK(xSpeed, zRotation, false);
  left->setPower(wheelSpeeds.left);
  right->setPower(wheelSpeeds.right);
}

void DifferentialDrive::curvatureDrive(double xSpeed, double zRotation,
                                       bool turnInPlace) {
  auto wheelSpeeds =
      frc::DifferentialDrive::CurvatureDriveIK(xSpeed, zRotation, turnInPlace);
  left->setPower(wheelSpeeds.left);
  right->setPower(wheelSpeeds.right);
}

void DifferentialDrive::tankDrive(double leftSpeed, double rightSpeed) {
  auto wheelSpeeds = frc::DifferentialDrive::TankDriveIK(leftSpeed, rightSpeed);
  left->setPower(wheelSpeeds.left);
  right->setPower(wheelSpeeds.right);
}

void DifferentialDrive::driveWheelSpeeds(
    units::meters_per_second_t leftVelocity,
    units::meters_per_second_t rightVelocity) {
  left->setVelocity(leftVelocity);
  right->setVelocity(rightVelocity);
}

void DifferentialDrive::driveWheelSpeeds(
    frc::DifferentialDriveWheelSpeeds wheelSpeeds) {
  left->setVelocity(wheelSpeeds.left);
  right->setVelocity(wheelSpeeds.right);
}

frc::DifferentialDriveWheelSpeeds DifferentialDrive::getWheelSpeeds() const {
  return {left->getVelocity(), right->getVelocity()};
}

void DifferentialDrive::driveChassisSpeeds(frc::ChassisSpeeds chassisSpeeds) {
  driveWheelSpeeds(kinematics.ToWheelSpeeds(chassisSpeeds));
}

frc::ChassisSpeeds DifferentialDrive::getChassisSpeeds() const {
  return kinematics.ToChassisSpeeds(getWheelSpeeds());
}

frc::Pose2d DifferentialDrive::getPose() const {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.GetEstimatedPosition();
}

frc::Pose2d DifferentialDrive::updatePose() {
  // Lock thread for saftey and updte position
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.Update(gyro.GetRotation2d(), left->getPosition(),
                              right->getPosition());
}

void DifferentialDrive::resetPose(const frc::Pose2d &pose) {
  // Lock thread for saftey and reset position
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.ResetPosition(gyro.GetRotation2d(), left->getPosition(),
                              right->getPosition(), pose);
}

frc2::CommandPtr DifferentialDrive::followWPILibTrajectory(
    frc::Trajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  return frc2::RamseteCommand(
             trajectory, [this]() { return getPose(); }, ramseteController,
             kinematics, [this](auto l, auto r) { driveWheelSpeeds(l, r); },
             driveRequirments)
      .ToPtr();
}

frc2::CommandPtr DifferentialDrive::followPPTrajectory(
    pathplanner::PathPlannerTrajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  return pathplanner::PPRamseteCommand(
             trajectory, [this]() { return getPose(); }, ramseteController,
             kinematics, [this](auto l, auto r) { driveWheelSpeeds(l, r); },
             driveRequirments)
      .ToPtr();
}

frc2::CommandPtr DifferentialDrive::fullPPAuto(
    std::vector<pathplanner::PathPlannerTrajectory> trajectoryGroup,
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap,
    std::initializer_list<frc2::Subsystem *> driveRequirments) {

  if (trajectoryGroup.size() < 1) {
    return frc2::cmd::None();
  }

  // Dummy auto builder just used to generate stop commands.
  pathplanner::RamseteAutoBuilder autoBuilder(
      []() { return frc::Pose2d(); }, [](auto) {}, ramseteController,
      kinematics, [](auto, auto) {}, eventMap, {});

  std::vector<frc2::CommandPtr> commands;

  commands.emplace_back(frc2::InstantCommand([this, trajectoryGroup]() {
    resetPose(trajectoryGroup.front().getInitialPose());
  }));

  for (auto trajectory : trajectoryGroup) {
    commands.emplace_back(
        autoBuilder.stopEventGroup(trajectory.getStartStopEvent()));
    commands.emplace_back(
        followPPTrajectoryWithEvents(trajectory, eventMap, driveRequirments));
  }

  commands.emplace_back(
      autoBuilder.stopEventGroup(trajectoryGroup.back().getEndStopEvent()));

  return frc2::cmd::Sequence(std::move(commands));
}

} // namespace rmb