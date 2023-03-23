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
    std::unique_ptr<LinearVelocityController> right,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::DifferentialDriveKinematics kinematics,
    frc::RamseteController ramseteController, std::string visionTable,
    const frc::Pose2d &initalPose)
    : BaseDrive(visionTable), left(std::move(left)), right(std::move(right)),
      gyro(gyro), kinematics(kinematics), ramseteController(ramseteController),
      poseEstimator(kinematics, gyro->GetRotation2d(), left->getPosition(),
                    right->getPosition(), initalPose) {}

DifferentialDrive::DifferentialDrive(
    std::unique_ptr<LinearVelocityController> left,
    std::unique_ptr<LinearVelocityController> right,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::DifferentialDriveKinematics kinematics,
    frc::RamseteController ramseteController, const frc::Pose2d &initalPose)
    : left(std::move(left)), right(std::move(right)), gyro(gyro),
      kinematics(kinematics), ramseteController(ramseteController),
      poseEstimator(kinematics, gyro->GetRotation2d(), left->getPosition(),
                    right->getPosition(), initalPose) {}

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
  return poseEstimator.Update(gyro->GetRotation2d(), left->getPosition(),
                              right->getPosition());
}

void DifferentialDrive::resetPose(const frc::Pose2d &pose) {
  // Lock thread for saftey and reset position
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.ResetPosition(gyro->GetRotation2d(), left->getPosition(),
                              right->getPosition(), pose);
}

void DifferentialDrive::addVisionMeasurments(const frc::Pose2d &poseEstimate,
                                             units::second_t time) {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.AddVisionMeasurement(poseEstimate, time);
}

void DifferentialDrive::setVisionSTDevs(wpi::array<double, 3> standardDevs) {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.SetVisionMeasurementStdDevs(standardDevs);
}

frc2::CommandPtr DifferentialDrive::followWPILibTrajectory(
    frc::Trajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  return frc2::RamseteCommand(
             trajectory, [this]() { return getPose(); }, ramseteController,
             kinematics, [this](auto l, auto r) { driveWheelSpeeds(l, r); },
             driveRequirements)
      .ToPtr();
}

frc2::CommandPtr DifferentialDrive::followPPTrajectory(
    pathplanner::PathPlannerTrajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  return pathplanner::PPRamseteCommand(
             trajectory, [this]() { return getPose(); }, ramseteController,
             kinematics, [this](auto l, auto r) { driveWheelSpeeds(l, r); },
             driveRequirements)
      .ToPtr();
}

} // namespace rmb
