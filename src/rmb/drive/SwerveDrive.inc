
#pragma once

#include "rmb/drive/BaseDrive.h"
#include "rmb/drive/SwerveDrive.h"

#include <units/angle.h>
#include <units/base.h>
#include <units/length.h>
#include <units/math.h>

#include <array>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc2/command/SwerveControllerCommand.h>

#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>nsl

#include "rmb/drive/SwerveModule.h"

namespace rmb {

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::HolonomicDriveController holonomicController, std::string visionTable,
    const frc::Pose2d &initalPose)
    : BaseDrive(visionTable),
      SwerveDrive(modules, gyro, holonomicController, initalPose) {}

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::HolonomicDriveController holonomicController,
    const frc::Pose2d &initalPose)
    : modules(modules), gyro(gyro), holonomicController(holonomicController) {

  std::array<frc::Translation2d, NumModules> translations;
  for (int i = 0; i < NumModules; i++) {
    translations[i] = modules[i].getModuleTranslation();
  }

  kinematics = frc::SwerveDriveKinematics<NumModules>(translations);
  poseEstimator = frc::SwerveDrivePoseEstimator<NumModules>(
      kinematics, gyro->GetRotation2d(), getModulePositions(), initalPose);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveCatesian(double xSpeed, double ySpeed,
                                            double zRotation,
                                            bool fieldOriented) {

  // Correct x and y speeds for field oriented driving.
  double correctedXSpeed = xSpeed;
  double correctedYSpeed = ySpeed;

  if (fieldOriented) {
    double magnitude = std::sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
    frc::Rotation2d angle(xSpeed, ySpeed);

    correctedXSpeed = magnitude * (angle - getPose().Rotation()).Cos();
    correctedXSpeed = magnitude * (angle - getPose().Rotation()).Cos();
  }

  // Find out how far the furthest module is from the center for normalizing
  // rotation. This way at max rotation the furthest module will run at max
  // power.
  units::meter_t maxModuleRadius = 0_m;
  for (SwerveModule module : modules) {
    maxModuleRadius =
        units::math::max(module.getModuleTranslation().Norm(), maxModuleRadius);
  }

  // Calulate the power to apply to each module
  std::array<SwerveModulePower, NumModules> powers;
  for (int i = 0; i < modules.size(); i++) {
    // Get module position in polar coordinates.
    units::meter_t moduleRadius = modules[i].getModuleTranslation().Norm();
    frc::Rotation2d moduleAngle = modules[i].getModuleTranslation().Angle();

    // Get vector components for the power.
    double normalizedRadius = moduleRadius / maxModuleRadius;
    double xComp =
        correctedXSpeed + (normalizedRadius * zRotation * moduleAngle.Sin());
    double yComp =
        correctedYSpeed + (normalizedRadius * zRotation * moduleAngle.Cos());

    // Convert from components to polar coordinates.
    powers[i] = {std::sqrt(xComp * xComp + yComp * yComp),
                 frc::Rotation2d(xComp, yComp)};
  }

  // Determine the maximum requested power for normalizing
  double maxPower = 1.0;
  for (SwerveModulePower power : powers) {
    maxPower = std::max(std::abs(maxPower), power.power);
  }

  // Normalize moduel power so no moduel is over 1.0
  for (SwerveModulePower power : powers) {
    power.power /= maxPower;
  }

  // Drive at proposed powers.
  driveModulePower(powers);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::drivePolar(double speed,
                                         const frc::Rotation2d &angle,
                                         double zRotation, bool fieldOriented) {

  // Handle field oriented driving before passing to drive cartesian to avoid
  // redundant processing.
  frc::Rotation2d driveAngle = angle;
  if (fieldOriented) {
    driveAngle -= getPose().Rotation();
  }

  // Decompose into components.
  double xSpeed = speed * driveAngle.Cos();
  double ySpeed = speed * driveAngle.Sin();

  // drive cartesian with components.
  driveCatesian(xSpeed, ySpeed, zRotation, false);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModulePower(
    std::array<SwerveModulePower, NumModules> powers) {
  for (int i = 0; i < NumModules; i++) {
    modules[i].setPower(powers[i]);
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModuleStates(
    std::array<frc::SwerveModuleState, NumModules> states) {
  for (int i = 0; i < NumModules; i++) {
    modules[i].setState(states[i]);
  }
}

template <size_t NumModules>
std::array<frc::SwerveModuleState, NumModules>
SwerveDrive<NumModules>::getModuleStates() const {
  std::array<frc::SwerveModuleState, NumModules> states;
  for (int i = 0; i < NumModules; i++) {
    states[i] = modules[i].getState();
  }
}

template <size_t NumModules>
std::array<frc::SwerveModulePosition, NumModules>
SwerveDrive<NumModules>::getModulePositions() const {
  std::array<frc::SwerveModuleState, NumModules> states;
  for (int i = 0; i < NumModules; i++) {
    states[i] = modules[i].getPosition();
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveChassisSpeeds(
    frc::ChassisSpeeds chassisSpeeds) {
  driveModuleStates(kinematics.ToSwerveModuleStates(chassisSpeeds));
}

template <size_t NumModules>
frc::ChassisSpeeds SwerveDrive<NumModules>::getChassisSpeeds() const {
  return kinematics.ToChassisSpeeds(getModuleStates());
}

template <size_t NumModules>
frc::Pose2d SwerveDrive<NumModules>::getPose() const {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.GetEstimatedPosition();
}

template <size_t NumModules> frc::Pose2d SwerveDrive<NumModules>::updatePose() {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.Update(gyro->GetRotation2d(), updatePose());
}

template <size_t NumModules>
void SwerveDrive<NumModules>::resetPose(const frc::Pose2d &pose) {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.ResetPosition(gyro->GetRotation2d(), getModulePositions(),
                              pose);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::addVisionMeasurments(
    const frc::Pose2d &poseEstimate, units::second_t time) {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.AddVisionMeasurement(poseEstimate, time);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::setVisionSTDevs(
    wpi::array<double, 3> standardDevs) {
  // Lock thread for saftey and get positon.
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.SetVisionMeasurementStdDevs(standardDevs);
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::followWPILibTrajectory(
    frc::Trajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  return frc2::SwerveControllerCommand<NumModules>(
             trajectory, [this]() { return getPose(); }, kinematics,
             holonomicController, [this](auto states) { States(states); },
             driveRequirements)
      .ToPtr();
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::followPPTrajectory(
    pathplanner::PathPlannerTrajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {
  return pathplanner::PPSwerveControllerCommand(
             trajectory, [this]() { return getPose(); },
             holonomicController.getXController(),
             holonomicController.getYController(),
             holonomicController.getThetaController(),
             [this](auto chassisSpeeds) { driveChassisSpeeds(chassisSpeeds); },
             driveRequirements)
      .ToPtr();
}

} // namespace rmb
