#pragma once

#include "frc/geometry/Pose2d.h"
#include "pathplanner/lib/path/PathConstraints.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/util/HolonomicPathFollowerConfig.h"
#include "pathplanner/lib/util/PIDConstants.h"
#include "pathplanner/lib/util/ReplanningConfig.h"
#include "rmb/drive/BaseDrive.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/length.h"
#include "units/math.h"
#include "units/time.h"
#include "units/velocity.h"

#include "frc/controller/HolonomicDriveController.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/interfaces/Gyro.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"

#include "frc/geometry/Translation2d.h"

#include "frc2/command//SwerveControllerCommand.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/Subsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "rmb/drive/SwerveDrive.h"
#include "rmb/drive/SwerveModule.h"
#include "rmb/motorcontrol/feedforward/Feedforward.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"
#include "wpi/array.h"
#include "wpi/sendable/SendableRegistry.h"

#include <array>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iostream>

#include <Eigen/Core>
#include <memory>

#include "pathplanner/lib/commands/FollowPathHolonomic.h"
#include "pathplanner/lib/commands/PathfindHolonomic.h"
#include "pathplanner/lib/path/PathConstraints.h"
#include "pathplanner/lib/util/HolonomicPathFollowerConfig.h"
#include "pathplanner/lib/util/ReplanningConfig.h"
#include <mutex>

namespace rmb {

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const rmb::Gyro> gyro,
    frc::HolonomicDriveController holonomicController, std::string visionTable,
    units::meters_per_second_t maxModuleSpeed, const frc::Pose2d &initialPose)
    : modules(std::move(modules)), gyro(gyro),
      kinematics(std::array<frc::Translation2d, NumModules>{}),
      holonomicController(holonomicController),
      poseEstimator(frc::SwerveDrivePoseEstimator<NumModules>(
          kinematics, gyro->getRotation(), getModulePositions(), initialPose)),
      maxModuleSpeed(maxModuleSpeed) {
  std::array<frc::Translation2d, NumModules> translations;
  for (size_t i = 0; i < NumModules; i++) {
    translations[i] = modules[i].getModuleTranslation();
  }
  nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("swervedrive");

  ntPositionTopics = table->GetDoubleArrayTopic("mod_positions").Publish();
  ntVelocityTopics = table->GetDoubleArrayTopic("mod_velocities").Publish();

  ntPositionErrorTopics =
      table->GetDoubleArrayTopic("mod_position_errors").Publish();
  ntVelocityErrorTopics =
      table->GetDoubleArrayTopic("mod_velocity_errors").Publish();

  ntTargetPositionTopics =
      table->GetDoubleArrayTopic("mod_position_targets").Publish();
  ntTargetVelocityTopics =
      table->GetDoubleArrayTopic("mod_velocity_targets").Publish();

  units::meter_t maxDistance = 0.0_m;
  for (SwerveModule &module : this->modules) {
    auto &translation = module.getModuleTranslation();
    units::meter_t distance =
        translation.Distance(frc::Translation2d(0.0_m, 0.0_m));

    maxDistance = units::math::max(distance, maxDistance);
  }

  largestModuleDistance = maxDistance;
}

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const rmb::Gyro> gyro,
    frc::HolonomicDriveController holonomicController,
    units::meters_per_second_t maxModuleSpeed, const frc::Pose2d &initialPose)
    : SwerveDrive(std::move(modules), gyro, holonomicController, "",
                  maxModuleSpeed, initialPose) {}

template <size_t NumModules>
std::array<frc::SwerveModulePosition, NumModules>
SwerveDrive<NumModules>::getModulePositions() const {
  std::array<frc::SwerveModulePosition, NumModules> states;
  for (size_t i = 0; i < NumModules; i++) {
    states[i] = modules[i].getPosition();
  }

  return states;
}

template <size_t NumModules>
std::array<frc::SwerveModuleState, NumModules>
SwerveDrive<NumModules>::getModuleStates() const {
  std::array<frc::SwerveModuleState, NumModules> states;
  for (size_t i = 0; i < NumModules; i++) {
    states[i] = modules[i].getState();
  }

  return states;
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveCartesian(double xSpeed, double ySpeed,
                                             double zRotation,
                                             bool fieldOriented) {

  Eigen::Vector2d robotRelativeVXY = Eigen::Vector2d(xSpeed, ySpeed);

  if (fieldOriented) {
    units::radian_t vXYRotationAngle = -gyro->getRotation().Radians();

    Eigen::Matrix2d vXYRotation{
        {std::cos(vXYRotationAngle()), -std::sin(vXYRotationAngle())},
        {std::sin(vXYRotationAngle()), std::cos(vXYRotationAngle())}};

    robotRelativeVXY = vXYRotation * robotRelativeVXY;
  }

  /**
   * https://dominik.win/blog/programming-swerve-drive/
   *
   * output = \vec{v} + w * perpendicular{m}
   * where v is the net translational velocity and m is the module's translation
   * from the center
   *
   * And we define our perpendicular functions as
   * perpendicular(x, y) => (y, -x)
   *
   * so,
   * output_x = vx * 1 + vy * 0 + w * y
   * output_y = vx * 0 + vy * 1 + w * -x
   */

  std::array<SwerveModulePower, NumModules> powers;
  double largestPower = 1.0;

  for (size_t i = 0; i < modules.size(); i++) {
    SwerveModule &module = modules[i];

    double output_x =
        robotRelativeVXY.x() +
        zRotation * module.getModuleTranslation().Y() / largestModuleDistance;

    double output_y =
        robotRelativeVXY.y() +
        zRotation * -module.getModuleTranslation().X() / largestModuleDistance;

    frc::Rotation2d moduleRotation{output_x, output_y};
    double modulePower = std::sqrt(output_x * output_x + output_y * output_y);

    powers[i] = SwerveModulePower{modulePower, moduleRotation};

    if (modulePower > largestPower) {
      largestPower = modulePower;
    }
  }

  // Normalize
  for (SwerveModulePower &power : powers) {
    power.power /= largestPower;
  }

  // Optimize
  for (size_t i = 0; i < modules.size(); i++) {
    powers[i] =
        SwerveModulePower::Optimize(powers[i], modules[i].getState().angle);
  }

  driveModulePowers(powers);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModuleStates(
    std::array<frc::SwerveModuleState, NumModules> states) {
  for (size_t i = 0; i < NumModules; i++) {
    modules[i].setState(states[i]);
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::drivePolar(double speed,
                                         const frc::Rotation2d &angle,
                                         double zRotation, bool fieldOriented) {
  double vx = speed * angle.Cos();
  double vy = speed * angle.Sin();

  driveCartesian(vx, vy, zRotation, fieldOriented);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModulePowers(
    std::array<SwerveModulePower, NumModules> powers) {
  // std::cout << "powers: ";
  for (size_t i = 0; i < NumModules; i++) {
    modules[i].setPower(powers[i]);
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveChassisSpeeds(
    frc::ChassisSpeeds chassisSpeeds) {
  auto states = kinematics.ToSwerveModuleStates(chassisSpeeds);
  kinematics.DesaturateWheelSpeeds(&states, maxModuleSpeed);
  driveModuleStates(states);
}

template <size_t NumModules>
frc::ChassisSpeeds SwerveDrive<NumModules>::getChassisSpeeds() const {
  return kinematics.ToChassisSpeeds(
      wpi::array<frc::SwerveModuleState, NumModules>(getModuleStates()));
}

template <size_t NumModules>
frc::Pose2d SwerveDrive<NumModules>::getPose() const {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.GetEstimatedPosition();
}

template <size_t NumModules> frc::Pose2d SwerveDrive<NumModules>::updatePose() {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  return poseEstimator.Update(
      frc::Rotation2d((units::radian_t)gyro->getZRotation()),
      getModulePositions());
}

template <size_t NumModules>
void SwerveDrive<NumModules>::updateNTDebugInfo(bool openLoopVelocity) {
  std::array<double, NumModules> velocityErrors;
  for (size_t i = 0; i < NumModules; i++) {
    units::meters_per_second_t error = 0.0_mps;
    if (!openLoopVelocity) {
      error = modules[i].getTargetState().speed - modules[i].getState().speed;
    } else {

      velocityErrors[i] = error();
    }
    ntVelocityErrorTopics.Set(velocityErrors);

    std::array<double, NumModules> positionErrors;
    for (size_t i = 0; i < NumModules; i++) {
      units::turn_t error = modules[i].getTargetRotation().Degrees() -
                            modules[i].getState().angle.Degrees();

      positionErrors[i] = error();
    }
    ntPositionErrorTopics.Set(positionErrors);

    std::array<double, NumModules> velocityTargets;
    for (size_t i = 0; i < NumModules; i++) {
      double target = 0.0;

      if (!openLoopVelocity) {
        target = modules[i].getTargetVelocity()();
      } else {
        target = modules[i].getPower().power;
      }

      velocityTargets[i] = target;
    }
    ntTargetVelocityTopics.Set(velocityTargets);

    std::array<double, NumModules> positionTargets;
    for (size_t i = 0; i < NumModules; i++) {
      units::degree_t target = modules[i].getTargetRotation().Degrees();

      positionTargets[i] = target();
    }
    ntTargetPositionTopics.Set(positionTargets);

    std::array<double, NumModules> positions;
    for (size_t i = 0; i < NumModules; i++) {
      units::turn_t position = modules[i].getState().angle.Degrees();

      positions[i] = position();
    }
    ntPositionTopics.Set(positions);

    std::array<double, NumModules> velocities;
    for (size_t i = 0; i < NumModules; i++) {
      units::meters_per_second_t velocity = modules[i].getState().speed;

      velocities[i] = velocity();
    }
    ntVelocityTopics.Set(velocities);
  }
}

template <size_t NumModules>
std::array<frc::SwerveModuleState, NumModules>
SwerveDrive<NumModules>::getTargetModuleStates() const {
  std::array<frc::SwerveModuleState, NumModules> targetStates;
  for (size_t i = 0; i < NumModules; i++) {
    targetStates[i] = modules[i].getTargetState();
  }

  return targetStates;
}

template <size_t NumModules>
void SwerveDrive<NumModules>::resetPose(const frc::Pose2d &pose) {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.ResetPosition(gyro->getRotation(), getModulePositions(), pose);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::addVisionMeasurments(
    const frc::Pose2d &poseEstimate, units::second_t time) {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.AddVisionMeasurement(poseEstimate, time);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::setVisionSTDevs(
    wpi::array<double, 3> standardDevs) {
  std::lock_guard<std::mutex> lock(visionThreadMutex);
  poseEstimator.SetVisionMeasurementStdDevs(standardDevs);
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::followWPILibTrajectory(
    frc::Trajectory trajectory,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  return frc2::SwerveControllerCommand<NumModules>(
             trajectory, [this]() { return getPose(); }, kinematics,
             holonomicController,
             [this](std::array<frc::SwerveModuleState, NumModules> states) {
               driveModuleStates(states);
             },
             driveRequirements)
      .ToPtr();
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::followPPPath(
    std::shared_ptr<pathplanner::PathPlannerPath> path,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  pathplanner::ReplanningConfig replanningConfig;
  units::second_t period;
  pathplanner::HolonomicPathFollowerConfig holonomicPathFollowerConfig(
      maxModuleSpeed, largestModuleDistance, replanningConfig, period);

  return pathplanner::FollowPathHolonomic(
             path, [this]() { return getPose(); },
             [this]() { return getChassisSpeeds(); },
             [this](frc::ChassisSpeeds chassisSpeeds) {
               driveChassisSpeeds(chassisSpeeds);
             },
             holonomicPathFollowerConfig, []() { return true; },
             driveRequirements)
      .ToPtr();
}

template <size_t NumModules>
frc2::CommandPtr SwerveDrive<NumModules>::FollowGeneratedPPPath(
    frc::Pose2d targetPose, pathplanner::PathConstraints contraints,
    std::initializer_list<frc2::Subsystem *> driveRequirements) {

  pathplanner::ReplanningConfig replanningConfig;
  units::second_t period;
  pathplanner::HolonomicPathFollowerConfig holonomicPathFollowerConfig(
      maxModuleSpeed, largestModuleDistance, replanningConfig, period);
  units::meter_t rotationDelayDistance = 0_m;

  return pathplanner::PathfindHolonomic(
             targetPose, contraints, [this]() { return getPose(); },
             [this]() { return getChassisSpeeds(); },
             [this](frc::ChassisSpeeds chassisSpeeds) {
               driveChassisSpeeds(chassisSpeeds);
             },
             holonomicPathFollowerConfig, driveRequirements,
             rotationDelayDistance)
      .ToPtr();
}

template <size_t NumModules> void SwerveDrive<NumModules>::stop() {
  for (auto &module : modules) {
    module.stop();
  }
}

} // namespace rmb
