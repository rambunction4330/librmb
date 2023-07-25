#include "rmb/drive/SwerveDrive.h"

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"

#include "frc/geometry/Translation2d.h"

#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"
#include "units/velocity.h"

namespace rmb {

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::HolonomicDriveController holonomicController, std::string visionTable,
    units::meters_per_second_t maxSpeed,
    units::radians_per_second_t maxRotation, const frc::Pose2d &initialPose) {}

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::HolonomicDriveController holonomicController,
    units::meters_per_second_t maxSpeed,
    units::radians_per_second_t maxRotation, const frc::Pose2d &initialPose) {

  std::array<frc::Translation2d, NumModules> translations;

  kinematics = frc::SwerveDriveKinematics<NumModules>(translations);
  poseEstimator = frc::SwerveDrivePoseEstimator<NumModules>(
      kinematics, gyro->GetRotation2d(), getModulePositions(), initialPose);
}

template <size_t NumModules>
std::array<frc::SwerveModulePosition, NumModules>
SwerveDrive<NumModules>::getModulePositions() const {
  std::array<frc::SwerveModulePosition, NumModules> states;
  for (int i = 0; i < NumModules; i++) {
    states[i] = modules[i].getPosition();
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
void SwerveDrive<NumModules>::driveCatesian(double xSpeed, double ySpeed,
                                            double zRotation,
                                            bool fieldOriented) {
  frc::ChassisSpeeds speeds;

  double newXSpeed = xSpeed;
  double newYSpeed = ySpeed;
  double magnitude = std::sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

  if (magnitude > 1) {
    newXSpeed = xSpeed / magnitude;
    newYSpeed = ySpeed / magnitude;
  }

  if (fieldOriented) {
    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        newXSpeed * maxSpeed, newYSpeed * maxSpeed,
        units::radians_per_second_t(zRotation * maxRotation),
        gyro->GetRotation2d());
  } else {
    speeds = frc::ChassisSpeeds{newXSpeed * maxSpeed, ySpeed * maxSpeed,
                                zRotation * maxRotation};
  }
  driveModuleStates(kinematics.ToSwerveModuleStates(speeds));
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModuleStates(
    std::array<frc::SwerveModuleState, NumModules> states) {
  for (int i = 0; i < NumModules; i++) {
    modules[i].setState(states[i]);
  }
}

template <size_t NumModules>
void SwerveDrive<NumModules>::drivePolar(double speed,
                                         const frc::Rotation2d &angle,
                                         double zRotation, bool fieldOriented) {

  double vx = speed * angle.Cos();
  double vy = speed * angle.Sin();

  driveCatesian(vx, vy, zRotation, fieldOriented);
}

template <size_t NumModules>
void SwerveDrive<NumModules>::driveModulePower(
    std::array<SwerveModulePower, NumModules> powers) {
  for (int i = 0; i < NumModules; i++) {
    modules[i].setPower(powers[i]);
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
  return poseEstimator.GetEstimatedPosition();
}

template <size_t NumModules> frc::Pose2d SwerveDrive<NumModules>::updatePose() {
  poseEstimator.Update(gyro->GetRotation2d(), getModulePositions());
}

template <size_t NumModules>
void SwerveDrive<NumModules>::resetPose(const frc::Pose2d &pose) {
  poseEstimator.ResetPosition(gyro->GetRotation2d(), getModulePositions(),
                              pose);
}

template <size_t NumModules>
void addVisionMeasurments(const frc::Pose2d &poseEstimate,
                          units::second_t time) {}

template <size_t NumModules>
void setVisionSTDevs(wpi::array<double, 3> standardDevs) {}

template <size_t NumModules> bool isHolonomic() {}

template <size_t NumModules>
frc2::CommandPtr
followPPTrajectory(pathplanner::PathPlannerTrajectory trajectory,
                   std::initializer_list<frc2::Subsystem *> driveRequirements) {
}

} // namespace rmb
  // namespace rmb
