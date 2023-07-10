#include "rmb/drive/SwerveDrive.h"

#include "Eigen/src/Core/util/ForwardDeclarations.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"

#include "frc/geometry/Translation2d.h"

#include "pathplanner/lib/PathPlannerTrajectory.h"
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
    units::meters_per_second_t maxXSpeed, units::meters_per_second_t maxYSpeed,
    units::radians_per_second_t maxRotation, const frc::Pose2d &initialPose) {}

template <size_t NumModules>
SwerveDrive<NumModules>::SwerveDrive(
    std::array<SwerveModule, NumModules> modules,
    std::shared_ptr<const frc::Gyro> gyro,
    frc::HolonomicDriveController holonomicController,
    units::meters_per_second_t maxXSpeed, units::meters_per_second_t maxYSpeed,
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

  if (fieldOriented) {
    speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xSpeed * maxXSpeed, ySpeed * maxYSpeed,
        units::radians_per_second_t(zRotation * maxRotation),
        gyro->GetRotation2d());
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

}

} // namespace rmb
