// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "frc/controller/HolonomicDriveController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "rmb/drive/SwerveDrive.h"
#include "rmb/drive/SwerveModule.h"
#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/Talon/TalonFXPositionController.h"
#include "rmb/motorcontrol/Talon/TalonFXVelocityController.h"
#include "units/angle.h"

#include <frc2/command/CommandScheduler.h>
#include <memory>

#include "Constants.h"
#include "units/time.h"

void Robot::RobotInit() {
  // Because Aiden is evil & lazy
  std::array<rmb::SwerveModule, 4> modules = {
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::velocityControllerCreateInfo),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::positionControllerCreateInfo),
          frc::Translation2d(-1_ft, 1_ft), true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::velocityControllerCreateInfo1),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::positionControllerCreateInfo1),
          frc::Translation2d(1_ft, 1_ft), true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::velocityControllerCreateInfo2),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::positionControllerCreateInfo2),
          frc::Translation2d(1_ft, -1_ft), true),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::TalonFXVelocityController>(
                            constants::velocityControllerCreateInfo3),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::TalonFXPositionController>(
              constants::positionControllerCreateInfo3),
          frc::Translation2d(-1_ft, -1_ft), true),

  };

  swerveDrive = std::make_unique<rmb::SwerveDrive<4>>(
      std::move(modules), gyro,
      frc::HolonomicDriveController(
          frc::PIDController(1.0f, 0.0f, 0.0f),
          frc::PIDController(1.0f, 0.0f, 0.0f),
          frc::ProfiledPIDController<units::radian>(
              1, 0, 0,
              frc::TrapezoidProfile<units::radian>::Constraints(
                  6.28_rad_per_s, 3.14_rad_per_s / 1_s))),
      7.0_mps);

  frc::SmartDashboard::PutNumber("joyX", 0.0);
  frc::SmartDashboard::PutNumber("joyY", 0.0);
  frc::SmartDashboard::PutNumber("joyTwist", 0.0);
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() { gyro->resetZRotation(); }

inline static double ensureMagnitudeMax(double val, double mag) {
  return wpi::sgn(val) * std::clamp(std::abs(val), 0.0, mag);
}

void Robot::TeleopPeriodic() {
  // units::millisecond_t startTime = frc::Timer::GetFPGATimestamp();
  // swerveDrive->driveCartesian(joystick.GetX() * (joystick.GetThrottle()),
  //                             -joystick.GetY() * joystick.GetThrottle(),
  //                             -joystick.GetTwist() * joystick.GetThrottle(),
  //                             false);

  // std::cout << ((units::millisecond_t)frc::Timer::GetFPGATimestamp() -
  //               startTime)()
  //           << std::endl;
  const double maxOpenloop = 0.15;

  swerveDrive->driveCartesian(
      ensureMagnitudeMax(gamepad.GetLeftX(), maxOpenloop),
      -ensureMagnitudeMax(gamepad.GetLeftY(), maxOpenloop),
      -ensureMagnitudeMax(gamepad.GetRightY(), maxOpenloop), true);

  frc::SmartDashboard::PutNumber("angle", gyro->getRotation().Degrees()());

  swerveDrive->updateNTDebugInfo(true);
}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {
  // swerveDrive->driveCartesian(joystick.GetX() * (joystick.GetThrottle()),
  //                             -joystick.GetY() * joystick.GetThrottle(),
  //                             -joystick.GetTwist() * joystick.GetThrottle(),
  //                             false);
  // swerveDrive->driveCartesian(frc::SmartDashboard::GetNumber("joyX", 0.0),
  //                             -frc::SmartDashboard::GetNumber("joyY", 0.0),
  //                             frc::SmartDashboard::GetNumber("joyTwist",
  //                             0.0), false);
  //
  // swerveDrive->updateNTDebugInfo(true);
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
