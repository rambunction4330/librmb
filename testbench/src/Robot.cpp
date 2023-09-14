// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "frc/controller/HolonomicDriveController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "rmb/drive/SwerveDrive.h"
#include "rmb/drive/SwerveModule.h"
#include "rmb/motorcontrol/AngularVelocityController.h"
#include "rmb/motorcontrol/falcon/FalconPositionController.h"
#include "rmb/motorcontrol/falcon/FalconVelocityController.h"
#include "units/angle.h"

#include <alloca.h>
#include <frc2/command/CommandScheduler.h>
#include <limits>
#include <memory>

#include "Constants.h"
#include "wpi/raw_ostream.h"
#include <iostream>

void Robot::RobotInit() {

  // Because Aiden is evil & lazy
  std::array<rmb::SwerveModule, 4> modules = {
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo),
          frc::Translation2d(-1_ft, 1_ft)),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo1),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo1),
          frc::Translation2d(1_ft, 1_ft)),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo2),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo2),
          frc::Translation2d(1_ft, -1_ft)),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo3),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo3),
          frc::Translation2d(-1_ft, -1_ft)),

  };

  gyro = std::make_shared<AHRS>(constants::gyroPort);

  swerveDrive = std::make_unique<rmb::SwerveDrive<4>>(
      std::move(modules), gyro,
      frc::HolonomicDriveController(
          frc2::PIDController(1.0f, 0.0f, 0.0f),
          frc2::PIDController(1.0f, 0.0f, 0.0f),
          frc::ProfiledPIDController<units::radian>(
              1, 0, 0,
              frc::TrapezoidProfile<units::radian>::Constraints(
                  6.28_rad_per_s, 3.14_rad_per_s / 1_s))),
      5.0_mps, 1.0_tps);
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  swerveDrive->driveCartesian(-joystick.GetX(), joystick.GetY(),
                              joystick.GetTwist(), false);

  for (size_t i = 0; i < swerveDrive->getModules().size(); i++) {
    const auto &module = swerveDrive->getModules()[i];
    module.smartdashboardDisplayTargetState(std::to_string(i));
  }
}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
