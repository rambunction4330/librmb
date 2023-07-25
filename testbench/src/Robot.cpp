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
          frc::Translation2d(1_m, 1_m)),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo1),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo1),
          frc::Translation2d(1_m, 1_m)),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo1),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo1),
          frc::Translation2d(1_m, 1_m)),
      rmb::SwerveModule(
          rmb::asLinear(std::make_unique<rmb::FalconVelocityController>(
                            constants::velocityControllerCreateInfo1),
                        constants::wheelCircumference / 1_tr),
          std::make_unique<rmb::FalconPositionController>(
              constants::positionControllerCreateInfo1),
          frc::Translation2d(1_m, 1_m)),

  };

  gyro = std::make_shared<AHRS>(constants::gyroPort);

  auto swerveDrive = rmb::SwerveDrive<4>(
      std::move(modules), gyro,
      frc::HolonomicDriveController(
          frc2::PIDController(1.0f, 0.0f, 0.0f),
          frc2::PIDController(1.0f, 0.0f, 0.0f),
          frc::ProfiledPIDController<units::radian>(
              1, 0, 0,
              frc::TrapezoidProfile<units::radian>::Constraints(
                  6.28_rad_per_s, 3.14_rad_per_s / 1_s))),
      1.0_mps, 1.0_tps);
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  positionController->zeroPosition();
}

void Robot::TestPeriodic() {
  velocityController->setVelocity(10_tps);
  // velocityController->setPower(0.5);

  // positionController->setPower(0.2);
  positionController->setPosition(0.25_tr);
  std::cout << "position: "
            << ((units::degree_t)positionController->getPosition())()
            << std::endl;
  std::cout << ((units::turns_per_second_t)velocityController->getVelocity())()
            << std::endl;
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
