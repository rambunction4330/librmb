// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "rmb/motorcontrol/falcon/FalconPositionController.h"
#include "rmb/motorcontrol/falcon/FalconVelocityController.h"

#include <frc2/command/CommandScheduler.h>
#include <limits>
#include <memory>

#include <iostream>

void Robot::RobotInit() {
  rmb::FalconVelocityController::CreateInfo createInfo{
      .config =
          {
              .id = 2,
              .inverted = false,
          },
      .pidConfig =
          {
              .d = 0.0,
              .ff = 0.045,
              .closedLoopMaxPercentOutput = 1.0,
          },
      .profileConfig = {.maxVelocity = 100_tps,
                        .minVelocity = -100_tps,
                        .maxAcceleration = 1.0_rad_per_s_sq},
      .feedbackConfig = {.gearRatio = 6.12f},
      .openLoopConfig = {.minOutput = -1.0,
                         .maxOutput = 1.0,
                         .rampRate = 1.0_s},
      .canCoderConfig = {.useCANCoder = false},
  };

  rmb::FalconPositionController::CreateInfo positionControllerCreateInfo{
      .config = {.id = 1, .inverted = false},
      .pidConfig = {.p = 0.100, .d = 0.5, .ff = 0.000, .tolerance = 0.5_deg},
      .range = {.minPosition =
                    -(units::radian_t)std::numeric_limits<double>::infinity(),
                .maxPosition =
                    (units::radian_t)std::numeric_limits<double>::infinity()},
      .feedbackConfig =
          {
              .gearRatio = 12.8,
          },
      .openLoopConfig = {},
      .canCoderConfig = {.useCANCoder = false},
  };

  velocityController =
      std::make_unique<rmb::FalconVelocityController>(createInfo);

  positionController = std::make_unique<rmb::FalconPositionController>(
      positionControllerCreateInfo);
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
  // velocityController->setVelocity(2_tps);
  velocityController->setPower(0.5);

  // positionController->setPower(0.2);
  positionController->setPosition(0.25_tr);
  std::cout << "position: "
            << ((units::turn_t)positionController->getPosition())()
            << std::endl;
  // std::cout <<
  // ((units::turns_per_second_t)velocityController->getVelocity())()
  //           << std::endl;
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
