// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "rmb/motorcontrol/falcon/FalconVelocityController.h"

#include <frc2/command/CommandScheduler.h>
#include <memory>

void Robot::RobotInit() {
  rmb::FalconVelocityController::CreateInfo createInfo {
    .config = {
      .id = 0,
      .inverted = false,
    },
    .pidConfig = {
      .p = 0.1f,
      .closedLoopMaxPercentOutput = 1.0f,
    },
    .profileConfig = {
      .maxVelocity = 100_tps,
      .minVelocity = -100_tps,
      .maxAcceleration = 1.0_rad_per_s_sq
    },
    .feedbackConfig = {
      .gearRatio = 1.0f
    },
    .canCoderConfig = {
      .useCANCoder = false
    },
  };

  velocityController = std::make_unique<rmb::FalconVelocityController>(createInfo);
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() {
  velocityController->setPower(0.2);
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
