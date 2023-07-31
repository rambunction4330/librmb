// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "rmb/motorcontrol/falcon/FalconPositionController.h"
#include <rmb/motorcontrol/falcon/FalconVelocityController.h>

#include <rmb/drive/SwerveDrive.h>

#include <AHRS.h>

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  std::unique_ptr<rmb::FalconVelocityController> velocityController;
  std::unique_ptr<rmb::FalconPositionController> positionController;

  std::shared_ptr<AHRS> gyro;

  RobotContainer m_container;
};
