
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "RobotContainer.h"

#include "networktables/DoubleTopic.h"
#include "rmb/sensors/AHRS/AHRSGyro.h"
#include <rmb/motorcontrol/Talon/TalonFXPositionController.h>
#include <rmb/motorcontrol/Talon/TalonFXVelocityController.h>

#include <rmb/motorcontrol/Talon/TalonFXVelocityController.h>

#include <rmb/controller/LogitechGamepad.h>
#include <rmb/controller/LogitechJoystick.h>

#include <rmb/drive/SwerveDrive.h>

#include <AHRS.h>

class Robot : public frc::TimedRobot {
public:
  Robot() : frc::TimedRobot(40_ms) {}
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

  std::unique_ptr<rmb::SwerveDrive<4>> swerveDrive;

  // rmb::LogitechJoystick joystick = rmb::LogitechJoystick(0, 0.05);
  rmb::LogitechGamepad gamepad = rmb::LogitechGamepad(0, 0.05, false);

  std::shared_ptr<rmb::AHRSGyro> gyro =
      std::make_shared<rmb::AHRSGyro>(constants::gyroPort);

  RobotContainer m_container;
};
