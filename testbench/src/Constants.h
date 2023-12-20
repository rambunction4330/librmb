#pragma once

#include <rmb/motorcontrol/Talon/TalonFXPositionController.h>
#include <rmb/motorcontrol/Talon/TalonFXVelocityController.h>

#include <frc/SerialPort.h>

namespace constants {

const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityModulePIDConfig =
    {
        .p = 1.5,
        .i = 0.0003,
        .d = 0.00,
        .ff = 0.000,
        .closedLoopMaxPercentOutput = 1.0,
};
const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityModulePIDConfig1 =
    {
        .p = 0.40,
        .i = 0.0003,
        .d = 0.00,
        .ff = 0.000,
        .closedLoopMaxPercentOutput = 1.0,
};
const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityModulePIDConfig2 =
    {
        .p = 0.40,
        .i = 0.0003,
        .d = 0.00,
        .ff = 0.000,
        .closedLoopMaxPercentOutput = 1.0,
};
const rmb::TalonFXVelocityControllerHelper::PIDConfig velocityModulePIDConfig3 =
    {
        .p = 0.40,
        .i = 0.0003,
        .d = 0.00,
        .ff = 0.000,
        .closedLoopMaxPercentOutput = 1.0,
};

const units::turn_t module1Zero(-0.156250);
const units::turn_t module2Zero(-0.173828);
const units::turn_t module3Zero(-0.227295);
const units::turn_t module4Zero(-0.879883);

const units::meter_t wheelCircumference = 1.0_m;

const frc::SerialPort::Port gyroPort = frc::SerialPort::Port::kMXP;

const rmb::TalonFXPositionControllerHelper::PIDConfig positionModulePIDConfig =
    {.p = 0.8f, .i = 0.0000f, .d = 0.0f, .ff = 0.000, .tolerance = 0.1_deg};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo{
    .config =
        {
            .id = 10,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo{
    .config = {.id = 12, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 11,
                       .zeroPosition = module1Zero,
                       .remoteSensorSlot = 0},
};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo1{
    .config =
        {
            .id = 20,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig1,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo1{
    .config = {.id = 22, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 21,
                       .zeroPosition = module2Zero,
                       .remoteSensorSlot = 0},
};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo2{
    .config =
        {
            .id = 30,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig2,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo2{
    .config = {.id = 32, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 31,
                       .zeroPosition = module3Zero,
                       .remoteSensorSlot = 0},
};

const rmb::TalonFXVelocityController::CreateInfo velocityControllerCreateInfo3{
    .config =
        {
            .id = 40,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig3,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.sensorToMechanismRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::TalonFXPositionController::CreateInfo positionControllerCreateInfo3{
    .config = {.id = 42, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = true},
    .feedbackConfig =
        {
            .sensorToMechanismRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 41,
                       .zeroPosition = module4Zero,
                       .remoteSensorSlot = 0},
};

} // namespace constants
