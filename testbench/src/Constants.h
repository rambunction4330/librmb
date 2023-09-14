#pragma once

#include <rmb/motorcontrol/falcon/FalconPositionController.h>
#include <rmb/motorcontrol/falcon/FalconVelocityController.h>

#include <frc/SerialPort.h>

namespace constants {

const rmb::FalconVelocityControllerHelper::PIDConfig velocityModulePIDConfig = {
    .p = 0.130,
    .i = 0.0001,
    .d = 0.5,
    .ff = 0.00,
    .closedLoopMaxPercentOutput = 1.0,
};

const rmb::FalconPositionController::RawCANCoderPositionUnit_t
    module1Zero(690.0);
const rmb::FalconPositionController::RawCANCoderPositionUnit_t
    module2Zero(784.0);
const rmb::FalconPositionController::RawCANCoderPositionUnit_t
    module3Zero(926.0);
const rmb::FalconPositionController::RawCANCoderPositionUnit_t
    module4Zero(3567.0);

const units::meter_t wheelCircumference = 1.0_m;

const frc::SerialPort::Port gyroPort = frc::SerialPort::Port::kMXP;

const rmb::FalconPositionControllerHelper::PIDConfig positionModulePIDConfig = {
    .p = 1.000f, .i = 0.0f, .d = 1.0f, .ff = 0.000, .tolerance = 0.1_deg};

const rmb::FalconVelocityController::CreateInfo velocityControllerCreateInfo{
    .config =
        {
            .id = 10,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.gearRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::FalconPositionController::CreateInfo positionControllerCreateInfo{
    .config = {.id = 12, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = false},
    .feedbackConfig =
        {
            .gearRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 11,
                       .zeroPosition = module1Zero,
                       .remoteSensorSlot = 0},
};

const rmb::FalconVelocityController::CreateInfo velocityControllerCreateInfo1{
    .config =
        {
            .id = 20,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.gearRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::FalconPositionController::CreateInfo positionControllerCreateInfo1{
    .config = {.id = 22, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = false},
    .feedbackConfig =
        {
            .gearRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 21,
                       .zeroPosition = module2Zero,
                       .remoteSensorSlot = 0},
};

const rmb::FalconVelocityController::CreateInfo velocityControllerCreateInfo2{
    .config =
        {
            .id = 30,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.gearRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::FalconPositionController::CreateInfo positionControllerCreateInfo2{
    .config = {.id = 32, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = false},
    .feedbackConfig =
        {
            .gearRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 31,
                       .zeroPosition = module3Zero,
                       .remoteSensorSlot = 0},
};

const rmb::FalconVelocityController::CreateInfo velocityControllerCreateInfo3{
    .config =
        {
            .id = 40,
            .inverted = false,
        },
    .pidConfig = velocityModulePIDConfig,
    .profileConfig = {.maxVelocity = 100_tps,
                      .minVelocity = -100_tps,
                      .maxAcceleration = 1.0_rad_per_s_sq},
    .feedbackConfig = {.gearRatio = 6.12f},
    .openLoopConfig = {.minOutput = -1.0, .maxOutput = 1.0, .rampRate = 1.0_s},
    .canCoderConfig = {.useCANCoder = false},
};

const rmb::FalconPositionController::CreateInfo positionControllerCreateInfo3{
    .config = {.id = 42, .inverted = false},
    .pidConfig = positionModulePIDConfig,
    .range = {.minPosition =
                  -(units::radian_t)std::numeric_limits<double>::infinity(),
              .maxPosition =
                  (units::radian_t)std::numeric_limits<double>::infinity(),
              .isContinuous = false},
    .feedbackConfig =
        {
            .gearRatio = 12.8,
        },
    .openLoopConfig = {},
    .canCoderConfig = {.useCANCoder = true,
                       .id = 41,
                       .zeroPosition = module4Zero,
                       .remoteSensorSlot = 0},
};

} // namespace constants
