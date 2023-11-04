#pragma once

#include <optional>

#include "rmb/motorcontrol/AngularVelocityController.h"

#include "TalonFXPositionController.h"
#include "ctre/phoenix/sensors/WPI_CANCoder.h"
#include "units/angular_velocity.h"

namespace rmb {
namespace TalonFXVelocityControllerHelper {

struct OpenLoopConfig {
  double minOutput = -1.0, maxOutput = 1.0;
  units::second_t rampRate = 1.0_s;
};

struct ProfileConfig {
  units::radians_per_second_t maxVelocity = 0.0_rad_per_s,
                              minVelocity = 0.0_rad_per_s;
  units::radians_per_second_squared_t maxAcceleration = 0.0_rad_per_s_sq;
};
} // namespace TalonFXVelocityControllerHelper

class TalonFXVelocityController : public AngularVelocityController {
public:
  typedef units::unit<std::ratio<1, 2048>, units::turns> InternalEncoderTick;
  typedef units::unit_t<InternalEncoderTick> InternalEncoderTick_t;

  typedef units::compound_unit<InternalEncoderTick,
                               units::inverse<units::deciseconds>>
      RawInternalVelocityUnit;
  typedef units::unit_t<RawInternalVelocityUnit> RawInternalVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, InternalEncoderTick>
      RawInternalPositionUnit;
  typedef units::unit_t<RawInternalPositionUnit> RawInternalPositionUnit_t;

  //-------------CANCoder
  // Units---------------------------------------------------
  typedef units::unit<std::ratio<1, 4096>, units::turns> CANCoderTick;
  typedef units::unit_t<CANCoderTick> CANCoderTick_t;

  typedef units::compound_unit<CANCoderTick, units::inverse<units::deciseconds>>
      RawCANCoderVelocityUnit;
  typedef units::unit_t<RawCANCoderVelocityUnit> RawCANCoderVelocityUnit_t;

  typedef units::unit<std::ratio<1, 1>, CANCoderTick> RawCANCoderPositionUnit;
  typedef units::unit_t<RawCANCoderPositionUnit> RawCANCoderPositionUnit_t;

  struct CreateInfo {
    TalonFXPositionControllerHelper::MotorConfig config;
    AngularVelocityController::PIDConfig pidConfig;
    TalonFXVelocityControllerHelper::ProfileConfig profileConfig;
    TalonFXPositionControllerHelper::FeedbackConfig feedbackConfig;
    TalonFXVelocityControllerHelper::OpenLoopConfig openLoopConfig;
    TalonFXPositionControllerHelper::CANCoderConfig canCoderConfig;
  };

  TalonFXVelocityController(const CreateInfo &createInfo);

  //--------------------------------------------------
  // Methods Inherited from AngularVelocityController
  //--------------------------------------------------

  /**
   * Sets the target angular velocity.
   *
   * @param velocity The target angular velocity in radians per second.
   */
  void setVelocity(units::radians_per_second_t velocity) override;
  
  void setPIDConstants(PIDConfig config) override; 
  /**
   * Gets the target angular velocity.
   *
   * @return The target angular velocity in radians per second.
   */
  units::radians_per_second_t getTargetVelocity() const override;

  /**
   * Common interface for setting a mechanism's raw power output.
   */
  virtual void setPower(double power) override;

  /**
   * Common interface for disabling a mechanism.
   */
  virtual void disable() override;

  /**
   * Common interface to stop the mechanism until `setPosition` is called again.
   */
  virtual void stop() override;

  //---------------------------------------
  // Methods Inherited from AngularEncoder
  //---------------------------------------

  /**
   * Gets the angular velocity of the motor.
   *
   * @return The velocity of the motor in radians per second.
   */
  units::radians_per_second_t getVelocity() const override;

  /**
   * Gets the angular position of an motor.
   *
   * @return The position of the motor in radians.
   */
  units::radian_t getPosition() const override;

  /**
   * Zeros the angular positon the motor so the current position is set to
   * the offset. In the case of an absolute encoder this sets the zero offset
   * with no regard to the current position.
   *
   * @param offset the offset from the current angular position at which to
   *               set the zero position.
   */
  void zeroPosition(units::radian_t offset = 0_rad) override;

  //----------------------------------------------------------
  // Methods Inherited from AngularvelocityFeedbackController
  //----------------------------------------------------------

  /**
   * Gets the motor's tolerance.
   *
   * @return the motor's tolerance in radians per second.
   */
  virtual units::radians_per_second_t getTolerance() const override;

private:
  mutable ctre::phoenix::motorcontrol::can::WPI_TalonFX motorcontroller;

  float gearRatio = 0.0;

  units::radians_per_second_t tolerance = 0.0_tps;

  TalonFXVelocityControllerHelper::ProfileConfig profileConfig;

  const bool usingCANCoder;

  std::optional<ctre::phoenix::sensors::WPI_CANCoder> canCoder;
};


} // namespace rmb
