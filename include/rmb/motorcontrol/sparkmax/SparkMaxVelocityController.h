#pragma once

#include <memory>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/base.h>

#include <rev/CANSparkMax.h>

#include "rmb/motorcontrol/VelocityController.h"
#include "rmb/motorcontrol/feedforward/SimpleMotorFeedforward.h"

namespace rmb {

/**
 * A wrapper around the SparkMax motorcontroller that allows for the user to set
 * and get the velocity of the motor accurately through PID functionallity
 */
template <typename DistanceUnit>
class SparkMaxVelocityController : public VelocityController<DistanceUnit> {
public:
  using Distance_t = typename VelocityController<DistanceUnit>::
      Distance_t; /**< @see VelocityController<DistanceUnit>::Distance_t*/

  using VeloctyUnit = typename VelocityController<DistanceUnit>::
      VelocityUnit; /**< @see VelocityController<DistanceUnit>::VelocityUnit*/
  using Velocity_t = typename VelocityController<DistanceUnit>::
      Velocity_t; /**< @see VelocityController<DistanceUnit>::Velocity_t*/

  using AccelerationUnit = typename VelocityController<DistanceUnit>::
      AccelerationUnit; /**< @see
                           VelocityController<DistanceUnit>::AccelerationUnit*/
  using Acceleration_t = typename VelocityController<DistanceUnit>::
      Acceleration_t; /**< @see
                         VelocityController<DistanceUnit>::Acceleration_t*/

  using ConversionUnit = typename units::compound_unit<
      DistanceUnit,
      units::inverse<units::radians>>; /**< The ratio between the user-defined
                                          DistanceUnit and radians. This is used
                                          for converting to and from
                                          user-defined units and raw units. For
                                          example, if a wheel has a wheel with a
                                          circumference of 2 meters, the
                                          conversion from meters to radians (2pi
                                          = 1 rotation) would be:

                                                2 meters / 2pi radians
                                                (1/pi) m/rad
                                                conversion ??? 0.3183

                                            */
  using ConversionUnit_t =
      typename units::unit_t<ConversionUnit>; /**< Type declaration of
                                                 ConversionUnit*/

  // raw velocity is in rpm. 1 rotation * 2pi rad = 2pi rad
  using RawUnit = typename units::unit<
      std::ratio<2>, units::radians,
      std::ratio<1>>; /**< The SparkMax does it's distances in rotations.
                           The rotations quantity is based on radians. The math
                         for the conversion is as follows:

                               1 rotation * 2pi rad = 2pi rad

                           Thus the multiplier provided to the units:: library
                         is 2pi
                      */
  using RawUnit_t =
      typename units::unit_t<RawUnit>; /**< Type declaration for RawUnit*/

  using RawVelocity = typename units::compound_unit<
      RawUnit,
      units::inverse<units::minutes>>; /**< The SparkMax does it's velocities in
                                          rotations per minute */
  using RawVelocity_t =
      typename units::unit_t<RawVelocity>; /**< Type declaration for
                                              RawVelocity*/
  using RawAccel = typename units::compound_unit<
      RawVelocity,
      units::inverse<units::seconds>>; /**< Oddly, the SparkMax performs it's
                                          acceleration calculations in rotations
                                          per minute per second. */
  using RawAccel_t =
      typename units::unit_t<RawAccel>; /**< Type declaration for RawAccel*/

  /**
   * The PID constants used by the constructor of SparkMaxVelocityController.
   */
  struct PIDConfig {
    double
        p = 0.000057181, /**< Proportional Gain @see
                            https://en.wikipedia.org/wiki/PID_controller#Proportional*/
        i = 0.0,         /**< Integral Gain @see
                            https://en.wikipedia.org/wiki/PID_controller#Integral*/
        d = 0.0,         /**< Derivative Gain @see
                            https://en.wikipedia.org/wiki/PID_controller#Derivative*/
        f = 0.0;         /**< Feed Forward*/

    double iZone =
               0.0, /**< Bound in which the error needs to be in for the
integral value to take effect
@see
https://codedocs.revrobotics.com/cpp/classrev_1_1_c_a_n_p_i_d_controller.html#a4e724c38342a820d23ba79be0929a839*/
        iMaxAccumulator = 0.0; /**< \deprecated*/

    double maxOutput = 1.0, /**< Maximum motor output. Has to be a value between
                               between 0.0 and 1.0*/
        minOutput = -1.0;   /**< Minimum motor output. Has to be a value between
                               0.0 and 1.0*/

    // SmartMotion config
    bool usingSmartMotion = true; /**< Whether or not to use Smart Motion.
                                       If true, the encoder will take into
                                     account the params below*/

    Velocity_t maxVelocity = Velocity_t(25), /**< Smart Motion maximum velocity
                                                in user defined velocity units*/
        minVelocity = Velocity_t(0); /**< Smart Motion minimum velocity in user
                                        defined velocity units*/

    Acceleration_t maxAccel = Acceleration_t(
        10); /**< Maximum allowed acceleration in user defined accel units*/
    Velocity_t allowedErr = Velocity_t(
        0.9); /**< Allowed velocity error in user defined velocity units*/
    rev::SparkMaxPIDController::AccelStrategy accelStrategy =
        rev::SparkMaxPIDController::AccelStrategy::
            kSCurve; /**< The acceleration strategy to use. Can be
                          kSCurve or kTrapezoidal*/
  };

  /**
   * Type definition for the Follower functionality that the SparkMaxes provide.
   * A follower is a motor that Will "follow" the specified motor. You can
   * specify followers in the constructor.
   */
  struct Follower {
    int id;                                /**< device ID of the follower */
    rev::CANSparkMax::MotorType motorType; /**< Motor type of the follower. Can
                                              be kBrushless or kBrushed */
    bool inverted; /**< Whether or not the motor is inverted or not. true for
                      inverted.*/
  };

  /** \deprecated
   * Creates a new SparkMaxVelocityController with the specified device id
   * @param deviceID the ID of the target SparkMax motor controller
   */
  [[deprecated(
      "Do not use this constructor in competition. This is not "
      "guaranteed to work!")]] SparkMaxVelocityController(int deviceID);

  /**
   * Creates a SparkMaxVelocityController
   * @param deviceID the ID of the target SparkMax motor controller
   * @param config PID constants to be set on the SparkMax's PID controller.
   *               These are set once.
   * @param conversionUnit This is the ratio between DistanceUnits(the units
   * that the user passes in) and units::radians. For example, if the unit
   * passed in is rotations, the process to find the conversion unit would be \n
   *
   *                           1 rotation / 2pi rad =>
   *                           1/2pi rot/rad
   *                           conversion ??? 0.1592
   *
   * @param feedforward the feedforward to be used. Defaults to
   * noFeedforward<DistanceUnit>
   * @param followers a list of followers to follow the motor
   * @param alternateEncoder whether or not the VelocityController should use
   * the encoder plugged into the SparkMax's data port
   * @param countPerRevolution the number of ticks per revolution of the
   * alternate encoder
   * @param motorType the type of motor. Can be kBrushed or kBrushless.
   */
  SparkMaxVelocityController(
      int deviceID, const PIDConfig &config,
      ConversionUnit_t conversionUnit = ConversionUnit_t(1),
      const Feedforward<DistanceUnit> &feedforward =
          noFeedforward<DistanceUnit>,
      std::initializer_list<Follower> followers = {},
      bool alternateEncoder = false, int countPerRevolution = 4096,
      rev::CANSparkMax::MotorType motorType =
          rev::CANSparkMax::MotorType::kBrushless);

  /**
   * Sets the target velocity of the motorcontroller
   * @param velocity The target velocity
   */
  void setVelocity(Velocity_t velocity) override;

  /**
   * Returns the current velocity of the motor
   * @return The current velocity of the motor
   */
  Velocity_t getVelocity() override;

  /**
   * Toggles motorcontroller inversion
   * @param inverted If this is true, the motorcontroller will be inverted
   */
  inline void setInverted(bool inverted) override {
    sparkMax.SetInverted(inverted);
  };

  /**
   * Get the inversion status of the motorcontroller
   * @return The inversion status of the motorcontroller.
   *          If this is true, the motor controller is inverted
   */
  inline bool getInverted() override { return sparkMax.GetInverted(); };

private:
  rev::CANSparkMax sparkMax;
  std::unique_ptr<rev::RelativeEncoder> sparkMaxEncoder;
  rev::SparkMaxPIDController sparkMaxPIDController;
  ConversionUnit_t conversion;
  const Feedforward<DistanceUnit> &feedforward;

  rev::CANSparkMax::ControlType controlType;
  std::vector<std::unique_ptr<rev::CANSparkMax>> followers;
};
} // namespace rmb
