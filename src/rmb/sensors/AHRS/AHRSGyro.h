#pragma once

#include "AHRS.h"
#include "frc/SerialPort.h"
#include "frc/geometry/Rotation2d.h"
#include "units/acceleration.h"
#include <memory>
#include <rmb/sensors/gyro.h>

namespace rmb {
class AHRSGyro : public Gyro {
public:
  AHRSGyro(frc::SerialPort::Port port);

  virtual ~AHRSGyro() = default;

  virtual units::turn_t getZRotation() const override;
  virtual void resetZRotation() override;
  virtual frc::Rotation2d getRotation() const override;

  virtual units::meters_per_second_squared_t getXAcceleration() const override;
  virtual units::meters_per_second_squared_t getYAcceleration() const override;
  virtual units::meters_per_second_squared_t getZAcceleration() const override;

  virtual units::meters_per_second_t getXVelocity() const override;
  virtual units::meters_per_second_t getYVelocity() const override;
  virtual units::meters_per_second_t getZVelocity() const override;

private:
  std::unique_ptr<AHRS> gyro;
};
} // namespace rmb
