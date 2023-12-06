#include "AHRS.h"
#include "frc/geometry/Rotation2d.h"
#include "units/velocity.h"
#include <memory>
#include <rmb/sensors/AHRS/AHRSGyro.h>

namespace rmb {
AHRSGyro::AHRSGyro(std::unique_ptr<AHRS> gyro) : gyro(std::move(gyro)) {}

AHRSGyro::AHRSGyro(AHRS &&gyro)
    : gyro(std::make_unique<AHRS>(std::move(gyro))) {}

AHRSGyro::AHRSGyro(int id) : AHRSGyro(AHRS(frc::SerialPort::Port(id))) {}

units::turn_t AHRSGyro::AHRSGyro::getZRotation() const {
  return units::degree_t(gyro->GetRotation2d().Degrees()) - offset;
}

frc::Rotation2d AHRSGyro::getRotation() const {
  return gyro->GetRotation2d().RotateBy(units::degree_t(offset));
}

void AHRSGyro::resetZRotation(units::turn_t offset) { this->offset = offset; }

units::meters_per_second_squared_t AHRSGyro::getXAcceleration() const {
  return units::meters_per_second_squared_t(this->gyro->GetRawAccelX());
}

units::meters_per_second_squared_t AHRSGyro::getYAcceleration() const {
  return units::meters_per_second_squared_t(this->gyro->GetRawAccelY());
}

units::meters_per_second_squared_t AHRSGyro::getZAcceleration() const {
  return units::meters_per_second_squared_t(this->gyro->GetRawAccelZ());
}

units::meters_per_second_t AHRSGyro::getXVelocity() const {
  return units::meters_per_second_t(this->gyro->GetVelocityX());
}

units::meters_per_second_t AHRSGyro::getYVelocity() const {
  return units::meters_per_second_t(this->gyro->GetVelocityY());
}

units::meters_per_second_t AHRSGyro::getZVelocity() const {
  return units::meters_per_second_t(this->gyro->GetVelocityZ());
}
} // namespace rmb
