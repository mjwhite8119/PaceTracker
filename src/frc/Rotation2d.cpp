/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/Rotation2d.h"
#include <cmath>

Rotation2d::Rotation2d(float value)
    : m_value(value),
      m_cos(std::cos(value)),
      m_sin(std::sin(value)) {}

Rotation2d::Rotation2d(float x, float y) {
  const auto magnitude = std::hypot(x, y);
  if (magnitude > 1e-6) {
    m_sin = y / magnitude;
    m_cos = x / magnitude;
  } else {
    m_sin = 0.0;
    m_cos = 1.0;
  }
  m_value = float(std::atan2(m_sin, m_cos));
}

Rotation2d Rotation2d::operator+(const Rotation2d& other) const {
  return RotateBy(other);
}

Rotation2d& Rotation2d::operator+=(const Rotation2d& other) {
  float cos = Cos() * other.Cos() - Sin() * other.Sin();
  float sin = Cos() * other.Sin() + Sin() * other.Cos();
  m_cos = cos;
  m_sin = sin;
  m_value = float(std::atan2(m_sin, m_cos));
  return *this;
}

Rotation2d Rotation2d::operator-(const Rotation2d& other) const {
  return *this + -other;
}

Rotation2d& Rotation2d::operator-=(const Rotation2d& other) {
  *this += -other;
  return *this;
}

Rotation2d Rotation2d::operator-() const { return Rotation2d(-m_value); }

Rotation2d Rotation2d::operator*(float scalar) const {
  return Rotation2d(m_value * scalar);
}

bool Rotation2d::operator==(const Rotation2d& other) const {
  return std::abs(m_value - other.m_value) < 1E-9;
}

bool Rotation2d::operator!=(const Rotation2d& other) const {
  return !operator==(other);
}

Rotation2d Rotation2d::RotateBy(const Rotation2d& other) const {
  return {Cos() * other.Cos() - Sin() * other.Sin(),
          Cos() * other.Sin() + Sin() * other.Cos()};
}
