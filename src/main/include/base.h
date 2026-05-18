#pragma once

#include <CANVenom.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>

class Base {
public:
    void RobotInit();
    void TeleopPeriodic();

private:
    pwf::CANVenom m_leftMotor{1};
    pwf::CANVenom m_rightMotor{2};
    frc::XboxController m_driverController{0};
    frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
};