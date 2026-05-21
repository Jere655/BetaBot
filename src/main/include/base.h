#pragma once

#include <CANVenom.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/Compressor.h>
#include <rev/SparkMax.h>

class Base {
public:
    void RobotInit();
    void TeleopPeriodic();

private:
    pwf::CANVenom m_leftMotor{1};
    pwf::CANVenom m_rightMotor{2};
    frc::XboxController m_driverController{0};
    frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
    frc::DoubleSolenoid m_intake{3, frc::PneumaticsModuleType::CTREPCM, 0, 1};
    frc::Compressor m_compressor{3, frc::PneumaticsModuleType::CTREPCM};
    rev::spark::SparkMax m_intakeMotor{17, rev::spark::SparkMax::MotorType::kBrushless};
};