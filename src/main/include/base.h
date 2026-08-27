#pragma once

#include <CANVenom.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
    rev::spark::SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();

    // État du toggle de déploiement pneumatique
    bool m_intakeLeve = false;
    bool m_xPrecedent = false;

    // Mouvement automatique de l'intake (0 = repos, 1 = monte, 2 = descend)
    int  m_etatIntake = 0;
    bool m_aPrecedent = false;
    bool m_bPrecedent = false;
};