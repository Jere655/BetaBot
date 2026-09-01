#pragma once

#include <CANVenom.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include "Vision.h"

class Base {
public:
    void RobotInit();
    void TeleopPeriodic();
    void AutonomousInit();
    void AutonomousPeriodic();

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
    bool m_yPrecedent = false;

    Vision m_vision;

    // --- Odométrie ---
    double m_prevLeftPos = 0.0;
    double m_prevRightPos = 0.0;
    double m_heading = 0.0;

    // --- Machine d'état autonome ---
    enum class AutoState {
        SEARCH,
        APPROACH,
        DEPLOY_INTAKE,
        ALIGN_BACK,
        REVERSE_COLLECT,
        COLLECTING,
        RETURN_SEARCH,
        OBSTACLE_AVOID
    };
    AutoState m_autoState = AutoState::SEARCH;
    double m_autoTimer = 0.0;
    double m_searchDirection = 1.0;
    double m_collectionHeading = 0.0;
    double m_ballLostTimer = 0.0;

    void AutoUpdateOdometry();
    bool AutoBallDetected() const;
    double AutoBallCenterX() const;
    double AutoBallArea() const;
    void AutoDrive(double speed, double rotation);
    void AutoLowerIntake();
    void AutoRaiseIntake();
    void AutoRunIntakeRoller(double speed);
    void AutoStopIntake();
};