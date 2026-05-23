#include "Base.h"

void Base::RobotInit() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor);
    m_rightMotor.SetInverted(true);
    m_compressor.EnableDigital();

    // Curseurs SmartDashboard — valeurs par défaut
    frc::SmartDashboard::SetDefaultNumber("Vitesse Base (%)",   38.0);
    frc::SmartDashboard::SetDefaultNumber("Vitesse Intake (%)",  0.0);
}

void Base::TeleopPeriodic() {
    // Lire les curseurs (0 à 100 → 0.0 à 1.0)
    double limiteBase   = frc::SmartDashboard::GetNumber("Vitesse Base (%)",   38.0) / 100.0;
    double limiteIntake = frc::SmartDashboard::GetNumber("Vitesse Intake (%)",  0.0) / 100.0;

    // Sécurité : limiter entre 0.0 et 1.0
    limiteBase   = std::clamp(limiteBase,   0.0, 1.0);
    limiteIntake = std::clamp(limiteIntake, 0.0, 1.0);

    // --- Base de propulsion ---
    double vitesse  = m_driverController.GetLeftY()  * limiteBase;
    double rotation = m_driverController.GetRightX() * limiteBase;

    if (std::abs(vitesse)  < 0.1)  vitesse  = 0;
    if (std::abs(rotation) < 0.15) rotation = 0;

    m_robotDrive.ArcadeDrive(vitesse, rotation);

    // --- Solénoïde intake ---
    if (m_driverController.GetXButton()) {
        m_intake.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        m_intake.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    // --- Moteur intake ---
    double rt = m_driverController.GetRightTriggerAxis();
    if (rt > 0.1) {
        m_intakeMotor.Set(rt * limiteIntake);
    } else if (m_driverController.GetXButton()) {
        m_intakeMotor.Set(-0.5 * limiteIntake);
    } else {
        m_intakeMotor.Set(0);
    }

    // Affichage des valeurs actives sur le dashboard
    frc::SmartDashboard::PutNumber("Vitesse Base active (%)",   limiteBase   * 100.0);
    frc::SmartDashboard::PutNumber("Vitesse Intake active (%)", limiteIntake * 100.0);
}