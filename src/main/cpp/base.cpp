#include "Base.h"

void Base::RobotInit() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor);
    m_rightMotor.SetInverted(true);
    m_compressor.EnableDigital();
}

void Base::TeleopPeriodic() {
    double vitesse  =  m_driverController.GetLeftY();
    double rotation =  m_driverController.GetRightX();

    if (std::abs(vitesse)  < 0.1) vitesse  = 0;
    if (std::abs(rotation) < 0.15) rotation = 0;

    m_robotDrive.ArcadeDrive(vitesse, rotation);

    if (m_driverController.GetXButton()) {
        m_intake.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        m_intake.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    double rt = m_driverController.GetRightTriggerAxis();
    if (rt > 0.1) {
        m_intakeMotor.Set(rt);
    } else if (m_driverController.GetXButton()) {
        m_intakeMotor.Set(-0.5);
    } else {
        m_intakeMotor.Set(0);
    }
}