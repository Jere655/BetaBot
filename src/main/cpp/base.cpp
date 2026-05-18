#include "Base.h"

void Base::RobotInit() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor);
    m_rightMotor.SetInverted(true);
}

void Base::TeleopPeriodic() {
    double vitesse  =  m_driverController.GetLeftY();
    double rotation =  m_driverController.GetRightX();

    if (std::abs(vitesse)  < 0.1) vitesse  = 0;
    if (std::abs(rotation) < 0.1) rotation = 0;

    m_robotDrive.ArcadeDrive(vitesse, rotation);
}