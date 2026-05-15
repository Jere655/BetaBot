#include <Base.h>
 
void base::robot_init() {

    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightMotor);

    m_rightMotor.SetInverted(true);
}
 
 
void base::teleop_periodic() {
    m_robotDrive.TankDrive(-m_driverController.GetLeftY(), -m_driverController.GetRightY());
}