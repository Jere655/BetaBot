#include "Robot.h"

void Robot::RobotInit() {
    m_base.RobotInit();
}

void Robot::RobotPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
    m_base.TeleopPeriodic();
}

void Robot::AutonomousInit() {
    m_base.AutonomousInit();
}

void Robot::AutonomousPeriodic() {
    m_base.AutonomousPeriodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif