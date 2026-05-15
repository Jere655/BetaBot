#include "Robot.h"

void Robot::RobotInit(){
  m_base.robot_init();
}

void Robot::RobotPeriodic(){
}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic(){
  m_base.teleop_periodic();
}

void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {

}

void Robot::AutonomousInit() {
}


void Robot::AutonomousPeriodic() {
}





#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
