#pragma once

#include <frc/TimedRobot.h>
#include "Base.h"

class Robot : public frc::TimedRobot {
public:
    void RobotInit();          // Roule une fois à l'ouverture du robot
    void RobotPeriodic();      // Roule en permanence

    void DisabledInit();       // Roule une fois quand mis en disabled
    void DisabledPeriodic();   // Roule en permanence en disabled

    void TeleopInit();         // Roule une fois quand mis en teleop
    void TeleopPeriodic();     // Roule en permanence en teleop

    void AutonomousInit();     // Roule une fois quand mis en autonomous
    void AutonomousPeriodic(); // Roule en permanence en autonomous

private:
    Base m_base;
};