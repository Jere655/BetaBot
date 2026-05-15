#pragma once

#include <frc/TimedRobot.h>
#include <iostream>

#include <Base.h>


class Robot : public frc::TimedRobot {
    public:
        void RobotInit();         //Le code dans robot init va rouller unt foie quand le robot est ouvert
        void RobotPeriodic();     //Le code dans robot periodic va roulleren permenance quand le robot est ouvert

        void DisabledInit();      //Le code dans robot init va rouller unt foie quand le robot est mis a disable
        void DisabledPeriodic();  //Le code dans robot periodic va rouller en permenance que le robot est diable

        void TeleopInit();         //Le code dans robot init va rouller unt foie quand le robot est mis a teleop
        void TeleopPeriodic();     //Le code dans robot periodic va rouller en permenance que le robot est teleop
        
        void AutonomousInit();     //Le code dans robot init va rouller unt foie quand le robot est mis a autonomous
        void AutonomousPeriodic(); //Le code dans robot periodic va rouller en permenance que le robot est autonomous

        //il exist aussi test init et test periodic
    protected:
    private:
        base m_base;
};