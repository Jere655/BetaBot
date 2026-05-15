#pragma once

#include <CANVenom.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
 
class base {
    public:    // varoables public
        void robot_init();
        void teleop_periodic();
    protected: // pas vraiment utiliser

    private:   // variables privée
        pwf::CANVenom m_leftMotor{1};
 
        pwf::CANVenom m_rightMotor{2};
        
        frc::XboxController m_driverController{0};
        
        frc::DifferentialDrive m_robotDrive{
            m_leftMotor, m_rightMotor
        };
};