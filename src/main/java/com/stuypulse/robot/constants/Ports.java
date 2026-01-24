/************************ PROJECT KITBOT ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }
    //TODO: GET PORTS
    public interface Turret {
        int TURRET_MOTOR = 0;
        int ENCODER_18t = 0;   
        int ENCODER_17t = 0; 
    }
    public interface Swerve {
        public interface Drive {
            int FRONT_LEFT = 32;
            int BACK_LEFT = 22;
            int BACK_RIGHT = 21;
            int FRONT_RIGHT = 50;
        }
        public interface Turn {
            int FRONT_LEFT = 42;
            int BACK_LEFT = 11;
            int BACK_RIGHT = 0;
            int FRONT_RIGHT = 13;
        }
        public interface CANCoderIds {
            int FRONT_LEFT = 4;
            int BACK_LEFT = 3;
            int BACK_RIGHT = 1;
            int FRONT_RIGHT = 2;
        }
    }

    public interface Superstructure {
        int INTAKE_SHOOTER_MOTOR = 14;
        int INDEXER_MOTOR = 2;
    }
}
