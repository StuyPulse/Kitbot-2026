/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
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

    public interface Swerve {
        public interface Drive {
            int FRONT_LEFT = 11;
            int BACK_LEFT = 13;
            int BACK_RIGHT = 15;
            int FRONT_RIGHT = 17;
        }
        public interface Turn {
            int FRONT_LEFT = 10;
            int BACK_LEFT = 12;
            int BACK_RIGHT = 14;
            int FRONT_RIGHT = 16;
        }
        public interface CANCoderIds {
            int FRONT_LEFT = 1;
            int BACK_LEFT = 2;
            int BACK_RIGHT = 3;
            int FRONT_RIGHT = 4;
        }
    }

    public interface Superstructure {
        int INTAKE_SHOOTER_MOTOR = 20;
        int INDEXER_MOTOR = 21;
    }
}
