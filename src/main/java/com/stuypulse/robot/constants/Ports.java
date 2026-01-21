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
            int FRONT_LEFT = 17;
            int BACK_LEFT = 15;
            int BACK_RIGHT = 10;
            int FRONT_RIGHT = 12;
        }
        public interface Turn {
            int FRONT_LEFT = 16;
            int BACK_LEFT = 14;
            int BACK_RIGHT = 11;
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
        int INTAKE_SHOOTER_MOTOR = 20;
        int INDEXER_MOTOR = 51;
    }
}
