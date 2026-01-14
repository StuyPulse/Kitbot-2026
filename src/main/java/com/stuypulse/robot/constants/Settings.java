/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface Settings {

    double DT = 0.020;
    boolean DEBUG_MODE = true;
    CANBus CANIVORE = new CANBus("DEFAULT_NAME", "./logs/example.hoot");

    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", true);
    }

    public interface Superstructure {
        boolean intakeShooterInverted = false;
        boolean indexerInverted = false;

        public interface Intake_Shooter_Speeds {
            // states we have
            // intaking <
            // outtaking <
            // preparing to shoot (we dont need additional values for this, we'll just run the shooter to speed while not touching the indexer)
            // shooting <
            // we dont need anything for STOP because its 0, 0

            // TODO: find acutal values for these
            double INTAKE_SPEED = 0.5;
            double OUTTAKE_SPEED = -0.5;
            double SHOOTING_SPEED = 1;
            
            // TODO: Tune ts
            double SHOOT_TOLERANCE_RPM = 50.0;
        }

        public interface Indexer_Speeds {
            // for the indexer we really only need it to run in one direction or the other, 
            // we don't need anything in between. 
            // you can have one value for when youre intaking
            // and another for when you are outtaking, whether through the shooter or back out through the intake
            double INTAKE_SPEED = 1;
            double OUTTAKE_SPEED = -1;
        }
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        
        public interface Constraints {    
            double MAX_VELOCITY_M_PER_S = 4.3;
            double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400);
            double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(900);
    
            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_M_PER_S,
                    MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S,
                    MAX_ANGULAR_ACCEL_RAD_PER_S);
        }

        public interface Alignment {
            public interface Constraints {
                double DEFAULT_MAX_VELOCITY = 4.3;
                double DEFAULT_MAX_ACCELERATION = 15.0;
                double DEFUALT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400);
                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900);
            }

            public interface Tolerances {
                double X_TOLERANCE = Units.inchesToMeters(2.0); 
                double Y_TOLERANCE = Units.inchesToMeters(2.0);
                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(2.0);

                Pose2d POSE_TOLERANCE = new Pose2d(
                    Units.inchesToMeters(2.0), 
                    Units.inchesToMeters(2.0), 
                    Rotation2d.fromDegrees(2.0));

                double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {
            }
        }
    }
   

}
