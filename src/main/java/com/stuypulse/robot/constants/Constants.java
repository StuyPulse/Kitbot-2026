package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    public interface Turret {
        int encoderTeeth18 = 18;
        int encoderTeeth17 = 17;
        int bigGearTeeth = 84;

        double RANGE = 420.0;

        public final Pose2d robotToTurret = new Pose2d(
            new Translation2d(Units.inchesToMeters(14.750), Units.inchesToMeters(-0.715)), 
            new Rotation2d()
        );
    }

    public interface ClimberHopper {
        // TODO: get these limits
        double MIN_HEIGHT_METERS = 0;
        double MAX_HEIGHT_METERS = 1;

        double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (Encoders.NUM_ROTATIONS_TO_REACH_TOP / Encoders.GEARING)) / 2 / Math.PI;

        double MASS_KG = 1;

        public interface Encoders {
            // TODO: get these
            double GEARING = 52.0/12.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (0.480 / 13); // Number of rotations that the motor has to spin, NOT the gear
            double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }
}
