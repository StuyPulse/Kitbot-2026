package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    public interface Turret {
        public final int encoderTeeth18 = 18;
        public final int encoderTeeth17 = 17;
        public final int bigGearTeeth = 84;

        public final Pose2d robotToTurret = new Pose2d(
            new Translation2d(Units.inchesToMeters(14.750), Units.inchesToMeters(-0.715)), 
            new Rotation2d()
        );
    }
}
