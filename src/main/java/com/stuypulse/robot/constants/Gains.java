package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {
    public interface Swerve {
        public interface Alignment {
            double kP = 0.0;
            double kI = 0;
            double kD = 0.0;
            double akP = 0.0;
            double akI = 0;
            double akD = 0.0;

            PIDConstants XY = new PIDConstants(0.0, 0, 0.0);
            PIDConstants THETA = new PIDConstants(0.0, 0, 0.0);
        }
    }
}
