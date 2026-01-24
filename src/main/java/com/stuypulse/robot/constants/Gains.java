package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {
    public interface Turret {
        double kS = 0.0;
        double kV = 0.2;
        double kA = 0.01;

        double kP = 1.0;
        double kI = 0.0;
        double kD = 0.05;
    }
    
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
