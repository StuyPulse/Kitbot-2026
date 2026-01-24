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

    public interface Shooter {
        public interface PID {
            double kP = 2.0;
            double kI = 0.0;
            double kD = 0.20;
        }

        public interface FF {
            double kS = 0.1;
            double kV = 0.2;
            double kA = 0.01;
            double kG = 0.0;
        }
    }

    public interface Swerve {
        public interface Drive {
            double kS = 0.0;
            double kV = 0.124;
            double kA = 0.0;
            double kP = 0.1;
            double kI = 0.0;
            double kD = 0.0;
        }
        public interface Turn {
            double kS = 0.1;
            double kV = 2.66;
            double kA = 0.0;
            double kP = 100.0;
            double kI = 0.0;
            double kD = 0.5;
        }
        public interface Motion {
            PIDConstants XY = new PIDConstants(2.0, 0, 0.25);
            PIDConstants THETA = new PIDConstants(5.0, 0, 0.2);
        }
        public interface Alignment {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
            double akP = 0.0;
            double akI = 0.0;
            double akD = 0.0;

            PIDConstants XY = new PIDConstants(0.0, 0.0, 0.0);
            PIDConstants THETA = new PIDConstants(0.0, 0.0, 0.0);
        }
    }
}
