package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.stuypulse.stuylib.network.SmartNumber;

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
            double kS = 0.17608;
            double kV = 0.11448;
            double kA = 0.0059131;
            double kP = 0.096506;
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
            SmartNumber xkP = new SmartNumber("Gains/Alignment/xkP", 3.2);
            SmartNumber xkI = new SmartNumber("Gains/Alignment/xkI", 0.0);
            SmartNumber xkD = new SmartNumber("Gains/Alignment/xkD", 0.0);

            SmartNumber ykP = new SmartNumber("Gains/Alignment/ykP", 3.2);
            SmartNumber ykI = new SmartNumber("Gains/Alignment/ykI", 0.0);
            SmartNumber ykD = new SmartNumber("Gains/Alignment/ykD", 0.0);

            SmartNumber akP = new SmartNumber("Gains/Alignment/akP", 3.0);
            SmartNumber akI = new SmartNumber("Gains/Alignment/akI", 0.0);
            SmartNumber akD = new SmartNumber("Gains/Alignment/akD", 0.0);

            
            // PIDConstants XY = new PIDConstants(3.2, 0, 0.25);
            // PIDConstants THETA = new PIDConstants(3.0, 0, 0.1);

            PIDConstants X = new PIDConstants(xkP.doubleValue(), xkI.doubleValue(), xkD.doubleValue());
            PIDConstants Y = new PIDConstants(ykP.doubleValue(), ykI.doubleValue(), ykD.doubleValue());
            PIDConstants THETA = new PIDConstants(akP.doubleValue(), akI.doubleValue(), akD.doubleValue());
        }
    }
}
