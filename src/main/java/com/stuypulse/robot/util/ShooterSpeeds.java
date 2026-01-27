package com.stuypulse.robot.util;

public class ShooterSpeeds {

    private final Number shooterRPM;

    public ShooterSpeeds() {
        this(0);
    }

    public ShooterSpeeds(Number shooterRPM) {
        this.shooterRPM = shooterRPM;
    }


    public double getRPM() {
        return shooterRPM.doubleValue();
    }

}
