package com.stuypulse.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    public static final Shooter instance;

    static {
            instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        // add ShooterState constructor and potential states for the game
    }
}
