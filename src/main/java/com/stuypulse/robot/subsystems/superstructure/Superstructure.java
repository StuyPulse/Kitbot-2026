package com.stuypulse.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    public static final Superstructure instance;

    static {
            instance = new SuperstructureImpl();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public enum SuperstructureState {
        // add ShooterState constructor and potential states for the game
    }
}
