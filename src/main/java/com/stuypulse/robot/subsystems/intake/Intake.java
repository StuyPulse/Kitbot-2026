package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    public static final Intake instance;

    static {
            instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {
        // add IntakeState constructor and potential states for the game
    }
}
