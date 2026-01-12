package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

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
        IDLE(0), SHOOTING(1E10);

        private double speed;

        private ShooterState(double speed) {
            this.speed = speed;
        }

        public double getState() {
            return this.speed;
        }
    }

    private ShooterState state;

    public void setState(ShooterState state) {
        this.state = state;
    }

    protected Shooter() {
        this.state = ShooterState.IDLE;
    }

    public abstract void shooterMotorConfig(NeutralModeValue mode);

}
