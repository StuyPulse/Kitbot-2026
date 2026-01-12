package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings;

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
        // for example different times we want to run the wheels at different speeds
        // like SHOOTING can be one state where we run the wheels at a certain speed
        // while INTAKING or STOWING could be where we don't run the wheels
        
        SHOOTING(Settings.Shooter.SHOOTER_SHOOT_SPEED),
        INTAKING(Settings.Shooter.SHOOTER_INTAKE_SPEED),
        STOP(0);
    
        private Number speed;

        private ShooterState(Number speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return this.speed.doubleValue();
        }
    }

    private ShooterState state;

    protected Shooter() {
        this.state = state.STOP;
    }

    public ShooterState getState() {
        return state;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }
}
