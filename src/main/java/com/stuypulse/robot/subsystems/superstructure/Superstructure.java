package com.stuypulse.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.robot.constants.Settings;

public abstract class Superstructure extends SubsystemBase {
    public static final Superstructure instance;

    static {
        instance = new SuperstructureImpl();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public enum SuperstructureState {
        SHOOTING(Settings.Superstructure.SHOOTING_INTAKE_SHOOTER_SPEED, Settings.Superstructure.SHOOTING_INDEXER_SPEED),
        INTAKING(Settings.Superstructure.INTAKING_INTAKE_SHOOTER_SPEED, Settings.Superstructure.INTAKING_INDEXER_SPEED),
        OUTTAKING(Settings.Superstructure.OUTTAKING_INTAKE_SHOOTER_SPEED, Settings.Superstructure.OUTTAKING_INDEXER_SPEED),
        IDLE(0,0);
        


        private double intakeShooterSpeed;
        private double indexerSpeed;

        private SuperstructureState(double intakeSpeed, double indexerSpeed) {
            this.intakeShooterSpeed = intakeSpeed;
            this.indexerSpeed = indexerSpeed;
        }

        public double getIntakeShooterSpeed() {
            return this.intakeShooterSpeed;
        }

        public double getIndexerSpeed() {
            return this.indexerSpeed;
        }
    }

    public SuperstructureState state;

    public SuperstructureState getState(){
        return this.state;
    }

    public void setState(SuperstructureState state) {
        this.state = state;
    }
    
    public Superstructure() {
        this.state = SuperstructureState.IDLE;
    }

    public abstract void setVelocities();
    public abstract double getShooterVelocity();
}