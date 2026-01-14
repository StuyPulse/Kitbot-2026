package com.stuypulse.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.stuypulse.robot.constants.Settings;


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
        INTAKING(Settings.Superstructure.Intake_Shooter_Speeds.INTAKE_SPEED, Settings.Superstructure.Indexer_Speeds.INTAKE_SPEED),
        OUTTAKING(Settings.Superstructure.Intake_Shooter_Speeds.OUTTAKE_SPEED, Settings.Superstructure.Indexer_Speeds.OUTTAKE_SPEED),
        PREPARE_TO_SHOOT(Settings.Superstructure.Intake_Shooter_Speeds.SHOOTING_SPEED, 0),
        SHOOTING(Settings.Superstructure.Intake_Shooter_Speeds.SHOOTING_SPEED, Settings.Superstructure.Indexer_Speeds.OUTTAKE_SPEED),
        STOP(0, 0);
                
        private Number intake_shooter_speed;
        private Number indexer_speed;

        private SuperstructureState(Number intake_shooter_speed, Number indexer_speed) {
            this.intake_shooter_speed = intake_shooter_speed;
            this.indexer_speed = indexer_speed;
        }

        public double getIntakeShooterSpeed() {
            return this.intake_shooter_speed.doubleValue();
        }

        public double getIndexerSpeed() {
            return this.indexer_speed.doubleValue();
        }
    }

    protected SuperstructureState state;

    protected Superstructure() {
        this.state = SuperstructureState.STOP;
    }

    public SuperstructureState getState() {
        return state;
    }

    public void setState(SuperstructureState state) {
        this.state = state;
    }
}
