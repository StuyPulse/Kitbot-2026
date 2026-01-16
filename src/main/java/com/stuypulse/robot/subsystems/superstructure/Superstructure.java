package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    public static final Superstructure instance;

    static {
        if (Robot.isReal()) instance = new SuperstructureImpl(); 
        else instance = new SuperstructureSim();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public enum SuperstructureState {
        // add ShooterState constructor and potential states for the game
        STOP(0, 0),
        SHOOT(Settings.Superstructure.SHOOTER_INTAKE_SHOOT, Settings.Superstructure.SHOOTER_INTAKE_SHOOT),
        INTAKE(Settings.Superstructure.SHOOTER_INTAKE_INTAKE, Settings.Superstructure.SHOOTER_INTAKE_INTAKE),
        OUTTAKE(Settings.Superstructure.SHOOTER_INTAKE_OUTTAKE, Settings.Superstructure.SHOOTER_INTAKE_OUTTAKE);

        private double shooterState;
        private double indexerState;

        private SuperstructureState(double shooterState, double indexerState) {
            this.shooterState = shooterState;
            this.indexerState = indexerState;
        }

        public double getShooterState() {
            return this.shooterState;
        }

        public double getIndexerState() {
            return this.indexerState;
        }
    }

    private SuperstructureState state;

    protected Superstructure() {
        this.state = SuperstructureState.STOP;
    }

    public void setState() {
        this.state = SuperstructureState.STOP;
    }

    public SuperstructureState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Superstructure/State", getState().toString());
    }
}
