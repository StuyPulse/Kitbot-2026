package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Superstructure extends SubsystemBase {
    public static final Superstructure instance;

    static {
        instance = new SuperstructureImpl();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public enum SuperstructureState {
        //TODO: implamentation of interpolation for these states
        OUTTAKING(Settings.Superstructure.Intake_Shooter.OUTTAKE_SPEED, Settings.Superstructure.Indexer.OUTTAKE_SPEED),
        INTAKING(Settings.Superstructure.Intake_Shooter.INTAKE_SPEED, Settings.Superstructure.Indexer.INTAKE_SPEED),
        PREPARING(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED_RPM, 0.0),
        SHOOTING(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED_RPM, Settings.Superstructure.Indexer.OUTTAKE_SPEED),
        STOP(0.0, 0.0);

        private double shooter_speed;
        private double indexer_speed;

        private SuperstructureState(double shooter_speed, double indexer_speed) {
            this.shooter_speed = shooter_speed;
            this.indexer_speed = indexer_speed;
        }

        public double getMainWheelsTargetSpeed() {
            return this.shooter_speed;
        }

        public double getIndexerTargetSpeed() {
            return this.indexer_speed;
        }
    }

    private SuperstructureState state;

    protected Superstructure() {
        this.state = SuperstructureState.INTAKING;
    }

    public SuperstructureState getState() {
        return state;
    }

    public void setState(SuperstructureState state) {
        this.state = state;
    }

    public abstract boolean atTargetVelocity();
    public abstract double getIntakeShooterMotorRPM();

    @Override
    public void periodic() {
        SmartDashboard.putString("SuperStructure/State", state.name());
        SmartDashboard.putString("States/SuperStructure", state.name());
    }
}
