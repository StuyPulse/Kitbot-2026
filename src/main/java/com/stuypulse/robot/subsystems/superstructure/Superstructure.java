package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Superstructure extends SubsystemBase {

    private static final Superstructure instance;

    static {
        instance = new SuperstructureImpl();
    }

    public static Superstructure getInstance() {
        return instance;
    }

    public enum MainWheelState {
        OUTTAKING(-1.0),
        INTAKING(1.0),
        PREPARING(0.5),
        SHOOTING(3.0),
        STOP(0.0);

        private final double shooterSpeed;

        private MainWheelState(double shooterSpeed) {
            this.shooterSpeed = shooterSpeed;
        }

        public double getMainWheelsTargetSpeed() {
            return shooterSpeed;
        }
    }

    public enum IndexerState {
        OUTTAKING(-1.0),
        INTAKING(1.0),
        PREPARING(0.5),
        SHOOTING(1.0),
        STOP(0.0);

        private final double indexerSpeed;

        private IndexerState(double indexerSpeed) {
            this.indexerSpeed = indexerSpeed;
        }

        public double getIndexerTargetSpeed() {
            return indexerSpeed;
        }
    }

    private MainWheelState mainWheelState;
    private IndexerState indexerState;

    public Superstructure() {
        mainWheelState = MainWheelState.STOP;
        indexerState = IndexerState.STOP;
    }

    public void setMainWheelState(MainWheelState state) {
        this.mainWheelState = state;
    }

    public MainWheelState getMainWheelState() {
        return mainWheelState;
    }

    public void setIndexerState(IndexerState state) {
        this.indexerState = state;
    }

    public IndexerState getIndexerState() {
        return indexerState;
    }

    public abstract boolean atTargetVelocity();

    @Override
    public void periodic() {}
}