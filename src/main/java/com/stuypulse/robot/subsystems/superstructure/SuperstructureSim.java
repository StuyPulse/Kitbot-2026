package com.stuypulse.robot.subsystems.superstructure;
import com.stuypulse.stuylib.network.SmartNumber;


public class SuperstructureSim extends Superstructure {
    private final SmartNumber shooterIntake;
    private final SmartNumber indexer;

    protected SuperstructureSim() {
        super();
        this.shooterIntake = new SmartNumber("Shooter/Speed", 0);
        this.indexer = new SmartNumber("Index/Motor", 0);
    }

    public void setMotors(){
        shooterIntake.set(this.getState().getShooterState());
        indexer.set(this.getState().getIndexerState());
    }

    @Override
    public void periodic() {
        setMotors();
    }
}