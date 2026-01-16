package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.stuylib.network.SmartNumber;

public class SuperstructureSim extends Superstructure {
    private SmartNumber intakeShooterMotor;
    private SmartNumber indexerMotor;

    protected SuperstructureSim() {
        super();
        this.intakeShooterMotor = new SmartNumber("Shooter / Speed", 0);
        this.indexerMotor = new SmartNumber("Index / Motor", 0);
    }

    public void setMotors() {
        intakeShooterMotor.set(this.getState().getShooterState());
        indexerMotor.set(this.getState().getIndexerState());
    }

    @Override
    public void periodic() {
        setMotors();
    }
}
