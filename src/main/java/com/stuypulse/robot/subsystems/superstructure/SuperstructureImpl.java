package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class SuperstructureImpl extends Superstructure {

    private final TalonFX intakeShooterMotor;
    private final SparkMax indexerMotor;

    protected SuperstructureImpl() {
        super();
        intakeShooterMotor = new TalonFX(Ports.Superstructure.intakeShooterMotor);
        indexerMotor = new SparkMax(Ports.Superstructure.indexerMotor, MotorType.kBrushed);

        Motors.Superstructure.INTAKE_SHOOTER_MOTOR_CONFIG.configure(intakeShooterMotor);

        // set with actual motor values
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        indexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // double check 
        
    }

    public void setMotors() {
        intakeShooterMotor.set(this.getState().getShooterState());
        indexerMotor.set(this.getState().getIndexerState());
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Settings.EnabledSubsystems.SUPERSTRUCTURE.get()) {
            intakeShooterMotor.set(this.getState().getShooterState());
            indexerMotor.set(this.getState().getIndexerState());
        } else {
            intakeShooterMotor.set(0);
            indexerMotor.set(0);
        }

    }
}
