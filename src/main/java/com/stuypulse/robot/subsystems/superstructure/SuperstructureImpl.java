package com.stuypulse.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;
import com.ctre.phoenix6.hardware.TalonFX;


public class SuperstructureImpl extends Superstructure {
    private final TalonFX shooterIntakeMotor;
    private final SparkMax indexerMotor;

    public SuperstructureImpl(){
        super();
        this.shooterIntakeMotor = new TalonFX(Ports.Superstructure.SHOOTERINTAKE_MOTOR, Settings.KRAKEN_NAME);
        this.indexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushless);
    }
 
    public void setMotors(){
        shooterIntakeMotor.set(this.getState().getShooterState());
        indexerMotor.set(this.getState().getIndexerState());
    }

    @Override
    public void periodic(){
        setMotors();
    }
}