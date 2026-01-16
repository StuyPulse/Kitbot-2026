package com.stuypulse.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Motors;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class SuperstructureImpl extends Superstructure {
    private final TalonFX shooterIntakeMotor;
    private final SparkMax indexerMotor;
    

    public SuperstructureImpl(){
        super();
        this.shooterIntakeMotor = new TalonFX(Ports.Superstructure.SHOOTERINTAKE_MOTOR, Settings.KRAKEN_NAME);
        Motors.Superstructure.SHOOTER_MOTOR_CONFIG.configure(shooterIntakeMotor);
        this.indexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushless);
        SparkBaseConfig indexerMotorConfig = new SparkMaxConfig().inverted(Settings.Superstructure.INDEXER_INVERTED).idleMode(IdleMode.kBrake);
        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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