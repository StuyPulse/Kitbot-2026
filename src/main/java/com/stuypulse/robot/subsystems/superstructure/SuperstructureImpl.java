package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureImpl extends Superstructure {
    private SparkMax indexerMotor;
    private TalonFX intakeMotor;

    public SuperstructureImpl() {
        super();
        indexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR_PORT, MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(config.inverted(true));
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeMotor = new TalonFX(Ports.Superstructure.INTAKE_MOTOR_PORT);
        MotorOutputConfigs shooterConfig = new MotorOutputConfigs();
        shooterConfig.withInverted(InvertedValue.Clockwise_Positive);
        intakeMotor.getConfigurator().apply(shooterConfig);
    }

    public void setVelocities(){
        indexerMotor.set(state.getIndexerSpeed());
        intakeMotor.set(state.getIntakeShooterSpeed());
    }
    
    public double getShooterVelocity(){
        return intakeMotor.get();
    }

    @Override
    public void periodic() {
    }

}
