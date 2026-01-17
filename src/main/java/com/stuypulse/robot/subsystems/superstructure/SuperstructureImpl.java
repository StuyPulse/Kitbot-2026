package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureImpl extends Superstructure {
    private SparkMax indexerMotor;
    private TalonFX intakeMotor;

    public SuperstructureImpl() {
        super();
        indexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR_PORT, MotorType.kBrushed);

        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.apply(indexerConfig.inverted(true)); // Place, Holder
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        intakeMotor = new TalonFX(Ports.Superstructure.INTAKE_MOTOR_PORT);
        
        TalonFXConfig intakeConfig = Motors.Superstructure.MOTOR_CONFIG;
        intakeConfig.configure(intakeMotor);
        
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


        setVelocities();
    }

}
