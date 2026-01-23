package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureImpl extends Superstructure {
    
    private final TalonFX intakeShooterMotor;
    private final SparkMax indexMotor;
    
    public SuperstructureImpl() {
        super();
        
        intakeShooterMotor = new TalonFX(Ports.Superstructure.INTAKE_SHOOTER_MOTOR, "Swerve Drive Drive");
        Motors.Superstructure.intakeShooterMotorConfig.configure(intakeShooterMotor);  
        
        indexMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        indexMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    private void setMotorsBasedOnState() {
        intakeShooterMotor.setControl(new DutyCycleOut(state.getMainWheelsSpeed()));
        indexMotor.set(state.getIndexerSpeed());
    }

    @Override
    public void periodic() {
        setMotorsBasedOnState();
        shooterAtTargetVelocity = (Math.abs(intakeShooterMotor.getVelocity().getValueAsDouble() - state.getMainWheelsSpeed()) <= Settings.Superstructure.Intake_Shooter.SHOOT_TOLERANCE_RPM);
        SmartDashboard.putString("SuperStructure/State", getState().toString());
        SmartDashboard.putNumber("SuperStructure/Main Wheel/Speed", getState().getMainWheelsSpeed());
        SmartDashboard.putNumber("SuperStructure/Indexer/Speed", getState().getIndexerSpeed());
    }
}
