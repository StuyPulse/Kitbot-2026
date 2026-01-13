package com.stuypulse.robot.subsystems.superstructure;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class SupertructureImpl extends Superstructure {
    
    private final SparkMax IntakeShootMotor;
    private final SparkMax IndexerMotor;
    
    public SupertructureImpl() {
        super();
        this.IntakeShootMotor = new SparkMax(Ports.Superstructure.INTAKE_SHOOTER_MOTOR, MotorType.kBrushless);
        SparkBaseConfig intakeShootMotorConfig = new SparkMaxConfig().inverted(Settings.Superstructure.intakeShooterInverted).idleMode(IdleMode.kBrake);
        IntakeShootMotor.configure(intakeShootMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.IndexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        SparkBaseConfig indexerMotorConfig = new SparkMaxConfig().inverted(Settings.Superstructure.indexerInverted).idleMode(IdleMode.kBrake);
        IndexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setMotorsBasedOnState() {
        IntakeShootMotor.set(state.getMainWheelsSpeed());
        IndexerMotor.set(state.getIndexerSpeed());
    }

    @Override
    public void periodic() {
        setMotorsBasedOnState();
    }
}
