package com.stuypulse.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;

public class SupertructureImpl extends Superstructure {
    
    private final SparkMax IntakeShootMotor;
    private final SparkMax IndexerMotor;
    
    public SupertructureImpl() {
        super();
        this.IntakeShootMotor = new SparkMax(Ports.Superstructure.INTAKE_SHOOTER_MOTOR, MotorType.kBrushless);
        this.IndexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
    }
}
