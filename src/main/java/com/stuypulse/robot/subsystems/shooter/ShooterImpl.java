package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;

public class ShooterImpl extends Shooter {

    private final SparkMax motor;

    public ShooterImpl() {
        super();
        this.motor = new SparkMax(Ports.Shooter.MOTOR, MotorType.kBrushless);
    }
}
