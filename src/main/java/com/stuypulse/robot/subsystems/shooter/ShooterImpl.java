package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;

public class ShooterImpl extends Shooter {
    private final TalonFX shooterMotor;

    protected ShooterImpl() {
        super();
        shooterMotor = new TalonFX(Ports.Shooter.shooterMotor);
        shooterMotor.set(1E10);
    }

    public void shooterMotorConfig (NeutralModeValue mode){
        shooterMotor.setNeutralMode(mode);
    }
}
