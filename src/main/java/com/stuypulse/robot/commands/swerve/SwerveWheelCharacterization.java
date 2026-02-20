package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveWheelCharacterization extends Command {
    private final CommandSwerveDrivetrain swerve;
    private double wheelInitial;
    private double gyroInitial;
    private double driveRadius;
    private double gyroDelta;
    private double wheelDelta;

    public SwerveWheelCharacterization() { 
        swerve = CommandSwerveDrivetrain.getInstance();
        gyroInitial = swerve.getPigeon2().getRotation2d().getRadians();
        wheelInitial = swerve.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
        driveRadius = Math.sqrt(17.15 * 17.15 + 19.7 * 19.7); // Got from TunerConstants, field is private. 
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        gyroInitial = swerve.getPigeon2().getRotation2d().getRadians();
        wheelInitial = swerve.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
    }

    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(2.0)
        );
        
        gyroDelta = swerve.getPigeon2().getRotation2d().getRadians() - gyroInitial;
    }


}

