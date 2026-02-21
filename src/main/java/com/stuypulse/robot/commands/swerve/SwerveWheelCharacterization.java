package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveWheelCharacterization extends Command {
    private final CommandSwerveDrivetrain swerve;
    private double[] wheelInitial;
    private double gyroInitial;
    private double driveRadius;
    private double gyroDelta;
    private double wheelDelta;
    private double wheelRadius;
    private double[] wheelCurrent;

    public SwerveWheelCharacterization() { 
        swerve = CommandSwerveDrivetrain.getInstance();
        gyroInitial = swerve.getPigeon2().getRotation2d().getRadians();
        wheelInitial = swerve.getRadiusCharacterizationModulePositions();
        driveRadius = Math.sqrt(17.15 * 17.15 + 19.7 * 19.7); // Got from TunerConstants, field is private. 
        addRequirements(swerve);
    }

    // @Override
    // public void initialize() {
    //     gyroInitial = swerve.getPigeon2().getRotation2d().getRadians();
    //     wheelInitial = swerve.getRadiusCharacterizationModulePositions();
    // }

    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.5)
        );
        
        gyroDelta = swerve.getPigeon2().getRotation2d().getRadians() - gyroInitial;
        wheelCurrent = swerve.getRadiusCharacterizationModulePositions();
        for(int i = 0; i < 4; i++){
           wheelDelta += (wheelCurrent[i] - wheelInitial[i]) / 4;
        }
        wheelRadius = gyroDelta * driveRadius / wheelDelta;

        SmartDashboard.putNumber("Radius Characterization/Radius", wheelRadius);
        SmartDashboard.putNumber("Radius Characterization/Gyro Delta", gyroDelta);
        SmartDashboard.putNumber("Radius Characterization/Wheel Delta", wheelDelta);
    }

    
}

