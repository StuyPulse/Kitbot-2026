package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveWheelCharacterization extends Command {

    private static final double ROTATIONAL_RATE = 0.5; // rad/s
    private static final double DRIVE_RADIUS = Math.sqrt(17.15 * 17.15 + 19.7 * 19.7) * 0.0254; // convert cm -> meters if needed

    private final CommandSwerveDrivetrain swerve;

    private double[] wheelInitial;
    private Rotation2d lastAngle;
    private double gyroDelta;

    public SwerveWheelCharacterization() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        wheelInitial = swerve.getRadiusCharacterizationModulePositions();
        lastAngle = swerve.getPigeon2().getRotation2d();
        gyroDelta = 0.0;
    }

    @Override
    public void execute() {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(ROTATIONAL_RATE)
        );

        Rotation2d currentAngle = swerve.getPigeon2().getRotation2d();
        gyroDelta += Math.abs(currentAngle.minus(lastAngle).getRadians());
        lastAngle = currentAngle;

        double[] wheelCurrent = swerve.getRadiusCharacterizationModulePositions();
        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(wheelCurrent[i] - wheelInitial[i]) / 4.0;
        }

        double wheelRadius = (gyroDelta * DRIVE_RADIUS) / wheelDelta;

        SmartDashboard.putNumber("Radius Characterization/Radius", wheelRadius);
        SmartDashboard.putNumber("Radius Characterization/Gyro Delta", gyroDelta);
        SmartDashboard.putNumber("Radius Characterization/Wheel Delta", wheelDelta);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
        .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        double[] wheelCurrent = swerve.getRadiusCharacterizationModulePositions();
        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(wheelCurrent[i] - wheelInitial[i]) / 4.0;
        }

        double wheelRadius = (gyroDelta * DRIVE_RADIUS) / wheelDelta;

        System.out.println("********** Wheel Radius Characterization Results **********");
        System.out.printf("\tWheel Delta: %.9f radians%n", wheelDelta);
        System.out.printf("\tGyro Delta:  %.9f radians%n", gyroDelta);
        System.out.printf("\tWheel Radius: %.9f meters / %.9f inches%n", wheelRadius, wheelRadius * 39.3701);
    }

    @Override
    public boolean isFinished() {
        return false; // driver cancels manually
    }
}