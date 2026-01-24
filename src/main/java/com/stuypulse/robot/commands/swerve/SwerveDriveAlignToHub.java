package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.util.AngleVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveAlignToHub extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric drive;

    private final AngleController controller;
    private final IStream angleVelocity;

    public SwerveDriveAlignToHub() {
        swerve = CommandSwerveDrivetrain.getInstance();
        
        drive = new SwerveRequest.FieldCentric()
            .withDeadband(Settings.Swerve.Constraints.MAX_VELOCITY_M_PER_S * Settings.Driver.Drive.DEADBAND.get())
            .withRotationalDeadband(Settings.Swerve.Constraints.MAX_ANGULAR_ACCEL_RAD_PER_S * Settings.Driver.Turn.DEADBAND.get())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

        controller = new AnglePIDController(Gains.Swerve.Motion.THETA.kP, Gains.Swerve.Motion.THETA.kI, Gains.Swerve.Motion.THETA.kD)
            .setOutputFilter(x -> -x);

        AngleVelocity derivative = new AngleVelocity();

        angleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(getTargetAngle())))
            .filtered(
                new LowPassFilter(Assist.ANGLE_DERIV_RC),
                // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
                // so that angular velocity doesn't oscillate
                x -> x * Math.min(1, getDistanceToTarget() / Assist.REDUCED_FF_DIST),
                // new RateLimit(Settings.Swerve.MAX_ANGULAR_ACCEL),
                x -> SLMath.clamp(x, -Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S, Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S),
                x -> -x
            );
        
        addRequirements(swerve);
    }

    private Rotation2d getTargetAngle() {
        Translation2d currentPose = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceHubPose().getTranslation();
        return currentPose.minus(speakerPose).getAngle();
    }

    private double getDistanceToTarget() {
        Translation2d currentPose = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceHubPose().getTranslation();
        return currentPose.getDistance(speakerPose);
    }

    protected double getAngleError() {
        return controller.getError().getRotation2d().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return controller.isDoneDegrees(Alignment.Tolerances.THETA_TOLERANCE.get());
    }

    @Override
    public void execute() {
        swerve.drive(new Vector2D(new Translation2d()), SLMath.clamp(angleVelocity.get() 
                                                        + controller.update(
                                                            Angle.fromRotation2d(getTargetAngle()), 
                                                            Angle.fromRotation2d(swerve.getPose().getRotation())),
                                                        -Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S,
                                                        Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S
                                                        )
                                                        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Vector2D(new Translation2d()), 0);
    }
}
