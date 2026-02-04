package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ShotCalculator;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveMovmentAlignToHub extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Superstructure superstructure;
    private final Gamepad gamepad;
    private ChassisSpeeds prevfieldRelRobotSpeeds;
    private ChassisSpeeds fieldRelRobotSpeeds;
    private VStream speed;
    private FieldObject2d virtualHub;

    private final AngleController controller;
    private final IStream angleVelocity;

    public SwerveDriveMovmentAlignToHub(Gamepad gamepad) {
        this.gamepad = gamepad;
        superstructure = Superstructure.getInstance();
        prevfieldRelRobotSpeeds = new ChassisSpeeds();
        fieldRelRobotSpeeds = new ChassisSpeeds();
        swerve = CommandSwerveDrivetrain.getInstance();

        virtualHub = Field.FIELD2D.getObject("virtual hub");

        controller = new AnglePIDController(Gains.Swerve.Motion.THETA.kP, Gains.Swerve.Motion.THETA.kI,
                Gains.Swerve.Motion.THETA.kD)
                .setOutputFilter(x -> -x);

        AngleVelocity derivative = new AngleVelocity();

        angleVelocity = IStream.create(() -> controller.getOutput())
                .filtered(
                        new LowPassFilter(Assist.ANGLE_DERIV_RC),
                        // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
                        // so that angular velocity doesn't oscillate
                        x -> x * Math.min(1, getDistanceToTarget() / Assist.REDUCED_FF_DIST),
                        // new RateLimit(Settings.Swerve.MAX_ANGULAR_ACCEL),
                        x -> SLMath.clamp(x, -Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S,
                                Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S),
                        x -> -x);

        speed = VStream.create(this::getDriverInputAsVelocity)
        .filtered(
            new VDeadZone(Drive.DEADBAND),
            x -> x.clamp(1),
            x -> x.pow(Drive.POWER.get()),
            x -> x.mul(Swerve.Constraints.MAX_VELOCITY_M_PER_S),
            new VRateLimit(Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED),
            new VLowPassFilter(Drive.RC)
        );

        addRequirements(swerve);
    }

    private Rotation2d getTargetAngle() {
        Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getPose();
        prevfieldRelRobotSpeeds = fieldRelRobotSpeeds;
        fieldRelRobotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(),
                currentPose.getRotation());
        Translation2d speakerPose = ShotCalculator
                .solveShootOnTheFly(new Pose3d(currentPose),
                        (Robot.isBlue()) ? new Pose3d(Field.hubCenter)
                                : new Pose3d(Field.transformToOppositeAlliance(Field.hubCenter)),
                        prevfieldRelRobotSpeeds, fieldRelRobotSpeeds, superstructure.getState().getMainWheelsTargetSpeed() / 60.0, 5, 0.01)
                .estimateTargetPose().getTranslation().toTranslation2d();
        virtualHub.setPose(new Pose2d(speakerPose, new Rotation2d()));
        return currentPose.getTranslation().minus(speakerPose).getAngle();
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
        return false;
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(gamepad.getLeftStick().y, -gamepad.getLeftStick().x);
    }

    @Override
    public void execute() {
        controller.update(Angle.fromRotation2d(getTargetAngle()), Angle.fromRotation2d(swerve.getPose().getRotation()));
        SmartDashboard.putNumber("Swerve/Movment Align/ angle", getAngleError());
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(speed.get().x)
            .withVelocityY(speed.get().y)
            .withRotationalRate(-angleVelocity.getAsDouble())
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Vector2D(new Translation2d()), 0);
    }
}