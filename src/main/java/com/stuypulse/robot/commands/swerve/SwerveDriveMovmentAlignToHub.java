package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.InterpolationUtil;
import com.stuypulse.robot.util.ShotCalculator;
import com.stuypulse.robot.util.ShotCalculator.AlignAngleSolution;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.controller.PIDController;
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
    private FieldObject2d fieldVirtualHub;
    private FieldObject2d hub;
    private AlignAngleSolution virtualhub;
    private Pose2d currentPose;
    private StopWatch mTimer;
    private PIDController anglePid;
    // private Angle mLastError;
    // private IFilter mDFilter;

    private final AngleController controller;

    public SwerveDriveMovmentAlignToHub(Gamepad gamepad) {

        anglePid = new PIDController(Gains.Swerve.Alignment.THETA.kP, Gains.Swerve.Alignment.THETA.kI, Gains.Swerve.Alignment.THETA.kD);
        anglePid.setTolerance(Settings.Swerve.Constraints.angletoleranceDegrees);
        // anglePid.enableContinuousInput(); TODO: implament
        this.gamepad = gamepad;
        virtualhub = new AlignAngleSolution(new Rotation2d(), new Rotation2d(), new Pose3d());
        superstructure = Superstructure.getInstance();
        mTimer = new StopWatch();
        // mLastError = Angle.kZero;
        // mDFilter = IFilter.create(x -> x);

        prevfieldRelRobotSpeeds = new ChassisSpeeds();
        fieldRelRobotSpeeds = new ChassisSpeeds();

        swerve = CommandSwerveDrivetrain.getInstance();
        currentPose = swerve.getPose();

        fieldVirtualHub = Field.FIELD2D.getObject("virtual hub");
        hub = Field.FIELD2D.getObject("Hub");

        controller = new AnglePIDController(Gains.Swerve.Motion.THETA.kP, Gains.Swerve.Motion.THETA.kI,
                Gains.Swerve.Motion.THETA.kD)
                .setOutputFilter(x -> -x);
        
        // AngleVelocity derivative = new AngleVelocity();

        // angleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(getSolution().requiredYaw())))
        //     .filtered(
        //         new LowPassFilter(Assist.ANGLE_DERIV_RC),
        //         // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
        //         // so that angular velocity doesn't oscillate
        //         x -> x * Math.min(1, getDistanceToTarget(getSolution().estimateTargetPose().getTranslation().toTranslation2d()) / Assist.REDUCED_FF_DIST),
        //         // new RateLimit(Settings.Swerve.MAX_ANGULAR_ACCEL),
        //         x -> SLMath.clamp(x, -Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S, Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S),
        //         x -> -x
        //     );

        speed = VStream.create(this::getDriverInputAsVelocity)
        .filtered(
            new VDeadZone(Drive.DEADBAND),
            x -> x.clamp(1),
            x -> x.pow(Drive.POWER),
            x -> x.mul(Swerve.Constraints.MAX_VELOCITY_M_PER_S),
            new VRateLimit(Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED),
            new VLowPassFilter(Drive.RC)
        );

        addRequirements(swerve);
    }

    public AlignAngleSolution getSolution() {
        hub.setPose(Field.transformToOppositeAlliance(Field.getAllianceHubPose()));
        currentPose = swerve.getPose();
        prevfieldRelRobotSpeeds = fieldRelRobotSpeeds;
        fieldRelRobotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(),
                currentPose.getRotation());
        Pigeon2 gyro = swerve.getPigeon2();
        virtualhub = ShotCalculator
                .solveShootOnTheFly(new Pose3d(currentPose),
                        Field.getAllianceHubPose3d(),
                        gyro.getAccelerationX().getValueAsDouble(), gyro.getAccelerationY().getValueAsDouble(), fieldRelRobotSpeeds, superstructure.getState().getMainWheelsTargetSpeed() / 60.0, 3, 0.1);
        // virtualhub = Field.transformToOppositeAlliance(virtualhub); 
        fieldVirtualHub.setPose(Robot.isBlue() ? virtualhub.estimateTargetPose().toPose2d() : Field.transformToOppositeAlliance(virtualhub.estimateTargetPose().toPose2d()));
        return virtualhub;
    }

    private double getDistanceToTarget(Translation2d target) {
        Translation2d currentPose = swerve.getPose().getTranslation();
        return currentPose.getDistance(target);
    }

    protected double getAngleError() {
        return controller.getError().getRotation2d().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private Vector2D getDriverInputAsVelocity() {
        return new Vector2D(-gamepad.getLeftStick().x, -gamepad.getLeftStick().y);
    }

    private Rotation2d getErrorDegrees(Rotation2d setpoint, Rotation2d current) {
        double setpointDegrees = setpoint.getDegrees();
        double currentDegrees = current.getDegrees();
        Rotation2d differance = (Math.abs(setpointDegrees - currentDegrees) > 180) ?  Rotation2d.fromDegrees(Math.abs(currentDegrees) - 180) : setpoint.minus(current);
        return differance;
    }

    @Override
    public void execute() {
        AlignAngleSolution solution = getSolution();
        swerve.setExpectedHubPose(solution.estimateTargetPose().toPose2d());
        Rotation2d targetangle = getSolution().requiredYaw();
        Rotation2d currentRotation = swerve.getPose().getRotation();
        Rotation2d error = getErrorDegrees(Rotation2d.k180deg, currentRotation);
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Swerve/Movment Align/Target angle Radians", Rotation2d.k180deg.getRadians());
            SmartDashboard.putNumber("Swerve/Movment Align/Current Angle Radians", currentRotation.getRadians());
            SmartDashboard.putNumber("Swerve/Movment Align/Controller Error", error.getDegrees());
            SmartDashboard.putNumber("Swerve/Movment Align/Distance to Target", getDistanceToTarget(solution.estimateTargetPose().toPose2d().getTranslation()));
        }

        if (getDistanceToTarget(solution.estimateTargetPose().toPose2d().getTranslation()) < Settings.Superstructure.interpolation.MAX_SHOOT_DISTANCE) {
            swerve.drive(
                speed.get(),
                SLMath.clamp(anglePid.calculate(
                    error.getDegrees(),
                    0.0
                    ),
                    -Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S,
                    Settings.Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S
                )
            );
            // Angle error = Angle.fromRotation2d(Rotation2d.k180deg).sub(Angle.fromRotation2d(currentPose.getRotation()));
            // // double dt = mTimer.reset();
            // // double derivative = mDFilter.get(error.velocityRadians(mLastError, dt));
            // // SmartDashboard.putNumber("Swerve/Movment Align/Derivative", derivative);
            // // SmartDashboard.putNumber("Swerve/Movment Align/DT", dt);
            // // mLastError = error;
        } else {
            swerve.drive(new Vector2D(new Translation2d()), 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Vector2D(new Translation2d()), 0);
        anglePid.close();
    }

    @Override
    public void initialize() {
        mTimer.reset();
        anglePid.reset();
    }

}