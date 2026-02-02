package com.stuypulse.robot.subsystems.turret;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.TurretVisualizer;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Turret extends SubsystemBase {
    public static Turret instance;
    public TurretState state;

    static {
        if (Robot.isReal()) {
            instance = new TurretImpl();
        } else {
            instance = new TurretSim();
        }
    }

    public static Turret getInstance() {
        return instance;
    }

    public Turret() {
        state = TurretState.ZERO;
    }

    public enum TurretState {
        ZERO,
        FERRYING,
        POINT_AT_HUB,
        STOP;
    }

    public Rotation2d getTargetAngle() {
        return switch (getTurretState()) {
            case ZERO -> new Rotation2d(); 
            case FERRYING -> getFerryAngle();
            case POINT_AT_HUB -> getPointAtHubAngle();
            case STOP -> getAngle();
        };
    }

    public void setTurretState(TurretState targetState) {
        state = targetState;
    }

    public TurretState getTurretState() {
        return state;
    }

    public boolean atTargetAngle() {
        return Math.abs(getAngle().minus(getTargetAngle()).getDegrees()) < Settings.Turret.TOLERANCE_DEG;
    }

    public Rotation2d getPointAtHubAngle() {
        return getPointAtTargetAngle(Field.getAllianceHubPose());
    }

    public Rotation2d getFerryAngle() {
        Pose2d robot = CommandSwerveDrivetrain.getInstance().getPose();
        return getPointAtTargetAngle(Field.getFerryZonePose(robot.getTranslation()));
    }

    public abstract Rotation2d getAngle();

    public abstract boolean exceedsOneRotation();

    public abstract SysIdRoutine getSysIdRoutine();
    

    @Override
    public void periodic() {
        SmartDashboard.putString("Turret/State", getTurretState().name());
        SmartDashboard.putString("States/Turret", getTurretState().name());
        
        if (Settings.DEBUG_MODE) {
            if (Settings.EnabledSubsystems.TURRET.get()) {
                TurretVisualizer.getInstance().updateTurretAngle(getAngle(), atTargetAngle());
            }
            else {
                TurretVisualizer.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }

    public Rotation2d getPointAtTargetAngle(Pose2d targetPose) {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Translation2d robotToTurret = Constants.Turret.robotToTurret.getTranslation();
        Vector2D robot = new Vector2D(robotPose.getTranslation());

        Vector2D target = new Vector2D(targetPose.getX(), targetPose.getY());
        Vector2D robotToTarget = target.sub(robot);
        Vector2D turretToTarget = robotToTarget.add(new Vector2D(robotToTurret));
        Vector2D zeroVector = new Vector2D(robotPose.getRotation().getCos(), robotPose.getRotation().getSin());

        // https://www.youtube.com/watch?v=_VuZZ9_58Wg
        double crossProduct = zeroVector.x * robotToTarget.y - zeroVector.y * robotToTarget.x;
        double dotProduct = zeroVector.dot(robotToTarget);

        SmartDashboard.putNumber("Turret/Turret to Target Vector X", robotToTarget.x);
        SmartDashboard.putNumber("Turret/Turret to Target Vector Y", robotToTarget.y);
        SmartDashboard.putNumber("Turret/Target Pose X", targetPose.getX());
        SmartDashboard.putNumber("Turret/Target Pose Y", targetPose.getY());
        SmartDashboard.putNumber("Turret/Zero Vector X", zeroVector.x);
        SmartDashboard.putNumber("Turret/Zero Vector Y", zeroVector.y);

        Rotation2d targetAngle = Rotation2d.fromRadians(-Math.atan2(crossProduct, dotProduct));

        // TODO: need to test logic on blue alliance
        // if (!Robot.isBlue()) {
        //     angleRadians += Math.PI; // Flip target angle for red alliance

        //     while (angleRadians < 0) angleRadians += 2 * Math.PI;
        //     while (angleRadians >= 2 * Math.PI) angleRadians -= 2 * Math.PI;
        // }

        return targetAngle;
    }
}
