package com.stuypulse.robot.subsystems.turret;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.TurretVisualizer;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public abstract class Turret extends SubsystemBase {
    private static final Turret instance;
    private TurretState state;

    static {
        instance = Robot.isReal() ? new TurretImpl() : new TurretSim();
    }

    public static Turret getInstance() {
        return instance;
    }

    public Turret() {
        state = TurretState.IDLE;
    }

    public enum TurretState {
        IDLE,
        SHOOTING,
        FERRYING;
    }

    public Rotation2d getTargetAngle() {
        return switch (getState()) {
            case IDLE -> getAngle(); 
            case FERRYING -> getFerryAngle();
            case SHOOTING -> getScoringAngle();
        };
    }

    public boolean atTargetAngle() {
        return Math.abs(getAngle().minus(getTargetAngle()).getDegrees()) < Settings.Turret.TOLERANCE_DEG;
    }

    public Rotation2d getScoringAngle() {
        return getPointAtTargetAngle(Field.getAllianceHubPose());
    }

    public Rotation2d getFerryAngle() {
        Pose2d robot = CommandSwerveDrivetrain.getInstance().getPose();
        return getPointAtTargetAngle(Field.getFerryZonePose(robot.getTranslation()));
    }

    public abstract Rotation2d getAngle();

    public abstract SysIdRoutine getSysIdRoutine();
   
    public void setState(TurretState state) {
        this.state = state;
    }

    public TurretState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Turret/State", state.name());
        SmartDashboard.putString("States/Turret", state.name());

        if (Settings.DEBUG_MODE) {
            if (Settings.EnabledSubsystems.TURRET.get()) {
                TurretVisualizer.getInstance().updateTurretAngle(getAngle().plus((Robot.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg)), atTargetAngle());
            }
            else {
                TurretVisualizer.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }

    // Should match implementation on mini turret
    // Current logic is as of 1/31
    public Rotation2d getPointAtTargetAngle(Pose2d targetPose) {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Vector2D robot = new Vector2D(robotPose.getTranslation());

        Vector2D target = new Vector2D(targetPose.getX(), targetPose.getY());
        Vector2D robotToTarget = target.sub(robot);
        Vector2D zeroVector = new Vector2D(robotPose.getRotation().getCos(), robotPose.getRotation().getSin());

        // https://www.youtube.com/watch?v=_VuZZ9_58Wg
        double crossProduct = zeroVector.x * robotToTarget.y - zeroVector.y * robotToTarget.x;
        double dotProduct = zeroVector.dot(robotToTarget);

        SmartDashboard.putNumber("Turret/Robot to Target Vector X", robotToTarget.x);
        SmartDashboard.putNumber("Turret/Robot to Target Vector Y", robotToTarget.y);
        SmartDashboard.putNumber("Turret/Target Pose X", targetPose.getX());
        SmartDashboard.putNumber("Turret/Target Pose Y", targetPose.getY());
        SmartDashboard.putNumber("Turret/Zero Vector X", zeroVector.x);
        SmartDashboard.putNumber("Turret/Zero Vector Y", zeroVector.y);

        Rotation2d targetAngle = (Robot.isReal() ?
            Rotation2d.fromRadians(-Math.atan2(crossProduct, dotProduct)) :
            Rotation2d.fromRadians(Math.atan2(crossProduct, dotProduct)));

        return targetAngle;
    }
}