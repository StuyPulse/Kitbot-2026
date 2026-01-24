package com.stuypulse.robot.subsystems.turret;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.TurretVisualizer;

import edu.wpi.first.math.geometry.Rotation2d;
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
        POINT_AT_HUB;
    }

    public Rotation2d getTargetAngle() {
        return switch (getTurretState()) {
            case ZERO -> new Rotation2d(); 
            case FERRYING -> getFerryAngle();
            case POINT_AT_HUB -> getPointAtHubAngle();
        };
    }

    public void setTurretState(TurretState targetState) {
        state = targetState;
    }

    public TurretState getTurretState() {
        return state;
    }

    public abstract Rotation2d getTurretAngle();

    public abstract boolean atTargetAngle();

    public abstract Rotation2d getPointAtHubAngle();

    public abstract Rotation2d getFerryAngle();

    public abstract SysIdRoutine getSysIdRoutine();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Angle (Deg)", getTurretAngle().getDegrees());

        if (Settings.DEBUG_MODE) {
            if (Settings.EnabledSubsystems.TURRET.get()) {
                TurretVisualizer.getInstance().updateTurretAngle(getTurretAngle(), atTargetAngle());
            }
            else {
                TurretVisualizer.getInstance().updateTurretAngle(new Rotation2d(), false);
            }
        }
    }
}
