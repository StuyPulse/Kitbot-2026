package com.stuypulse.robot.subsystems.turret;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HubUtil;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.math.Vector2D;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretSim extends Turret {

    private DCMotorSim sim;
    private final LinearSystemLoop<N2, N1, N2> controller;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private double maxAngularVelRadiansPerSecond;
    private double maxAngularAccelRadiansPerSecondSquared;

    private Optional<Double> voltageOverride;

    public TurretSim() {
        LinearSystem<N2, N1, N2> linearSystem = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.0, 2.80);

        sim = new DCMotorSim(
            linearSystem, 
            DCMotor.getKrakenX60(1)
        );
        
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<N2, N1, N2>(
            linearSystem, 
            VecBuilder.fill(0.00001, 100), 
            VecBuilder.fill(12),
            Settings.DT);

        KalmanFilter<N2, N1, N2> kalmanFilter = new KalmanFilter<>(
            Nat.N2(), 
            Nat.N2(), 
            linearSystem, 
            VecBuilder.fill(3.0, 3.0), 
            VecBuilder.fill(0.01, 0.01), 
            Settings.DT);

        controller = new LinearSystemLoop<>(linearSystem, lqr, kalmanFilter, 12.0, Settings.DT);

        maxAngularVelRadiansPerSecond = Units.degreesToRadians(200.0);
        maxAngularAccelRadiansPerSecondSquared = Units.degreesToRadians(400.0);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxAngularVelRadiansPerSecond, maxAngularAccelRadiansPerSecondSquared);
        profile = new TrapezoidProfile(constraints);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        voltageOverride = Optional.empty();
    }

    @Override
    public Rotation2d getPointAtHubAngle() {
        Vector2D robot = new Vector2D(CommandSwerveDrivetrain.getInstance().getPose().getTranslation());
        Vector2D hub = new Vector2D(HubUtil.getAllianceHubPose().getTranslation());
        Vector2D robotToHub = hub.sub(robot).normalize();
        Vector2D zeroVector = new Vector2D(0.0, 1.0);

        // https://www.youtube.com/watch?v=_VuZZ9_58Wg
        double crossProduct = zeroVector.x * robotToHub.y - zeroVector.y * robotToHub.x;
        double dotProduct = zeroVector.dot(robotToHub);

        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(crossProduct, dotProduct));

        return targetAngle;
    }

    @Override
    public Rotation2d getFerryAngle() {
        return new Rotation2d();
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getTurretAngle().minus(getTargetAngle()).getDegrees()) < Settings.Turret.TOLERANCE_DEG;
    }

    @Override
    public Rotation2d getTurretAngle() {
        return Rotation2d.fromRadians(sim.getAngularPositionRad());
    }

    private void setVoltageOverride(Optional<Double> volts) {
        voltageOverride = volts;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        goal = new TrapezoidProfile.State(getTargetAngle().getRadians(), 0.0);
        setpoint = profile.calculate(Settings.DT, setpoint, goal);

        SmartDashboard.putNumber("Turret/Constraints/Max vel (deg per s)", Units.radiansToDegrees(maxAngularVelRadiansPerSecond));
        SmartDashboard.putNumber("Turret/Constraints/Max accel (deg per s per s)", Units.radiansToDegrees(maxAngularAccelRadiansPerSecondSquared));

        SmartDashboard.putNumber("Turret/Setpoint (deg)", Units.radiansToDegrees(setpoint.position));
        SmartDashboard.putNumber("Turret/Target (deg)", Units.radiansToDegrees(getTargetAngle().getRadians()));
        SmartDashboard.putBoolean("Turret/At Target", atTargetAngle());
        SmartDashboard.putNumber("Turret/Error: abs(turret - target) (deg)", Math.abs(getTurretAngle().minus(getTargetAngle()).getDegrees()));

        controller.setNextR(VecBuilder.fill(setpoint.position, 0.0)); // try setpoint.velocity as second arg later
        controller.correct(VecBuilder.fill(sim.getAngularPositionRad(), sim.getAngularVelocityRadPerSec()));
        controller.predict(Settings.DT);

        if (Settings.EnabledSubsystems.TURRET.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInputVoltage(voltageOverride.get());
            } else {
                sim.setInputVoltage(controller.getU(0));
            }
        } else {
            sim.setInputVoltage(0);
        }

        sim.update(Settings.DT);
    }



    /* USELESS, DON'T DELETE */
    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> getTurretAngle().getRotations(),
                () -> sim.getAngularVelocityRadPerSec(),
                () -> sim.getInputVoltage(),
                getInstance());
    }
}
