
package com.stuypulse.robot.subsystems.turret;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HubUtil;
import com.stuypulse.robot.util.HubUtil.FERRY_TARGET_POSITIONS;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretImpl extends Turret {
    private TalonFX turretMotor;

    private CANcoder encoder1;
    private CANcoder encoder2;
    private CANBus canbus;
    private boolean hasUsedAbsoluteEncoder;
    private boolean exceededOneRotation;
   // private FERRY_TARGET_POSITIONS targetPosition;
    private Optional<Double> voltageOverride;

    public TurretImpl() {
        CANBus canbus = Settings.CANIVORE; 
        turretMotor = new TalonFX(Ports.Turret.TURRET_MOTOR, canbus);
        turretMotor.setPosition(0.0);

        Motors.Turret.MOTOR_CONFIG.configure(turretMotor);

        // encoder1 = new CANcoder(Ports.Turret.ENCODER_18t, canbus);
        // encoder1.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
        //         new MagnetSensorConfigs()
        //                 .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        //                 .withMagnetOffset(-167/360.0)
        //                 .withAbsoluteSensorDiscontinuityPoint(1)));

        // encoder2 = new CANcoder(Ports.Turret.ENCODER_17t, canbus);
        // encoder2.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
        //         new MagnetSensorConfigs()
        //                 .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        //                 .withMagnetOffset(
        //                 160/360.0)
        //                 .withAbsoluteSensorDiscontinuityPoint(1)));

        hasUsedAbsoluteEncoder = false;
        voltageOverride = Optional.empty();
        //targetPosition = FERRY_TARGET_POSITIONS.LEFT_WALL;
        // just default to this ?
    }


    public Rotation2d getPointAtHubAngle() {
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Vector2D hub = new Vector2D(HubUtil.getAllianceHubPose().getTranslation());
        Vector2D robot = new Vector2D(robotPose.getTranslation());

        Vector2D target = new Vector2D(hub.x, hub.y);
        Vector2D robotToTarget = target.sub(robot);
        Vector2D zeroVector = new Vector2D(robotPose.getRotation().getCos(), robotPose.getRotation().getSin());

        SmartDashboard.putNumber("Turret/Robot to Hub X", robotToTarget.x);
        SmartDashboard.putNumber("Turret/Robot to Hub Y", robotToTarget.y);

        SmartDashboard.putNumber("Turret/Zero Vector X", zeroVector.x);
        SmartDashboard.putNumber("Turret/Zero Vector Y", zeroVector.y);

        SmartDashboard.putNumber("Turret/Hub Pose X", hub.x);
        SmartDashboard.putNumber("Turret/Hub Pose Y", hub.y);

        SmartDashboard.putNumber("Turret/Robot Pose X", robotPose.getX());
        SmartDashboard.putNumber("Turret/Robot Pose Y", robotPose.getY());

        // need to normalize this when we do the cross product with the unit vector so cos theta is not scaled! 
        // Vector2D zeroVector = new Vector2D(0.0, 1.0);

        // https://www.youtube.com/watch?v=_VuZZ9_58Wg
        double crossProduct = zeroVector.x * robotToTarget.y - zeroVector.y * robotToTarget.x;
        double dotProduct = zeroVector.dot(robotToTarget);

        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(crossProduct, dotProduct));
        return targetAngle;

        // return robotToTarget.getTranslation2d().getAngle();
    }

    @Override
    public Rotation2d getFerryAngle() {
        // Vector2D robot = new Vector2D(CommandSwerveDrivetrain.getInstance().getPose().getTranslation());
        // Vector2D robotToHub = robot
        //         .sub(new Vector2D(targetPosition.getFerryTargetPose().getTranslation())).normalize();
        // Vector2D zeroVector = new Vector2D(0.0, 1.0);
        // // define this as a constant somewhere?

        // Rotation2d angle = Rotation2d
        //         .fromDegrees(Math.acos(robotToHub.dot(zeroVector) / robotToHub.magnitude() * zeroVector.magnitude()));
        // return angle;
        return new Rotation2d();
    }

    @Override
    public boolean exceedsOneRotation() {
        return this.exceededOneRotation;
    }

    // confirm that the angle range is [0, 360)
    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().minus(getTargetAngle()).getDegrees() + 180.0) < Settings.Turret.TOLERANCE_DEG;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    }

    // private Rotation2d getEncoderPos18t() {
    //     return Rotation2d.fromRotations((encoder1.getAbsolutePosition().getValueAsDouble())).minus(Settings.Turret.EIGHTEEN_TEETH_GEAR_OFFSET);
    //     // need to apply offsets here
    // }

    // private Rotation2d getEncoderPos17t() {
    //     return Rotation2d.fromRotations((encoder2.getAbsolutePosition().getValueAsDouble())).minus(Settings.Turret.SEVENTEETH_TEETH_GEAR_OFFSET);  
    //     // need to apply offsets here
    // }



    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> getAngle().getRotations(),
                () -> turretMotor.getVelocity().getValueAsDouble(),
                () -> turretMotor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }

    private void setVoltageOverride(Optional<Double> volts) {
        voltageOverride = volts;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!Settings.EnabledSubsystems.TURRET.get() || getTurretState() == TurretState.STOP) {
            turretMotor.setVoltage(0);
        } else {
            turretMotor.setControl(new PositionVoltage(getTargetAngle().getRotations()));
        }
        
        // SmartDashboard.putNumber("Turret/Pos 18t", getEncoderPos18t().getDegrees());
        // SmartDashboard.putNumber("Turret/Absolute Angle", getAbsoluteTurretAngle().getDegrees());
        SmartDashboard.putString("Turret/State", getTurretState().toString());
        SmartDashboard.putNumber("Turret/Relative Encoder", getAngle().getDegrees());
        SmartDashboard.putBoolean("Turret/Exceeded Rotation", exceededOneRotation);
        SmartDashboard.putNumber("Turret/Position", turretMotor.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("Turret/Voltage", turretMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Turret/Get Hub Target Angke", getPointAtHubAngle().getDegrees());
    }
}
