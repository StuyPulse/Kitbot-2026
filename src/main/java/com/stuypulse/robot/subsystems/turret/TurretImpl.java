package com.stuypulse.robot.subsystems.turret;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HubUtil.FERRY_TARGET_POSITIONS;
import com.stuypulse.robot.util.HubUtil;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretImpl extends Turret {
    private TalonFX turretMotor;

    private CANcoder encoder1;
    private CANcoder encoder2;
    private boolean hasUsedAbsoluteEncoder;
    private FERRY_TARGET_POSITIONS targetPosition;
    private Optional<Double> voltageOverride;

    public TurretImpl() {
        turretMotor = new TalonFX(Ports.Turret.TURRET_MOTOR, "swerve");

        Motors.Turret.MOTOR_CONFIG.configure(turretMotor);

        encoder1 = new CANcoder(Ports.Turret.ENCODER_18t, "swerve");
        encoder1.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withAbsoluteSensorDiscontinuityPoint(1)));

        encoder2 = new CANcoder(Ports.Turret.ENCODER_17t, "swerve");
        encoder2.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withAbsoluteSensorDiscontinuityPoint(1)));

        hasUsedAbsoluteEncoder = false;
        voltageOverride = Optional.empty();
        targetPosition = FERRY_TARGET_POSITIONS.LEFT_WALL;
        // just default to this ?
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
        Vector2D robot = new Vector2D(CommandSwerveDrivetrain.getInstance().getPose().getTranslation());
        Vector2D robotToHub = robot
                .sub(new Vector2D(targetPosition.getFerryTargetPose().getTranslation())).normalize();
        Vector2D zeroVector = new Vector2D(0.0, 1.0);
        // define this as a constant somewhere?

        Rotation2d angle = Rotation2d
                .fromDegrees(Math.acos(robotToHub.dot(zeroVector) / robotToHub.magnitude() * zeroVector.magnitude()));
        return angle;
    }

    // confirm that the angle range is [0, 360)
    @Override
    public boolean atTargetAngle() {
        return Math.abs(getTurretAngle().minus(getTargetAngle()).getDegrees() + 180.0) < Settings.Turret.TOLERANCE_DEG;
    }

    @Override
    public Rotation2d getTurretAngle() {
        return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    }

    private Rotation2d getEncoderPos18t() {
        return Rotation2d.fromRotations((encoder1.getAbsolutePosition().getValueAsDouble()));
    }

    private Rotation2d getEncoderPos17t() {
        return Rotation2d.fromRotations((encoder2.getAbsolutePosition().getValueAsDouble()));
    }

    public Rotation2d getAbsoluteTurretAngle() {
        long gcdaandb[] = { 0, -1, 1 };

        Rotation2d encoder17tPosition = getEncoderPos17t();
        double numberOfGearTeethRotated17 = (encoder17tPosition.getRotations()
                * (double) Constants.Turret.encoderTeeth17);

        Rotation2d encoder18tPosition = getEncoderPos18t();
        double numberOfGearTeethRotated18 = (encoder18tPosition.getRotations()
                * (double) Constants.Turret.encoderTeeth18);

        double crt_Partial17 = numberOfGearTeethRotated17 * gcdaandb[2] * Constants.Turret.encoderTeeth18;
        double crt_Partial18 = numberOfGearTeethRotated18 * gcdaandb[1] * Constants.Turret.encoderTeeth17;

        // Java's % operator is not actually the same as the modulo operator
        double crt_pos = (crt_Partial17 + crt_Partial18)
                % (Constants.Turret.encoderTeeth17 * Constants.Turret.encoderTeeth18);

        crt_pos = (crt_pos < 0) ? (crt_pos + Constants.Turret.encoderTeeth17 * Constants.Turret.encoderTeeth18)
                : crt_pos;

        double turretAngle = (crt_pos / (double) Constants.Turret.bigGearTeeth) * 360;

        SmartDashboard.putNumber("Turret/CRT partial17", crt_Partial17);
        SmartDashboard.putNumber("Turret/CRT partial18", crt_Partial18);
        SmartDashboard.putNumber("Turret/CRT teeth17", numberOfGearTeethRotated17);
        SmartDashboard.putNumber("Turret/CRT teeth18", numberOfGearTeethRotated18);
        SmartDashboard.putNumber("Turret/CRT crt_pos", crt_pos);

        return Rotation2d.fromDegrees(turretAngle);
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> getTurretAngle().getRotations(),
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

        if (!hasUsedAbsoluteEncoder || getTurretAngle().getRotations() > 1.0 || getTurretAngle().getRotations() < 0.0) {
            turretMotor.setPosition((getAbsoluteTurretAngle().getDegrees() % 360.0) / 360.0);
            hasUsedAbsoluteEncoder = true;
            System.out.println("Absolute Encoder Reset triggered");
        }

        if (Settings.EnabledSubsystems.TURRET.get()) {
            if (voltageOverride.isPresent()) {
                turretMotor.setControl(new MotionMagicVoltage(getTargetAngle().getRotations()));
            } else {
                turretMotor.setVoltage(voltageOverride.orElse(0.0));
            }
        } else {
            turretMotor.setVoltage(0);
        }

        SmartDashboard.putNumber("Turret/Pos 17t", getEncoderPos17t().getDegrees());
        SmartDashboard.putNumber("Turret/Pos 18t", getEncoderPos18t().getDegrees());
        SmartDashboard.putNumber("Turret/Absolute Angle", getAbsoluteTurretAngle().getDegrees());
        SmartDashboard.putString("Turret/State", getTurretState().toString());
        SmartDashboard.putNumber("Turret/Relative Encoder", getTurretAngle().getDegrees());

        SmartDashboard.putNumber("Turret/Position", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Voltage", turretMotor.getMotorVoltage().getValueAsDouble());
    }
}
