
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
    private Optional<Double> voltageOverride;

    public TurretImpl() {
        CANBus canbus = Settings.CANIVORE;
        turretMotor = new TalonFX(Ports.Turret.TURRET_MOTOR, canbus);
        turretMotor.setPosition(0.0);

        Motors.Turret.MOTOR_CONFIG.configure(turretMotor);

        // encoder1 = new CANcoder(Ports.Turret.ENCODER_18t, canbus);
        // encoder1.getConfigurator().apply(new
        // CANcoderConfiguration().withMagnetSensor(
        // new MagnetSensorConfigs()
        // .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        // .withMagnetOffset(-167/360.0)
        // .withAbsoluteSensorDiscontinuityPoint(1)));

        // encoder2 = new CANcoder(Ports.Turret.ENCODER_17t, canbus);
        // encoder2.getConfigurator().apply(new
        // CANcoderConfiguration().withMagnetSensor(
        // new MagnetSensorConfigs()
        // .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        // .withMagnetOffset(
        // 160/360.0)
        // .withAbsoluteSensorDiscontinuityPoint(1)));

        hasUsedAbsoluteEncoder = false;
        voltageOverride = Optional.empty();
        // targetPosition = FERRY_TARGET_POSITIONS.LEFT_WALL;
        // just default to this ?
    }

    // @Override
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
    // return
    // Rotation2d.fromRotations((encoder1.getAbsolutePosition().getValueAsDouble())).minus(Settings.Turret.EIGHTEEN_TEETH_GEAR_OFFSET);
    // // need to apply offsets here
    // }

    // private Rotation2d getEncoderPos17t() {
    // return
    // Rotation2d.fromRotations((encoder2.getAbsolutePosition().getValueAsDouble())).minus(Settings.Turret.SEVENTEETH_TEETH_GEAR_OFFSET);
    // // need to apply offsets here
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

    private double wrappingMath(double target, double current) {
        double delta = (target - current) % 360;

        if(delta > 180.0) delta -= 360;
        else if(delta < -180) delta += 360;

        if(Math.abs(current + delta) < Constants.Turret.RANGE) return delta;

        return delta < 0 ? delta + 360 : delta - 360;
    }

    double targetTemp = 0.0;

    @Override
    public void periodic() {
        super.periodic();

        if (!Settings.EnabledSubsystems.TURRET.get() || getTurretState() == TurretState.STOP) {
            turretMotor.setVoltage(0);
        } else {
            targetTemp -= .01;
            if (targetTemp > 1.0 + 1.0/6.0) {
                turretMotor.setControl(new PositionVoltage(targetTemp - 1.0));
                targetTemp -= 1;
            }

            if (targetTemp < -1.0 - 1.0/6.0) {
                turretMotor.setControl(new PositionVoltage(targetTemp + 1.0));
                targetTemp += 1;              
            }

            turretMotor.setControl(new PositionVoltage(targetTemp));
        }

        double actualTarget = getAngle().getRotations() +
                            getAngle().getRotations() % 1 - targetTemp; //account for disabling continuous wrapping

        // SmartDashboard.putNumber("Turret/Pos 18t", getEncoderPos18t().getDegrees());
        // SmartDashboard.putNumber("Turret/Absolute Angle",
        // getAbsoluteTurretAngle().getDegrees());
        SmartDashboard.putNumber("Turret/Relative Encoder", getAngle().getDegrees());
        SmartDashboard.putBoolean("Turret/Exceeded Rotation", exceededOneRotation);
        SmartDashboard.putNumber("Turret/Position", turretMotor.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber("Turret/Voltage", turretMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Turret/Get Hub Target Angke", getPointAtHubAngle().getDegrees());
    }
}
