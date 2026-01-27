package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureImpl extends Superstructure {

    private final TalonFX intakeShooterMotor;
    private final SparkMax indexMotor;

    private final SmartNumber shooterTargetRPM;
    private final SmartNumber indexerTargetRPM;

    public SuperstructureImpl() {
        
        intakeShooterMotor = new TalonFX(Ports.Superstructure.INTAKE_SHOOTER_MOTOR, "Swerve Drive Drive");
        Motors.Superstructure.intakeShooterMotorConfig.configure(intakeShooterMotor);

        indexMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        indexMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTargetRPM = new SmartNumber("Superstructure/Shooter Target RPM", getHubShotSpeeds().getRPM());
        indexerTargetRPM = new SmartNumber("Superstructure/Indexer Target RPM", getIndexerState().getIndexerTargetSpeed());

    }

    private double getShooterRPM() {
        return intakeShooterMotor.getVelocity().getValueAsDouble();
    }

    private void setShooterRPM(double rpm) {
        intakeShooterMotor.setControl(new VelocityVoltage(rpm));
    }

    private void setShooterVoltage(double volts) {
        intakeShooterMotor.setVoltage(volts);
    }

    private double getIndexerRPM() {
        return indexMotor.getBusVoltage() * indexMotor.getAppliedOutput();
    }

    private void setIndexerRPM(double rpm) {
        indexMotor.set(rpm);
    }

    private void setIndexerVoltage(double volts) {
        indexMotor.setVoltage(volts);
    }

    @Override
    public boolean atTargetVelocity() {
        double current = getShooterRPM();
        double target = shooterTargetRPM.get();
        return Math.abs(current - target) < Settings.Superstructure.Intake_Shooter.SHOOT_TOLERANCE_RPM;
    }

    private void setShooterTargetSpeeds(double rpm) {
        shooterTargetRPM.set(rpm);
    }

    private void setIndexerTargetSpeeds(double rpm) {
        indexerTargetRPM.set(rpm);
    }

    private ShooterSpeeds getHubShotSpeeds() {
        Pose2d hubPose = Field.getAllianceHubPose();
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        double distance = robotPose.minus(hubPose).getTranslation().getNorm() - Field.LENGTH / 2;

        if (distance <= 1.5) {
            return new ShooterSpeeds(4000);
        } else {
            return new ShooterSpeeds(5500);
        }
    }

    private void setShooterBasedOnState() {
        switch (getMainWheelState()) {
            case OUTTAKING:
                setShooterTargetSpeeds(Settings.Superstructure.Intake_Shooter.OUTTAKE_SPEED);
                break;
            case INTAKING:
                setShooterTargetSpeeds(Settings.Superstructure.Intake_Shooter.INTAKE_SPEED);
                break;
            case PREPARING:
                setShooterTargetSpeeds(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED_RPM);
                break;
            case SHOOTING:
                setShooterTargetSpeeds(getHubShotSpeeds().getRPM());
                break;
            default: // STOP
                setShooterTargetSpeeds(0.0);
                break;
        }

        if (shooterTargetRPM.get() == 0) {
            setShooterVoltage(0);
        } else {
            setShooterRPM(shooterTargetRPM.get());
        }
    }

    private void setIndexerBasedOnState() {
        switch (getIndexerState()) {
            case OUTTAKING:
                setIndexerTargetSpeeds(Settings.Superstructure.Intake_Shooter.OUTTAKE_SPEED);
                break;
            case INTAKING:
                setIndexerTargetSpeeds(Settings.Superstructure.Intake_Shooter.INTAKE_SPEED);
                break;
            case PREPARING:
                setIndexerTargetSpeeds(Settings.Superstructure.Intake_Shooter.SHOOT_SPEED_RPM);
                break;
            case SHOOTING:
                setIndexerTargetSpeeds(getIndexerState().getIndexerTargetSpeed());
                break;
            default: // STOP
                setIndexerTargetSpeeds(0.0);
                break;
        }

        if (indexerTargetRPM.get() == 0) {
            setIndexerVoltage(0);
        } else {
            setIndexerRPM(indexerTargetRPM.get());
        }
    }

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.SUPERSTRUCTURE.get()) {
            setShooterBasedOnState();
            setIndexerBasedOnState();
        } else {
            setShooterVoltage(0);
            indexMotor.setVoltage(0);
        }

        SmartDashboard.putString("Superstructure/Main Wheel State", getMainWheelState().toString());
        SmartDashboard.putString("Superstructure/Indexer State", getIndexerState().toString());

        SmartDashboard.putNumber("Superstructure/Shooter Target RPM", shooterTargetRPM.get());
        SmartDashboard.putNumber("Superstructure/Shooter Current RPM", getShooterRPM());

        SmartDashboard.putNumber("Superstructure/Indexer Output", indexMotor.getAppliedOutput());
    }
}