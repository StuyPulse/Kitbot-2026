package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Gains.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.InterpolationUtil;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureImpl extends Superstructure {
    
    private final TalonFX intakeShooterMotor;
    private final SparkMax indexMotor;
    private final CommandSwerveDrivetrain swerve;
    private SmartNumber shooterRpm;
    
    public SuperstructureImpl() {

        shooterRpm = new SmartNumber("SuperStructure/Settings/Testing RPM", 0.0);
        swerve = CommandSwerveDrivetrain.getInstance();
        intakeShooterMotor = new TalonFX(Ports.Superstructure.INTAKE_SHOOTER_MOTOR, "Swerve Drive Drive");
        Motors.Superstructure.intakeShooterMotorConfig.configure(intakeShooterMotor);  
        
        indexMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        indexMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public boolean atTargetVelocity() {
        double shooterVel = intakeShooterMotor.getVelocity().getValueAsDouble();
        double targetVel = getState().getIndexerTargetSpeed();

        return Math.abs(shooterVel - targetVel) < Settings.Superstructure.Intake_Shooter.SHOOT_TOLERANCE_RPM;
    }

    @Override
    public double getIntakeShooterMotorRPM() {
        return intakeShooterMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    private void setMotorsBasedOnState() {
        switch (getState()) {
            case SHOOTING:
                intakeShooterMotor.setControl(new VelocityVoltage(InterpolationUtil.getRpmfromdistance(swerve.getDistanceFromHub()) / 60.0));
                break;
            case PREPARING:
                intakeShooterMotor.setControl(new VelocityVoltage(getState().getMainWheelsTargetSpeed() / 60.0));
                break;
            case TESTING:
                intakeShooterMotor.setControl(new VelocityVoltage(shooterRpm.getAsDouble() / 60.0));
                break;
            case INTERPOLATION:
                intakeShooterMotor.setControl(new VelocityVoltage(InterpolationUtil.getRpmfromdistance(swerve.getDistanceFromHub())));
            default:
                intakeShooterMotor.setControl(new DutyCycleOut(getState().getMainWheelsTargetSpeed()));
                break;
        }
        
        if (atTargetVelocity()) {
            indexMotor.set(getState().getIndexerTargetSpeed());
        }
    }

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.SUPERSTRUCTURE.get()) {
            setMotorsBasedOnState();
        } else {
            intakeShooterMotor.setVoltage(0);
            indexMotor.setVoltage(0);
        }

        SmartDashboard.putString("SuperStructure/State", getState().toString());
        SmartDashboard.putNumber("SuperStructure/Main Wheels Target Speed", getState().getMainWheelsTargetSpeed());
        SmartDashboard.putNumber("SuperStructure/Indexer Target Speed", getState().getIndexerTargetSpeed());
        SmartDashboard.putNumber("SuperStructure/DistanceFromGoal", swerve.getDistanceFromHub());
        SmartDashboard.putNumber("SuperStructure/Main Wheels Current Speed (RPM)", intakeShooterMotor.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber("SuperStructure/Indexer Applied DutyCycle", indexMotor.getAppliedOutput());
        SmartDashboard.putNumber("SuperStructure/Interpolated RPM", InterpolationUtil.getRpmfromdistance(swerve.getDistanceFromHub()));
    }
}
