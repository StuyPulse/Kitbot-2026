package com.stuypulse.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.robot.util.ShooterInterpolation;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureImpl extends Superstructure {
    
    private final TalonFX IntakeShootMotor;
    private final SparkMax IndexerMotor;
    private final SmartNumber targetRPM;
    
    public SuperstructureImpl() {
        super();
        
        IntakeShootMotor = new TalonFX(Ports.Superstructure.INTAKE_SHOOTER_MOTOR, "Swerve Drive Drive");
        Motors.Superstructure.intakeShooterMotorConfig.configure(IntakeShootMotor);  
        
        IndexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        IndexerMotor.configure(Motors.Superstructure.indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        targetRPM = new SmartNumber("Shooter/Left Target RPM", getHubShotSpeeds().getRPM());

    }


    private void setMotorsBasedOnState() {
        IntakeShootMotor.setControl(new DutyCycleOut(state.getMainWheelsSpeed()));
        IndexerMotor.set(state.getIndexerSpeed());
    }

    private void setTargetSpeeds(ShooterSpeeds speeds) {
        this.targetRPM.set(speeds.getRPM());

    }

    private ShooterSpeeds getHubShotSpeeds() {
        Pose2d hubPose = Field.getAllianceHubPose();
        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        double distanceToHub = robotPose.minus(hubPose).getTranslation().getNorm() - Field.LENGTH / 2;
        
        if (distanceToHub <= 1.5) {
            return new ShooterSpeeds(0);
        }
        else { // art of the steal
            return new ShooterSpeeds(0);
        }
        // return new ShooterSpeeds(ShooterInterpolation.getRPM(distanceToHub), 500);
    }

    private void setMainWheelsBasedOnState() {
        switch (getState()) {
            case FERRY: // placeholder
                setTargetSpeeds(getHubShotSpeeds());
                break;
            case SHOOTING:
                setTargetSpeeds(getHubShotSpeeds());
                break;
            default:
                setTargetSpeeds(new ShooterSpeeds());
                break;
        }
    }

    @Override
    public void periodic() {
        setMotorsBasedOnState();
        shooterAtTargetVelocity = (Math.abs(IntakeShootMotor.getVelocity().getValueAsDouble() - state.getMainWheelsSpeed()) <= Settings.Superstructure.Intake_Shooter.SHOOT_TOLERANCE_RPM);
        SmartDashboard.putString("SuperStructure/State", getState().toString());
        SmartDashboard.putNumber("SuperStructure/Main Wheel/Speed", getState().getMainWheelsSpeed());
        SmartDashboard.putNumber("SuperStructure/Indexer/Speed", getState().getIndexerSpeed());
    }
}
