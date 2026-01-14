package com.stuypulse.robot.subsystems.superstructure;
 
import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class SuperstructureImpl extends Superstructure {
    private final TalonFX IntakeShooterMotor;
    private final SparkMax IndexerMotor;
    private boolean shooterAtTargetVelocity;

    public SuperstructureImpl() {
        super();
        IntakeShooterMotor = new TalonFX(Ports.Superstructure.INTAKE_SHOOTER_MOTOR);
        TalonFXConfig intakeShooterMotorConfig = new TalonFXConfig()
            .withCurrentLimitAmps(40)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(Settings.Superstructure.intakeShooterInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        intakeShooterMotorConfig.configure(IntakeShooterMotor);

        IndexerMotor = new SparkMax(Ports.Superstructure.INDEXER_MOTOR, MotorType.kBrushed);
        SparkBaseConfig indexerMotorConfig = new SparkMaxConfig().inverted(Settings.Superstructure.indexerInverted).idleMode(IdleMode.kBrake);
        IndexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void setMotorsBasedOnState() {
        IntakeShooterMotor.set(state.getIntakeShooterSpeed());
        IndexerMotor.set(state.getIndexerSpeed());
    }

    @Override
    public void periodic() {
        setMotorsBasedOnState();

        // TODO: Is this supposed to be here?
        // idk hopefully
        shooterAtTargetVelocity = (Math.abs(IntakeShooterMotor.getVelocity().getValueAsDouble() - state.getIntakeShooterSpeed()) < Settings.Superstructure.Intake_Shooter_Speeds.SHOOT_TOLERANCE_RPM);
    }
}
