/************************ PROJECT KITBOT ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import javax.print.DocFlavor.STRING;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.WheelTestAuton;
import com.stuypulse.robot.commands.climberhopper.ClimberDown;
import com.stuypulse.robot.commands.climberhopper.ClimberHopperDefaultCommand;
import com.stuypulse.robot.commands.climberhopper.ClimberUp;
import com.stuypulse.robot.commands.superstructure.SuperstructureIntake;
import com.stuypulse.robot.commands.superstructure.SuperstructureOuttake;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveResetRotation;
import com.stuypulse.robot.commands.swerve.SwerveWheelCharacterization;
import com.stuypulse.robot.commands.turret.TurretFerry;
import com.stuypulse.robot.commands.turret.TurretIdle;
import com.stuypulse.robot.commands.turret.TurretShoot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
// import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystems
    public final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    // public final LimelightVision limelight = LimelightVision.getInstance();
    public final Superstructure superstructure = Superstructure.getInstance();
    public final Turret turret = Turret.getInstance();
    private final ClimberHopper climberHopper = ClimberHopper.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        SmartDashboard.putData("Field", Field.FIELD2D);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        climberHopper.setDefaultCommand(new ClimberHopperDefaultCommand());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        // TODO: Put HopperClimber button bindings
        driver.getLeftBumper()
            .whileTrue(new ClimberUp());

        driver.getRightBumper()
            .whileTrue(new ClimberDown());

        // driver.getTopButton()
        //     .whileTrue(new SuperstructureShoot())
        //     .whileFalse(new SuperstructureIntake());

        // driver.getTopButton()
        //     .whileTrue(new TurretShoot())
        //     .whileFalse(new TurretIdle());

        // driver.getLeftButton()
        // .whileTrue(new SwerveDriveAlignToHub());

        // driver.getTopButton()
        // .onTrue(new SuperstructureSetState(SuperstructureState.PREPARING)
        // .andThen(new WaitUntilAtTargetVelocity())
        // .andThen(new WaitCommand(1))
        // .andThen(new SuperstructureShoot()))
        // .onFalse(new SuperstructureStop());

        driver.getBottomButton()
                .whileTrue(new SuperstructureOuttake())
                .whileFalse(new SuperstructureIntake());

        driver.getDPadUp()
                .onTrue(new SwerveResetRotation());

        driver.getLeftButton().whileTrue(new TurretShoot())
                .onFalse(new TurretIdle());

        driver.getRightButton().whileTrue(new TurretFerry())
                .onFalse(new TurretIdle());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Swerve Quasistatic Forward", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("Swerve Quasistatic Backward", swerve.sysIdQuasistatic(Direction.kReverse));

        autonChooser.addOption("Swerve Dynamic Forward", swerve.sysIdDynamic(Direction.kForward));
        autonChooser.addOption("Swerve Dynamic Backward", swerve.sysIdDynamic(Direction.kReverse));
        
        autonChooser.addOption("Wheel Characterization", new SwerveWheelCharacterization());

        AutonConfig STRAIGHT_TEST = new AutonConfig("Straight Line Odometry Test", WheelTestAuton::new,
            "Example Path");
        STRAIGHT_TEST.register(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
