/************************ PROJECT KITBOT ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.stuypulse.robot.commands.auton.Box;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.StraightLine;
import com.stuypulse.robot.commands.auton.TwoCycleBottom;
import com.stuypulse.robot.commands.superstructure.SuperstructureIntake;
import com.stuypulse.robot.commands.superstructure.SuperstructureOuttake;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.superstructure.SuperstructureTesting;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignToHub;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveMovmentAlignToHub;
import com.stuypulse.robot.commands.swerve.SwerveResetRotation;
import com.stuypulse.robot.commands.turret.SetTurretFerry;
import com.stuypulse.robot.commands.turret.SetTurretPointAtHub;
import com.stuypulse.robot.commands.turret.SetTurretZero;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Subsystems
    public final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    public final Superstructure superstructure = Superstructure.getInstance();
    public final LimelightVision vision = LimelightVision.getInstance();
    // public final Turret turret = Turret.getInstance();

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
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getTopButton()
                .whileTrue(new SwerveDriveAlignToHub()
                    .alongWith(new SuperstructureShoot()))
                .whileFalse(new SuperstructureIntake());

        // driver.getLeftButton()
        //     .whileTrue(new SwerveDriveAlignToHub());

        // driver.getTopButton()
        // .onTrue(new SuperstructureSetState(SuperstructureState.PREPARING)
        // .andThen(new WaitUntilAtTargetVelocity())
        // .andThen(new WaitCommand(1))
        // .andThen(new SuperstructureShoot()))
        // .onFalse(new SuperstructureStop());

        driver.getBottomButton()
                .whileTrue(new SuperstructureOuttake());
                // .whileFalse(new SuperstructureIntake());

        driver.getDPadUp()
                .onTrue(new SwerveResetRotation());

        // driver.getLeftButton().whileTrue(new SetTurretPointAtHub())
        //         .onFalse(new SetTurretZero());

        // driver.getRightButton().whileTrue(new SetTurretFerry())
        //         .onFalse(new SetTurretZero());
        driver.getDPadDown()
            .onTrue(new SuperstructureTesting());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {

        AutonConfig TWO_CYCLE_BOTTOM = new AutonConfig("2 Cycle Bottom", TwoCycleBottom::new,
        "Bump Start To NZ", "NZ To Shoot", "To Shoot For Real", "Bump Start To NZ", "NZ To Shoot", "To Shoot For Real");
        TWO_CYCLE_BOTTOM.register(autonChooser);

        AutonConfig STRAIGHT_LINE = new AutonConfig("Straight Line", StraightLine::new, 
            "Straight Line");
        STRAIGHT_LINE.register(autonChooser);

        AutonConfig BOX = new AutonConfig("Box", Box::new, 
            "Box 1", "Box 2", "Box 3", "Box 4");
        BOX.register(autonChooser);

        // try {
        //     autonChooser.addOption("Depot HP Climb Mid", new PathPlannerAuto("Depot HP Climb Mid"));
        //     autonChooser.addOption("Depot HP Climb Right", new PathPlannerAuto("Depot HP Climb Right"));
        //     autonChooser.addOption("HP Depot Climb Left", new PathPlannerAuto("HP Depot Climb Left"));
        //     autonChooser.addOption("Depot HP Climb Mid", new PathPlannerAuto("HP Depot Climb Mid"));

        // } catch (AutoBuilderException e) {
        //     DriverStation.reportError("AutoBuilderException: " + e.getMessage(), e.getStackTrace());
        // }

        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Swerve Translation Quasistatic Forward", swerve.sysIdTranslationQuasistatic(Direction.kForward));
        autonChooser.addOption("Swerve Translation Quasistatic Backward", swerve.sysIdTranslationQuasistatic(Direction.kReverse));
        autonChooser.addOption("Swerve Translation Dynamic Forward", swerve.sysIdTranslationDynamic(Direction.kForward));
        autonChooser.addOption("Swerve Translation Dynamic Backward", swerve.sysIdTranslationDynamic(Direction.kReverse));

        autonChooser.addOption("Swerve Rotation Dynamic Forward", swerve.sysidDynamicRotation(Direction.kForward));
        autonChooser.addOption("Swerve Rotation Dynamic Forward", swerve.sysidDynamicRotation(Direction.kReverse));
        autonChooser.addOption("Swerve Rotation Quasistatic Forward", swerve.sysidQuasistaticRotation(Direction.kForward));
        autonChooser.addOption("Swerve Rotation Quasistatic Forward", swerve.sysidQuasistaticRotation(Direction.kReverse));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
