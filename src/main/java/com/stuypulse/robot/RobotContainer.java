/************************ PROJECT KITBOT ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.superstructure.SuperstructureIntake;
import com.stuypulse.robot.commands.superstructure.SuperstructureOuttake;
// import com.stuypulse.robot.commands.superstructure.SuperstructureOuttake;
import com.stuypulse.robot.commands.superstructure.SuperstructureSetState;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.superstructure.SuperstructureStop;
import com.stuypulse.robot.commands.superstructure.WaitUntilAtTargetVelocity;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignToHub;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveResetRotation;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystems
    public final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    public final Superstructure superstructure = Superstructure.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        swerve.configureAutoBuilder();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
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
            .whileTrue(new SuperstructureShoot())
            .whileFalse(new SuperstructureIntake());

        driver.getLeftButton()
            .whileTrue(new SwerveDriveAlignToHub());

        // driver.getTopButton()
        //     .onTrue(new SuperstructureSetState(SuperstructureState.PREPARING)
        //         .andThen(new WaitUntilAtTargetVelocity())
        //         .andThen(new WaitCommand(1))
        //         .andThen(new SuperstructureShoot()))
        //     .onFalse(new SuperstructureStop());

        driver.getBottomButton()
            .whileTrue(new SuperstructureOuttake())
            .whileFalse(new SuperstructureIntake());

        driver.getDPadUp()
            .onTrue(new SwerveResetRotation());

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Swerve Quasistatic Forward", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("Swerve Quasistatic Backward", swerve.sysIdQuasistatic(Direction.kReverse));

        autonChooser.addOption("Swerve Dynamic Forward", swerve.sysIdQuasistatic(Direction.kForward));
        autonChooser.addOption("Swerve Dynamic Backward", swerve.sysIdQuasistatic(Direction.kReverse));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
