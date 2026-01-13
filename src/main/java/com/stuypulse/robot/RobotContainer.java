/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.superstructure.SuperstructureIntake;
import com.stuypulse.robot.commands.superstructure.SuperstructureOuttake;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.superstructure.SuperstructureStop;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
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
        //swerve.setDefaultCommand(new SwerveDriveDrive());     PLEASE FIX HEHE
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getLeftTriggerButton()
            .onTrue(new SuperstructureIntake())
            .onFalse(new SuperstructureStop());

        driver.getRightTriggerButton()
            .onTrue(new SuperstructureShoot())
            .onFalse(new SuperstructureStop());

        driver.getRightBumper()
            .onTrue(new SuperstructureOuttake())
            .onFalse(new SuperstructureStop());

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
