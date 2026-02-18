package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;


public class SwerveDriveClimbAlign extends ConditionalCommand{
    public SwerveDriveClimbAlign(){
        super(new SwerveDriveClimbAlignTop(), new SwerveDriveClimbAlignBot(), () -> Field.closerToTop());
    }
}
