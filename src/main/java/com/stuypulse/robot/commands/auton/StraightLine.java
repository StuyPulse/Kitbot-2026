package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLine extends SequentialCommandGroup {
    
    public StraightLine(PathPlannerPath... paths) {

        addCommands(

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])

        );

    }

}
