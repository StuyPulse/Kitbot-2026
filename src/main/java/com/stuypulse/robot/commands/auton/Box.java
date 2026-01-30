package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Box extends SequentialCommandGroup {
    
    public Box(PathPlannerPath... paths) {

        addCommands(

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])

        );

    }

}
