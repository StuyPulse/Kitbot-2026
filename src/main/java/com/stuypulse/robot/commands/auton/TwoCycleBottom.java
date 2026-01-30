package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.superstructure.SuperstructureShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignToHub;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoCycleBottom extends SequentialCommandGroup {
    
    public TwoCycleBottom(PathPlannerPath... paths) {

        addCommands(

            // Shoot Preloads
            new SwerveDriveAlignToHub(),
            new SuperstructureShoot(),

            // Collect Fuel (1)
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            new WaitCommand(1),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),

            // Shoot Fuel (1)
            new SwerveDriveAlignToHub(),
            new SuperstructureShoot(),

            // Collect Fuel (2)
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
            new WaitCommand(1),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),

            // Shoot Fuel (2)
            new SwerveDriveAlignToHub(),
            new SuperstructureShoot()

        );

    }

}