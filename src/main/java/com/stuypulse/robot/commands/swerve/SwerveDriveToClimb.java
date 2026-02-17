package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment.Targets;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.math.Vector2D;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDriveToClimb extends Command{
    private final CommandSwerveDrivetrain swerve;
    private Pose2d targetPose;
    
    public SwerveDriveToClimb() {
        swerve = CommandSwerveDrivetrain.getInstance();
        
        targetPose = getTargetPose();
        addRequirements(swerve);
    }

    private Pose2d getTargetPose(){
        Pose2d closestRung = Field.getClosestTowerSide(swerve.getPose());
        Translation2d offsetTranslation;
        if (swerve.getPose().getY() >= Field.towerCenter.getY()){
            offsetTranslation = new Translation2d(0, Targets.DISTANCE_TO_RUNGS);
        } else {
            offsetTranslation = new Translation2d(0, -Targets.DISTANCE_TO_RUNGS);
        }
        return new Pose2d(closestRung.getTranslation().plus(offsetTranslation), closestRung.getRotation());
    }

    // @Override
    // public boolean isFinished() {
    //     return new SwerveDrivePIDToPose(targetPose).isFinished(); 
    // }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(new SwerveDrivePIDToPose(targetPose));
        SmartDashboard.putNumber("Target Pose X", targetPose.getX());
        SmartDashboard.putNumber("Target Pose Y", targetPose.getY());
        SmartDashboard.putNumber("Target Pose Rotation", targetPose.getRotation().getDegrees());


    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Vector2D(new Translation2d()), 0);
    }

}
