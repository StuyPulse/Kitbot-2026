package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.commands.swerve.pidToPose.SwerveDrivePIDToPose;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Vector2D;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDriveToClimb extends Command{
    private final CommandSwerveDrivetrain swerve;
    private Pose2d targetPose;
    
    public SwerveDriveToClimb(Gamepad driver) {
        swerve = CommandSwerveDrivetrain.getInstance();
        
        targetPose = getTargetPose();
        addRequirements(swerve);
    }

    private Pose2d getTargetPose(){
        Pose2d closestRung = Field.getClosesTowerSide(swerve.getPose());
        Translation2d offsetTranslation = new Translation2d(Units.inchesToMeters(-(Field.getTowerPose().getY() - closestRung.getY())), closestRung.getRotation());
        return new Pose2d(closestRung.getTranslation().plus(offsetTranslation), closestRung.getRotation());
    }

    @Override
    public boolean isFinished() {
        return new SwerveDrivePIDToPose(targetPose).isFinished(); 
    }

    @Override
    public void execute() {
        new SwerveDrivePIDToPose(targetPose); 
    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Vector2D(new Translation2d()), 0);
    }

}
