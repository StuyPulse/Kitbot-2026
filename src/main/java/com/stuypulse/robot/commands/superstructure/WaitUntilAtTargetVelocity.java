package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WaitUntilAtTargetVelocity extends WaitUntilCommand {
    public WaitUntilAtTargetVelocity() {
        super(() -> Superstructure.getInstance().atTargetVelocity());
    }
}
