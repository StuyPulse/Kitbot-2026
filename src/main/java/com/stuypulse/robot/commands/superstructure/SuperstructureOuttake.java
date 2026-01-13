package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperstructureOuttake extends InstantCommand {
     
    private Superstructure superstructure;

    public SuperstructureOuttake() {
        this.superstructure = Superstructure.getInstance();
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(SuperstructureState.OUTTAKING);
    }
}
