package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperstructureSetState extends InstantCommand {
     
    private final Superstructure superstructure;
    private final SuperstructureState state;

    public SuperstructureSetState(SuperstructureState state) {
        this.superstructure = Superstructure.getInstance();
        this.state = state;

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(state);
    }
}
