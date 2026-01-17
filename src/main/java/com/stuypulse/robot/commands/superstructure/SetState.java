package com.stuypulse.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;

public class SetState extends Command {
    private final SuperstructureState state;

    private final Superstructure superstructure;
    
    public SetState(SuperstructureState state) {
        this.state = state;
        this.superstructure = Superstructure.getInstance();
    
        addRequirements(superstructure);

    }

    @Override
    public void initialize() {
        superstructure.setState(this.state);
    }

}