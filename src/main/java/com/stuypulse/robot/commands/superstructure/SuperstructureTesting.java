package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureTesting extends SuperstructureSetState{
    public SuperstructureTesting() {
        super(SuperstructureState.TESTING);
    }
}
