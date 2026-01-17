package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SetOuttake extends SetState{
    public SetOuttake() {
        super(SuperstructureState.OUTTAKING);
    }
}