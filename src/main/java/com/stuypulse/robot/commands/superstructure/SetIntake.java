package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SetIntake extends SetState{
    public SetIntake() {
        super(SuperstructureState.INTAKING);
    }
}