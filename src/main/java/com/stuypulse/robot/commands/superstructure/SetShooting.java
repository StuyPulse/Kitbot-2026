package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SetShooting extends SetState{
    public SetShooting() {
        super(SuperstructureState.SHOOTING);
    }
}