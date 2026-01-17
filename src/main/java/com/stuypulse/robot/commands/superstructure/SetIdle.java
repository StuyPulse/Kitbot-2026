package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SetIdle extends SetState{
    public SetIdle() {
        super(SuperstructureState.IDLE);
    }
}