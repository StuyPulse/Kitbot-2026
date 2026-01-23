package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureOuttake extends SuperstructureSetState {
    public SuperstructureOuttake() {
        super(SuperstructureState.OUTTAKING);
    }
}
