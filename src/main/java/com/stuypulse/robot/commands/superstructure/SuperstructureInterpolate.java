package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class SuperstructureInterpolate extends SuperstructureSetState {
    public SuperstructureInterpolate() {
        super(SuperstructureState.INTERPOLATION);
    }
}
